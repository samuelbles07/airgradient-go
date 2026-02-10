#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include <math.h>
#include <limits.h>
#include "esp_log_level.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "soc/gpio_num.h"
#include "sps30.h"
#include "gdey0213b74.h"
#include "nand_storage_service.h"

#include "ui/dashboard_ui.h"
#include "gps_service.h"

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

// Set to 1 to deep sleep the panel each iteration.
// This forces a full refresh (basemap+values) on every update.
#ifndef GO_DISPLAY_EPD_SLEEP_EACH_ITERATION
#define GO_DISPLAY_EPD_SLEEP_EACH_ITERATION 1
#endif

#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000 // 100 kHz

#define GPIO_EN_PM1 GPIO_NUM_26 // GPIO 26 - PM sensor load switch + I2C isolator enable
#define GPIO_QON GPIO_NUM_5
#define GPIO_WDT GPIO_NUM_2
#define UART_GPS_TX GPIO_NUM_11
#define UART_GPS_RX GPIO_NUM_12
#define UART_GPS_PORT UART_NUM_1
#define UART_GPS_BAUD 9600

static void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static const char *TAG = "GO";

static bool init_sps30_sensor(i2c_master_bus_handle_t bus_handle);
static void init_qon_button();
static void resetExtWatchdog();
static void log_gps_data(const GPSService::Data &d);
static void dump_all_storage_records(NandStorageService *storage);
static NandStorageService storage;
static uint32_t next_record_id = 0;
static bool storage_logging_enabled = true;
static int32_t deg_to_e7(double deg) { return (int32_t)llround(deg * 10000000.0); }
static uint16_t pm25_to_x10(float ugm3) {
  if (!(ugm3 >= 0.0f)) { // catches NaN too
    return 0xFFFF;
  }
  const int v = (int)lroundf(ugm3 * 10.0f);
  if (v < 0)
    return 0;
  if (v > 65534)
    return 65534;
  return (uint16_t)v;
}

sps30_handle_t sps30_handle;

static QueueHandle_t gpio_evt_queue = NULL;
static void button_task(void *arg);
static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

extern "C" void app_main(void) {
  esp_log_level_set("GO", ESP_LOG_INFO);

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = 25;
  buscfg.miso_io_num = 24;
  buscfg.sclk_io_num = 23;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 4096;
  esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
    return;
  }

  // Configure I2C master bus
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_MASTER_PORT,
      .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
      .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      // .flags.enable_internal_pullup = true,
  };
  bus_cfg.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

  init_sps30_sensor(bus_handle);
  init_qon_button();

  // Init GPS
  GPSService::Config gpsConfig;
  gpsConfig.uart_num = UART_GPS_PORT;
  gpsConfig.rx_pin = UART_GPS_RX;
  gpsConfig.tx_pin = UART_GPS_TX;
  gpsConfig.baud_rate = UART_GPS_BAUD;
  gpsConfig.log_raw_nmea = true;
  GPSService gps;
  ESP_ERROR_CHECK(gps.init(gpsConfig));
  ESP_ERROR_CHECK(gps.start());

  NandStorageService::Config scfg;
  scfg.spi_host = SPI2_HOST;
  scfg.cs_pin = GPIO_NUM_4;
  scfg.clock_speed_hz = 10 * 1000 * 1000;
  scfg.mount_path = "/nand";
  scfg.records_path = "/nand/log.bin";
  ESP_ERROR_CHECK(storage.init(scfg));
  ESP_ERROR_CHECK(storage.start());
  // Optional: wait for mount/file open so we can set next_record_id.
  for (int i = 0; i < 100 && !storage.is_ready(); ++i) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  dump_all_storage_records(&storage);
  storage.clear_sync(pdMS_TO_TICKS(2000));
  if (storage.is_ready()) {
    uint32_t count = 0;
    if (storage.get_count_sync(&count, pdMS_TO_TICKS(1000)) == ESP_OK) {
      next_record_id = count; // continue IDs from existing file length
    }
  } else {
    ESP_LOGW(TAG, "storage not ready yet; will start logging when ready");
  }

  // Init E-Paper Display
  ssd1680x::Config cfg;
  cfg.host = SPI2_HOST;
  cfg.pins.busy = GPIO_NUM_10;
  cfg.pins.rst = GPIO_NUM_9;
  cfg.pins.dc = GPIO_NUM_15;
  cfg.pins.cs = GPIO_NUM_0;
  cfg.devcfg.clock_speed_hz = 4 * 1000 * 1000;
  cfg.devcfg.mode = 0;
  cfg.devcfg.queue_size = 1;
  cfg.devcfg.flags = SPI_DEVICE_HALFDUPLEX;

  // Keep large objects off the main task stack.
  static ssd1680x::panels::GDEY0213B74 epd(cfg);
  static ui::DashboardUI ui(epd);

  err = ui.init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "UI init failed: %s", esp_err_to_name(err));
    return;
  }

  float pm25 = 0.7f;
  uint32_t displayRefreshStart = MILLIS();
  uint32_t lastPmRead = MILLIS();
  uint32_t lastWdtReset = MILLIS();
  sps30_measurement_t sps30_result;
  while (1) {

    if ((MILLIS() - lastWdtReset) > 60000) {
      lastWdtReset = MILLIS();
      resetExtWatchdog();
    }

    // Interval pm
    if ((MILLIS() - lastPmRead) >= 1000) {
      lastPmRead = MILLIS();
      sps30_read_measurement(sps30_handle, &sps30_result);
      pm25 = sps30_result.pm2p5_mass;
      ESP_LOGI(TAG, "pm25: %.1f", pm25);
      auto gpsData = gps.get();
      log_gps_data(gpsData);

      NandStorageService::Record r;
      r.id = next_record_id++;
      r.timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
      r.pm25_ugm3_x10 = pm25_to_x10(pm25);
      if (gpsData.fix_valid) {
        r.latitude_e7 = deg_to_e7(gpsData.latitude_deg);
        r.longitude_e7 = deg_to_e7(gpsData.longitude_deg);
      } else {
        r.latitude_e7 = INT32_MIN;  // sentinel for “invalid”
        r.longitude_e7 = INT32_MIN; // sentinel for “invalid”
      }
      (void)storage.enqueue_record(r, false, 0);
    }

    // Interval refresh
    if ((MILLIS() - displayRefreshStart) >= 10000) {
      displayRefreshStart = MILLIS();

      ui.set_pm25_ugm3(pm25);

      // Update the display.
#if GO_DISPLAY_EPD_SLEEP_EACH_ITERATION
      err = ui.full_refresh();
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI full_refresh failed: %s", esp_err_to_name(err));
      }

      err = epd.deep_sleep();
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "EPD deep_sleep failed: %s", esp_err_to_name(err));
      }
#else
      err = ui.refresh();
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI refresh failed: %s", esp_err_to_name(err));
      }
#endif
    }

    delay_ms(10);
  }
}

bool init_sps30_sensor(i2c_master_bus_handle_t bus_handle) {
  // Configure EN_PM1 GPIO (IO26) for PM sensor power control
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << GPIO_EN_PM1);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Enable PM sensor power (TPS27081A load switch + TMUX121 I2C isolator)
  gpio_set_level((gpio_num_t)GPIO_EN_PM1, 1);
  ESP_LOGI(TAG, "EN_PM1 enabled (IO26=HIGH) - PM sensor powered");

  // Wait for power stabilization
  vTaskDelay(pdMS_TO_TICKS(100));

  sps30_config_t cfg = {.i2c_address = 0x69, .i2c_clock_speed = I2C_MASTER_FREQ_HZ};
  esp_err_t ret = sps30_init(bus_handle, &cfg, &sps30_handle);
  if (ret != ESP_OK) {
    return false;
  }

  ret = sps30_start_measurement(sps30_handle);
  if (ret != ESP_OK)
    return ret;
  vTaskDelay(pdMS_TO_TICKS(3000));
  uint32_t status = 0;
  if (sps30_read_status_register(sps30_handle, &status) == ESP_OK) {
    ESP_LOGI(TAG, "SPS30 status after 3s: 0x%08X", status);
  }
  bool ready = false;
  if (sps30_read_data_ready(sps30_handle, &ready) == ESP_OK) {
    ESP_LOGI(TAG, "SPS30 data-ready: %s", ready ? "YES" : "NO");
  }
  // sps30_stop_measurement(sps30_handle);
  // sps30_sleep(sps30_handle);
  // ESP_LOGI(TAG, "SPS30 ready (sleep mode)");
  ESP_LOGI(TAG, "SPS30 ready");
  return true;
}

void init_qon_button() {
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << GPIO_QON);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&io_conf);

  // Create queue
  gpio_evt_queue = xQueueCreate(3, sizeof(uint32_t));

  // Start task to handle events
  xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
  // Install GPIO ISR service
  gpio_install_isr_service(0);
  // Hook ISR handler
  gpio_isr_handler_add(GPIO_QON, gpio_isr_handler, (void *)GPIO_QON);
}

void button_task(void *arg) {
  uint32_t io_num;
  uint32_t lastButtonPressed = MILLIS();
  while (1) {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      // Handle debouncing
      if ((MILLIS() - lastButtonPressed) <= 300) {
        continue;
      }
      lastButtonPressed = MILLIS();

      int level = gpio_get_level(static_cast<gpio_num_t>(io_num));
      if (level == 0) {
        ESP_LOGI(TAG, "Button PRESSED (GPIO %d)", io_num);
      } else {
        ESP_LOGI(TAG, "Button RELEASED (GPIO %d)", io_num);
      }
    }
  }
}

void resetExtWatchdog() {
  ESP_LOGI(TAG, "Watchdog reset");
  gpio_set_level(GPIO_WDT, 1);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(GPIO_WDT, 0);
}

void log_gps_data(const GPSService::Data &d) {
  char time_buf[32];
  if (d.utc.date_valid && d.utc.time_valid) {
    snprintf(time_buf, sizeof(time_buf), "%04d-%02d-%02d %02d:%02d:%02dZ", d.utc.year, d.utc.month,
             d.utc.day, d.utc.hour, d.utc.min, d.utc.sec);
  } else if (d.utc.time_valid) {
    snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02dZ", d.utc.hour, d.utc.min, d.utc.sec);
  } else {
    snprintf(time_buf, sizeof(time_buf), "--");
  }
  if (d.fix_valid) {
    ESP_LOGI(TAG,
             "fix=1 q=%d sats=%d lat=%.6f lon=%.6f alt=%s%.1fm spd=%s%.1fkn trk=%s%.0fdeg time=%s "
             "ant=%s last_sentence=%" PRIu64 "ms",
             d.fix_quality, d.satellites, d.latitude_deg, d.longitude_deg,
             d.altitude_valid ? "" : "~", d.altitude_m, d.speed_valid ? "" : "~", d.speed_knots,
             d.track_valid ? "" : "~", d.track_deg, time_buf,
             GPSService::antenna_status_to_str(d.antenna_status), d.last_sentence_ms);
  } else {
    ESP_LOGI(TAG,
             "fix=0 q=%d sats=%d lat=-- lon=-- alt=-- spd=-- trk=-- time=%s ant=%s "
             "last_sentence=%" PRIu64 "ms",
             d.fix_quality, d.satellites, time_buf,
             GPSService::antenna_status_to_str(d.antenna_status), d.last_sentence_ms);
  }
}

void dump_all_storage_records(NandStorageService *storage) {
  if (!storage) {
    return;
  }
  if (!storage->is_ready()) {
    ESP_LOGW(TAG, "storage not ready; skip dump");
    return;
  }
  uint32_t total = 0;
  esp_err_t err = storage->get_count_sync(&total, pdMS_TO_TICKS(2000));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "get_count_sync failed: %s", esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "storage records: total=%" PRIu32, total);
  static constexpr uint32_t CHUNK = 64;
  static NandStorageService::Record buf[CHUNK];
  uint32_t idx = 0;
  while (idx < total) {
    uint32_t nread = 0;
    const uint32_t want = (total - idx > CHUNK) ? CHUNK : (total - idx);
    err = storage->read_range_sync(idx, buf, want, &nread, pdMS_TO_TICKS(5000));
    if (err != ESP_OK && nread == 0) {
      ESP_LOGE(TAG, "read_range_sync failed at idx=%" PRIu32 ": %s", idx, esp_err_to_name(err));
      return;
    }
    for (uint32_t i = 0; i < nread; ++i) {
      const auto &r = buf[i];
      const bool gps_valid = (r.latitude_e7 != INT32_MIN && r.longitude_e7 != INT32_MIN);
      const double lat = gps_valid ? ((double)r.latitude_e7 / 10000000.0) : 0.0;
      const double lon = gps_valid ? ((double)r.longitude_e7 / 10000000.0) : 0.0;
      const bool pm_valid = (r.pm25_ugm3_x10 != 0xFFFF);
      const double pm25 = pm_valid ? ((double)r.pm25_ugm3_x10 / 10.0) : 0.0;
      if (gps_valid && pm_valid) {
        ESP_LOGI(TAG, "rec id=%" PRIu32 " ts=%" PRIu64 " pm25=%.1f lat=%.7f lon=%.7f", r.id,
                 r.timestamp_ms, pm25, lat, lon);
      } else if (pm_valid) {
        ESP_LOGI(TAG, "rec id=%" PRIu32 " ts=%" PRIu64 " pm25=%.1f lat=-- lon=--", r.id,
                 r.timestamp_ms, pm25);
      } else {
        ESP_LOGI(TAG, "rec id=%" PRIu32 " ts=%" PRIu64 " pm25=-- lat=%s lon=%s", r.id,
                 r.timestamp_ms, gps_valid ? "OK" : "--", gps_valid ? "OK" : "--");
      }
    }
    idx += nread;
    // If we hit a CRC error, the service returns err!=OK with partial reads possible.
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "dump stopped early at idx=%" PRIu32 " (%s)", idx, esp_err_to_name(err));
      return;
    }
  }
  ESP_LOGI(TAG, "storage dump complete");
}
