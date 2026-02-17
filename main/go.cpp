
#include <stdint.h>

#include "esp_mac.h"
#include <string>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "esp_console.h"
#include "cJSON.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "button_service.h"
#include "gps_service.h"
#include "nand_storage_service.h"
#include "go_constants.h"
#include "WiFiManager.h"
#include "bq25629.h"

#include "gdey0213b74.h"
#include "ui/dashboard_ui.h"

#include "sps30.h"

// NOTE: Temporary constants
#define NO_INACTIVE_NO_SLEEP 1
#define TRACKING_DISPLAY_SLEEP 0

enum class State {
  Idle = 0,
  Inactive,
  Sync,
  Tracking,
  Shutdown,
};

RTC_DATA_ATTR static State RTC_LAST_STATE = State::Idle;
RTC_DATA_ATTR static uint32_t RTC_TRACKING_SESSION_ID = 0;
static WiFiManager g_wifiManager;

static bool is_valid_rtc_state(State s) {
  switch (s) {
  case State::Idle:
  case State::Inactive:
  case State::Sync:
  case State::Tracking:
  case State::Shutdown:
    return true;
  }
  return false;
}

struct Inputs {
  bool button_short = false;
  bool button_long = false;
  bool boot_long = false;
  bool touch_right_long = false;
  bool touch_left_long = false;
  bool touch_enter_long = false;
};

enum class GoInputEventType : uint8_t {
  ButtonShort = 1,
  ButtonLong = 2,
  BootLong = 3,
  TouchRightLong = 4,
  TouchLeftLong = 5,
  TouchEnterLong = 6,
};

struct GoInputEvent {
  GoInputEventType type;
};

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static inline void sleep_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static esp_err_t init_ext_watchdog(void) {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << GO_WDT_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
    return err;
  }
  (void)gpio_set_level(GO_WDT_GPIO, 0);
  return ESP_OK;
}

static void reset_ext_watchdog(void) {
  (void)gpio_set_level(GO_WDT_GPIO, 1);
  sleep_ms(GO_WDT_RESET_PULSE_MS);
  (void)gpio_set_level(GO_WDT_GPIO, 0);
}

static int32_t deg_to_e7(double deg) { return (int32_t)llround(deg * 10000000.0); }

static uint16_t pm25_to_x10(float ugm3) {
  if (!(ugm3 >= 0.0f)) {
    return 0xFFFF;
  }

  const int v = (int)lroundf(ugm3 * 10.0f);
  if (v < 0) {
    return 0;
  }
  if (v > 65534) {
    return 65534;
  }
  return (uint16_t)v;
}

static bool utc_to_epoch_ms(const GPSService::UtcTime &utc, uint64_t *out_ms) {
  if (out_ms == nullptr) {
    return false;
  }
  *out_ms = 0;
  if (!utc.date_valid || !utc.time_valid) {
    return false;
  }

  // Basic range validation.
  if (utc.year < 1970 || utc.month < 1 || utc.month > 12 || utc.day < 1 || utc.day > 31) {
    return false;
  }
  if (utc.hour < 0 || utc.hour > 23 || utc.min < 0 || utc.min > 59 || utc.sec < 0 || utc.sec > 59) {
    return false;
  }

  // days_from_civil() (Howard Hinnant) adapted for UTC.
  int y = utc.year;
  unsigned m = (unsigned)utc.month;
  unsigned d = (unsigned)utc.day;

  if (m <= 2) {
    y -= 1;
  }

  int era = 0;
  if (y >= 0) {
    era = y / 400;
  } else {
    era = (y - 399) / 400;
  }

  const unsigned yoe = (unsigned)(y - era * 400);

  unsigned mp = 0;
  if (m > 2) {
    mp = m - 3;
  } else {
    mp = m + 9;
  }
  const unsigned doy = (153U * mp + 2U) / 5U + d - 1U;

  const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  const int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;

  const int64_t sec =
      days * 86400LL + (int64_t)utc.hour * 3600LL + (int64_t)utc.min * 60LL + (int64_t)utc.sec;
  if (sec < 0) {
    return false;
  }
  *out_ms = (uint64_t)sec * 1000ULL;
  return true;
}

static void format_rfc3339_utc(uint64_t epoch_ms, char *out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return;
  }
  out[0] = '\0';

  const time_t sec = (time_t)(epoch_ms / 1000ULL);
  struct tm tm_utc;
  memset(&tm_utc, 0, sizeof(tm_utc));
  if (gmtime_r(&sec, &tm_utc) == nullptr) {
    (void)snprintf(out, out_len, "1970-01-01T00:00:00Z");
    return;
  }

  (void)snprintf(out, out_len, "%04d-%02d-%02dT%02d:%02d:%02dZ", tm_utc.tm_year + 1900,
                 tm_utc.tm_mon + 1, tm_utc.tm_mday, tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);
}

static const char *state_name(State s) {
  switch (s) {
  case State::Idle:
    return "IDLE";
  case State::Inactive:
    return "INACTIVE";
  case State::Sync:
    return "SYNC";
  case State::Tracking:
    return "TRACKING";
  case State::Shutdown:
    return "SHUTDOWN";
  }
  return "UNKNOWN";
}

static const char *vbus_status_name(drivers::VBusStatus s) {
  switch (s) {
  case drivers::VBusStatus::NO_ADAPTER:
    return "NO_ADAPTER";
  case drivers::VBusStatus::USB_SDP:
    return "USB_SDP";
  case drivers::VBusStatus::USB_CDP:
    return "USB_CDP";
  case drivers::VBusStatus::USB_DCP:
    return "USB_DCP";
  case drivers::VBusStatus::UNKNOWN_ADAPTER:
    return "UNKNOWN_ADAPTER";
  case drivers::VBusStatus::NON_STANDARD:
    return "NON_STANDARD";
  case drivers::VBusStatus::OTG_MODE:
    return "OTG_MODE";
  }
  return "UNDEFINED";
}

static bool is_usb_c_adapter_present(drivers::VBusStatus s) {
  switch (s) {
  case drivers::VBusStatus::USB_SDP:
  case drivers::VBusStatus::USB_CDP:
  case drivers::VBusStatus::USB_DCP:
  case drivers::VBusStatus::UNKNOWN_ADAPTER:
  case drivers::VBusStatus::NON_STANDARD:
    return true;
  case drivers::VBusStatus::NO_ADAPTER:
  case drivers::VBusStatus::OTG_MODE:
    return false;
  }
  return false;
}

static esp_err_t init_sps30_sensor(i2c_master_bus_handle_t bus_handle, sps30_handle_t *out) {
  if (out == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  *out = nullptr;

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << GO_PM_POWER_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
    return err;
  }

  (void)gpio_set_level(GO_PM_POWER_GPIO, GO_PM_POWER_ON_LEVEL);
  sleep_ms(GO_SPS30_POWER_STABILIZE_DELAY_MS);

  sps30_config_t cfg = {};
  cfg.i2c_address = GO_SPS30_I2C_ADDRESS;
  cfg.i2c_clock_speed = GO_SPS30_I2C_CLOCK_SPEED_HZ;

  err = sps30_init(bus_handle, &cfg, out);
  if (err != ESP_OK) {
    return err;
  }

  err = sps30_start_measurement(*out);
  if (err != ESP_OK) {
    return err;
  }

  sleep_ms(GO_SPS30_WARMUP_DELAY_MS);
  ESP_LOGI(GO_TAG, "SPS30 ready");
  return ESP_OK;
}

static esp_err_t init_charger(i2c_master_bus_handle_t bus_handle, drivers::BQ25629 **out) {
  if (out == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  *out = nullptr;

  static drivers::BQ25629 charger(bus_handle);

  drivers::BQ25629_Config cfg = {
      .charge_voltage_mv = 4200,
      .charge_current_ma = 500,
      .input_current_limit_ma = 1500,
      .input_voltage_limit_mv = 4600,
      .min_system_voltage_mv = 3520,
      .precharge_current_ma = 30,
      .term_current_ma = 20,
      .enable_charging = true,
      .enable_otg = false,
      .enable_adc = true,
  };

  esp_err_t err = charger.init(cfg);
  if (err != ESP_OK) {
    return err;
  }

  err = charger.set_watchdog_timeout(drivers::WatchdogTimeout::Sec200);
  if (err != ESP_OK) {
    return err;
  }

  err = charger.enable_pmid_5v_boost();
  if (err != ESP_OK) {
    return err;
  }

  err = charger.reset_watchdog();
  if (err != ESP_OK) {
    return err;
  }

  ESP_LOGI(GO_TAG, "BQ25629 PMID boost enabled");
  *out = &charger;
  return ESP_OK;
}

static void log_gps_data(const GPSService::Data &d) {
  if (!d.has_sentence) {
    ESP_LOGI(GO_TAG, "gps: no sentence");
    return;
  }

  char time_buf[32];
  time_buf[0] = '\0';
  if (d.utc.date_valid && d.utc.time_valid) {
    (void)snprintf(time_buf, sizeof(time_buf), "%04d-%02d-%02d %02d:%02d:%02dZ", d.utc.year,
                   d.utc.month, d.utc.day, d.utc.hour, d.utc.min, d.utc.sec);
  } else if (d.utc.time_valid) {
    (void)snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02dZ", d.utc.hour, d.utc.min, d.utc.sec);
  } else {
    (void)snprintf(time_buf, sizeof(time_buf), "--");
  }

  if (d.fix_valid) {
    ESP_LOGI(
        GO_TAG, "gps: fix=1 q=%d sats=%d lat=%.6f lon=%.6f time=%s last_sentence=%" PRIu64 "ms",
        d.fix_quality, d.satellites, d.latitude_deg, d.longitude_deg, time_buf, d.last_sentence_ms);
  } else {
    ESP_LOGI(GO_TAG, "gps: fix=0 q=%d sats=%d time=%s last_sentence=%" PRIu64 "ms", d.fix_quality,
             d.satellites, time_buf, d.last_sentence_ms);
  }
}

static esp_err_t init_display(ui::DashboardUI **ui_out, ssd1680x::panels::GDEY0213B74 **epd_out) {
  if (ui_out == nullptr || epd_out == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  *ui_out = nullptr;
  *epd_out = nullptr;

  ssd1680x::Config cfg;
  cfg.host = GO_SPI_HOST;
  cfg.mirror_x = true;
  cfg.pins.busy = GO_EPD_BUSY_GPIO;
  cfg.pins.rst = GO_EPD_RST_GPIO;
  cfg.pins.dc = GO_EPD_DC_GPIO;
  cfg.pins.cs = GO_EPD_CS_GPIO;

  cfg.devcfg.clock_speed_hz = GO_EPD_CLOCK_SPEED_HZ;
  cfg.devcfg.mode = GO_EPD_SPI_MODE;
  cfg.devcfg.queue_size = GO_EPD_SPI_QUEUE_SIZE;
  cfg.devcfg.flags = GO_EPD_SPI_DEVICE_FLAGS;

  static ssd1680x::panels::GDEY0213B74 epd(cfg);
  static ui::DashboardUI ui(epd);

  esp_err_t err = ui.init();
  if (err != ESP_OK) {
    return err;
  }

  *ui_out = &ui;
  *epd_out = &epd;
  return ESP_OK;
}

static std::string buildSerialNumber() {
  uint8_t mac_address[6];
  esp_err_t err = esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
  if (err != ESP_OK) {
    ESP_LOGE(GO_TAG, "Failed to get MAC address (%s)", esp_err_to_name(err));
    return {};
  }

  char result[13] = {0};
  snprintf(result, sizeof(result), "%02x%02x%02x%02x%02x%02x", mac_address[0], mac_address[1],
           mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
  std::string sn = std::string(result);

  return sn;
}

static bool wifi_connect(const std::string &sn) {
  std::string ssid = std::string("airgradient-") + sn;
  if (g_wifiManager.autoConnect(ssid.c_str(), "cleanair") == false) {
    ESP_LOGE(GO_TAG, "Failed connect to WiFi");
    return false;
  }
  return true;
}

void wifi_disconnect() { g_wifiManager.disconnect(true); }

static bool post_request(const std::string &sn, const std::string &data) {
  esp_http_client_config_t config = {};
  char url[96] = {0};
  (void)snprintf(url, sizeof(url), "http://hw.airgradient.com/sensors/airgradient:%s/measures",
                 sn.c_str());
  config.url = url;
  config.method = HTTP_METHOD_POST;
  config.cert_pem = nullptr;
  config.timeout_ms = 10000;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    ESP_LOGW(GO_TAG, "http client init failed");
    return false;
  }

  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, data.c_str(), data.length());

  const esp_err_t perr = esp_http_client_perform(client);
  if (perr != ESP_OK) {
    ESP_LOGW(GO_TAG, "http perform failed (%s)", esp_err_to_name(perr));
    esp_http_client_cleanup(client);
    return false;
  }

  const int responseCode = esp_http_client_get_status_code(client);
  if (responseCode != 200 && responseCode != 201) {
    static constexpr int MAX_LOG_BODY = 512;
    char body[MAX_LOG_BODY + 1];
    int total = 0;
    while (total < MAX_LOG_BODY) {
      const int n = esp_http_client_read(client, body + total, MAX_LOG_BODY - total);
      if (n <= 0) {
        break;
      }
      total += n;
    }
    body[total] = '\0';

    if (total > 0) {
      ESP_LOGW(GO_TAG, "http status=%d url=%s body=%s", responseCode, url, body);
    } else {
      ESP_LOGW(GO_TAG, "http status=%d url=%s (no body)", responseCode, url);
    }
  }
  esp_http_client_cleanup(client);
  return (responseCode == 200 || responseCode == 201);
}

static void initConsole() {
  fflush(stdout);
  fsync(fileno(stdout));
  esp_console_deinit();

  /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  /* Move the caret to the beginning of the next line on '\n' */
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  /* Enable blocking mode on stdin and stdout */
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  usb_serial_jtag_driver_config_t jtag_config = {
      .tx_buffer_size = 256,
      .rx_buffer_size = 256,
  };

  /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&jtag_config));

  /* Tell vfs to use usb-serial-jtag driver */
  usb_serial_jtag_vfs_use_driver();

  /* Initialize the console */
  esp_console_config_t console_config = {.max_cmdline_length = CONSOLE_MAX_CMDLINE_LENGTH,
                                         .max_cmdline_args = CONSOLE_MAX_CMDLINE_ARGS,
#if CONFIG_LOG_COLORS
                                         .hint_color = atoi(LOG_COLOR_CYAN)
#endif
  };
  ESP_ERROR_CHECK(esp_console_init(&console_config));
}

class GoController {
public:
  GoController(ButtonService *buttons, QueueHandle_t input_queue, sps30_handle_t sps30,
               i2c_master_bus_handle_t i2c_bus, GPSService *gps, ui::DashboardUI *ui,
               ssd1680x::panels::GDEY0213B74 *epd, NandStorageService *storage,
               drivers::BQ25629 *charger, uint32_t last_wdt_reset_ms, uint32_t last_bq_wdt_reset_ms)
      : buttons_(buttons), input_queue_(input_queue), sps30_(sps30), i2c_bus_(i2c_bus), gps_(gps),
        ui_(ui), epd_(epd), storage_(storage), charger_(charger),
        last_wdt_reset_ms_(last_wdt_reset_ms), last_bq_wdt_reset_ms_(last_bq_wdt_reset_ms) {}

  void OnButtonEvent(int32_t id, const ButtonService::Payload *p) {
    if (p == nullptr) {
      return;
    }
    if (input_queue_ == nullptr) {
      return;
    }

    const ButtonService::Event ev = static_cast<ButtonService::Event>(id);
    if (p->source == ButtonService::Source::Physical) {
      if (p->id == 0) {
        if (ev == ButtonService::Event::ShortPress) {
          GoInputEvent e;
          e.type = GoInputEventType::ButtonShort;
          ESP_LOGI(GO_TAG, "event: QON short");
          (void)xQueueSend(input_queue_, &e, 0);
        } else if (ev == ButtonService::Event::LongPress) {
          GoInputEvent e;
          e.type = GoInputEventType::ButtonLong;
          ESP_LOGI(GO_TAG, "event: QON long");
          (void)xQueueSend(input_queue_, &e, 0);
        }
      } else if (p->id == 1) {
        if (ev == ButtonService::Event::LongPress) {
          GoInputEvent e;
          e.type = GoInputEventType::BootLong;
          ESP_LOGI(GO_TAG, "event: BOOT long");
          (void)xQueueSend(input_queue_, &e, 0);
        }
      }
      return;
    }

    if (p->source == ButtonService::Source::Touch) {
      if (ev != ButtonService::Event::LongPress) {
        return;
      }

      GoInputEvent e;
      if (p->id == GO_TOUCH_RIGHT_ID) {
        e.type = GoInputEventType::TouchRightLong;
        ESP_LOGI(GO_TAG, "event: touch right long");
        (void)xQueueSend(input_queue_, &e, 0);
      } else if (p->id == GO_TOUCH_ENTER_ID) {
        e.type = GoInputEventType::TouchEnterLong;
        ESP_LOGI(GO_TAG, "event: touch enter long");
        (void)xQueueSend(input_queue_, &e, 0);
      }
      return;
    }
  }

  void Run(void) {
    _init();
    while (true) {
      _kick_watchdogs_if_needed();
      _poll_usb_c_if_needed();
      Inputs inputs = _poll_inputs();
      _step(inputs);
      sleep_ms(GO_MAIN_LOOP_DELAY_MS);
    }
  }

private:
  ButtonService *buttons_ = nullptr;
  QueueHandle_t input_queue_ = nullptr;
  sps30_handle_t sps30_ = nullptr;
  i2c_master_bus_handle_t i2c_bus_ = nullptr;
  GPSService *gps_ = nullptr;
  ui::DashboardUI *ui_ = nullptr;
  ssd1680x::panels::GDEY0213B74 *epd_ = nullptr;
  NandStorageService *storage_ = nullptr;
  drivers::BQ25629 *charger_ = nullptr;

  uint32_t last_wdt_reset_ms_ = 0;
  uint32_t tracking_session_id_ = 0;
  uint32_t last_bq_vbus_poll_ms_ = 0;
  uint32_t last_bq_wdt_reset_ms_ = 0;

  State _state = State::Idle;
  uint32_t _state_enter_ms = 0;
  uint32_t _last_idle_measure_ms = 0;
  bool _sync_started = false;
  bool _tracking_started = false;
  bool _shutdown_started = false;
  std::string _serial_number;
  bool _sync_wifi_connected = false;
  bool charger_vbus_seen_ = false;
  bool usb_c_adapter_present_ = false;
  drivers::VBusStatus last_vbus_status_ = drivers::VBusStatus::NO_ADAPTER;

  void _init(void) {
    tracking_session_id_ = RTC_TRACKING_SESSION_ID;
    _serial_number = buildSerialNumber();

    State last = RTC_LAST_STATE;
    if (!is_valid_rtc_state(last)) {
      last = State::Idle;
    }

    const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    State initial = State::Idle;
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
      initial = State::Idle;
    } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
      if (last == State::Tracking) {
        initial = State::Tracking;
      } else {
        ESP_LOGI(GO_TAG, "Tracking stopped");
        initial = State::Idle;
      }
    } else {
      initial = State::Idle;
    }

    ESP_LOGI(GO_TAG, "wake cause=%d last=%s initial=%s", (int)cause, state_name(last),
             state_name(initial));
    _transition(initial);
  }

  void _start_new_tracking_session(void) {
    static constexpr uint32_t MIN_ID = 10000;
    static constexpr uint32_t SPAN = 90000;
    const uint32_t new_id = (esp_random() % SPAN) + MIN_ID;
    RTC_TRACKING_SESSION_ID = new_id;
    tracking_session_id_ = new_id;
    ESP_LOGI(GO_TAG, "tracking session id=%05" PRIu32, tracking_session_id_);
  }

  Inputs _poll_inputs(void) {
    Inputs in;
    if (input_queue_ == nullptr) {
      return in;
    }

    GoInputEvent ev;
    while (xQueueReceive(input_queue_, &ev, 0) == pdTRUE) {
      if (ev.type == GoInputEventType::ButtonShort) {
        in.button_short = true;
      } else if (ev.type == GoInputEventType::ButtonLong) {
        in.button_long = true;
      } else if (ev.type == GoInputEventType::BootLong) {
        in.boot_long = true;
      } else if (ev.type == GoInputEventType::TouchRightLong) {
        in.touch_right_long = true;
      } else if (ev.type == GoInputEventType::TouchLeftLong) {
        in.touch_left_long = true;
      } else if (ev.type == GoInputEventType::TouchEnterLong) {
        in.touch_enter_long = true;
      }
    }
    return in;
  }

  void _clear_tracking_logs(void) {
    if (storage_ == nullptr) {
      ESP_LOGW(GO_TAG, "clear logs: storage not configured");
      return;
    }
    if (!storage_->is_ready()) {
      ESP_LOGW(GO_TAG, "clear logs: storage not ready");
      return;
    }

    const TickType_t to = pdMS_TO_TICKS(GO_NAND_CMD_TIMEOUT_MS);
    const esp_err_t err = storage_->clear_sync(to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "clear logs: failed: %s", esp_err_to_name(err));
      return;
    }
    ESP_LOGI(GO_TAG, "clear logs: ok");
  }

  void _update_battery_ui(void) {
    if (ui_ == nullptr) {
      return;
    }

    if (charger_ == nullptr) {
      (void)ui_->set_battery_percent(-1);
      return;
    }

    uint8_t perc = 0;
    const esp_err_t err = charger_->estimate_battery_percent(perc);
    if (err != ESP_OK) {
      (void)ui_->set_battery_percent(-1);
      return;
    }

    (void)ui_->set_battery_percent((int)perc);
  }

  void _step(const Inputs &in) {
    switch (_state) {
    case State::Idle:
      _state_idle(in);
      break;
    case State::Inactive:
      _state_inactive(in);
      break;
    case State::Sync:
      _state_sync(in);
      break;
    case State::Tracking:
      _state_tracking(in);
      break;
    case State::Shutdown:
      _state_shutdown(in);
      break;
    }
  }

  void _transition(State next) {
    if (next == _state) {
      return;
    }

    ESP_LOGI(GO_TAG, "state %s -> %s", state_name(_state), state_name(next));
    _state = next;
    _state_enter_ms = now_ms();

    if (_state == State::Idle) {
      _last_idle_measure_ms = 0;
      _sync_started = false;
      _tracking_started = false;
      _shutdown_started = false;
    }
    if (_state == State::Sync) {
      _sync_started = false;
    }
    if (_state == State::Tracking) {
      _tracking_started = false;
    }
    if (_state == State::Shutdown) {
      _shutdown_started = false;
    }
  }

  void _state_idle(const Inputs &in) {
    _kick_watchdogs_if_needed();

    // Transitions from diagram.
    if (in.touch_right_long) {
      _start_new_tracking_session();
      _transition(State::Tracking);
      return;
    }
    if (in.touch_enter_long) {
      _transition(State::Sync);
      return;
    }
    if (in.boot_long) {
      _clear_tracking_logs();
      return;
    }
    if (in.button_long) {
      _transition(State::Shutdown);
      return;
    }

#if NO_INACTIVE_NO_SLEEP == 0
    if (in.button_short) {
      _transition(State::Inactive);
      return;
    }

    // Auto-inactive after timeout
    const uint32_t inactive_elapsed_ms = now_ms() - _state_enter_ms;
    if (inactive_elapsed_ms >= (uint32_t)GO_IDLE_INACTIVE_TIMEOUT_MS) {
      _transition(State::Inactive);
      return;
    }
#endif

    // Periodic measurement + display.
    if (_last_idle_measure_ms == 0) {
      _last_idle_measure_ms = now_ms();
    }
    const uint32_t measure_elapsed_ms = now_ms() - _last_idle_measure_ms;
    if (measure_elapsed_ms >= (uint32_t)GO_IDLE_MEASURE_INTERVAL_MS) {
      _idle_measure_and_display();
      _last_idle_measure_ms = now_ms();
    }
  }

  void _kick_watchdogs_if_needed(void) {
    const uint32_t now = now_ms();
    const uint32_t ext_elapsed_ms = now - last_wdt_reset_ms_;
    if (ext_elapsed_ms >= GO_WDT_RESET_INTERVAL_MS) {
      reset_ext_watchdog();
      last_wdt_reset_ms_ = now;
    }

    if (charger_ != nullptr) {
      const uint32_t bq_elapsed_ms = now - last_bq_wdt_reset_ms_;
      if (bq_elapsed_ms >= GO_BQ_WDT_RESET_INTERVAL_MS) {
        const esp_err_t err = charger_->reset_watchdog();
        if (err != ESP_OK) {
          ESP_LOGW(GO_TAG, "BQ25629 watchdog reset failed: %s", esp_err_to_name(err));
        }
        last_bq_wdt_reset_ms_ = now;
      }
    }
  }

  void _poll_usb_c_if_needed(void) {
    if (charger_ == nullptr) {
      return;
    }

    const uint32_t now = now_ms();
    if ((now - last_bq_vbus_poll_ms_) < GO_BQ_VBUS_POLL_INTERVAL_MS) {
      return;
    }
    last_bq_vbus_poll_ms_ = now;

    drivers::VBusStatus vbus = drivers::VBusStatus::NO_ADAPTER;
    const esp_err_t err = charger_->get_vbus_status(vbus);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "BQ25629 get_vbus_status failed: %s", esp_err_to_name(err));
      return;
    }

    const bool adapter_present = is_usb_c_adapter_present(vbus);
    if (!charger_vbus_seen_) {
      charger_vbus_seen_ = true;
      usb_c_adapter_present_ = adapter_present;
      last_vbus_status_ = vbus;
      ESP_LOGI(GO_TAG, "USB-C initial: %s (vbus=%s)", adapter_present ? "plugged" : "unplugged",
               vbus_status_name(vbus));

      if (ui_ != nullptr) {
        (void)ui_->set_charging(adapter_present);
      }
      return;
    }

    if (vbus != last_vbus_status_) {
      ESP_LOGI(GO_TAG, "BQ25629 vbus: %s -> %s", vbus_status_name(last_vbus_status_),
               vbus_status_name(vbus));
    }

    if (adapter_present != usb_c_adapter_present_) {
      if (ui_ != nullptr) {
        (void)ui_->set_charging(adapter_present);
      }
      if (adapter_present) {
        ESP_LOGI(GO_TAG, "USB-C plugged");
        _handle_usb_c_plugged_event();
      } else {
        ESP_LOGW(GO_TAG, "USB-C unplugged: wait PMID to 5V then re-init SPS30");
        _enable_pmid_wait_and_reinit_sps30("USB-C unplugged");
      }
      usb_c_adapter_present_ = adapter_present;
    }

    last_vbus_status_ = vbus;
  }

  void _handle_usb_c_plugged_event(void) {
    if (charger_ == nullptr) {
      ESP_LOGW(GO_TAG, "USB-C plugged: charger unavailable");
      return;
    }

    drivers::BQ25629_ADC_Data adc = {};
    const esp_err_t adc_err = charger_->read_adc(adc);
    if (adc_err == ESP_OK) {
      if (adc.vpmid_mv >= GO_BQ_VPMID_READY_MV) {
        ESP_LOGI(GO_TAG, "USB-C plugged: PMID=%umV ready, re-initializing SPS30",
                 (unsigned)adc.vpmid_mv);
        _reinit_sps30("USB-C plugged");
        return;
      }
      ESP_LOGW(GO_TAG, "USB-C plugged: PMID=%umV not ready, enabling PMID boost",
               (unsigned)adc.vpmid_mv);
    } else {
      ESP_LOGW(GO_TAG, "USB-C plugged: PMID read failed (%s), enabling PMID boost",
               esp_err_to_name(adc_err));
    }

    _enable_pmid_wait_and_reinit_sps30("USB-C plugged");
  }

  void _enable_pmid_wait_and_reinit_sps30(const char *reason) {
    if (charger_ == nullptr) {
      ESP_LOGW(GO_TAG, "PMID wait skipped: charger unavailable");
      return;
    }

    esp_err_t err = charger_->enable_pmid_5v_boost();
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "PMID boost enable failed: %s", esp_err_to_name(err));
      return;
    }

    uint32_t last_log_ms = 0;
    uint32_t last_reenable_ms = now_ms();
    while (true) {
      _kick_watchdogs_if_needed();

      drivers::BQ25629_ADC_Data adc = {};
      err = charger_->read_adc(adc);
      if (err == ESP_OK) {
        if (adc.vpmid_mv >= GO_BQ_VPMID_READY_MV) {
          ESP_LOGI(GO_TAG, "%s: PMID ready %umV, re-initializing SPS30", reason,
                   (unsigned)adc.vpmid_mv);
          break;
        }

        const uint32_t now = now_ms();
        if ((now - last_log_ms) >= 1000) {
          ESP_LOGI(GO_TAG, "%s: waiting PMID %umV (target >= %umV)", reason, (unsigned)adc.vpmid_mv,
                   (unsigned)GO_BQ_VPMID_READY_MV);
          last_log_ms = now;
        }
      } else {
        const uint32_t now = now_ms();
        if ((now - last_log_ms) >= 1000) {
          ESP_LOGW(GO_TAG, "%s: PMID ADC read failed while waiting: %s", reason,
                   esp_err_to_name(err));
          last_log_ms = now;
        }
      }

      const uint32_t now = now_ms();
      if ((now - last_reenable_ms) >= GO_BQ_VPMID_REENABLE_INTERVAL_MS) {
        (void)charger_->enable_pmid_5v_boost();
        last_reenable_ms = now;
      }

      sleep_ms(GO_BQ_VPMID_POLL_INTERVAL_MS);
    }

    _reinit_sps30(reason);
  }

  void _reinit_sps30(const char *reason) {
    if (i2c_bus_ == nullptr) {
      ESP_LOGW(GO_TAG, "SPS30 re-init skipped: I2C bus unavailable");
      return;
    }

    if (sps30_ != nullptr) {
      const esp_err_t del_err = sps30_delete(sps30_);
      if (del_err != ESP_OK) {
        ESP_LOGW(GO_TAG, "SPS30 delete failed before re-init: %s", esp_err_to_name(del_err));
        return;
      }
      sps30_ = nullptr;
    }

    sps30_handle_t new_sps30 = nullptr;
    const esp_err_t init_err = init_sps30_sensor(i2c_bus_, &new_sps30);
    if (init_err != ESP_OK) {
      ESP_LOGW(GO_TAG, "SPS30 re-init failed (%s): %s", reason, esp_err_to_name(init_err));
      return;
    }

    sps30_ = new_sps30;
    ESP_LOGI(GO_TAG, "SPS30 re-initialized (%s)", reason);
  }

  void _state_inactive(const Inputs &in) {
    // INACTIVE: deep sleep until physical button is pressed.
    // Note: deep sleep resets the chip; wake handling/persistence comes later.
    (void)in;
    _inactive_enter_deep_sleep();
  }

  void _state_sync(const Inputs &in) {
    (void)in;
    // SYNC: connect to Wi-Fi and send stored data; then return to IDLE.
    if (!_sync_started) {
      _sync_started = true;
      _sync_begin();
    }

    const bool finished = _sync_step();
    if (finished) {
      _sync_end();
      _transition(State::Idle);
      return;
    }
  }

  void _state_tracking(const Inputs &in) {
    // TRACKING: boot -> measure -> save -> display -> sleep.
    // Diagram: touch (long) toggles back to IDLE.
    if (in.touch_right_long) {
#if NO_INACTIVE_NO_SLEEP == 1
      // In dev mode we don't reboot between modes, but TRACKING deep-sleeps the panel after
      // full_refresh(). Ensure we wake and restore basemap prerequisites before switching to
      // IDLE (which uses partial refresh).
      if (ui_ != nullptr) {
        (void)ui_->set_tracking(false);
        (void)ui_->set_syncing(false);
        (void)ui_->set_gps_fixed(false);
        const esp_err_t err = ui_->full_refresh();
        if (err != ESP_OK) {
          ESP_LOGW(GO_TAG, "ui full_refresh failed: %s", esp_err_to_name(err));
        }
      }
#endif
      _transition(State::Idle);
      return;
    }

    if (!_tracking_started) {
      _tracking_started = true;
      _tracking_begin();
    }

    const bool finished = _tracking_step();
    if (finished) {
      _tracking_end();
      _tracking_enter_sleep();
      return;
    }
  }

  void _state_shutdown(const Inputs &in) {
    (void)in;
    if (_shutdown_started) {
      return;
    }
    _shutdown_started = true;
    _shutdown_now();
  }

  void _shutdown_now(void) {
    ESP_LOGI(GO_TAG, "shutdown: begin");

    // Ensure we have time to complete slow steps.
    reset_ext_watchdog();
    last_wdt_reset_ms_ = now_ms();

    // Stop button processing/interrupts so we don't fight I2C while shutting down.
    if (buttons_ != nullptr) {
      const esp_err_t err = buttons_->pre_light_sleep();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: buttons pre_light_sleep failed: %s", esp_err_to_name(err));
      }
    }

    // Best-effort: disconnect Wi-Fi if it was enabled.
    if (_sync_wifi_connected) {
      wifi_disconnect();
      _sync_wifi_connected = false;
    }

    // Flush queued storage writes.
    if (storage_ != nullptr && storage_->is_ready()) {
      const TickType_t to = pdMS_TO_TICKS(GO_NAND_CMD_TIMEOUT_MS);
      const esp_err_t err = storage_->flush_sync(to);
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: storage flush failed: %s", esp_err_to_name(err));
      }
    }

    // Clear the display and put the panel to sleep.
    if (ui_ != nullptr) {
      ESP_LOGI(GO_TAG, "shutdown: display clear");
      const uint32_t t0 = now_ms();
      const esp_err_t err = ui_->clear_and_sleep();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: ui clear_and_sleep failed: %s", esp_err_to_name(err));
      }
      ESP_LOGI(GO_TAG, "shutdown: display clear done (%" PRIu32 "ms)", now_ms() - t0);
    }
    // else if (epd_ != nullptr) {
    //   ESP_LOGI(GO_TAG, "shutdown: display clear (raw)");
    //   const uint32_t t0 = now_ms();
    //   esp_err_t err = epd_->ensure_init_full();
    //   if (err != ESP_OK) {
    //     ESP_LOGW(GO_TAG, "shutdown: epd ensure_init_full failed: %s", esp_err_to_name(err));
    //   } else {
    //     err = epd_->clear_white();
    //     if (err != ESP_OK) {
    //       ESP_LOGW(GO_TAG, "shutdown: epd clear_white failed: %s", esp_err_to_name(err));
    //     }
    //     sleep_ms(4000);
    //     err = epd_->deep_sleep();
    //     if (err != ESP_OK) {
    //       ESP_LOGW(GO_TAG, "shutdown: epd deep_sleep failed: %s", esp_err_to_name(err));
    //     }
    //   }
    //   ESP_LOGI(GO_TAG, "shutdown: display clear done (%" PRIu32 "ms)", now_ms() - t0);
    // }

    // Cut PM sensor rail.
    (void)gpio_set_level(GO_PM_POWER_GPIO, 0);

    // Stop GPS task/UART.
    if (gps_ != nullptr) {
      const esp_err_t err = gps_->stop();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: gps stop failed: %s", esp_err_to_name(err));
      }
    }

    // Enter ship mode: BATFET disconnects and system powers off.
    if (charger_ == nullptr) {
      ESP_LOGW(GO_TAG, "shutdown: charger not available; ship mode skipped");
    } else {
      const esp_err_t err = charger_->enter_ship_mode();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: enter_ship_mode failed: %s", esp_err_to_name(err));
      }
    }

    // waiting an actual shutdown from BMS
    esp_deep_sleep_start();
  }

  // ----- Placeholder implementations (fill in later) -----

  void _idle_measure_and_display(void) {
    if (sps30_ == nullptr) {
      ESP_LOGW(GO_TAG, "SPS30 not initialized");
      return;
    }

    sps30_measurement_t m;
    const esp_err_t err = sps30_read_measurement(sps30_, &m);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "SPS30 read failed: %s", esp_err_to_name(err));
      return;
    }

    const float pm25 = m.pm2p5_mass;
    ESP_LOGI(GO_TAG, "pm25: %.1f", (double)pm25);

    GPSService::Data d;
    bool gps_ok = false;
    if (gps_ != nullptr) {
      d = gps_->get();
      gps_ok = true;
      log_gps_data(d);
    }

    if (ui_ != nullptr) {
      (void)ui_->set_tracking(false);
      (void)ui_->set_syncing(false);
      (void)ui_->set_gps_fixed(gps_ok && d.fix_valid);
      (void)ui_->set_pm25_ugm3(pm25);

      _update_battery_ui();

      if (gps_ok && d.utc.time_valid) {
        (void)ui_->set_time_hm(d.utc.hour, d.utc.min);
      }
      const esp_err_t ui_err = ui_->refresh();
      if (ui_err != ESP_OK) {
        ESP_LOGW(GO_TAG, "ui refresh failed: %s", esp_err_to_name(ui_err));
      }
    }
  }

  void _inactive_enter_deep_sleep(void) {
    // TODO: configure wakeup source (physical button) and enter deep sleep.
    if (epd_ != nullptr) {
      const esp_err_t err = epd_->deep_sleep();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "epd deep_sleep failed: %s", esp_err_to_name(err));
      }
    }

    ESP_LOGI(GO_TAG, "inactive: entering deep sleep (stub)");
    esp_err_t err = ESP_OK;
    if (buttons_ != nullptr) {
      err = buttons_->enable_deep_sleep_wakeup();
    } else {
      ESP_LOGE(GO_TAG, "deep sleep wake config failed: buttons not initialized");
      return;
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "deep sleep wake config failed: %s", esp_err_to_name(err));
    }

    (void)esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    RTC_LAST_STATE = State::Inactive;
    esp_deep_sleep_start();
  }

  void _sync_begin(void) {
    ESP_LOGI(GO_TAG, "sync: begin");

    if (ui_ != nullptr) {
      (void)ui_->set_tracking(false);
      (void)ui_->set_syncing(true);
      bool gf = false;
      if (gps_ != nullptr) {
        const GPSService::Data d = gps_->get();
        gf = d.fix_valid;
      }
      (void)ui_->set_gps_fixed(gf);
      const esp_err_t ui_err = ui_->refresh();
      if (ui_err != ESP_OK) {
        ESP_LOGW(GO_TAG, "ui refresh failed: %s", esp_err_to_name(ui_err));
      }
    }

    _sync_wifi_connected = wifi_connect(_serial_number);
    if (!_sync_wifi_connected) {
      ESP_LOGW(GO_TAG, "sync: wifi connect failed");
    }
  }

  bool _sync_step(void) {
    if (!_sync_wifi_connected) {
      ESP_LOGW(GO_TAG, "sync: wifi not connected");
      return true;
    }
    if (storage_ == nullptr) {
      ESP_LOGW(GO_TAG, "sync: storage not configured");
      return true;
    }
    if (!storage_->is_ready()) {
      ESP_LOGW(GO_TAG, "sync: storage not ready");
      return true;
    }

    const TickType_t to = pdMS_TO_TICKS(GO_SYNC_CMD_TIMEOUT_MS);
    const uint64_t interval_ms = (uint64_t)GO_TRACKING_SLEEP_INTERVAL_S * 1000ULL;

    uint32_t total = 0;
    esp_err_t err = storage_->get_count_sync(&total, to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "sync: get_count failed: %s", esp_err_to_name(err));
      return true;
    }

    if (total == 0) {
      ESP_LOGI(GO_TAG, "sync: nothing to send");
      return true;
    }

    ESP_LOGI(GO_TAG, "sync: total records=%" PRIu32, total);

    uint32_t idx = 0;
    bool any_route_sent = false;

    while (idx < total) {
      NandStorageService::Record first;
      uint32_t nread = 0;
      err = storage_->read_range_sync(idx, &first, 1, &nread, to);
      if (err != ESP_OK || nread != 1) {
        ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", idx, esp_err_to_name(err));
        ESP_LOGW(GO_TAG, "sync: failed; keeping log");
        return true;
      }

      const uint32_t route_id = first.id;
      uint32_t route_start = idx;
      uint32_t route_end = route_start;

      while (route_end < total) {
        NandStorageService::Record r;
        nread = 0;
        err = storage_->read_range_sync(route_end, &r, 1, &nread, to);
        if (err != ESP_OK || nread != 1) {
          ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", route_end,
                   esp_err_to_name(err));
          ESP_LOGW(GO_TAG, "sync: failed; keeping log");
          return true;
        }
        if (r.id != route_id) {
          break;
        }
        route_end += 1;
      }

      ESP_LOGI(GO_TAG, "sync: route=%" PRIu32 " records=%" PRIu32, route_id,
               route_end - route_start);

      bool route_ok = true;

      // Find the first non-zero GPS timestamp for this route so we can backfill earlier
      // records that have timestamp_ms==0.
      bool have_anchor = false;
      uint32_t anchor_idx = 0;
      uint64_t anchor_ts_ms = 0;
      for (uint32_t probe = route_start; probe < route_end; ++probe) {
        NandStorageService::Record r;
        nread = 0;
        err = storage_->read_range_sync(probe, &r, 1, &nread, to);
        if (err != ESP_OK || nread != 1) {
          ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", probe, esp_err_to_name(err));
          ESP_LOGW(GO_TAG, "sync: failed; keeping log");
          return true;
        }
        if (r.timestamp_ms != 0) {
          have_anchor = true;
          anchor_idx = probe;
          anchor_ts_ms = r.timestamp_ms;
          break;
        }
      }

      if (!have_anchor) {
        // No GPS time ever became valid for this route; we can't synthesize timestamps.
        ESP_LOGW(GO_TAG, "sync: route=%" PRIu32 " has no timestamps; skipping", route_id);
        route_ok = false;
      }

      uint64_t last_ts_ms = 0;
      uint32_t cur = route_start;

      while (route_ok && cur < route_end) {
        NandStorageService::Record batch[GO_SYNC_BATCH_MAX];
        uint64_t ts_ms[GO_SYNC_BATCH_MAX];
        uint32_t n = 0;

        while (n < GO_SYNC_BATCH_MAX && cur < route_end) {
          NandStorageService::Record r;
          nread = 0;
          err = storage_->read_range_sync(cur, &r, 1, &nread, to);
          if (err != ESP_OK || nread != 1) {
            ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", cur, esp_err_to_name(err));
            ESP_LOGW(GO_TAG, "sync: failed; keeping log");
            return true;
          }

          const uint32_t abs_idx = cur;
          uint64_t t = r.timestamp_ms;
          if (t == 0) {
            if (last_ts_ms != 0) {
              t = last_ts_ms + interval_ms;
            } else if (abs_idx < anchor_idx) {
              const uint32_t diff = anchor_idx - abs_idx;
              const uint64_t backfill = (uint64_t)diff * interval_ms;
              if (anchor_ts_ms <= backfill) {
                t = 0;
              } else {
                t = anchor_ts_ms - backfill;
              }
            } else {
              // abs_idx==anchor_idx should have had a non-zero timestamp.
              t = 0;
            }
          }
          if (t == 0) {
            ESP_LOGW(GO_TAG, "sync: route=%" PRIu32 " idx=%" PRIu32 " timestamp unresolved",
                     route_id, abs_idx);
            route_ok = false;
            break;
          }
          last_ts_ms = t;

          batch[n] = r;
          ts_ms[n] = t;
          n += 1;
          cur += 1;
        }

        if (!route_ok) {
          break;
        }
        if (n == 0) {
          break;
        }

        // Stable sort by timestamp ascending.
        for (uint32_t i = 1; i < n; ++i) {
          const NandStorageService::Record r = batch[i];
          const uint64_t t = ts_ms[i];
          uint32_t j = i;
          while (j > 0 && ts_ms[j - 1] > t) {
            batch[j] = batch[j - 1];
            ts_ms[j] = ts_ms[j - 1];
            j -= 1;
          }
          batch[j] = r;
          ts_ms[j] = t;
        }

        // Build JSON payload.
        cJSON *root = cJSON_CreateObject();
        cJSON *arr = cJSON_CreateArray();
        cJSON_AddItemToObject(root, "measures", arr);

        for (uint32_t i = 0; i < n; ++i) {
          cJSON *m = cJSON_CreateObject();
          char date[32];
          format_rfc3339_utc(ts_ms[i], date, sizeof(date));
          cJSON_AddStringToObject(m, "date", date);

          if (batch[i].latitude_e7 == INT32_MIN || batch[i].longitude_e7 == INT32_MIN) {
            cJSON_AddNullToObject(m, "lat");
            cJSON_AddNullToObject(m, "lng");
          } else {
            cJSON_AddNumberToObject(m, "lat", (double)batch[i].latitude_e7 / 10000000.0);
            cJSON_AddNumberToObject(m, "lng", (double)batch[i].longitude_e7 / 10000000.0);
          }

          if (batch[i].pm25_ugm3_x10 == 0xFFFF) {
            cJSON_AddNullToObject(m, "pm02");
          } else {
            cJSON_AddNumberToObject(m, "pm02", (double)batch[i].pm25_ugm3_x10 / 10.0);
          }

          cJSON_AddNumberToObject(m, "route", (double)batch[i].id);
          cJSON_AddItemToArray(arr, m);
        }

        char *json = cJSON_PrintUnformatted(root);
        std::string payload;
        if (json != nullptr) {
          payload.assign(json);
          cJSON_free(json);
        }
        cJSON_Delete(root);

        if (payload.empty()) {
          ESP_LOGW(GO_TAG, "sync: json build failed");
          route_ok = false;
          break;
        }

        ESP_LOGI(GO_TAG, "sync: post route=%" PRIu32 " n=%" PRIu32, route_id, n);
        if (!post_request(_serial_number, payload)) {
          ESP_LOGW(GO_TAG, "sync: post failed; skipping route=%" PRIu32, route_id);
          route_ok = false;
          break;
        }
      }

      if (route_ok) {
        any_route_sent = true;
        ESP_LOGI(GO_TAG, "sync: route=%" PRIu32 " ok", route_id);
      }

      idx = route_end;
    }

    if (!any_route_sent) {
      ESP_LOGW(GO_TAG, "sync: no routes sent; keeping log");
      return true;
    }

    err = storage_->clear_sync(to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "sync: clear failed: %s", esp_err_to_name(err));
      return true;
    }

    ESP_LOGI(GO_TAG, "sync: cleared");
    return true;
  }

  void _sync_end(void) {
    // TODO: stop Wi-Fi / cleanup.
    ESP_LOGI(GO_TAG, "sync: end");
    if (_sync_wifi_connected) {
      wifi_disconnect();
      _sync_wifi_connected = false;
    }

    if (ui_ != nullptr) {
      (void)ui_->set_syncing(false);
      (void)ui_->set_tracking(false);
      // (void)ui_->set_gps_fixed(false);
      const esp_err_t ui_err = ui_->refresh();
      if (ui_err != ESP_OK) {
        ESP_LOGW(GO_TAG, "ui refresh failed: %s", esp_err_to_name(ui_err));
      }
    }
  }

  void _tracking_begin(void) {
    // TODO: initialize tracking cycle.
    ESP_LOGI(GO_TAG, "tracking: begin route=%05" PRIu32, tracking_session_id_);
  }

  bool _tracking_step(void) {
    // TODO: measure -> save to storage -> display.
    if (sps30_ == nullptr) {
      ESP_LOGW(GO_TAG, "SPS30 not initialized");
      return true;
    }

    sps30_measurement_t m;
    const esp_err_t err = sps30_read_measurement(sps30_, &m);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "SPS30 read failed: %s", esp_err_to_name(err));
      return true;
    }

    const float pm25 = m.pm2p5_mass;
    ESP_LOGI(GO_TAG, "pm25: %.1f", (double)pm25);

    GPSService::Data d;
    bool gps_ok = false;
    if (gps_ != nullptr) {
      d = gps_->get();
      gps_ok = true;
      log_gps_data(d);
    }

    if (ui_ != nullptr) {
      (void)ui_->set_tracking(true);
      (void)ui_->set_syncing(false);
      (void)ui_->set_gps_fixed(gps_ok && d.fix_valid);
      (void)ui_->set_pm25_ugm3(pm25);

      _update_battery_ui();

      if (gps_ok && d.utc.time_valid) {
        (void)ui_->set_time_hm(d.utc.hour, d.utc.min);
      }

#if TRACKING_DISPLAY_SLEEP == 1
      const esp_err_t ui_err = ui_->full_refresh();
      if (ui_err != ESP_OK) {
        ESP_LOGW(GO_TAG, "ui full_refresh failed: %s", esp_err_to_name(ui_err));
      }
#else
      const esp_err_t ui_err = ui_->refresh();
      if (ui_err != ESP_OK) {
        ESP_LOGW(GO_TAG, "ui refresh failed: %s", esp_err_to_name(ui_err));
      }
#endif
    }

#if TRACKING_DISPLAY_SLEEP == 1
    if (epd_ != nullptr) {
      const esp_err_t err = epd_->deep_sleep();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "epd deep_sleep failed: %s", esp_err_to_name(err));
      }
    }
#endif

    _tracking_write_record(pm25, gps_ok, d);
    return true;
  }

  void _tracking_write_record(float pm25, bool gps_ok, const GPSService::Data &d) {
    if (storage_ == nullptr) {
      return;
    }
    if (!storage_->is_ready()) {
      ESP_LOGW(GO_TAG, "storage not ready; skip write");
      return;
    }

    NandStorageService::Record r;
    r.id = tracking_session_id_;
    {
      uint64_t epoch_ms = 0;
      if (gps_ok) {
        (void)utc_to_epoch_ms(d.utc, &epoch_ms);
      }
      r.timestamp_ms = epoch_ms;
    }
    r.pm25_ugm3_x10 = pm25_to_x10(pm25);

    if (gps_ok && d.fix_valid) {
      r.latitude_e7 = deg_to_e7(d.latitude_deg);
      r.longitude_e7 = deg_to_e7(d.longitude_deg);
    } else {
      r.latitude_e7 = INT32_MIN;
      r.longitude_e7 = INT32_MIN;
    }

    const TickType_t to = pdMS_TO_TICKS(GO_NAND_CMD_TIMEOUT_MS);
    esp_err_t err = storage_->enqueue_record(r, true, to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "storage enqueue failed: %s", esp_err_to_name(err));
      return;
    }

#if NO_INACTIVE_NO_SLEEP == 0
    err = storage_->flush_sync(to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "storage flush failed: %s", esp_err_to_name(err));
      return;
    }
#endif
  }

  void _tracking_end(void) {
    // TODO: finalize tracking cycle (flush storage, etc.).
    ESP_LOGI(GO_TAG, "tracking: end (stub)");
  }

  void _tracking_enter_sleep(void) {

#if NO_INACTIVE_NO_SLEEP == 1
    vTaskDelay(pdMS_TO_TICKS(GO_TRACKING_SLEEP_INTERVAL_S * 1000));
    return;
#endif // NO_INACTIVE_NO_SLEEP == 1

    ESP_LOGI(GO_TAG, "tracking: entering deep sleep (stub)");

    esp_err_t err = ESP_OK;
    if (buttons_ != nullptr) {
      err = buttons_->enable_deep_sleep_wakeup();
    } else {
      ESP_LOGE(GO_TAG, "deep sleep wake config failed: buttons not initialized");
      return;
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "deep sleep wake config failed: %s", esp_err_to_name(err));
    }

    err = esp_sleep_enable_timer_wakeup((uint64_t)GO_TRACKING_SLEEP_INTERVAL_S * 1000000ULL);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "timer wake config failed: %s", esp_err_to_name(err));
    }

    RTC_LAST_STATE = State::Tracking;
    esp_deep_sleep_start();
  }
};

static void on_button_event(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
  (void)base;
  if (arg == nullptr) {
    return;
  }
  if (event_data == nullptr) {
    return;
  }
  GoController *go = static_cast<GoController *>(arg);
  go->OnButtonEvent(id, static_cast<const ButtonService::Payload *>(event_data));
}

extern "C" void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(100));
  // Re-initialize console after deepsleep
  esp_sleep_wakeup_cause_t wakeUpReason = esp_sleep_get_wakeup_cause();
  if (wakeUpReason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    initConsole();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  esp_log_level_set(GO_TAG, ESP_LOG_INFO);
  sleep_ms(GO_BOOT_DELAY_MS);

  ESP_ERROR_CHECK(init_ext_watchdog());
  reset_ext_watchdog();
  const uint32_t wdt_last_reset_ms = now_ms();
  uint32_t bq_wdt_last_reset_ms = now_ms();

  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = GO_I2C_MASTER_PORT;
  bus_cfg.sda_io_num = (gpio_num_t)GO_I2C_MASTER_SDA_IO;
  bus_cfg.scl_io_num = (gpio_num_t)GO_I2C_MASTER_SCL_IO;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = GO_I2C_GLITCH_IGNORE_CNT;
  bus_cfg.flags.enable_internal_pullup = GO_I2C_INTERNAL_PULLUPS;

  i2c_master_bus_handle_t bus_handle = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

  drivers::BQ25629 *charger_ptr = nullptr;
  {
    const esp_err_t err = init_charger(bus_handle, &charger_ptr);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "BQ25629 init failed: %s", esp_err_to_name(err));
      charger_ptr = nullptr;
    } else {
      bq_wdt_last_reset_ms = now_ms();
    }
  }

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = GO_SPI_MOSI_GPIO;
  buscfg.miso_io_num = GO_SPI_MISO_GPIO;
  buscfg.sclk_io_num = GO_SPI_SCLK_GPIO;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = GO_SPI_MAX_TRANSFER_SZ;
  ESP_ERROR_CHECK(spi_bus_initialize(GO_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  ui::DashboardUI *ui_ptr = nullptr;
  ssd1680x::panels::GDEY0213B74 *epd_ptr = nullptr;
  {
    const esp_err_t err = init_display(&ui_ptr, &epd_ptr);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "display init failed: %s", esp_err_to_name(err));
      ui_ptr = nullptr;
      epd_ptr = nullptr;
    }
  }

  static NandStorageService storage;
  NandStorageService *storage_ptr = nullptr;
  {
    NandStorageService::Config scfg;
    scfg.spi_host = GO_SPI_HOST;
    scfg.cs_pin = GO_NAND_CS_GPIO;
    scfg.clock_speed_hz = GO_NAND_CLOCK_SPEED_HZ;
    scfg.mount_path = GO_NAND_MOUNT_PATH;
    scfg.records_path = GO_NAND_RECORDS_PATH;

    esp_err_t err = storage.init(scfg);
    if (err == ESP_OK) {
      err = storage.start();
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "storage init/start failed: %s", esp_err_to_name(err));
      storage_ptr = nullptr;
    } else {
      storage_ptr = &storage;
      for (uint32_t i = 0; i < GO_NAND_READY_WAIT_RETRIES; ++i) {
        if (storage.is_ready()) {
          break;
        }
        sleep_ms(GO_NAND_READY_WAIT_DELAY_MS);
      }
      if (!storage.is_ready()) {
        ESP_LOGW(GO_TAG, "storage not ready yet; will start logging when ready");
      }
    }
  }

  ButtonService::Config bcfg;
  bcfg.qon_gpio = GO_BUTTON_QON_GPIO;
  bcfg.boot_gpio = GO_BUTTON_BOOT_GPIO;
  bcfg.cap_alert_gpio = GO_TOUCH_ALERT_GPIO;
  bcfg.cap_alert_active_low = GO_TOUCH_ALERT_ACTIVE_LOW;
  bcfg.qon_active_low = GO_BUTTON_QON_ACTIVE_LOW;
  bcfg.boot_active_low = GO_BUTTON_BOOT_ACTIVE_LOW;
  bcfg.cap_required = GO_TOUCH_REQUIRED;
  bcfg.debounce_ms = GO_BUTTON_DEBOUNCE_MS;
  bcfg.long_press_ms = GO_BUTTON_LONG_PRESS_MS;
  bcfg.touch_enable_mask = GO_TOUCH_ENABLE_MASK;
  bcfg.touch_interrupt_enable_mask = GO_TOUCH_INTERRUPT_ENABLE_MASK;
  bcfg.touch_calibrate = GO_TOUCH_CALIBRATE;
  bcfg.touch_calibrate_mask = GO_TOUCH_CALIBRATE_MASK;

  ButtonService buttons(bus_handle, bcfg);
  ESP_ERROR_CHECK(buttons.init());

  static GPSService gps;
  GPSService *gps_ptr = nullptr;
  {
    GPSService::Config gps_cfg;
    gps_cfg.uart_num = GO_GPS_UART_PORT;
    gps_cfg.rx_pin = GO_GPS_UART_RX_GPIO;
    gps_cfg.tx_pin = GO_GPS_UART_TX_GPIO;
    gps_cfg.baud_rate = GO_GPS_UART_BAUD;
    gps_cfg.log_raw_nmea = GO_GPS_LOG_RAW_NMEA;

    esp_err_t err = gps.init(gps_cfg);
    if (err == ESP_OK) {
      err = gps.start();
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "GPS init/start failed: %s", esp_err_to_name(err));
      gps_ptr = nullptr;
    } else {
      gps_ptr = &gps;
    }
  }

  sps30_handle_t sps30 = nullptr;
  {
    const esp_err_t err = init_sps30_sensor(bus_handle, &sps30);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "SPS30 init failed: %s", esp_err_to_name(err));
      sps30 = nullptr;
    }
  }

  QueueHandle_t input_queue = xQueueCreate((UBaseType_t)GO_INPUT_QUEUE_LEN, sizeof(GoInputEvent));
  if (input_queue == nullptr) {
    ESP_LOGE(GO_TAG, "input queue create failed");
    return;
  }

  GoController go(&buttons, input_queue, sps30, bus_handle, gps_ptr, ui_ptr, epd_ptr, storage_ptr,
                  charger_ptr, wdt_last_reset_ms, bq_wdt_last_reset_ms);
  ESP_ERROR_CHECK(
      esp_event_handler_register(BUTTON_SERVICE_EVENT, ESP_EVENT_ANY_ID, &on_button_event, &go));

  ESP_LOGI(GO_TAG, "boot");
  go.Run();
}
