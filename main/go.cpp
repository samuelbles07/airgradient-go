
#include <stdint.h>

#include <inttypes.h>
#include <stdio.h>
#include <fcntl.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "button_service.h"
#include "gps_service.h"
#include "go_constants.h"

#include "gdey0213b74.h"
#include "ui/dashboard_ui.h"

#include "sps30.h"

enum class State {
  Idle = 0,
  Inactive,
  Sync,
  Tracking,
};

RTC_DATA_ATTR static State RTC_LAST_STATE = State::Idle;

static bool is_valid_rtc_state(State s) {
  switch (s) {
  case State::Idle:
  case State::Inactive:
  case State::Sync:
  case State::Tracking:
    return true;
  }
  return false;
}

struct Inputs {
  bool button_short = false;
  bool button_long = false;
  bool touch_long = false;
};

enum class GoInputEventType : uint8_t {
  ButtonShort = 1,
  ButtonLong = 2,
  TouchLong = 3,
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
  }
  return "UNKNOWN";
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
               GPSService *gps, ui::DashboardUI *ui, ssd1680x::panels::GDEY0213B74 *epd,
               uint32_t last_wdt_reset_ms)
      : buttons_(buttons),
        input_queue_(input_queue),
        sps30_(sps30),
        gps_(gps),
        ui_(ui),
        epd_(epd),
        last_wdt_reset_ms_(last_wdt_reset_ms) {
  }

  void OnButtonEvent(int32_t id, const ButtonService::Payload *p) {
    if (p == nullptr) {
      return;
    }
    if (input_queue_ == nullptr) {
      return;
    }

    const ButtonService::Event ev = static_cast<ButtonService::Event>(id);
    if (p->source == ButtonService::Source::Physical) {
      if (ev == ButtonService::Event::ShortPress) {
        GoInputEvent e;
        e.type = GoInputEventType::ButtonShort;
        ESP_LOGI(GO_TAG, "event: physical short");
        (void)xQueueSend(input_queue_, &e, 0);
      } else if (ev == ButtonService::Event::LongPress) {
        GoInputEvent e;
        e.type = GoInputEventType::ButtonLong;
        ESP_LOGI(GO_TAG, "event: physical long");
        (void)xQueueSend(input_queue_, &e, 0);
      }
      return;
    }

    if (p->source == ButtonService::Source::Touch) {
      if (p->id == GO_TRACKING_TOUCH_ID && ev == ButtonService::Event::LongPress) {
        GoInputEvent e;
        e.type = GoInputEventType::TouchLong;
        ESP_LOGI(GO_TAG, "event: touch long");
        (void)xQueueSend(input_queue_, &e, 0);
      }
      return;
    }
  }

  void Run(void) {
    _init();
    while (true) {
      Inputs inputs = _poll_inputs();
      _step(inputs);
      sleep_ms(GO_MAIN_LOOP_DELAY_MS);
    }
  }

private:
  ButtonService *buttons_ = nullptr;
  QueueHandle_t input_queue_ = nullptr;
  sps30_handle_t sps30_ = nullptr;
  GPSService *gps_ = nullptr;
  ui::DashboardUI *ui_ = nullptr;
  ssd1680x::panels::GDEY0213B74 *epd_ = nullptr;

  uint32_t last_wdt_reset_ms_ = 0;

  State _state = State::Idle;
  uint32_t _state_enter_ms = 0;
  uint32_t _last_idle_measure_ms = 0;
  bool _sync_started = false;
  bool _tracking_started = false;

  void _init(void) {
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
      } else if (ev.type == GoInputEventType::TouchLong) {
        in.touch_long = true;
      }
    }
    return in;
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
    }
    if (_state == State::Sync) {
      _sync_started = false;
    }
    if (_state == State::Tracking) {
      _tracking_started = false;
    }
  }

  void _state_idle(const Inputs &in) {
    _kick_watchdog_if_needed();

    // Transitions from diagram.
    if (in.touch_long) {
      _transition(State::Tracking);
      return;
    }
    if (in.button_long) {
      _transition(State::Sync);
      return;
    }
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

  void _kick_watchdog_if_needed(void) {
    const uint32_t elapsed_ms = now_ms() - last_wdt_reset_ms_;
    if (elapsed_ms < GO_WDT_RESET_INTERVAL_MS) {
      return;
    }
    reset_ext_watchdog();
    last_wdt_reset_ms_ = now_ms();
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
    if (in.touch_long) {
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
      (void)ui_->set_pm25_ugm3(pm25);
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
    // TODO: start Wi-Fi and prepare to send stored data.
    ESP_LOGI(GO_TAG, "sync: begin (stub)");
  }

  bool _sync_step(void) {
    // TODO: advance sync state machine; return true when finished.
    return true;
  }

  void _sync_end(void) {
    // TODO: stop Wi-Fi / cleanup.
    ESP_LOGI(GO_TAG, "sync: end (stub)");
  }

  void _tracking_begin(void) {
    // TODO: initialize tracking cycle.
    ESP_LOGI(GO_TAG, "tracking: begin (stub)");
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
      (void)ui_->set_pm25_ugm3(pm25);
      if (gps_ok && d.utc.time_valid) {
        (void)ui_->set_time_hm(d.utc.hour, d.utc.min);
      }

      const esp_err_t ui_err = ui_->full_refresh();
      if (ui_err != ESP_OK) {
        ESP_LOGW(GO_TAG, "ui full_refresh failed: %s", esp_err_to_name(ui_err));
      }

    }

    if (epd_ != nullptr) {
      const esp_err_t err = epd_->deep_sleep();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "epd deep_sleep failed: %s", esp_err_to_name(err));
      }
    }
    return true;
  }

  void _tracking_end(void) {
    // TODO: finalize tracking cycle (flush storage, etc.).
    ESP_LOGI(GO_TAG, "tracking: end (stub)");
  }

  void _tracking_enter_sleep(void) {
    // TODO: configure wakeup sources/timer and enter tracking sleep.
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

  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = GO_I2C_MASTER_PORT;
  bus_cfg.sda_io_num = (gpio_num_t)GO_I2C_MASTER_SDA_IO;
  bus_cfg.scl_io_num = (gpio_num_t)GO_I2C_MASTER_SCL_IO;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = GO_I2C_GLITCH_IGNORE_CNT;
  bus_cfg.flags.enable_internal_pullup = GO_I2C_INTERNAL_PULLUPS;

  i2c_master_bus_handle_t bus_handle = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = GO_SPI_MOSI_GPIO;
  buscfg.miso_io_num = GO_SPI_MISO_GPIO;
  buscfg.sclk_io_num = GO_SPI_SCLK_GPIO;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = GO_SPI_MAX_TRANSFER_SZ;
  ESP_ERROR_CHECK(spi_bus_initialize(GO_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  ButtonService::Config bcfg;
  bcfg.physical_gpio = GO_BUTTON_PHYSICAL_GPIO;
  bcfg.cap_alert_gpio = GO_TOUCH_ALERT_GPIO;
  bcfg.cap_alert_active_low = GO_TOUCH_ALERT_ACTIVE_LOW;
  bcfg.physical_active_low = GO_BUTTON_PHYSICAL_ACTIVE_LOW;
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

  QueueHandle_t input_queue = xQueueCreate((UBaseType_t)GO_INPUT_QUEUE_LEN, sizeof(GoInputEvent));
  if (input_queue == nullptr) {
    ESP_LOGE(GO_TAG, "input queue create failed");
    return;
  }

  GoController go(&buttons, input_queue, sps30, gps_ptr, ui_ptr, epd_ptr, wdt_last_reset_ms);
  ESP_ERROR_CHECK(
      esp_event_handler_register(BUTTON_SERVICE_EVENT, ESP_EVENT_ANY_ID, &on_button_event, &go));

  ESP_LOGI(GO_TAG, "boot");
  go.Run();
}
