
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "button_service.h"
#include "go_constants.h"

enum class State {
  Idle = 0,
  Inactive,
  Sync,
  Tracking,
};

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

static inline uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline void sleep_ms(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static const char* state_name(State s) {
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

class GoController {
 public:
  GoController(ButtonService* buttons, QueueHandle_t input_queue)
      : buttons_(buttons), input_queue_(input_queue) {
  }

  void OnButtonEvent(int32_t id, const ButtonService::Payload* p) {
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
        (void)xQueueSend(input_queue_, &e, 0);
      } else if (ev == ButtonService::Event::LongPress) {
        GoInputEvent e;
        e.type = GoInputEventType::ButtonLong;
        (void)xQueueSend(input_queue_, &e, 0);
      }
      return;
    }

    if (p->source == ButtonService::Source::Touch) {
      if (p->id == GO_TRACKING_TOUCH_ID && ev == ButtonService::Event::LongPress) {
        GoInputEvent e;
        e.type = GoInputEventType::TouchLong;
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
  ButtonService* buttons_ = nullptr;
  QueueHandle_t input_queue_ = nullptr;

  State _state = State::Idle;
  uint32_t _state_enter_ms = 0;
  uint32_t _last_idle_measure_ms = 0;
  bool _sync_started = false;
  bool _tracking_started = false;

  void _init(void) {
    // TODO: read persisted boot state (RTC/NVS) to pick initial state.
    _transition(State::Idle);
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

  void _step(const Inputs& in) {
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

  void _state_idle(const Inputs& in) {
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

    // Auto-inactive after timeout ("30s inactive").
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

  void _state_inactive(const Inputs& in) {
    // INACTIVE: deep sleep until physical button is pressed.
    // Note: deep sleep resets the chip; wake handling/persistence comes later.
    (void)in;
    if (GO_ENABLE_DEEP_SLEEP) {
      _inactive_enter_deep_sleep();
      return;
    }

    // Skeleton fallback (no deep sleep): stay here until a simulated press.
    if (in.button_short || in.button_long) {
      _transition(State::Idle);
      return;
    }
  }

  void _state_sync(const Inputs& in) {
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

  void _state_tracking(const Inputs& in) {
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
    // TODO: take measurements then display.
    ESP_LOGD(GO_TAG, "idle: measure + display");
  }

  void _inactive_enter_deep_sleep(void) {
    // TODO: configure wakeup source (physical button) and enter deep sleep.
    ESP_LOGI(GO_TAG, "inactive: entering deep sleep (stub)");
    if (buttons_ != nullptr) {
      const esp_err_t err = buttons_->enable_deep_sleep_wakeup();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "deep sleep wake config failed: %s", esp_err_to_name(err));
      }
    }
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
    return true;
  }

  void _tracking_end(void) {
    // TODO: finalize tracking cycle (flush storage, etc.).
    ESP_LOGI(GO_TAG, "tracking: end (stub)");
  }

  void _tracking_enter_sleep(void) {
    // TODO: configure wakeup sources/timer and enter tracking sleep.
    if (GO_ENABLE_DEEP_SLEEP) {
      ESP_LOGI(GO_TAG, "tracking: entering deep sleep (stub)");
      esp_deep_sleep_start();
      return;
    }
    ESP_LOGI(GO_TAG, "tracking: sleep disabled (stub)");
    _transition(State::Idle);
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
  esp_log_level_set(GO_TAG, ESP_LOG_INFO);
  sleep_ms(GO_BOOT_DELAY_MS);

  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = GO_I2C_MASTER_PORT;
  bus_cfg.sda_io_num = (gpio_num_t)GO_I2C_MASTER_SDA_IO;
  bus_cfg.scl_io_num = (gpio_num_t)GO_I2C_MASTER_SCL_IO;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = GO_I2C_GLITCH_IGNORE_CNT;
  bus_cfg.flags.enable_internal_pullup = GO_I2C_INTERNAL_PULLUPS;

  i2c_master_bus_handle_t bus_handle = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

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

  QueueHandle_t input_queue = xQueueCreate((UBaseType_t)GO_INPUT_QUEUE_LEN, sizeof(GoInputEvent));
  if (input_queue == nullptr) {
    ESP_LOGE(GO_TAG, "input queue create failed");
    return;
  }

  GoController go(&buttons, input_queue);
  ESP_ERROR_CHECK(
      esp_event_handler_register(BUTTON_SERVICE_EVENT, ESP_EVENT_ANY_ID, &on_button_event, &go));

  ESP_LOGI(GO_TAG, "boot");
  go.Run();
}
