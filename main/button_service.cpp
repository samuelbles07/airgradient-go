#include "button_service.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

ESP_EVENT_DEFINE_BASE(BUTTON_SERVICE_EVENT);

static const char *TAG = "button_service";

static esp_err_t gpio_install_isr_service_once(int flags) {
  esp_err_t err = gpio_install_isr_service(flags);
  if (err == ESP_ERR_INVALID_STATE) {
    return ESP_OK;
  }
  return err;
}

uint32_t ButtonService::_now_ms() {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

ButtonService::ButtonService(i2c_master_bus_handle_t i2c_bus, const Config &cfg)
    : bus_(i2c_bus),
      cfg_(cfg),
      cap1203_(i2c_bus),
      cap_ready_(false),
      queue_(nullptr),
      task_(nullptr),
      task_suspended_(false),
      cap_isr_(),
      phy_isr_(),
      touch_timer_ctx_{{nullptr, Source::Touch, 0}, {nullptr, Source::Touch, 1},
                       {nullptr, Source::Touch, 2}},
      physical_timer_ctx_{nullptr, Source::Physical, 0},
      last_touch_mask_(0),
      touch_press_ms_{0, 0, 0},
      touch_long_fired_{false, false, false},
      touch_long_timer_{nullptr, nullptr, nullptr},
      physical_pressed_(false),
      physical_last_change_ms_(0),
      physical_press_ms_(0),
      physical_long_fired_(false),
      physical_long_timer_(nullptr) {
  cap_isr_.self = this;
  cap_isr_.source = Source::Touch;
  phy_isr_.self = this;
  phy_isr_.source = Source::Physical;

  for (int i = 0; i < 3; ++i) {
    touch_timer_ctx_[i].self = this;
    touch_timer_ctx_[i].source = Source::Touch;
    touch_timer_ctx_[i].id = (uint8_t)i;
  }
  physical_timer_ctx_.self = this;
  physical_timer_ctx_.source = Source::Physical;
  physical_timer_ctx_.id = 0;
}

ButtonService::~ButtonService() {
  (void)deinit();
}

esp_err_t ButtonService::init() {
  if (bus_ == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  if (cfg_.cap_alert_gpio == GPIO_NUM_MAX || cfg_.physical_gpio == GPIO_NUM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }
  if (cfg_.long_press_ms == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_RETURN_ON_ERROR(_init_event_loop(), TAG, "event loop init failed");
  ESP_RETURN_ON_ERROR(_init_gpio(), TAG, "gpio init failed");
  ESP_RETURN_ON_ERROR(_init_cap1203(), TAG, "cap1203 init failed");
  ESP_RETURN_ON_ERROR(_init_timers(), TAG, "timer init failed");

  if (queue_ == nullptr) {
    queue_ = xQueueCreate(16, sizeof(IsrEvent));
    if (queue_ == nullptr) {
      return ESP_ERR_NO_MEM;
    }
  }

  ESP_RETURN_ON_ERROR(_install_isr_handlers(), TAG, "isr init failed");

  if (task_ == nullptr) {
    BaseType_t ok = xTaskCreate(&ButtonService::_task_thunk, "button_service", 4096, this,
                               10, &task_);
    if (ok != pdPASS) {
      task_ = nullptr;
      return ESP_ERR_NO_MEM;
    }
  }

  return ESP_OK;
}

esp_err_t ButtonService::deinit() {
  (void)_rm_isr_handlers();

  if (task_ != nullptr && task_suspended_) {
    vTaskResume(task_);
    task_suspended_ = false;
  }

  if (task_ != nullptr) {
    vTaskDelete(task_);
    task_ = nullptr;
  }

  if (queue_ != nullptr) {
    vQueueDelete(queue_);
    queue_ = nullptr;
  }

  for (int i = 0; i < 3; ++i) {
    if (touch_long_timer_[i] != nullptr) {
      esp_timer_stop(touch_long_timer_[i]);
      esp_timer_delete(touch_long_timer_[i]);
      touch_long_timer_[i] = nullptr;
    }
  }

  if (physical_long_timer_ != nullptr) {
    esp_timer_stop(physical_long_timer_);
    esp_timer_delete(physical_long_timer_);
    physical_long_timer_ = nullptr;
  }

  cap_ready_ = false;
  last_touch_mask_ = 0;
  memset(touch_press_ms_, 0, sizeof(touch_press_ms_));
  memset(touch_long_fired_, 0, sizeof(touch_long_fired_));
  physical_pressed_ = false;
  physical_last_change_ms_ = 0;
  physical_press_ms_ = 0;
  physical_long_fired_ = false;
  task_suspended_ = false;

  return ESP_OK;
}

esp_err_t ButtonService::pre_light_sleep() {
  if (task_ != nullptr && xTaskGetCurrentTaskHandle() != task_) {
    vTaskSuspend(task_);
    task_suspended_ = true;
  }

  // Prevent ISRs from posting while we prepare state.
  ESP_RETURN_ON_ERROR(gpio_intr_disable(cfg_.cap_alert_gpio), TAG,
                      "disable cap gpio interrupt failed");
  ESP_RETURN_ON_ERROR(gpio_intr_disable(cfg_.physical_gpio), TAG,
                      "disable physical gpio interrupt failed");

  // Stop timers so they don't fire during sleep transition.
  for (int i = 0; i < 3; ++i) {
    if (touch_long_timer_[i] != nullptr) {
      (void)esp_timer_stop(touch_long_timer_[i]);
    }
  }
  if (physical_long_timer_ != nullptr) {
    (void)esp_timer_stop(physical_long_timer_);
  }

  // Clear any stale CAP1203 latch so ALERT# does not remain asserted.
  if (cap_ready_) {
    (void)cap1203_.clearInterrupt();
  }

  return ESP_OK;
}

esp_err_t ButtonService::post_light_sleep() {
  const uint32_t now = _now_ms();

  // Re-sync physical state.
  {
    const int level = gpio_get_level(cfg_.physical_gpio);
    bool active = false;
    if (cfg_.physical_active_low) {
      active = (level == 0);
    } else {
      active = (level != 0);
    }

    physical_pressed_ = active;
    physical_last_change_ms_ = now;
    physical_long_fired_ = false;
    if (active) {
      physical_press_ms_ = now;
      if (physical_long_timer_ != nullptr) {
        (void)esp_timer_stop(physical_long_timer_);
        (void)esp_timer_start_once(physical_long_timer_,
                                  (uint64_t)cfg_.long_press_ms * 1000ULL);
      }
    } else {
      physical_press_ms_ = 0;
      if (physical_long_timer_ != nullptr) {
        (void)esp_timer_stop(physical_long_timer_);
      }
    }
  }

  // Re-sync touch state and clear the latch.
  if (cap_ready_) {
    uint8_t mask = 0;
    if (cap1203_.readSensorInputStatus(&mask) == ESP_OK) {
      mask &= 0x07;
      last_touch_mask_ = mask;

      for (uint8_t i = 0; i < 3; ++i) {
        const bool pressed = (mask & (1U << i)) != 0;
        touch_long_fired_[i] = false;
        if (pressed) {
          touch_press_ms_[i] = now;
          if (touch_long_timer_[i] != nullptr) {
            (void)esp_timer_stop(touch_long_timer_[i]);
            (void)esp_timer_start_once(touch_long_timer_[i],
                                      (uint64_t)cfg_.long_press_ms * 1000ULL);
          }
        } else {
          touch_press_ms_[i] = 0;
          if (touch_long_timer_[i] != nullptr) {
            (void)esp_timer_stop(touch_long_timer_[i]);
          }
        }
      }
    }

    (void)cap1203_.clearInterrupt();
  }

  // Re-enable GPIO interrupts.
  ESP_RETURN_ON_ERROR(gpio_intr_enable(cfg_.cap_alert_gpio), TAG,
                      "enable cap gpio interrupt failed");
  ESP_RETURN_ON_ERROR(gpio_intr_enable(cfg_.physical_gpio), TAG,
                      "enable physical gpio interrupt failed");

  if (task_ != nullptr && task_suspended_) {
    vTaskResume(task_);
    task_suspended_ = false;
  }

  return ESP_OK;
}

esp_err_t ButtonService::_init_event_loop() {
  esp_err_t err = esp_event_loop_create_default();
  if (err == ESP_ERR_INVALID_STATE) {
    return ESP_OK;
  }
  return err;
}

esp_err_t ButtonService::_init_gpio() {
  gpio_config_t cap = {};
  cap.pin_bit_mask = (1ULL << cfg_.cap_alert_gpio);
  cap.mode = GPIO_MODE_INPUT;
  cap.pull_up_en = GPIO_PULLUP_ENABLE;
  cap.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cap.intr_type = GPIO_INTR_ANYEDGE;
  ESP_RETURN_ON_ERROR(gpio_config(&cap), TAG, "cap alert gpio config failed");

  gpio_config_t phy = {};
  phy.pin_bit_mask = (1ULL << cfg_.physical_gpio);
  phy.mode = GPIO_MODE_INPUT;
  phy.pull_up_en = GPIO_PULLUP_ENABLE;
  phy.pull_down_en = GPIO_PULLDOWN_DISABLE;
  phy.intr_type = GPIO_INTR_ANYEDGE;
  ESP_RETURN_ON_ERROR(gpio_config(&phy), TAG, "physical gpio config failed");

  const int level = gpio_get_level(cfg_.physical_gpio);
  bool active = false;
  if (cfg_.physical_active_low) {
    active = (level == 0);
  } else {
    active = (level != 0);
  }
  physical_pressed_ = active;
  return ESP_OK;
}

esp_err_t ButtonService::_init_cap1203() {
  cap_ready_ = false;

  const esp_err_t err = cap1203_.init(cfg_.cap1203);
  if (err != ESP_OK) {
    if (cfg_.cap_required) {
      return err;
    }
    ESP_LOGW(TAG, "CAP1203 init failed (%s); touch disabled", esp_err_to_name(err));
    return ESP_OK;
  }

  // Minimal deterministic setup for interrupt-driven status reads.
  ESP_RETURN_ON_ERROR(cap1203_.enableInputs(cfg_.touch_enable_mask), TAG,
                      "enable inputs failed");
  ESP_RETURN_ON_ERROR(
      cap1203_.setSensitivity(cfg_.touch_delta_sense, cfg_.touch_base_shift), TAG,
      "set sensitivity failed");
  for (uint8_t i = 0; i < 3; ++i) {
    ESP_RETURN_ON_ERROR(cap1203_.setThreshold(i, cfg_.touch_threshold[i]), TAG,
                        "set threshold failed");
  }
  ESP_RETURN_ON_ERROR(cap1203_.setInterruptEnable(cfg_.touch_interrupt_enable_mask), TAG,
                      "enable interrupts failed");
  if (cfg_.touch_calibrate) {
    ESP_RETURN_ON_ERROR(cap1203_.calibrate(cfg_.touch_calibrate_mask), TAG,
                        "calibrate failed");
  }
  (void)cap1203_.clearInterrupt();
  last_touch_mask_ = 0;

  cap_ready_ = true;
  return ESP_OK;
}

esp_err_t ButtonService::_init_timers() {
  if (physical_long_timer_ == nullptr) {
    esp_timer_create_args_t t = {};
    t.callback = &ButtonService::_timer_thunk;
    t.arg = (void *)&physical_timer_ctx_;
    t.dispatch_method = ESP_TIMER_TASK;
    t.name = "btn_phy_long";
    ESP_RETURN_ON_ERROR(esp_timer_create(&t, &physical_long_timer_), TAG,
                        "create physical timer failed");
  }

  for (int i = 0; i < 3; ++i) {
    if (touch_long_timer_[i] != nullptr) {
      continue;
    }

    esp_timer_create_args_t t = {};
    t.callback = &ButtonService::_timer_thunk;
    t.arg = (void *)&touch_timer_ctx_[i];
    t.dispatch_method = ESP_TIMER_TASK;
    t.name = "btn_touch_long";
    ESP_RETURN_ON_ERROR(esp_timer_create(&t, &touch_long_timer_[i]), TAG,
                        "create touch timer failed");
  }

  return ESP_OK;
}

esp_err_t ButtonService::_install_isr_handlers() {
  ESP_RETURN_ON_ERROR(gpio_install_isr_service_once(0), TAG, "gpio isr service failed");

  // Best-effort cleanup in case init() is called again.
  (void)gpio_isr_handler_remove(cfg_.cap_alert_gpio);
  (void)gpio_isr_handler_remove(cfg_.physical_gpio);

  ESP_RETURN_ON_ERROR(gpio_isr_handler_add(cfg_.cap_alert_gpio, &ButtonService::_gpio_isr,
                                          (void *)&cap_isr_),
                      TAG, "cap isr handler add failed");
  ESP_RETURN_ON_ERROR(gpio_isr_handler_add(cfg_.physical_gpio, &ButtonService::_gpio_isr,
                                          (void *)&phy_isr_),
                      TAG, "physical isr handler add failed");

  return ESP_OK;
}

esp_err_t ButtonService::_rm_isr_handlers() {
  if (cfg_.cap_alert_gpio != GPIO_NUM_MAX) {
    (void)gpio_isr_handler_remove(cfg_.cap_alert_gpio);
  }
  if (cfg_.physical_gpio != GPIO_NUM_MAX) {
    (void)gpio_isr_handler_remove(cfg_.physical_gpio);
  }
  return ESP_OK;
}

void IRAM_ATTR ButtonService::_gpio_isr(void *arg) {
  const IsrCtx *ctx = (const IsrCtx *)arg;
  if (ctx == nullptr || ctx->self == nullptr) {
    return;
  }

  IsrEvent ev;
  ev.source = ctx->source;
  (void)xQueueSendFromISR(ctx->self->queue_, &ev, nullptr);
}

void ButtonService::_task_thunk(void *arg) {
  static_cast<ButtonService *>(arg)->_task();
}

void ButtonService::_task() {
  IsrEvent ev;
  while (true) {
    if (xQueueReceive(queue_, &ev, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (ev.source == Source::Touch) {
      _handle_cap1203_irq();
    } else if (ev.source == Source::Physical) {
      _handle_physical_irq();
    }
  }
}

void ButtonService::_handle_cap1203_irq() {
  if (!cap_ready_) {
    return;
  }

  uint8_t mask = 0;
  if (cap1203_.readSensorInputStatus(&mask) != ESP_OK) {
    return;
  }
  mask &= 0x07;

  // Required for non-stuck status and to deassert ALERT#.
  (void)cap1203_.clearInterrupt();

  const uint32_t now = _now_ms();
  for (uint8_t i = 0; i < 3; ++i) {
    const bool was = (last_touch_mask_ & (1U << i)) != 0;
    const bool is = (mask & (1U << i)) != 0;

    if (is && !was) {
      touch_press_ms_[i] = now;
      touch_long_fired_[i] = false;

      Payload p = {.source = Source::Touch, .id = i, .touch_mask = mask, .duration_ms = 0};
      _emit(Event::Press, p);

      if (touch_long_timer_[i] != nullptr) {
        (void)esp_timer_stop(touch_long_timer_[i]);
        (void)esp_timer_start_once(touch_long_timer_[i], (uint64_t)cfg_.long_press_ms * 1000ULL);
      }
    } else if (!is && was) {
      uint32_t dur = 0;
      if (touch_press_ms_[i] != 0) {
        dur = now - touch_press_ms_[i];
      }

      if (touch_long_timer_[i] != nullptr) {
        (void)esp_timer_stop(touch_long_timer_[i]);
      }

      Payload p = {.source = Source::Touch, .id = i, .touch_mask = mask, .duration_ms = dur};
      _emit(Event::Release, p);

      if (!touch_long_fired_[i]) {
        if (dur >= cfg_.long_press_ms) {
          _emit(Event::LongPress, p);
        } else {
          _emit(Event::ShortPress, p);
        }
      }

      touch_press_ms_[i] = 0;
      touch_long_fired_[i] = false;
    }
  }

  last_touch_mask_ = mask;
}

void ButtonService::_handle_physical_irq() {
  const uint32_t now = _now_ms();
  if (physical_last_change_ms_ != 0 && (now - physical_last_change_ms_) < cfg_.debounce_ms) {
    return;
  }

  const int level = gpio_get_level(cfg_.physical_gpio);
  bool active = false;
  if (cfg_.physical_active_low) {
    active = (level == 0);
  } else {
    active = (level != 0);
  }
  if (active == physical_pressed_) {
    return;
  }

  physical_last_change_ms_ = now;

  if (active) {
    physical_pressed_ = true;
    physical_press_ms_ = now;
    physical_long_fired_ = false;

    Payload p = {.source = Source::Physical, .id = 0, .touch_mask = 0, .duration_ms = 0};
    _emit(Event::Press, p);

    if (physical_long_timer_ != nullptr) {
      (void)esp_timer_stop(physical_long_timer_);
      (void)esp_timer_start_once(physical_long_timer_, (uint64_t)cfg_.long_press_ms * 1000ULL);
    }
  } else {
    physical_pressed_ = false;
    uint32_t dur = 0;
    if (physical_press_ms_ != 0) {
      dur = now - physical_press_ms_;
    }

    if (physical_long_timer_ != nullptr) {
      (void)esp_timer_stop(physical_long_timer_);
    }

    Payload p = {.source = Source::Physical, .id = 0, .touch_mask = 0, .duration_ms = dur};
    _emit(Event::Release, p);

    if (!physical_long_fired_) {
      if (dur >= cfg_.long_press_ms) {
        _emit(Event::LongPress, p);
      } else {
        _emit(Event::ShortPress, p);
      }
    }

    physical_press_ms_ = 0;
    physical_long_fired_ = false;
  }
}

void ButtonService::_timer_thunk(void *arg) {
  const TimerCtx *ctx = (const TimerCtx *)arg;
  if (ctx == nullptr || ctx->self == nullptr) {
    return;
  }
  ctx->self->_on_long_press_timer(ctx->source, ctx->id);
}

void ButtonService::_on_long_press_timer(Source source, uint8_t id) {
  const uint32_t now = _now_ms();

  if (source == Source::Physical) {
    if (!physical_pressed_ || physical_long_fired_) {
      return;
    }
    physical_long_fired_ = true;
    uint32_t dur = 0;
    if (physical_press_ms_ != 0) {
      dur = now - physical_press_ms_;
    }
    Payload p = {.source = Source::Physical, .id = 0, .touch_mask = 0, .duration_ms = dur};
    _emit(Event::LongPress, p);
    return;
  }

  if (id >= 3) {
    return;
  }
  const bool pressed = (last_touch_mask_ & (1U << id)) != 0;
  if (!pressed || touch_long_fired_[id]) {
    return;
  }
  touch_long_fired_[id] = true;
  uint32_t dur = 0;
  if (touch_press_ms_[id] != 0) {
    dur = now - touch_press_ms_[id];
  }
  Payload p = {.source = Source::Touch, .id = id, .touch_mask = last_touch_mask_, .duration_ms = dur};
  _emit(Event::LongPress, p);
}

void ButtonService::_emit(Event ev, const Payload &p) {
  // Best-effort post.
  (void)esp_event_post(BUTTON_SERVICE_EVENT, (int32_t)ev, &p, sizeof(p), 0);
}

esp_err_t ButtonService::disable_wakeup_sources() {
  ESP_RETURN_ON_ERROR(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO), TAG,
                      "disable gpio wake failed");
  ESP_RETURN_ON_ERROR(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1), TAG,
                      "disable ext1 wake failed");
  return ESP_OK;
}

esp_err_t ButtonService::enable_light_sleep_wakeup() {
  ESP_RETURN_ON_ERROR(disable_wakeup_sources(), TAG, "disable wake failed");

  gpio_int_type_t cap_level = GPIO_INTR_HIGH_LEVEL;
  if (cfg_.cap_alert_active_low) {
    cap_level = GPIO_INTR_LOW_LEVEL;
  }

  gpio_int_type_t phy_level = GPIO_INTR_HIGH_LEVEL;
  if (cfg_.physical_active_low) {
    phy_level = GPIO_INTR_LOW_LEVEL;
  }

  ESP_RETURN_ON_ERROR(gpio_wakeup_enable(cfg_.cap_alert_gpio, cap_level), TAG,
                      "cap wake enable failed");
  ESP_RETURN_ON_ERROR(gpio_wakeup_enable(cfg_.physical_gpio, phy_level), TAG,
                      "physical wake enable failed");
  ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG, "gpio wakeup failed");

  return ESP_OK;
}

esp_err_t ButtonService::enable_deep_sleep_wakeup() {
  ESP_RETURN_ON_ERROR(disable_wakeup_sources(), TAG, "disable wake failed");

  uint64_t mask = 1ULL << (uint32_t)cfg_.physical_gpio;
  esp_sleep_ext1_wakeup_mode_t mode = ESP_EXT1_WAKEUP_ANY_HIGH;
  if (cfg_.physical_active_low) {
    mode = ESP_EXT1_WAKEUP_ANY_LOW;
  }
  return esp_sleep_enable_ext1_wakeup(mask, mode);
}
