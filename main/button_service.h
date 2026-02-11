#ifndef AIRGRADIENT_GO_MAIN_BUTTON_SERVICE_H
#define AIRGRADIENT_GO_MAIN_BUTTON_SERVICE_H

#include <stdint.h>

#include <driver/gpio.h>

#include "esp_event.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "cap1203.h"

// ESP Event base for button events.
ESP_EVENT_DECLARE_BASE(BUTTON_SERVICE_EVENT);

class ButtonService {
public:
  enum class Source : uint8_t {
    Touch = 1,
    Physical = 2,
  };

  enum class Event : int32_t {
    Press = 1,
    Release = 2,
    ShortPress = 3,
    LongPress = 4,
  };

  struct Payload {
    Source source;
    uint8_t id;
    uint8_t touch_mask;
    uint32_t duration_ms;
  };

  struct Config {
    // GPIO for CAP1203 ALERT# interrupt output (active-low by default).
    // Use GPIO_NUM_MAX if not connected (touch can be optional via cap_required=false).
    gpio_num_t cap_alert_gpio = GPIO_NUM_MAX;

    // GPIO for the single physical button input.
    gpio_num_t physical_gpio = GPIO_NUM_MAX;

    // If true, CAP1203 ALERT# is considered asserted when the pin reads low.
    bool cap_alert_active_low = true;

    // If true, the physical button is considered pressed when the pin reads low.
    bool physical_active_low = true;

    // If true, init() fails if CAP1203 init/probe fails. If false, touch is disabled on failure.
    bool cap_required = false;

    // Debounce window (ms) for the physical button edge handling.
    uint32_t debounce_ms = 30;

    // Long press threshold (ms) used for both touch + physical.
    uint32_t long_press_ms = 1000;

    // CAP1203 driver init parameters (I2C address, bus speed, timeout).
    CAP1203Config cap1203;

    // CAP1203 sensor enable mask (bit0=CS1, bit1=CS2, bit2=CS3).
    uint8_t touch_enable_mask = 0x07;

    // CAP1203 interrupt enable mask (bit0=CS1, bit1=CS2, bit2=CS3).
    // This controls which channels can assert ALERT#.
    uint8_t touch_interrupt_enable_mask = 0x07;

    // CAP1203 sensitivity control fields for SENSITIVITY_CONTROL (0x1F):
    // delta_sense: 0..7 (0 = most sensitive, 7 = least sensitive)
    uint8_t touch_delta_sense = 1;

    // base_shift: 0..15 (usually left at 0x0F unless you know you need otherwise)
    uint8_t touch_base_shift = 0x0F;

    // Per-channel touch thresholds written to 0x30..0x32 (0..127).
    // Index 0=CS1, 1=CS2, 2=CS3.
    uint8_t touch_threshold[3] = {64, 64, 64};

    // If true, service triggers a CAP1203 calibration during init().
    bool touch_calibrate = true;

    // Calibration mask (bit0=CS1, bit1=CS2, bit2=CS3).
    uint8_t touch_calibrate_mask = 0x07;
  };

  ButtonService(i2c_master_bus_handle_t i2c_bus, const Config &cfg);
  ~ButtonService();

  ButtonService(const ButtonService &) = delete;
  ButtonService &operator=(const ButtonService &) = delete;

  esp_err_t init();
  esp_err_t deinit();

  // Call right before esp_light_sleep_start().
  // Disables GPIO interrupts, stops long-press timers, and clears any stale CAP1203 interrupt latch.
  esp_err_t pre_light_sleep();

  // Call right after esp_light_sleep_start() returns.
  // Re-enables GPIO interrupts, clears CAP1203 latch, and re-syncs internal pressed state.
  esp_err_t post_light_sleep();

  esp_err_t disable_wakeup_sources();
  esp_err_t enable_light_sleep_wakeup();
  esp_err_t enable_deep_sleep_wakeup();

private:
  struct IsrCtx {
    ButtonService *self;
    Source source;
  };

  struct TimerCtx {
    ButtonService *self;
    Source source;
    uint8_t id;
  };

  struct IsrEvent {
    Source source;
  };

  static void IRAM_ATTR _gpio_isr(void *arg);
  static void _task_thunk(void *arg);
  void _task();

  static void _timer_thunk(void *arg);
  void _on_long_press_timer(Source source, uint8_t id);

  esp_err_t _init_event_loop();
  esp_err_t _init_gpio();
  esp_err_t _init_cap1203();
  esp_err_t _init_timers();
  esp_err_t _install_isr_handlers();
  esp_err_t _rm_isr_handlers();

  void _handle_cap1203_irq();
  void _handle_physical_irq();

  void _emit(Event ev, const Payload &p);

  static uint32_t _now_ms();

  i2c_master_bus_handle_t bus_;
  Config cfg_;
  CAP1203 cap1203_;
  bool cap_ready_;

  QueueHandle_t queue_;
  TaskHandle_t task_;
  bool task_suspended_;

  IsrCtx cap_isr_;
  IsrCtx phy_isr_;

  TimerCtx touch_timer_ctx_[3];
  TimerCtx physical_timer_ctx_;

  // Touch state.
  uint8_t last_touch_mask_;
  uint32_t touch_press_ms_[3];
  bool touch_long_fired_[3];
  esp_timer_handle_t touch_long_timer_[3];

  // Physical button state.
  bool physical_pressed_;
  uint32_t physical_last_change_ms_;
  uint32_t physical_press_ms_;
  bool physical_long_fired_;
  esp_timer_handle_t physical_long_timer_;
};

#endif // AIRGRADIENT_GO_MAIN_BUTTON_SERVICE_H
