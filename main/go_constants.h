#ifndef AIRGRADIENT_GO_MAIN_GO_CONSTANTS_H
#define AIRGRADIENT_GO_MAIN_GO_CONSTANTS_H

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_types.h"

static constexpr char GO_TAG[] = "GO";

static constexpr int GO_BOOT_DELAY_MS = 1000;

static constexpr int GO_MAIN_LOOP_DELAY_MS = 50;
static constexpr int GO_IDLE_MEASURE_INTERVAL_MS = 5000;
static constexpr int GO_IDLE_INACTIVE_TIMEOUT_MS = 30000;

static constexpr uint32_t GO_INPUT_QUEUE_LEN = 16;

// Keep disabled in skeleton to avoid accidental sleep-lock during development.
static constexpr bool GO_ENABLE_DEEP_SLEEP = false;

// I2C master bus (for touch controller).
static constexpr gpio_num_t GO_I2C_MASTER_SCL_IO = GPIO_NUM_6;
static constexpr gpio_num_t GO_I2C_MASTER_SDA_IO = GPIO_NUM_7;
static constexpr auto GO_I2C_MASTER_PORT = I2C_NUM_0;
static constexpr int GO_I2C_GLITCH_IGNORE_CNT = 7;
static constexpr bool GO_I2C_INTERNAL_PULLUPS = true;

// Button service wiring.
static constexpr gpio_num_t GO_BUTTON_PHYSICAL_GPIO = GPIO_NUM_5;
static constexpr bool GO_BUTTON_PHYSICAL_ACTIVE_LOW = true;

static constexpr gpio_num_t GO_TOUCH_ALERT_GPIO = GPIO_NUM_1;
static constexpr bool GO_TOUCH_ALERT_ACTIVE_LOW = true;
static constexpr bool GO_TOUCH_REQUIRED = false;

// Button service behavior.
static constexpr uint32_t GO_BUTTON_DEBOUNCE_MS = 200;
static constexpr uint32_t GO_BUTTON_LONG_PRESS_MS = 2500;

// Choose a single touch channel for tracking toggles (0=CS1, 1=CS2, 2=CS3).
static constexpr uint8_t GO_TRACKING_TOUCH_ID = 0;
static constexpr uint8_t GO_TOUCH_ENABLE_MASK = 0x01;
static constexpr uint8_t GO_TOUCH_INTERRUPT_ENABLE_MASK = 0x01;
static constexpr bool GO_TOUCH_CALIBRATE = true;
static constexpr uint8_t GO_TOUCH_CALIBRATE_MASK = 0x01;

#endif // AIRGRADIENT_GO_MAIN_GO_CONSTANTS_H
