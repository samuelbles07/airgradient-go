#ifndef AIRGRADIENT_GO_MAIN_GO_CONSTANTS_H
#define AIRGRADIENT_GO_MAIN_GO_CONSTANTS_H

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

static constexpr char GO_TAG[] = "GO";

#define CONSOLE_MAX_CMDLINE_ARGS 8
#define CONSOLE_MAX_CMDLINE_LENGTH 256
#define CONSOLE_PROMPT_MAX_LEN (32)

static constexpr int GO_BOOT_DELAY_MS = 1000;

static constexpr int GO_MAIN_LOOP_DELAY_MS = 50;
static constexpr int GO_IDLE_MEASURE_INTERVAL_MS = 5000;
static constexpr int GO_IDLE_INACTIVE_TIMEOUT_MS = 60000;

static constexpr uint32_t GO_INPUT_QUEUE_LEN = 16;

// Tracking sleep interval.
static constexpr uint32_t GO_TRACKING_SLEEP_INTERVAL_S = 3 * 60;

// GPS (UART).
static constexpr uart_port_t GO_GPS_UART_PORT = UART_NUM_1;
static constexpr gpio_num_t GO_GPS_UART_TX_GPIO = GPIO_NUM_11;
static constexpr gpio_num_t GO_GPS_UART_RX_GPIO = GPIO_NUM_12;
static constexpr int GO_GPS_UART_BAUD = 9600;
static constexpr bool GO_GPS_LOG_RAW_NMEA = false;

// External watchdog.
static constexpr gpio_num_t GO_WDT_GPIO = GPIO_NUM_2;
static constexpr uint32_t GO_WDT_RESET_PULSE_MS = 20;
static constexpr uint32_t GO_WDT_RESET_INTERVAL_MS = 60 * 1000;

// EPD display (SPI + SSD1680x panel).
static constexpr spi_host_device_t GO_SPI_HOST = SPI2_HOST;
static constexpr gpio_num_t GO_SPI_MOSI_GPIO = GPIO_NUM_25;
static constexpr gpio_num_t GO_SPI_MISO_GPIO = GPIO_NUM_24;
static constexpr gpio_num_t GO_SPI_SCLK_GPIO = GPIO_NUM_23;
static constexpr int GO_SPI_MAX_TRANSFER_SZ = 4096;

static constexpr gpio_num_t GO_EPD_BUSY_GPIO = GPIO_NUM_10;
static constexpr gpio_num_t GO_EPD_RST_GPIO = GPIO_NUM_9;
static constexpr gpio_num_t GO_EPD_DC_GPIO = GPIO_NUM_15;
static constexpr gpio_num_t GO_EPD_CS_GPIO = GPIO_NUM_0;

static constexpr int GO_EPD_CLOCK_SPEED_HZ = 4 * 1000 * 1000;
static constexpr int GO_EPD_SPI_MODE = 0;
static constexpr int GO_EPD_SPI_QUEUE_SIZE = 1;
static constexpr uint32_t GO_EPD_SPI_DEVICE_FLAGS = SPI_DEVICE_HALFDUPLEX;

// SPS30 (PM sensor).
static constexpr gpio_num_t GO_PM_POWER_GPIO = GPIO_NUM_26;
static constexpr int GO_PM_POWER_ON_LEVEL = 1;

static constexpr uint16_t GO_SPS30_I2C_ADDRESS = 0x69;
static constexpr uint32_t GO_SPS30_I2C_CLOCK_SPEED_HZ = 100000;
static constexpr uint32_t GO_SPS30_POWER_STABILIZE_DELAY_MS = 100;
static constexpr uint32_t GO_SPS30_WARMUP_DELAY_MS = 3000;

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
