#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Senseair Sunrise default I2C address (7-bit)
#define SUNRISE_I2C_ADDR_DEFAULT 0x68

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint16_t addr_7bit;
} sunrise_i2c_t;

esp_err_t sunrise_i2c_create(i2c_master_bus_handle_t bus,
                             uint16_t addr_7bit,
                             uint32_t scl_speed_hz,
                             sunrise_i2c_t* out);
void sunrise_i2c_destroy(sunrise_i2c_t* s);

// Reads measurement mode + period. out_period_ms falls back to 4000ms on invalid values.
esp_err_t sunrise_i2c_read_config(sunrise_i2c_t* s, uint8_t* out_measurement_mode, int* out_period_ms);

// Reads CO2 (ppm) and error status.
esp_err_t sunrise_i2c_read_co2_ppm(sunrise_i2c_t* s, uint16_t* out_co2_ppm, uint8_t* out_error_status);

// Blocking calibration.
// - If target_ppm == 0, defaults to 400 ppm.
// - If measurement_period_ms <= 0, the function will try to read it from the sensor.
// - Uses the Sunrise background calibration command (0x7C06) and writes the calibration target
//   register (0x84/0x85) beforehand.
esp_err_t sunrise_i2c_force_calibration(sunrise_i2c_t* s, uint16_t target_ppm, int measurement_period_ms);

// Blocking background calibration.
// If measurement_period_ms <= 0, the function will try to read it from the sensor.
// Note: This does not set the calibration target register.
esp_err_t sunrise_i2c_background_calibration(sunrise_i2c_t* s, int measurement_period_ms);

#ifdef __cplusplus
}
#endif
