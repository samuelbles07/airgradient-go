#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Senseair S12 CO2 default I2C address (7-bit)
#define S12_I2C_ADDR_DEFAULT 0x68

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint16_t addr_7bit;
} s12_i2c_t;

esp_err_t s12_i2c_create(i2c_master_bus_handle_t bus,
                         uint16_t addr_7bit,
                         uint32_t scl_speed_hz,
                         s12_i2c_t* out);
void s12_i2c_destroy(s12_i2c_t* s);

// Reads measured CO2 concentration (ppm).
// Uses the "Measured concentration Filtered Pressure Compensated" register (0x06/0x07).
esp_err_t s12_i2c_read_co2_ppm(s12_i2c_t* s, uint16_t* out_co2_ppm);

// Blocking calibration.
// - If target_ppm == 0, defaults to 400 ppm.
// - If measurement_period_ms <= 0, the function will try to read it from the sensor.
// - Uses the S12 background calibration command (0x7C06) and writes the calibration target
//   register (0x84/0x85) beforehand.
esp_err_t s12_i2c_force_calibration(s12_i2c_t* s, uint16_t target_ppm, int measurement_period_ms);

// Blocking background calibration.
// If measurement_period_ms <= 0, the function will try to read it from the sensor.
// Note: This does not set the calibration target register.
esp_err_t s12_i2c_background_calibration(s12_i2c_t* s, int measurement_period_ms);

#ifdef __cplusplus
}
#endif
