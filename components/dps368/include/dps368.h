#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// DPS368 I2C addresses
#define DPS368_I2C_ADDR_SDO_GND 0x76  // SDO pin connected to GND
#define DPS368_I2C_ADDR_SDO_VDD 0x77  // SDO pin connected to VDD (default for this board)

// DPS368 Register Map
#define DPS368_REG_PSR_B2       0x00  // Pressure value (24-bit, MSB first)
#define DPS368_REG_PSR_B1       0x01
#define DPS368_REG_PSR_B0       0x02
#define DPS368_REG_TMP_B2       0x03  // Temperature value (24-bit, MSB first)
#define DPS368_REG_TMP_B1       0x04
#define DPS368_REG_TMP_B0       0x05
#define DPS368_REG_PRS_CFG      0x06  // Pressure configuration
#define DPS368_REG_TMP_CFG      0x07  // Temperature configuration
#define DPS368_REG_MEAS_CFG     0x08  // Measurement control
#define DPS368_REG_CFG_REG      0x09  // Configuration register
#define DPS368_REG_INT_STS      0x0A  // Interrupt status
#define DPS368_REG_FIFO_STS     0x0B  // FIFO status
#define DPS368_REG_RESET        0x0C  // Software reset
#define DPS368_REG_ID           0x0D  // Product and revision ID

// Product ID
#define DPS368_PROD_ID          0x10  // Expected product ID

// Measurement configuration bits
#define DPS368_MEAS_CFG_COEF_RDY  (1 << 7)  // Coefficients ready
#define DPS368_MEAS_CFG_SENSOR_RDY (1 << 6)  // Sensor ready
#define DPS368_MEAS_CFG_TMP_RDY   (1 << 5)  // Temperature ready
#define DPS368_MEAS_CFG_PRS_RDY   (1 << 4)  // Pressure ready
#define DPS368_MEAS_CFG_MEAS_CTRL_MASK 0x07 // Measurement control mask

// Measurement modes
#define DPS368_MODE_STANDBY       0x00  // Standby mode
#define DPS368_MODE_PRESSURE_ONCE 0x01  // One-shot pressure measurement
#define DPS368_MODE_TEMP_ONCE     0x02  // One-shot temperature measurement
#define DPS368_MODE_CONTINUOUS    0x07  // Continuous pressure and temperature

/**
 * @brief DPS368 pressure and temperature data
 */
typedef struct {
    float pressure_pa;      // Pressure in Pascals
    float temperature_c;    // Temperature in Celsius
    bool pressure_valid;    // Pressure measurement valid
    bool temp_valid;        // Temperature measurement valid
} dps368_data_t;

/**
 * @brief DPS368 device handle
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t address;
    bool initialized;
    // Calibration coefficients (to be read from device)
    int32_t c0, c1, c00, c10, c01, c11, c20, c21, c30;
} dps368_handle_t;

/**
 * @brief Initialize DPS368 pressure sensor
 * 
 * @param bus I2C master bus handle
 * @param address I2C address (0x76 or 0x77)
 * @param handle Pointer to store device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dps368_init(i2c_master_bus_handle_t bus, uint8_t address, dps368_handle_t **handle);

/**
 * @brief Read pressure and temperature from DPS368
 * 
 * @param handle Device handle
 * @param data Pointer to store measurement data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dps368_read(dps368_handle_t *handle, dps368_data_t *data);

/**
 * @brief Deinitialize DPS368
 * 
 * @param handle Device handle to free
 * @return esp_err_t ESP_OK on success
 */
esp_err_t dps368_deinit(dps368_handle_t *handle);

#ifdef __cplusplus
}
#endif
