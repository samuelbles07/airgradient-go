#ifndef STCC4_H
#define STCC4_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * STCC4 CO2 Sensor Library
 * Based on STCC4 datasheet v1 (June 2025)
 * 
 * I2C Address: 0x64 (default) or 0x65 (configurable via ADDR pin)
 * 
 * This library provides a complete interface to the Sensirion STCC4
 * thermal conductivity-based CO2 sensor with I2C communication.
 */

/* ===== Constants ===== */
#define STCC4_I2C_ADDR_DEFAULT  0x64
#define STCC4_I2C_ADDR_ALT      0x65

#define STCC4_PRODUCT_ID        0x0901018A

#define STCC4_I2C_FREQ_HZ       100000  // Use 100kHz for robustness

/* ===== I2C Command Codes ===== */
#define STCC4_CMD_START_CONTINUOUS      0x218B
#define STCC4_CMD_STOP_CONTINUOUS       0x3F86
#define STCC4_CMD_READ_MEASUREMENT      0xEC05
#define STCC4_CMD_SET_RHT_COMPENSATION  0xE000
#define STCC4_CMD_SET_PRESSURE          0xE016
#define STCC4_CMD_MEASURE_SINGLE_SHOT   0x219D
#define STCC4_CMD_ENTER_SLEEP_MODE      0x3650
#define STCC4_CMD_EXIT_SLEEP_MODE       0x00
#define STCC4_CMD_PERFORM_CONDITIONING  0x29BC
#define STCC4_CMD_SOFT_RESET            0x06
#define STCC4_CMD_FACTORY_RESET         0x3632
#define STCC4_CMD_SELF_TEST             0x278C
#define STCC4_CMD_ENABLE_TESTING_MODE   0x3FBC
#define STCC4_CMD_DISABLE_TESTING_MODE  0x3F3D
#define STCC4_CMD_FORCED_RECALIBRATION  0x362F
#define STCC4_CMD_GET_PRODUCT_ID        0x365B

/* ===== Execution Times (milliseconds) ===== */
#define STCC4_EXEC_READ_MEASUREMENT     1
#define STCC4_EXEC_STOP_CONTINUOUS      1000
#define STCC4_EXEC_SINGLE_SHOT          500
#define STCC4_EXEC_ENTER_SLEEP          1
#define STCC4_EXEC_EXIT_SLEEP           5
#define STCC4_EXEC_CONDITIONING         22
#define STCC4_EXEC_SOFT_RESET           10
#define STCC4_EXEC_FACTORY_RESET        90
#define STCC4_EXEC_SELF_TEST            360
#define STCC4_EXEC_FORCED_RECALIBRATION 90

/* ===== Measurement Data ===== */
typedef struct {
    uint16_t co2_ppm;           // CO2 concentration in ppm (0-32000)
    uint16_t temperature_raw;   // Raw temperature value
    uint16_t humidity_raw;      // Raw relative humidity value
    uint16_t sensor_status;     // Sensor status flags
    
    // Converted values
    float temperature_c;        // Temperature in Celsius
    float humidity_rh;          // Relative humidity in %RH
    bool testing_mode;          // Sensor is in testing mode
} stcc4_measurement_t;

/* ===== Self-Test Result ===== */
typedef struct {
    bool supply_voltage_ok;
    bool debug_bits;
    bool sht_connected;
    bool memory_ok;
    uint16_t raw_value;
} stcc4_self_test_result_t;

/* ===== Device Handle ===== */
typedef struct stcc4_dev_s {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint8_t i2c_addr;
    bool initialized;
} stcc4_dev_t;

/* ===== Function Prototypes ===== */

/**
 * @brief Initialize STCC4 sensor on existing I2C bus
 * 
 * @param[out] dev Device handle
 * @param bus_handle I2C bus handle (from i2c_new_master_bus)
 * @param i2c_addr I2C address (0x64 or 0x65)
 * @return ESP_OK on success
 */
esp_err_t stcc4_init(stcc4_dev_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);

/**
 * @brief Deinitialize STCC4 sensor
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_deinit(stcc4_dev_t *dev);

/**
 * @brief Get product ID and verify sensor communication
 * 
 * @param dev Device handle
 * @param[out] product_id Product ID
 * @param[out] serial_number 64-bit serial number
 * @return ESP_OK on success
 */
esp_err_t stcc4_get_product_id(stcc4_dev_t *dev, uint32_t *product_id, uint64_t *serial_number);

/**
 * @brief Start continuous CO2 measurement
 * 
 * Starts continuous measurement with 1s sampling interval.
 * Read measurements using stcc4_read_measurement().
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_start_continuous_measurement(stcc4_dev_t *dev);

/**
 * @brief Stop continuous measurement
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_stop_continuous_measurement(stcc4_dev_t *dev);

/**
 * @brief Read measurement data
 * 
 * Reads CO2, temperature, humidity and status from sensor.
 * 
 * @param dev Device handle
 * @param[out] measurement Measurement data structure
 * @return ESP_OK on success, ESP_ERR_INVALID_RESPONSE if no data available
 */
esp_err_t stcc4_read_measurement(stcc4_dev_t *dev, stcc4_measurement_t *measurement);

/**
 * @brief Perform single-shot measurement
 * 
 * Executes on-demand measurement. Recommended sampling interval: 5-600s.
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_measure_single_shot(stcc4_dev_t *dev);

/**
 * @brief Set temperature and humidity compensation
 * 
 * Used when SHT4x sensor is NOT controlled by STCC4's internal interface.
 * Default values: 25°C, 50%RH. Power cycle resets to defaults.
 * 
 * @param dev Device handle
 * @param temperature_c Temperature in Celsius (-45 to +130°C)
 * @param humidity_rh Relative humidity in %RH (0 to 100)
 * @return ESP_OK on success
 */
esp_err_t stcc4_set_rht_compensation(stcc4_dev_t *dev, float temperature_c, float humidity_rh);

/**
 * @brief Set pressure compensation
 * 
 * Updates pressure value for compensation. Default: 101,300 Pa.
 * Accepted range: 40,000 - 110,000 Pa (values outside are clipped).
 * 
 * @param dev Device handle
 * @param pressure_pa Pressure in Pascals
 * @return ESP_OK on success
 */
esp_err_t stcc4_set_pressure_compensation(stcc4_dev_t *dev, uint16_t pressure_pa);

/**
 * @brief Perform sensor conditioning
 * 
 * Recommended after >3 hours idle period to improve CO2 measurement accuracy.
 * Executes in ~22ms. Follow with start_continuous or single_shot measurement.
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_perform_conditioning(stcc4_dev_t *dev);

/**
 * @brief Enter sleep mode (low power)
 * 
 * Reduces current consumption to ~1 μA. Retains RHT/pressure compensation
 * and ASC state. Wake with exit_sleep_mode().
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_enter_sleep_mode(stcc4_dev_t *dev);

/**
 * @brief Exit sleep mode
 * 
 * Wakes sensor from sleep to idle mode.
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_exit_sleep_mode(stcc4_dev_t *dev);

/**
 * @brief Soft reset
 * 
 * Triggers I2C general call reset. Resets sensor to post-power-on state.
 * Not acknowledged by sensor.
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_soft_reset(stcc4_dev_t *dev);

/**
 * @brief Factory reset
 * 
 * Resets FRC and ASC algorithm history. Re-enables bypass phase.
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_factory_reset(stcc4_dev_t *dev);

/**
 * @brief Perform self-test
 * 
 * Checks sensor functionality. Should run during stable conditions.
 * Successful test returns 0x0000 or 0x0010.
 * 
 * @param dev Device handle
 * @param[out] result Self-test result
 * @return ESP_OK on success
 */
esp_err_t stcc4_perform_self_test(stcc4_dev_t *dev, stcc4_self_test_result_t *result);

/**
 * @brief Enable testing mode
 * 
 * Pauses ASC algorithm and disables ASC state updates.
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_enable_testing_mode(stcc4_dev_t *dev);

/**
 * @brief Disable testing mode
 * 
 * Exits testing mode and resumes ASC algorithm.
 * 
 * @param dev Device handle
 * @return ESP_OK on success
 */
esp_err_t stcc4_disable_testing_mode(stcc4_dev_t *dev);

/**
 * @brief Forced recalibration (FRC)
 * 
 * Corrects CO2 output with externally provided target concentration.
 * Ensure sensor and environment are stable.
 * 
 * Recommended operation:
 * 1. Operate sensor ≥30s (continuous) or ≥30 measurements (single-shot)
 * 2. If continuous, stop_continuous_measurement()
 * 3. Call this function with target CO2 concentration
 * 
 * @param dev Device handle
 * @param target_co2_ppm Target CO2 concentration (0-32,000 ppm)
 * @param[out] correction_ppm Applied FRC correction (optional, can be NULL)
 * @return ESP_OK on success, ESP_ERR_INVALID_RESPONSE if command failed
 */
esp_err_t stcc4_perform_forced_recalibration(stcc4_dev_t *dev, uint16_t target_co2_ppm, 
                                              int16_t *correction_ppm);

/* ===== Utility Functions ===== */

/**
 * @brief Calculate CRC-8 checksum for I2C data
 * 
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * Initial value: 0xFF
 * 
 * @param data Two bytes of data
 * @return CRC-8 checksum
 */
uint8_t stcc4_calculate_crc(uint8_t byte0, uint8_t byte1);

/**
 * @brief Verify CRC-8 checksum
 * 
 * @param data Two bytes of data
 * @param crc Expected CRC value
 * @return true if CRC is valid
 */
bool stcc4_verify_crc(uint8_t byte0, uint8_t byte1, uint8_t crc);

#ifdef __cplusplus
}
#endif

#endif // STCC4_H
