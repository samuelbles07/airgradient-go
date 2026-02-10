#ifndef __SPS30_H__
#define __SPS30_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SPS30 I2C Address and Configuration
 */
#define SPS30_I2C_ADDR              0x69  // I2C address (SEL pin low)
#define SPS30_I2C_DEV_CLK_SPD       UINT32_C(100000)  // 100 kHz
#define SPS30_I2C_XFR_TIMEOUT_MS    500

/*
 * SPS30 Commands
 */
#define SPS30_CMD_START_MEASUREMENT         0x0010
#define SPS30_CMD_STOP_MEASUREMENT          0x0104
#define SPS30_CMD_READ_DATA_READY           0x0202
#define SPS30_CMD_READ_MEASUREMENT          0x0300
#define SPS30_CMD_GET_DEVICE_INFO           0xd002
#define SPS30_CMD_RESET                     0xd304
#define SPS30_CMD_GET_STATUS_REGISTER       0xd206

/*
 * SPS30 Measurement Data Structure
 */
typedef struct {
    float pm1p0_mass;      // PM1.0 mass concentration [µg/m³]
    float pm2p5_mass;      // PM2.5 mass concentration [µg/m³]
    float pm4p0_mass;      // PM4.0 mass concentration [µg/m³]
    float pm10p0_mass;     // PM10 mass concentration [µg/m³]
    float pm0p5_number;    // PM0.5 number concentration [#/cm³]
    float pm1p0_number;    // PM1.0 number concentration [#/cm³]
    float pm2p5_number;    // PM2.5 number concentration [#/cm³]
    float pm4p0_number;    // PM4.0 number concentration [#/cm³]
    float pm10p0_number;   // PM10 number concentration [#/cm³]
    float typical_size;    // Typical particle size [µm]
} sps30_measurement_t;

/*
 * SPS30 Device Configuration
 */
typedef struct {
    uint16_t i2c_address;
    uint32_t i2c_clock_speed;
} sps30_config_t;

/*
 * SPS30 Device Handle (opaque)
 */
typedef struct sps30_handle_s *sps30_handle_t;

/*
 * @brief Initialize SPS30 sensor on I2C master bus
 *
 * @param[in] master_handle I2C master bus handle
 * @param[in] config SPS30 device configuration
 * @param[out] sps30_handle SPS30 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_init(i2c_master_bus_handle_t master_handle, 
                     const sps30_config_t *config,
                     sps30_handle_t *sps30_handle);

/*
 * @brief Start SPS30 measurement mode
 *
 * @param[in] handle SPS30 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_start_measurement(sps30_handle_t handle);

/*
 * @brief Stop SPS30 measurement mode
 *
 * @param[in] handle SPS30 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_stop_measurement(sps30_handle_t handle);

/*
 * @brief Read measurement data from SPS30
 *
 * @param[in] handle SPS30 device handle
 * @param[out] measurement Measurement data structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_read_measurement(sps30_handle_t handle, 
                                 sps30_measurement_t *measurement);

/*
 * @brief Soft-reset SPS30 sensor
 *
 * @param[in] handle SPS30 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_reset(sps30_handle_t handle);

/*
 * @brief Read device status register
 *
 * @param[in] handle SPS30 device handle
 * @param[out] status 32-bit status register value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_read_status_register(sps30_handle_t handle, uint32_t *status);

/*
 * @brief Check if data is ready
 *
 * @param[in] handle SPS30 device handle
 * @param[out] ready True if data is ready
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_read_data_ready(sps30_handle_t handle, bool *ready);

/*
 * @brief Enter sleep mode (power <50 μA)
 *
 * @param[in] handle SPS30 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_sleep(sps30_handle_t handle);

/*
 * @brief Wake up from sleep mode
 *
 * @param[in] handle SPS30 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_wakeup(sps30_handle_t handle);

/*
 * @brief Remove and delete SPS30 device handle
 *
 * @param[in] handle SPS30 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sps30_delete(sps30_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // __SPS30_H__
