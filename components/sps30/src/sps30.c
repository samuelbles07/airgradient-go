/*
 * SPS30 Particulate Matter Sensor Driver
 * I2C Interface Implementation
 */

#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sps30.h>

static const char *TAG = "sps30";

/*
 * SPS30 Internal Device Structure
 */
struct sps30_handle_s {
    i2c_master_dev_handle_t i2c_handle;
    sps30_config_t config;
    bool measuring;
};

/*
 * CRC-8 Calculation for SPS30
 */
static uint8_t sps30_calc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/*
 * Convert IEEE 754 bytes to float
 */
static float bytes_to_float(const uint8_t *bytes)
{
    uint32_t value = ((uint32_t)bytes[0] << 24) |
                     ((uint32_t)bytes[1] << 16) |
                     ((uint32_t)bytes[2] << 8) |
                     ((uint32_t)bytes[3]);
    return *(float *)&value;
}

/*
 * I2C Write Command (command + CRC)
 */
static esp_err_t sps30_i2c_write_command(sps30_handle_t handle, uint16_t cmd)
{
    uint8_t buffer[3];
    buffer[0] = (cmd >> 8) & 0xFF;
    buffer[1] = cmd & 0xFF;
    buffer[2] = sps30_calc_crc8(buffer, 2);

    esp_err_t ret = i2c_master_transmit(handle->i2c_handle, buffer, 3, SPS30_I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write command failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/*
 * I2C Read Data with CRC Verification
 */
static esp_err_t sps30_i2c_read(sps30_handle_t handle, uint8_t *buffer, uint16_t len)
{
    esp_err_t ret = i2c_master_receive(handle->i2c_handle, buffer, len, SPS30_I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify CRC for every 3-byte word (data[0:1] + crc[2])
    for (uint16_t i = 0; i < len; i += 3) {
        if (i + 2 < len) {
            uint8_t expected_crc = sps30_calc_crc8(&buffer[i], 2);
            if (buffer[i + 2] != expected_crc) {
                ESP_LOGE(TAG, "CRC mismatch at offset %d: got 0x%02x, expected 0x%02x", 
                         i, buffer[i + 2], expected_crc);
                return ESP_ERR_INVALID_CRC;
            }
        }
    }

    return ESP_OK;
}

/*
 * Initialize SPS30
 */
esp_err_t sps30_init(i2c_master_bus_handle_t master_handle,
                     const sps30_config_t *config,
                     sps30_handle_t *sps30_handle)
{
    if (!master_handle || !config || !sps30_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing SPS30 at address 0x%02x (skipping probe, device detected by scan)...", 
             config->i2c_address);

    // Allocate handle
    sps30_handle_t handle = (sps30_handle_t)calloc(1, sizeof(struct sps30_handle_s));
    if (!handle) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    // Copy configuration
    handle->config = *config;

    // Add device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_address,
        .scl_speed_hz = config->i2c_clock_speed,
    };

    esp_err_t ret = i2c_master_bus_add_device(master_handle, &dev_cfg, &handle->i2c_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        free(handle);
        return ret;
    }
    ESP_LOGI(TAG, "Device added to I2C bus successfully");

    // Power-up delay
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reset sensor
    ret = sps30_reset(handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Reset failed, continuing anyway");
    }

    // Wait after reset
    vTaskDelay(pdMS_TO_TICKS(100));

    handle->measuring = false;
    *sps30_handle = handle;

    ESP_LOGI(TAG, "SPS30 initialized at address 0x%02x", config->i2c_address);
    return ESP_OK;
}

/*
 * Start Measurement
 */
esp_err_t sps30_start_measurement(sps30_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Sending start measurement command (0x0010) with IEEE754 float format...");
    
    // I2C "Set Pointer & Write Data": [Pointer_MSB][Pointer_LSB][Data0][Data1][CRC]
    // NO CRC after pointer! Data packet = [format][dummy] + CRC
    uint8_t buffer[5];
    buffer[0] = (SPS30_CMD_START_MEASUREMENT >> 8) & 0xFF;  // Pointer MSB
    buffer[1] = SPS30_CMD_START_MEASUREMENT & 0xFF;          // Pointer LSB
    buffer[2] = 0x03;  // Data0: Format = IEEE754 float
    buffer[3] = 0x00;  // Data1: Dummy byte
    buffer[4] = sps30_calc_crc8(&buffer[2], 2);              // CRC for data packet [2:3]
    
    ESP_LOGI(TAG, "TX: %02X %02X %02X %02X %02X", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    
    esp_err_t ret = i2c_master_transmit(handle->i2c_handle, buffer, 5, SPS30_I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start measurement: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    handle->measuring = true;
    ESP_LOGI(TAG, "Measurement started - fan should be spinning now (45-65mA draw)");
    return ESP_OK;
}

/*
 * Stop Measurement
 */
esp_err_t sps30_stop_measurement(sps30_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = sps30_i2c_write_command(handle, SPS30_CMD_STOP_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop measurement");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    handle->measuring = false;
    ESP_LOGI(TAG, "Measurement stopped");
    return ESP_OK;
}

/*
 * Helper to extract float from I2C 6-byte format: [MSB][LSB][CRC][MSB][LSB][CRC]
 */
static float extract_float_i2c(const uint8_t *data)
{
    uint8_t bytes[4];
    bytes[0] = data[0];  // MSB word 1
    bytes[1] = data[1];  // LSB word 1
    bytes[2] = data[3];  // MSB word 2 (skip CRC at [2])
    bytes[3] = data[4];  // LSB word 2
    return bytes_to_float(bytes);
}

/*
 * Read Measurement
 * I2C format: Every 2 bytes followed by CRC = 3 bytes per word
 * 1 Float (4 bytes) = 2 words = 6 bytes ([MSB][LSB][CRC][MSB][LSB][CRC])
 * 10 Floats = 60 bytes total
 */
esp_err_t sps30_read_measurement(sps30_handle_t handle, sps30_measurement_t *measurement)
{
    if (!handle || !measurement) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->measuring) {
        ESP_LOGW(TAG, "Sensor not in measurement mode");
        return ESP_ERR_INVALID_STATE;
    }

    // Write read measurement command
    esp_err_t ret = sps30_i2c_write_command(handle, SPS30_CMD_READ_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read command");
        return ret;
    }

    // Delay for sensor to prepare data
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read 60 bytes: 10 floats × 6 bytes (4 data + 2 CRC)
    uint8_t buffer[60];
    ret = sps30_i2c_read(handle, buffer, 60);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }

    // Extract values (each float = 6 bytes in I2C format)
    measurement->pm1p0_mass    = extract_float_i2c(&buffer[0]);
    measurement->pm2p5_mass    = extract_float_i2c(&buffer[6]);
    measurement->pm4p0_mass    = extract_float_i2c(&buffer[12]);
    measurement->pm10p0_mass   = extract_float_i2c(&buffer[18]);
    measurement->pm0p5_number  = extract_float_i2c(&buffer[24]);
    measurement->pm1p0_number  = extract_float_i2c(&buffer[30]);
    measurement->pm2p5_number  = extract_float_i2c(&buffer[36]);
    measurement->pm4p0_number  = extract_float_i2c(&buffer[42]);
    measurement->pm10p0_number = extract_float_i2c(&buffer[48]);
    measurement->typical_size  = extract_float_i2c(&buffer[54]);

    ESP_LOGD(TAG, "PM2.5: %.2f µg/m³, PM10: %.2f µg/m³", 
             measurement->pm2p5_mass, measurement->pm10p0_mass);

    return ESP_OK;
}

/*
 * Reset
 */
esp_err_t sps30_reset(sps30_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = sps30_i2c_write_command(handle, SPS30_CMD_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reset command failed");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

/*
 * Read Device Status Register (0xD206)
 */
esp_err_t sps30_read_status_register(sps30_handle_t handle, uint32_t *status)
{
    if (!handle || !status) {
        return ESP_ERR_INVALID_ARG;
    }

    // Write command to read status
    esp_err_t ret = sps30_i2c_write_command(handle, SPS30_CMD_GET_STATUS_REGISTER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write status read command");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    // Read 6 bytes: [MSB0][LSB0][CRC0][MSB1][LSB1][CRC1]
    uint8_t buffer[6];
    ret = sps30_i2c_read(handle, buffer, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // Combine into 32-bit status
    *status = ((uint32_t)buffer[0] << 24) | 
              ((uint32_t)buffer[1] << 16) |
              ((uint32_t)buffer[3] << 8) | 
              buffer[4];

    ESP_LOGI(TAG, "Status Register: 0x%08X", *status);
    
    // Decode important bits
    if (*status & (1 << 4)) ESP_LOGW(TAG, "  Fan failure detected!");
    if (*status & (1 << 21)) ESP_LOGW(TAG, "  Fan speed out of range!");
    if (*status & (1 << 5)) ESP_LOGW(TAG, "  Laser failure detected!");
    
    return ESP_OK;
}

/*
 * Read Data-Ready Flag (0x0202)
 */
esp_err_t sps30_read_data_ready(sps30_handle_t handle, bool *ready)
{
    if (!handle || !ready) {
        return ESP_ERR_INVALID_ARG;
    }

    // Write command
    esp_err_t ret = sps30_i2c_write_command(handle, SPS30_CMD_READ_DATA_READY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data-ready command");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    // Read 3 bytes: [MSB][LSB][CRC]
    uint8_t buffer[3];
    ret = sps30_i2c_read(handle, buffer, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t flag = ((uint16_t)buffer[0] << 8) | buffer[1];
    *ready = (flag == 0x0001);
    
    ESP_LOGI(TAG, "Data-ready flag: 0x%04X (%s)", flag, *ready ? "READY" : "NOT READY");
    return ESP_OK;
}

/*
 * Enter sleep mode (CMD 0x1001) - power consumption <50 μA
 */
esp_err_t sps30_sleep(sps30_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Command: 0x1001 (no data)
    uint8_t cmd_buf[2] = {0x10, 0x01};
    
    esp_err_t ret = i2c_master_transmit(handle->i2c_handle, cmd_buf, 2, SPS30_I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send sleep command");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "SPS30 entered sleep mode");
    return ESP_OK;
}

/*
 * Wake up from sleep mode (CMD 0x1103)
 * Special sequence: send wake-up, wait 100ms
 */
esp_err_t sps30_wakeup(sps30_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Command: 0x1103 (no data)
    uint8_t cmd_buf[2] = {0x11, 0x03};
    
    esp_err_t ret = i2c_master_transmit(handle->i2c_handle, cmd_buf, 2, SPS30_I2C_XFR_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send wake-up command");
        return ret;
    }

    // Wake-up requires 100ms before next command per datasheet
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "SPS30 woke up from sleep");
    return ESP_OK;
}

/*
 * Delete and Clean Up
 */
esp_err_t sps30_delete(sps30_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->measuring) {
        sps30_stop_measurement(handle);
    }

    if (handle->i2c_handle) {
        i2c_master_bus_rm_device(handle->i2c_handle);
    }

    free(handle);
    ESP_LOGI(TAG, "SPS30 deleted");
    return ESP_OK;
}
