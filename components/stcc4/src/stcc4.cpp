#include "stcc4.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STCC4";

/* ===== CRC Calculation ===== */
/**
 * CRC-8 checksum calculation
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * Initial value: 0xFF
 */
uint8_t stcc4_calculate_crc(uint8_t byte0, uint8_t byte1) {
    uint8_t crc = 0xFF;
    
    // Process first byte
    crc ^= byte0;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x31;
        } else {
            crc = crc << 1;
        }
        crc &= 0xFF;
    }
    
    // Process second byte
    crc ^= byte1;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x31;
        } else {
            crc = crc << 1;
        }
        crc &= 0xFF;
    }
    
    return crc;
}

bool stcc4_verify_crc(uint8_t byte0, uint8_t byte1, uint8_t crc) {
    return stcc4_calculate_crc(byte0, byte1) == crc;
}

/* ===== Conversion Formulas (from Section 3.5) ===== */
static float convert_temperature_output(uint16_t raw_value) {
    return (175.0f * raw_value / 65535.0f) - 45.0f;
}

static float convert_humidity_output(uint16_t raw_value) {
    return (125.0f * raw_value / 65535.0f) - 6.0f;
}

static uint16_t convert_temperature_input(float temperature_c) {
    return (uint16_t)((temperature_c + 45.0f) * 65535.0f / 175.0f);
}

static uint16_t convert_humidity_input(float humidity_rh) {
    return (uint16_t)((humidity_rh + 6.0f) * 65535.0f / 125.0f);
}

static uint16_t convert_pressure_input(uint32_t pressure_pa) {
    // Clip to valid range: 40,000 - 110,000 Pa
    if (pressure_pa < 40000) pressure_pa = 40000;
    if (pressure_pa > 110000) pressure_pa = 110000;
    return (uint16_t)(pressure_pa / 2);
}

/* ===== I2C Communication ===== */

static esp_err_t stcc4_i2c_write_command(stcc4_dev_t *dev, uint16_t cmd) {
    // STCC4 commands are 16-bit only (no CRC on command)
    uint8_t buf[2];
    buf[0] = (cmd >> 8) & 0xFF;
    buf[1] = cmd & 0xFF;
    return i2c_master_transmit(dev->dev_handle, buf, 2, -1);
}

static esp_err_t stcc4_i2c_write_command_with_data(stcc4_dev_t *dev, uint16_t cmd, 
                                                    const uint8_t *data, size_t data_len) {
    uint8_t buf[24];
    int pos = 0;
    
    // Command (no CRC)
    buf[pos++] = (cmd >> 8) & 0xFF;
    buf[pos++] = cmd & 0xFF;
    
    // Data words with CRC per 2-byte word
    for (size_t i = 0; i < data_len; i += 2) {
        buf[pos++] = data[i];
        buf[pos++] = data[i + 1];
        buf[pos++] = stcc4_calculate_crc(data[i], data[i + 1]);
    }
    
    return i2c_master_transmit(dev->dev_handle, buf, pos, -1);
}

static esp_err_t stcc4_i2c_read(stcc4_dev_t *dev, uint8_t *buf, size_t len) {
    return i2c_master_receive(dev->dev_handle, buf, len, -1);
}

static esp_err_t stcc4_read_word(stcc4_dev_t *dev, uint16_t *value) {
    uint8_t buf[3];
    esp_err_t ret = stcc4_i2c_read(dev, buf, 3);
    if (ret != ESP_OK) return ret;
    
    // Verify CRC
    if (!stcc4_verify_crc(buf[0], buf[1], buf[2])) {
        ESP_LOGW(TAG, "CRC error reading word");
        return ESP_ERR_INVALID_CRC;
    }
    
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

/* ===== Public Functions ===== */

esp_err_t stcc4_init(stcc4_dev_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr) {
    if (!dev || !bus_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    dev->bus_handle = bus_handle;
    dev->i2c_addr = i2c_addr;
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = STCC4_I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {.disable_ack_check = false},
    };
    
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    dev->initialized = true;
    ESP_LOGI(TAG, "STCC4 initialized with I2C address 0x%02X", i2c_addr);
    
    return ESP_OK;
}

esp_err_t stcc4_deinit(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = i2c_master_bus_rm_device(dev->dev_handle);
    dev->initialized = false;
    
    return ret;
}

esp_err_t stcc4_get_product_id(stcc4_dev_t *dev, uint32_t *product_id, uint64_t *serial_number) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send get_product_id command
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_GET_PRODUCT_ID);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait 10ms for sensor to prepare data
    
    // Read product ID (4 bytes / 2 words)
    uint16_t product_id_word = 0;
    ret = stcc4_read_word(dev, &product_id_word);
    if (ret != ESP_OK) return ret;
    
    uint8_t buf[9];
    ret = stcc4_i2c_read(dev, buf, 9);  // 4 more words (8 bytes) + CRC
    if (ret != ESP_OK) return ret;
    
    // Verify CRCs for remaining words
    if (!stcc4_verify_crc(buf[0], buf[1], buf[2])) {
        ESP_LOGW(TAG, "CRC error reading product ID (word 2)");
        return ESP_ERR_INVALID_CRC;
    }
    if (!stcc4_verify_crc(buf[3], buf[4], buf[5])) {
        ESP_LOGW(TAG, "CRC error reading product ID (word 3)");
        return ESP_ERR_INVALID_CRC;
    }
    if (!stcc4_verify_crc(buf[6], buf[7], buf[8])) {
        ESP_LOGW(TAG, "CRC error reading product ID (word 4)");
        return ESP_ERR_INVALID_CRC;
    }
    
    if (product_id) {
        *product_id = ((uint32_t)product_id_word << 16) | 
                      (((uint16_t)buf[0] << 8) | buf[1]);
    }
    
    if (serial_number) {
        *serial_number = (((uint64_t)buf[3] << 56) | ((uint64_t)buf[4] << 48) |
                          ((uint64_t)buf[6] << 40) | ((uint64_t)buf[7] << 32) |
                          ((uint64_t)buf[3] << 24) | ((uint64_t)buf[4] << 16));
        // Note: Serial number format depends on actual datasheet layout
    }
    
    ESP_LOGI(TAG, "Product ID: 0x%08X", *product_id);
    
    return ESP_OK;
}

esp_err_t stcc4_start_continuous_measurement(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_START_CONTINUOUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Continuous measurement started (1s sampling interval)");
    return ESP_OK;
}

esp_err_t stcc4_stop_continuous_measurement(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_STOP_CONTINUOUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop continuous measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_STOP_CONTINUOUS));
    
    ESP_LOGI(TAG, "Continuous measurement stopped");
    return ESP_OK;
}

esp_err_t stcc4_read_measurement(stcc4_dev_t *dev, stcc4_measurement_t *measurement) {
    if (!dev || !dev->initialized || !measurement) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send read command
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_READ_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send read command: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_READ_MEASUREMENT));
    
    // Read all measurement data (12 bytes: 4 words × (2 bytes + 1 CRC))
    uint8_t buf[12];
    ret = stcc4_i2c_read(dev, buf, 12);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read measurement data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Verify all CRCs
    if (!stcc4_verify_crc(buf[0], buf[1], buf[2])) {
        ESP_LOGW(TAG, "CRC error in CO2 data");
        return ESP_ERR_INVALID_CRC;
    }
    if (!stcc4_verify_crc(buf[3], buf[4], buf[5])) {
        ESP_LOGW(TAG, "CRC error in temperature data");
        return ESP_ERR_INVALID_CRC;
    }
    if (!stcc4_verify_crc(buf[6], buf[7], buf[8])) {
        ESP_LOGW(TAG, "CRC error in humidity data");
        return ESP_ERR_INVALID_CRC;
    }
    if (!stcc4_verify_crc(buf[9], buf[10], buf[11])) {
        ESP_LOGW(TAG, "CRC error in status data");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Extract raw values
    measurement->co2_ppm = ((uint16_t)buf[0] << 8) | buf[1];
    measurement->temperature_raw = ((uint16_t)buf[3] << 8) | buf[4];
    measurement->humidity_raw = ((uint16_t)buf[6] << 8) | buf[7];
    measurement->sensor_status = ((uint16_t)buf[9] << 8) | buf[10];
    
    // Convert to physical units
    measurement->temperature_c = convert_temperature_output(measurement->temperature_raw);
    measurement->humidity_rh = convert_humidity_output(measurement->humidity_raw);
    
    // Extract testing mode bit (bit 14 of sensor_status)
    measurement->testing_mode = (measurement->sensor_status & 0x4000) != 0;
    
    ESP_LOGD(TAG, "CO2: %u ppm, T: %.2f°C, RH: %.1f%%, Status: 0x%04X", 
             measurement->co2_ppm, measurement->temperature_c, 
             measurement->humidity_rh, measurement->sensor_status);
    
    return ESP_OK;
}

esp_err_t stcc4_measure_single_shot(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_MEASURE_SINGLE_SHOT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start single-shot measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_SINGLE_SHOT));
    
    ESP_LOGD(TAG, "Single-shot measurement completed");
    return ESP_OK;
}

esp_err_t stcc4_set_rht_compensation(stcc4_dev_t *dev, float temperature_c, float humidity_rh) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert to raw values
    uint16_t temp_raw = convert_temperature_input(temperature_c);
    uint16_t hum_raw = convert_humidity_input(humidity_rh);
    
    // Prepare data: T (2 bytes + CRC) + RH (2 bytes + CRC)
    uint8_t data[6];
    data[0] = (temp_raw >> 8) & 0xFF;
    data[1] = temp_raw & 0xFF;
    data[2] = stcc4_calculate_crc(data[0], data[1]);
    data[3] = (hum_raw >> 8) & 0xFF;
    data[4] = hum_raw & 0xFF;
    data[5] = stcc4_calculate_crc(data[3], data[4]);
    
    esp_err_t ret = stcc4_i2c_write_command_with_data(dev, STCC4_CMD_SET_RHT_COMPENSATION, 
                                                       data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set RHT compensation: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "RHT compensation set: T=%.2f°C, RH=%.1f%%", temperature_c, humidity_rh);
    return ESP_OK;
}

esp_err_t stcc4_set_pressure_compensation(stcc4_dev_t *dev, uint16_t pressure_pa) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint16_t press_raw = convert_pressure_input(pressure_pa);
    
    uint8_t data[3];
    data[0] = (press_raw >> 8) & 0xFF;
    data[1] = press_raw & 0xFF;
    data[2] = stcc4_calculate_crc(data[0], data[1]);
    
    esp_err_t ret = stcc4_i2c_write_command_with_data(dev, STCC4_CMD_SET_PRESSURE, data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pressure compensation: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Pressure compensation set: %u Pa", pressure_pa);
    return ESP_OK;
}

esp_err_t stcc4_perform_conditioning(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_PERFORM_CONDITIONING);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform conditioning: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_CONDITIONING));
    
    ESP_LOGI(TAG, "Sensor conditioning completed");
    return ESP_OK;
}

esp_err_t stcc4_enter_sleep_mode(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_ENTER_SLEEP_MODE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter sleep mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_ENTER_SLEEP));
    
    ESP_LOGD(TAG, "Entered sleep mode");
    return ESP_OK;
}

esp_err_t stcc4_exit_sleep_mode(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Datasheet 3.4.8: exit_sleep_mode is device address (write) + payload byte 0x00,
    // and the payload byte is NOT acknowledged. Some I2C drivers report this as an error.
    // Strategy: send the payload, ignore immediate NACK error, wait 5ms, then verify by
    // attempting a benign command (get_product_id). If it succeeds, consider wake successful.
    uint8_t payload = 0x00;
    esp_err_t ret = i2c_master_transmit(dev->dev_handle, &payload, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Exit sleep write returned: %s (expected NACK on payload)", esp_err_to_name(ret));
    }

    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_EXIT_SLEEP));

    uint32_t pid = 0;
    esp_err_t verify = stcc4_get_product_id(dev, &pid, nullptr);
    if (verify == ESP_OK) {
        ESP_LOGD(TAG, "Exited sleep mode");
        return ESP_OK;
    }

    // One more attempt after a short delay
    vTaskDelay(pdMS_TO_TICKS(5));
    ret = i2c_master_transmit(dev->dev_handle, &payload, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Second exit sleep write: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_EXIT_SLEEP));
    verify = stcc4_get_product_id(dev, &pid, nullptr);
    if (verify == ESP_OK) {
        ESP_LOGD(TAG, "Exited sleep mode (2nd attempt)");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to verify wake from sleep");
    return ESP_ERR_INVALID_STATE;
}

esp_err_t stcc4_soft_reset(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Soft reset uses I2C general call (address 0x00)
    // This is typically not supported by the I2C master driver directly
    // For now, we'll log this limitation
    ESP_LOGE(TAG, "Soft reset via I2C general call not yet implemented");
    
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t stcc4_factory_reset(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_FACTORY_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform factory reset: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_FACTORY_RESET));
    
    // Read result word
    uint16_t result;
    ret = stcc4_read_word(dev, &result);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read factory reset result: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (result == 0x0000) {
        ESP_LOGI(TAG, "Factory reset completed successfully");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Factory reset failed: 0x%04X", result);
        return ESP_ERR_INVALID_RESPONSE;
    }
}

esp_err_t stcc4_perform_self_test(stcc4_dev_t *dev, stcc4_self_test_result_t *result) {
    if (!dev || !dev->initialized || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_SELF_TEST);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform self-test: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_SELF_TEST));
    
    uint16_t test_result;
    ret = stcc4_read_word(dev, &test_result);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read self-test result: %s", esp_err_to_name(ret));
        return ret;
    }
    
    result->raw_value = test_result;
    result->supply_voltage_ok = (test_result & 0x0001) == 0;
    result->sht_connected = (test_result & 0x0010) == 0;
    result->memory_ok = (test_result & 0x0060) == 0;
    
    bool success = (test_result == 0x0000 || test_result == 0x0010);
    
    ESP_LOGI(TAG, "Self-test: %s (0x%04X) - Supply: %s, SHT: %s, Memory: %s",
             success ? "PASS" : "FAIL", test_result,
             result->supply_voltage_ok ? "OK" : "ERROR",
             result->sht_connected ? "OK" : "NOT_CONNECTED",
             result->memory_ok ? "OK" : "ERROR");
    
    return success ? ESP_OK : ESP_FAIL;
}

esp_err_t stcc4_enable_testing_mode(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_ENABLE_TESTING_MODE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable testing mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Testing mode enabled");
    return ESP_OK;
}

esp_err_t stcc4_disable_testing_mode(stcc4_dev_t *dev) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = stcc4_i2c_write_command(dev, STCC4_CMD_DISABLE_TESTING_MODE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable testing mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Testing mode disabled");
    return ESP_OK;
}

esp_err_t stcc4_perform_forced_recalibration(stcc4_dev_t *dev, uint16_t target_co2_ppm,
                                              int16_t *correction_ppm) {
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (target_co2_ppm > 32000) {
        ESP_LOGE(TAG, "Target CO2 out of range (max 32000 ppm): %u", target_co2_ppm);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[3];
    data[0] = (target_co2_ppm >> 8) & 0xFF;
    data[1] = target_co2_ppm & 0xFF;
    data[2] = stcc4_calculate_crc(data[0], data[1]);
    
    esp_err_t ret = stcc4_i2c_write_command_with_data(dev, STCC4_CMD_FORCED_RECALIBRATION, 
                                                       data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform FRC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(STCC4_EXEC_FORCED_RECALIBRATION));
    
    uint16_t frc_result;
    ret = stcc4_read_word(dev, &frc_result);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read FRC result: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (frc_result == 0xFFFF) {
        ESP_LOGE(TAG, "FRC command failed");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Convert correction from raw value
    // Correction = output - 32768 (as signed value offset)
    int32_t correction_raw = (int32_t)frc_result - 32768;
    
    if (correction_ppm) {
        *correction_ppm = (int16_t)correction_raw;
    }
    
    ESP_LOGI(TAG, "FRC completed: target=%u ppm, correction=%d ppm", 
             target_co2_ppm, (int16_t)correction_raw);
    
    return ESP_OK;
}
