#include "dps368.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "DPS368";

// Helper function to read a register
static esp_err_t dps368_read_reg(dps368_handle_t *handle, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, data, len, 1000);
}

// Helper function to write a register
static esp_err_t dps368_write_reg(dps368_handle_t *handle, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(handle->i2c_dev, buf, 2, 1000);
}

// Read calibration coefficients
static esp_err_t dps368_read_calibration(dps368_handle_t *handle) {
    uint8_t coef[18];
    esp_err_t ret;
    
    // Read coefficient registers (0x10-0x21)
    ret = dps368_read_reg(handle, 0x10, coef, 18);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration coefficients");
        return ret;
    }
    
    // Parse all coefficients per datasheet Section 8.11 (Table 18)
    // c0: 12-bit 2's complement (addr 0x10-0x11)
    handle->c0 = ((int32_t)coef[0] << 4) | ((coef[1] >> 4) & 0x0F);
    if (handle->c0 & 0x800) handle->c0 -= 0x1000; // Sign extend 12-bit
    
    // c1: 12-bit 2's complement (addr 0x11-0x12)
    handle->c1 = (((int32_t)(coef[1] & 0x0F) << 8) | coef[2]);
    if (handle->c1 & 0x800) handle->c1 -= 0x1000;
    
    // c00: 20-bit 2's complement (addr 0x13-0x15)
    handle->c00 = ((int32_t)coef[3] << 12) | ((int32_t)coef[4] << 4) | ((coef[5] >> 4) & 0x0F);
    if (handle->c00 & 0x80000) handle->c00 -= 0x100000;
    
    // c10: 20-bit 2's complement (addr 0x15-0x17)
    handle->c10 = (((int32_t)(coef[5] & 0x0F) << 16) | ((int32_t)coef[6] << 8) | coef[7]);
    if (handle->c10 & 0x80000) handle->c10 -= 0x100000;
    
    // c01: 16-bit 2's complement (addr 0x18-0x19)
    handle->c01 = ((int32_t)coef[8] << 8) | coef[9];
    if (handle->c01 & 0x8000) handle->c01 -= 0x10000;
    
    // c11: 16-bit 2's complement (addr 0x1A-0x1B)
    handle->c11 = ((int32_t)coef[10] << 8) | coef[11];
    if (handle->c11 & 0x8000) handle->c11 -= 0x10000;
    
    // c20: 16-bit 2's complement (addr 0x1C-0x1D)
    handle->c20 = ((int32_t)coef[12] << 8) | coef[13];
    if (handle->c20 & 0x8000) handle->c20 -= 0x10000;
    
    // c21: 16-bit 2's complement (addr 0x1E-0x1F)
    handle->c21 = ((int32_t)coef[14] << 8) | coef[15];
    if (handle->c21 & 0x8000) handle->c21 -= 0x10000;
    
    // c30: 16-bit 2's complement (addr 0x20-0x21)
    handle->c30 = ((int32_t)coef[16] << 8) | coef[17];
    if (handle->c30 & 0x8000) handle->c30 -= 0x10000;
    
    ESP_LOGI(TAG, "Calibration coefficients:");
    ESP_LOGI(TAG, "  c0=%ld, c1=%ld", handle->c0, handle->c1);
    ESP_LOGI(TAG, "  c00=%ld, c10=%ld, c01=%ld", handle->c00, handle->c10, handle->c01);
    ESP_LOGI(TAG, "  c11=%ld, c20=%ld, c21=%ld, c30=%ld", 
             handle->c11, handle->c20, handle->c21, handle->c30);
    
    return ESP_OK;
}

esp_err_t dps368_init(i2c_master_bus_handle_t bus, uint8_t address, dps368_handle_t **out_handle) {
    if (!bus || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    dps368_handle_t *handle = (dps368_handle_t *)calloc(1, sizeof(dps368_handle_t));
    if (!handle) {
        return ESP_ERR_NO_MEM;
    }
    
    handle->address = address;
    
    // Add I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000, // 100 kHz
    };
    
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &handle->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s", address, esp_err_to_name(ret));
        free(handle);
        return ret;
    }
    
    // Read product ID
    uint8_t prod_id;
    ret = dps368_read_reg(handle, DPS368_REG_ID, &prod_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read product ID");
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
        return ret;
    }
    
    ESP_LOGI(TAG, "DPS368 Product ID: 0x%02X (expected 0x%02X)", prod_id, DPS368_PROD_ID);
    
    // Soft reset
    ret = dps368_write_reg(handle, DPS368_REG_RESET, 0x89);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset sensor");
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for reset
    
    // Wait for coefficients to be ready
    uint8_t meas_cfg;
    for (int i = 0; i < 10; i++) {
        ret = dps368_read_reg(handle, DPS368_REG_MEAS_CFG, &meas_cfg, 1);
        if (ret == ESP_OK && (meas_cfg & DPS368_MEAS_CFG_COEF_RDY)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (!(meas_cfg & DPS368_MEAS_CFG_COEF_RDY)) {
        ESP_LOGW(TAG, "Coefficients not ready, sensor may not work properly");
    }
    
    // Read calibration coefficients
    ret = dps368_read_calibration(handle);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
        return ret;
    }
    
    // Configure pressure measurement: 1 sample/sec, 16x oversampling (PM_RATE=000, PM_PRC=0100)
    ret = dps368_write_reg(handle, DPS368_REG_PRS_CFG, 0x04);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pressure measurement");
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
        return ret;
    }
    
    // Configure temperature measurement: 1 sample/sec, 16x oversampling, external sensor (MEMS)
    // TMP_EXT=1, TMP_RATE=000, TMP_PRC=0100
    ret = dps368_write_reg(handle, DPS368_REG_TMP_CFG, 0x80 | 0x04);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure temperature measurement");
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
        return ret;
    }
    
    // Enable P_SHIFT and T_SHIFT for 16x oversampling (per datasheet Table 9)
    // P_SHIFT=1 (bit 2), T_SHIFT=1 (bit 3)
    ret = dps368_write_reg(handle, DPS368_REG_CFG_REG, 0x0C);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable result bit shift");
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
        return ret;
    }
    
    // Set to continuous mode
    ret = dps368_write_reg(handle, DPS368_REG_MEAS_CFG, DPS368_MODE_CONTINUOUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous measurement");
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
        return ret;
    }
    
    handle->initialized = true;
    *out_handle = handle;
    
    ESP_LOGI(TAG, "DPS368 initialized at address 0x%02X", address);
    return ESP_OK;
}

esp_err_t dps368_read(dps368_handle_t *handle, dps368_data_t *data) {
    if (!handle || !handle->initialized || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    uint8_t meas_cfg;
    
    // Check if measurements are ready
    ret = dps368_read_reg(handle, DPS368_REG_MEAS_CFG, &meas_cfg, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    data->pressure_valid = (meas_cfg & DPS368_MEAS_CFG_PRS_RDY) != 0;
    data->temp_valid = (meas_cfg & DPS368_MEAS_CFG_TMP_RDY) != 0;
    
    if (!data->pressure_valid && !data->temp_valid) {
        return ESP_ERR_NOT_FOUND; // No new data
    }
    
    // Read temperature first (needed for pressure compensation)
    float tmp_scaled = 0.0f;
    if (data->temp_valid) {
        uint8_t tmp_raw[3];
        ret = dps368_read_reg(handle, DPS368_REG_TMP_B2, tmp_raw, 3);
        if (ret != ESP_OK) {
            return ret;
        }
        
        int32_t tmp_raw_val = ((int32_t)tmp_raw[0] << 16) | ((int32_t)tmp_raw[1] << 8) | tmp_raw[2];
        if (tmp_raw_val & 0x800000) tmp_raw_val -= 0x1000000; // Sign extend 24-bit
        
        // Temperature calculation per datasheet Section 4.9.2
        // kT = 253952 for 16x oversampling with shift enabled (Table 9)
        tmp_scaled = (float)tmp_raw_val / 253952.0f;
        data->temperature_c = (float)handle->c0 * 0.5f + (float)handle->c1 * tmp_scaled;
    }
    
    // Read pressure (24-bit, MSB first)
    if (data->pressure_valid) {
        uint8_t psr_raw[3];
        ret = dps368_read_reg(handle, DPS368_REG_PSR_B2, psr_raw, 3);
        if (ret != ESP_OK) {
            return ret;
        }
        
        int32_t psr_raw_val = ((int32_t)psr_raw[0] << 16) | ((int32_t)psr_raw[1] << 8) | psr_raw[2];
        if (psr_raw_val & 0x800000) psr_raw_val -= 0x1000000; // Sign extend 24-bit
        
        // Pressure calculation per datasheet Section 4.9.1
        // kP = 253952 for 16x oversampling with shift enabled (Table 9)
        float psr_scaled = (float)psr_raw_val / 253952.0f;
        
        // Full compensation formula (datasheet Section 4.9.1):
        // Pcomp = c00 + Praw_sc*(c10 + Praw_sc*(c20 + Praw_sc*c30)) + Traw_sc*c01 + Traw_sc*Praw_sc*(c11 + Praw_sc*c21)
        data->pressure_pa = (float)handle->c00 + 
                           psr_scaled * ((float)handle->c10 + 
                           psr_scaled * ((float)handle->c20 + 
                           psr_scaled * (float)handle->c30)) +
                           tmp_scaled * (float)handle->c01 +
                           tmp_scaled * psr_scaled * ((float)handle->c11 + 
                           psr_scaled * (float)handle->c21);
    }
    
    ESP_LOGD(TAG, "Pressure: %.1f Pa, Temp: %.1f°C", data->pressure_pa, data->temperature_c);
    
    return ESP_OK;
}

esp_err_t dps368_deinit(dps368_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->i2c_dev) {
        // Set to standby mode
        dps368_write_reg(handle, DPS368_REG_MEAS_CFG, DPS368_MODE_STANDBY);
        i2c_master_bus_rm_device(handle->i2c_dev);
    }
    
    free(handle);
    return ESP_OK;
}
