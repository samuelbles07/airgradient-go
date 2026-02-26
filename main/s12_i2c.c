#include "s12_i2c.h"

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

// S12 register map
#define REG_CO2_FILT_PRES_COMP_HI 0x06

#define REG_ERROR_STATUS_LSB 0x01

#define REG_CALIBRATION_STATUS 0x81
#define REG_CALIBRATION_COMMAND_MSB 0x82
#define REG_CALIBRATION_TARGET_MSB 0x84

#define REG_MEASUREMENT_PERIOD_MSB 0x96

#define CALIBRATION_STATUS_BACKGROUND_COMPLETE 0x20
// Per S12 documentation: ErrorStatus low byte has Calibration on bit3 (0x08)
#define ERROR_STATUS_LSB_CALIBRATION 0x08

#define CALIBRATION_PREFIX 0x7C
#define CALIBRATION_BACKGROUND 0x06

#define CALIBRATION_MAX_ATTEMPTS 10

#define IO_RETRY_COUNT 5
#define IO_RETRY_DELAY_MS 10

static const char* TAG = "s12_i2c";

static esp_err_t s12_read_reg_retry_(s12_i2c_t* s, uint8_t reg, uint8_t* buf, size_t len);

static esp_err_t s12_write_retry_(s12_i2c_t* s, const uint8_t* buf, size_t len) {
    esp_err_t last_err = ESP_FAIL;

    for (int i = 0; i < IO_RETRY_COUNT; i++) {
        last_err = i2c_master_transmit(s->dev, buf, len, 1000);
        if (last_err == ESP_OK) {
            return ESP_OK;
        }
        (void)i2c_master_probe(s->bus, s->addr_7bit, 20);
        vTaskDelay(pdMS_TO_TICKS(IO_RETRY_DELAY_MS));
    }

    ESP_LOGD(TAG, "write failed after retries: %s", esp_err_to_name(last_err));
    return last_err;
}

static esp_err_t s12_read_measurement_period_ms_(s12_i2c_t* s, int* out_ms) {
    if (out_ms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data[2] = {0};
    esp_err_t err = s12_read_reg_retry_(s, REG_MEASUREMENT_PERIOD_MSB, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    // MEAS_PRD is 11-bit value in seconds: MSB[2:0] in 0x96 and LSB in 0x97.
    uint16_t period_s = (uint16_t)(((uint16_t)(data[0] & 0x07) << 8) | (uint16_t)data[1]);
    if (period_s == 0) {
        period_s = 1;
    }
    *out_ms = (int)period_s * 1000;
    return ESP_OK;
}

static esp_err_t s12_read_reg_retry_(s12_i2c_t* s, uint8_t reg, uint8_t* buf, size_t len) {
    esp_err_t last_err = ESP_FAIL;

    for (int i = 0; i < IO_RETRY_COUNT; i++) {
        last_err = i2c_master_transmit_receive(s->dev, &reg, 1, buf, len, 1000);
        if (last_err == ESP_OK) {
            return ESP_OK;
        }

        // Tickle the bus in case the target is waking up.
        (void)i2c_master_probe(s->bus, s->addr_7bit, 20);
        vTaskDelay(pdMS_TO_TICKS(IO_RETRY_DELAY_MS));
    }

    ESP_LOGD(TAG, "read reg 0x%02X failed after retries: %s", reg, esp_err_to_name(last_err));
    return last_err;
}

static esp_err_t s12_write_calibration_target_(s12_i2c_t* s, uint16_t target_ppm) {
    const uint8_t w[] = {REG_CALIBRATION_TARGET_MSB, (uint8_t)((target_ppm >> 8) & 0xFF),
                         (uint8_t)(target_ppm & 0xFF)};
    return s12_write_retry_(s, w, sizeof(w));
}

esp_err_t s12_i2c_create(i2c_master_bus_handle_t bus,
                         uint16_t addr_7bit,
                         uint32_t scl_speed_hz,
                         s12_i2c_t* out) {
    if (bus == NULL || out == NULL || addr_7bit > 0x7F || scl_speed_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_7bit,
        .scl_speed_hz = scl_speed_hz,
        .scl_wait_us = 0,
        .flags.disable_ack_check = 0,
    };

    s12_i2c_t s = {
        .bus = bus,
        .dev = NULL,
        .addr_7bit = addr_7bit,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s.dev);
    if (err != ESP_OK) {
        return err;
    }

    *out = s;
    return ESP_OK;
}

void s12_i2c_destroy(s12_i2c_t* s) {
    if (s == NULL) {
        return;
    }
    if (s->dev != NULL) {
        (void)i2c_master_bus_rm_device(s->dev);
        s->dev = NULL;
    }
    s->bus = NULL;
}

esp_err_t s12_i2c_read_co2_ppm(s12_i2c_t* s, uint16_t* out_co2_ppm) {
    if (s == NULL || s->dev == NULL || out_co2_ppm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2] = {0};
    esp_err_t err = s12_read_reg_retry_(s, REG_CO2_FILT_PRES_COMP_HI, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    *out_co2_ppm = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    return ESP_OK;
}

esp_err_t s12_i2c_background_calibration(s12_i2c_t* s, int measurement_period_ms) {
    if (s == NULL || s->dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (measurement_period_ms <= 0) {
        int period_ms = 0;
        esp_err_t err = s12_read_measurement_period_ms_(s, &period_ms);
        if (err == ESP_OK && period_ms > 0) {
            measurement_period_ms = period_ms;
        } else {
            measurement_period_ms = 4000;
        }
    }

    // Clear previous calibration status.
    {
        const uint8_t w[] = {REG_CALIBRATION_STATUS, 0x00};
        esp_err_t err = s12_write_retry_(s, w, sizeof(w));
        if (err != ESP_OK) {
            return err;
        }
    }

    // Start background calibration: write 0x7C06 to 0x82/0x83.
    {
        const uint8_t w[] = {REG_CALIBRATION_COMMAND_MSB, CALIBRATION_PREFIX, CALIBRATION_BACKGROUND};
        esp_err_t err = s12_write_retry_(s, w, sizeof(w));
        if (err != ESP_OK) {
            return err;
        }
    }

    for (int attempt = 0; attempt < CALIBRATION_MAX_ATTEMPTS; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(measurement_period_ms));

        uint8_t e_status = 0;
        esp_err_t err = s12_read_reg_retry_(s, REG_ERROR_STATUS_LSB, &e_status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (e_status & ERROR_STATUS_LSB_CALIBRATION) {
            return ESP_FAIL;
        }

        uint8_t cal_status = 0;
        err = s12_read_reg_retry_(s, REG_CALIBRATION_STATUS, &cal_status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (cal_status & CALIBRATION_STATUS_BACKGROUND_COMPLETE) {
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t s12_i2c_force_calibration(s12_i2c_t* s, uint16_t target_ppm, int measurement_period_ms) {
    if (s == NULL || s->dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (target_ppm == 0) {
        target_ppm = 400;
    }
    esp_err_t err = s12_write_calibration_target_(s, target_ppm);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "write target failed: %s", esp_err_to_name(err));
        return err;
    }
    return s12_i2c_background_calibration(s, measurement_period_ms);
}
