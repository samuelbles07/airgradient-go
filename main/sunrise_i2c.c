#include "sunrise_i2c.h"

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

// Register addresses (per Senseair Sunrise I2C documentation / Arduino-style example)
#define REG_ERROR_STATUS 0x01
#define REG_MEASUREMENT_MODE 0x95

#define REG_CALIBRATION_STATUS 0x81
#define REG_CALIBRATION_COMMAND_MSB 0x82
#define REG_CALIBRATION_TARGET_MSB 0x84

#define CALIBRATION_STATUS_BACKGROUND_COMPLETE 0x20

// Sunrise: calibration error on error status (byte at REG_ERROR_STATUS) bit7 (0x80)
#define ERROR_STATUS_CALIBRATION 0x80

#define CALIBRATION_PREFIX 0x7C
#define CALIBRATION_BACKGROUND 0x06

#define CALIBRATION_MAX_ATTEMPTS 10

#define IO_RETRY_COUNT 5
#define IO_RETRY_DELAY_MS 10

static const char* TAG = "sunrise_i2c";

static esp_err_t sunrise_write_retry_(sunrise_i2c_t* s, const uint8_t* buf, size_t len) {
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

static esp_err_t sunrise_read_reg_retry_(sunrise_i2c_t* s, uint8_t reg, uint8_t* buf, size_t len) {
    esp_err_t last_err = ESP_FAIL;

    for (int i = 0; i < IO_RETRY_COUNT; i++) {
        last_err = i2c_master_transmit_receive(s->dev, &reg, 1, buf, len, 1000);
        if (last_err == ESP_OK) {
            return ESP_OK;
        }

        // Tickle the bus to wake the sensor.
        (void)i2c_master_probe(s->bus, s->addr_7bit, 20);

        vTaskDelay(pdMS_TO_TICKS(IO_RETRY_DELAY_MS));
    }

    ESP_LOGD(TAG, "read reg 0x%02X failed after retries: %s", reg, esp_err_to_name(last_err));
    return last_err;
}

static esp_err_t sunrise_write_calibration_target_(sunrise_i2c_t* s, uint16_t target_ppm) {
    const uint8_t w[] = {REG_CALIBRATION_TARGET_MSB, (uint8_t)((target_ppm >> 8) & 0xFF),
                         (uint8_t)(target_ppm & 0xFF)};
    return sunrise_write_retry_(s, w, sizeof(w));
}

esp_err_t sunrise_i2c_create(i2c_master_bus_handle_t bus,
                             uint16_t addr_7bit,
                             uint32_t scl_speed_hz,
                             sunrise_i2c_t* out) {
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

    sunrise_i2c_t s = {
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

void sunrise_i2c_destroy(sunrise_i2c_t* s) {
    if (s == NULL) {
        return;
    }
    if (s->dev != NULL) {
        (void)i2c_master_bus_rm_device(s->dev);
        s->dev = NULL;
    }
    s->bus = NULL;
}

esp_err_t sunrise_i2c_read_config(sunrise_i2c_t* s, uint8_t* out_measurement_mode, int* out_period_ms) {
    if (s == NULL || s->dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cfg[7] = {0};
    esp_err_t err = sunrise_read_reg_retry_(s, REG_MEASUREMENT_MODE, cfg, sizeof(cfg));
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t meas_mode = cfg[0];
    const int16_t meas_period_s = (int16_t)((int16_t)(int8_t)cfg[1] << 8) | (int16_t)cfg[2];

    if (out_measurement_mode != NULL) {
        *out_measurement_mode = meas_mode;
    }
    if (out_period_ms != NULL) {
        if (meas_period_s > 0) {
            *out_period_ms = (int)meas_period_s * 1000;
        } else {
            *out_period_ms = 4000;
        }
    }
    return ESP_OK;
}

esp_err_t sunrise_i2c_read_co2_ppm(sunrise_i2c_t* s, uint16_t* out_co2_ppm, uint8_t* out_error_status) {
    if (s == NULL || s->dev == NULL || out_co2_ppm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[7] = {0};
    esp_err_t err = sunrise_read_reg_retry_(s, REG_ERROR_STATUS, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    if (out_error_status != NULL) {
        *out_error_status = data[0];
    }

    *out_co2_ppm = (uint16_t)(((uint16_t)data[5] << 8) | (uint16_t)data[6]);
    return ESP_OK;
}

esp_err_t sunrise_i2c_background_calibration(sunrise_i2c_t* s, int measurement_period_ms) {
    if (s == NULL || s->dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (measurement_period_ms <= 0) {
        int period_ms = 0;
        esp_err_t err = sunrise_i2c_read_config(s, NULL, &period_ms);
        if (err == ESP_OK && period_ms > 0) {
            measurement_period_ms = period_ms;
        } else {
            measurement_period_ms = 4000;
        }
    }

    // Clear previous calibration status.
    {
        const uint8_t w[] = {REG_CALIBRATION_STATUS, 0x00};
        esp_err_t err = sunrise_write_retry_(s, w, sizeof(w));
        if (err != ESP_OK) {
            return err;
        }
    }

    // Start background calibration: write 0x7C06 to 0x82/0x83.
    {
        const uint8_t w[] = {REG_CALIBRATION_COMMAND_MSB, CALIBRATION_PREFIX, CALIBRATION_BACKGROUND};
        esp_err_t err = sunrise_write_retry_(s, w, sizeof(w));
        if (err != ESP_OK) {
            return err;
        }
    }

    // Calibration completes after the next measurement period (sometimes 2 if not aligned).
    for (int attempt = 0; attempt < CALIBRATION_MAX_ATTEMPTS; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(measurement_period_ms));

        uint8_t e_status = 0;
        esp_err_t err = sunrise_read_reg_retry_(s, REG_ERROR_STATUS, &e_status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (e_status & ERROR_STATUS_CALIBRATION) {
            return ESP_FAIL;
        }

        uint8_t cal_status = 0;
        err = sunrise_read_reg_retry_(s, REG_CALIBRATION_STATUS, &cal_status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (cal_status & CALIBRATION_STATUS_BACKGROUND_COMPLETE) {
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t sunrise_i2c_force_calibration(sunrise_i2c_t* s, uint16_t target_ppm, int measurement_period_ms) {
    if (s == NULL || s->dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (target_ppm == 0) {
        target_ppm = 400;
    }
    esp_err_t err = sunrise_write_calibration_target_(s, target_ppm);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "write target failed: %s", esp_err_to_name(err));
        return err;
    }
    return sunrise_i2c_background_calibration(s, measurement_period_ms);
}
