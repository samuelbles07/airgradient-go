/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "STCC4Sensor.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include <string.h>

STCC4Sensor::STCC4Sensor(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr)
    : bus_handle_(bus_handle), i2c_addr_(i2c_addr), have_temp_hum_(false) {
  memset(&dev_, 0, sizeof(dev_));
  last_temp_hum_.temperature = MeasuresInvalid::TEMPERATURE;
  last_temp_hum_.humidity = MeasuresInvalid::HUMIDITY;
}

STCC4Sensor::~STCC4Sensor() {
  if (dev_.initialized) {
    (void)stcc4_stop_continuous_measurement(&dev_);
    (void)stcc4_deinit(&dev_);
  }
}

bool STCC4Sensor::init() {
  if (bus_handle_ == nullptr) {
    return false;
  }

  if (dev_.initialized) {
    return true;
  }

  esp_err_t err = stcc4_init(&dev_, bus_handle_, i2c_addr_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "stcc4_init failed: %s", esp_err_to_name(err));
    memset(&dev_, 0, sizeof(dev_));
    return false;
  }

  err = stcc4_start_continuous_measurement(&dev_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "stcc4_start_continuous_measurement failed: %s", esp_err_to_name(err));
    (void)stcc4_deinit(&dev_);
    memset(&dev_, 0, sizeof(dev_));
    return false;
  }

  return true;
}

bool STCC4Sensor::read(CO2Data &out) {
  out.co2 = MeasuresInvalid::CO2;

  if (!dev_.initialized) {
    return false;
  }

  stcc4_measurement_t m;
  memset(&m, 0, sizeof(m));

  const esp_err_t err = stcc4_read_measurement(&dev_, &m);
  if (err != ESP_OK) {
    if (err == ESP_ERR_INVALID_RESPONSE) {
      ESP_LOGD(TAG, "stcc4_read_measurement not ready");
    } else {
      ESP_LOGW(TAG, "stcc4_read_measurement failed: %s", esp_err_to_name(err));
    }
    return false;
  }

  out.co2 = (int)m.co2_ppm;

  last_temp_hum_.temperature = m.temperature_c;
  last_temp_hum_.humidity = m.humidity_rh;
  have_temp_hum_ = true;
  return true;
}

TempHumData STCC4Sensor::temp_hum_data() {
  if (!have_temp_hum_) {
    TempHumData data;
    data.temperature = MeasuresInvalid::TEMPERATURE;
    data.humidity = MeasuresInvalid::HUMIDITY;
    return data;
  }
  return last_temp_hum_;
}

bool STCC4Sensor::force_calibration(uint16_t target_ppm) {
  if (!dev_.initialized) {
    ESP_LOGW(TAG, "force_calibration: sensor not initialized");
    return false;
  }

  if (target_ppm == 0) {
    target_ppm = 400;
  }

  // Follow the recommended sequence from the STCC4 library header:
  // stop continuous -> FRC -> start continuous.
  esp_err_t err = stcc4_stop_continuous_measurement(&dev_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "force_calibration: stop continuous failed: %s", esp_err_to_name(err));
    // Continue anyway; FRC may still succeed.
  }

  int16_t correction = 0;
  err = stcc4_perform_forced_recalibration(&dev_, target_ppm, &correction);
  const bool ok = (err == ESP_OK);
  if (ok) {
    ESP_LOGI(TAG, "force_calibration: target=%u ppm correction=%d ppm", (unsigned)target_ppm,
             (int)correction);
  } else {
    ESP_LOGW(TAG, "force_calibration failed: %s", esp_err_to_name(err));
  }

  esp_err_t restart = stcc4_start_continuous_measurement(&dev_);
  if (restart != ESP_OK) {
    ESP_LOGW(TAG, "force_calibration: restart continuous failed: %s", esp_err_to_name(restart));
  }

  return ok;
}
