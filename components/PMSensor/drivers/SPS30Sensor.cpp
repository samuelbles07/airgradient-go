/*
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "SPS30Sensor.hpp"

#include "esp_log.h"

SPS30Sensor::SPS30Sensor(i2c_master_bus_handle_t bus_handle)
    : bus_handle_(bus_handle), handle_(nullptr) {}

SPS30Sensor::~SPS30Sensor() {
  deinit();
}

void SPS30Sensor::deinit() {
  if (handle_ == nullptr) {
    return;
  }
  (void)sps30_stop_measurement(handle_);
  (void)sps30_delete(handle_);
  handle_ = nullptr;
}

bool SPS30Sensor::init() {
  if (bus_handle_ == nullptr) {
    return false;
  }

  if (handle_ != nullptr) {
    return true;
  }

  sps30_config_t cfg = {};
  cfg.i2c_address = SPS30_I2C_ADDR;
  cfg.i2c_clock_speed = SPS30_I2C_DEV_CLK_SPD;

  esp_err_t err = sps30_init(bus_handle_, &cfg, &handle_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "sps30_init failed: %s", esp_err_to_name(err));
    handle_ = nullptr;
    return false;
  }

  err = sps30_start_measurement(handle_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "sps30_start_measurement failed: %s", esp_err_to_name(err));
    (void)sps30_delete(handle_);
    handle_ = nullptr;
    return false;
  }

  return true;
}

bool SPS30Sensor::read(PMData &out) {
  // Initialize to invalid sentinels.
  out.pm_01 = MeasuresInvalid::PM;
  out.pm_25 = MeasuresInvalid::PM;
  out.pm_10 = MeasuresInvalid::PM;
  out.pm_01_sp = MeasuresInvalid::PM;
  out.pm_25_sp = MeasuresInvalid::PM;
  out.pm_10_sp = MeasuresInvalid::PM;
  out.pm_03_pc = MeasuresInvalid::PM;
  out.pm_05_pc = MeasuresInvalid::PM;
  out.pm_01_pc = MeasuresInvalid::PM;
  out.pm_25_pc = MeasuresInvalid::PM;
  out.pm_5_pc = MeasuresInvalid::PM;
  out.pm_10_pc = MeasuresInvalid::PM;

  if (handle_ == nullptr) {
    return false;
  }

  sps30_measurement_t m = {};
  const esp_err_t err = sps30_read_measurement(handle_, &m);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "sps30_read_measurement failed: %s", esp_err_to_name(err));
    return false;
  }

  // Atmospheric mass concentrations.
  out.pm_01 = m.pm1p0_mass;
  out.pm_25 = m.pm2p5_mass;
  out.pm_10 = m.pm10p0_mass;

  // TODO: Missing mass 4.0 and count 4.0

  // // Particle counts: SPS30 reports [#/cm^3]. PMData expects [#/0.1L] = [#/100cm^3].
  // // Multiply by 100.
  // out.pm_05_pc = m.pm0p5_number * 100.0f;
  // out.pm_01_pc = m.pm1p0_number * 100.0f;
  // out.pm_25_pc = m.pm2p5_number * 100.0f;
  // out.pm_10_pc = m.pm10p0_number * 100.0f;

  out.pm_05_pc = m.pm0p5_number;
  out.pm_01_pc = m.pm1p0_number;
  out.pm_25_pc = m.pm2p5_number;
  out.pm_10_pc = m.pm10p0_number;

  return true;
}

TempHumData SPS30Sensor::temp_hum_data() {
  TempHumData data;
  data.temperature = MeasuresInvalid::TEMPERATURE;
  data.humidity = MeasuresInvalid::HUMIDITY;
  return data;
}
