#include "cap1203.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "cap1203";

CAP1203::CAP1203(i2c_master_bus_handle_t bus) : bus_(bus), dev_(nullptr), config_() {}

CAP1203::~CAP1203() { _rm_device(); }

void CAP1203::_rm_device() {
  if (dev_ == nullptr) {
    return;
  }

  i2c_master_bus_rm_device(dev_);
  dev_ = nullptr;
}

esp_err_t CAP1203::_add_device() {
  if (bus_ == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  if (dev_ != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config_.i2c_address,
      .scl_speed_hz = config_.scl_speed_hz,
  };

  return i2c_master_bus_add_device(bus_, &dev_cfg, &dev_);
}

esp_err_t CAP1203::init(const CAP1203Config &config) {
  config_ = config;

  if (bus_ == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  if (config_.timeout_ms <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  _rm_device();

  ESP_RETURN_ON_ERROR(i2c_master_probe(bus_, config_.i2c_address, config_.timeout_ms), TAG,
                      "i2c probe failed");

  ESP_RETURN_ON_ERROR(_add_device(), TAG, "add device failed");

  esp_err_t err = probe();
  if (err != ESP_OK) {
    _rm_device();
    return err;
  }

  return ESP_OK;
}

esp_err_t CAP1203::readRegister(uint8_t reg, uint8_t *data) {
  if (dev_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (data == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  return i2c_master_transmit_receive(dev_, &reg, 1, data, 1, config_.timeout_ms);
}

esp_err_t CAP1203::writeRegister(uint8_t reg, uint8_t data) {
  if (dev_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  uint8_t write_buf[2] = {reg, data};
  return i2c_master_transmit(dev_, write_buf, sizeof(write_buf), config_.timeout_ms);
}

esp_err_t CAP1203::readRegisters(uint8_t reg, uint8_t *data, size_t len) {
  if (dev_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (data == nullptr || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  return i2c_master_transmit_receive(dev_, &reg, 1, data, len, config_.timeout_ms);
}

esp_err_t CAP1203::probe() {
  uint8_t product_id = 0;
  uint8_t manufacturer_id = 0;
  uint8_t revision = 0;

  ESP_RETURN_ON_ERROR(readRegister(CAP1203_REG_PRODUCT_ID, &product_id), TAG,
                      "read product id failed");
  ESP_RETURN_ON_ERROR(readRegister(CAP1203_REG_MANUFACTURER_ID, &manufacturer_id), TAG,
                      "read manufacturer id failed");
  ESP_RETURN_ON_ERROR(readRegister(CAP1203_REG_REVISION, &revision), TAG, "read revision failed");

  ESP_LOGI(TAG, "CAP1203 found - Product ID: 0x%02X, Manufacturer: 0x%02X, Revision: 0x%02X",
           product_id, manufacturer_id, revision);

  if (product_id != CAP1203_PRODUCT_ID_EXPECTED) {
    return ESP_ERR_NOT_FOUND;
  }
  if (manufacturer_id != CAP1203_MANUFACTURER_ID_EXPECTED) {
    return ESP_ERR_NOT_FOUND;
  }

  return ESP_OK;
}

esp_err_t CAP1203::readSensorInputStatus(uint8_t *status) {
  return readRegister(CAP1203_REG_SENSOR_INPUT_STATUS, status);
}

esp_err_t CAP1203::readGeneralStatus(uint8_t *status) {
  return readRegister(CAP1203_REG_GENERAL_STATUS, status);
}

esp_err_t CAP1203::readNoiseFlagStatus(uint8_t *status) {
  return readRegister(CAP1203_REG_NOISE_FLAG_STATUS, status);
}

esp_err_t CAP1203::clearInterrupt() {
  uint8_t main_ctrl = 0;
  ESP_RETURN_ON_ERROR(readRegister(CAP1203_REG_MAIN_CONTROL, &main_ctrl), TAG,
                      "read main control failed");

  if ((main_ctrl & 0x01) != 0) {
    main_ctrl &= (uint8_t)~0x01;
    ESP_RETURN_ON_ERROR(writeRegister(CAP1203_REG_MAIN_CONTROL, main_ctrl), TAG,
                        "clear int bit failed");
  }

  uint8_t status = 0;
  return readRegister(CAP1203_REG_GENERAL_STATUS, &status);
}

esp_err_t CAP1203::enableInputs(uint8_t mask) {
  return writeRegister(CAP1203_REG_SENSOR_INPUT_ENABLE, mask);
}

esp_err_t CAP1203::setInterruptEnable(uint8_t mask) {
  return writeRegister(CAP1203_REG_INTERRUPT_ENABLE, mask);
}

esp_err_t CAP1203::setSensitivity(uint8_t delta_sense, uint8_t base_shift) {
  if (delta_sense > 7) {
    return ESP_ERR_INVALID_ARG;
  }
  if (base_shift > 0x0F) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t value = (uint8_t)((delta_sense << 4) | base_shift);
  return writeRegister(CAP1203_REG_SENSITIVITY_CONTROL, value);
}

esp_err_t CAP1203::setThreshold(uint8_t sensor_index, uint8_t threshold) {
  if (sensor_index > 2) {
    return ESP_ERR_INVALID_ARG;
  }
  if (threshold > 127) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg = (uint8_t)(CAP1203_REG_SENSOR_INPUT_1_THRESHOLD + sensor_index);
  return writeRegister(reg, threshold);
}

esp_err_t CAP1203::calibrate(uint8_t mask) {
  return writeRegister(CAP1203_REG_CALIBRATION_ACTIVATE, mask);
}
