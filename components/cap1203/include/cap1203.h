#ifndef CAP1203_H_
#define CAP1203_H_

#include <stddef.h>
#include <stdint.h>

#include <driver/i2c_master.h>
#include "esp_err.h"

static constexpr uint16_t CAP1203_DEFAULT_I2C_ADDRESS = 0x28;
static constexpr uint32_t CAP1203_DEFAULT_I2C_SCL_SPEED_HZ = 100000;
static constexpr int CAP1203_DEFAULT_I2C_TIMEOUT_MS = 1000;

static constexpr uint8_t CAP1203_PRODUCT_ID_EXPECTED = 0x6D;
static constexpr uint8_t CAP1203_MANUFACTURER_ID_EXPECTED = 0x5D;

static constexpr uint8_t CAP1203_REG_MAIN_CONTROL = 0x00;
static constexpr uint8_t CAP1203_REG_GENERAL_STATUS = 0x02;
static constexpr uint8_t CAP1203_REG_SENSOR_INPUT_STATUS = 0x03;
static constexpr uint8_t CAP1203_REG_NOISE_FLAG_STATUS = 0x0A;
static constexpr uint8_t CAP1203_REG_SENSOR_INPUT_1_DELTA = 0x10;
static constexpr uint8_t CAP1203_REG_SENSITIVITY_CONTROL = 0x1F;
static constexpr uint8_t CAP1203_REG_SENSOR_INPUT_ENABLE = 0x21;
static constexpr uint8_t CAP1203_REG_INTERRUPT_ENABLE = 0x27;
static constexpr uint8_t CAP1203_REG_CALIBRATION_ACTIVATE = 0x26;
static constexpr uint8_t CAP1203_REG_SENSOR_INPUT_1_THRESHOLD = 0x30;
static constexpr uint8_t CAP1203_REG_PRODUCT_ID = 0xFD;
static constexpr uint8_t CAP1203_REG_MANUFACTURER_ID = 0xFE;
static constexpr uint8_t CAP1203_REG_REVISION = 0xFF;

struct CAP1203Config {
  uint16_t i2c_address = CAP1203_DEFAULT_I2C_ADDRESS;
  uint32_t scl_speed_hz = CAP1203_DEFAULT_I2C_SCL_SPEED_HZ;
  int timeout_ms = CAP1203_DEFAULT_I2C_TIMEOUT_MS;
};

class CAP1203 {
public:
  explicit CAP1203(i2c_master_bus_handle_t bus);
  ~CAP1203();

  CAP1203(const CAP1203 &) = delete;
  CAP1203 &operator=(const CAP1203 &) = delete;

  esp_err_t init(const CAP1203Config &config = CAP1203Config());

  esp_err_t probe();

  esp_err_t readRegister(uint8_t reg, uint8_t *data);
  esp_err_t writeRegister(uint8_t reg, uint8_t data);
  esp_err_t readRegisters(uint8_t reg, uint8_t *data, size_t len);

  esp_err_t readSensorInputStatus(uint8_t *status);
  esp_err_t readGeneralStatus(uint8_t *status);
  esp_err_t readNoiseFlagStatus(uint8_t *status);

  esp_err_t clearInterrupt();

  esp_err_t enableInputs(uint8_t mask);
  esp_err_t setInterruptEnable(uint8_t mask);

  esp_err_t setSensitivity(uint8_t delta_sense, uint8_t base_shift);
  esp_err_t setThreshold(uint8_t sensor_index, uint8_t threshold);
  esp_err_t calibrate(uint8_t mask);

  uint16_t i2c_address() const { return config_.i2c_address; }
  i2c_master_dev_handle_t dev_handle() const { return dev_; }

private:
  esp_err_t _add_device();
  void _rm_device();

  i2c_master_bus_handle_t bus_;
  i2c_master_dev_handle_t dev_;
  CAP1203Config config_;
};

#endif  // AIRGRADIENT_GO_COMPONENTS_CAP1203_INCLUDE_CAP1203_H
