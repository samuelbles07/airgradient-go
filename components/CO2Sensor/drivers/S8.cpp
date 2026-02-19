/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "S8.hpp"
#include "RTOS.h"
#include "esp_log.h"
#include "modbus_crc.hpp"
#include <cstring>

S8::S8(AirgradientSerial &serial) : serial_(serial), _isCalibrating(false) {
  memset(_buf, 0, BUF_SIZE);
}

bool S8::init() {
  // Clear buffer
  while (serial_.available()) {
    serial_.read();
  }

  // Verify communication by reading firmware version
  RTOS::delay_ms(100);

  int16_t version;
  if (!_readRegister(FUNC_READ_INPUT_REGISTERS, IR_FW_VERSION, version)) {
    ESP_LOGE(TAG, "Failed to read firmware version");
    return false;
  }

  ESP_LOGI(TAG, "Firmware version: %d.%d", (version >> 8) & 0xFF, version & 0xFF);
  ESP_LOGI(TAG, "Sensor initialized, warming up");

  return true;
}

bool S8::read(CO2Data &out) {
  // Initialize to invalid sentinel
  out.co2 = MeasuresInvalid::CO2;

  int16_t co2;
  if (!_readRegister(FUNC_READ_INPUT_REGISTERS, IR_SPACE_CO2, co2)) {
    ESP_LOGW(TAG, "Failed to read CO2 value");
    return false;
  }

  out.co2 = static_cast<int>(co2);
  ESP_LOGD(TAG, "CO2: %d ppm", out.co2);

  return true;
}

TempHumData S8::temp_hum_data() {
  TempHumData data;
  data.temperature = MeasuresInvalid::TEMPERATURE;
  data.humidity = MeasuresInvalid::HUMIDITY;
  return data;
}

bool S8::setBaselineCalibration() {
  if (!_clearAcknowledgement()) {
    ESP_LOGE(TAG, "Failed to clear acknowledgement before calibration");
    return false;
  }

  // Send background calibration command
  if (!_writeRegister(HR_SPECIAL_COMMAND, CMD_BACKGROUND_CALIBRATION)) {
    ESP_LOGE(TAG, "Failed to start baseline calibration");
    return false;
  }

  _isCalibrating = true;
  ESP_LOGI(TAG, "Baseline calibration started");
  return true;
}

bool S8::isBaselineCalibrationDone() {
  if (!_isCalibrating) {
    return true;
  }

  int16_t ack = _getAcknowledgement();
  if (ack < 0) {
    return false;
  }

  if (ack & ACK_CO2_BACKGROUND_CALIB) {
    _isCalibrating = false;
    ESP_LOGI(TAG, "Baseline calibration complete");
    return true;
  }

  return false;
}

bool S8::setAbcPeriod(int hours) {
  if (hours < 0 || hours > 4800) {
    ESP_LOGE(TAG, "Invalid ABC period: %d (must be 0-4800)", hours);
    return false;
  }

  // Check current period
  int currentPeriod = getAbcPeriod();
  if (currentPeriod == hours) {
    ESP_LOGD(TAG, "ABC period already set to %d hours", hours);
    return true;
  }

  if (!_writeRegister(HR_ABC_PERIOD, static_cast<uint16_t>(hours))) {
    ESP_LOGE(TAG, "Failed to set ABC period");
    return false;
  }

  ESP_LOGI(TAG, "ABC period set to %d hours", hours);
  return true;
}

int S8::getAbcPeriod() {
  int16_t period;
  if (!_readRegister(FUNC_READ_HOLDING_REGISTERS, HR_ABC_PERIOD, period)) {
    ESP_LOGW(TAG, "Failed to read ABC period");
    return -1;
  }

  ESP_LOGD(TAG, "ABC period: %d hours", period);
  return static_cast<int>(period);
}

void S8::_sendCommand(uint8_t func, uint16_t reg, uint16_t value) {
  _buf[0] = MODBUS_ADDRESS;
  _buf[1] = func;
  _buf[2] = (reg >> 8) & 0xFF;
  _buf[3] = reg & 0xFF;
  _buf[4] = (value >> 8) & 0xFF;
  _buf[5] = value & 0xFF;

  uint16_t crc = modbus_crc16(_buf, 6);
  _buf[6] = crc & 0xFF;
  _buf[7] = (crc >> 8) & 0xFF;

  serial_.write(_buf, 8);
}

uint8_t S8::_readBytes(uint8_t maxBytes, uint32_t timeoutMs) {
  uint8_t numBytes = 0;
  uint64_t startTime = RTOS::get_time_ms();

  while (numBytes < maxBytes) {
    if (serial_.available()) {
      int byte = serial_.read();
      if (byte >= 0) {
        _buf[numBytes++] = static_cast<uint8_t>(byte);
      }
    }

    if ((RTOS::get_time_ms() - startTime) >= timeoutMs) {
      break;
    }

    RTOS::delay_ms(1);
  }

  return numBytes;
}

bool S8::_validateResponse(uint8_t func, uint8_t numBytes, uint8_t expectedLen) {
  // Check length if specified
  if (expectedLen > 0 && numBytes != expectedLen) {
    ESP_LOGW(TAG, "Invalid response length: %d (expected %d)", numBytes, expectedLen);
    return false;
  }

  // Minimum valid response is 5 bytes (addr + func + len + crc_lo + crc_hi)
  if (numBytes < 5) {
    ESP_LOGW(TAG, "Response too short: %d bytes", numBytes);
    return false;
  }

  // Validate CRC
  uint16_t receivedCrc = (_buf[numBytes - 1] << 8) | _buf[numBytes - 2];
  uint16_t calculatedCrc = modbus_crc16(_buf, numBytes - 2);

  if (receivedCrc != calculatedCrc) {
    ESP_LOGW(TAG, "CRC mismatch: received 0x%04X, calculated 0x%04X", receivedCrc, calculatedCrc);
    return false;
  }

  // Validate address and function code
  if (_buf[0] != MODBUS_ADDRESS) {
    ESP_LOGW(TAG, "Invalid address: 0x%02X", _buf[0]);
    return false;
  }

  if (_buf[1] != func) {
    ESP_LOGW(TAG, "Invalid function code: 0x%02X (expected 0x%02X)", _buf[1], func);
    return false;
  }

  // For read commands, validate byte count
  if (func == FUNC_READ_HOLDING_REGISTERS || func == FUNC_READ_INPUT_REGISTERS) {
    uint8_t byteCount = _buf[2];
    if (numBytes != byteCount + 5) {
      ESP_LOGW(TAG, "Byte count mismatch: %d+5 != %d", byteCount, numBytes);
      return false;
    }
  }

  return true;
}

bool S8::_readRegister(uint8_t func, uint16_t reg, int16_t &value) {
  // Send read command
  _sendCommand(func, reg, 0x0001);

  // Wait for response
  memset(_buf, 0, BUF_SIZE);
  uint8_t numBytes = _readBytes(7, TIMEOUT_MS);

  // Validate response
  if (!_validateResponse(func, numBytes, 7)) {
    return false;
  }

  // Extract value (bytes 3 and 4)
  value = static_cast<int16_t>((_buf[3] << 8) | _buf[4]);
  return true;
}

bool S8::_writeRegister(uint16_t reg, uint16_t value) {
  uint8_t sentBuf[8];

  // Send write command
  _sendCommand(FUNC_WRITE_SINGLE_REGISTER, reg, value);

  // Save sent bytes for comparison
  memcpy(sentBuf, _buf, 8);

  // Wait for response (echoes request)
  memset(_buf, 0, BUF_SIZE);
  uint8_t numBytes = _readBytes(8, TIMEOUT_MS);

  // Response should match request for successful write
  if (numBytes != 8 || memcmp(sentBuf, _buf, 8) != 0) {
    ESP_LOGW(TAG, "Write register response mismatch");
    return false;
  }

  return true;
}

bool S8::_clearAcknowledgement() {
  if (!_writeRegister(HR_ACKNOWLEDGEMENT, 0x0000)) {
    ESP_LOGW(TAG, "Failed to clear acknowledgement");
    return false;
  }

  ESP_LOGD(TAG, "Acknowledgement cleared");
  return true;
}

int16_t S8::_getAcknowledgement() {
  int16_t ack;
  if (!_readRegister(FUNC_READ_HOLDING_REGISTERS, HR_ACKNOWLEDGEMENT, ack)) {
    ESP_LOGW(TAG, "Failed to read acknowledgement");
    return -1;
  }

  return ack;
}
