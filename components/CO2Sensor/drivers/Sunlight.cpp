/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "Sunlight.hpp"
#include "MeasuresTypes.h"
#include "modbus_crc.hpp"
#include "RTOS.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include <cstring>

Sunlight::Sunlight(AirgradientSerial &serial, MeasurementMode mode, gpio_num_t io_power,
                   int measurementPeriodSeconds)
    : serial_(serial), _io_power(io_power), _isCalibrating(false), _measurementTriggered(false),
      _lastTriggerTime(0), _currentMode(mode), _measurementPeriodSeconds(measurementPeriodSeconds) {
  memset(_request, 0, BUF_SIZE);
  memset(_response, 0, BUF_SIZE);
  memset(_values, 0, BUF_SIZE * sizeof(uint16_t));
}

bool Sunlight::init() {
  // Clear serial buffer to remove stale data
  while (serial_.available() > 0) {
    serial_.read();
  }

  // Print device ID (non-critical, don't fail on error)
  printDeviceId();

  // Read all configuration registers in one command (mode, period, samples)
  // HR_MEASUREMENT_MODE (0x000A), HR_MEASUREMENT_PERIOD (0x000B), HR_MEASUREMENT_SAMPLES (0x000C)
  if (!_readRegisters(FUNC_READ_HOLDING_REGISTERS, HR_MEASUREMENT_MODE, 3)) {
    ESP_LOGE(TAG, "Failed to communicate with Sunlight sensor");
    return false;
  }

  // Extract current hardware configuration
  uint16_t currentMode = _values[0];
  uint16_t currentPeriod = _values[1];
  uint16_t currentSamples = _values[2];

  // Print current hardware configuration
  ESP_LOGI(TAG, "=== Current Hardware Configuration ===");
  ESP_LOGI(TAG, "  Measurement Mode: %s (0x%04X)",
           (currentMode == MODE_SINGLE) ? "single" : "continuous", currentMode);
  ESP_LOGI(TAG, "  Measurement Period: %d seconds", currentPeriod);
  ESP_LOGI(TAG, "  Measurement Samples: %d", currentSamples);

  // Save expected values from constructor BEFORE updating cached
  bool configurationChanged = false;
  MeasurementMode expectedMode = _currentMode;
  uint16_t expectedPeriod = _measurementPeriodSeconds;

  // Update cached mode with actual hardware value
  _currentMode = static_cast<MeasurementMode>(currentMode);

  // Check and configure measurement mode
  if (currentMode != static_cast<uint16_t>(expectedMode)) {
    ESP_LOGW(TAG, "Mode mismatch: hardware=%s, expected=%s - auto-correcting",
             (currentMode == MODE_SINGLE) ? "single" : "continuous",
             (expectedMode == MODE_SINGLE) ? "single" : "continuous");

    // Write new mode
    uint16_t modeValue = static_cast<uint16_t>(expectedMode);
    if (!_writeRegisters(HR_MEASUREMENT_MODE, 1, &modeValue)) {
      ESP_LOGE(TAG, "Failed to write measurement mode");
      return false;
    }

    _currentMode = expectedMode;
    configurationChanged = true;
    ESP_LOGI(TAG, "Measurement mode changed to %s",
             (expectedMode == MODE_SINGLE) ? "single" : "continuous");
  }

  // Check and configure measurement period (continuous mode only)
  if (_currentMode == MODE_CONTINUOUS) {
    if (currentPeriod != expectedPeriod) {
      ESP_LOGW(TAG, "Period mismatch: hardware=%d, expected=%d - auto-correcting", currentPeriod,
               expectedPeriod);

      // Write new period
      if (!_writeRegisters(HR_MEASUREMENT_PERIOD, 1, &expectedPeriod)) {
        ESP_LOGE(TAG, "Failed to write measurement period");
        return false;
      }

      _measurementPeriodSeconds = expectedPeriod;
      configurationChanged = true;
      ESP_LOGI(TAG, "Measurement period changed to %d seconds", expectedPeriod);
    }
  }

  // Power cycle if any configuration was changed
  if (configurationChanged) {
    ESP_LOGI(TAG, "Configuration changed, power cycling sensor...");
    if (!powerCycle()) {
      ESP_LOGW(TAG, "Power cycle failed or not available");
      // Don't fail init if power cycle not available
    }
  }

  return true;
}

bool Sunlight::read(CO2Data &out) {
  // Initialize to invalid sentinel
  out.co2 = MeasuresInvalid::CO2;

  // In single-shot mode, trigger NEXT measurement at the START
  if (_currentMode == MODE_SINGLE) {
    uint64_t currentTime = RTOS::get_time_ms();
    bool canTrigger =
        !_measurementTriggered || (currentTime - _lastTriggerTime) >= TRIGGER_COOLDOWN_MS;

    if (canTrigger) {
      if (triggerMeasurement()) {
        _measurementTriggered = true;
        _lastTriggerTime = currentTime;
        ESP_LOGI(TAG, "Triggered next measurement");
        return false;
      } else {
        ESP_LOGW(TAG, "Failed to trigger measurement");
      }
    }
  }

  // Read error status and CO2 value atomically (IR 0x0000 to 0x0003 = 4 registers)
  // Register layout: [0]=error_status, [1]=alarm_status, [2]=output_status, [3]=CO2
  if (!_readRegisters(FUNC_READ_INPUT_REGISTERS, IR_ERROR_STATUS, 4)) {
    ESP_LOGE(TAG, "Failed to read sensor registers");
    return false;
  }

  // Extract values
  int errorStatus = (int)_values[0];
  int co2Value = (int)_values[3];
  ESP_LOGD(TAG, "Error status: 0x%04X, CO2: %d", errorStatus, co2Value);

  // Handle critical errors
  if (errorStatus & ERR_FATAL) {
    ESP_LOGE(TAG, "Fatal sensor error detected");
    return false;
  }
  if (errorStatus & ERR_CALIBRATION) {
    ESP_LOGW(TAG, "Calibration error detected");
    return false;
  }

  // If no measurement available yet, return false
  if (errorStatus & ERR_NO_MEASUREMENT) {
    ESP_LOGI(TAG, "Measurement in progress, not ready yet");
    return false;
  }

  // Log other non-critical errors
  if (errorStatus & ERR_I2C) {
    ESP_LOGW(TAG, "I2C error detected");
  }
  if (errorStatus & ERR_ALGORITHM) {
    ESP_LOGW(TAG, "Algorithm error detected");
  }
  if (errorStatus & ERR_SELFDIAG) {
    ESP_LOGW(TAG, "Self-diagnostic error detected");
  }
  if (errorStatus & ERR_OUT_OF_RANGE) {
    ESP_LOGW(TAG, "Out of range error detected");
  }
  if (errorStatus & ERR_MEMORY) {
    ESP_LOGW(TAG, "Memory error detected");
  }

  // Return the measurement
  out.co2 = co2Value;
  return true;
}

TempHumData Sunlight::temp_hum_data() {
  TempHumData data;
  data.temperature = MeasuresInvalid::TEMPERATURE;
  data.humidity = MeasuresInvalid::HUMIDITY;
  return data;
}

bool Sunlight::setBaselineCalibration() {
  // Check if sensor is readable before starting calibration
  CO2Data testData;
  if (!read(testData)) {
    ESP_LOGE(TAG, "Cannot read sensor, calibration aborted");
    return false;
  }

  // Check error status
  int errorStatus = _getErrorStatus();
  if (errorStatus < 0) {
    ESP_LOGE(TAG, "Failed to read error status, calibration aborted");
    return false;
  }

  // Clear calibration status
  if (!_clearCalibrationStatus()) {
    ESP_LOGE(TAG, "Failed to clear calibration status");
    return false;
  }

  // Start calibration by writing command to HR2
  uint16_t calibCmd = CMD_BACKGROUND_CALIBRATION;
  if (!_writeRegisters(HR_SPECIAL_COMMAND, 1, &calibCmd)) {
    ESP_LOGE(TAG, "Failed to start calibration");
    return false;
  }

  ESP_LOGI(TAG, "Baseline calibration started");
  _isCalibrating = true;

  return true;
}

bool Sunlight::isBaselineCalibrationDone() {
  if (!_isCalibrating) {
    return true; // Not calibrating
  }

  // Check error status for calibration errors
  int errorStatus = _getErrorStatus();
  if (errorStatus >= 0 && (errorStatus & ERR_CALIBRATION)) {
    ESP_LOGE(TAG, "Calibration error detected");
    _isCalibrating = false;
    return false;
  }

  // Check calibration status register
  int calStatus = _getCalibrationStatus();
  if (calStatus < 0) {
    ESP_LOGW(TAG, "Failed to read calibration status");
    return false;
  }

  if (calStatus & CAL_STATUS_BACKGROUND_DONE) {
    ESP_LOGI(TAG, "Calibration complete");
    _isCalibrating = false;
    return true;
  }

  // Still calibrating
  return false;
}

bool Sunlight::setAbcPeriod(int hours) {
  // Read current ABC period
  if (!_readRegisters(FUNC_READ_HOLDING_REGISTERS, HR_ABC_PERIOD, 1)) {
    ESP_LOGE(TAG, "Failed to read ABC period");
    return false;
  }

  if (_values[0] == (uint16_t)hours) {
    ESP_LOGI(TAG, "ABC period already set to %d hours", hours);
    return true;
  }

  // Write new ABC period
  uint16_t newPeriod = (uint16_t)hours;
  if (!_writeRegisters(HR_ABC_PERIOD, 1, &newPeriod)) {
    ESP_LOGE(TAG, "Failed to write ABC period");
    return false;
  }

  ESP_LOGI(TAG, "ABC period set to %d hours", hours);
  return true;
}

int Sunlight::getAbcPeriod() {
  if (!_readRegisters(FUNC_READ_HOLDING_REGISTERS, HR_ABC_PERIOD, 1)) {
    ESP_LOGE(TAG, "Failed to read ABC period");
    return -1;
  }

  return (int)_values[0];
}

bool Sunlight::setMeasurementMode(MeasurementMode mode) {
  // Check if already in target mode
  if (_currentMode == mode) {
    ESP_LOGI(TAG, "Measurement mode already set to %s",
             (mode == MODE_SINGLE) ? "single" : "continuous");
    return false; // No change needed
  }

  // Write new mode
  uint16_t modeValue = static_cast<uint16_t>(mode);
  if (!_writeRegisters(HR_MEASUREMENT_MODE, 1, &modeValue)) {
    ESP_LOGE(TAG, "Failed to write measurement mode");
    return false;
  }

  // Update cached mode
  _currentMode = mode;

  ESP_LOGI(TAG, "Measurement mode changed to %s", (mode == MODE_SINGLE) ? "single" : "continuous");
  return true;
}

bool Sunlight::setMeasurementPeriod(uint16_t seconds) {
  // Read current period
  if (!_readRegisters(FUNC_READ_HOLDING_REGISTERS, HR_MEASUREMENT_PERIOD, 1)) {
    ESP_LOGE(TAG, "Failed to read measurement period");
    return false;
  }

  if (_values[0] == seconds) {
    ESP_LOGI(TAG, "Measurement period already set to %d seconds", seconds);
    return false; // No change needed
  }

  // Write new period
  if (!_writeRegisters(HR_MEASUREMENT_PERIOD, 1, &seconds)) {
    ESP_LOGE(TAG, "Failed to write measurement period");
    return false;
  }

  ESP_LOGI(TAG, "Measurement period set to %d seconds", seconds);
  ESP_LOGW(TAG, "Sensor restart required for changes to take effect");
  return true;
}

bool Sunlight::setMeasurementSamples(uint16_t samples) {
  // Read current samples
  if (!_readRegisters(FUNC_READ_HOLDING_REGISTERS, HR_MEASUREMENT_SAMPLES, 1)) {
    ESP_LOGE(TAG, "Failed to read measurement samples");
    return false;
  }

  if (_values[0] == samples) {
    ESP_LOGI(TAG, "Measurement samples already set to %d", samples);
    return false; // No change needed
  }

  // Write new samples
  if (!_writeRegisters(HR_MEASUREMENT_SAMPLES, 1, &samples)) {
    ESP_LOGE(TAG, "Failed to write measurement samples");
    return false;
  }

  ESP_LOGI(TAG, "Measurement samples set to %d", samples);
  ESP_LOGW(TAG, "Sensor restart required for changes to take effect");
  return true;
}

bool Sunlight::triggerMeasurement() {
  // Trigger single measurement by writing to HR34
  uint16_t trigger = 0x0001;
  if (!_writeRegisters(HR_START_SINGLE_MEASUREMENT, 1, &trigger)) {
    ESP_LOGE(TAG, "Failed to trigger single measurement");
    return false;
  }

  ESP_LOGD(TAG, "Single measurement triggered");
  return true;
}

bool Sunlight::isSingleMode() {
  if (!_readRegisters(FUNC_READ_HOLDING_REGISTERS, HR_MEASUREMENT_MODE, 1)) {
    ESP_LOGE(TAG, "Failed to read measurement mode");
    return false; // Assume continuous mode on error
  }

  return (_values[0] == MODE_SINGLE);
}

bool Sunlight::printDeviceId() {
  char id[64];
  memset(id, 0, 64);

  // Vendor Name
  if (!_readDeviceIdentification(0, id, sizeof(id))) {
    ESP_LOGE(TAG, "Failed to read vendor name");
    return false;
  }
  ESP_LOGI(TAG, "Vendor name: %s", id);

  // Product code
  memset(id, 0, 64);
  if (!_readDeviceIdentification(1, id, sizeof(id))) {
    ESP_LOGE(TAG, "Failed to read product code");
    return false;
  }
  ESP_LOGI(TAG, "Product code: %s", id);

  // Major minor revision
  memset(id, 0, 64);
  if (!_readDeviceIdentification(2, id, sizeof(id))) {
    ESP_LOGE(TAG, "Failed to read MajorMinorRevision");
    return false;
  }
  ESP_LOGI(TAG, "MajorMinorRevision: %s", id);

  return true;
}

// Private helper methods

bool Sunlight::_readRegisters(uint8_t func, uint16_t reg, uint16_t numReg) {
  // Build Modbus request
  _request[0] = MODBUS_ADDRESS;
  _request[1] = func;
  _request[2] = (reg >> 8) & 0xFF;
  _request[3] = reg & 0xFF;
  _request[4] = (numReg >> 8) & 0xFF;
  _request[5] = numReg & 0xFF;

  // Calculate CRC
  uint16_t crc = modbus_crc16(_request, 6);
  _request[6] = crc & 0xFF;
  _request[7] = (crc >> 8) & 0xFF;

  // Send request
  serial_.write(_request, 8);

  // Wait for response
  int expectedBytes = 5 + (numReg * 2); // Address + Func + ByteCount + Data + CRC
  int numBytes = _waitForResponse(expectedBytes);

  if (numBytes < 0) {
    ESP_LOGE(TAG, "Timeout waiting for response");
    return false;
  }

  // Validate response
  if (!_validateResponse(func, numBytes)) {
    return false;
  }

  // Extract register values from response
  int dataOffset = 3; // Skip address, function, byte count
  for (uint16_t i = 0; i < numReg; i++) {
    _values[i] = ((uint16_t)_response[dataOffset] << 8) | _response[dataOffset + 1];
    dataOffset += 2;
  }

  return true;
}

bool Sunlight::_writeRegisters(uint16_t reg, uint16_t numReg, const uint16_t *values) {
  uint8_t numBytes = numReg * 2;

  // Build Modbus request
  _request[0] = MODBUS_ADDRESS;
  _request[1] = FUNC_WRITE_MULTIPLE_REGISTERS;
  _request[2] = (reg >> 8) & 0xFF;
  _request[3] = reg & 0xFF;
  _request[4] = (numReg >> 8) & 0xFF;
  _request[5] = numReg & 0xFF;
  _request[6] = numBytes;

  // Add register values
  int offset = 7;
  for (uint16_t i = 0; i < numReg; i++) {
    _request[offset++] = (values[i] >> 8) & 0xFF;
    _request[offset++] = values[i] & 0xFF;
  }

  // Calculate CRC
  uint16_t crc = modbus_crc16(_request, offset);
  _request[offset++] = crc & 0xFF;
  _request[offset++] = (crc >> 8) & 0xFF;

  // Send request
  serial_.write(_request, offset);

  // Wait for response (write response is 8 bytes)
  int responseBytes = _waitForResponse(8);
  if (responseBytes < 0) {
    ESP_LOGE(TAG, "Timeout waiting for write response");
    return false;
  }

  // Validate response
  if (!_validateResponse(FUNC_WRITE_MULTIPLE_REGISTERS, responseBytes)) {
    return false;
  }

  return true;
}

bool Sunlight::_readDeviceIdentification(uint8_t objectId, char *out, size_t outLen) {
  uint8_t request[7];

  // Build Modbus Device ID request
  request[0] = MODBUS_ADDRESS;
  request[1] = 0x2B; // Encapsulated Interface
  request[2] = 0x0E; // MEI type: Device Identification
  request[3] = 0x04; // Read Basic Device Identification
  request[4] = objectId;

  // CRC
  uint16_t crc = modbus_crc16(request, 5);
  request[5] = crc & 0xFF;
  request[6] = crc >> 8;

  // Send request
  serial_.write(request, 7);

  // Response is variable-length, wait for "enough" with longer timeout
  // Device ID responses can be slow, use 2000ms timeout
  int numBytes = _waitForResponse(256, 2000);
  if (numBytes < 5) {
    ESP_LOGE(TAG, "Invalid or no Device ID response (received %d bytes)", numBytes);
    return false;
  }

  // // Log received bytes for debugging
  // ESP_LOGD(TAG, "Device ID response: %d bytes", numBytes);
  // if (numBytes > 0) {
  //   char hexStr[128];
  //   int offset = 0;
  //   for (int i = 0; i < numBytes && i < 20; i++) {
  //     offset += snprintf(hexStr + offset, sizeof(hexStr) - offset, "%02X ", _response[i]);
  //   }
  //   if (numBytes > 20) {
  //     snprintf(hexStr + offset, sizeof(hexStr) - offset, "...");
  //   }
  //   ESP_LOGD(TAG, "Raw data: %s", hexStr);
  // }

  // Check for Modbus exception response (5 bytes: Addr + ExceptionFunc + Code + CRC)
  if (numBytes == 5 && (_response[1] & 0x80)) {
    uint8_t exceptionCode = _response[2];
    ESP_LOGW(TAG, "Device ID not supported - Modbus exception: code %d", exceptionCode);
    return false;
  }

  // Check minimum length for valid Device ID response
  if (numBytes < 9) {
    ESP_LOGE(TAG, "Device ID response too short (received %d bytes, expected ≥9)", numBytes);
    return false;
  }

  // Validate header
  if (_response[0] != MODBUS_ADDRESS || _response[1] != 0x2B || _response[2] != 0x0E) {
    ESP_LOGE(TAG, "Invalid Device ID header (addr=0x%02X, func=0x%02X, mei=0x%02X)",
             _response[0], _response[1], _response[2]);
    return false;
  }

  /*
   Response layout:
   [0] Addr
   [1] 0x2B
   [2] 0x0E
   [3] ReadCode
   [4] ConformityLevel
   [5] MoreFollows
   [6] NextObjectId
   [7] ObjectCount
   [8...] Objects
  */

  uint8_t objCount = _response[7];
  int idx = 8;

  for (uint8_t i = 0; i < objCount; i++) {
    if (idx + 2 > numBytes)
      break;

    uint8_t id = _response[idx++];
    uint8_t len = _response[idx++];

    if (idx + len > numBytes)
      break;

    if (id == objectId) {
      size_t copyLen = (len < outLen - 1) ? len : outLen - 1;
      memcpy(out, &_response[idx], copyLen);
      out[copyLen] = '\0';
      return true;
    }

    idx += len;
  }

  ESP_LOGW(TAG, "Device ID object %d not found", objectId);
  return false;
}

int Sunlight::_waitForResponse(int expectedBytes, uint32_t timeoutMs) {
  uint64_t startTime = RTOS::get_time_ms();
  int availableBytes = 0;
  uint64_t lastByteTime = 0;

  // Wait for first byte
  while (serial_.available() == 0) {
    uint64_t elapsed = RTOS::get_time_ms() - startTime;
    if (elapsed > timeoutMs) {
      return -1; // Timeout
    }
  }

  // Wait for all bytes with inter-packet detection
  lastByteTime = RTOS::get_time_ms();

  while (true) {
    int newAvailable = serial_.available();
    uint64_t currentTime = RTOS::get_time_ms();

    if (newAvailable != availableBytes) {
      // New bytes arrived
      availableBytes = newAvailable;
      lastByteTime = currentTime;
    } else if (currentTime - lastByteTime >= INTER_PACKET_INTERVAL_MS) {
      // No new bytes for inter-packet interval, assume complete
      break;
    }

    // Check overall timeout
    if (currentTime - startTime > timeoutMs) {
      ESP_LOGW(TAG, "Response timeout after %d bytes", availableBytes);
      break;
    }
  }

  // Read available bytes
  int bytesToRead = (availableBytes < BUF_SIZE) ? availableBytes : BUF_SIZE;
  for (int i = 0; i < bytesToRead; i++) {
    _response[i] = serial_.read();
  }

  return bytesToRead;
}

bool Sunlight::_validateResponse(uint8_t func, uint8_t numBytes) {
  // Minimum response is 5 bytes (addr + func + data + CRC)
  if (numBytes < 5) {
    ESP_LOGE(TAG, "Response too short: %d bytes", numBytes);
    return false;
  }

  // Verify CRC
  uint16_t expectedCrc = modbus_crc16(_response, numBytes - 2);
  uint16_t receivedCrc = _response[numBytes - 2] | (_response[numBytes - 1] << 8);

  if (expectedCrc != receivedCrc) {
    ESP_LOGE(TAG, "CRC mismatch: expected 0x%04X, got 0x%04X", expectedCrc, receivedCrc);
    return false;
  }

  // Check for Modbus exception
  if (_response[1] == (func | 0x80)) {
    uint8_t exceptionCode = _response[2];
    ESP_LOGE(TAG, "Modbus exception: code %d", exceptionCode);
    return false;
  }

  // Verify function code
  if (_response[1] != func) {
    ESP_LOGE(TAG, "Function code mismatch: expected 0x%02X, got 0x%02X", func, _response[1]);
    return false;
  }

  return true;
}

int Sunlight::_getErrorStatus() {
  if (!_readRegisters(FUNC_READ_INPUT_REGISTERS, IR_ERROR_STATUS, 1)) {
    return -1;
  }

  return (int)_values[0];
}

bool Sunlight::_clearCalibrationStatus() {
  uint16_t resetValue = CAL_STATUS_RESET;
  if (!_writeRegisters(HR_CALIBRATION_STATUS, 1, &resetValue)) {
    ESP_LOGE(TAG, "Failed to clear calibration status");
    return false;
  }

  return true;
}

int Sunlight::_getCalibrationStatus() {
  if (!_readRegisters(FUNC_READ_HOLDING_REGISTERS, HR_CALIBRATION_STATUS, 1)) {
    return -1;
  }

  return (int)_values[0];
}

bool Sunlight::powerCycle() {
  if (_io_power == GPIO_NUM_MAX) {
    ESP_LOGW(TAG, "Sensor restart required for changes to take effect");
    return false;
  }

  gpio_set_level(_io_power, 0); // Turn off CO2 sensor
  RTOS::delay_ms(2000);
  gpio_set_level(_io_power, 1); // Turn on CO2 sensor
  RTOS::delay_ms(3000);         // stabilize

  // Wait for valid CO2 reading (non-zero value)
  ESP_LOGI(TAG, "Waiting for valid CO2 reading after restart...");
  int retry_count = 0;
  const int max_retries = 30; // Maximum 30 attempts (about 30 seconds)

  do {
    CO2Data data;
    read(data);

    retry_count++;
    ESP_LOGI(TAG, "CO2 reading attempt %d: %d ppm", retry_count, data.co2);

    if (data.is_valid()) {
      ESP_LOGI(TAG, "Valid CO2 reading obtained: %d ppm", data.co2);
      break;
    }

    RTOS::delay_ms(2400); // wait 1 second before next attempt
  } while (retry_count < max_retries);

  if (retry_count >= max_retries) {
    ESP_LOGW(TAG, "Maximum retry attempts reached, proceeding with "
                  "current reading");
    return false;
  }

  return true;
}
