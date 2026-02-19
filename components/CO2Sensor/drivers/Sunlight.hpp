/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef SUNLIGHT_HPP
#define SUNLIGHT_HPP

#include "AirgradientSerial.hpp"
#include "CO2Sensor.hpp"
#include <stdint.h>
#include "driver/gpio.h"
#include "soc/gpio_num.h"

/**
 * @brief SenseAir Sunlight CO2 sensor driver
 *
 * Communicates with sensor using Modbus RTU protocol over serial (9600 baud).
 * Supports CO2 reading, automatic baseline calibration (ABC), manual calibration,
 * and measurement mode configuration (continuous/single-shot).
 */
class Sunlight : public CO2Sensor {
public:
  // Measurement modes
  enum MeasurementMode {
    MODE_CONTINUOUS = 0x0000,
    MODE_SINGLE = 0x0001,
  };

  /**
   * @brief Construct Sunlight sensor with serial interface
   * @param serial Reference to initialized AirgradientSerial instance
   * @param mode Expected measurement mode (MODE_CONTINUOUS or MODE_SINGLE)
   * @param io_power gpio to power cycle the sensor if any
   * @param measurementPeriodSeconds Measurement period in seconds (default: 4)
   * @note init() will auto-correct hardware mode if it doesn't match expected mode
   */
  explicit Sunlight(AirgradientSerial &serial, MeasurementMode mode,
                    gpio_num_t io_power = GPIO_NUM_MAX, int measurementPeriodSeconds = 4);
  virtual ~Sunlight() = default;

  // CO2Sensor interface implementation
  bool init() override;
  bool read(CO2Data &out) override;

  bool support_temp_hum() const override { return false; }
  TempHumData temp_hum_data() override;

  /**
   * @brief Perform manual baseline calibration to 400 PPM
   * @note Sensor must be in clean air environment for 5 minutes before calibration
   * @return true if calibration started successfully, false otherwise
   */
  bool setBaselineCalibration();

  /**
   * @brief Check if baseline calibration is complete
   * @return true if calibration done, false if still in progress
   */
  bool isBaselineCalibrationDone();

  /**
   * @brief Set ABC (Automatic Baseline Calibration) period
   * @param hours Calibration period in hours (0 = disable, 4-4800 = enable)
   * @return true if successful, false otherwise
   */
  bool setAbcPeriod(int hours);

  /**
   * @brief Get current ABC period
   * @return ABC period in hours, -1 on error
   */
  int getAbcPeriod();

  /**
   * @brief Set measurement mode
   * @param mode Measurement mode (MODE_CONTINUOUS or MODE_SINGLE)
   * @return true if mode changed (requires sensor restart), false if already set
   * @note Sensor restart required for changes to take effect
   */
  bool setMeasurementMode(MeasurementMode mode);

  /**
   * @brief Set measurement period
   * @param seconds Measurement period in seconds
   * @return true if period changed (requires sensor restart), false if already set
   * @note Sensor restart required for changes to take effect
   */
  bool setMeasurementPeriod(uint16_t seconds);

  /**
   * @brief Set number of measurement samples
   * @param samples Number of samples per measurement
   * @return true if samples changed (requires sensor restart), false otherwise
   * @note Sensor restart required for changes to take effect
   */
  bool setMeasurementSamples(uint16_t samples);

  /**
   * @brief Trigger a single measurement
   * @return true if successful, false otherwise
   * @note Only works when sensor is in single-shot mode
   */
  bool triggerMeasurement();

  /**
   * @brief Check if sensor is in single-shot mode
   * @return true if in single mode, false if in continuous mode
   */
  bool isSingleMode();

  /**
   * @brief Log sensor information
   */
  bool printDeviceId();

  /**
   * @brief Power cycle sensor if io_power is set
   */
  bool powerCycle();

private:
  const char *const TAG = "Sunlight";
  AirgradientSerial &serial_;
  gpio_num_t _io_power;

  static constexpr int BAUDRATE = 9600;
  static constexpr int TIMEOUT_MS = 180; // Sunlight-specific shorter timeout
  static constexpr int INTER_PACKET_INTERVAL_MS = 5;
  static constexpr int BUF_SIZE = 256;
  static constexpr int CALIBRATION_CHECK_INTERVAL_MS = 5000;
  static constexpr int CALIBRATION_MAX_ATTEMPTS = 12;

  uint8_t _request[BUF_SIZE];
  uint8_t _response[BUF_SIZE];
  uint16_t _values[BUF_SIZE];
  bool _isCalibrating;

  // Single-shot mode state management
  bool _measurementTriggered;
  uint64_t _lastTriggerTime;
  MeasurementMode _currentMode; // Cache current measurement mode
  static constexpr uint32_t TRIGGER_COOLDOWN_MS = 3000;

  int _measurementPeriodSeconds;

  // Modbus function codes
  enum ModbusFunction {
    FUNC_READ_HOLDING_REGISTERS = 0x03,
    FUNC_READ_INPUT_REGISTERS = 0x04,
    FUNC_WRITE_MULTIPLE_REGISTERS = 0x10,
    FUNC_DEVICE_IDENTIFICATION_REGISTERS = 0x2B,
  };

  // Input Registers (Read-only)
  enum InputRegister {
    IR_ERROR_STATUS = 0x0000,
    IR_SPACE_CO2 = 0x0003,
  };

  // Holding Registers (Read/Write)
  enum HoldingRegister {
    HR_CALIBRATION_STATUS = 0x0000, // HR1
    HR_SPECIAL_COMMAND = 0x0001,    // HR2
    HR_MEASUREMENT_MODE = 0x000A,
    HR_MEASUREMENT_PERIOD = 0x000B,
    HR_MEASUREMENT_SAMPLES = 0x000C,
    HR_ABC_PERIOD = 0x000D,
    HR_METER_CONTROL = 0x0012,
    HR_START_SINGLE_MEASUREMENT = 0x0021, // HR34
  };

  // Special commands
  enum SpecialCommand {
    CMD_BACKGROUND_CALIBRATION = 0x7C06,
    CMD_FACTORY_CALIBRATION = 0x7C02,
  };

  // Calibration status flags (HR1)
  enum CalibrationStatus {
    CAL_STATUS_RESET = 0x0000,
    CAL_STATUS_BACKGROUND_DONE = 0x0020,
  };

  // Error status flags (IR1)
  enum ErrorStatus {
    ERR_FATAL = 0x0001,
    ERR_I2C = 0x0002,
    ERR_ALGORITHM = 0x0004,
    ERR_CALIBRATION = 0x0008,
    ERR_SELFDIAG = 0x0010,
    ERR_OUT_OF_RANGE = 0x0020,
    ERR_MEMORY = 0x0040,
    ERR_NO_MEASUREMENT = 0x0080,
  };

  // Modbus error codes
  enum ModbusError {
    ERR_COMMUNICATION = -1,
    ERR_ILLEGAL_FUNCTION = -2,
    ERR_ILLEGAL_DATA_ADDRESS = -3,
    ERR_ILLEGAL_DATA_VALUE = -4,
    ERR_SLAVE_FAILURE = -5,
  };

  static constexpr uint8_t MODBUS_ADDRESS = 0x68;

  /**
   * @brief Send Modbus command and read response
   * @param func Function code
   * @param reg Register address
   * @param numReg Number of registers
   * @return true if successful, false otherwise
   */
  bool _readRegisters(uint8_t func, uint16_t reg, uint16_t numReg);

  /**
   * @brief Write multiple registers
   * @param reg Register address
   * @param numReg Number of registers
   * @param values Values to write
   * @return true if successful, false otherwise
   */
  bool _writeRegisters(uint16_t reg, uint16_t numReg, const uint16_t *values);

  /**
   * @brief Read bytes from serial with timeout
   * @param maxBytes Maximum bytes to read
   * @param timeoutMs Timeout in milliseconds
   * @return Number of bytes read, -1 on timeout
   */
  int _readBytes(uint8_t maxBytes, uint32_t timeoutMs);

  bool _readDeviceIdentification(uint8_t objectId, char *out, size_t outLen);

  /**
   * @brief Wait for complete Modbus response with inter-packet detection
   * @param expectedBytes Expected number of bytes
   * @param timeoutMs Timeout in milliseconds (default: TIMEOUT_MS)
   * @return Number of bytes received, -1 on timeout
   */
  int _waitForResponse(int expectedBytes, uint32_t timeoutMs = TIMEOUT_MS);

  /**
   * @brief Validate Modbus response
   * @param func Expected function code
   * @param numBytes Number of bytes received
   * @return true if response valid, false otherwise
   */
  bool _validateResponse(uint8_t func, uint8_t numBytes);

  /**
   * @brief Get error status register
   * @return Error status flags, 0 if no error, -1 on read failure
   */
  int _getErrorStatus();

  /**
   * @brief Clear calibration status
   * @return true if successful, false otherwise
   */
  bool _clearCalibrationStatus();

  /**
   * @brief Get calibration status
   * @return Calibration status flags, -1 on error
   */
  int _getCalibrationStatus();
};

#endif // SUNLIGHT_HPP
