/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef PMS5003_BASE_HPP
#define PMS5003_BASE_HPP

#include "AirgradientSerial.hpp"
#include "MeasuresTypes.h"
#include "PMSensor.hpp"
#include <stdint.h>

/**
 * @brief Base class for PMS5003 and PMS5003T sensors
 *
 * Handles common protocol operations:
 * - Frame parsing with checksum validation
 * - Sleep/wake commands
 * - Active/passive mode switching
 * - Serial communication via AirgradientSerial
 *
 * Derived classes override parsing logic for specific sensor variants.
 */
class PMS5003Base : public PMSensor {
public:
  /**
   * @brief Construct PMS5003 base with serial interface
   * @param serial Reference to initialized AirgradientSerial instance
   */
  explicit PMS5003Base(AirgradientSerial &serial);
  virtual ~PMS5003Base() = default;

  /**
   * @brief Initialize sensor communication
   * @return true if initialization successful, false otherwise
   */
  bool init();

  /**
   * @brief Enter sleep mode for low power consumption
   */
  void sleep();

  /**
   * @brief Wake up from sleep mode
   * @note Stable data requires ~30 seconds after wakeup
   */
  void wakeUp();

  /**
   * @brief Set active mode (sensor sends data automatically)
   */
  void activeMode();

  /**
   * @brief Set passive mode (sensor sends data only on request)
   */
  void passiveMode();

  /**
   * @brief Check if sensor is connected
   * @return true if sensor responds to requests, false otherwise
   */
  bool isConnected();

protected:
  /**
   * @brief Operating mode enumeration
   */
  enum class Mode { ACTIVE, PASSIVE };

  /**
   * @brief Raw payload buffer for frame data
   */
  static constexpr uint8_t PAYLOAD_SIZE = 30;
  uint8_t _payload[PAYLOAD_SIZE];

  /**
   * @brief Clear receive buffer
   */
  void clearBuffer();

  /**
   * @brief Request data in passive mode
   */
  void requestRead();

  /**
   * @brief Read frame with timeout
   * @param timeoutMs Timeout in milliseconds
   * @return true if valid frame received, false on timeout or error
   */
  bool readFrame(uint16_t timeoutMs);

  /**
   * @brief Convert two bytes to uint16_t (big-endian)
   * @param high High byte
   * @param low Low byte
   * @return 16-bit value
   */
  uint16_t _makeWord(uint8_t high, uint8_t low) const;

private:
  const char *const TAG = "PMS5003Base";
  AirgradientSerial &serial_;
  Mode _mode;

  uint8_t _index;
  uint16_t _frameLen;
  uint16_t _checksum;
  uint16_t _calculatedChecksum;

  /**
   * @brief Process one byte from serial stream
   * @return true if complete valid frame received, false otherwise
   */
  bool _processByte();
};

#endif // PMS5003_BASE_HPP
