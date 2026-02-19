/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "PMS5003Base.hpp"
#include "RTOS.h"
#include "esp_log.h"
#include <cstring>

PMS5003Base::PMS5003Base(AirgradientSerial &serial)
    : serial_(serial), _mode(Mode::ACTIVE), _index(0), _frameLen(0), _checksum(0),
      _calculatedChecksum(0) {
  memset(_payload, 0, PAYLOAD_SIZE);
}

bool PMS5003Base::init() {
  // Serial should already be initialized by caller
  // Just verify we can communicate
  passiveMode();
  RTOS::delay_ms(100);
  return isConnected();
}

void PMS5003Base::sleep() {
  uint8_t command[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
  serial_.write(command, sizeof(command));
}

void PMS5003Base::wakeUp() {
  uint8_t command[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
  serial_.write(command, sizeof(command));
}

void PMS5003Base::activeMode() {
  uint8_t command[] = {0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};
  serial_.write(command, sizeof(command));
  _mode = Mode::ACTIVE;
}

void PMS5003Base::passiveMode() {
  uint8_t command[] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
  serial_.write(command, sizeof(command));
  _mode = Mode::PASSIVE;
}

bool PMS5003Base::isConnected() {
  for (int i = 0; i < 3; i++) {
    clearBuffer();
    requestRead();
    if (readFrame(1000)) {
      return true;
    }
    RTOS::delay_ms(200);
  }
  return false;
}

void PMS5003Base::clearBuffer() {
  int bytesCleared = 0;
  while (serial_.available()) {
    serial_.read();
    bytesCleared++;
  }
  ESP_LOGD(TAG, "Cleared %d byte(s)", bytesCleared);
}

void PMS5003Base::requestRead() {
  if (_mode == Mode::PASSIVE) {
    uint8_t command[] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};
    serial_.write(command, sizeof(command));
  }
}

bool PMS5003Base::readFrame(uint16_t timeoutMs) {
  uint64_t start = RTOS::get_time_ms();
  bool frameReady = false;

  _index = 0; // Reset state machine

  do {
    if (_processByte()) {
      frameReady = true;
      break;
    }
  } while ((RTOS::get_time_ms() - start) < timeoutMs);

  return frameReady;
}

bool PMS5003Base::_processByte() {
  if (!serial_.available()) {
    return false;
  }

  uint8_t ch = serial_.read();
  ESP_LOGV(TAG, "%.2x", ch);

  switch (_index) {
  case 0:
    // Fixed start character 0x42
    if (ch != 0x42) {
      return false;
    }
    _calculatedChecksum = ch;
    break;

  case 1:
    // Fixed second character 0x4D
    if (ch != 0x4D) {
      _index = 0;
      return false;
    }
    _calculatedChecksum += ch;
    break;

  case 2:
    // High byte of frame length
    _calculatedChecksum += ch;
    _frameLen = ch << 8;
    break;

  case 3:
    // Low byte of frame length
    _frameLen |= ch;
    // Valid frame lengths: 2*9+2=20, 2*13+2=28, 2*17+2=36
    if (_frameLen != 20 && _frameLen != 28 && _frameLen != 36) {
      _index = 0;
      return false;
    }
    _calculatedChecksum += ch;
    break;

  default:
    // Checksum high byte
    if (_index == _frameLen + 2) {
      _checksum = ch << 8;
    }
    // Checksum low byte
    else if (_index == _frameLen + 2 + 1) {
      _checksum |= ch;

      // Validate checksum
      if (_calculatedChecksum == _checksum) {
        _index = 0;
        return true; // Valid frame complete
      }

      _index = 0;
      return false;
    }
    // Payload byte
    else {
      _calculatedChecksum += ch;
      uint8_t payloadIndex = _index - 4;

      if (payloadIndex < PAYLOAD_SIZE) {
        _payload[payloadIndex] = ch;
      }
    }
    break;
  }

  _index++;
  return false;
}

uint16_t PMS5003Base::_makeWord(uint8_t high, uint8_t low) const {
  return (uint16_t)((high << 8) | low);
}
