// SPDX-License-Identifier: MIT

#ifndef AIRGRADIENT_GO_MAIN_BLE_STREAM_H
#define AIRGRADIENT_GO_MAIN_BLE_STREAM_H

#include <stddef.h>
#include <stdint.h>

#include <atomic>
#include <string>

#include "esp_err.h"

class NimBLEServer;
class NimBLEService;
class NimBLECharacteristic;
class BLEStreamCharCallbacks;
class BLEStreamServerCallbacks;

// BLEStream: minimal BLE GATT server with two notify characteristics.
// - measures: JSON object (single sample)
// - status: JSON object (device/status)
class BLEStream {
 public:
  BLEStream() = default;
  ~BLEStream();

  BLEStream(const BLEStream&) = delete;
  BLEStream& operator=(const BLEStream&) = delete;

  esp_err_t start(const char* device_name);
  void stop();

  bool is_running() const { return running_.load(); }

  bool measures_subscribed() const { return measures_subscribed_.load(); }
  bool status_subscribed() const { return status_subscribed_.load(); }

  void notify_measures(const std::string& json);
  void notify_status(const std::string& json);

 private:
  friend class BLEStreamCharCallbacks;
  friend class BLEStreamServerCallbacks;
  void set_measures_subscribed_(bool v);
  void set_status_subscribed_(bool v);

  std::atomic<bool> running_{false};
  std::atomic<bool> measures_subscribed_{false};
  std::atomic<bool> status_subscribed_{false};

  NimBLEServer* server_ = nullptr;
  NimBLEService* service_ = nullptr;
  NimBLECharacteristic* measures_char_ = nullptr;
  NimBLECharacteristic* status_char_ = nullptr;

  BLEStreamCharCallbacks* measures_cb_ = nullptr;
  BLEStreamCharCallbacks* status_cb_ = nullptr;

  BLEStreamServerCallbacks* server_cb_ = nullptr;
};

#endif // AIRGRADIENT_GO_MAIN_BLE_STREAM_H
