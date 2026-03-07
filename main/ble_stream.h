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
class BLEStreamConfigCallbacks;
class BLEStreamHistoryCallbacks;

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

  // BLE liveness supervisor; call periodically from main loop.
  void tick(uint32_t now_ms);

  bool is_running() const { return running_.load(); }

  bool measures_subscribed() const { return measures_subscribed_.load(); }
  bool status_subscribed() const { return status_subscribed_.load(); }
  bool history_subscribed() const { return history_subscribed_.load(); }

  void notify_measures(const std::string& json);
  void notify_status(const std::string& json);
  bool notify_history(const std::string& json);
  bool take_pending_tracking_sleep_interval_s(uint32_t* out);
  bool take_pending_co2_force_calib(uint16_t* out_ppm);
  bool take_pending_tracking(bool* out_enabled);
  bool take_pending_flash_erase();
  bool take_pending_history_start();

 private:
  friend class BLEStreamCharCallbacks;
  friend class BLEStreamServerCallbacks;
  friend class BLEStreamConfigCallbacks;
  friend class BLEStreamHistoryCallbacks;
  void set_measures_subscribed_(bool v);
  void set_status_subscribed_(bool v);
  void set_history_subscribed_(bool v);

  void request_tracking_sleep_interval_s_(uint32_t s);
  void request_co2_force_calib_(uint16_t ppm);
  void request_tracking_(bool enabled);
  void request_flash_erase_();
  void request_history_start_();

  std::atomic<bool> running_{false};
  std::atomic<bool> measures_subscribed_{false};
  std::atomic<bool> status_subscribed_{false};
  std::atomic<bool> history_subscribed_{false};

  std::atomic<bool> pending_tracking_sleep_interval_{false};
  std::atomic<uint32_t> pending_tracking_sleep_interval_s_{0};

  std::atomic<bool> pending_co2_force_calib_{false};
  std::atomic<uint16_t> pending_co2_force_calib_ppm_{0};

  std::atomic<bool> pending_tracking_{false};
  std::atomic<bool> pending_tracking_enabled_{false};

  std::atomic<bool> pending_flash_erase_{false};

  std::atomic<bool> pending_history_start_{false};

  NimBLEServer* server_ = nullptr;
  NimBLEService* service_ = nullptr;
  NimBLECharacteristic* measures_char_ = nullptr;
  NimBLECharacteristic* status_char_ = nullptr;
  NimBLECharacteristic* config_char_ = nullptr;
  NimBLECharacteristic* history_char_ = nullptr;

  std::string last_device_name_;
  uint32_t last_health_check_ms_ = 0;
  uint8_t adv_restart_fail_streak_ = 0;
  uint32_t last_adv_restart_ms_ = 0;
  uint8_t ble_restart_attempts_ = 0;
  uint32_t last_ble_restart_ms_ = 0;

  // Notify failure supervisor (measures + status only).
  uint8_t measures_notify_fail_streak_ = 0;
  uint8_t status_notify_fail_streak_ = 0;
  uint8_t measures_notify_cooldowns_left_ = 0;
  uint8_t status_notify_cooldowns_left_ = 0;
  uint32_t measures_notify_suppress_until_ms_ = 0;
  uint32_t status_notify_suppress_until_ms_ = 0;
  std::atomic<bool> restart_due_to_notify_{false};

  BLEStreamCharCallbacks* measures_cb_ = nullptr;
  BLEStreamCharCallbacks* status_cb_ = nullptr;

  BLEStreamHistoryCallbacks* history_cb_ = nullptr;

  BLEStreamConfigCallbacks* config_cb_ = nullptr;

  BLEStreamServerCallbacks* server_cb_ = nullptr;
};

#endif // AIRGRADIENT_GO_MAIN_BLE_STREAM_H
