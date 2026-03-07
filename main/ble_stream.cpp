// SPDX-License-Identifier: MIT

#include "ble_stream.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

// esp-nimble-cpp
#include "NimBLEDevice.h"

#include "cJSON.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include <vector>

static const char* const TAG = "BLEStream";

static constexpr const char* SERVICE_UUID = "d1c0c0a0-6b48-4b2a-9b1d-59f9f2b0a1e1";
static constexpr const char* MEASURES_CHAR_UUID = "d1c0c0a1-6b48-4b2a-9b1d-59f9f2b0a1e1";
static constexpr const char* STATUS_CHAR_UUID = "d1c0c0a2-6b48-4b2a-9b1d-59f9f2b0a1e1";
static constexpr const char* CONFIG_CHAR_UUID = "d1c0c0a3-6b48-4b2a-9b1d-59f9f2b0a1e1";
static constexpr const char* HISTORY_CHAR_UUID = "d1c0c0a4-6b48-4b2a-9b1d-59f9f2b0a1e1";

static constexpr uint32_t TRACKING_SLEEP_MIN_S = 1;
static constexpr uint32_t TRACKING_SLEEP_MAX_S = 86400;

static inline uint32_t ble_now_ms_(void) {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

bool BLEStream::ble_restart_window_should_esp_restart_(uint32_t now_ms, const char* reason) {
  static constexpr uint32_t WINDOW_MS = 30U * 60U * 1000U;

  if (ble_restart_window_start_ms_ == 0 || (now_ms - ble_restart_window_start_ms_) > WINDOW_MS) {
    ble_restart_window_start_ms_ = now_ms;
    ble_restart_window_count_ = 0;
  }

  if (ble_restart_window_count_ < 255) {
    ble_restart_window_count_++;
  }

  ESP_LOGW(TAG, "ble restart window: count=%u window_ms=%" PRIu32 " reason=%s",
           (unsigned)ble_restart_window_count_, (uint32_t)(now_ms - ble_restart_window_start_ms_),
           (reason != nullptr) ? reason : "");

  if (ble_restart_window_count_ >= 3) {
    ESP_LOGE(TAG, "ble restart window exceeded; esp_restart now");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return true;
  }

  return false;
}

static bool start_advertising_checked_(const char* ctx) {
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  if (adv == nullptr) {
    ESP_LOGW(TAG, "advertising start(%s) failed: getAdvertising=null", (ctx != nullptr) ? ctx : "");
    return false;
  }

  const bool ok = adv->start();
  const bool active = adv->isAdvertising();
  ESP_LOGI(TAG, "advertising start(%s): ok=%d active=%d", (ctx != nullptr) ? ctx : "", (int)ok,
           (int)active);
  return ok && active;
}

class BLEStreamCharCallbacks : public NimBLECharacteristicCallbacks {
 public:
  enum class Kind { Measures, Status };
  BLEStreamCharCallbacks(BLEStream* s, Kind k) : s_(s), k_(k) {}

  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo,
                   uint16_t subValue) override {
    (void)pCharacteristic;
    (void)connInfo;

    const bool notify_enabled = (subValue & 0x0001) != 0;
    if (s_ == nullptr) {
      return;
    }
    if (k_ == Kind::Measures) {
      s_->set_measures_subscribed_(notify_enabled);
      ESP_LOGI(TAG, "measures subscribe=%d", (int)notify_enabled);
    } else {
      s_->set_status_subscribed_(notify_enabled);
      ESP_LOGI(TAG, "status subscribe=%d", (int)notify_enabled);
    }
  }

 private:
  BLEStream* s_ = nullptr;
  Kind k_;
};

class BLEStreamServerCallbacks : public NimBLEServerCallbacks {
 public:
  explicit BLEStreamServerCallbacks(BLEStream* s) : s_(s) {}

  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    const uint16_t itvl = connInfo.getConnInterval();
    const uint16_t to = connInfo.getConnTimeout();

    printf("Interval: %d\n", (int)connInfo.getConnInterval());
    printf("Latency: %d\n", (int)connInfo.getConnLatency());
    printf("Timeout: %d\n", (int)connInfo.getConnTimeout());
    printf("Interval_us: %" PRIu32 "\n", (uint32_t)itvl * 1250U);
    printf("Timeout_ms: %" PRIu32 "\n", (uint32_t)to * 10U);

    if (pServer != nullptr) {
      // Request faster connection params for notification throughput.
      // Units: interval in 1.25ms, timeout in 10ms.
      static constexpr uint16_t MIN_ITVL = 12;   // 15ms
      static constexpr uint16_t MAX_ITVL = 24;   // 30ms
      static constexpr uint16_t LATENCY = 0;
      // Keep the currently negotiated supervision timeout (iOS may ignore changes).
      const uint16_t TIMEOUT = to;
      printf("Requested minInterval=%u maxInterval=%u latency=%u timeout=%u\n",
             (unsigned)MIN_ITVL, (unsigned)MAX_ITVL, (unsigned)LATENCY, (unsigned)TIMEOUT);
      pServer->updateConnParams(connInfo.getConnHandle(), MIN_ITVL, MAX_ITVL, LATENCY, TIMEOUT);
    }

    // Reset notify failure supervisor state for the new connection.
    if (s_ != nullptr) {
      s_->measures_notify_fail_streak_ = 0;
      s_->status_notify_fail_streak_ = 0;
      s_->measures_notify_cooldowns_left_ = 0;
      s_->status_notify_cooldowns_left_ = 0;
      s_->measures_notify_suppress_until_ms_ = 0;
      s_->status_notify_suppress_until_ms_ = 0;
      s_->restart_due_to_notify_.store(false, std::memory_order_relaxed);
    }

    // Enforce single connection for stability: stop advertising once connected.
    const bool adv_stopped = NimBLEDevice::stopAdvertising();
    ESP_LOGI(TAG, "advertising stop(connect): ok=%d", (int)adv_stopped);
  }

  void onConnParamsUpdate(NimBLEConnInfo& connInfo) override {
    const uint16_t itvl = connInfo.getConnInterval();
    const uint16_t lat = connInfo.getConnLatency();
    const uint16_t to = connInfo.getConnTimeout();

    printf("ConnParamsUpdate Interval: %u\n", (unsigned)itvl);
    printf("ConnParamsUpdate Latency: %u\n", (unsigned)lat);
    printf("ConnParamsUpdate Timeout: %u\n", (unsigned)to);
    printf("ConnParamsUpdate Interval_us: %" PRIu32 "\n", (uint32_t)itvl * 1250U);
    printf("ConnParamsUpdate Timeout_ms: %" PRIu32 "\n", (uint32_t)to * 10U);
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer;
    (void)connInfo;
    (void)reason;

    if (s_ == nullptr) {
      return;
    }
    s_->set_measures_subscribed_(false);
    s_->set_status_subscribed_(false);
    s_->set_history_subscribed_(false);

    // Belt-and-suspenders: ensure advertising restarts.
    (void)start_advertising_checked_("disconnect");

    if (s_ != nullptr) {
      s_->measures_notify_fail_streak_ = 0;
      s_->status_notify_fail_streak_ = 0;
      s_->measures_notify_cooldowns_left_ = 0;
      s_->status_notify_cooldowns_left_ = 0;
      s_->measures_notify_suppress_until_ms_ = 0;
      s_->status_notify_suppress_until_ms_ = 0;
      s_->restart_due_to_notify_.store(false, std::memory_order_relaxed);
    }
  }

 private:
  BLEStream* s_ = nullptr;
};

class BLEStreamHistoryCallbacks : public NimBLECharacteristicCallbacks {
 public:
  explicit BLEStreamHistoryCallbacks(BLEStream* s) : s_(s) {}

  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo,
                   uint16_t subValue) override {
    (void)pCharacteristic;
    (void)connInfo;

    const bool notify_enabled = (subValue & 0x0001) != 0;
    if (s_ == nullptr) {
      return;
    }
    s_->set_history_subscribed_(notify_enabled);
    ESP_LOGI(TAG, "history subscribe=%d", (int)notify_enabled);
  }

  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    if (s_ == nullptr || pCharacteristic == nullptr) {
      return;
    }

    const NimBLEAttValue v = pCharacteristic->getValue();
    if (v.size() == 0) {
      return;
    }

    // Accept either byte 0x01 or ASCII "1".
    if (v.size() == 1) {
      const uint8_t b = (uint8_t)v.data()[0];
      if (b == 0x01 || b == (uint8_t)'1') {
        s_->request_history_start_();
        ESP_LOGI(TAG, "history start requested");
      }
    }
  }

 private:
  BLEStream* s_ = nullptr;
};

class BLEStreamConfigCallbacks : public NimBLECharacteristicCallbacks {
 public:
  explicit BLEStreamConfigCallbacks(BLEStream* s) : s_(s) {}

  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    if (s_ == nullptr || pCharacteristic == nullptr) {
      return;
    }

    const NimBLEAttValue v = pCharacteristic->getValue();
    if (v.size() > 0 && v.c_str() != nullptr) {
      cJSON* root = cJSON_ParseWithLength(v.c_str(), v.size());
      if (cJSON_IsObject(root)) {
        const cJSON* t = cJSON_GetObjectItemCaseSensitive(root, "trackingSleepS");
        if (cJSON_IsNumber(t)) {
          const double dv = t->valuedouble;
          const uint32_t nv = (uint32_t)llround(dv);
          if ((double)nv == dv && nv >= TRACKING_SLEEP_MIN_S && nv <= TRACKING_SLEEP_MAX_S) {
            s_->request_tracking_sleep_interval_s_(nv);
            ESP_LOGI(TAG, "config trackingSleepS=%" PRIu32, nv);
          }
        }

        const cJSON* c = cJSON_GetObjectItemCaseSensitive(root, "co2ForceCalib");
        if (cJSON_IsNumber(c)) {
          const double dv = c->valuedouble;
          const uint32_t nv = (uint32_t)llround(dv);
          if ((double)nv == dv && nv >= 1 && nv <= 32000) {
            s_->request_co2_force_calib_((uint16_t)nv);
            ESP_LOGI(TAG, "config co2ForceCalib=%" PRIu32, nv);
          }
        }

        const cJSON* tr = cJSON_GetObjectItemCaseSensitive(root, "tracking");
        if (cJSON_IsBool(tr)) {
          const bool enabled = cJSON_IsTrue(tr);
          s_->request_tracking_(enabled);
          ESP_LOGI(TAG, "config tracking=%d", (int)enabled);
        } else if (cJSON_IsNumber(tr)) {
          const double dv = tr->valuedouble;
          const uint32_t nv = (uint32_t)llround(dv);
          if ((double)nv == dv && (nv == 0 || nv == 1)) {
            const bool enabled = (nv == 1);
            s_->request_tracking_(enabled);
            ESP_LOGI(TAG, "config tracking=%d", (int)enabled);
          }
        }

        const cJSON* fe = cJSON_GetObjectItemCaseSensitive(root, "flashErase");
        if (cJSON_IsBool(fe)) {
          if (cJSON_IsTrue(fe)) {
            s_->request_flash_erase_();
            ESP_LOGI(TAG, "config flashErase=1");
          }
        } else if (cJSON_IsNumber(fe)) {
          const double dv = fe->valuedouble;
          const uint32_t nv = (uint32_t)llround(dv);
          if ((double)nv == dv && nv == 1) {
            s_->request_flash_erase_();
            ESP_LOGI(TAG, "config flashErase=1");
          }
        }
      }
      cJSON_Delete(root);
    }

    // Write-only characteristic: nothing to echo.
  }

 private:
  BLEStream* s_ = nullptr;
};

BLEStream::~BLEStream() {
  stop();
}

void BLEStream::set_measures_subscribed_(bool v) {
  measures_subscribed_.store(v);
}

void BLEStream::set_status_subscribed_(bool v) {
  status_subscribed_.store(v);
}

void BLEStream::set_history_subscribed_(bool v) {
  history_subscribed_.store(v);
}

esp_err_t BLEStream::start(const char* device_name) {
  if (running_.load()) {
    return ESP_OK;
  }
  if (device_name == nullptr || device_name[0] == '\0') {
    device_name = "AirGradientGo";
  }

  last_device_name_.assign(device_name);

  measures_subscribed_.store(false);
  status_subscribed_.store(false);
  history_subscribed_.store(false);

  pending_history_start_.store(false);
  pending_tracking_.store(false);
  pending_flash_erase_.store(false);

  // Reset advertising health state for this session.
  last_health_check_ms_ = 0;
  adv_restart_fail_streak_ = 0;
  last_adv_restart_ms_ = 0;

  // Reset notify failure supervisor state.
  measures_notify_fail_streak_ = 0;
  status_notify_fail_streak_ = 0;
  measures_notify_cooldowns_left_ = 0;
  status_notify_cooldowns_left_ = 0;
  measures_notify_suppress_until_ms_ = 0;
  status_notify_suppress_until_ms_ = 0;
  restart_due_to_notify_.store(false, std::memory_order_relaxed);

  if (!NimBLEDevice::init(std::string(device_name))) {
    ESP_LOGW(TAG, "NimBLEDevice::init failed");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "BLE device name: %s", device_name);

  // Best-effort: prefer large MTU for JSON notifications.
  (void)NimBLEDevice::setMTU(256);

  server_ = NimBLEDevice::createServer();
  if (server_ == nullptr) {
    ESP_LOGW(TAG, "createServer failed");
    (void)NimBLEDevice::deinit(true);
    return ESP_FAIL;
  }

  if (server_cb_ == nullptr) {
    server_cb_ = new BLEStreamServerCallbacks(this);
  }
  server_->setCallbacks(server_cb_, false);
  server_->advertiseOnDisconnect(true);

  service_ = server_->createService(SERVICE_UUID);
  if (service_ == nullptr) {
    ESP_LOGW(TAG, "createService failed");
    (void)NimBLEDevice::deinit(true);
    server_ = nullptr;
    return ESP_FAIL;
  }

  measures_char_ = service_->createCharacteristic(MEASURES_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  status_char_ = service_->createCharacteristic(STATUS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  config_char_ = service_->createCharacteristic(
      CONFIG_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR, 128);
  history_char_ = service_->createCharacteristic(
      HISTORY_CHAR_UUID,
      NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
      512);
  if (measures_char_ == nullptr || status_char_ == nullptr || config_char_ == nullptr || history_char_ == nullptr) {
    ESP_LOGW(TAG, "createCharacteristic failed");
    (void)NimBLEDevice::deinit(true);
    server_ = nullptr;
    service_ = nullptr;
    measures_char_ = nullptr;
    status_char_ = nullptr;
    config_char_ = nullptr;
    history_char_ = nullptr;
    return ESP_FAIL;
  }

  if (measures_cb_ == nullptr) {
    measures_cb_ = new BLEStreamCharCallbacks(this, BLEStreamCharCallbacks::Kind::Measures);
  }
  if (status_cb_ == nullptr) {
    status_cb_ = new BLEStreamCharCallbacks(this, BLEStreamCharCallbacks::Kind::Status);
  }
  measures_char_->setCallbacks(measures_cb_);
  status_char_->setCallbacks(status_cb_);

  if (history_cb_ == nullptr) {
    history_cb_ = new BLEStreamHistoryCallbacks(this);
  }
  history_char_->setCallbacks(history_cb_);

  if (config_cb_ == nullptr) {
    config_cb_ = new BLEStreamConfigCallbacks(this);
  }
  config_char_->setCallbacks(config_cb_);

  service_->start();
  server_->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  if (adv != nullptr) {
    (void)adv->reset();
    adv->addServiceUUID(SERVICE_UUID);
    adv->enableScanResponse(true);
    (void)adv->setName(std::string(device_name));
    (void)start_advertising_checked_("start");
  } else {
    ESP_LOGW(TAG, "getAdvertising returned null");
  }

  running_.store(true);
  ESP_LOGI(TAG, "started name=%s", device_name);
  return ESP_OK;
}

void BLEStream::stop() {
  if (!running_.load()) {
    measures_subscribed_.store(false);
    status_subscribed_.store(false);
    return;
  }

  measures_subscribed_.store(false);
  status_subscribed_.store(false);
  history_subscribed_.store(false);

  pending_history_start_.store(false);
  pending_tracking_.store(false);
  pending_flash_erase_.store(false);

  // Stop advertising and disconnect peers best-effort.
  (void)NimBLEDevice::stopAdvertising();
  if (server_ != nullptr) {
    const std::vector<uint16_t> peers = server_->getPeerDevices();
    for (uint16_t h : peers) {
      (void)server_->disconnect(h);
    }
  }

  (void)NimBLEDevice::deinit(true);

  server_ = nullptr;
  service_ = nullptr;
  measures_char_ = nullptr;
  status_char_ = nullptr;
  config_char_ = nullptr;
  history_char_ = nullptr;
  running_.store(false);
  measures_notify_fail_streak_ = 0;
  status_notify_fail_streak_ = 0;
  measures_notify_cooldowns_left_ = 0;
  status_notify_cooldowns_left_ = 0;
  measures_notify_suppress_until_ms_ = 0;
  status_notify_suppress_until_ms_ = 0;
  restart_due_to_notify_.store(false, std::memory_order_relaxed);
  ESP_LOGI(TAG, "stopped");
}

void BLEStream::tick(uint32_t now_ms) {
  if (!running_.load()) {
    return;
  }

  // Notify escalation: restart BLE stack (rate-limited).
  if (restart_due_to_notify_.load(std::memory_order_relaxed)) {
    static constexpr uint32_t BLE_RESTART_BACKOFF_MS = 60000;
    if (last_ble_restart_ms_ == 0 || (now_ms - last_ble_restart_ms_) >= BLE_RESTART_BACKOFF_MS) {
      last_ble_restart_ms_ = now_ms;
      if (ble_restart_attempts_ < 255) {
        ble_restart_attempts_++;
      }
      if (last_device_name_.empty()) {
        last_device_name_.assign("AirGradientGo");
      }

      ESP_LOGW(TAG, "notify: restarting BLE stack attempt=%u", (unsigned)ble_restart_attempts_);
      restart_due_to_notify_.store(false, std::memory_order_relaxed);

      (void)ble_restart_window_should_esp_restart_(now_ms, "notify");
      stop();
      (void)start(last_device_name_.c_str());
    }
    return;
  }

  static constexpr uint32_t HEALTH_INTERVAL_MS = 10000;
  if (last_health_check_ms_ != 0 && (now_ms - last_health_check_ms_) < HEALTH_INTERVAL_MS) {
    return;
  }
  last_health_check_ms_ = now_ms;

  bool need_restart = false;

  if (!NimBLEDevice::isInitialized() || server_ == nullptr) {
    ESP_LOGW(TAG, "health: NimBLE not initialized or server null");
    need_restart = true;
  } else {
    const uint8_t connected = server_->getConnectedCount();
    if (connected != 0) {
      // Connected: advertising should already be stopped for stability.
      return;
    }

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    if (adv == nullptr) {
      ESP_LOGW(TAG, "health: getAdvertising null");
      need_restart = true;
    } else if (adv->isAdvertising()) {
      adv_restart_fail_streak_ = 0;
      return;
    } else {
      // Not connected and not advertising: attempt restart with backoff.
      static constexpr uint32_t ADV_RESTART_BACKOFF_MS = 5000;
      if (last_adv_restart_ms_ == 0 || (now_ms - last_adv_restart_ms_) >= ADV_RESTART_BACKOFF_MS) {
        last_adv_restart_ms_ = now_ms;

        const bool ok = start_advertising_checked_("health");
        if (ok) {
          adv_restart_fail_streak_ = 0;
          return;
        }
      }

      if (adv_restart_fail_streak_ < 255) {
        adv_restart_fail_streak_++;
      }
      ESP_LOGW(TAG, "health: advertising restart failed streak=%u", (unsigned)adv_restart_fail_streak_);
      need_restart = (adv_restart_fail_streak_ >= 3);
    }
  }

  if (!need_restart) {
    return;
  }

  static constexpr uint32_t BLE_RESTART_BACKOFF_MS = 60000;
  if (last_ble_restart_ms_ != 0 && (now_ms - last_ble_restart_ms_) < BLE_RESTART_BACKOFF_MS) {
    return;
  }
  last_ble_restart_ms_ = now_ms;
  if (ble_restart_attempts_ < 255) {
    ble_restart_attempts_++;
  }

  if (last_device_name_.empty()) {
    last_device_name_.assign("AirGradientGo");
  }

  ESP_LOGW(TAG, "health: restarting BLE stack attempt=%u", (unsigned)ble_restart_attempts_);
  (void)ble_restart_window_should_esp_restart_(now_ms, "health");
  stop();
  (void)start(last_device_name_.c_str());
}

void BLEStream::request_tracking_sleep_interval_s_(uint32_t s) {
  pending_tracking_sleep_interval_s_.store(s, std::memory_order_relaxed);
  pending_tracking_sleep_interval_.store(true, std::memory_order_relaxed);
}

void BLEStream::request_co2_force_calib_(uint16_t ppm) {
  pending_co2_force_calib_ppm_.store(ppm, std::memory_order_relaxed);
  pending_co2_force_calib_.store(true, std::memory_order_relaxed);
}

void BLEStream::request_tracking_(bool enabled) {
  pending_tracking_enabled_.store(enabled, std::memory_order_relaxed);
  pending_tracking_.store(true, std::memory_order_relaxed);
}

void BLEStream::request_flash_erase_() {
  pending_flash_erase_.store(true, std::memory_order_relaxed);
}

void BLEStream::request_history_start_() {
  pending_history_start_.store(true, std::memory_order_relaxed);
}


bool BLEStream::take_pending_tracking_sleep_interval_s(uint32_t* out) {
  if (out == nullptr) {
    return false;
  }
  const bool had = pending_tracking_sleep_interval_.exchange(false, std::memory_order_relaxed);
  if (!had) {
    return false;
  }
  *out = pending_tracking_sleep_interval_s_.load(std::memory_order_relaxed);
  return true;
}

bool BLEStream::take_pending_co2_force_calib(uint16_t* out_ppm) {
  if (out_ppm == nullptr) {
    return false;
  }
  const bool had = pending_co2_force_calib_.exchange(false, std::memory_order_relaxed);
  if (!had) {
    return false;
  }
  *out_ppm = pending_co2_force_calib_ppm_.load(std::memory_order_relaxed);
  return true;
}

bool BLEStream::take_pending_tracking(bool* out_enabled) {
  if (out_enabled == nullptr) {
    return false;
  }
  const bool had = pending_tracking_.exchange(false, std::memory_order_relaxed);
  if (!had) {
    return false;
  }
  *out_enabled = pending_tracking_enabled_.load(std::memory_order_relaxed);
  return true;
}

bool BLEStream::take_pending_flash_erase() {
  return pending_flash_erase_.exchange(false, std::memory_order_relaxed);
}

bool BLEStream::take_pending_history_start() {
  return pending_history_start_.exchange(false, std::memory_order_relaxed);
}

void BLEStream::notify_measures(const std::string& json) {
  if (!running_.load() || !measures_subscribed_.load() || measures_char_ == nullptr) {
    return;
  }
  if (json.empty()) {
    return;
  }

  if (restart_due_to_notify_.load(std::memory_order_relaxed)) {
    return;
  }
  if (server_ == nullptr || server_->getConnectedCount() == 0) {
    return;
  }

  static constexpr uint32_t COOLDOWN_MS = 20000;
  static constexpr uint8_t FAIL_STREAK_TRIGGER = 3;
  static constexpr uint8_t COOLDOWN_CYCLES = 3;

  const uint32_t now = ble_now_ms_();
  if (measures_notify_suppress_until_ms_ != 0 && (int32_t)(now - measures_notify_suppress_until_ms_) < 0) {
    return;
  }

  const bool ok = measures_char_->notify((const uint8_t*)json.data(), json.size());
  if (!ok) {
    if (measures_notify_cooldowns_left_ > 0) {
      measures_notify_cooldowns_left_--;
      if (measures_notify_cooldowns_left_ == 0) {
        ESP_LOGW(TAG, "measures notify failed after cooldowns; requesting BLE restart");
        restart_due_to_notify_.store(true, std::memory_order_relaxed);
      } else {
        measures_notify_suppress_until_ms_ = now + COOLDOWN_MS;
        ESP_LOGW(TAG, "measures notify still failing; cooldown %ums cycles_left=%u", (unsigned)COOLDOWN_MS,
                 (unsigned)measures_notify_cooldowns_left_);
      }
      return;
    }

    if (measures_notify_fail_streak_ < 255) {
      measures_notify_fail_streak_++;
    }
    if (measures_notify_fail_streak_ >= FAIL_STREAK_TRIGGER) {
      measures_notify_fail_streak_ = 0;
      measures_notify_cooldowns_left_ = COOLDOWN_CYCLES;
      measures_notify_suppress_until_ms_ = now + COOLDOWN_MS;
      ESP_LOGW(TAG, "measures notify failed %u times; cooldown %ums cycles=%u", (unsigned)FAIL_STREAK_TRIGGER,
               (unsigned)COOLDOWN_MS, (unsigned)COOLDOWN_CYCLES);
    } else if (measures_notify_fail_streak_ == 1) {
      ESP_LOGW(TAG, "measures notify failed (len=%u)", (unsigned)json.size());
    }
    return;
  }

  measures_notify_fail_streak_ = 0;
  measures_notify_cooldowns_left_ = 0;
  measures_notify_suppress_until_ms_ = 0;
}

void BLEStream::notify_status(const std::string& json) {
  if (!running_.load() || !status_subscribed_.load() || status_char_ == nullptr) {
    return;
  }
  if (json.empty()) {
    return;
  }

  if (restart_due_to_notify_.load(std::memory_order_relaxed)) {
    return;
  }
  if (server_ == nullptr || server_->getConnectedCount() == 0) {
    return;
  }

  static constexpr uint32_t COOLDOWN_MS = 20000;
  static constexpr uint8_t FAIL_STREAK_TRIGGER = 3;
  static constexpr uint8_t COOLDOWN_CYCLES = 3;

  const uint32_t now = ble_now_ms_();
  if (status_notify_suppress_until_ms_ != 0 && (int32_t)(now - status_notify_suppress_until_ms_) < 0) {
    return;
  }

  const bool ok = status_char_->notify((const uint8_t*)json.data(), json.size());
  if (!ok) {
    if (status_notify_cooldowns_left_ > 0) {
      status_notify_cooldowns_left_--;
      if (status_notify_cooldowns_left_ == 0) {
        ESP_LOGW(TAG, "status notify failed after cooldowns; requesting BLE restart");
        restart_due_to_notify_.store(true, std::memory_order_relaxed);
      } else {
        status_notify_suppress_until_ms_ = now + COOLDOWN_MS;
        ESP_LOGW(TAG, "status notify still failing; cooldown %ums cycles_left=%u", (unsigned)COOLDOWN_MS,
                 (unsigned)status_notify_cooldowns_left_);
      }
      return;
    }

    if (status_notify_fail_streak_ < 255) {
      status_notify_fail_streak_++;
    }
    if (status_notify_fail_streak_ >= FAIL_STREAK_TRIGGER) {
      status_notify_fail_streak_ = 0;
      status_notify_cooldowns_left_ = COOLDOWN_CYCLES;
      status_notify_suppress_until_ms_ = now + COOLDOWN_MS;
      ESP_LOGW(TAG, "status notify failed %u times; cooldown %ums cycles=%u", (unsigned)FAIL_STREAK_TRIGGER,
               (unsigned)COOLDOWN_MS, (unsigned)COOLDOWN_CYCLES);
    } else if (status_notify_fail_streak_ == 1) {
      ESP_LOGW(TAG, "status notify failed (len=%u)", (unsigned)json.size());
    }
    return;
  }

  status_notify_fail_streak_ = 0;
  status_notify_cooldowns_left_ = 0;
  status_notify_suppress_until_ms_ = 0;
}

bool BLEStream::notify_history(const std::string& json) {
  if (!running_.load() || !history_subscribed_.load() || history_char_ == nullptr) {
    return false;
  }
  if (json.empty()) {
    return false;
  }
  const bool ok = history_char_->notify((const uint8_t*)json.data(), json.size());
  if (!ok) {
    ESP_LOGW(TAG, "history notify failed (len=%u)", (unsigned)json.size());
  }
  return ok;
}
