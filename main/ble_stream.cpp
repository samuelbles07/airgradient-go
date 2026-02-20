// SPDX-License-Identifier: MIT

#include "ble_stream.h"

#include "esp_log.h"

// esp-nimble-cpp
#include "NimBLEDevice.h"

#include "cJSON.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include <vector>

static const char* const TAG = "BLEStream";

static constexpr const char* SERVICE_UUID = "d1c0c0a0-6b48-4b2a-9b1d-59f9f2b0a1e1";
static constexpr const char* MEASURES_CHAR_UUID = "d1c0c0a1-6b48-4b2a-9b1d-59f9f2b0a1e1";
static constexpr const char* STATUS_CHAR_UUID = "d1c0c0a2-6b48-4b2a-9b1d-59f9f2b0a1e1";
static constexpr const char* CONFIG_CHAR_UUID = "d1c0c0a3-6b48-4b2a-9b1d-59f9f2b0a1e1";

static constexpr uint32_t TRACKING_SLEEP_MIN_S = 1;
static constexpr uint32_t TRACKING_SLEEP_MAX_S = 86400;

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

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer;
    (void)connInfo;
    (void)reason;

    if (s_ == nullptr) {
      return;
    }
    s_->set_measures_subscribed_(false);
    s_->set_status_subscribed_(false);

    // Belt-and-suspenders: ensure advertising restarts.
    (void)NimBLEDevice::startAdvertising();
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

esp_err_t BLEStream::start(const char* device_name) {
  if (running_.load()) {
    return ESP_OK;
  }
  if (device_name == nullptr || device_name[0] == '\0') {
    device_name = "AirGradientGo";
  }

  measures_subscribed_.store(false);
  status_subscribed_.store(false);

  if (!NimBLEDevice::init(std::string(device_name))) {
    ESP_LOGW(TAG, "NimBLEDevice::init failed");
    return ESP_FAIL;
  }

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
  if (measures_char_ == nullptr || status_char_ == nullptr || config_char_ == nullptr) {
    ESP_LOGW(TAG, "createCharacteristic failed");
    (void)NimBLEDevice::deinit(true);
    server_ = nullptr;
    service_ = nullptr;
    measures_char_ = nullptr;
    status_char_ = nullptr;
    config_char_ = nullptr;
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
    adv->start();
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
  running_.store(false);
  ESP_LOGI(TAG, "stopped");
}

void BLEStream::request_tracking_sleep_interval_s_(uint32_t s) {
  pending_tracking_sleep_interval_s_.store(s, std::memory_order_relaxed);
  pending_tracking_sleep_interval_.store(true, std::memory_order_relaxed);
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

void BLEStream::notify_measures(const std::string& json) {
  if (!running_.load() || !measures_subscribed_.load() || measures_char_ == nullptr) {
    return;
  }
  if (json.empty()) {
    return;
  }
  const bool ok = measures_char_->notify((const uint8_t*)json.data(), json.size());
  if (!ok) {
    ESP_LOGW(TAG, "measures notify failed (len=%u)", (unsigned)json.size());
  }
}

void BLEStream::notify_status(const std::string& json) {
  if (!running_.load() || !status_subscribed_.load() || status_char_ == nullptr) {
    return;
  }
  if (json.empty()) {
    return;
  }
  const bool ok = status_char_->notify((const uint8_t*)json.data(), json.size());
  if (!ok) {
    ESP_LOGW(TAG, "status notify failed (len=%u)", (unsigned)json.size());
  }
}
