// SPDX-License-Identifier: MIT

#include "gps_service.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "esp_timer.h"

extern "C" {
#include <gpgga.h>
#include <gprmc.h>
#include <gptxt.h>
#include <nmea.h>
#include <parser.h>
}

static const char* TAG = "GPS";

static bool contains_token(const char* data, size_t len, const char* token) {
  if (!data || !token) {
    return false;
  }
  const size_t token_len = strlen(token);
  if (token_len == 0 || token_len > len) {
    return false;
  }
  for (size_t i = 0; i + token_len <= len; ++i) {
    if (memcmp(data + i, token, token_len) == 0) {
      return true;
    }
  }
  return false;
}

static double position_to_decimal(const nmea_position* pos) {
  if (!pos || pos->cardinal == NMEA_CARDINAL_DIR_UNKNOWN) {
    return 0.0;
  }
  double degrees = (double)pos->degrees + (pos->minutes / 60.0);
  if (pos->cardinal == NMEA_CARDINAL_DIR_SOUTH || pos->cardinal == NMEA_CARDINAL_DIR_WEST) {
    degrees = -degrees;
  }
  return degrees;
}

uint64_t GPSService::now_ms_() {
  return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

GPSService::GPSService() = default;

GPSService::~GPSService() {
  stop();
  if (stopped_sem_ != nullptr) {
    vSemaphoreDelete(stopped_sem_);
    stopped_sem_ = nullptr;
  }
}

esp_err_t GPSService::init(const Config& cfg) {
  if (initialized_) {
    return ESP_OK;
  }
  esp_log_level_set(TAG, ESP_LOG_DEBUG);

  if (cfg.rx_pin == GPIO_NUM_NC) {
    ESP_LOGE(TAG, "GPSService init: rx_pin not set");
    return ESP_ERR_INVALID_ARG;
  }

  // TAU1113 is usually receive-only for NMEA; allow tx_pin NC.
  cfg_ = cfg;

  if (stopped_sem_ == nullptr) {
    stopped_sem_ = xSemaphoreCreateBinary();
    if (stopped_sem_ == nullptr) {
      ESP_LOGE(TAG, "GPSService init: failed to create semaphore");
      return ESP_ERR_NO_MEM;
    }
  }

  // Ensure libnmea parsers are loaded (ESP-IDF does not always run constructor hooks
  // the same way as hosted environments).
  static bool parsers_loaded = false;
  if (!parsers_loaded) {
    int n = nmea_load_parsers();
    if (n <= 0) {
      ESP_LOGE(TAG, "GPSService init: nmea_load_parsers failed");
      return ESP_FAIL;
    }
    parsers_loaded = true;
  }

  uart_config_t uart_cfg = {
      .baud_rate = cfg_.baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret = uart_param_config(cfg_.uart_num, &uart_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPSService init: uart_param_config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  const int tx_pin = (cfg_.tx_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : (int)cfg_.tx_pin;
  const int rx_pin = (int)cfg_.rx_pin;
  ret = uart_set_pin(cfg_.uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPSService init: uart_set_pin failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_driver_install(cfg_.uart_num, cfg_.rx_buffer_size, 0, 0, nullptr, 0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "GPSService init: uart_driver_install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  uart_flush_input(cfg_.uart_num);
  initialized_ = true;

  ESP_LOGI(TAG, "GPSService UART ready (TAU1113) uart=%d baud=%d tx=%d rx=%d",
           (int)cfg_.uart_num, cfg_.baud_rate, (int)cfg_.tx_pin, (int)cfg_.rx_pin);
  return ESP_OK;
}

esp_err_t GPSService::start() {
  if (!initialized_) {
    ESP_LOGE(TAG, "GPSService start: call init() first");
    return ESP_ERR_INVALID_STATE;
  }
  if (task_handle_ != nullptr) {
    return ESP_OK;
  }

  stop_requested_ = false;
  (void)xSemaphoreTake(stopped_sem_, 0);

  BaseType_t ok = pdFAIL;
  if (cfg_.task_core == tskNO_AFFINITY) {
    ok = xTaskCreate(&GPSService::task_thunk_, "gps", cfg_.task_stack, this, cfg_.task_priority, &task_handle_);
  } else {
    ok = xTaskCreatePinnedToCore(&GPSService::task_thunk_, "gps", cfg_.task_stack, this, cfg_.task_priority,
                                &task_handle_, cfg_.task_core);
  }

  if (ok != pdPASS) {
    task_handle_ = nullptr;
    ESP_LOGE(TAG, "GPSService start: failed to create task");
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "GPSService started");
  return ESP_OK;
}

esp_err_t GPSService::stop() {
  if (task_handle_ == nullptr && !initialized_) {
    return ESP_OK;
  }

  stop_requested_ = true;

  if (task_handle_ != nullptr && stopped_sem_ != nullptr) {
    // Wait for task to exit.
    if (xSemaphoreTake(stopped_sem_, pdMS_TO_TICKS(2000)) != pdTRUE) {
      ESP_LOGW(TAG, "GPSService stop: task did not stop in time; deleting");
      vTaskDelete(task_handle_);
    }
    task_handle_ = nullptr;
  }

  if (initialized_) {
    uart_flush_input(cfg_.uart_num);
    esp_err_t ret = uart_driver_delete(cfg_.uart_num);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "GPSService stop: uart_driver_delete failed: %s", esp_err_to_name(ret));
    }
    initialized_ = false;
  }

  // Reset parse state.
  line_len_ = 0;
  memset(line_buf_, 0, sizeof(line_buf_));

  ESP_LOGI(TAG, "GPSService stopped");
  return ESP_OK;
}

GPSService::Data GPSService::get() const {
  Data out;
  portENTER_CRITICAL(&data_mux_);
  out = data_;
  portEXIT_CRITICAL(&data_mux_);
  return out;
}

const char* GPSService::antenna_status_to_str(AntennaStatus s) {
  switch (s) {
    case AntennaStatus::Ok: return "OK";
    case AntennaStatus::Open: return "OPEN";
    case AntennaStatus::Short: return "SHORT";
    case AntennaStatus::Unknown:
    default: return "UNKNOWN";
  }
}

void GPSService::task_thunk_(void* arg) {
  static_cast<GPSService*>(arg)->task_();
}

void GPSService::task_() {
  uint8_t rx[64];

  while (!stop_requested_) {
    int n = uart_read_bytes(cfg_.uart_num, rx, sizeof(rx), pdMS_TO_TICKS(cfg_.read_timeout_ms));
    const uint64_t now = now_ms_();

    if (n > 0) {
      for (int i = 0; i < n; ++i) {
        const char c = (char)rx[i];

        if (c == '$') {
          line_len_ = 0;
        }

        if (line_len_ < (sizeof(line_buf_) - 1)) {
          line_buf_[line_len_++] = c;
        } else {
          // Overflow: drop the line.
          line_len_ = 0;
          continue;
        }

        if (c == '\n') {
          // Ensure CRLF ending for libnmea validation.
          if (line_len_ >= 2 && line_buf_[line_len_ - 2] != '\r') {
            if (line_len_ < (sizeof(line_buf_) - 1)) {
              line_buf_[line_len_ - 1] = '\r';
              line_buf_[line_len_++] = '\n';
            }
          }

          if (line_len_ > 0) {
            if (cfg_.log_raw_nmea) {
              // Log without CRLF.
              size_t log_len = line_len_;
              if (log_len >= 2 && line_buf_[log_len - 2] == '\r' && line_buf_[log_len - 1] == '\n') {
                log_len -= 2;
              }
              ESP_LOGD(TAG, "NMEA RX: %.*s", (int)log_len, line_buf_);
            }
            handle_sentence_(line_buf_, line_len_, now);
          }

          line_len_ = 0;
        }
      }
    }

    log_status_(now);
  }

  if (stopped_sem_ != nullptr) {
    xSemaphoreGive(stopped_sem_);
  }
  vTaskDelete(nullptr);
}

void GPSService::handle_sentence_(char* line, size_t len, uint64_t now_ms) {
  if (line == nullptr || len < 9) {
    return;
  }

  // TAU1113 antenna status token (commonly in GPTXT).
  AntennaStatus ant = AntennaStatus::Unknown;
  if (contains_token(line, len, "ANT_OK")) {
    ant = AntennaStatus::Ok;
  } else if (contains_token(line, len, "ANT_OPEN")) {
    ant = AntennaStatus::Open;
  } else if (contains_token(line, len, "ANT_SHORT")) {
    ant = AntennaStatus::Short;
  }
  if (ant != AntennaStatus::Unknown) {
    portENTER_CRITICAL(&data_mux_);
    data_.antenna_status = ant;
    data_.last_antenna_ms = now_ms;
    portEXIT_CRITICAL(&data_mux_);
  }

  nmea_s* parsed = nmea_parse(line, len, cfg_.check_checksum ? 1 : 0);
  if (parsed == nullptr) {
    return;
  }

  portENTER_CRITICAL(&data_mux_);
  data_.has_sentence = true;
  data_.last_sentence_ms = now_ms;
  portEXIT_CRITICAL(&data_mux_);

  if (parsed->type == NMEA_GPGGA) {
    update_from_gga_(parsed, now_ms);
  } else if (parsed->type == NMEA_GPRMC) {
    update_from_rmc_(parsed, now_ms);
  } else if (parsed->type == NMEA_GPTXT) {
    update_from_txt_(parsed, now_ms);
  }

  nmea_free(parsed);
}

void GPSService::update_from_gga_(const void* gga_ptr, uint64_t now_ms) {
  const nmea_gpgga_s* gga = static_cast<const nmea_gpgga_s*>(gga_ptr);
  if (gga == nullptr) {
    return;
  }

  const bool fix = (gga->position_fix > 0);
  const int sats = gga->n_satellites;
  const bool ok = fix && (sats > 0);
  const double lat = position_to_decimal(&gga->latitude);
  const double lon = position_to_decimal(&gga->longitude);

  portENTER_CRITICAL(&data_mux_);
  data_.fix_quality = (int)gga->position_fix;
  data_.fix_valid = fix;
  data_.satellites = sats;
  if (fix) {
    data_.last_fix_ms = now_ms;
  }

  if (ok) {
    if (gga->latitude.cardinal != NMEA_CARDINAL_DIR_UNKNOWN && gga->longitude.cardinal != NMEA_CARDINAL_DIR_UNKNOWN) {
      data_.latitude_deg = lat;
      data_.longitude_deg = lon;
    }

    if (gga->altitude_unit == 'M') {
      data_.altitude_m = gga->altitude;
      data_.altitude_valid = true;
    } else {
      data_.altitude_valid = false;
    }

    // GGA time is time-of-day only.
    if (gga->time.tm_hour >= 0 && gga->time.tm_hour <= 23 && gga->time.tm_min >= 0 && gga->time.tm_min <= 59 &&
        gga->time.tm_sec >= 0 && gga->time.tm_sec <= 59) {
      data_.utc.hour = gga->time.tm_hour;
      data_.utc.min = gga->time.tm_min;
      data_.utc.sec = gga->time.tm_sec;
      data_.utc.time_valid = true;
      data_.last_time_ms = now_ms;
    } else {
      data_.utc.time_valid = false;
    }
  } else {
    data_.utc.time_valid = false;
    data_.utc.date_valid = false;
    data_.altitude_valid = false;
    data_.speed_valid = false;
    data_.track_valid = false;
  }
  portEXIT_CRITICAL(&data_mux_);
}

void GPSService::update_from_rmc_(const void* rmc_ptr, uint64_t now_ms) {
  const nmea_gprmc_s* rmc = static_cast<const nmea_gprmc_s*>(rmc_ptr);
  if (rmc == nullptr) {
    return;
  }

  const bool fix = rmc->valid;
  const bool ok = fix;  // satellites gate evaluated inside critical section
  const double lat = position_to_decimal(&rmc->latitude);
  const double lon = position_to_decimal(&rmc->longitude);

  portENTER_CRITICAL(&data_mux_);
  data_.fix_valid = fix;
  if (fix) {
    data_.last_fix_ms = now_ms;
  }

  const bool sats_ok = (data_.satellites > 0);
  if (ok && sats_ok) {
    if (rmc->latitude.cardinal != NMEA_CARDINAL_DIR_UNKNOWN && rmc->longitude.cardinal != NMEA_CARDINAL_DIR_UNKNOWN) {
      data_.latitude_deg = lat;
      data_.longitude_deg = lon;
    }

    data_.speed_knots = rmc->gndspd_knots;
    data_.speed_valid = true;

    data_.track_deg = rmc->track_deg;
    data_.track_valid = true;

    // RMC has date+time.
    const struct tm* t = &rmc->date_time;
    if (t->tm_hour >= 0 && t->tm_hour <= 23 && t->tm_min >= 0 && t->tm_min <= 59 && t->tm_sec >= 0 && t->tm_sec <= 59) {
      data_.utc.hour = t->tm_hour;
      data_.utc.min = t->tm_min;
      data_.utc.sec = t->tm_sec;
      data_.utc.time_valid = true;
      data_.last_time_ms = now_ms;
    } else {
      data_.utc.time_valid = false;
    }

    if (t->tm_year >= 70 && t->tm_mon >= 0 && t->tm_mon <= 11 && t->tm_mday >= 1 && t->tm_mday <= 31) {
      data_.utc.year = t->tm_year + 1900;
      data_.utc.month = t->tm_mon + 1;
      data_.utc.day = t->tm_mday;
      data_.utc.date_valid = true;
    } else {
      data_.utc.date_valid = false;
    }
  } else {
    data_.utc.time_valid = false;
    data_.utc.date_valid = false;
    data_.altitude_valid = false;
    data_.speed_valid = false;
    data_.track_valid = false;
  }
  portEXIT_CRITICAL(&data_mux_);
}

void GPSService::update_from_txt_(const void* txt_ptr, uint64_t now_ms) {
  const nmea_gptxt_s* txt = static_cast<const nmea_gptxt_s*>(txt_ptr);
  if (txt == nullptr || txt->text[0] == '\0') {
    return;
  }

  AntennaStatus ant = AntennaStatus::Unknown;
  if (strstr(txt->text, "ANT_OK") != nullptr) {
    ant = AntennaStatus::Ok;
  } else if (strstr(txt->text, "ANT_OPEN") != nullptr) {
    ant = AntennaStatus::Open;
  } else if (strstr(txt->text, "ANT_SHORT") != nullptr) {
    ant = AntennaStatus::Short;
  }
  if (ant != AntennaStatus::Unknown) {
    portENTER_CRITICAL(&data_mux_);
    data_.antenna_status = ant;
    data_.last_antenna_ms = now_ms;
    portEXIT_CRITICAL(&data_mux_);
  }
}

void GPSService::log_status_(uint64_t now_ms) {
  if (cfg_.status_log_interval_ms == 0) {
    return;
  }
  if (last_status_log_ms_ != 0 && (now_ms - last_status_log_ms_) < cfg_.status_log_interval_ms) {
    return;
  }
  last_status_log_ms_ = now_ms;

  Data d = get();
  const bool has_sentence = d.has_sentence && d.last_sentence_ms != 0 && (now_ms - d.last_sentence_ms) <= cfg_.sentence_timeout_ms;
  const bool has_fix = d.fix_valid && d.last_fix_ms != 0 && (now_ms - d.last_fix_ms) <= cfg_.fix_timeout_ms;
  const bool has_time = d.utc.time_valid && d.last_time_ms != 0 && (now_ms - d.last_time_ms) <= cfg_.sentence_timeout_ms;

  AntennaStatus ant = d.antenna_status;
  if (d.last_antenna_ms == 0 || (now_ms - d.last_antenna_ms) > (uint64_t)cfg_.sentence_timeout_ms * 3ULL) {
    ant = AntennaStatus::Unknown;
  }

  const char* ant_str = "UNKNOWN";
  if (ant == AntennaStatus::Ok) ant_str = "OK";
  else if (ant == AntennaStatus::Open) ant_str = "OPEN";
  else if (ant == AntennaStatus::Short) ant_str = "SHORT";

  const char* status = !has_sentence ? "OFF" : (has_fix ? "FIX" : "SEARCHING");

  char alt_buf[16];
  char spd_buf[16];
  char trk_buf[16];
  strcpy(alt_buf, "--");
  strcpy(spd_buf, "--");
  strcpy(trk_buf, "--");
  if (d.altitude_valid) {
    snprintf(alt_buf, sizeof(alt_buf), "%.1f", d.altitude_m);
  }
  if (d.speed_valid) {
    snprintf(spd_buf, sizeof(spd_buf), "%.1f", d.speed_knots);
  }
  if (d.track_valid) {
    snprintf(trk_buf, sizeof(trk_buf), "%.0f", d.track_deg);
  }

  if (has_time) {
    if (has_fix) {
      ESP_LOGI(TAG,
               "GPS status=%s fix_q=%d sats=%d time=%02d:%02d ant=%s lat=%.5f lon=%.5f alt=%s spd=%skn trk=%sdeg",
               status, d.fix_quality, d.satellites, d.utc.hour, d.utc.min, ant_str, d.latitude_deg, d.longitude_deg,
               alt_buf, spd_buf, trk_buf);
    } else {
      ESP_LOGI(TAG, "GPS status=%s fix_q=%d sats=%d time=%02d:%02d ant=%s lat=-- lon=--",
               status, d.fix_quality, d.satellites, d.utc.hour, d.utc.min, ant_str);
    }
  } else {
    if (has_fix) {
      ESP_LOGI(TAG,
               "GPS status=%s fix_q=%d sats=%d time=--:-- ant=%s lat=%.5f lon=%.5f alt=%s spd=%skn trk=%sdeg",
               status, d.fix_quality, d.satellites, ant_str, d.latitude_deg, d.longitude_deg, alt_buf, spd_buf,
               trk_buf);
    } else {
      ESP_LOGI(TAG, "GPS status=%s fix_q=%d sats=%d time=--:-- ant=%s lat=-- lon=--",
               status, d.fix_quality, d.satellites, ant_str);
    }
  }
}
