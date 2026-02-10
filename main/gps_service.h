// SPDX-License-Identifier: MIT

#ifndef AIRGRADIENT_GO_GPS_SERVICE_H
#define AIRGRADIENT_GO_GPS_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

class GPSService {
 public:
  enum class AntennaStatus {
    Unknown = 0,
    Ok,
    Open,
    Short,
  };

  struct Config {
    uart_port_t uart_num = UART_NUM_1;
    gpio_num_t tx_pin = GPIO_NUM_NC;  // MCU_TX -> GPS RX
    gpio_num_t rx_pin = GPIO_NUM_NC;  // MCU_RX <- GPS TX
    int baud_rate = 9600;

    int rx_buffer_size = 2048;

    // Task settings
    uint32_t task_stack = 4096;
    UBaseType_t task_priority = 10;
    BaseType_t task_core = tskNO_AFFINITY;

    // NMEA parsing
    bool check_checksum = true;
    bool log_raw_nmea = false;

    // UART read behavior.
    // Short timeout keeps stop() responsive and CPU usage low.
    int read_timeout_ms = 200;

    // Status logging
    uint32_t status_log_interval_ms = 5000;
    uint32_t sentence_timeout_ms = 2000;
    uint32_t fix_timeout_ms = 5000;
  };

  struct UtcTime {
    bool time_valid = false;
    bool date_valid = false;
    int year = 0;
    int month = 0;  // 1-12
    int day = 0;    // 1-31
    int hour = 0;   // 0-23
    int min = 0;    // 0-59
    int sec = 0;    // 0-59
  };

  struct Data {
    bool has_sentence = false;
    uint64_t last_sentence_ms = 0;

    bool fix_valid = false;
    int fix_quality = 0;  // GGA position_fix
    int satellites = 0;
    uint64_t last_fix_ms = 0;

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;

    bool altitude_valid = false;
    double altitude_m = 0.0;

    bool speed_valid = false;
    double speed_knots = 0.0;

    bool track_valid = false;
    double track_deg = 0.0;

    UtcTime utc;
    uint64_t last_time_ms = 0;

    AntennaStatus antenna_status = AntennaStatus::Unknown;
    uint64_t last_antenna_ms = 0;
  };

  GPSService();
  ~GPSService();

  esp_err_t init(const Config& cfg);
  esp_err_t start();
  esp_err_t stop();

  Data get() const;

  static const char* antenna_status_to_str(AntennaStatus s);

private:
  static void task_thunk_(void* arg);
  void task_();

  void handle_sentence_(char* line, size_t len, uint64_t now_ms);
  void update_from_gga_(const void* gga, uint64_t now_ms);
  void update_from_rmc_(const void* rmc, uint64_t now_ms);
  void update_from_txt_(const void* txt, uint64_t now_ms);

  void log_status_(uint64_t now_ms);

  static uint64_t now_ms_();

  // Note: this lock protects data_ only.
  mutable portMUX_TYPE data_mux_ = portMUX_INITIALIZER_UNLOCKED;
  Data data_;

  Config cfg_;
  bool initialized_ = false;
  volatile bool stop_requested_ = false;

  TaskHandle_t task_handle_ = nullptr;
  SemaphoreHandle_t stopped_sem_ = nullptr;

  // Line assembly
  char line_buf_[128] = {0};
  size_t line_len_ = 0;

  uint64_t last_status_log_ms_ = 0;
};

#endif
