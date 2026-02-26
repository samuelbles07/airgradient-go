// SPDX-License-Identifier: MIT

#ifndef AIRGRADIENT_GO_NAND_STORAGE_SERVICE_H
#define AIRGRADIENT_GO_NAND_STORAGE_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

class NandStorageService {
 public:
  struct Config {
    // SPI bus: bus must already be initialized with spi_bus_initialize().
    spi_host_device_t spi_host = SPI2_HOST;
    gpio_num_t cs_pin = GPIO_NUM_MAX;
    int clock_speed_hz = 10 * 1000 * 1000;
    uint8_t spi_device_flags = SPI_DEVICE_HALFDUPLEX;

    // NAND/FATFS mount.
    const char* mount_path = "/nand";
    const char* records_path = "/nand/measurements.bin";
    bool format_if_mount_failed = true;
    int max_files = 4;
    size_t allocation_unit_size = 16 * 1024;

    // Worker task.
    uint32_t task_stack = 8192;
    UBaseType_t task_priority = 5;
    BaseType_t task_core = tskNO_AFFINITY;

    // Command queue.
    uint32_t queue_len = 32;

    // Periodic sync policy.
    // Setting either to 0 disables that trigger.
    uint32_t sync_every_n_records = 12;  // e.g. every 60s at 5s cadence.
    uint32_t sync_interval_ms = 60000;

    // Read behavior.
    bool verify_crc_on_read = true;
  };

  // Record fields (simple for now).
  // Latitude/longitude are degrees * 1e7 (E7 fixed-point).
  // pm*_ugm3_x10 is ug/m3 * 10.
  // Particle counts are stored as x10 (unit depends on PM sensor driver).
  struct Record {
    uint32_t id = 0;
    uint64_t timestamp_ms = 0;
    int32_t latitude_e7 = INT32_MIN;
    int32_t longitude_e7 = INT32_MIN;

    // PM mass concentrations.
    uint16_t pm01_ugm3_x10 = 0xFFFF;
    uint16_t pm25_ugm3_x10 = 0xFFFF;
    uint16_t pm10_ugm3_x10 = 0xFFFF;

    // Particle counts.
    uint32_t pc05_x10 = 0xFFFFFFFFu;
    uint32_t pc10_x10 = 0xFFFFFFFFu;
    uint32_t pc25_x10 = 0xFFFFFFFFu;
    uint32_t pc100_x10 = 0xFFFFFFFFu;

    // CO2.
    uint16_t co2_ppm = 0xFFFF;

    // SCD4x test-only CO2.
    uint16_t scd4x = 0xFFFF;

    // Senseair test-only CO2 (I2C).
    uint16_t s12 = 0xFFFF;
    uint16_t sunlight = 0xFFFF;

    // Ambient temperature/humidity.
    int16_t temperature_c_x100 = (int16_t)INT16_MIN;
    uint16_t humidity_rh_x100 = 0xFFFF;

    // Pressure.
    uint32_t pressure_pa = 0xFFFFFFFFu;

    // VOC/NOx raw signals.
    uint16_t tvoc_raw = 0xFFFF;
    uint16_t nox_raw = 0xFFFF;
  };

  NandStorageService();
  ~NandStorageService();

  esp_err_t init(const Config& cfg);
  esp_err_t start();
  esp_err_t stop();

  bool is_ready() const;
  esp_err_t last_error() const;

  // Enqueue a record write.
  // This returns when the command is queued (not when the write completes).
  // If force_sync=true, the worker fsyncs after writing this record.
  // Use flush_sync() to block until all prior queued writes are durable.
  esp_err_t enqueue_record(const Record& rec, bool force_sync, TickType_t timeout_ticks);

  // Enqueue a clear operation.
  // This returns when the command is queued (not when the clear completes).
  esp_err_t enqueue_clear(TickType_t timeout_ticks);

  // Synchronous operations (handled by the worker task).
  // These calls block until the worker completes the request.
  esp_err_t flush_sync(TickType_t send_timeout_ticks);
  esp_err_t clear_sync(TickType_t send_timeout_ticks);
  esp_err_t get_count_sync(uint32_t* out_count, TickType_t send_timeout_ticks);

  // Read up to max_records starting at start_idx (0 = oldest).
  esp_err_t read_range_sync(uint32_t start_idx,
                            Record* out_records,
                            uint32_t max_records,
                            uint32_t* out_read,
                            TickType_t send_timeout_ticks);

  // Convenience: read from the beginning.
  esp_err_t read_all_sync(Record* out_records,
                          uint32_t max_records,
                          uint32_t* out_read,
                          TickType_t send_timeout_ticks);

 private:
  static void task_thunk_(void* arg);
  void task_();

  Config cfg_;
  bool initialized_ = false;
  volatile bool stop_requested_ = false;

  volatile bool ready_ = false;
  volatile esp_err_t last_error_ = ESP_OK;

  TaskHandle_t task_handle_ = nullptr;
  SemaphoreHandle_t stopped_sem_ = nullptr;
  QueueHandle_t queue_ = nullptr;

  // Owned copies of path strings (cfg points may not live forever).
  char mount_path_[64] = {0};
  char records_path_[128] = {0};
};

#endif
