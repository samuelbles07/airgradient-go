// SPDX-License-Identifier: MIT

#include "nand_storage_service.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_vfs_fat_nand.h"

#include "spi_nand_flash.h"

static const char* TAG = "NAND";

// On-disk format.
static constexpr uint32_t RECORDS_MAGIC = 0x314D4741;  // "AGM1" little-endian.
static constexpr uint16_t RECORDS_VERSION = 2;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint16_t record_size;
  uint16_t reserved;
} records_header_t;

typedef struct __attribute__((packed)) {
  uint32_t id;
  uint64_t timestamp_ms;
  int32_t latitude_e7;
  int32_t longitude_e7;

  // PM.
  uint16_t pm01_ugm3_x10;
  uint16_t pm25_ugm3_x10;
  uint16_t pm10_ugm3_x10;

  // Particle counts.
  uint32_t pc05_x10;
  uint32_t pc10_x10;
  uint32_t pc25_x10;
  uint32_t pc100_x10;

  // CO2 + Temp/Hum.
  uint16_t co2_ppm;
  int16_t temperature_c_x100;
  uint16_t humidity_rh_x100;

  // Pressure.
  uint32_t pressure_pa;

  // VOC/NOx.
  uint16_t tvoc_raw;
  uint16_t nox_raw;

  uint16_t crc16;
} records_disk_t;

static uint64_t now_ms() {
  return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static bool write_exact(FILE* f, const void* data, size_t len) {
  if (!f || !data || len == 0) {
    return false;
  }
  return fwrite(data, 1, len, f) == len;
}

static esp_err_t fsync_file(FILE* f) {
  if (!f) {
    return ESP_ERR_INVALID_ARG;
  }

  if (fflush(f) != 0) {
    return ESP_FAIL;
  }
  const int fd = fileno(f);
  if (fd < 0) {
    return ESP_FAIL;
  }
  if (fsync(fd) != 0) {
    return ESP_FAIL;
  }
  return ESP_OK;
}

struct SyncReply {
  esp_err_t err = ESP_OK;
  uint32_t a = 0;
};

enum class CommandType : uint8_t {
  WriteRecord = 0,
  Clear,
  Flush,
  GetCount,
  ReadRange,
  Stop,
};

struct Command {
  CommandType type = CommandType::WriteRecord;
  NandStorageService::Record record;
  bool force_sync = false;

  // ReadRange.
  uint32_t start_idx = 0;
  uint32_t max_records = 0;
  NandStorageService::Record* out_records = nullptr;

  // Sync completion.
  SemaphoreHandle_t done = nullptr;
  SyncReply* reply = nullptr;
};

struct WorkerState {
  spi_device_handle_t spi = nullptr;
  spi_nand_flash_device_t* nand = nullptr;
  bool mounted = false;

  FILE* f = nullptr;  // write-only handle (append)
  uint32_t record_count = 0;
  uint32_t records_since_sync = 0;
  uint64_t last_sync_ms = 0;
};

static esp_err_t read_header(FILE* f, records_header_t* out) {
  if (!f || !out) {
    return ESP_ERR_INVALID_ARG;
  }
  if (fseek(f, 0, SEEK_SET) != 0) {
    return ESP_FAIL;
  }
  if (fread(out, 1, sizeof(*out), f) != sizeof(*out)) {
    return ESP_FAIL;
  }
  return ESP_OK;
}

static bool header_valid(const records_header_t& h) {
  if (h.magic != RECORDS_MAGIC) {
    return false;
  }
  if (h.version != RECORDS_VERSION) {
    return false;
  }
  if (h.header_size != sizeof(records_header_t)) {
    return false;
  }
  if (h.record_size != sizeof(records_disk_t)) {
    return false;
  }
  return true;
}

static esp_err_t write_new_header(const char* path) {
  if (!path) {
    return ESP_ERR_INVALID_ARG;
  }

  FILE* f = fopen(path, "wb");
  if (!f) {
    return ESP_FAIL;
  }

  records_header_t h;
  memset(&h, 0, sizeof(h));
  h.magic = RECORDS_MAGIC;
  h.version = RECORDS_VERSION;
  h.header_size = (uint16_t)sizeof(records_header_t);
  h.record_size = (uint16_t)sizeof(records_disk_t);
  h.reserved = 0;

  esp_err_t ret = ESP_OK;
  if (!write_exact(f, &h, sizeof(h))) {
    ret = ESP_FAIL;
  } else {
    ret = fsync_file(f);
  }
  fclose(f);
  return ret;
}

static esp_err_t maybe_truncate_partial(const char* path, size_t header_size, size_t record_size) {
  struct stat st;
  if (stat(path, &st) != 0) {
    return ESP_FAIL;
  }
  if ((size_t)st.st_size < header_size) {
    return ESP_FAIL;
  }
  const size_t payload = (size_t)st.st_size - header_size;
  const size_t rem = payload % record_size;
  if (rem == 0) {
    return ESP_OK;
  }

  const off_t new_size = (off_t)((size_t)st.st_size - rem);
  int fd = open(path, O_RDWR);
  if (fd < 0) {
    return ESP_FAIL;
  }
  const int rc = ftruncate(fd, new_size);
  close(fd);
  return (rc == 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t open_and_validate(const char* records_path, WorkerState* st) {
  if (!records_path || !st) {
    return ESP_ERR_INVALID_ARG;
  }

  struct stat statbuf;
  const bool exists = (stat(records_path, &statbuf) == 0);
  if (!exists) {
    ESP_LOGI(TAG, "records file missing; creating %s", records_path);
    ESP_RETURN_ON_ERROR(write_new_header(records_path), TAG,
                        "write_new_header failed");
  }

  // Validate header.
  FILE* rf = fopen(records_path, "rb");
  if (!rf) {
    return ESP_FAIL;
  }
  records_header_t h;
  const esp_err_t hdr_err = read_header(rf, &h);
  fclose(rf);

  if (hdr_err != ESP_OK || !header_valid(h)) {
    // Old/unknown format: discard file and recreate.
    ESP_LOGW(TAG, "records header invalid; deleting file");
    (void)remove(records_path);
    ESP_RETURN_ON_ERROR(write_new_header(records_path), TAG, "write_new_header failed");
  }

  // Truncate partial tail (power-loss partial record).
  ESP_RETURN_ON_ERROR(
      maybe_truncate_partial(records_path, sizeof(records_header_t),
                             sizeof(records_disk_t)),
      TAG, "truncate failed");

  // Open append handle for writes.
  if (st->f) {
    fclose(st->f);
    st->f = nullptr;
  }
  st->f = fopen(records_path, "ab");
  if (!st->f) {
    return ESP_FAIL;
  }

  // Count records.
  if (stat(records_path, &statbuf) == 0 && (size_t)statbuf.st_size >= sizeof(records_header_t)) {
    const size_t payload = (size_t)statbuf.st_size - sizeof(records_header_t);
    st->record_count = (uint32_t)(payload / sizeof(records_disk_t));
  } else {
    st->record_count = 0;
  }

  st->records_since_sync = 0;
  st->last_sync_ms = now_ms();
  return ESP_OK;
}

static esp_err_t do_periodic_sync(const NandStorageService::Config& cfg,
                                  WorkerState* st,
                                  bool force) {
  if (!st || !st->f) {
    return ESP_ERR_INVALID_STATE;
  }

  if (force) {
    const esp_err_t err = fsync_file(st->f);
    if (err == ESP_OK) {
      st->records_since_sync = 0;
      st->last_sync_ms = now_ms();
    }
    return err;
  }

  const uint64_t now = now_ms();
  const bool need_count =
      (cfg.sync_every_n_records != 0 &&
       st->records_since_sync >= cfg.sync_every_n_records);
  const bool need_time =
      (cfg.sync_interval_ms != 0 && st->records_since_sync > 0 &&
       (now - st->last_sync_ms) >= cfg.sync_interval_ms);
  if (!need_count && !need_time) {
    return ESP_OK;
  }

  const esp_err_t err = fsync_file(st->f);
  if (err == ESP_OK) {
    st->records_since_sync = 0;
    st->last_sync_ms = now;
  }
  return err;
}

NandStorageService::NandStorageService() = default;

NandStorageService::~NandStorageService() {
  stop();
  if (queue_ != nullptr) {
    vQueueDelete(queue_);
    queue_ = nullptr;
  }
  if (stopped_sem_ != nullptr) {
    vSemaphoreDelete(stopped_sem_);
    stopped_sem_ = nullptr;
  }
}

esp_err_t NandStorageService::init(const Config& cfg) {
  if (initialized_) {
    return ESP_OK;
  }

  if (cfg.cs_pin == GPIO_NUM_MAX) {
    ESP_LOGE(TAG, "NandStorageService init: cs_pin not set");
    return ESP_ERR_INVALID_ARG;
  }
  if (!cfg.mount_path || !cfg.records_path) {
    return ESP_ERR_INVALID_ARG;
  }
  if (cfg.queue_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  cfg_ = cfg;
  snprintf(mount_path_, sizeof(mount_path_), "%s", cfg.mount_path);
  snprintf(records_path_, sizeof(records_path_), "%s", cfg.records_path);

  if (stopped_sem_ == nullptr) {
    stopped_sem_ = xSemaphoreCreateBinary();
    if (stopped_sem_ == nullptr) {
      ESP_LOGE(TAG, "NandStorageService init: failed to create semaphore");
      return ESP_ERR_NO_MEM;
    }
  }

  if (queue_ == nullptr) {
    queue_ = xQueueCreate((UBaseType_t)cfg_.queue_len, sizeof(Command));
    if (queue_ == nullptr) {
      ESP_LOGE(TAG, "NandStorageService init: failed to create queue");
      return ESP_ERR_NO_MEM;
    }
  }

  last_error_ = ESP_OK;
  ready_ = false;
  initialized_ = true;
  return ESP_OK;
}

esp_err_t NandStorageService::start() {
  if (!initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  if (task_handle_ != nullptr) {
    return ESP_OK;
  }

  stop_requested_ = false;
  ready_ = false;
  (void)xSemaphoreTake(stopped_sem_, 0);

  BaseType_t ok = pdFAIL;
  if (cfg_.task_core == tskNO_AFFINITY) {
    ok = xTaskCreate(&NandStorageService::task_thunk_, "nand", cfg_.task_stack, this,
                     cfg_.task_priority, &task_handle_);
  } else {
    ok = xTaskCreatePinnedToCore(&NandStorageService::task_thunk_, "nand", cfg_.task_stack,
                                 this, cfg_.task_priority, &task_handle_, cfg_.task_core);
  }

  if (ok != pdPASS) {
    task_handle_ = nullptr;
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

esp_err_t NandStorageService::stop() {
  if (task_handle_ == nullptr && !initialized_) {
    return ESP_OK;
  }

  stop_requested_ = true;

  if (queue_ != nullptr) {
    Command cmd = {};
    cmd.type = CommandType::Stop;
    (void)xQueueSend(queue_, &cmd, 0);
  }

  if (task_handle_ != nullptr && stopped_sem_ != nullptr) {
    if (xSemaphoreTake(stopped_sem_, pdMS_TO_TICKS(3000)) != pdTRUE) {
      ESP_LOGW(TAG, "NandStorageService stop: task did not stop in time; deleting");
      vTaskDelete(task_handle_);
    }
    task_handle_ = nullptr;
  }

  ready_ = false;
  initialized_ = false;
  return ESP_OK;
}

bool NandStorageService::is_ready() const {
  return ready_;
}

esp_err_t NandStorageService::last_error() const {
  return last_error_;
}

esp_err_t NandStorageService::enqueue_record(const Record& rec,
                                             bool force_sync,
                                             TickType_t timeout_ticks) {
  if (!initialized_ || queue_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  Command cmd = {};
  cmd.type = CommandType::WriteRecord;
  cmd.record = rec;
  cmd.force_sync = force_sync;
  return (xQueueSend(queue_, &cmd, timeout_ticks) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t NandStorageService::enqueue_clear(TickType_t timeout_ticks) {
  if (!initialized_ || queue_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  Command cmd = {};
  cmd.type = CommandType::Clear;
  cmd.done = nullptr;
  cmd.reply = nullptr;
  cmd.force_sync = true;
  return (xQueueSend(queue_, &cmd, timeout_ticks) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t NandStorageService::flush_sync(TickType_t send_timeout_ticks) {
  if (!initialized_ || queue_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  SyncReply reply;
  StaticSemaphore_t sem_buf;
  SemaphoreHandle_t done = xSemaphoreCreateBinaryStatic(&sem_buf);
  (void)xSemaphoreTake(done, 0);

  Command cmd = {};
  cmd.type = CommandType::Flush;
  cmd.done = done;
  cmd.reply = &reply;

  if (xQueueSend(queue_, &cmd, send_timeout_ticks) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  (void)xSemaphoreTake(done, portMAX_DELAY);
  return reply.err;
}

esp_err_t NandStorageService::clear_sync(TickType_t send_timeout_ticks) {
  if (!initialized_ || queue_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  SyncReply reply;
  StaticSemaphore_t sem_buf;
  SemaphoreHandle_t done = xSemaphoreCreateBinaryStatic(&sem_buf);
  (void)xSemaphoreTake(done, 0);

  Command cmd = {};
  cmd.type = CommandType::Clear;
  cmd.force_sync = true;
  cmd.done = done;
  cmd.reply = &reply;

  if (xQueueSend(queue_, &cmd, send_timeout_ticks) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  (void)xSemaphoreTake(done, portMAX_DELAY);
  return reply.err;
}

esp_err_t NandStorageService::get_count_sync(uint32_t* out_count, TickType_t send_timeout_ticks) {
  if (!out_count) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!initialized_ || queue_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  SyncReply reply;
  StaticSemaphore_t sem_buf;
  SemaphoreHandle_t done = xSemaphoreCreateBinaryStatic(&sem_buf);
  (void)xSemaphoreTake(done, 0);

  Command cmd = {};
  cmd.type = CommandType::GetCount;
  cmd.done = done;
  cmd.reply = &reply;

  if (xQueueSend(queue_, &cmd, send_timeout_ticks) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  (void)xSemaphoreTake(done, portMAX_DELAY);
  if (reply.err == ESP_OK) {
    *out_count = reply.a;
  }
  return reply.err;
}

esp_err_t NandStorageService::read_range_sync(uint32_t start_idx,
                                             Record* out_records,
                                             uint32_t max_records,
                                             uint32_t* out_read,
                                             TickType_t send_timeout_ticks) {
  if (!out_records || max_records == 0 || !out_read) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!initialized_ || queue_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  SyncReply reply;
  StaticSemaphore_t sem_buf;
  SemaphoreHandle_t done = xSemaphoreCreateBinaryStatic(&sem_buf);
  (void)xSemaphoreTake(done, 0);

  Command cmd = {};
  cmd.type = CommandType::ReadRange;
  cmd.start_idx = start_idx;
  cmd.max_records = max_records;
  cmd.out_records = out_records;
  cmd.done = done;
  cmd.reply = &reply;

  if (xQueueSend(queue_, &cmd, send_timeout_ticks) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  (void)xSemaphoreTake(done, portMAX_DELAY);
  *out_read = reply.a;
  return reply.err;
}

esp_err_t NandStorageService::read_all_sync(Record* out_records,
                                           uint32_t max_records,
                                           uint32_t* out_read,
                                           TickType_t send_timeout_ticks) {
  return read_range_sync(0, out_records, max_records, out_read, send_timeout_ticks);
}

void NandStorageService::task_thunk_(void* arg) {
  static_cast<NandStorageService*>(arg)->task_();
}

void NandStorageService::task_() {
  WorkerState st;
  esp_err_t ret = ESP_OK;

  spi_device_interface_config_t devcfg = {};
  spi_nand_flash_config_t nand_cfg = {};
  esp_vfs_fat_mount_config_t mount_cfg = {};

  ESP_LOGI(TAG, "NAND task starting");

  // Add NAND device to SPI bus.
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  devcfg.dummy_bits = 0;
  devcfg.mode = 0;
  devcfg.clock_source = SPI_CLK_SRC_DEFAULT;
  devcfg.duty_cycle_pos = 128;
  devcfg.cs_ena_pretrans = 0;
  devcfg.cs_ena_posttrans = 0;
  devcfg.clock_speed_hz = cfg_.clock_speed_hz;
  devcfg.input_delay_ns = 0;
  devcfg.spics_io_num = (int)cfg_.cs_pin;
  devcfg.flags = cfg_.spi_device_flags;
  devcfg.queue_size = 10;

  ret = spi_bus_add_device(cfg_.spi_host, &devcfg, &st.spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
    last_error_ = ret;
    goto out;
  }

  // Init NAND device.
  nand_cfg.device_handle = st.spi;
  nand_cfg.gc_factor = 4;
  nand_cfg.io_mode = SPI_NAND_IO_MODE_SIO;
  nand_cfg.flags = cfg_.spi_device_flags;

  ret = spi_nand_flash_init_device(&nand_cfg, &st.nand);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "spi_nand_flash_init_device failed: %s", esp_err_to_name(ret));
    st.nand = nullptr;
    last_error_ = ret;
    goto out;
  }

  // Mount FATFS.
  mount_cfg.format_if_mount_failed = cfg_.format_if_mount_failed;
  mount_cfg.max_files = cfg_.max_files;
  mount_cfg.allocation_unit_size = cfg_.allocation_unit_size;
  mount_cfg.disk_status_check_enable = false;
  mount_cfg.use_one_fat = false;

  ret = esp_vfs_fat_nand_mount(mount_path_, st.nand, &mount_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_vfs_fat_nand_mount failed: %s", esp_err_to_name(ret));
    last_error_ = ret;
    goto out;
  }
  st.mounted = true;

  // Ensure records file exists and open it.
  ret = open_and_validate(records_path_, &st);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "open_and_validate failed: %s", esp_err_to_name(ret));
    last_error_ = ret;
    goto out;
  }

  ready_ = true;
  last_error_ = ESP_OK;
  ESP_LOGI(TAG, "NAND ready: %s", records_path_);

  while (!stop_requested_) {
    Command cmd;
    const TickType_t wait = pdMS_TO_TICKS(250);
    if (xQueueReceive(queue_, &cmd, wait) != pdTRUE) {
      (void)do_periodic_sync(cfg_, &st, false);
      continue;
    }

    if (!ready_ && cmd.type != CommandType::Stop) {
      if (cmd.done && cmd.reply) {
        cmd.reply->err = ESP_ERR_INVALID_STATE;
        cmd.reply->a = 0;
        xSemaphoreGive(cmd.done);
      }
      continue;
    }

    switch (cmd.type) {
      case CommandType::WriteRecord: {
        if (!st.f) {
          ret = open_and_validate(records_path_, &st);
          if (ret != ESP_OK) {
            last_error_ = ret;
            ready_ = false;
            break;
          }
        }

        records_disk_t d;
        memset(&d, 0, sizeof(d));
        d.id = cmd.record.id;
        d.timestamp_ms = cmd.record.timestamp_ms;
        d.latitude_e7 = cmd.record.latitude_e7;
        d.longitude_e7 = cmd.record.longitude_e7;
        d.pm01_ugm3_x10 = cmd.record.pm01_ugm3_x10;
        d.pm25_ugm3_x10 = cmd.record.pm25_ugm3_x10;
        d.pm10_ugm3_x10 = cmd.record.pm10_ugm3_x10;
        d.pc05_x10 = cmd.record.pc05_x10;
        d.pc10_x10 = cmd.record.pc10_x10;
        d.pc25_x10 = cmd.record.pc25_x10;
        d.pc100_x10 = cmd.record.pc100_x10;
        d.co2_ppm = cmd.record.co2_ppm;
        d.temperature_c_x100 = cmd.record.temperature_c_x100;
        d.humidity_rh_x100 = cmd.record.humidity_rh_x100;
        d.pressure_pa = cmd.record.pressure_pa;
        d.tvoc_raw = cmd.record.tvoc_raw;
        d.nox_raw = cmd.record.nox_raw;
        d.crc16 = crc16_ccitt((const uint8_t*)&d, sizeof(d) - sizeof(d.crc16));

        const bool ok = fwrite(&d, 1, sizeof(d), st.f) == sizeof(d);
        if (!ok) {
          last_error_ = ESP_FAIL;
          ready_ = false;
          break;
        }

        st.record_count++;
        st.records_since_sync++;
        (void)do_periodic_sync(cfg_, &st, cmd.force_sync);
        break;
      }

      case CommandType::Clear: {
        esp_err_t err = ESP_OK;
        if (st.f) {
          (void)fsync_file(st.f);
          fclose(st.f);
          st.f = nullptr;
        }

        (void)remove(records_path_);
        err = write_new_header(records_path_);
        if (err == ESP_OK) {
          err = open_and_validate(records_path_, &st);
        }

        if (err != ESP_OK) {
          last_error_ = err;
          ready_ = false;
        }

        if (cmd.done && cmd.reply) {
          cmd.reply->err = err;
          cmd.reply->a = 0;
          xSemaphoreGive(cmd.done);
        }
        break;
      }

      case CommandType::Flush: {
        esp_err_t err = ESP_OK;
        if (st.f) {
          err = fsync_file(st.f);
          if (err == ESP_OK) {
            st.records_since_sync = 0;
            st.last_sync_ms = now_ms();
          }
        }
        if (cmd.done && cmd.reply) {
          cmd.reply->err = err;
          cmd.reply->a = 0;
          xSemaphoreGive(cmd.done);
        }
        break;
      }

      case CommandType::GetCount: {
        if (cmd.done && cmd.reply) {
          cmd.reply->err = ESP_OK;
          cmd.reply->a = st.record_count;
          xSemaphoreGive(cmd.done);
        }
        break;
      }

      case CommandType::ReadRange: {
        SyncReply* reply = cmd.reply;
        if (!cmd.done || reply == nullptr || cmd.out_records == nullptr || cmd.max_records == 0) {
          if (cmd.done && reply) {
            reply->err = ESP_ERR_INVALID_ARG;
            reply->a = 0;
            xSemaphoreGive(cmd.done);
          }
          break;
        }

        // Ensure write buffers are visible to the read handle.
        if (st.f) {
          (void)fflush(st.f);
        }

        FILE* rf = fopen(records_path_, "rb");
        if (!rf) {
          reply->err = ESP_ERR_NOT_FOUND;
          reply->a = 0;
          xSemaphoreGive(cmd.done);
          break;
        }

        records_header_t h;
        esp_err_t err = read_header(rf, &h);
        if (err != ESP_OK || !header_valid(h)) {
          fclose(rf);
          reply->err = ESP_FAIL;
          reply->a = 0;
          xSemaphoreGive(cmd.done);
          break;
        }

        const uint32_t total = st.record_count;
        if (cmd.start_idx >= total) {
          fclose(rf);
          reply->err = ESP_OK;
          reply->a = 0;
          xSemaphoreGive(cmd.done);
          break;
        }

        const uint32_t remaining = total - cmd.start_idx;
        const uint32_t want = (cmd.max_records < remaining) ? cmd.max_records : remaining;
        const long offset = (long)sizeof(records_header_t) +
                            (long)cmd.start_idx * (long)sizeof(records_disk_t);
        if (fseek(rf, offset, SEEK_SET) != 0) {
          fclose(rf);
          reply->err = ESP_FAIL;
          reply->a = 0;
          xSemaphoreGive(cmd.done);
          break;
        }

        uint32_t read_n = 0;
        for (; read_n < want; ++read_n) {
          records_disk_t d;
          if (fread(&d, 1, sizeof(d), rf) != sizeof(d)) {
            break;
          }

          if (cfg_.verify_crc_on_read) {
            const uint16_t calc = crc16_ccitt((const uint8_t*)&d, sizeof(d) - sizeof(d.crc16));
            if (calc != d.crc16) {
              err = ESP_ERR_INVALID_CRC;
              break;
            }
          }

          cmd.out_records[read_n].id = d.id;
          cmd.out_records[read_n].timestamp_ms = d.timestamp_ms;
          cmd.out_records[read_n].latitude_e7 = d.latitude_e7;
          cmd.out_records[read_n].longitude_e7 = d.longitude_e7;
          cmd.out_records[read_n].pm01_ugm3_x10 = d.pm01_ugm3_x10;
          cmd.out_records[read_n].pm25_ugm3_x10 = d.pm25_ugm3_x10;
          cmd.out_records[read_n].pm10_ugm3_x10 = d.pm10_ugm3_x10;
          cmd.out_records[read_n].pc05_x10 = d.pc05_x10;
          cmd.out_records[read_n].pc10_x10 = d.pc10_x10;
          cmd.out_records[read_n].pc25_x10 = d.pc25_x10;
          cmd.out_records[read_n].pc100_x10 = d.pc100_x10;
          cmd.out_records[read_n].co2_ppm = d.co2_ppm;
          cmd.out_records[read_n].temperature_c_x100 = d.temperature_c_x100;
          cmd.out_records[read_n].humidity_rh_x100 = d.humidity_rh_x100;
          cmd.out_records[read_n].pressure_pa = d.pressure_pa;
          cmd.out_records[read_n].tvoc_raw = d.tvoc_raw;
          cmd.out_records[read_n].nox_raw = d.nox_raw;
        }

        fclose(rf);
        reply->err = err;
        reply->a = read_n;
        xSemaphoreGive(cmd.done);
        break;
      }

      case CommandType::Stop:
      default: {
        stop_requested_ = true;
        break;
      }
    }
  }

out:
  ready_ = false;

  if (st.f) {
    (void)fsync_file(st.f);
    fclose(st.f);
    st.f = nullptr;
  }
  if (st.mounted) {
    (void)esp_vfs_fat_nand_unmount(mount_path_, st.nand);
    st.mounted = false;
  }
  if (st.nand) {
    spi_nand_flash_deinit_device(st.nand);
    st.nand = nullptr;
  }
  if (st.spi) {
    spi_bus_remove_device(st.spi);
    st.spi = nullptr;
  }

  if (stopped_sem_ != nullptr) {
    xSemaphoreGive(stopped_sem_);
  }
  vTaskDelete(nullptr);
}
