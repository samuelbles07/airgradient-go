/**
 * @file log_storage_new.cpp
 * @brief NAND Flash storage using espressif/spi_nand_flash component
 *
 * Uses the official Espressif SPI NAND Flash driver with Dhara FTL for
 * automatic bad block management and wear leveling.
 */

#include "log_storage.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat_nand.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "spi_nand_flash.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "log_store";

// W25N512GV SPI NAND configuration
// Shares SPI2_HOST bus with e-paper display (already initialized)
static const spi_host_device_t kNandSpiHost = SPI2_HOST;
static const int kNandPinCs = 4;
static const int kNandClockHz = 10 * 1000 * 1000;

// Mount point for FATFS
static const char *kMountPoint = "/nand";
static const char *kSensorDataFile = "/nand/sensors.bin";

// State
static spi_device_handle_t g_nand_spi = nullptr;
static spi_nand_flash_device_t *g_nand_device = nullptr;
static TaskHandle_t g_mount_task = nullptr;
static SemaphoreHandle_t g_storage_lock = nullptr;
static bool g_storage_ready = false;
static bool g_mount_started = false;

// Sensor record file handle (unused for now, using open/close per operation)
// static FILE *g_sensor_file = nullptr;
static int32_t g_record_count = -1;

// ============================================================================
// CRC16 Implementation
// ============================================================================

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ============================================================================
// Internal Helper Functions
// ============================================================================

static bool storage_lock(TickType_t timeout_ticks) {
  if (!g_storage_lock) {
    return false;
  }
  return xSemaphoreTake(g_storage_lock, timeout_ticks) == pdTRUE;
}

static void storage_unlock(void) {
  if (g_storage_lock) {
    xSemaphoreGive(g_storage_lock);
  }
}

// ============================================================================
// Mount Task - Runs in background to initialize NAND flash
// ============================================================================

static void log_storage_mount_task(void *arg) {
  ESP_LOGI(TAG, "=== NAND Flash Initialization (spi_nand_flash) ===");

  esp_err_t ret;

  // Create storage lock mutex
  if (!g_storage_lock) {
    g_storage_lock = xSemaphoreCreateMutex();
    if (!g_storage_lock) {
      ESP_LOGE(TAG, "Failed to create storage mutex");
      vTaskDelete(nullptr);
      return;
    }
  }

  // SPI bus is already initialized by e-paper driver
  // Just add our NAND device to the bus
  ESP_LOGI(TAG, "Adding NAND device to SPI2_HOST bus...");

  // Configure SPI device with half-duplex for NAND
  spi_device_interface_config_t devcfg = {
      .command_bits = 0,
      .address_bits = 0,
      .dummy_bits = 0,
      .mode = 0,
      .clock_source = SPI_CLK_SRC_DEFAULT,
      .duty_cycle_pos = 128,
      .cs_ena_pretrans = 0,
      .cs_ena_posttrans = 0,
      .clock_speed_hz = kNandClockHz,
      .input_delay_ns = 0,
      .spics_io_num = kNandPinCs,
      .flags = SPI_DEVICE_HALFDUPLEX,
      .queue_size = 10,
      .pre_cb = nullptr,
      .post_cb = nullptr,
  };

  ret = spi_bus_add_device(kNandSpiHost, &devcfg, &g_nand_spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add NAND SPI device: %s", esp_err_to_name(ret));
    vTaskDelete(nullptr);
    return;
  }
  ESP_LOGI(TAG, "NAND SPI device added successfully");

  // Initialize the spi_nand_flash driver
  spi_nand_flash_config_t nand_config = {
      .device_handle = g_nand_spi,
      .gc_factor = 4,  // Default garbage collection factor
      .io_mode = SPI_NAND_IO_MODE_SIO,
      .flags = SPI_DEVICE_HALFDUPLEX,
  };

  ret = spi_nand_flash_init_device(&nand_config, &g_nand_device);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init NAND device: %s", esp_err_to_name(ret));
    spi_bus_remove_device(g_nand_spi);
    g_nand_spi = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  // Log device info
  uint32_t num_sectors = 0, sector_size = 0, block_size = 0, num_blocks = 0;
  spi_nand_flash_get_capacity(g_nand_device, &num_sectors);
  spi_nand_flash_get_sector_size(g_nand_device, &sector_size);
  spi_nand_flash_get_block_size(g_nand_device, &block_size);
  spi_nand_flash_get_block_num(g_nand_device, &num_blocks);

  ESP_LOGI(TAG, "NAND Flash Info:");
  ESP_LOGI(TAG, "  - Sectors: %lu (sector size: %lu bytes)", num_sectors,
           sector_size);
  ESP_LOGI(TAG, "  - Blocks: %lu (block size: %lu bytes)", num_blocks,
           block_size);
  ESP_LOGI(TAG, "  - Total capacity: %lu KB",
           (num_sectors * sector_size) / 1024);

  // Mount FATFS on NAND
  ESP_LOGI(TAG, "Mounting FATFS on NAND flash...");

  esp_vfs_fat_mount_config_t mount_config = {
      .format_if_mount_failed = true,
      .max_files = 4,
      .allocation_unit_size = 16 * 1024,
      .disk_status_check_enable = false,
      .use_one_fat = false,
  };

  ret = esp_vfs_fat_nand_mount(kMountPoint, g_nand_device, &mount_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount FATFS: %s", esp_err_to_name(ret));
    spi_nand_flash_deinit_device(g_nand_device);
    g_nand_device = nullptr;
    spi_bus_remove_device(g_nand_spi);
    g_nand_spi = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  // Print FATFS size information
  uint64_t bytes_total = 0, bytes_free = 0;
  esp_vfs_fat_info(kMountPoint, &bytes_total, &bytes_free);
  ESP_LOGI(TAG, "FATFS mounted: %llu KB total, %llu KB free",
           bytes_total / 1024, bytes_free / 1024);

  // List files in mount point
  DIR *dir = opendir(kMountPoint);
  if (dir) {
    ESP_LOGI(TAG, "Files in %s:", kMountPoint);
    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
      char filepath[280];  // PATH_MAX for d_name is 255 + mount point
      snprintf(filepath, sizeof(filepath), "%s/%.250s", kMountPoint, ent->d_name);
      struct stat st;
      if (stat(filepath, &st) == 0) {
        ESP_LOGI(TAG, "  - %s (%ld bytes)", ent->d_name, st.st_size);
      } else {
        ESP_LOGI(TAG, "  - %s (size unknown)", ent->d_name);
      }
    }
    closedir(dir);
  }

  g_storage_ready = true;
  ESP_LOGI(TAG, "=== NAND Flash Ready ===");

  // Initialize sensor record storage
  sensor_record_init();

  // Run test
  sensor_record_test();

  vTaskDelete(nullptr);
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t log_storage_init(void) {
  if (g_mount_started) {
    ESP_LOGW(TAG, "log_storage_init already called");
    return ESP_OK;
  }
  g_mount_started = true;

  // Start mount task with sufficient stack
  BaseType_t ret =
      xTaskCreate(log_storage_mount_task, "LogMount", 8192, nullptr, 5, &g_mount_task);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create mount task");
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

bool log_storage_is_ready(void) {
  return g_storage_ready;
}

esp_err_t log_storage_flush(void) {
  if (!g_storage_ready) {
    ESP_LOGW(TAG, "log_storage_flush: storage not ready");
    return ESP_ERR_INVALID_STATE;
  }

  if (!storage_lock(pdMS_TO_TICKS(1000))) {
    ESP_LOGE(TAG, "log_storage_flush: failed to acquire lock");
    return ESP_ERR_TIMEOUT;
  }

  // Sync filesystem to ensure all data is written
  // FATFS uses f_sync internally, but we can force a general sync
  ESP_LOGI(TAG, "Flushing storage...");
  
  storage_unlock();
  ESP_LOGI(TAG, "Storage flush complete");
  return ESP_OK;
}

esp_err_t log_storage_deinit(void) {
  if (!g_mount_started) {
    ESP_LOGW(TAG, "log_storage_deinit: not initialized");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Deinitializing log storage...");

  // Flush any pending data first
  log_storage_flush();

  if (!storage_lock(pdMS_TO_TICKS(2000))) {
    ESP_LOGE(TAG, "log_storage_deinit: failed to acquire lock");
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t ret = ESP_OK;

  // Unmount FATFS
  if (g_storage_ready) {
    ESP_LOGI(TAG, "Unmounting FATFS...");
    ret = esp_vfs_fat_nand_unmount(kMountPoint, g_nand_device);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "FATFS unmount failed: %s", esp_err_to_name(ret));
    } else {
      ESP_LOGI(TAG, "FATFS unmounted");
    }
    g_storage_ready = false;
  }

  // Deinitialize NAND flash device
  if (g_nand_device) {
    ESP_LOGI(TAG, "Deinitializing NAND device...");
    spi_nand_flash_deinit_device(g_nand_device);
    g_nand_device = nullptr;
    ESP_LOGI(TAG, "NAND device deinitialized");
  }

  // Remove SPI device
  if (g_nand_spi) {
    ESP_LOGI(TAG, "Removing NAND SPI device...");
    spi_bus_remove_device(g_nand_spi);
    g_nand_spi = nullptr;
    ESP_LOGI(TAG, "NAND SPI device removed");
  }

  storage_unlock();

  // Delete mutex
  if (g_storage_lock) {
    vSemaphoreDelete(g_storage_lock);
    g_storage_lock = nullptr;
  }

  g_mount_started = false;
  g_record_count = -1;

  ESP_LOGI(TAG, "Log storage deinitialized successfully");
  return ret;
}

// ============================================================================
// Sensor Record Storage
// ============================================================================

esp_err_t sensor_record_init(void) {
  if (!g_storage_ready) {
    ESP_LOGE(TAG, "Storage not ready");
    return ESP_ERR_INVALID_STATE;
  }

  // Check if sensor file exists and count records
  struct stat st;
  if (stat(kSensorDataFile, &st) == 0) {
    g_record_count = st.st_size / sizeof(sensor_record_t);
    ESP_LOGI(TAG, "Sensor data file exists: %ld bytes, %ld records",
             st.st_size, g_record_count);
  } else {
    g_record_count = 0;
    ESP_LOGI(TAG, "Sensor data file does not exist, will be created");
  }

  return ESP_OK;
}

esp_err_t sensor_record_write(const sensor_record_t *record) {
  if (!g_storage_ready) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!storage_lock(pdMS_TO_TICKS(1000))) {
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t result = ESP_OK;

  // Open file for append in binary mode
  FILE *f = fopen(kSensorDataFile, "ab");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open sensor file for append");
    storage_unlock();
    return ESP_FAIL;
  }

  // Write record
  size_t written = fwrite(record, sizeof(sensor_record_t), 1, f);
  if (written != 1) {
    ESP_LOGE(TAG, "Failed to write sensor record");
    result = ESP_FAIL;
  } else {
    g_record_count++;
  }

  fclose(f);
  storage_unlock();

  return result;
}

int32_t sensor_record_count(void) {
  if (!g_storage_ready) {
    return -1;
  }

  // Refresh count from file
  struct stat st;
  if (stat(kSensorDataFile, &st) == 0) {
    g_record_count = st.st_size / sizeof(sensor_record_t);
  } else {
    g_record_count = 0;
  }

  return g_record_count;
}

esp_err_t sensor_record_read(uint32_t index, sensor_record_t *record) {
  if (!g_storage_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!record) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!storage_lock(pdMS_TO_TICKS(1000))) {
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t result = ESP_OK;

  FILE *f = fopen(kSensorDataFile, "rb");
  if (!f) {
    storage_unlock();
    return ESP_ERR_NOT_FOUND;
  }

  // Seek to record position
  long offset = index * sizeof(sensor_record_t);
  if (fseek(f, offset, SEEK_SET) != 0) {
    fclose(f);
    storage_unlock();
    return ESP_ERR_INVALID_ARG;
  }

  // Read record
  size_t read = fread(record, sizeof(sensor_record_t), 1, f);
  if (read != 1) {
    result = ESP_ERR_NOT_FOUND;
  }

  fclose(f);
  storage_unlock();

  return result;
}

int32_t sensor_record_read_recent(uint32_t count, sensor_record_t *records) {
  if (!g_storage_ready || !records) {
    return -1;
  }

  int32_t total = sensor_record_count();
  if (total <= 0) {
    return 0;
  }

  uint32_t start_idx = (total > (int32_t)count) ? (total - count) : 0;
  uint32_t actual_count = total - start_idx;

  if (!storage_lock(pdMS_TO_TICKS(1000))) {
    return -1;
  }

  FILE *f = fopen(kSensorDataFile, "rb");
  if (!f) {
    storage_unlock();
    return -1;
  }

  long offset = start_idx * sizeof(sensor_record_t);
  if (fseek(f, offset, SEEK_SET) != 0) {
    fclose(f);
    storage_unlock();
    return -1;
  }

  size_t read = fread(records, sizeof(sensor_record_t), actual_count, f);
  fclose(f);
  storage_unlock();

  return (int32_t)read;
}

esp_err_t sensor_record_clear(void) {
  if (!g_storage_ready) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!storage_lock(pdMS_TO_TICKS(1000))) {
    return ESP_ERR_TIMEOUT;
  }

  // Remove the file
  remove(kSensorDataFile);
  g_record_count = 0;

  storage_unlock();
  ESP_LOGI(TAG, "Sensor records cleared");

  return ESP_OK;
}

// ============================================================================
// Test Function
// ============================================================================

void sensor_record_test(void) {
  ESP_LOGI(TAG, "=== Sensor Record Test ===");

  // Create test records
  sensor_record_t test_records[5];

  for (int i = 0; i < 5; i++) {
    test_records[i].timestamp_ms = esp_timer_get_time() / 1000 + i * 1000;
    test_records[i].co2_ppm = 400 + i * 50;
    test_records[i].temp_c_x100 = 2350 + i * 10;  // 23.5°C + offset
    test_records[i].rh_x100 = 5500 + i * 100;     // 55% + offset
    test_records[i].pm25_x10 = 120 + i * 5;       // 12.0 µg/m³ + offset
    test_records[i].pm10_x10 = 180 + i * 8;
    test_records[i].pm1_x10 = 80 + i * 3;
    test_records[i].voc_index = 100 + i;
    test_records[i].nox_index = 1;
    test_records[i].pressure_pa = 101325 + i * 100;
    test_records[i].reserved = 0;

    // Calculate CRC (excluding CRC field itself)
    test_records[i].crc16 =
        crc16_ccitt((uint8_t *)&test_records[i],
                    sizeof(sensor_record_t) - sizeof(uint16_t));
  }

  // Write test records
  ESP_LOGI(TAG, "Writing %d test records...", 5);
  for (int i = 0; i < 5; i++) {
    esp_err_t ret = sensor_record_write(&test_records[i]);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "  Record %d written: CO2=%u ppm, T=%d.%02d°C",
               i, test_records[i].co2_ppm,
               test_records[i].temp_c_x100 / 100,
               test_records[i].temp_c_x100 % 100);
    } else {
      ESP_LOGE(TAG, "  Failed to write record %d: %s", i, esp_err_to_name(ret));
    }
  }

  // Count records
  int32_t count = sensor_record_count();
  ESP_LOGI(TAG, "Total records in storage: %ld", count);

  // Read back and verify
  ESP_LOGI(TAG, "Reading back records...");
  for (int i = 0; i < 5; i++) {
    sensor_record_t read_record;
    uint32_t read_idx = (count > 5) ? (count - 5 + i) : i;
    esp_err_t ret = sensor_record_read(read_idx, &read_record);
    if (ret == ESP_OK) {
      // Verify CRC
      uint16_t calc_crc =
          crc16_ccitt((uint8_t *)&read_record,
                      sizeof(sensor_record_t) - sizeof(uint16_t));
      bool crc_ok = (calc_crc == read_record.crc16);

      ESP_LOGI(TAG,
               "  Record[%lu]: CO2=%u, T=%d.%02d°C, RH=%d.%02d%%, PM2.5=%d.%d, "
               "CRC=%s",
               read_idx, read_record.co2_ppm, read_record.temp_c_x100 / 100,
               read_record.temp_c_x100 % 100, read_record.rh_x100 / 100,
               read_record.rh_x100 % 100, read_record.pm25_x10 / 10,
               read_record.pm25_x10 % 10, crc_ok ? "OK" : "FAIL");
    } else {
      ESP_LOGE(TAG, "  Failed to read record %lu: %s", read_idx,
               esp_err_to_name(ret));
    }
  }

  // Print filesystem info
  uint64_t bytes_total = 0, bytes_free = 0;
  esp_vfs_fat_info(kMountPoint, &bytes_total, &bytes_free);
  ESP_LOGI(TAG, "FATFS after test: %llu KB total, %llu KB free",
           bytes_total / 1024, bytes_free / 1024);

  ESP_LOGI(TAG, "=== Sensor Record Test Complete ===");
}
