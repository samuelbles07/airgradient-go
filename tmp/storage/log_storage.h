#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize log storage and NAND flash
esp_err_t log_storage_init(void);

// Check if SPIFFS is mounted and ready
bool log_storage_is_ready(void);

// Flush any pending writes and sync to storage
esp_err_t log_storage_flush(void);

// Deinitialize log storage (unmount FATFS, release resources)
esp_err_t log_storage_deinit(void);

// ============================================================================
// Binary Sensor Record Storage
// ============================================================================

// Packed binary record for storing sensor data (32 bytes fixed size)
typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;  // Timestamp in milliseconds since boot
  uint16_t co2_ppm;       // CO2 in ppm (0-65535)
  int16_t temp_c_x100;    // Temperature * 100 (e.g., 2350 = 23.50°C)
  int16_t rh_x100;        // Relative humidity * 100 (e.g., 5500 = 55.00%)
  uint16_t pm25_x10;      // PM2.5 * 10 (e.g., 123 = 12.3 µg/m³)
  uint16_t pm10_x10;      // PM10 * 10
  uint16_t pm1_x10;       // PM1.0 * 10
  uint16_t voc_index;     // VOC index (1-500)
  uint16_t nox_index;     // NOx index (1-500)
  uint32_t pressure_pa;   // Pressure in Pascals (e.g., 101325)
  uint16_t reserved;      // Reserved for alignment/future use
  uint16_t crc16;         // CRC16 for data integrity
} sensor_record_t;

// Initialize sensor record storage (creates/opens sensor_data.bin)
esp_err_t sensor_record_init(void);

// Write a sensor record to flash (appends to file)
esp_err_t sensor_record_write(const sensor_record_t *record);

// Get total number of records stored
int32_t sensor_record_count(void);

// Read a sensor record by index (0 = oldest)
esp_err_t sensor_record_read(uint32_t index, sensor_record_t *record);

// Read the most recent N records (returns actual count read)
int32_t sensor_record_read_recent(uint32_t count, sensor_record_t *records);

// Clear all sensor records
esp_err_t sensor_record_clear(void);

// ============================================================================
// Test Functions
// ============================================================================

// Test function: Write sample sensor data and read it back
void sensor_record_test(void);

#ifdef __cplusplus
}
#endif
