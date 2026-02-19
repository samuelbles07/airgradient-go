# STCC4 CO2 Sensor Library

A complete C/C++ driver library for the Sensirion STCC4 thermal conductivity-based CO2 sensor with I2C interface.

## Component Structure

```
stcc4/
├── CMakeLists.txt          # ESP-IDF component configuration
├── idf_component.yml       # Component metadata
├── README.md               # This file
├── include/
│   └── stcc4.h             # Public API header
├── src/
│   └── stcc4.cpp           # Implementation
└── example/
    └── stcc4_example.cpp   # Usage examples
```

## Features

- ✅ Continuous measurement mode (1s sampling interval)
- ✅ Single-shot on-demand measurements (configurable 5-600s interval)
- ✅ Temperature and humidity compensation (RHT)
- ✅ Pressure compensation for improved accuracy
- ✅ Sleep mode support for low-power applications
- ✅ Self-test and diagnostics
- ✅ Forced recalibration (FRC) for sensor calibration
- ✅ CRC-8 checksum verification for all I2C transactions
- ✅ Complete error handling and logging
- ✅ Non-blocking I2C communication

## Hardware Specifications

| Parameter | Value |
|-----------|-------|
| I2C Address | 0x64 (default) or 0x65 (configurable) |
| I2C Speed | 100 kHz - 1 MHz (400 kHz standard) |
| Supply Voltage | 2.7 - 5.5 V (3.3V recommended) |
| Measurement Range | 380 - 32,000 ppm CO2 |
| Accuracy | ±(100 ppm + 10%) @ 400-5000 ppm |
| Response Time | ~20s (63% step change from 2000→400 ppm) |
| Power Consumption | 950 µA (continuous), 100 µA (single-shot), 1 µA (sleep) |

## Installation

The library is already installed as a component in `/components/stcc4/`.

To use it in your project, simply include:

```c
#include "stcc4.h"
```

And link against the stcc4 component in your CMakeLists.txt:

```cmake
idf_component_register(
    SRCS "your_file.cpp"
    REQUIRES stcc4
)
```

## Quick Start

### 1. Initialize the sensor

```c
#include "stcc4.h"
#include "driver/i2c_master.h"

stcc4_dev_t sensor;
i2c_master_bus_handle_t bus_handle;

// Create I2C bus (see ESP-IDF documentation)
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_6,
    .sda_io_num = GPIO_NUM_5,
    .glitch_filter_en = true,
};
i2c_new_master_bus(&i2c_mst_config, &bus_handle);

// Initialize sensor
stcc4_init(&sensor, bus_handle, STCC4_I2C_ADDR_DEFAULT);

// Optional: Verify sensor communication (with retry)
uint32_t product_id = 0;
for (int i = 0; i < 3; i++) {
    vTaskDelay(pdMS_TO_TICKS(50));
    if (stcc4_get_product_id(&sensor, &product_id, NULL) == ESP_OK) {
        printf("STCC4 Product ID: 0x%08X\n", product_id);
        break;
    }
}
```

### 2. Continuous Measurement

```c
stcc4_measurement_t measurement;

// Start continuous measurement
stcc4_start_continuous_measurement(&sensor);

// Wait for first measurement to be ready
vTaskDelay(pdMS_TO_TICKS(1500));

// Read every second
while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    if (stcc4_read_measurement(&sensor, &measurement) == ESP_OK) {
        printf("CO2: %u ppm, Temp: %.2f°C, RH: %.1f%%\n",
               measurement.co2_ppm,
               measurement.temperature_c,
               measurement.humidity_rh);
    }
}

// Stop when done
stcc4_stop_continuous_measurement(&sensor);
```

### 3. Single-Shot Measurement (Low Power)

```c
stcc4_measurement_t measurement;

while (true) {
    // Wake from sleep
    stcc4_exit_sleep_mode(&sensor);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Perform measurement
    stcc4_measure_single_shot(&sensor);
    
    // Read result
    if (stcc4_read_measurement(&sensor, &measurement) == ESP_OK) {
        printf("CO2: %u ppm, Temp: %.2f°C, RH: %.1f%%\n",
               measurement.co2_ppm,
               measurement.temperature_c,
               measurement.humidity_rh);
    }
    
    // Sleep for 10 seconds
    stcc4_enter_sleep_mode(&sensor);
    vTaskDelay(pdMS_TO_TICKS(10000));
}
```

### 4. RHT Compensation (External Sensor)

If you have an external temperature/humidity sensor (not connected to STCC4's internal interface):

```c
// Update compensation values (e.g., from SHT4x)
float temperature_c = 22.5;
float humidity_rh = 45.0;

stcc4_set_rht_compensation(&sensor, temperature_c, humidity_rh);
```

### 5. Pressure Compensation

```c
// Set pressure (improves accuracy, optional)
// Valid range: 40,000 - 110,000 Pa
// Default: 101,300 Pa (sea level)
stcc4_set_pressure_compensation(&sensor, 101300);
```

### 6. Sensor Diagnostics

```c
stcc4_self_test_result_t test;

if (stcc4_perform_self_test(&sensor, &test) == ESP_OK) {
    printf("Self-test passed\n");
} else {
    printf("Self-test failed: 0x%04X\n", test.raw_value);
    printf("  Supply voltage: %s\n", test.supply_voltage_ok ? "OK" : "ERROR");
    printf("  SHT connected: %s\n", test.sht_connected ? "YES" : "NO");
    printf("  Memory: %s\n", test.memory_ok ? "OK" : "ERROR");
}
```

### 7. Sensor Conditioning (After Long Idle)

For improved accuracy after >3 hours idle:

```c
stcc4_perform_conditioning(&sensor);

// Then start measurements
stcc4_start_continuous_measurement(&sensor);
```

### 8. Forced Recalibration (FRC)

Calibrate sensor to a known CO2 concentration:

```c
int16_t correction;

// Ensure sensor has been running for ≥30 seconds
// Then call FRC with target concentration
stcc4_perform_forced_recalibration(&sensor, 400, &correction);

printf("FRC applied: %d ppm correction\n", correction);
```

## API Reference

### Initialization

- `stcc4_init()` - Initialize sensor on I2C bus
- `stcc4_deinit()` - Deinitialize sensor

### Measurement Modes

- `stcc4_start_continuous_measurement()` - Start 1s interval measurements
- `stcc4_stop_continuous_measurement()` - Stop continuous mode
- `stcc4_measure_single_shot()` - Single on-demand measurement
- `stcc4_read_measurement()` - Read latest measurement data

### Power Management

- `stcc4_enter_sleep_mode()` - Enter low-power sleep (1 µA)
- `stcc4_exit_sleep_mode()` - Wake from sleep
- `stcc4_perform_conditioning()` - Condition sensor after idle period

### Compensation & Calibration

- `stcc4_set_rht_compensation()` - Set external RHT values
- `stcc4_set_pressure_compensation()` - Set atmospheric pressure
- `stcc4_perform_forced_recalibration()` - Calibrate to known CO2 level

### Diagnostics & Control

- `stcc4_get_product_id()` - Verify sensor and get serial number
- `stcc4_perform_self_test()` - Run self-test
- `stcc4_enable_testing_mode()` - Enable testing mode (pauses ASC)
- `stcc4_disable_testing_mode()` - Disable testing mode
- `stcc4_factory_reset()` - Reset to factory defaults
- `stcc4_soft_reset()` - Reset via I2C general call (not yet supported)

### Utilities

- `stcc4_calculate_crc()` - Calculate CRC-8 for data
- `stcc4_verify_crc()` - Verify CRC-8 checksum

## Data Structures

### `stcc4_measurement_t`

```c
typedef struct {
    uint16_t co2_ppm;           // CO2 concentration in ppm
    uint16_t temperature_raw;   // Raw temperature value
    uint16_t humidity_raw;      // Raw relative humidity value
    uint16_t sensor_status;     // Sensor status flags
    
    float temperature_c;        // Temperature in Celsius
    float humidity_rh;          // Relative humidity in %RH
    bool testing_mode;          // Sensor is in testing mode
} stcc4_measurement_t;
```

### `stcc4_self_test_result_t`

```c
typedef struct {
    bool supply_voltage_ok;     // Supply voltage within spec
    bool debug_bits;            // Debug information
    bool sht_connected;         // SHT sensor connected to STCC4
    bool memory_ok;             // Internal memory OK
    uint16_t raw_value;         // Raw self-test result
} stcc4_self_test_result_t;
```

## Important Notes

### Automatic Self-Calibration (ASC)

The sensor includes a built-in automatic self-calibration algorithm that assumes:
- Sensor is exposed to fresh air (~400 ppm CO2) at least once per week
- ASC state is saved every 2 hours (continuous) or after 720 measurements (single-shot)

To use ASC:
- Ensure the sensor sees ~400 ppm air regularly
- Do NOT expose to artificially low CO2 concentrations (<400 ppm)

To disable ASC temporarily, use testing mode:
```c
stcc4_enable_testing_mode(&sensor);
// ... run tests ...
stcc4_disable_testing_mode(&sensor);
```

### Warm-Up Time

- **Initial Power-Up**: Full accuracy after 12 hours in continuous mode
- **After 3+ Hours Idle**: Full accuracy after 1 hour (or use `stcc4_perform_conditioning()`)
- **Normal Operation**: Ready within 1 second

### I2C Communication Notes

- All data transfers use CRC-8 checksums (automatically verified)
- Clock stretching is NOT supported
- Recommended I2C speed: 400 kHz (standard mode)
- Sensor responds with NACK if no measurement data available

### Temperature/Humidity Integration

The sensor requires external temperature and humidity compensation for ±(100 ppm + 10%) accuracy.

**Option 1**: STCC4 controls SHT4x via dedicated I2C interface (SDA_C/SCL_C pins)
- Internal automatic compensation
- No API calls needed

**Option 2**: Manual compensation via `stcc4_set_rht_compensation()`
- Use external SHT4x or other RHT sensor
- Update values periodically (1-2x per measurement interval recommended)

## Logging

The library uses ESP-IDF logging macros. Configure log level:

```c
// In menuconfig or code:
esp_log_level_set("STCC4", ESP_LOG_INFO);
```

## Example Project Structure

```
main/
  ├── CMakeLists.txt
  ├── app_main.cpp (includes stcc4.h)
  └── ...

components/
  └── stcc4/  (this library)
      ├── CMakeLists.txt
      ├── stcc4.h
      ├── stcc4.cpp
      └── README.md
```

## Compilation

Build with ESP-IDF:

```bash
source ~/esp/v5.5.1/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

## Performance Characteristics

| Operation | Time | Current |
|-----------|------|---------|
| Continuous measurement (per cycle) | 1s cycle | 950 µA avg |
| Single-shot measurement | 500 ms | 100 µA avg (10s interval) |
| Sleep mode | - | 1 µA |
| Self-conditioning | 22 ms | - |
| Self-test | 360 ms | - |
| Factory reset | 90 ms | - |
| FRC calibration | 90 ms | - |

## Troubleshooting

### I2C Communication Issues

```c
// Verify sensor is present and responsive
uint32_t product_id;
uint64_t serial_num;
if (stcc4_get_product_id(&sensor, &product_id, &serial_num) == ESP_OK) {
    printf("Sensor found: ID=0x%08X, SN=0x%016llX\n", product_id, serial_num);
} else {
    printf("Sensor not responding - check I2C connections\n");
}
```

### Inaccurate Readings

1. **Check RHT Compensation**: Ensure temperature/humidity values are accurate
2. **Verify Pressure**: Set correct atmospheric pressure if needed
3. **Allow Warm-Up**: Wait 20+ seconds for stabilization
4. **Check ASC**: Sensor hasn't been exposed to fresh air (~400 ppm) recently
5. **Run Self-Test**: `stcc4_perform_self_test()` to check sensor health

### CRC Errors

Usually indicate I2C bus noise:
- Check pull-up resistors (1.5-2.2 kΩ typical)
- Reduce I2C speed if needed
- Check cable length and quality
- Add series resistors if bus is long (>30cm)

## Reference

- Sensirion STCC4 Datasheet: See [documents/STCC4.md](../../documents/STCC4.md)
- ESP-IDF I2C Documentation: https://docs.espressif.com/projects/esp-idf/
- Sensirion Website: https://www.sensirion.com/

## License

This library is provided as-is for use with the AirGradient CO2 monitoring system.

## Support

For issues or questions, refer to the datasheet or check sensor logs:

```c
esp_log_level_set("STCC4", ESP_LOG_DEBUG);
```

## Changelog

### v1.0.0 (2025-01-08)

- Initial release
- Full STCC4 sensor support
- All measurement modes and compensation features
- Comprehensive error handling and logging
- CRC verification for data integrity
