# STCC4 Library Examples

This directory contains usage examples for the STCC4 CO2 sensor library.

## Examples Included

### 1. Continuous Measurement Mode
**Function**: `stcc4_continuous_example()`

Demonstrates continuous CO2 measurement with 1-second sampling interval.

**Features**:
- Sensor initialization and verification
- Self-test execution
- Sensor conditioning
- Continuous measurement mode
- Reading CO2, temperature, and humidity data

**Use Case**: Applications requiring real-time CO2 monitoring.

---

### 2. Single-Shot Low-Power Mode
**Function**: `stcc4_single_shot_example()`

Demonstrates on-demand measurements with sleep mode for battery-powered applications.

**Features**:
- Wake from sleep
- Single-shot measurement
- Data reading
- Sleep mode entry (1 µA consumption)

**Use Case**: Battery-powered devices with infrequent measurements.

**Power Consumption**:
- Active: ~100 µA average (10s interval)
- Sleep: 1 µA

---

### 3. RHT Compensation Mode
**Function**: `stcc4_with_rht_compensation_example()`

Shows how to use external temperature and humidity values for improved accuracy.

**Features**:
- Manual RHT compensation update
- Continuous measurement with compensation
- Integration with external RHT sensor

**Use Case**: When using external SHT4x or other RHT sensor not connected to STCC4's internal interface.

**Note**: This is only needed if the SHT4x is NOT connected to STCC4's SDA_C/SCL_C pins.

---

### 4. Forced Recalibration (FRC)
**Function**: `stcc4_frc_example()`

Demonstrates sensor calibration to a known CO2 concentration.

**Features**:
- Sensor stabilization (30+ seconds)
- FRC execution
- Correction value retrieval

**Use Case**: Initial calibration or periodic recalibration to a known reference gas.

**Requirements**:
- Stable environment
- Known CO2 concentration (e.g., 400 ppm fresh air)
- Minimum 30 seconds operation before FRC

---

## How to Use Examples

### Option 1: Copy to Your Project
Copy the example functions to your main application file:

```cpp
#include "stcc4.h"

extern "C" void app_main(void) {
    // Create I2C bus
    i2c_master_bus_handle_t bus_handle;
    // ... initialize I2C bus ...
    
    // Run example
    xTaskCreate(stcc4_continuous_example, "stcc4_task", 4096, bus_handle, 5, NULL);
}
```

### Option 2: Reference Implementation
Use these examples as reference when implementing your own sensor integration.

---

## Prerequisites

Before using these examples, you must:

1. **Initialize I2C Bus**
   ```cpp
   i2c_master_bus_config_t i2c_mst_config = {
       .clk_source = I2C_CLK_SRC_DEFAULT,
       .i2c_port = I2C_NUM_0,
       .scl_io_num = GPIO_NUM_6,
       .sda_io_num = GPIO_NUM_5,
       .glitch_filter_en = true,
   };
   i2c_master_bus_handle_t bus_handle;
   i2c_new_master_bus(&i2c_mst_config, &bus_handle);
   ```

2. **Connect Hardware**
   - SCL (GPIO 6) → STCC4 Pin 1
   - SDA (GPIO 5) → STCC4 Pin 5
   - 3.3V → STCC4 Pin 7 (VDD)
   - GND → STCC4 Pin 4 (VSS)
   - GND → STCC4 Pin 3 (ADDR) for default address 0x64

---

## Example Output

### Continuous Mode Output
```
I (1234) STCC4_EXAMPLE: STCC4 sensor found - Product ID: 0x0901018A
I (1567) STCC4_EXAMPLE: Self-test: PASSED
I (1589) STCC4_EXAMPLE: Conditioning sensor...
I (23600) STCC4_EXAMPLE: Starting continuous measurement...
[0] CO2: 450 ppm, Temp: 22.50°C, RH: 45.2%, Status: 0x0000
[1] CO2: 452 ppm, Temp: 22.51°C, RH: 45.3%, Status: 0x0000
[2] CO2: 448 ppm, Temp: 22.49°C, RH: 45.1%, Status: 0x0000
...
```

### Single-Shot Mode Output
```
I (2345) STCC4_EXAMPLE: Single-shot measurement mode (low power)
Cycle 0: CO2: 445 ppm, Temp: 22.30°C, RH: 44.8%
I (2890) STCC4_EXAMPLE: Sleeping...
Cycle 1: CO2: 447 ppm, Temp: 22.32°C, RH: 44.9%
I (13400) STCC4_EXAMPLE: Sleeping...
...
```

---

## Troubleshooting

### Sensor Not Responding
```
E (1234) STCC4_EXAMPLE: Failed to get product ID - sensor not responding
```
**Solution**: Check I2C connections, pull-up resistors, and sensor power supply.

### CRC Errors
```
W (2345) STCC4: CRC error reading measurement data
```
**Solution**: 
- Check I2C bus noise and cable length
- Verify proper pull-up resistors (1.5-2.2 kΩ)
- Reduce I2C speed if necessary

### Invalid Readings
**Solution**:
- Allow 20+ seconds warm-up time
- Use `stcc4_perform_conditioning()` after >3 hours idle
- Verify RHT compensation is working
- Run self-test: `stcc4_perform_self_test()`

---

## Building Examples

These examples are provided as reference code. To build with your project:

1. **Include the header**:
   ```cpp
   #include "stcc4.h"
   ```

2. **Add to CMakeLists.txt**:
   ```cmake
   idf_component_register(
       SRCS "your_main.cpp"
       REQUIRES stcc4
   )
   ```

3. **Build**:
   ```bash
   idf.py build
   ```

---

## Additional Resources

- **Component README**: [../README.md](../README.md) - Full library documentation
- **API Reference**: [../include/stcc4.h](../include/stcc4.h) - Complete API
- **Datasheet**: [../../../documents/STCC4.md](../../../documents/STCC4.md) - Sensor specifications

---

## Notes

- All examples use FreeRTOS tasks and delay functions
- Examples are designed for ESP32-C6 but work on other ESP32 variants
- Adjust GPIO pins according to your hardware design
- Examples assume sensor is at default I2C address 0x64
- Recommended to run self-test during initialization
- Allow proper warm-up time for accurate readings

For detailed API documentation, see [../include/stcc4.h](../include/stcc4.h).
