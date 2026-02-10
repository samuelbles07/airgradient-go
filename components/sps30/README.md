# SPS30 Particulate Matter Sensor Component

SPS30 I2C driver for ESP-IDF.

## Features

- I2C communication (100 kHz)
- Read PM1.0, PM2.5, PM4.0, PM10 mass concentration (µg/m³)
- Read PM0.5, PM1.0, PM2.5, PM4.0, PM10 number concentration (#/cm³)
- Typical particle size measurement
- CRC-8 verification for all data
- Start/stop measurement mode
- Sensor reset functionality

## API

- `sps30_init()` – Initialize on I2C bus
- `sps30_start_measurement()` – Begin taking readings
- `sps30_stop_measurement()` – Stop measurements
- `sps30_read_measurement()` – Get PM and particle data
- `sps30_reset()` – Soft-reset sensor
- `sps30_delete()` – Cleanup and remove device

## Example Usage

```c
#include "sps30.h"

sps30_config_t sps_cfg = {
    .i2c_address = 0x69,
    .i2c_clock_speed = 100000,
};

sps30_handle_t sps_handle;
esp_err_t ret = sps30_init(i2c_master_bus, &sps_cfg, &sps_handle);

if (ret == ESP_OK) {
    sps30_start_measurement(sps_handle);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    sps30_measurement_t data;
    sps30_read_measurement(sps_handle, &data);
    
    printf("PM2.5: %.2f µg/m³\n", data.pm2p5_mass);
    printf("PM10: %.2f µg/m³\n", data.pm10p0_mass);
}
```
