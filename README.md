# AirGradient Go

Firmware for the AirGradient Go portable air-quality monitor.

This repository is an ESP-IDF 5.5 C/C++ project targeting `esp32c5`. The current application is centered around `main/go.cpp`, which brings together the sensor stack, e-paper UI, BLE, GPS, charger/battery handling, and NAND-backed data logging for a battery-powered handheld device.

## What The Firmware Does

- Reads air-quality and environment data from particulate, CO2, VOC/NOx, and pressure sensors.
- Uses GPS data for portable tracking sessions and location-aware measurement logs.
- Renders a monochrome e-paper dashboard and menu-driven device UI.
- Streams live data and device state over BLE, and can export stored history over BLE as well.
- Stores local records on external SPI NAND via FATFS for later BLE export.

## Current App Structure

### Main application

- `main/go.cpp` - active firmware entrypoint and main controller/state machine.
- `main/go_constants.h` - hardware pins, timing, and device constants.
- `main/ble_stream.*` - custom BLE GATT service for measures, status, config, and history export.
- `main/gps_service.*` - UART/NMEA GPS task and parsed fix/time state.
- `main/nand_storage_service.*` - async record logging, reads, erase, and storage helpers.
- `main/button_service.*` - physical button and CAP1203 touch-input handling.

### UI

- `main/dashboard/` - e-paper rendering, layout logic, partial updates, and display driver glue.
- `AirGradient-Go-Complete-UI-Spec.md` - detailed on-device UI specification.
- `airgradient-go-ui-reference/` - copied website simulator snapshots used as UI reference.

### Reusable components

- `components/PMSensor/` - particulate sensor abstractions and SPS30/PMS5003-family drivers.
- `components/CO2Sensor/` - CO2 sensor drivers, including `STCC4Sensor` and other variants.
- `components/TVOCNOxSensor/` - SGP41 VOC/NOx support.
- `components/dps368/` - pressure sensor driver.
- `components/bq25629/` - charger/power-management driver.
- `components/libnmea/`, `components/u8g2/`, `components/edp-ssd1680x/` - protocol and display support.

### Managed dependencies

- `managed_components/h2zero__esp-nimble-cpp/` - NimBLE C++ wrapper used for BLE.
- `managed_components/espressif__spi_nand_flash/` and `managed_components/espressif__dhara/` - SPI NAND + Dhara stack.
- `dependencies.lock` and `main/idf_component.yml` - component-manager dependency definitions.

## Runtime Features

The main firmware implements a device state machine with `Idle`, `Inactive`, `Tracking`, and `Shutdown` states.

Notable runtime behavior visible in the codebase:

- Persistent UI settings stored in NVS.
- Tracking sessions with route/session IDs kept across sleep using RTC memory.
- BLE advertising named like `AirGradientGo-<serial>`.
- Local history export over BLE.
- Charger watchdog handling, external watchdog kicking, and deep-sleep aware startup.
- Menu screens for home, settings, settings choices, tag list, about, confirm, and shutdown views.

## Hardware/Platform Notes

- Target MCU: `ESP32-C5` (`sdkconfig`, `dependencies.lock`).
- Framework: `ESP-IDF 5.5.2`.
- Display: Good Display `GDEY0213B74` 2.13" monochrome e-paper panel driven through SSD1680-compatible code.
- Input: physical buttons plus CAP1203 touch buttons.
- Storage: external SPI NAND mounted through FATFS for measurement logs.
- Connectivity: BLE and UART GPS.

The active app initializes support for these sensor families when present:

- PM: Sensirion SPS30.
- CO2: STCC4, plus test-only integrations for SCD4x and Senseair I2C variants.
- VOC/NOx: Sensirion SGP41.
- Pressure: DPS368.

## Important Docs

- `BLE_STREAM.md` - BLE UUIDs, payload formats, and control commands.
- `AirGradient-Go-Complete-UI-Spec.md` - detailed screen and interaction spec.
- `STYLE_GUIDE.md` - project C/C++ formatting and coding conventions.

## Build/Developer Workflow

This is a standard ESP-IDF CMake project:

```bash
idf.py set-target esp32c5
idf.py menuconfig
idf.py build
idf.py flash monitor
```

Useful repo-level files:

- `CMakeLists.txt` - declares the `airgradient-go` project.
- `main/CMakeLists.txt` - lists the active application sources and required components.
- `sdkconfig` - current project configuration.
- `partitions.csv` - custom partition layout.

## Notes For Contributors

- `main/airgradient-go.cpp` appears to be an older standalone app variant and is not part of the current `main/CMakeLists.txt` source list.
- `build/` and `managed_components/` are generated or dependency-managed directories.
- The best entrypoint for understanding current behavior is `main/go.cpp`.
