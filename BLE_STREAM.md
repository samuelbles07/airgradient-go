# BLE Stream (AirGradient Go)

This document describes the Bluetooth Low Energy (BLE) GATT service implemented by the firmware.

Implementation reference:
- `main/ble_stream.h`
- `main/ble_stream.cpp`
- `main/go.cpp`

## Overview

- The device exposes a single custom GATT service with multiple characteristics.
- Notifications are used to stream live measurements, device status, and (optionally) export stored history.
- The firmware requests a larger MTU (best-effort): `NimBLEDevice::setMTU(256)`.

## Device Name / Advertising

- Advertised name: `AirGradientGo-<serial>` (serial is derived from the Wi-Fi MAC).
- Advertising includes the service UUID and scan response is enabled.

## Service

- Service UUID: `d1c0c0a0-6b48-4b2a-9b1d-59f9f2b0a1e1`

## Characteristics

| Name | UUID | Properties | Purpose |
| --- | --- | --- | --- |
| measures | `d1c0c0a1-6b48-4b2a-9b1d-59f9f2b0a1e1` | Notify | Live measurement stream (semicolon-separated payload). |
| status | `d1c0c0a2-6b48-4b2a-9b1d-59f9f2b0a1e1` | Notify | Device/status stream (JSON payload). |
| config | `d1c0c0a3-6b48-4b2a-9b1d-59f9f2b0a1e1` | Write, Write Without Response | JSON configuration/control commands (write-only). |
| history | `d1c0c0a4-6b48-4b2a-9b1d-59f9f2b0a1e1` | Notify, Write, Write Without Response | Stored-data export (write `1` to start; semicolon-separated payload). |

Notes:
- `config` is write-only (no read/echo).
- `history` is both writeable (to trigger export) and notifiable (to receive exported records).

## Payload Formats

### 1) `measures` (Notify)

Format: positional, semicolon-separated fields.

- Fields are always in the same order.
- Invalid/missing values are sent as an empty field (i.e. nothing between two `;`).
- Timestamp is sent as epoch milliseconds. If not available, the field is empty.
- Latitude/longitude are sent as decimal degrees strings (may include `-`), up to 7 decimals with trailing zeros trimmed.
- The payload always ends with a trailing `;`.

Field order (19 fields):

0. `ts_ms` (epoch ms, or empty)
1. `lat` (decimal degrees)
2. `lng` (decimal degrees)
3. `pm01_x10` (PM1.0 ug/m3 * 10)
4. `pm25_x10` (PM2.5 ug/m3 * 10)
5. `pm10_x10` (PM10 ug/m3 * 10)
6. `pc05_x10` (particle count 0.5 * 10)
7. `pc10_x10` (particle count 1.0 * 10)
8. `pc25_x10` (particle count 2.5 * 10)
9. `pc100_x10` (particle count 10 * 10)
10. `rco2_ppm` (STCC4 CO2 ppm)
11. `scd4x_ppm` (SCD4x CO2 ppm, test integration)
12. `atmp_c_x100` (temperature C * 100)
13. `rhum_x100` (relative humidity % * 100)
14. `pres_pa` (pressure in Pa)
15. `tvoc_raw` (SGP41 raw)
16. `nox_raw` (SGP41 raw)
17. `s12_ppm` (Senseair S12 CO2 ppm, test integration)
18. `sunlight_ppm` (Senseair Sunrise CO2 ppm, test integration; BLE field name is `sunlight`)

Example:

```text
1708837804000;43.237289;76.891928;190;202;204;1274;1507;1514;1515;2596;400;534;3929;92752;33897;19157;;;
```

Example (invalid values are empty fields; here `lng`, `pm10_x10`, `rco2_ppm`, `scd4x_ppm` are missing):

```text
1708837804000;43.237289;;190;202;;1274;1507;1514;1515;;;534;3929;92752;33897;19157;;;
```

Parsing tip: split on `;` and keep empty fields (do not drop empty tokens).

Pseudo code (illustrative, not language-specific):

```text
function splitPayload(s):
  assert s endsWith ";"
  parts = split(s, ";", keepEmpty=true)
  parts.removeLast()  // remove trailing empty token caused by final ';'
  return parts

function parseOptInt(str):
  if str == "" then return null
  return toIntBase10(str)

function parseOptFloat(str):
  if str == "" then return null
  return toFloat(str)

function parseMeasuresPayload(s):
  f = splitPayload(s)
  assert f.length == 19

  ts_ms     = parseOptInt(f[0])
  lat       = parseOptFloat(f[1])
  lng       = parseOptFloat(f[2])

  pm01_x10  = parseOptInt(f[3])
  pm25_x10  = parseOptInt(f[4])
  pm10_x10  = parseOptInt(f[5])

  pc05_x10  = parseOptInt(f[6])
  pc10_x10  = parseOptInt(f[7])
  pc25_x10  = parseOptInt(f[8])
  pc100_x10 = parseOptInt(f[9])

  rco2_ppm  = parseOptInt(f[10])
  scd4x_ppm = parseOptInt(f[11])

  atmp_x100 = parseOptInt(f[12])  // signed
  rhum_x100 = parseOptInt(f[13])
  pres_pa   = parseOptInt(f[14])
  tvoc_raw  = parseOptInt(f[15])
  nox_raw   = parseOptInt(f[16])
  s12_ppm   = parseOptInt(f[17])
  sunlight  = parseOptInt(f[18])
```

### 2) `history` (Notify)

Each notification corresponds to one stored record from NAND storage.

History payload prefixes the measures payload with `route_id` and `last`:

Format:

`route_id;last;<measures_payload>`

Field order:

0. `route_id` (tracking session id / route id)
1. `last` (`1` if this is the last record in storage, else `0`)
2..20. Same 19 fields as `measures` (see above)

Example:

```text
12345;0;1708837804000;43.237289;76.891928;190;202;204;1274;1507;1514;1515;2596;400;534;3929;92752;33897;19157;;;
```

Example (invalid values are empty fields; here `lng`, `pm10_x10`, `rco2_ppm`, `scd4x_ppm` are missing):

```text
12345;0;1708837804000;43.237289;;190;202;;1274;1507;1514;1515;;;534;3929;92752;33897;19157;;;
```

Pseudo code (illustrative, not language-specific):

```text
function parseHistoryPayload(s):
  f = splitPayload(s)
  assert f.length == 21

  route_id = parseOptInt(f[0])
  last     = (f[1] == "1")

  // Measures fields are f[2]..f[20] in the same order as the measures payload.
  // (You can parse them by position directly, or reconstruct a measures string and reuse a parser.)
```

### 3) `status` (Notify)

Format: JSON object.

Core fields:
- `state`: string enum: `IDLE`, `INACTIVE`, `TRACKING`, `SHUTDOWN`
- `trackingSleepS`: number (seconds)

Optional fields:
- `route`: number (only present when `state == "TRACKING"`)
- `gps_fix`: boolean (only present when GPS service is available)
- `gps_sats`: number (only present when GPS service is available)
- `battery_percent`: number (only present when battery percent is available)
- `flashAvail`: number (kilobytes; only present when NAND storage is mounted/ready)
- `charging`: boolean (only present when charging state is known)
- `co2Calibrating`: boolean (only present while a CO2 force-calibration request is pending/running)

### 4) `config` (Write / Write Without Response)

Format: JSON object.

Supported keys:

- `trackingSleepS`: integer seconds
  - Accepted range: 1..86400

- `co2ForceCalib`: integer target CO2 ppm
  - Accepted range: 1..32000
  - Executed only in `IDLE`.
  - Runs STCC4 calibration first (if supported), then SCD4x forced recalibration (if SCD4x is initialized).
  - While pending/running, `status` notifications include `co2Calibrating=true`.

- `tracking`: boolean
  - `true`: start tracking (only in `IDLE`, allocates a new route id)
  - `false`: stop tracking (only in `TRACKING`)

- `flashErase`: boolean
  - `true`: clears NAND tracking log (only in `IDLE`)

Example:

```json
{"trackingSleepS": 10}
```

```json
{"co2ForceCalib": 400}
```

```json
{"tracking": true}
```

```json
{"tracking": false}
```

```json
{"flashErase": true}
```

## History Export Procedure

Trigger:
- Subscribe to `history` notifications.
- Write a single byte `0x01` (or ASCII string `"1"` with length 1) to the `history` characteristic.

Behavior:
- Export is only started if the device is currently in `IDLE`. If not in `IDLE`, the trigger is ignored.
- Export is a blocking loop: while exporting, the firmware does not process buttons/UI/normal state stepping.
- Export stops when the app receives a packet with `last=1`.
