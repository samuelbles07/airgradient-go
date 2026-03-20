#pragma once

#include <stddef.h>
#include <stdint.h>

#include <esp_err.h>

#include "MeasuresTypes.h"
#include "dashboard/display_driver.h"
#include "dashboard/u8g2_c_api.h"

namespace dashboard {

enum class Screen : uint8_t {
  Home = 0,
  MainMenu,
  Settings,
  SettingsChoice,
  TagList,
  About,
  Confirm,
  Shutdown,
};

enum class Metric : uint8_t {
  None = 0,
  Pm25,
  Co2,
  Temp,
  Humidity,
  Tvoc,
  Nox,
};

static constexpr size_t MAX_LIST_ROWS = 9;

struct ListRow {
  const char *text = nullptr;
  bool disabled = false;
};

struct Values {
  int co2_ppm = MeasuresInvalid::CO2;
  float pm25_ugm3 = MeasuresInvalid::PM;
  float temperature_c = MeasuresInvalid::TEMPERATURE;
  float humidity_pct = MeasuresInvalid::HUMIDITY;
  float tvoc_raw = (float)MeasuresInvalid::TVOC;
  float nox_raw = (float)MeasuresInvalid::NOX;
  float pressure_hpa = -1.0f;
  float altitude_m = -1.0f;

  uint8_t hour = 0xFF;
  uint8_t minute = 0xFF;

  uint8_t battery_pct = 0xFF;
  bool is_battery_charging = false;

  bool locked = false;
  bool ble_enabled = true;
  bool ble_connected = false;
  bool wifi_enabled = true;
  bool gps_enabled = true;
  bool gps_fix = false;
  bool tracking_active = false;
  bool display_off = false;
  bool use_fahrenheit = false;
  bool pm_use_usaqi = false;

  Screen screen = Screen::Home;
  Metric active_metric = Metric::None;

  ListRow rows[MAX_LIST_ROWS] = {};
  uint8_t row_count = 0;
  uint8_t selected_row = 0;
  bool show_separator_after_back = false;

  const char *about_title = nullptr;
  const char *about_firmware = nullptr;
  const char *about_serial = nullptr;
  const char *about_hardware = nullptr;

  const float *chart_samples = nullptr;
  uint8_t chart_count = 0;
  float chart_min = 0.0f;
  float chart_max = 0.0f;

  const char *snackbar_text = nullptr;
};

struct Config {
  uint32_t max_partial_ops;
  display_driver::Config display;
};

class Dashboard {
public:
  static constexpr int REGION_MAX_H = 230;

  explicit Dashboard(Config cfg);

  esp_err_t init(const Values &initial);

  // Assumes values always change; updates via partial or full.
  // Blocking: yes (waits for EPD BUSY)
  void update(const Values &values);

  // Clears the screen (white) using a full update.
  // Blocking: yes (waits for EPD BUSY)
  void clear();

  void deep_sleep();

private:
  bool _is_header_changed(const Values &next) const;

  void _render_frame(const Values &values);
  void _update_full();
  void _update_partial();

  void _copy_full_width_region(int y_start, int h);

  Config _cfg;
  Values _values;
  bool _is_inited;

  uint32_t _partial_ops;

  u8g2_t _u8g2;

  static constexpr int BUF_ROW_BYTES = 16;
  static constexpr int BUF_TILE_HEIGHT = 32;
  static constexpr int BUF_HEIGHT_PX = BUF_TILE_HEIGHT * 8;
  static constexpr int BUF_SIZE_BYTES = BUF_ROW_BYTES * BUF_HEIGHT_PX;
  uint8_t _buf[BUF_SIZE_BYTES];

  static constexpr int REGION_W = 128;
  uint8_t _region_buf[BUF_ROW_BYTES * REGION_MAX_H];
};

} // namespace dashboard
