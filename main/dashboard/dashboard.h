#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "dashboard/display_driver.h"
#include "dashboard/u8g2_c_api.h"

namespace dashboard {

inline constexpr uint8_t STATUS_SYNC = 1U << 0;
inline constexpr uint8_t STATUS_GPS_FIX = 1U << 1;
inline constexpr uint8_t STATUS_TRACKING = 1U << 2;

struct Values {
  int co2_ppm;
  float pm25_ugm3;
  float temperature_c;
  int humidity_pct;

  uint8_t hour;
  uint8_t minute;

  uint8_t battery_pct;
  bool is_battery_charging;

  uint8_t status_mask;
};

struct Config {
  uint32_t max_partial_ops;
  display_driver::Config display;
};

class Dashboard {
public:
  static constexpr int REGION_MAX_H = 96;

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
