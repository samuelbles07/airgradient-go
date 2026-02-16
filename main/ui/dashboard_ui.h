// SPDX-License-Identifier: MIT

#ifndef GO_DISPLAY_UI_DASHBOARD_UI_H
#define GO_DISPLAY_UI_DASHBOARD_UI_H

#include <stdint.h>

#include "esp_err.h"

#include "gdey0213b74.h"

namespace ui {

class DashboardUI {
 public:
  explicit DashboardUI(ssd1680x::panels::GDEY0213B74& epd);

  esp_err_t init();

  // Apply the current values to the display using partial refresh.
  // Uses a batched two-phase clear+draw update to reduce ghosting.
  esp_err_t refresh();

  // Force a full refresh (basemap + values). Useful after deep sleep.
  esp_err_t full_refresh();

  // Clear the full display to white and put the panel into deep sleep.
  // Intended for system shutdown.
  esp_err_t clear_and_sleep();

  esp_err_t set_time_hm(int hh, int mm);
  esp_err_t set_pm25_ugm3(float v);
  esp_err_t set_co2_ppm(int v);
  esp_err_t set_temp_c(float v);
  esp_err_t set_humidity_pct(int v);
  esp_err_t set_tvoc(float v);
  esp_err_t set_nox(float v);
  esp_err_t set_pressure_hpa(int v);
  esp_err_t set_altitude_m(int v);

  esp_err_t set_battery_percent(int percent);

  esp_err_t set_gps_fixed(bool fixed);
  esp_err_t set_tracking(bool tracking);
  esp_err_t set_syncing(bool syncing);

 private:
  struct Rect {
    int x;
    int y;
    int w;
    int h;
  };

  static constexpr bool MIRROR_X = false;

  // Extra inset for text drawing to avoid left edge clipping.
  static constexpr int TEXT_INSET = 8;

  static constexpr size_t RAW_LEN(const Rect& r) { return (size_t)(r.w * r.h) / 8; }

  static constexpr int W = ssd1680x::panels::GDEY0213B74::WIDTH;
  static constexpr int H = ssd1680x::panels::GDEY0213B74::HEIGHT;

  static constexpr Rect CLOCK_R = {0, 0, 48, 16};
  static constexpr Rect PM_VALUE_R = {0, 32, 128, 32};
  static constexpr Rect CO2_VALUE_R = {0, 80, 128, 32};

  static constexpr Rect BATTERY_R = {0, H - 35, W, 16};

  // Top-right GPS-fix indicator icon.
  static constexpr Rect GPS_FIX_R = {W - 16, 0, 16, 16};

  // Top-right status icons.
  static constexpr Rect SYNC_R = {W - 32, 0, 16, 16};
  static constexpr Rect TRACKING_R = {W - 48, 0, 16, 16};

  static constexpr int GRID_Y = 124;
  static constexpr int ROW_H = 32;
  static constexpr int COL_W = 64;
  static constexpr int VALUE_Y_OFF = 14;
  static constexpr int VALUE_H = 18;

  static constexpr Rect TEMP_R = {0, GRID_Y + 0 * ROW_H + VALUE_Y_OFF, COL_W, VALUE_H};
  static constexpr Rect HUM_R = {64, GRID_Y + 0 * ROW_H + VALUE_Y_OFF, COL_W, VALUE_H};
  static constexpr Rect TVOC_R = {0, GRID_Y + 1 * ROW_H + VALUE_Y_OFF, COL_W, VALUE_H};
  static constexpr Rect NOX_R = {64, GRID_Y + 1 * ROW_H + VALUE_Y_OFF, COL_W, VALUE_H};
  static constexpr Rect PRES_R = {0, GRID_Y + 2 * ROW_H + VALUE_Y_OFF, COL_W, VALUE_H};
  static constexpr Rect ALT_R = {64, GRID_Y + 2 * ROW_H + VALUE_Y_OFF, COL_W, VALUE_H};

  // u8g2 buffer height must be a multiple of 8.
  static constexpr int BASEMAP_PAD_H = 256;
  static constexpr size_t BASEMAP_PAD_LEN = (size_t)(W / 8) * (size_t)BASEMAP_PAD_H;

  static constexpr int TILE_PAD_H = 24;  // for VALUE_H=18

  static constexpr size_t CLOCK_BUFSZ = (size_t)(CLOCK_R.w / 8) * (size_t)CLOCK_R.h;
  static constexpr size_t PM_BUFSZ = (size_t)(PM_VALUE_R.w / 8) * (size_t)PM_VALUE_R.h;
  static constexpr size_t CO2_BUFSZ = (size_t)(CO2_VALUE_R.w / 8) * (size_t)CO2_VALUE_R.h;
  static constexpr size_t BATTERY_BUFSZ = (size_t)(BATTERY_R.w / 8) * (size_t)BATTERY_R.h;
  static constexpr size_t GPS_FIX_BUFSZ = (size_t)(GPS_FIX_R.w / 8) * (size_t)GPS_FIX_R.h;
  static constexpr size_t SYNC_BUFSZ = (size_t)(SYNC_R.w / 8) * (size_t)SYNC_R.h;
  static constexpr size_t TRACKING_BUFSZ = (size_t)(TRACKING_R.w / 8) * (size_t)TRACKING_R.h;
  static constexpr size_t TEMP_BUFSZ = (size_t)(TEMP_R.w / 8) * (size_t)TILE_PAD_H;
  static constexpr size_t HUM_BUFSZ = (size_t)(HUM_R.w / 8) * (size_t)TILE_PAD_H;
  static constexpr size_t TVOC_BUFSZ = (size_t)(TVOC_R.w / 8) * (size_t)TILE_PAD_H;
  static constexpr size_t NOX_BUFSZ = (size_t)(NOX_R.w / 8) * (size_t)TILE_PAD_H;
  static constexpr size_t PRES_BUFSZ = (size_t)(PRES_R.w / 8) * (size_t)TILE_PAD_H;
  static constexpr size_t ALT_BUFSZ = (size_t)(ALT_R.w / 8) * (size_t)TILE_PAD_H;

  ssd1680x::panels::GDEY0213B74& epd_;

  uint8_t basemap_[BASEMAP_PAD_LEN];
  uint8_t frame_[BASEMAP_PAD_LEN];

  uint8_t buf_clock_[CLOCK_BUFSZ];
  uint8_t buf_pm_[PM_BUFSZ];
  uint8_t buf_co2_[CO2_BUFSZ];
  uint8_t buf_battery_[BATTERY_BUFSZ];
  uint8_t buf_tracking_[TRACKING_BUFSZ];
  uint8_t buf_sync_[SYNC_BUFSZ];
  uint8_t buf_gps_fix_[GPS_FIX_BUFSZ];
  uint8_t buf_temp_[TEMP_BUFSZ];
  uint8_t buf_hum_[HUM_BUFSZ];
  uint8_t buf_tvoc_[TVOC_BUFSZ];
  uint8_t buf_nox_[NOX_BUFSZ];
  uint8_t buf_pres_[PRES_BUFSZ];
  uint8_t buf_alt_[ALT_BUFSZ];

  char clock_[8] = {0};
  char pm_[16] = {0};
  char co2_[16] = {0};
  char temp_[16] = {0};
  char hum_[16] = {0};
  char tvoc_[16] = {0};
  char nox_[16] = {0};
  char pres_[16] = {0};
  char alt_[16] = {0};
  char battery_[8] = {'-', '-', '%', '\0'};

  bool gps_fixed_ = false;
  bool tracking_ = false;
  bool syncing_ = false;

  uint32_t refresh_count_ = 0;
  static constexpr uint32_t FAST_BASEMAP_EVERY = 4;    // every 20s at 5s cadence
  static constexpr uint32_t FULL_BASEMAP_EVERY = 24;   // every 2 min at 5s cadence

  esp_err_t render_static_();
  void render_text_(const Rect& r, uint8_t* buf, size_t len, const char* text, const uint8_t* font, bool centered);
  void render_value_left_(const Rect& r, uint8_t* buf, size_t len, const char* text, const uint8_t* font);
  esp_err_t batch_write_all_();
  void render_full_frame_();
};

}  // namespace ui

#endif
