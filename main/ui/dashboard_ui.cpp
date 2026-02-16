// SPDX-License-Identifier: MIT

#include "ui/dashboard_ui.h"

#include <cstdint>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ui/u8g2_canvas.h"

extern "C" {
extern const uint8_t u8g2_font_6x10_tr[];
extern const uint8_t u8g2_font_10x20_tn[];
}

namespace ui {

static const uint8_t GPS_FIX_XBM[] = {
    0x80, 0x01, 0xE0, 0x07, 0xF0, 0x0F, 0x78, 0x1E, 0x38, 0x1C, 0x18, 0x18, 0x38, 0x1C, 0x38, 0x1C,
    0xF8, 0x1F, 0xF0, 0x0F, 0xF0, 0x0F, 0xE0, 0x07, 0xE0, 0x07, 0xC0, 0x03, 0x80, 0x01, 0x00, 0x00,
};

static const uint8_t SYNC_XBM[] = {
    0x00, 0x00, 0x80, 0x01, 0xF2, 0x0F, 0x1A, 0x18, 0x0E, 0x30, 0x06, 0x20, 0x3E, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x04, 0x7C, 0x04, 0x60, 0x0C, 0x70, 0x18, 0x58, 0xF0, 0x4F, 0x80, 0x01, 0x00, 0x00,
};

static const uint8_t TRACKING_XBM[] = {
    0x00, 0x00, 0x10, 0x00, 0x38, 0x00, 0xFC, 0x0F, 0x38, 0x10, 0x00, 0x20, 0x00, 0x00, 0x00, 0x18,
    0x10, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x1E, 0xF0, 0x1F, 0x00, 0x1E, 0x00, 0x0C, 0x00, 0x00,
};

DashboardUI::DashboardUI(ssd1680x::panels::GDEY0213B74 &epd) : epd_(epd) {}

esp_err_t DashboardUI::init() {
  esp_err_t err = epd_.ensure_init_full();
  if (err != ESP_OK) {
    return err;
  }
  err = epd_.clear_white();
  if (err != ESP_OK) {
    return err;
  }

  err = render_static_();
  if (err != ESP_OK) {
    return err;
  }
  memcpy(frame_, basemap_, sizeof(frame_));
  refresh_count_ = 0;
  return epd_.set_basemap_bw(frame_, ssd1680x::panels::GDEY0213B74::BUFFER_SIZE);
}

esp_err_t DashboardUI::render_static_() {
  U8g2Canvas c;
  c.attach(basemap_, sizeof(basemap_), W, H, MIRROR_X);
  c.clear_white();
  c.set_font(u8g2_font_6x10_tr);
  c.set_color_black();

  // Clock is dynamic; leave background empty.

  // Section labels.
  c.draw_str_centered(DashboardUI::TEXT_INSET + 1, 18, W - DashboardUI::TEXT_INSET, 12,
                      "PM2.5 (UG/M3)");
  c.draw_str_centered(DashboardUI::TEXT_INSET + 1, 66, W - DashboardUI::TEXT_INSET, 12,
                      "CO2 (PPM)");

  // Divider between hero section and grid.
  c.draw_hline(0, GRID_Y - 6, W);

  // Grid labels.
  c.draw_str(TEXT_INSET, GRID_Y + 0 * ROW_H + 2, "Temp");
  c.draw_str(64 + TEXT_INSET, GRID_Y + 0 * ROW_H + 2, "Humidity");
  c.draw_str(TEXT_INSET, GRID_Y + 1 * ROW_H + 2, "TVOC");
  c.draw_str(64 + TEXT_INSET, GRID_Y + 1 * ROW_H + 2, "NOx");
  c.draw_str(TEXT_INSET, GRID_Y + 2 * ROW_H + 2, "Pressure");
  c.draw_str(64 + TEXT_INSET, GRID_Y + 2 * ROW_H + 2, "Altitude");

  return ESP_OK;
}

void DashboardUI::render_text_(const Rect &r, uint8_t *buf, size_t len, const char *text,
                               const uint8_t *font, bool centered) {
  if (buf == nullptr || len == 0) {
    return;
  }
  U8g2Canvas w;
  w.attach(buf, len, r.w, r.h, MIRROR_X);
  w.clear_white();
  w.set_font(font);
  w.set_color_black();
  if (text == nullptr) {
    return;
  }
  if (centered) {
    w.draw_str_centered(TEXT_INSET, 0, r.w - TEXT_INSET, r.h, text);
  } else {
    w.draw_str(TEXT_INSET, 3, text);
  }
}

void DashboardUI::render_value_left_(const Rect &r, uint8_t *buf, size_t len, const char *text,
                                     const uint8_t *font) {
  if (buf == nullptr || len == 0) {
    return;
  }
  U8g2Canvas w;
  w.attach(buf, len, r.w, r.h, MIRROR_X);
  w.clear_white();
  w.set_font(font);
  w.set_color_black();
  if (text == nullptr) {
    return;
  }
  w.draw_str(TEXT_INSET, 2, text);
}

esp_err_t DashboardUI::batch_write_all_() {
  render_text_(CLOCK_R, buf_clock_, sizeof(buf_clock_), clock_, u8g2_font_6x10_tr, false);
  render_text_(PM_VALUE_R, buf_pm_, sizeof(buf_pm_), pm_, u8g2_font_10x20_tn, true);
  render_text_(BATTERY_R, buf_battery_, sizeof(buf_battery_), battery_, u8g2_font_6x10_tr, true);

  {
    U8g2Canvas w;
    w.attach(buf_tracking_, sizeof(buf_tracking_), TRACKING_R.w, TRACKING_R.h, MIRROR_X);
    w.clear_white();
    w.set_color_black();
    if (tracking_) {
      w.draw_xbmp(0, 0, TRACKING_R.w, TRACKING_R.h, TRACKING_XBM);
    }
  }
  {
    U8g2Canvas w;
    w.attach(buf_sync_, sizeof(buf_sync_), SYNC_R.w, SYNC_R.h, MIRROR_X);
    w.clear_white();
    w.set_color_black();
    if (syncing_) {
      w.draw_xbmp(0, 0, SYNC_R.w, SYNC_R.h, SYNC_XBM);
    }
  }
  {
    U8g2Canvas w;
    w.attach(buf_gps_fix_, sizeof(buf_gps_fix_), GPS_FIX_R.w, GPS_FIX_R.h, MIRROR_X);
    w.clear_white();
    w.set_color_black();
    if (gps_fixed_) {
      w.draw_xbmp(0, 0, GPS_FIX_R.w, GPS_FIX_R.h, GPS_FIX_XBM);
    }
  }
  // render_text_(CO2_VALUE_R, buf_co2_, sizeof(buf_co2_), co2_, u8g2_font_10x20_tn, true);
  // render_value_left_(TEMP_R, buf_temp_, sizeof(buf_temp_), temp_, u8g2_font_6x10_tr);
  // render_value_left_(HUM_R, buf_hum_, sizeof(buf_hum_), hum_, u8g2_font_6x10_tr);
  // render_value_left_(TVOC_R, buf_tvoc_, sizeof(buf_tvoc_), tvoc_, u8g2_font_6x10_tr);
  // render_value_left_(NOX_R, buf_nox_, sizeof(buf_nox_), nox_, u8g2_font_6x10_tr);
  // render_value_left_(PRES_R, buf_pres_, sizeof(buf_pres_), pres_, u8g2_font_6x10_tr);
  // render_value_left_(ALT_R, buf_alt_, sizeof(buf_alt_), alt_, u8g2_font_6x10_tr);

  esp_err_t err = epd_.partial_begin();
  if (err != ESP_OK)
    return err;

  err = epd_.partial_write_bw(CLOCK_R.x, CLOCK_R.y, CLOCK_R.w, CLOCK_R.h, buf_clock_,
                              RAW_LEN(CLOCK_R));
  if (err != ESP_OK)
    goto out_err;
  err = epd_.partial_write_bw(TRACKING_R.x, TRACKING_R.y, TRACKING_R.w, TRACKING_R.h, buf_tracking_,
                              RAW_LEN(TRACKING_R));
  if (err != ESP_OK)
    goto out_err;
  err = epd_.partial_write_bw(SYNC_R.x, SYNC_R.y, SYNC_R.w, SYNC_R.h, buf_sync_, RAW_LEN(SYNC_R));
  if (err != ESP_OK)
    goto out_err;
  err = epd_.partial_write_bw(GPS_FIX_R.x, GPS_FIX_R.y, GPS_FIX_R.w, GPS_FIX_R.h, buf_gps_fix_,
                              RAW_LEN(GPS_FIX_R));
  if (err != ESP_OK)
    goto out_err;
  err = epd_.partial_write_bw(PM_VALUE_R.x, PM_VALUE_R.y, PM_VALUE_R.w, PM_VALUE_R.h, buf_pm_,
                              RAW_LEN(PM_VALUE_R));
  if (err != ESP_OK)
    goto out_err;
  err = epd_.partial_write_bw(BATTERY_R.x, BATTERY_R.y, BATTERY_R.w, BATTERY_R.h, buf_battery_,
                              RAW_LEN(BATTERY_R));
  if (err != ESP_OK)
    goto out_err;
  // err = epd_.partial_write_bw(CO2_VALUE_R.x, CO2_VALUE_R.y, CO2_VALUE_R.w, CO2_VALUE_R.h, buf_co2_, RAW_LEN(CO2_VALUE_R));
  // if (err != ESP_OK) goto out_err;
  // err = epd_.partial_write_bw(TEMP_R.x, TEMP_R.y, TEMP_R.w, TEMP_R.h, buf_temp_, RAW_LEN(TEMP_R));
  // if (err != ESP_OK) goto out_err;
  // err = epd_.partial_write_bw(HUM_R.x, HUM_R.y, HUM_R.w, HUM_R.h, buf_hum_, RAW_LEN(HUM_R));
  // if (err != ESP_OK) goto out_err;
  // err = epd_.partial_write_bw(TVOC_R.x, TVOC_R.y, TVOC_R.w, TVOC_R.h, buf_tvoc_, RAW_LEN(TVOC_R));
  // if (err != ESP_OK) goto out_err;
  // err = epd_.partial_write_bw(NOX_R.x, NOX_R.y, NOX_R.w, NOX_R.h, buf_nox_, RAW_LEN(NOX_R));
  // if (err != ESP_OK) goto out_err;
  // err = epd_.partial_write_bw(PRES_R.x, PRES_R.y, PRES_R.w, PRES_R.h, buf_pres_, RAW_LEN(PRES_R));
  // if (err != ESP_OK) goto out_err;
  // err = epd_.partial_write_bw(ALT_R.x, ALT_R.y, ALT_R.w, ALT_R.h, buf_alt_, RAW_LEN(ALT_R));
  // if (err != ESP_OK) goto out_err;

  return epd_.partial_end_update();

out_err:
  // Reset partial-prepared state.
  (void)epd_.init_full();
  return err;
}

void DashboardUI::render_full_frame_() {
  memcpy(frame_, basemap_, sizeof(frame_));

  U8g2Canvas c;
  c.attach(frame_, sizeof(frame_), W, H, MIRROR_X);
  c.set_color_black();

  // Draw values at the same rectangles used for partial updates.
  c.set_font(u8g2_font_6x10_tr);
  c.draw_str(CLOCK_R.x + TEXT_INSET, CLOCK_R.y + 3, clock_);

  c.set_font(u8g2_font_10x20_tn);
  c.draw_str_centered(PM_VALUE_R.x, PM_VALUE_R.y, PM_VALUE_R.w, PM_VALUE_R.h, pm_);
  // c.draw_str_centered(CO2_VALUE_R.x, CO2_VALUE_R.y, CO2_VALUE_R.w, CO2_VALUE_R.h, co2_);

  c.set_font(u8g2_font_6x10_tr);
  c.draw_str_centered(BATTERY_R.x, BATTERY_R.y, BATTERY_R.w, BATTERY_R.h, battery_);

  if (tracking_) {
    c.draw_xbmp(TRACKING_R.x, TRACKING_R.y, TRACKING_R.w, TRACKING_R.h, TRACKING_XBM);
  }
  if (syncing_) {
    c.draw_xbmp(SYNC_R.x, SYNC_R.y, SYNC_R.w, SYNC_R.h, SYNC_XBM);
  }
  if (gps_fixed_) {
    c.draw_xbmp(GPS_FIX_R.x, GPS_FIX_R.y, GPS_FIX_R.w, GPS_FIX_R.h, GPS_FIX_XBM);
  }

  // c.set_font(u8g2_font_6x10_tr);
  // c.draw_str(TEMP_R.x + TEXT_INSET, TEMP_R.y + 2, temp_);
  // c.draw_str(HUM_R.x + TEXT_INSET, HUM_R.y + 2, hum_);
  // c.draw_str(TVOC_R.x + TEXT_INSET, TVOC_R.y + 2, tvoc_);
  // c.draw_str(NOX_R.x + TEXT_INSET, NOX_R.y + 2, nox_);
  // c.draw_str(PRES_R.x + TEXT_INSET, PRES_R.y + 2, pres_);
  // c.draw_str(ALT_R.x + TEXT_INSET, ALT_R.y + 2, alt_);
}

esp_err_t DashboardUI::refresh() {
  refresh_count_++;

  // Periodic maintenance refreshes to clear accumulated artifacts.
  // Full basemap refresh (slow) every ~2 minutes.
  if (FULL_BASEMAP_EVERY > 0 && (refresh_count_ % FULL_BASEMAP_EVERY) == 0) {
    render_full_frame_();
    return epd_.set_basemap_bw(frame_, ssd1680x::panels::GDEY0213B74::BUFFER_SIZE);
  }

  // Fast basemap refresh (still writes RAM1+RAM2) every ~20 seconds.
  if (FAST_BASEMAP_EVERY > 0 && (refresh_count_ % FAST_BASEMAP_EVERY) == 0) {
    render_full_frame_();
    return epd_.set_basemap_bw_fast(frame_, ssd1680x::panels::GDEY0213B74::BUFFER_SIZE);
  }

  // Single-pass partial refresh for all value windows.
  return batch_write_all_();
}

esp_err_t DashboardUI::full_refresh() {
  // Wake/re-init if needed.
  esp_err_t err = epd_.ensure_init_full();
  if (err != ESP_OK) {
    return err;
  }
  render_full_frame_();
  // set_basemap_bw() does a full update and restores partial prerequisites.
  err = epd_.set_basemap_bw(frame_, ssd1680x::panels::GDEY0213B74::BUFFER_SIZE);
  if (err == ESP_OK) {
    refresh_count_ = 0;
  }
  return err;
}

esp_err_t DashboardUI::set_time_hm(int hh, int mm) {
  snprintf(clock_, sizeof(clock_), "%02d:%02d", hh, mm);
  return ESP_OK;
}

esp_err_t DashboardUI::set_pm25_ugm3(float v) {
  // 1 decimal place, avoid -0.0
  if (fabsf(v) < 0.05f)
    v = 0.0f;
  snprintf(pm_, sizeof(pm_), "%.1f", (double)v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_co2_ppm(int v) {
  snprintf(co2_, sizeof(co2_), "%d", v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_temp_c(float v) {
  snprintf(temp_, sizeof(temp_), "%.1f C", (double)v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_humidity_pct(int v) {
  snprintf(hum_, sizeof(hum_), "%d %%", v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_tvoc(float v) {
  snprintf(tvoc_, sizeof(tvoc_), "%.1f", (double)v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_nox(float v) {
  snprintf(nox_, sizeof(nox_), "%.1f", (double)v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_pressure_hpa(int v) {
  snprintf(pres_, sizeof(pres_), "%d HPA", v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_altitude_m(int v) {
  snprintf(alt_, sizeof(alt_), "%d M", v);
  return ESP_OK;
}

esp_err_t DashboardUI::set_battery_percent(int percent) {
  if (percent < 0) {
    strncpy(battery_, "--%", sizeof(battery_));
    battery_[sizeof(battery_) - 1] = '\0';
    return ESP_OK;
  }
  if (percent > 100) {
    percent = 100;
  }
  snprintf(battery_, sizeof(battery_), "%d%%", percent);
  return ESP_OK;
}

esp_err_t DashboardUI::set_gps_fixed(bool fixed) {
  gps_fixed_ = fixed;
  return ESP_OK;
}

esp_err_t DashboardUI::set_tracking(bool tracking) {
  tracking_ = tracking;
  if (tracking_) {
    syncing_ = false;
  }
  return ESP_OK;
}

esp_err_t DashboardUI::set_syncing(bool syncing) {
  syncing_ = syncing;
  if (syncing_) {
    tracking_ = false;
  }
  return ESP_OK;
}

esp_err_t DashboardUI::clear_and_sleep() {
  // Use a full basemap update to fully whiten the panel (writes both RAM buffers)
  // and to restore deterministic panel state before sleeping.
  static uint8_t white[ssd1680x::panels::GDEY0213B74::BUFFER_SIZE];
  static bool inited = false;
  if (!inited) {
    memset(white, 0xFF, sizeof(white));
    inited = true;
  }

  esp_err_t err = epd_.ensure_init_full();
  if (err != ESP_OK) {
    return err;
  }

  err = epd_.set_basemap_bw(white, sizeof(white));
  if (err != ESP_OK) {
    // Fallback to simple clear if basemap update fails.
    err = epd_.clear_white();
    if (err != ESP_OK) {
      return err;
    }
  }

  // // Some panels deassert BUSY slightly early; give the physical update time to settle.
  // vTaskDelay(pdMS_TO_TICKS(4000));
  return epd_.deep_sleep();
}

} // namespace ui
