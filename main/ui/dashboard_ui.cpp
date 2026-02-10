// SPDX-License-Identifier: MIT

#include "ui/dashboard_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ui/u8g2_canvas.h"

extern "C" {
extern const uint8_t u8g2_font_6x10_tr[];
extern const uint8_t u8g2_font_10x20_tn[];
}

namespace ui {

DashboardUI::DashboardUI(ssd1680x::panels::GDEY0213B74& epd) : epd_(epd) {}

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
  c.draw_str_centered(DashboardUI::TEXT_INSET + 1, 18, W - DashboardUI::TEXT_INSET, 12, "PM2.5 (UG/M3)");
  c.draw_str_centered(DashboardUI::TEXT_INSET + 1, 66, W - DashboardUI::TEXT_INSET, 12, "CO2 (PPM)");

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

void DashboardUI::render_text_(const Rect& r, uint8_t* buf, size_t len, const char* text, const uint8_t* font,
                               bool centered) {
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
    w.draw_str(TEXT_INSET, 4, text);
  }
}

void DashboardUI::render_value_left_(const Rect& r, uint8_t* buf, size_t len, const char* text, const uint8_t* font) {
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
  render_text_(CO2_VALUE_R, buf_co2_, sizeof(buf_co2_), co2_, u8g2_font_10x20_tn, true);
  render_value_left_(TEMP_R, buf_temp_, sizeof(buf_temp_), temp_, u8g2_font_6x10_tr);
  render_value_left_(HUM_R, buf_hum_, sizeof(buf_hum_), hum_, u8g2_font_6x10_tr);
  render_value_left_(TVOC_R, buf_tvoc_, sizeof(buf_tvoc_), tvoc_, u8g2_font_6x10_tr);
  render_value_left_(NOX_R, buf_nox_, sizeof(buf_nox_), nox_, u8g2_font_6x10_tr);
  render_value_left_(PRES_R, buf_pres_, sizeof(buf_pres_), pres_, u8g2_font_6x10_tr);
  render_value_left_(ALT_R, buf_alt_, sizeof(buf_alt_), alt_, u8g2_font_6x10_tr);

  esp_err_t err = epd_.partial_begin();
  if (err != ESP_OK) return err;

  err = epd_.partial_write_bw(CLOCK_R.x, CLOCK_R.y, CLOCK_R.w, CLOCK_R.h, buf_clock_, RAW_LEN(CLOCK_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(PM_VALUE_R.x, PM_VALUE_R.y, PM_VALUE_R.w, PM_VALUE_R.h, buf_pm_, RAW_LEN(PM_VALUE_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(CO2_VALUE_R.x, CO2_VALUE_R.y, CO2_VALUE_R.w, CO2_VALUE_R.h, buf_co2_, RAW_LEN(CO2_VALUE_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(TEMP_R.x, TEMP_R.y, TEMP_R.w, TEMP_R.h, buf_temp_, RAW_LEN(TEMP_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(HUM_R.x, HUM_R.y, HUM_R.w, HUM_R.h, buf_hum_, RAW_LEN(HUM_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(TVOC_R.x, TVOC_R.y, TVOC_R.w, TVOC_R.h, buf_tvoc_, RAW_LEN(TVOC_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(NOX_R.x, NOX_R.y, NOX_R.w, NOX_R.h, buf_nox_, RAW_LEN(NOX_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(PRES_R.x, PRES_R.y, PRES_R.w, PRES_R.h, buf_pres_, RAW_LEN(PRES_R));
  if (err != ESP_OK) goto out_err;
  err = epd_.partial_write_bw(ALT_R.x, ALT_R.y, ALT_R.w, ALT_R.h, buf_alt_, RAW_LEN(ALT_R));
  if (err != ESP_OK) goto out_err;

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
  c.draw_str_centered(CO2_VALUE_R.x, CO2_VALUE_R.y, CO2_VALUE_R.w, CO2_VALUE_R.h, co2_);

  c.set_font(u8g2_font_6x10_tr);
  c.draw_str(TEMP_R.x + TEXT_INSET, TEMP_R.y + 2, temp_);
  c.draw_str(HUM_R.x + TEXT_INSET, HUM_R.y + 2, hum_);
  c.draw_str(TVOC_R.x + TEXT_INSET, TVOC_R.y + 2, tvoc_);
  c.draw_str(NOX_R.x + TEXT_INSET, NOX_R.y + 2, nox_);
  c.draw_str(PRES_R.x + TEXT_INSET, PRES_R.y + 2, pres_);
  c.draw_str(ALT_R.x + TEXT_INSET, ALT_R.y + 2, alt_);
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
  if (fabsf(v) < 0.05f) v = 0.0f;
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

}  // namespace ui
