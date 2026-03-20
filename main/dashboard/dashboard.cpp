#include "dashboard/dashboard.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <esp_log.h>

#include "dashboard/u8x8_c_api.h"

namespace dashboard {

static constexpr char LOG_TAG[] = "dashboard";

namespace {

struct ScopedBusAcquire {
  ScopedBusAcquire() : err(display_driver::bus_acquire()) {}
  ~ScopedBusAcquire() { display_driver::bus_release(); }

  esp_err_t err;
};

static constexpr int SCREEN_W = 128;
static constexpr int CONTENT_W = 122;
static constexpr int BODY_Y = 20;
static constexpr int BODY_H = 230;

static constexpr int STATUS_DIVIDER_Y = 19;
static constexpr int PM_BLOCK_Y = 27;
static constexpr int PM_BLOCK_H = 44;
static constexpr int PM_LABEL_BASELINE_Y = 41;
static constexpr int PM_VALUE_BASELINE_Y = 68;
static constexpr int CO2_BLOCK_Y = 74;
static constexpr int CO2_BLOCK_H = 47;
static constexpr int CO2_LABEL_BASELINE_Y = 90;
static constexpr int CO2_VALUE_BASELINE_Y = 117;
static constexpr int MAIN_DIVIDER_Y = 127;
static constexpr int GRID_DIVIDER_X = 61;
static constexpr int GRID_TOP_Y = 133;
static constexpr int GRID_LINE_1_Y = 162;
static constexpr int GRID_LINE_2_Y = 191;
static constexpr int GRID_STRONG_LINE_Y = 190;
static constexpr int GRID_BOTTOM_Y = 220;
static constexpr int LOGO_Y = 221;
static constexpr int LOGO_H = 24;
static constexpr int DISPLAY_OFF_LOGO_Y = 113;
static constexpr int DISPLAY_OFF_LOGO_H = 24;
static constexpr int SNACKBAR_Y = 232;
static constexpr int SNACKBAR_H = 18;
static constexpr int CHART_BOX_Y = 221;
static constexpr int CHART_BOX_H = 29;
static constexpr int PLOT_X = 3;
static constexpr int PLOT_Y = 225;
static constexpr int PLOT_W = 115;
static constexpr int PLOT_H = 22;
static constexpr int MAIN_MENU_BG_Y = 128;
static constexpr int MAIN_MENU_BG_H = 122;
static constexpr int FULL_SCREEN_BG_Y = 24;
static constexpr int FULL_SCREEN_BG_H = 226;

static constexpr int CELL_X[6] = {1, 62, 1, 62, 1, 62};
static constexpr int CELL_Y[6] = {134, 134, 163, 163, 192, 192};
static constexpr int CELL_W = 59;
static constexpr int CELL_H = 27;
static constexpr int LABEL_X[6] = {10, 68, 10, 68, 10, 68};
static constexpr int LABEL_Y[6] = {142, 142, 171, 171, 199, 199};
static constexpr int VALUE_X[6] = {10, 68, 10, 68, 10, 68};
static constexpr int VALUE_Y[6] = {155, 155, 184, 184, 213, 213};

static const u8x8_display_info_t U8X8_DISPLAY_INFO_EPD_128X250 = {
    /* chip_enable_level = */ 0,
    /* chip_disable_level = */ 1,
    /* post_chip_enable_wait_ns = */ 0,
    /* pre_chip_disable_wait_ns = */ 0,
    /* reset_pulse_width_ms = */ 0,
    /* post_reset_wait_ms = */ 0,
    /* sda_setup_time_ns = */ 0,
    /* sck_pulse_width_ns = */ 0,
    /* sck_clock_hz = */ 10000000UL,
    /* spi_mode = */ 0,
    /* i2c_bus_clock_100kHz = */ 0,
    /* data_setup_time_ns = */ 0,
    /* write_pulse_width_ns = */ 0,
    /* tile_width = */ 16,
    /* tile_hight = */ 32,
    /* default_x_offset = */ 0,
    /* flipmode_x_offset = */ 0,
    /* pixel_width = */ 128,
    /* pixel_height = */ 250,
};

static uint8_t u8x8_d_epd_128x250_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
  (void)arg_int;
  (void)arg_ptr;

  switch (msg) {
  case U8X8_MSG_DISPLAY_SETUP_MEMORY:
    u8x8_d_helper_display_setup_memory(u8x8, &U8X8_DISPLAY_INFO_EPD_128X250);
    return 1;
  default:
    return 1;
  }
}

static bool is_home_like(dashboard::Screen screen) {
  return screen == dashboard::Screen::Home || screen == dashboard::Screen::MainMenu;
}

static bool metric_has_chart(dashboard::Metric metric) {
  switch (metric) {
  case dashboard::Metric::Pm25:
  case dashboard::Metric::Co2:
  case dashboard::Metric::Temp:
  case dashboard::Metric::Humidity:
  case dashboard::Metric::Tvoc:
  case dashboard::Metric::Nox:
    return true;
  case dashboard::Metric::None:
  default:
    return false;
  }
}

static bool is_metric_valid(float value) { return value >= 0.0f; }

static float temp_for_display(float celsius, bool use_fahrenheit) {
  if (!is_metric_valid(celsius)) {
    return celsius;
  }
  if (!use_fahrenheit) {
    return celsius;
  }
  return celsius * 9.0f / 5.0f + 32.0f;
}

static int pm25_to_usaqi(float pm25) {
  struct Breakpoint {
    float c_low;
    float c_high;
    int i_low;
    int i_high;
  };

  static const Breakpoint kBreakpoints[] = {
      {0.0f, 12.0f, 0, 50},       {12.1f, 35.4f, 51, 100},    {35.5f, 55.4f, 101, 150},
      {55.5f, 150.4f, 151, 200},  {150.5f, 250.4f, 201, 300}, {250.5f, 350.4f, 301, 400},
      {350.5f, 500.4f, 401, 500},
  };

  if (!(pm25 >= 0.0f)) {
    return -1;
  }

  for (size_t i = 0; i < sizeof(kBreakpoints) / sizeof(kBreakpoints[0]); ++i) {
    const Breakpoint &bp = kBreakpoints[i];
    if (pm25 <= bp.c_high) {
      const float ratio = (pm25 - bp.c_low) / (bp.c_high - bp.c_low);
      return bp.i_low + (int)lroundf(ratio * (float)(bp.i_high - bp.i_low));
    }
  }

  return 500;
}

static void format_one_decimal(char *out, size_t out_size, float value) {
  const int scaled = (int)lroundf(value * 10.0f);
  const bool neg = scaled < 0;
  const unsigned int abs_scaled = (unsigned int)(neg ? -scaled : scaled);
  (void)snprintf(out, out_size, "%s%u.%01u", neg ? "-" : "", abs_scaled / 10U, abs_scaled % 10U);
}

static void format_pm_value(char *out, size_t out_size, float value, bool use_usaqi) {
  if (!is_metric_valid(value)) {
    (void)snprintf(out, out_size, "-");
    return;
  }

  if (use_usaqi) {
    const int aqi = pm25_to_usaqi(value);
    if (aqi > 500) {
      (void)snprintf(out, out_size, "500+");
      return;
    }
    (void)snprintf(out, out_size, "%d", aqi);
    return;
  }

  if (value > 999.0f) {
    (void)snprintf(out, out_size, "999+");
    return;
  }
  if (value >= 100.0f) {
    (void)snprintf(out, out_size, "%.0f", value);
    return;
  }
  format_one_decimal(out, out_size, value);
}

static void format_co2_value(char *out, size_t out_size, int value) {
  if (value == MeasuresInvalid::CO2) {
    (void)snprintf(out, out_size, "-");
    return;
  }
  if (value > 9999) {
    (void)snprintf(out, out_size, "9999+");
    return;
  }
  (void)snprintf(out, out_size, "%d", value);
}

static void format_temperature_value(char *out, size_t out_size, float celsius,
                                     bool use_fahrenheit) {
  if (!is_metric_valid(celsius)) {
    (void)snprintf(out, out_size, "-");
    return;
  }
  const float shown = temp_for_display(celsius, use_fahrenheit);
  (void)snprintf(out, out_size, "%.1f %c", shown, use_fahrenheit ? 'F' : 'C');
}

static void format_humidity_value(char *out, size_t out_size, float humidity) {
  if (!is_metric_valid(humidity)) {
    (void)snprintf(out, out_size, "-");
    return;
  }
  (void)snprintf(out, out_size, "%.0f %%", humidity);
}

static void format_index_value(char *out, size_t out_size, float value) {
  if (!is_metric_valid(value)) {
    (void)snprintf(out, out_size, "-");
    return;
  }
  if (fabsf(value - roundf(value)) < 0.05f) {
    (void)snprintf(out, out_size, "%.0f", value);
    return;
  }
  format_one_decimal(out, out_size, value);
}

static void format_pressure_value(char *out, size_t out_size, float pressure_hpa) {
  if (!is_metric_valid(pressure_hpa)) {
    (void)snprintf(out, out_size, "-");
    return;
  }
  if (pressure_hpa > 9999.0f) {
    (void)snprintf(out, out_size, "9999+ hPa");
    return;
  }
  (void)snprintf(out, out_size, "%.0f hPa", pressure_hpa);
}

static void format_altitude_value(char *out, size_t out_size, float altitude_m) {
  if (!is_metric_valid(altitude_m)) {
    (void)snprintf(out, out_size, "-");
    return;
  }
  if (altitude_m > 9999.0f) {
    (void)snprintf(out, out_size, "9999+ m");
    return;
  }
  (void)snprintf(out, out_size, "%.0f m", altitude_m);
}

static void format_chart_stat(char *out, size_t out_size, dashboard::Metric metric, float value,
                              bool use_fahrenheit, bool pm_use_usaqi) {
  switch (metric) {
  case dashboard::Metric::Pm25:
    format_pm_value(out, out_size, value, pm_use_usaqi);
    break;
  case dashboard::Metric::Co2:
    format_co2_value(out, out_size, (int)lroundf(value));
    break;
  case dashboard::Metric::Temp:
    format_temperature_value(out, out_size, value, use_fahrenheit);
    break;
  case dashboard::Metric::Humidity:
    format_humidity_value(out, out_size, value);
    break;
  case dashboard::Metric::Tvoc:
  case dashboard::Metric::Nox:
    format_index_value(out, out_size, value);
    break;
  case dashboard::Metric::None:
  default:
    (void)snprintf(out, out_size, "-");
    break;
  }
}

static void draw_text(u8g2_t *u8g2, int x, int baseline_y, const char *text) {
  if (text == nullptr) {
    return;
  }
  u8g2_DrawStr(u8g2, (u8g2_uint_t)x, (u8g2_uint_t)baseline_y, text);
}

static void draw_centered_text(u8g2_t *u8g2, int center_x, int baseline_y, const char *text) {
  if (text == nullptr) {
    return;
  }
  const int width = (int)u8g2_GetStrWidth(u8g2, text);
  draw_text(u8g2, center_x - width / 2, baseline_y, text);
}

static void draw_logo(u8g2_t *u8g2, int y, int h) {
  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  draw_centered_text(u8g2, CONTENT_W / 2, y + h / 2 + 4, "AirGradient");
}

static void draw_lock_icon(u8g2_t *u8g2, int x, int y, bool locked) {
  u8g2_DrawFrame(u8g2, x + 1, y + 5, 7, 6);
  if (locked) {
    u8g2_DrawBox(u8g2, x + 2, y + 6, 5, 4);
  }
  u8g2_DrawCircle(u8g2, x + 4, y + 5, 3, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
}

static void draw_tracking_dot(u8g2_t *u8g2) { u8g2_DrawDisc(u8g2, 98, 10, 2, U8G2_DRAW_ALL); }

static constexpr uint16_t BATTERY_GLYPH_CHARGING = 57914;
static constexpr uint16_t BATTERY_GLYPH_LVL_0_10 = 57932;
static constexpr uint16_t BATTERY_GLYPH_LVL_10_30 = 57933;
static constexpr uint16_t BATTERY_GLYPH_LVL_30_50 = 57935;
static constexpr uint16_t BATTERY_GLYPH_LVL_50_70 = 57937;
static constexpr uint16_t BATTERY_GLYPH_LVL_70_95 = 57939;
static constexpr uint16_t BATTERY_GLYPH_LVL_95_100 = 57940;

static uint16_t battery_glyph(bool is_charging, uint8_t pct) {
  if (is_charging) {
    return BATTERY_GLYPH_CHARGING;
  }
  const uint8_t v = (pct > 100U) ? 100U : pct;
  if (v <= 10U) {
    return BATTERY_GLYPH_LVL_0_10;
  }
  if (v <= 30U) {
    return BATTERY_GLYPH_LVL_10_30;
  }
  if (v <= 50U) {
    return BATTERY_GLYPH_LVL_30_50;
  }
  if (v <= 70U) {
    return BATTERY_GLYPH_LVL_50_70;
  }
  if (v <= 95U) {
    return BATTERY_GLYPH_LVL_70_95;
  }
  return BATTERY_GLYPH_LVL_95_100;
}

static void draw_glyph(u8g2_t *u8g2, int x, int baseline_y, uint16_t encoding) {
  u8g2_DrawGlyph(u8g2, (u8g2_uint_t)x, (u8g2_uint_t)baseline_y, encoding);
}

static void draw_status_bar(u8g2_t *u8g2, const Values &values) {
  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  draw_lock_icon(u8g2, 3, 2, values.locked);

  if (values.ble_enabled) {
    draw_text(u8g2, 10, 10, "BLE");
    if (values.ble_connected) {
      u8g2_DrawDisc(u8g2, 26, 6, 1, U8G2_DRAW_ALL);
    }
  }
  if (values.wifi_enabled) {
    draw_text(u8g2, 30, 10, "WiFi");
  }
  if (values.gps_enabled) {
    draw_text(u8g2, 56, 10, "GPS");
    if (values.gps_fix) {
      u8g2_DrawDisc(u8g2, 75, 6, 1, U8G2_DRAW_ALL);
    }
  }
  if (values.tracking_active) {
    draw_tracking_dot(u8g2);
  }

  if (values.battery_pct != 0xFFu || values.is_battery_charging) {
    u8g2_SetFont(u8g2, u8g2_font_siji_t_6x10);
    draw_glyph(u8g2, 100, 11, battery_glyph(values.is_battery_charging, values.battery_pct));
  }

  u8g2_DrawHLine(u8g2, 0, STATUS_DIVIDER_Y, CONTENT_W);
}

static void draw_chart(u8g2_t *u8g2, const Values &values) {
  u8g2_DrawHLine(u8g2, 0, GRID_STRONG_LINE_Y, CONTENT_W);
  u8g2_DrawHLine(u8g2, 0, GRID_STRONG_LINE_Y + 1, CONTENT_W);
  u8g2_DrawHLine(u8g2, 0, GRID_BOTTOM_Y, CONTENT_W);
  u8g2_DrawVLine(u8g2, PLOT_X, PLOT_Y, PLOT_H);
  u8g2_DrawHLine(u8g2, PLOT_X, PLOT_Y + PLOT_H, PLOT_W);

  if (values.chart_samples == nullptr || values.chart_count == 0) {
    return;
  }

  const float min_value = values.chart_min;
  const float max_value = values.chart_max;
  const float range = max_value - min_value;
  int prev_x = PLOT_X;
  int prev_y = PLOT_Y + PLOT_H / 2;

  for (int x = 0; x < PLOT_W; ++x) {
    const int sample_index =
        (x * (int)(values.chart_count - 1)) / ((PLOT_W > 1) ? (PLOT_W - 1) : 1);
    float sample = values.chart_samples[sample_index];
    int y = PLOT_Y + PLOT_H / 2;
    if (range > 0.001f) {
      const float norm = (sample - min_value) / range;
      y = PLOT_Y + (int)lroundf((1.0f - norm) * (float)PLOT_H);
    }
    if (y < PLOT_Y) {
      y = PLOT_Y;
    }
    if (y > (PLOT_Y + PLOT_H)) {
      y = PLOT_Y + PLOT_H;
    }
    const int draw_x = PLOT_X + x;
    if (x > 0) {
      u8g2_DrawLine(u8g2, prev_x, prev_y, draw_x, y);
    }
    prev_x = draw_x;
    prev_y = y;
  }
}

static void draw_cell(u8g2_t *u8g2, int index, const char *label, const char *value,
                      bool selected) {
  if (selected) {
    u8g2_DrawBox(u8g2, CELL_X[index], CELL_Y[index], CELL_W, CELL_H);
    u8g2_SetDrawColor(u8g2, 1);
  }

  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  draw_text(u8g2, LABEL_X[index], LABEL_Y[index], label);
  draw_text(u8g2, VALUE_X[index], VALUE_Y[index], value);

  if (selected) {
    u8g2_SetDrawColor(u8g2, 0);
  }
}

static void draw_home(u8g2_t *u8g2, const Values &values) {
  if (values.display_off) {
    draw_logo(u8g2, DISPLAY_OFF_LOGO_Y, DISPLAY_OFF_LOGO_H);
    return;
  }

  const bool chart_visible = metric_has_chart(values.active_metric);
  const bool pm_selected = values.active_metric == Metric::Pm25;
  const bool co2_selected = values.active_metric == Metric::Co2;

  if (pm_selected) {
    u8g2_DrawBox(u8g2, 0, PM_BLOCK_Y, CONTENT_W, PM_BLOCK_H);
  }
  if (co2_selected) {
    u8g2_DrawBox(u8g2, 0, CO2_BLOCK_Y, CONTENT_W, CO2_BLOCK_H);
  }

  u8g2_DrawHLine(u8g2, 0, MAIN_DIVIDER_Y, CONTENT_W);
  u8g2_DrawHLine(u8g2, 0, MAIN_DIVIDER_Y + 1, CONTENT_W);
  u8g2_DrawVLine(u8g2, GRID_DIVIDER_X, GRID_TOP_Y, 86);
  u8g2_DrawHLine(u8g2, 0, GRID_LINE_1_Y, CONTENT_W);
  if (!chart_visible) {
    u8g2_DrawHLine(u8g2, 0, GRID_LINE_2_Y, CONTENT_W);
    u8g2_DrawHLine(u8g2, 0, GRID_BOTTOM_Y, CONTENT_W);
  }

  char buffer[24];
  const char *pm_label = values.pm_use_usaqi ? "PM2.5 (USAQI)" : "PM2.5 (ug/m3)";
  const char *co2_label = "CO2 (ppm)";

  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  if (pm_selected) {
    u8g2_SetDrawColor(u8g2, 1);
  }
  draw_centered_text(u8g2, CONTENT_W / 2, PM_LABEL_BASELINE_Y, pm_label);
  u8g2_SetFont(u8g2, u8g2_font_10x20_tn);
  format_pm_value(buffer, sizeof(buffer), values.pm25_ugm3, values.pm_use_usaqi);
  draw_centered_text(u8g2, CONTENT_W / 2, PM_VALUE_BASELINE_Y, buffer);
  if (pm_selected) {
    u8g2_SetDrawColor(u8g2, 0);
  }

  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  if (co2_selected) {
    u8g2_SetDrawColor(u8g2, 1);
  }
  draw_centered_text(u8g2, CONTENT_W / 2, CO2_LABEL_BASELINE_Y, co2_label);
  u8g2_SetFont(u8g2, u8g2_font_10x20_tn);
  format_co2_value(buffer, sizeof(buffer), values.co2_ppm);
  draw_centered_text(u8g2, CONTENT_W / 2, CO2_VALUE_BASELINE_Y, buffer);
  if (co2_selected) {
    u8g2_SetDrawColor(u8g2, 0);
  }

  char temp_buf[24];
  char hum_buf[24];
  char tvoc_buf[24];
  char nox_buf[24];
  char left_bottom_buf[24];
  char right_bottom_buf[24];
  format_temperature_value(temp_buf, sizeof(temp_buf), values.temperature_c, values.use_fahrenheit);
  format_humidity_value(hum_buf, sizeof(hum_buf), values.humidity_pct);
  format_index_value(tvoc_buf, sizeof(tvoc_buf), values.tvoc_raw);
  format_index_value(nox_buf, sizeof(nox_buf), values.nox_raw);

  const char *left_bottom_label = "Pressure";
  const char *right_bottom_label = "Altitude";
  if (chart_visible) {
    left_bottom_label = "Min";
    right_bottom_label = "Max";
    format_chart_stat(left_bottom_buf, sizeof(left_bottom_buf), values.active_metric,
                      values.chart_min, values.use_fahrenheit, values.pm_use_usaqi);
    format_chart_stat(right_bottom_buf, sizeof(right_bottom_buf), values.active_metric,
                      values.chart_max, values.use_fahrenheit, values.pm_use_usaqi);
  } else {
    format_pressure_value(left_bottom_buf, sizeof(left_bottom_buf), values.pressure_hpa);
    format_altitude_value(right_bottom_buf, sizeof(right_bottom_buf), values.altitude_m);
  }

  draw_cell(u8g2, 0, "Temp", temp_buf, values.active_metric == Metric::Temp);
  draw_cell(u8g2, 1, "Humidity", hum_buf, values.active_metric == Metric::Humidity);
  draw_cell(u8g2, 2, "TVOC", tvoc_buf, values.active_metric == Metric::Tvoc);
  draw_cell(u8g2, 3, "NOx", nox_buf, values.active_metric == Metric::Nox);
  draw_cell(u8g2, 4, left_bottom_label, left_bottom_buf, false);
  draw_cell(u8g2, 5, right_bottom_label, right_bottom_buf, false);

  if (chart_visible) {
    draw_chart(u8g2, values);
  } else {
    draw_logo(u8g2, LOGO_Y, LOGO_H);
  }
}

static void draw_list_rows(u8g2_t *u8g2, const Values &values, bool full_screen) {
  const int row_base_y = full_screen ? 25 : 132;
  const int row_rect_x = 5;
  const int row_rect_w = 112;
  const int row_rect_h = 20;
  const int row_step = 22;
  const int text_baseline = full_screen ? 35 : 142;

  for (uint8_t i = 0; i < values.row_count && i < MAX_LIST_ROWS; ++i) {
    const int row_y = row_base_y + row_step * (int)i;
    const bool selected = i == values.selected_row && !values.rows[i].disabled;
    if (selected) {
      u8g2_DrawBox(u8g2, row_rect_x, row_y, row_rect_w, row_rect_h);
      u8g2_SetDrawColor(u8g2, 1);
    }
    u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
    draw_text(u8g2, 10, text_baseline + row_step * (int)i, values.rows[i].text);
    if (selected) {
      u8g2_SetDrawColor(u8g2, 0);
    }
  }

  if (values.show_separator_after_back) {
    u8g2_DrawHLine(u8g2, 7, 69, 108);
  }
}

static void draw_about(u8g2_t *u8g2, const Values &values) {
  u8g2_DrawBox(u8g2, 0, FULL_SCREEN_BG_Y, CONTENT_W, FULL_SCREEN_BG_H);
  u8g2_SetDrawColor(u8g2, 1);
  u8g2_DrawBox(u8g2, 0, FULL_SCREEN_BG_Y, CONTENT_W, FULL_SCREEN_BG_H);
  u8g2_SetDrawColor(u8g2, 0);
  draw_list_rows(u8g2, values, true);
  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  draw_text(u8g2, 10, 91, values.about_title);
  draw_text(u8g2, 10, 106, values.about_firmware);
  draw_text(u8g2, 10, 120, values.about_serial);
  draw_text(u8g2, 10, 133, values.about_hardware);
}

static void draw_full_screen_list(u8g2_t *u8g2, const Values &values) {
  u8g2_DrawBox(u8g2, 0, FULL_SCREEN_BG_Y, CONTENT_W, FULL_SCREEN_BG_H);
  u8g2_SetDrawColor(u8g2, 1);
  u8g2_DrawBox(u8g2, 0, FULL_SCREEN_BG_Y, CONTENT_W, FULL_SCREEN_BG_H);
  u8g2_SetDrawColor(u8g2, 0);
  draw_list_rows(u8g2, values, true);
}

static void draw_main_menu(u8g2_t *u8g2, const Values &values) {
  u8g2_DrawBox(u8g2, 0, MAIN_MENU_BG_Y, CONTENT_W, MAIN_MENU_BG_H);
  u8g2_SetDrawColor(u8g2, 1);
  u8g2_DrawBox(u8g2, 0, MAIN_MENU_BG_Y, CONTENT_W, MAIN_MENU_BG_H);
  u8g2_SetDrawColor(u8g2, 0);
  draw_list_rows(u8g2, values, false);
}

static void draw_snackbar(u8g2_t *u8g2, const char *text) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }
  u8g2_DrawBox(u8g2, 0, SNACKBAR_Y, CONTENT_W, SNACKBAR_H);
  u8g2_SetDrawColor(u8g2, 1);
  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  draw_centered_text(u8g2, CONTENT_W / 2, 241, text);
  u8g2_SetDrawColor(u8g2, 0);
}

static void draw_shutdown(u8g2_t *u8g2) {
  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  draw_centered_text(u8g2, CONTENT_W / 2, 115, "Powering off...");
  draw_centered_text(u8g2, CONTENT_W / 2, 135, "See you soon");
  draw_logo(u8g2, 218, 24);
}

} // namespace

Dashboard::Dashboard(Config cfg)
    : _cfg(cfg), _values(), _is_inited(false), _partial_ops(0), _u8g2(), _buf{}, _region_buf{} {
  if (_cfg.max_partial_ops == 0) {
    _cfg.max_partial_ops = 20;
  }
}

esp_err_t Dashboard::init(const Values &initial) {
  esp_err_t err = display_driver::init(_cfg.display);
  if (err != ESP_OK) {
    return err;
  }

  u8g2_SetupDisplay(&_u8g2, u8x8_d_epd_128x250_cb, u8x8_dummy_cb, u8x8_dummy_cb, u8x8_dummy_cb);
  u8g2_SetupBuffer(&_u8g2, _buf, BUF_TILE_HEIGHT, u8g2_ll_hvline_horizontal_right_lsb, U8G2_MIRROR);
  u8g2_SetFontMode(&_u8g2, 1);

  _values = initial;
  _is_inited = true;

  _render_frame(_values);
  _update_full();
  return ESP_OK;
}

void Dashboard::update(const Values &values) {
  if (!_is_inited) {
    return;
  }

  const bool header_changed = _is_header_changed(values);
  const bool can_partial =
      is_home_like(_values.screen) && is_home_like(values.screen) && !header_changed;

  _values = values;
  _render_frame(_values);

  if (!can_partial || _partial_ops >= _cfg.max_partial_ops) {
    _update_full();
    return;
  }

  _update_partial();
}

void Dashboard::clear() {
  if (!_is_inited) {
    return;
  }

  memset(_buf, 0xFF, sizeof(_buf));
  _update_full();
}

bool Dashboard::_is_header_changed(const Values &next) const {
  return _values.hour != next.hour || _values.minute != next.minute ||
         _values.battery_pct != next.battery_pct ||
         _values.is_battery_charging != next.is_battery_charging || _values.locked != next.locked ||
         _values.ble_enabled != next.ble_enabled || _values.ble_connected != next.ble_connected ||
         _values.wifi_enabled != next.wifi_enabled || _values.gps_enabled != next.gps_enabled ||
         _values.gps_fix != next.gps_fix || _values.tracking_active != next.tracking_active;
}

void Dashboard::deep_sleep() {
  ScopedBusAcquire bus;
  if (bus.err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "bus acquire failed: %s", esp_err_to_name(bus.err));
    return;
  }

  const esp_err_t sleep_err = display_driver::deep_sleep();
  if (sleep_err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "epd sleep failed: %s", esp_err_to_name(sleep_err));
  }
}

void Dashboard::_render_frame(const Values &values) {
  memset(_buf, 0xFF, sizeof(_buf));
  u8g2_SetDrawColor(&_u8g2, 0);

  if (values.screen == Screen::Shutdown) {
    draw_shutdown(&_u8g2);
    draw_snackbar(&_u8g2, values.snackbar_text);
    return;
  }

  draw_status_bar(&_u8g2, values);

  switch (values.screen) {
  case Screen::Home:
    draw_home(&_u8g2, values);
    break;
  case Screen::MainMenu:
    draw_home(&_u8g2, values);
    draw_main_menu(&_u8g2, values);
    break;
  case Screen::Settings:
  case Screen::SettingsChoice:
  case Screen::TagList:
  case Screen::Confirm:
    draw_full_screen_list(&_u8g2, values);
    break;
  case Screen::About:
    draw_about(&_u8g2, values);
    break;
  case Screen::Shutdown:
    break;
  }

  draw_snackbar(&_u8g2, values.snackbar_text);
}

void Dashboard::_copy_full_width_region(int y_start, int h) {
  for (int row = 0; row < h; ++row) {
    const int src_off = (y_start + row) * BUF_ROW_BYTES;
    const int dst_off = row * BUF_ROW_BYTES;
    memcpy(_region_buf + dst_off, _buf + src_off, BUF_ROW_BYTES);
  }
}

void Dashboard::_update_full() {
  const esp_err_t err = display_driver::bus_acquire();
  if (err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "bus acquire failed: %s", esp_err_to_name(err));
    return;
  }

  esp_err_t op_err = display_driver::hw_init_full();
  if (op_err == ESP_OK) {
    op_err = display_driver::set_ram_value_base_map(_buf);
  }

  display_driver::bus_release();

  if (op_err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "full update failed: %s", esp_err_to_name(op_err));
    return;
  }

  _partial_ops = 0;
}

void Dashboard::_update_partial() {
  const esp_err_t err = display_driver::bus_acquire();
  if (err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "bus acquire failed: %s", esp_err_to_name(err));
    return;
  }

  esp_err_t op_err = display_driver::part_begin();
  if (op_err != ESP_OK) {
    display_driver::bus_release();
    ESP_LOGE(LOG_TAG, "partial begin failed: %s", esp_err_to_name(op_err));
    return;
  }

  _copy_full_width_region(BODY_Y, BODY_H);
  op_err = display_driver::part_write_region(0, BODY_Y, _region_buf, BODY_H, SCREEN_W);
  if (op_err != ESP_OK) {
    display_driver::bus_release();
    ESP_LOGE(LOG_TAG, "partial body failed: %s", esp_err_to_name(op_err));
    return;
  }

  op_err = display_driver::part_commit();
  display_driver::bus_release();
  if (op_err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "partial commit failed: %s", esp_err_to_name(op_err));
    return;
  }

  _partial_ops++;
}

} // namespace dashboard
