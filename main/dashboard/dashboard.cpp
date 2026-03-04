#include "dashboard/dashboard.h"

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

} // namespace

static constexpr int SCREEN_W = 128;

static constexpr int BORDER = 2;
static constexpr int HEADER_Y = 18;
static constexpr int FOOTER_BASELINE_Y = 244;
static constexpr int FOOTER_LABEL_BASELINE_Y = FOOTER_BASELINE_Y - 20;

static constexpr int FOOTER_MARGIN_X = 14;
static constexpr int FOOTER_CELL_GAP_X = 12;
static constexpr int FOOTER_CELL_W = (SCREEN_W - 2 * FOOTER_MARGIN_X - FOOTER_CELL_GAP_X) / 2;
static constexpr int FOOTER_LEFT_CELL_X = FOOTER_MARGIN_X;
static constexpr int FOOTER_RIGHT_CELL_X = FOOTER_MARGIN_X + FOOTER_CELL_W + FOOTER_CELL_GAP_X;

static constexpr int HEADER_BAR_X = 8;
static constexpr int HEADER_BAR_W = 112;

// Visual adjustment: u8g2 ascent/descent centering tends to look too high in
// short header bands.
static constexpr int HEADER_CONTENT_Y = 4;
static constexpr int HEADER_CONTENT_H = HEADER_Y - HEADER_CONTENT_Y;

static constexpr int HEADER_SEPARATOR_H = 2;
static constexpr int MAIN_PAD_Y = 7;
static constexpr int MAIN_TOP_Y = HEADER_Y + HEADER_SEPARATOR_H + MAIN_PAD_Y;

static constexpr int CO2_LABEL_Y = MAIN_TOP_Y;
static constexpr int CO2_LABEL_H = 36;
static constexpr int CO2_VALUE_Y = CO2_LABEL_Y + 38;
static constexpr int CO2_VALUE_H = 52;

static constexpr int PM_LABEL_Y = CO2_LABEL_Y + 86;
static constexpr int PM_LABEL_H = 36;
static constexpr int PM_VALUE_Y = CO2_LABEL_Y + 122;
static constexpr int PM_VALUE_H = 50;

static constexpr int FOOTER_LINE_Y = PM_VALUE_Y + PM_VALUE_H + MAIN_PAD_Y;

// Partial refresh regions (full width, byte-aligned).
static constexpr int REGION_CO2_Y = CO2_VALUE_Y - 8;
static constexpr int REGION_CO2_H = 68;

static constexpr int REGION_PM_Y = PM_VALUE_Y - 8;
static constexpr int REGION_PM_H = 58;

static constexpr int REGION_FOOTER_Y = 222;
static constexpr int REGION_FOOTER_H = 28;

static_assert(REGION_CO2_H <= Dashboard::REGION_MAX_H, "region buf too small");
static_assert(REGION_PM_H <= Dashboard::REGION_MAX_H, "region buf too small");
static_assert(REGION_FOOTER_H <= Dashboard::REGION_MAX_H, "region buf too small");

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

static void draw_centered_str(u8g2_t *u8g2, int x, int y, int w, int h, const char *str) {
  const int sw = (int)u8g2_GetStrWidth(u8g2, str);
  const int ascent = (int)u8g2_GetAscent(u8g2);
  const int descent = (int)u8g2_GetDescent(u8g2);
  const int sh = ascent - descent;

  const int sx = x + (w - sw) / 2;
  const int baseline = y + (h - sh) / 2 + ascent;
  u8g2_DrawStr(u8g2, (u8g2_uint_t)sx, (u8g2_uint_t)baseline, str);
}

static void draw_left_str_vcentered(u8g2_t *u8g2, int x, int y, int h, const char *str) {
  const int ascent = (int)u8g2_GetAscent(u8g2);
  const int descent = (int)u8g2_GetDescent(u8g2);
  const int sh = ascent - descent;
  const int baseline = y + (h - sh) / 2 + ascent;
  u8g2_DrawStr(u8g2, (u8g2_uint_t)x, (u8g2_uint_t)baseline, str);
}

static void format_time_hhmm(char out[6], uint8_t hour, uint8_t minute) {
  const uint8_t hh = (uint8_t)(hour % 24U);
  const uint8_t mm = (uint8_t)(minute % 60U);

  out[0] = (char)('0' + (hh / 10U));
  out[1] = (char)('0' + (hh % 10U));
  out[2] = ':';
  out[3] = (char)('0' + (mm / 10U));
  out[4] = (char)('0' + (mm % 10U));
  out[5] = '\0';
}

static void draw_glyph_vcentered(u8g2_t *u8g2, int x, int y, int h, uint16_t encoding) {
  const int ascent = (int)u8g2_GetAscent(u8g2);
  const int descent = (int)u8g2_GetDescent(u8g2);
  const int sh = ascent - descent;
  const int baseline = y + (h - sh) / 2 + ascent;
  u8g2_DrawGlyph(u8g2, (u8g2_uint_t)x, (u8g2_uint_t)baseline, encoding);
}

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

static constexpr int STATUS_SLOT_W = 16;

struct StatusIcon {
  const uint8_t *font;
  uint16_t encoding;
};

static StatusIcon status_icon_for_bit(uint8_t bit) {
  switch (bit) {
  case STATUS_GPS_FIX:
    // Open Iconic WWW: 0047
    return StatusIcon{u8g2_font_open_iconic_www_2x_t, 0x0047};
  case STATUS_TRACKING:
    // Open Iconic WWW: 0046
    return StatusIcon{u8g2_font_open_iconic_www_2x_t, 0x0046};
  case STATUS_SYNC:
    // Open Iconic Arrow: 0057
    return StatusIcon{u8g2_font_open_iconic_arrow_2x_t, 0x0057};
  default:
    return StatusIcon{nullptr, 0};
  }
}

static uint8_t status_icons(uint8_t mask, StatusIcon out[3]) {
  uint8_t count = 0;
  auto add = [&](uint8_t bit) {
    if ((mask & bit) == 0) {
      return;
    }

    const StatusIcon icon = status_icon_for_bit(bit);
    if (icon.font == nullptr) {
      return;
    }
    out[count++] = icon;
  };

  // Priority order: sync, GPS fix, tracking.
  add(STATUS_SYNC);
  add(STATUS_GPS_FIX);
  add(STATUS_TRACKING);
  return count;
}

static void draw_centered_label_with_unit(u8g2_t *u8g2, int x, int y, int w, int h,
                                          const uint8_t *label_font, const char *label,
                                          const uint8_t *unit_font, const char *unit) {
  u8g2_SetFont(u8g2, label_font);
  const int label_w = (int)u8g2_GetStrWidth(u8g2, label);
  const int label_ascent = (int)u8g2_GetAscent(u8g2);
  const int label_descent = (int)u8g2_GetDescent(u8g2);

  u8g2_SetFont(u8g2, unit_font);
  const int unit_w = (int)u8g2_GetStrWidth(u8g2, unit);
  const int unit_ascent = (int)u8g2_GetAscent(u8g2);
  const int unit_descent = (int)u8g2_GetDescent(u8g2);

  const int gap = 2;
  const int total_w = label_w + gap + unit_w;
  const int sx = x + (w - total_w) / 2;

  const int max_ascent = (label_ascent > unit_ascent) ? label_ascent : unit_ascent;
  const int min_descent = (label_descent < unit_descent) ? label_descent : unit_descent;
  const int max_h = max_ascent - min_descent;
  const int baseline = y + (h - max_h) / 2 + max_ascent;

  u8g2_SetFont(u8g2, label_font);
  u8g2_DrawStr(u8g2, (u8g2_uint_t)sx, (u8g2_uint_t)baseline, label);

  u8g2_SetFont(u8g2, unit_font);
  u8g2_DrawStr(u8g2, (u8g2_uint_t)(sx + label_w + gap), (u8g2_uint_t)baseline, unit);
}

Dashboard::Dashboard(Config cfg)
    : _cfg(cfg), _values{}, _is_inited(false), _partial_ops(0), _u8g2(), _buf{}, _region_buf{} {
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

  _values = values;
  _render_frame(_values);

  if (header_changed) {
    _update_full();
    return;
  }

  if (_partial_ops >= _cfg.max_partial_ops) {
    _update_full();
  } else {
    _update_partial();
  }
}

bool Dashboard::_is_header_changed(const Values &next) const {
  if ((_values.hour != next.hour) || (_values.minute != next.minute)) {
    return true;
  }

  if (battery_glyph(_values.is_battery_charging, _values.battery_pct) !=
      battery_glyph(next.is_battery_charging, next.battery_pct)) {
    return true;
  }

  if (_values.status_mask != next.status_mask) {
    return true;
  }

  return false;
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

  const int header_margin_x = 12;
  const int header_gap_x = 2;

  int header_left_bound = header_margin_x;
  int header_right_bound = SCREEN_W - header_margin_x;

  // Inverted header background.
  u8g2_DrawBox(&_u8g2, HEADER_BAR_X, 0, HEADER_BAR_W, HEADER_Y);

  // Header content in white.
  u8g2_SetDrawColor(&_u8g2, 1);

  // Header: time (top-left).
  {
    char time_buf[6];
    format_time_hhmm(time_buf, values.hour, values.minute);
    u8g2_SetFont(&_u8g2, u8g2_font_helvB08_tf);
    draw_left_str_vcentered(&_u8g2, header_margin_x, HEADER_CONTENT_Y, HEADER_CONTENT_H, time_buf);
    header_left_bound = header_margin_x + (int)u8g2_GetStrWidth(&_u8g2, time_buf) + header_gap_x;
  }

  // Header: battery (top-right).
  {
    constexpr int BATTERY_SLOT_W = 16;
    const int slot_x = SCREEN_W - header_margin_x - BATTERY_SLOT_W;

    const uint16_t glyph = battery_glyph(values.is_battery_charging, values.battery_pct);
    u8g2_SetFont(&_u8g2, u8g2_font_siji_t_6x10);
    int glyph_w = (int)u8g2_GetGlyphWidth(&_u8g2, glyph);
    if (glyph_w < 0) {
      glyph_w = 0;
    }
    if (glyph_w > BATTERY_SLOT_W) {
      glyph_w = BATTERY_SLOT_W;
    }
    const int x = slot_x + (BATTERY_SLOT_W - glyph_w) / 2;
    draw_glyph_vcentered(&_u8g2, x, HEADER_CONTENT_Y, HEADER_CONTENT_H, glyph);

    header_right_bound = slot_x - header_gap_x;
  }

  // Header: status icons (top-middle).
  {
    StatusIcon icons[3] = {};
    const int icon_count = (int)status_icons(values.status_mask, icons);
    if (icon_count <= 0) {
      // Nothing to draw.
    } else {
      const int icon_gap = 2;
      const int total_w = icon_count * STATUS_SLOT_W + (icon_count - 1) * icon_gap;

      int start_x = header_right_bound - total_w;
      if (start_x < header_left_bound) {
        start_x = header_left_bound;
      }

      for (int i = 0; i < icon_count; ++i) {
        const int slot_x = start_x + i * (STATUS_SLOT_W + icon_gap);
        u8g2_SetFont(&_u8g2, icons[i].font);

        int glyph_w = (int)u8g2_GetGlyphWidth(&_u8g2, icons[i].encoding);
        if (glyph_w < 0) {
          glyph_w = 0;
        }
        if (glyph_w > STATUS_SLOT_W) {
          glyph_w = STATUS_SLOT_W;
        }
        const int x = slot_x + (STATUS_SLOT_W - glyph_w) / 2;
        draw_glyph_vcentered(&_u8g2, x, HEADER_CONTENT_Y, HEADER_CONTENT_H, icons[i].encoding);
      }
    }
  }

  // Back to black for the rest of the UI.
  u8g2_SetDrawColor(&_u8g2, 0);

  // Separator lines.
  u8g2_DrawHLine(&_u8g2, 8, HEADER_Y, 112);
  u8g2_DrawHLine(&_u8g2, 8, HEADER_Y + 1, 112);
  u8g2_DrawHLine(&_u8g2, 8, FOOTER_LINE_Y, 112);
  u8g2_DrawHLine(&_u8g2, 8, FOOTER_LINE_Y + 1, 112);

  // CO2 label and value.
  draw_centered_label_with_unit(&_u8g2, BORDER, CO2_LABEL_Y, SCREEN_W - 2 * BORDER, CO2_LABEL_H,
                                u8g2_font_logisoso18_tf, "CO2", u8g2_font_helvR12_tf, "(ppm)");
  char co2_buf[8];
  snprintf(co2_buf, sizeof(co2_buf), "%d", values.co2_ppm);
  u8g2_SetFont(&_u8g2, u8g2_font_logisoso38_tn);
  draw_centered_str(&_u8g2, BORDER, CO2_VALUE_Y, SCREEN_W - 2 * BORDER, CO2_VALUE_H, co2_buf);

  // PM2.5 label and value.
  draw_centered_label_with_unit(&_u8g2, BORDER, PM_LABEL_Y, SCREEN_W - 2 * BORDER, PM_LABEL_H,
                                u8g2_font_logisoso18_tf, "PM2.5", u8g2_font_helvR12_tf, "(ug/m3)");
  char pm_buf[8];
  snprintf(pm_buf, sizeof(pm_buf), "%d", values.pm25_ugm3);
  u8g2_SetFont(&_u8g2, u8g2_font_logisoso38_tn);
  draw_centered_str(&_u8g2, BORDER, PM_VALUE_Y, SCREEN_W - 2 * BORDER, PM_VALUE_H, pm_buf);

  // Footer: temperature.
  char temp_buf[8];
  snprintf(temp_buf, sizeof(temp_buf), "%d", values.temperature_c);
  {
    u8g2_SetFont(&_u8g2, u8g2_font_helvB14_tn);
    const int value_w = (int)u8g2_GetStrWidth(&_u8g2, temp_buf);

    u8g2_SetFont(&_u8g2, u8g2_font_helvR14_tf);
    constexpr uint16_t DEGREE_GLYPH = 0x00B0;
    int degree_w = (int)u8g2_GetGlyphWidth(&_u8g2, DEGREE_GLYPH);
    if (degree_w < 0) {
      degree_w = 0;
    }
    const int unit_w = (int)u8g2_GetStrWidth(&_u8g2, "C");

    const int group_w = value_w + 4 + degree_w + unit_w;
    const int group_x = FOOTER_LEFT_CELL_X + (FOOTER_CELL_W - group_w) / 2;

    u8g2_SetFont(&_u8g2, u8g2_font_helvB08_tf);
    const int label_w = (int)u8g2_GetStrWidth(&_u8g2, "Temp");
    const int label_x = FOOTER_LEFT_CELL_X + (FOOTER_CELL_W - label_w) / 2;
    u8g2_DrawStr(&_u8g2, (u8g2_uint_t)label_x, (u8g2_uint_t)FOOTER_LABEL_BASELINE_Y, "Temp");

    u8g2_SetFont(&_u8g2, u8g2_font_helvB14_tn);
    u8g2_DrawStr(&_u8g2, (u8g2_uint_t)group_x, (u8g2_uint_t)FOOTER_BASELINE_Y, temp_buf);

    u8g2_SetFont(&_u8g2, u8g2_font_helvR14_tf);
    const int unit_x = group_x + value_w + 4;
    if (degree_w > 0) {
      u8g2_DrawGlyph(&_u8g2, (u8g2_uint_t)unit_x, (u8g2_uint_t)FOOTER_BASELINE_Y, DEGREE_GLYPH);
    }
    u8g2_DrawStr(&_u8g2, (u8g2_uint_t)(unit_x + degree_w), (u8g2_uint_t)FOOTER_BASELINE_Y, "C");
  }

  // Footer: humidity.
  char hum_buf[8];
  snprintf(hum_buf, sizeof(hum_buf), "%d", values.humidity_pct);
  {
    u8g2_SetFont(&_u8g2, u8g2_font_helvB14_tn);
    const int value_w = (int)u8g2_GetStrWidth(&_u8g2, hum_buf);

    u8g2_SetFont(&_u8g2, u8g2_font_helvR14_tf);
    const int unit_w = (int)u8g2_GetStrWidth(&_u8g2, "%");

    const int group_w = value_w + 4 + unit_w;
    const int group_x = FOOTER_RIGHT_CELL_X + (FOOTER_CELL_W - group_w) / 2;

    u8g2_SetFont(&_u8g2, u8g2_font_helvB08_tf);
    const int label_w = (int)u8g2_GetStrWidth(&_u8g2, "Hum");
    const int label_x = FOOTER_RIGHT_CELL_X + (FOOTER_CELL_W - label_w) / 2;
    u8g2_DrawStr(&_u8g2, (u8g2_uint_t)label_x, (u8g2_uint_t)FOOTER_LABEL_BASELINE_Y, "Hum");

    u8g2_SetFont(&_u8g2, u8g2_font_helvB14_tn);
    u8g2_DrawStr(&_u8g2, (u8g2_uint_t)group_x, (u8g2_uint_t)FOOTER_BASELINE_Y, hum_buf);

    u8g2_SetFont(&_u8g2, u8g2_font_helvR14_tf);
    u8g2_DrawStr(&_u8g2, (u8g2_uint_t)(group_x + value_w + 4), (u8g2_uint_t)FOOTER_BASELINE_Y, "%");
  }
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

  _copy_full_width_region(REGION_CO2_Y, REGION_CO2_H);
  op_err = display_driver::part_write_region(0, REGION_CO2_Y, _region_buf, REGION_CO2_H, REGION_W);
  if (op_err != ESP_OK) {
    display_driver::bus_release();
    ESP_LOGE(LOG_TAG, "partial CO2 failed: %s", esp_err_to_name(op_err));
    return;
  }

  _copy_full_width_region(REGION_PM_Y, REGION_PM_H);
  op_err = display_driver::part_write_region(0, REGION_PM_Y, _region_buf, REGION_PM_H, REGION_W);
  if (op_err != ESP_OK) {
    display_driver::bus_release();
    ESP_LOGE(LOG_TAG, "partial PM failed: %s", esp_err_to_name(op_err));
    return;
  }

  _copy_full_width_region(REGION_FOOTER_Y, REGION_FOOTER_H);
  op_err =
      display_driver::part_write_region(0, REGION_FOOTER_Y, _region_buf, REGION_FOOTER_H, REGION_W);
  if (op_err != ESP_OK) {
    display_driver::bus_release();
    ESP_LOGE(LOG_TAG, "partial footer failed: %s", esp_err_to_name(op_err));
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
