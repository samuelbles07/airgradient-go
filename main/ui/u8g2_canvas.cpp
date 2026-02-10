// SPDX-License-Identifier: MIT

#include "ui/u8g2_canvas.h"

#include <string.h>

namespace ui {

static int pad8(int v) {
  return (v + 7) & ~7;
}

U8g2Canvas::U8g2Canvas() {}

static void draw_pixel_horizontal_right_lsb(u8g2_t* u8g2, u8g2_uint_t x, u8g2_uint_t y) {
  const uint8_t tile_width = u8g2_GetU8x8(u8g2)->display_info->tile_width;

  uint8_t bit_pos = (uint8_t)(x & 7);
  uint8_t mask = (uint8_t)(128u >> bit_pos);
  const uint16_t offset = (uint16_t)y * (uint16_t)tile_width + (uint16_t)(x >> 3);
  uint8_t* ptr = u8g2->tile_buf_ptr + offset;

  if (u8g2->draw_color <= 1) {
    *ptr |= mask;
  }
  if (u8g2->draw_color != 1) {
    *ptr ^= mask;
  }
}

// Same memory format as u8g2_ll_hvline_horizontal_right_lsb, but mirrors X while
// keeping the caller's coordinate system non-mirrored.
static void u8g2_ll_hvline_horizontal_right_lsb_mirror_x(u8g2_t* u8g2, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t len,
                                                        uint8_t dir) {
  const u8g2_uint_t w = (u8g2_uint_t)u8g2_GetU8x8(u8g2)->display_info->pixel_width;
  if (w == 0) {
    return;
  }
  if (dir == 0) {
    for (u8g2_uint_t i = 0; i < len; i++) {
      const u8g2_uint_t xm = (w - 1) - (x + i);
      draw_pixel_horizontal_right_lsb(u8g2, xm, y);
    }
  } else {
    const u8g2_uint_t xm = (w - 1) - x;
    for (u8g2_uint_t i = 0; i < len; i++) {
      draw_pixel_horizontal_right_lsb(u8g2, xm, y + i);
    }
  }
}

void U8g2Canvas::attach(uint8_t* buf, size_t buf_len, int w, int h, bool mirror_x) {
  setup_(buf, buf_len, w, h, mirror_x);
}

void U8g2Canvas::setup_(uint8_t* buf, size_t buf_len, int w, int h, bool mirror_x) {
  buf_ = buf;
  buf_len_ = buf_len;
  w_ = w;
  h_ = h;
  padded_h_ = pad8(h);

  // Width must be byte-aligned.
  if (buf_ == nullptr || w_ <= 0 || h_ <= 0 || (w_ % 8) != 0) {
    return;
  }

  const size_t bytes_per_row = (size_t)(w_ / 8);
  const size_t needed = bytes_per_row * (size_t)padded_h_;
  if (buf_len_ < needed) {
    // Invalid; leave object configured but drawing will be clipped by buffer size.
    return;
  }

  // Prepare a synthetic display info so u8g2 can compute dimensions.
  memset(&display_info_, 0, sizeof(display_info_));
  display_info_.tile_width = (uint8_t)(w_ / 8);
  display_info_.tile_height = (uint8_t)(padded_h_ / 8);
  display_info_.pixel_width = (uint16_t)w_;
  display_info_.pixel_height = (uint16_t)padded_h_;

  // Set up u8x8 defaults and assign display_info.
  u8x8_t* u8x8 = u8g2_GetU8x8(&u8g2_);
  u8x8_SetupDefaults(u8x8);
  u8x8_d_helper_display_setup_memory(u8x8, &display_info_);

  const u8g2_cb_t* rot = U8G2_R0;
  u8g2_draw_ll_hvline_cb ll = mirror_x ? u8g2_ll_hvline_horizontal_right_lsb_mirror_x
                                       : u8g2_ll_hvline_horizontal_right_lsb;
  u8g2_SetupBuffer(&u8g2_, buf_, (uint8_t)(padded_h_ / 8), ll, rot);

  // Clamp all drawing to the real height.
  u8g2_SetClipWindow(&u8g2_, 0, 0, (u8g2_uint_t)w_, (u8g2_uint_t)h_);
  u8g2_SetFontPosTop(&u8g2_);
  u8g2_SetDrawColor(&u8g2_, 0);
}

void U8g2Canvas::clear_white() {
  if (buf_ == nullptr || w_ <= 0 || h_ <= 0) {
    return;
  }
  const size_t bytes_per_row = (size_t)(w_ / 8);
  const size_t needed = bytes_per_row * (size_t)padded_h_;
  if (buf_len_ < needed) {
    return;
  }
  memset(buf_, 0xFF, needed);
}

void U8g2Canvas::set_font(const uint8_t* font) {
  if (font == nullptr) {
    return;
  }
  u8g2_SetFont(&u8g2_, font);
}

void U8g2Canvas::set_color_black() {
  u8g2_SetDrawColor(&u8g2_, 0);
}

int U8g2Canvas::str_width(const char* s) const {
  if (s == nullptr) {
    return 0;
  }
  return (int)u8g2_GetStrWidth((u8g2_t*)&u8g2_, s);
}

int U8g2Canvas::max_char_height() const {
  return (int)u8g2_GetMaxCharHeight((u8g2_t*)&u8g2_);
}

void U8g2Canvas::draw_str(int x, int y_top, const char* s) {
  if (s == nullptr) {
    return;
  }
  u8g2_DrawStr(&u8g2_, (u8g2_uint_t)x, (u8g2_uint_t)y_top, s);
}

void U8g2Canvas::draw_str_centered(int x, int y, int w, int h, const char* s) {
  if (s == nullptr) {
    return;
  }
  const int tw = str_width(s);
  const int th = max_char_height();
  int sx = x + (w - tw) / 2;
  int sy = y + (h - th) / 2;
  if (sx < x) sx = x;
  if (sy < y) sy = y;
  draw_str(sx, sy, s);
}

void U8g2Canvas::draw_hline(int x, int y, int w) {
  if (w <= 0) {
    return;
  }
  u8g2_DrawHLine(&u8g2_, (u8g2_uint_t)x, (u8g2_uint_t)y, (u8g2_uint_t)w);
}

}  // namespace ui
