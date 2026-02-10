// SPDX-License-Identifier: MIT

#ifndef GO_DISPLAY_UI_U8G2_CANVAS_H
#define GO_DISPLAY_UI_U8G2_CANVAS_H

#include <stddef.h>
#include <stdint.h>

#include "u8g2.h"

namespace ui {

// u8g2 helper to draw into a packed 1bpp buffer (8 pixels per byte, MSB first).
// Buffer is assumed to use: 1 = white, 0 = black (EPD-friendly).
class U8g2Canvas {
 public:
  U8g2Canvas();

  // Attach a buffer and set up u8g2 for drawing.
  // - w must be a multiple of 8.
  // - h is the logical/visible height.
  // - buf_len must be >= bytes_per_row * ceil(h/8)*8.
  void attach(uint8_t* buf, size_t buf_len, int w, int h, bool mirror_x);

  void clear_white();

  void set_font(const uint8_t* font);

  // Draw black text (clears bits).
  void set_color_black();

  int str_width(const char* s) const;
  int max_char_height() const;

  void draw_str(int x, int y_top, const char* s);
  void draw_str_centered(int x, int y, int w, int h, const char* s);

  void draw_hline(int x, int y, int w);

 private:
  void setup_(uint8_t* buf, size_t buf_len, int w, int h, bool mirror_x);

  u8g2_t u8g2_{};
  u8x8_display_info_t display_info_{};
  uint8_t* buf_ = nullptr;
  size_t buf_len_ = 0;
  int w_ = 0;
  int h_ = 0;
  int padded_h_ = 0;
};

}  // namespace ui

#endif
