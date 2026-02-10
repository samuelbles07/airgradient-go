// SPDX-License-Identifier: MIT

#ifndef EDP_SSD1680X_PANELS_GDEY0213B74_H
#define EDP_SSD1680X_PANELS_GDEY0213B74_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "ssd1680x.h"

namespace ssd1680x::panels {

// Good Display: GDEY0213B74 (SSD1680Z), 2.13" b/w.
// Controller addressing is 128x250; some variants list a smaller visible width.
class GDEY0213B74 : public ssd1680x::Device {
 public:
  static constexpr int WIDTH = 128;
  static constexpr int HEIGHT = 250;
  static constexpr size_t BUFFER_SIZE = (WIDTH * HEIGHT) / 8;

  explicit GDEY0213B74(const ssd1680x::Config& cfg);

  bool is_asleep() const { return _asleep; }

  // Forces a full initialization sequence.
  esp_err_t init_full();
  // Calls init_full() only if we are asleep or in a different mode.
  esp_err_t ensure_init_full();

  // Fast refresh mode (panel-specific; matches the reference init/update sequence).
  esp_err_t init_fast();
  esp_err_t ensure_init_fast();

  esp_err_t clear_white();
  esp_err_t clear_black();
  esp_err_t display_frame_bw(const uint8_t* buf, size_t len);
  esp_err_t display_frame_bw_fast(const uint8_t* buf, size_t len);

  // Base-map write required for stable partial refresh.
  esp_err_t set_basemap_bw(const uint8_t* buf, size_t len);
  // Base-map write using fast update (still writes RAM1+RAM2).
  esp_err_t set_basemap_bw_fast(const uint8_t* buf, size_t len);

  // Partial helpers to batch multiple window writes then update once.
  // partial_begin() must be called before any partial_write_bw() calls.
  // partial_write_bw() writes a window into RAM (no update).
  // partial_end_update() triggers the partial update sequence and ends the batch.
  //
  // Notes:
  // - Basemap must be set (set_basemap_bw) before partial operations.
  esp_err_t partial_begin();
  esp_err_t partial_write_bw(int x, int y, int w, int h, const uint8_t* buf, size_t len);
  esp_err_t partial_end_update();

  // Backwards-compatible alias.
  esp_err_t partial_update();

  // Full-screen partial update.
  esp_err_t display_partial_bw_all(const uint8_t* buf, size_t len);
  // Partial update of a window; width must be a multiple of 8.
  // buf format is 1bpp, row-major, window-sized (w*h/8 bytes).
  esp_err_t display_partial_bw(int x, int y, int w, int h, const uint8_t* buf, size_t len);

  esp_err_t deep_sleep();

 private:
  enum class Mode {
    Unknown = 0,
    Full,
    Fast,
  };

  // Tx-only helpers. These should be called with the SPI bus acquired.
  esp_err_t _set_ram_addr_defaults_full();
  esp_err_t _set_full_window_tx();
  esp_err_t _prep_full_tx();
  esp_err_t _trigger_update_full_tx();
  esp_err_t _trigger_update_fast_tx();
  esp_err_t _trigger_update_partial_tx();

  esp_err_t _write_ram(uint8_t ram_cmd, const uint8_t* buf, size_t len);
  esp_err_t _set_window(uint8_t x_start_bytes, uint8_t x_end_bytes, uint16_t y_start, uint16_t y_end);
  esp_err_t _set_cursor(uint8_t x_bytes, uint16_t y);

  bool _asleep = true;
  bool _partial_prepared = false;
  bool _basemap_valid = false;
  Mode _mode = Mode::Unknown;
};

}  // namespace ssd1680x::panels

#endif
