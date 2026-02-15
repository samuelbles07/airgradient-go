// SPDX-License-Identifier: MIT

#include "gdey0213b74.h"
#include "edp_err_macros.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ssd1680x::panels {

static uint8_t reverse_bits8(uint8_t v) {
  v = (uint8_t)(((v & 0xF0u) >> 4) | ((v & 0x0Fu) << 4));
  v = (uint8_t)(((v & 0xCCu) >> 2) | ((v & 0x33u) << 2));
  v = (uint8_t)(((v & 0xAAu) >> 1) | ((v & 0x55u) << 1));
  return v;
}

GDEY0213B74::GDEY0213B74(const ssd1680x::Config& cfg) : ssd1680x::Device(cfg) {}

esp_err_t GDEY0213B74::_set_ram_addr_defaults_full() {
  EDP_RETURN_ON_ERROR(write_cmd(0x4E));
  EDP_RETURN_ON_ERROR(write_data_u8(0x00));

  const uint16_t HEIGHT_M1 = (uint16_t)(HEIGHT - 1);
  EDP_RETURN_ON_ERROR(write_cmd(0x4F));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 & 0xFF)));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 >> 8)));

  return ESP_OK;
}

esp_err_t GDEY0213B74::_set_full_window_tx() {
  const uint16_t HEIGHT_M1 = (uint16_t)(HEIGHT - 1);
  const uint8_t X_END = (uint8_t)(WIDTH / 8 - 1);

  EDP_RETURN_ON_ERROR(write_cmd(0x11));
  EDP_RETURN_ON_ERROR(write_data_u8(0x01));

  EDP_RETURN_ON_ERROR(write_cmd(0x44));
  EDP_RETURN_ON_ERROR(write_data_u8(0x00));
  EDP_RETURN_ON_ERROR(write_data_u8(X_END));

  // Full init uses Y start=HEIGHT-1, Y end=0.
  EDP_RETURN_ON_ERROR(write_cmd(0x45));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 & 0xFF)));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 >> 8)));
  EDP_RETURN_ON_ERROR(write_data_u8(0x00));
  EDP_RETURN_ON_ERROR(write_data_u8(0x00));

  return ESP_OK;
}

esp_err_t GDEY0213B74::_prep_full_tx() {
  // Restore full-mode border and addressing in case partial updates modified them.
  EDP_RETURN_ON_ERROR(write_cmd(0x3C));
  EDP_RETURN_ON_ERROR(write_data_u8(0x05));
  EDP_RETURN_ON_ERROR(_set_full_window_tx());
  return _set_ram_addr_defaults_full();
}

esp_err_t GDEY0213B74::_trigger_update_full_tx() {
  EDP_RETURN_ON_ERROR(write_cmd(0x22));
  EDP_RETURN_ON_ERROR(write_data_u8(0xF7));
  return write_cmd(0x20);
}

esp_err_t GDEY0213B74::_trigger_update_fast_tx() {
  EDP_RETURN_ON_ERROR(write_cmd(0x22));
  EDP_RETURN_ON_ERROR(write_data_u8(0xC7));
  return write_cmd(0x20);
}

esp_err_t GDEY0213B74::_trigger_update_partial_tx() {
  EDP_RETURN_ON_ERROR(write_cmd(0x22));
  EDP_RETURN_ON_ERROR(write_data_u8(0xFF));
  return write_cmd(0x20);
}

esp_err_t GDEY0213B74::_write_ram(uint8_t ram_cmd, const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  EDP_RETURN_ON_ERROR(write_cmd(ram_cmd));
  return write_data(buf, len);
}

esp_err_t GDEY0213B74::_write_ram_maybe_mirror_x(uint8_t ram_cmd, const uint8_t* buf, int w, int h, size_t len) {
  if (buf == nullptr || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (w <= 0 || h <= 0 || (w % 8) != 0) {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t expected = (size_t)(w * h) / 8;
  if (len != expected) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!cfg().mirror_x) {
    return _write_ram(ram_cmd, buf, len);
  }

  const int bytes_per_row = w / 8;
  uint8_t rowbuf[WIDTH / 8];
  if (bytes_per_row > (int)sizeof(rowbuf)) {
    return ESP_ERR_INVALID_ARG;
  }

  EDP_RETURN_ON_ERROR(write_cmd(ram_cmd));

  for (int row = 0; row < h; ++row) {
    const uint8_t* s = buf + (size_t)row * (size_t)bytes_per_row;
    for (int i = 0; i < bytes_per_row; ++i) {
      rowbuf[i] = reverse_bits8(s[bytes_per_row - 1 - i]);
    }
    EDP_RETURN_ON_ERROR(write_data(rowbuf, (size_t)bytes_per_row));
  }

  return ESP_OK;
}

esp_err_t GDEY0213B74::_set_window(uint8_t x_start_bytes, uint8_t x_end_bytes, uint16_t y_start,
                                  uint16_t y_end) {
  EDP_RETURN_ON_ERROR(write_cmd(0x44));
  EDP_RETURN_ON_ERROR(write_data_u8(x_start_bytes));
  EDP_RETURN_ON_ERROR(write_data_u8(x_end_bytes));

  EDP_RETURN_ON_ERROR(write_cmd(0x45));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(y_start & 0xFF)));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(y_start >> 8)));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(y_end & 0xFF)));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(y_end >> 8)));

  return ESP_OK;
}

esp_err_t GDEY0213B74::_set_cursor(uint8_t x_bytes, uint16_t y) {
  EDP_RETURN_ON_ERROR(write_cmd(0x4E));
  EDP_RETURN_ON_ERROR(write_data_u8(x_bytes));

  EDP_RETURN_ON_ERROR(write_cmd(0x4F));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(y & 0xFF)));
  EDP_RETURN_ON_ERROR(write_data_u8((uint8_t)(y >> 8)));

  return ESP_OK;
}

esp_err_t GDEY0213B74::init_full() {
  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(init());

  EDP_RETURN_ON_ERROR(reset());
  EDP_RETURN_ON_ERROR(wait_idle());

  EDP_RETURN_ON_ERROR(bus_acquire());
  err = write_cmd(0x12);
  bus_release();
  EDP_RETURN_IF_ERROR(err);

  EDP_RETURN_ON_ERROR(wait_idle());

  const uint16_t HEIGHT_M1 = (uint16_t)(HEIGHT - 1);
  const uint8_t X_END = (uint8_t)(WIDTH / 8 - 1);

  EDP_RETURN_ON_ERROR(bus_acquire());

  EDP_GOTO_ON_ERROR(write_cmd(0x01), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 & 0xFF)), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 >> 8)), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x00), out_cfg);

  EDP_GOTO_ON_ERROR(write_cmd(0x11), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x01), out_cfg);

  EDP_GOTO_ON_ERROR(write_cmd(0x44), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x00), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(X_END), out_cfg);

  EDP_GOTO_ON_ERROR(write_cmd(0x45), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 & 0xFF)), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8((uint8_t)(HEIGHT_M1 >> 8)), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x00), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x00), out_cfg);

  EDP_GOTO_ON_ERROR(write_cmd(0x3C), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x05), out_cfg);

  EDP_GOTO_ON_ERROR(write_cmd(0x21), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x00), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x80), out_cfg);

  EDP_GOTO_ON_ERROR(write_cmd(0x18), out_cfg);
  EDP_GOTO_ON_ERROR(write_data_u8(0x80), out_cfg);

  EDP_GOTO_ON_ERROR(_set_ram_addr_defaults_full(), out_cfg);

out_cfg:
  bus_release();
  EDP_RETURN_IF_ERROR(err);

  EDP_RETURN_ON_ERROR(wait_idle());

  _asleep = false;
  _partial_prepared = false;
  // RAM contents should be preserved; basemap validity is unchanged.
  _mode = Mode::Full;
  return ESP_OK;
}

esp_err_t GDEY0213B74::ensure_init_full() {
  if (!_asleep && _mode == Mode::Full) {
    return ESP_OK;
  }
  return init_full();
}

esp_err_t GDEY0213B74::init_fast() {
  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(init());

  EDP_RETURN_ON_ERROR(reset());

  EDP_RETURN_ON_ERROR(bus_acquire());
  err = write_cmd(0x12);
  bus_release();
  EDP_RETURN_IF_ERROR(err);

  EDP_RETURN_ON_ERROR(wait_idle());

  EDP_RETURN_ON_ERROR(bus_acquire());

  EDP_GOTO_ON_ERROR(write_cmd(0x18), out_p1);
  EDP_GOTO_ON_ERROR(write_data_u8(0x80), out_p1);
  EDP_GOTO_ON_ERROR(write_cmd(0x22), out_p1);
  EDP_GOTO_ON_ERROR(write_data_u8(0xB1), out_p1);
  EDP_GOTO_ON_ERROR(write_cmd(0x20), out_p1);

out_p1:
  bus_release();
  EDP_RETURN_IF_ERROR(err);

  EDP_RETURN_ON_ERROR(wait_idle());

  EDP_RETURN_ON_ERROR(bus_acquire());

  EDP_GOTO_ON_ERROR(write_cmd(0x1A), out_p2);
  EDP_GOTO_ON_ERROR(write_data_u8(0x64), out_p2);
  EDP_GOTO_ON_ERROR(write_data_u8(0x00), out_p2);
  EDP_GOTO_ON_ERROR(write_cmd(0x22), out_p2);
  EDP_GOTO_ON_ERROR(write_data_u8(0x91), out_p2);
  EDP_GOTO_ON_ERROR(write_cmd(0x20), out_p2);

out_p2:
  bus_release();
  EDP_RETURN_IF_ERROR(err);

  EDP_RETURN_ON_ERROR(wait_idle());

  _asleep = false;
  _partial_prepared = false;
  // RAM contents should be preserved; basemap validity is unchanged.
  _mode = Mode::Fast;
  return ESP_OK;
}

esp_err_t GDEY0213B74::ensure_init_fast() {
  if (!_asleep && _mode == Mode::Fast) {
    return ESP_OK;
  }
  return init_fast();
}

esp_err_t GDEY0213B74::display_frame_bw(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len != BUFFER_SIZE) {
    return ESP_ERR_INVALID_ARG;
  }

  EDP_RETURN_ON_ERROR(ensure_init_full());

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_prep_full_tx(), out);
  EDP_GOTO_ON_ERROR(_write_ram_maybe_mirror_x(0x24, buf, WIDTH, HEIGHT, len), out);
  EDP_GOTO_ON_ERROR(_trigger_update_full_tx(), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);
  _basemap_valid = false;
  return wait_idle();
}

esp_err_t GDEY0213B74::display_frame_bw_fast(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len != BUFFER_SIZE) {
    return ESP_ERR_INVALID_ARG;
  }

  EDP_RETURN_ON_ERROR(ensure_init_fast());

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_prep_full_tx(), out);
  EDP_GOTO_ON_ERROR(_write_ram_maybe_mirror_x(0x24, buf, WIDTH, HEIGHT, len), out);
  EDP_GOTO_ON_ERROR(_trigger_update_fast_tx(), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);
  _basemap_valid = false;
  return wait_idle();
}

esp_err_t GDEY0213B74::clear_white() {
  static constexpr uint8_t WHITE = 0xFF;

  EDP_RETURN_ON_ERROR(ensure_init_full());

  uint8_t chunk[64];
  for (size_t i = 0; i < sizeof(chunk); i++) {
    chunk[i] = WHITE;
  }

  size_t remaining = BUFFER_SIZE;

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_prep_full_tx(), out);
  EDP_GOTO_ON_ERROR(write_cmd(0x24), out);

  while (remaining > 0) {
    const size_t n = (remaining > sizeof(chunk)) ? sizeof(chunk) : remaining;
    EDP_GOTO_ON_ERROR(write_data(chunk, n), out);
    remaining -= n;
  }

  EDP_GOTO_ON_ERROR(_trigger_update_full_tx(), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);
  _basemap_valid = false;
  return wait_idle();
}

esp_err_t GDEY0213B74::clear_black() {
  static constexpr uint8_t BLACK = 0x00;

  EDP_RETURN_ON_ERROR(ensure_init_full());

  uint8_t chunk[64];
  for (size_t i = 0; i < sizeof(chunk); i++) {
    chunk[i] = BLACK;
  }

  size_t remaining = BUFFER_SIZE;

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_prep_full_tx(), out);
  EDP_GOTO_ON_ERROR(write_cmd(0x24), out);

  while (remaining > 0) {
    const size_t n = (remaining > sizeof(chunk)) ? sizeof(chunk) : remaining;
    EDP_GOTO_ON_ERROR(write_data(chunk, n), out);
    remaining -= n;
  }

  EDP_GOTO_ON_ERROR(_trigger_update_full_tx(), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);
  _basemap_valid = false;
  return wait_idle();
}

esp_err_t GDEY0213B74::set_basemap_bw(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len != BUFFER_SIZE) {
    return ESP_ERR_INVALID_ARG;
  }

  EDP_RETURN_ON_ERROR(ensure_init_full());

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_prep_full_tx(), out);
  EDP_GOTO_ON_ERROR(_write_ram_maybe_mirror_x(0x24, buf, WIDTH, HEIGHT, len), out);
  EDP_GOTO_ON_ERROR(_write_ram_maybe_mirror_x(0x26, buf, WIDTH, HEIGHT, len), out);
  EDP_GOTO_ON_ERROR(_trigger_update_full_tx(), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);
  _basemap_valid = true;
  return wait_idle();
}

esp_err_t GDEY0213B74::set_basemap_bw_fast(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len != BUFFER_SIZE) {
    return ESP_ERR_INVALID_ARG;
  }

  EDP_RETURN_ON_ERROR(ensure_init_fast());

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_prep_full_tx(), out);
  EDP_GOTO_ON_ERROR(_write_ram_maybe_mirror_x(0x24, buf, WIDTH, HEIGHT, len), out);
  EDP_GOTO_ON_ERROR(_write_ram_maybe_mirror_x(0x26, buf, WIDTH, HEIGHT, len), out);
  EDP_GOTO_ON_ERROR(_trigger_update_fast_tx(), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);
  _basemap_valid = true;
  return wait_idle();
}

esp_err_t GDEY0213B74::display_partial_bw(int x, int y, int w, int h, const uint8_t* buf, size_t len) {
  if (buf == nullptr || w <= 0 || h <= 0 || (w % 8) != 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (x < 0 || y < 0 || (x + w) > WIDTH || (y + h) > HEIGHT) {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t expected = (size_t)(w * h) / 8;
  if (len != expected) {
    return ESP_ERR_INVALID_ARG;
  }

  EDP_RETURN_ON_ERROR(partial_begin());
  EDP_RETURN_ON_ERROR(partial_write_bw(x, y, w, h, buf, len));
  return partial_end_update();
}

esp_err_t GDEY0213B74::partial_begin() {
  if (!_basemap_valid) {
    return ESP_ERR_INVALID_STATE;
  }
  if (_partial_prepared) {
    return ESP_ERR_INVALID_STATE;
  }

  // Ensure we are in full init mode first.
  if (_asleep) {
    EDP_RETURN_ON_ERROR(init_full());
  } else if (_mode != Mode::Full) {
    EDP_RETURN_ON_ERROR(init_full());
  }

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(write_cmd(0x3C), out);
  EDP_GOTO_ON_ERROR(write_data_u8(0x80), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);

  _partial_prepared = true;
  _mode = Mode::Full;
  return ESP_OK;
}

esp_err_t GDEY0213B74::partial_write_bw(int x, int y, int w, int h, const uint8_t* buf, size_t len) {
  if (buf == nullptr || w <= 0 || h <= 0 || (w % 8) != 0 || (x % 8) != 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (x < 0 || y < 0 || (x + w) > WIDTH || (y + h) > HEIGHT) {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t expected = (size_t)(w * h) / 8;
  if (len != expected) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!_partial_prepared) {
    return ESP_ERR_INVALID_STATE;
  }

  int x_use = x;
  if (cfg().mirror_x) {
    x_use = WIDTH - w - x;
  }
  const uint8_t x_start_bytes = (uint8_t)(x_use / 8);
  const uint8_t x_end_bytes = (uint8_t)((x_use + w) / 8 - 1);

  // Match the full-init memory mapping: controller Y=HEIGHT-1 maps to screen Y=0.
  // With 0x11=0x01 (X++, Y--), writing row-major (top-to-bottom) works as expected
  // when we translate window/cursor Y into controller coordinates.
  const uint16_t y_end_disp = (uint16_t)(y + h - 1);
  const uint16_t y_start_ctrl = (uint16_t)(HEIGHT - 1 - y);
  const uint16_t y_end_ctrl = (uint16_t)(HEIGHT - 1 - y_end_disp);

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_set_window(x_start_bytes, x_end_bytes, y_start_ctrl, y_end_ctrl), out);
  EDP_GOTO_ON_ERROR(_set_cursor(x_start_bytes, y_start_ctrl), out);
  EDP_GOTO_ON_ERROR(_write_ram_maybe_mirror_x(0x24, buf, w, h, len), out);

out:
  bus_release();
  return err;
}

esp_err_t GDEY0213B74::partial_end_update() {
  if (!_partial_prepared) {
    return ESP_ERR_INVALID_STATE;
  }
  EDP_RETURN_ON_ERROR(init());
  if (_asleep) {
    EDP_RETURN_ON_ERROR(init_full());
  }

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(_trigger_update_partial_tx(), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);
  EDP_RETURN_ON_ERROR(wait_idle());
  _partial_prepared = false;
  _mode = Mode::Full;
  return ESP_OK;
}

esp_err_t GDEY0213B74::partial_update() {
  return partial_end_update();
}

esp_err_t GDEY0213B74::display_partial_bw_all(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len != BUFFER_SIZE) {
    return ESP_ERR_INVALID_ARG;
  }

  EDP_RETURN_ON_ERROR(init());
  if (_asleep) {
    EDP_RETURN_ON_ERROR(init_full());
  }

  EDP_RETURN_ON_ERROR(partial_begin());
  EDP_RETURN_ON_ERROR(partial_write_bw(0, 0, WIDTH, HEIGHT, buf, len));
  return partial_end_update();
}

esp_err_t GDEY0213B74::deep_sleep() {
  EDP_RETURN_ON_ERROR(init());

  esp_err_t err = ESP_OK;
  EDP_RETURN_ON_ERROR(bus_acquire());
  EDP_GOTO_ON_ERROR(write_cmd(0x10), out);
  EDP_GOTO_ON_ERROR(write_data_u8(0x01), out);

out:
  bus_release();
  EDP_RETURN_IF_ERROR(err);

  vTaskDelay(pdMS_TO_TICKS(100));
  _asleep = true;
  _partial_prepared = false;
  _basemap_valid = false;
  _mode = Mode::Unknown;
  return ESP_OK;
}

}  // namespace ssd1680x::panels
