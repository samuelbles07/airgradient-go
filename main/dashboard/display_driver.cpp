#include "dashboard/display_driver.h"

#include <stddef.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_rom_sys.h>

#include "dashboard/display_macros.h"

namespace dashboard::display_driver {
namespace {

constexpr char LOG_TAG[] = "display_driver";
constexpr int DEFAULT_SPI_CLOCK_HZ = 10 * 1000 * 1000;
constexpr size_t TX_BOUNCE_BYTES = 4096;

struct State {
  bool inited;
  bool bus_acquired;

  spi_host_device_t spi_host;
  int spi_clock_hz;
  uint32_t bus_acquire_timeout_ms;

  gpio_num_t pin_cs;
  gpio_num_t pin_dc;
  gpio_num_t pin_rst;
  gpio_num_t pin_busy;

  spi_device_handle_t spi;

  uint8_t *tx_bounce;
};

State state;

constexpr uint64_t gpio_bit(gpio_num_t pin) { return 1ULL << static_cast<uint64_t>(pin); }

inline void set_cs(int level) { gpio_set_level(state.pin_cs, level); }
inline void set_dc(int level) { gpio_set_level(state.pin_dc, level); }
inline void set_rst(int level) { gpio_set_level(state.pin_rst, level); }

void delay_ms(uint32_t ms) {
  if (ms == 0) {
    return;
  }

  const TickType_t ticks = pdMS_TO_TICKS(ms);
  if (ticks > 0) {
    vTaskDelay(ticks);
    return;
  }

  // Sub-tick delays.
  esp_rom_delay_us(ms * 1000U);
}

esp_err_t gpio_init(const Config &cfg) {
  gpio_config_t out_cfg = {};
  out_cfg.pin_bit_mask = gpio_bit(cfg.pin_cs) | gpio_bit(cfg.pin_dc) | gpio_bit(cfg.pin_rst);
  out_cfg.mode = GPIO_MODE_OUTPUT;
  out_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  out_cfg.intr_type = GPIO_INTR_DISABLE;
  esp_err_t err = gpio_config(&out_cfg);
  if (err != ESP_OK) {
    return err;
  }

  gpio_config_t in_cfg = {};
  in_cfg.pin_bit_mask = gpio_bit(cfg.pin_busy);
  in_cfg.mode = GPIO_MODE_INPUT;
  in_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  in_cfg.intr_type = GPIO_INTR_DISABLE;
  err = gpio_config(&in_cfg);
  if (err != ESP_OK) {
    return err;
  }

  set_cs(1);
  set_dc(1);
  set_rst(1);
  return ESP_OK;
}

esp_err_t spi_add_device(const Config &cfg) {
  if (state.spi != nullptr) {
    return ESP_OK;
  }

  spi_device_interface_config_t devcfg = {};
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  devcfg.dummy_bits = 0;
  devcfg.mode = 0;
  devcfg.clock_speed_hz = (cfg.spi_clock_hz > 0) ? cfg.spi_clock_hz : DEFAULT_SPI_CLOCK_HZ;
  devcfg.spics_io_num = -1; // Keep CS manual to match the reference implementation.
  devcfg.queue_size = 1;
  devcfg.flags = SPI_DEVICE_NO_DUMMY;
  devcfg.pre_cb = nullptr;
  devcfg.post_cb = nullptr;

  esp_err_t err = spi_bus_add_device(cfg.spi_host, &devcfg, &state.spi);
  if (err != ESP_OK) {
    return err;
  }

  if (state.tx_bounce == nullptr) {
    state.tx_bounce = static_cast<uint8_t *>(heap_caps_malloc(TX_BOUNCE_BYTES, MALLOC_CAP_DMA));
    if (state.tx_bounce == nullptr) {
      return ESP_ERR_NO_MEM;
    }
  }

  return ESP_OK;
}

esp_err_t spi_write_byte(uint8_t value) {
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_USE_TXDATA;
  t.length = 8;
  t.tx_data[0] = value;
  return spi_device_polling_transmit(state.spi, &t);
}

esp_err_t spi_write(const uint8_t *data, size_t len) {
  size_t offset = 0;
  while (offset < len) {
    const size_t chunk = (len - offset > TX_BOUNCE_BYTES) ? TX_BOUNCE_BYTES : (len - offset);

    memcpy(state.tx_bounce, data + offset, chunk);

    spi_transaction_t t = {};
    t.length = chunk * 8;
    t.tx_buffer = state.tx_bounce;
    const esp_err_t err = spi_device_polling_transmit(state.spi, &t);
    if (err != ESP_OK) {
      return err;
    }
    offset += chunk;
  }
  return ESP_OK;
}

esp_err_t write_cmd(uint8_t command) {
  set_cs(0);
  set_dc(0);
  const esp_err_t err = spi_write_byte(command);
  set_cs(1);
  return err;
}

esp_err_t write_data(uint8_t data) {
  set_cs(0);
  set_dc(1);
  const esp_err_t err = spi_write_byte(data);
  set_cs(1);
  return err;
}

esp_err_t write_data_bytes(const uint8_t *data, size_t len) {
  set_cs(0);
  set_dc(1);
  const esp_err_t err = spi_write(data, len);
  set_cs(1);
  return err;
}

// Busy: 1 = busy, 0 = idle/ready.
void wait_busy_low() {
  while (gpio_get_level(state.pin_busy) != 0) {
    // Yield to avoid task watchdog while waiting for the panel.
    vTaskDelay(1);
  }
}

esp_err_t update_full() {
  RETURN_ON_ERROR(write_cmd(0x22));
  RETURN_ON_ERROR(write_data(0xF7));
  RETURN_ON_ERROR(write_cmd(0x20));
  wait_busy_low();
  return ESP_OK;
}

esp_err_t update_partial() {
  RETURN_ON_ERROR(write_cmd(0x22));
  RETURN_ON_ERROR(write_data(0xFF));
  RETURN_ON_ERROR(write_cmd(0x20));
  wait_busy_low();
  return ESP_OK;
}

} // namespace

esp_err_t init(const Config &cfg) {
  if (state.inited) {
    const bool same_cfg = (state.spi_host == cfg.spi_host) &&
                          (state.spi_clock_hz == cfg.spi_clock_hz) &&
                          (state.bus_acquire_timeout_ms == cfg.bus_acquire_timeout_ms) &&
                          (state.pin_cs == cfg.pin_cs) && (state.pin_dc == cfg.pin_dc) &&
                          (state.pin_rst == cfg.pin_rst) && (state.pin_busy == cfg.pin_busy);
    return same_cfg ? ESP_OK : ESP_ERR_INVALID_STATE;
  }

  if ((static_cast<int>(cfg.pin_cs) < 0) || (static_cast<int>(cfg.pin_dc) < 0) ||
      (static_cast<int>(cfg.pin_rst) < 0) || (static_cast<int>(cfg.pin_busy) < 0)) {
    return ESP_ERR_INVALID_ARG;
  }

  state.spi_host = cfg.spi_host;
  state.spi_clock_hz = cfg.spi_clock_hz;
  state.bus_acquire_timeout_ms = cfg.bus_acquire_timeout_ms;
  state.pin_cs = cfg.pin_cs;
  state.pin_dc = cfg.pin_dc;
  state.pin_rst = cfg.pin_rst;
  state.pin_busy = cfg.pin_busy;

  esp_err_t err = gpio_init(cfg);
  if (err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "gpio init failed: %s", esp_err_to_name(err));
    return err;
  }

  err = spi_add_device(cfg);
  if (err != ESP_OK) {
    ESP_LOGE(LOG_TAG, "spi add device failed: %s", esp_err_to_name(err));
    return err;
  }

  delay_ms(100);

  state.bus_acquired = false;
  state.inited = true;
  ESP_LOGI(LOG_TAG, "EPD init done");
  return ESP_OK;
}

esp_err_t bus_acquire() {
  if (!state.inited || state.spi == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (state.bus_acquired) {
    return ESP_ERR_INVALID_STATE;
  }

  TickType_t wait_ticks = portMAX_DELAY;
  if (state.bus_acquire_timeout_ms != 0) {
    wait_ticks = pdMS_TO_TICKS(state.bus_acquire_timeout_ms);
  }

  const esp_err_t err = spi_device_acquire_bus(state.spi, wait_ticks);
  if (err != ESP_OK) {
    return err;
  }

  state.bus_acquired = true;
  return ESP_OK;
}

void bus_release() {
  if (!state.bus_acquired) {
    return;
  }
  if (state.spi != nullptr) {
    spi_device_release_bus(state.spi);
  }
  state.bus_acquired = false;
}

esp_err_t deep_sleep() {
  RETURN_ON_ERROR(write_cmd(0x10));
  RETURN_ON_ERROR(write_data(0x01));
  delay_ms(100);
  return ESP_OK;
}

esp_err_t hw_init_full() {
  set_rst(0);
  delay_ms(10);
  set_rst(1);
  delay_ms(10);

  wait_busy_low();
  esp_err_t err = write_cmd(0x12); // SWRESET
  RETURN_IF_ERROR(err);
  wait_busy_low();

  RETURN_ON_ERROR(write_cmd(0x01)); // Driver output control
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) % 256));
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) / 256));
  RETURN_ON_ERROR(write_data(0x00));

  RETURN_ON_ERROR(write_cmd(0x11)); // Data entry mode
  RETURN_ON_ERROR(write_data(0x01));

  RETURN_ON_ERROR(write_cmd(0x44)); // Set Ram-X address start/end
  RETURN_ON_ERROR(write_data(0x00));
  RETURN_ON_ERROR(write_data(WIDTH_PX / 8 - 1));

  RETURN_ON_ERROR(write_cmd(0x45)); // Set Ram-Y address start/end
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) % 256));
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) / 256));
  RETURN_ON_ERROR(write_data(0x00));
  RETURN_ON_ERROR(write_data(0x00));

  RETURN_ON_ERROR(write_cmd(0x3C)); // BorderWavefrom
  RETURN_ON_ERROR(write_data(0x05));

  RETURN_ON_ERROR(write_cmd(0x21)); // Display update control
  RETURN_ON_ERROR(write_data(0x00));
  RETURN_ON_ERROR(write_data(0x80));

  RETURN_ON_ERROR(write_cmd(0x18)); // Read built-in temperature sensor
  RETURN_ON_ERROR(write_data(0x80));

  RETURN_ON_ERROR(write_cmd(0x4E)); // Set RAM x address count to 0
  RETURN_ON_ERROR(write_data(0x00));
  RETURN_ON_ERROR(write_cmd(0x4F)); // Set RAM y address count to (HEIGHT-1)
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) % 256));
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) / 256));
  wait_busy_low();

  return ESP_OK;
}

esp_err_t set_ram_value_base_map(const uint8_t *data) {
  RETURN_ON_ERROR(write_cmd(0x24));
  RETURN_ON_ERROR(write_data_bytes(data, FRAME_BYTES));
  RETURN_ON_ERROR(write_cmd(0x26));
  RETURN_ON_ERROR(write_data_bytes(data, FRAME_BYTES));
  return update_full();
}

esp_err_t part_begin() {
  set_rst(0);
  delay_ms(10);
  set_rst(1);
  delay_ms(10);

  wait_busy_low();

  // Driver output control: match full init so Y mapping is stable after reset.
  RETURN_ON_ERROR(write_cmd(0x01));
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) % 256));
  RETURN_ON_ERROR(write_data((HEIGHT_PX - 1) / 256));
  RETURN_ON_ERROR(write_data(0x00));

  // Border waveform for partial refresh.
  RETURN_ON_ERROR(write_cmd(0x3C));
  RETURN_ON_ERROR(write_data(0x80));

  // Data entry mode: match full init.
  RETURN_ON_ERROR(write_cmd(0x11));
  RETURN_ON_ERROR(write_data(0x01));

  return ESP_OK;
}

esp_err_t part_write_region(unsigned int x_start_px, unsigned int y_start_px, const uint8_t *data,
                            unsigned int height_px, unsigned int width_px) {
  const unsigned int x_start = x_start_px / 8;
  const unsigned int x_end = x_start + width_px / 8 - 1;

  const unsigned int y_start_log = y_start_px;
  const unsigned int y_end_log = y_start_px + height_px - 1;

  // Map logical top-origin Y (0 = top) into controller Y coordinates used by hw_init_full.
  // In hw_init_full we set the full-screen Y window as (HEIGHT_PX-1) .. 0 and the RAM Y pointer
  // to (HEIGHT_PX-1), so the controller walks Y downward.
  const unsigned int y_start = (HEIGHT_PX - 1) - y_start_log;
  const unsigned int y_end = (HEIGHT_PX - 1) - y_end_log;

  RETURN_ON_ERROR(write_cmd(0x44));
  RETURN_ON_ERROR(write_data((uint8_t)x_start));
  RETURN_ON_ERROR(write_data((uint8_t)x_end));

  RETURN_ON_ERROR(write_cmd(0x45));
  RETURN_ON_ERROR(write_data((uint8_t)(y_start % 256)));
  RETURN_ON_ERROR(write_data((uint8_t)(y_start / 256)));
  RETURN_ON_ERROR(write_data((uint8_t)(y_end % 256)));
  RETURN_ON_ERROR(write_data((uint8_t)(y_end / 256)));

  RETURN_ON_ERROR(write_cmd(0x4E));
  RETURN_ON_ERROR(write_data((uint8_t)x_start));
  RETURN_ON_ERROR(write_cmd(0x4F));
  RETURN_ON_ERROR(write_data((uint8_t)(y_start % 256)));
  RETURN_ON_ERROR(write_data((uint8_t)(y_start / 256)));

  RETURN_ON_ERROR(write_cmd(0x24));
  return write_data_bytes(data, static_cast<size_t>(height_px) * width_px / 8);
}

esp_err_t part_commit() { return update_partial(); }

} // namespace dashboard::display_driver
