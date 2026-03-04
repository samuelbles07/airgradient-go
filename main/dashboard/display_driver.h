#pragma once

#include <stdint.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>

namespace dashboard::display_driver {

// Panel: Good Display GDEY0213B74 (2.13" b/w)
inline constexpr int WIDTH_PX = 128;
inline constexpr int HEIGHT_PX = 250;
inline constexpr int FRAME_BYTES = (WIDTH_PX * HEIGHT_PX) / 8;

struct Config {
  spi_host_device_t spi_host;
  int spi_clock_hz;

  uint32_t bus_acquire_timeout_ms;

  gpio_num_t pin_cs;
  gpio_num_t pin_dc;
  gpio_num_t pin_rst;
  gpio_num_t pin_busy;
};

// Initializes GPIOs + attaches a SPI device for the EPD.
// Requires the SPI bus for cfg.spi_host to be initialized by the caller.
// Note: this driver uses a DMA-capable bounce buffer for SPI transfers.
// SPI bus DMA selection is owned by the caller when initializing the SPI host.
esp_err_t init(const Config &cfg);

// Optionally acquire/release the SPI bus for exclusive access.
// If bus_acquire_timeout_ms is 0, waits indefinitely.
esp_err_t bus_acquire();
void bus_release();

// Optional: puts the EPD into deep sleep.
esp_err_t deep_sleep();

// Full screen refresh initialization.
esp_err_t hw_init_full();

// Writes one full frame into both controller planes and performs a full update.
// This matches the vendor "base map" requirement for stable partial updates.
esp_err_t set_ram_value_base_map(const uint8_t *data);

// Partial update batching: write N regions, then commit once.
esp_err_t part_begin();
esp_err_t part_write_region(unsigned int x_start_px, unsigned int y_start_px, const uint8_t *data,
                            unsigned int height_px, unsigned int width_px);
esp_err_t part_commit();

} // namespace dashboard::display_driver
