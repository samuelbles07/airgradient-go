// SPDX-License-Identifier: MIT

#ifndef EDP_SSD1680X_SSD1680X_H
#define EDP_SSD1680X_SSD1680X_H

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

namespace ssd1680x {

struct Pins {
  gpio_num_t busy = GPIO_NUM_MAX;
  gpio_num_t rst = GPIO_NUM_MAX;
  gpio_num_t dc = GPIO_NUM_MAX;
  gpio_num_t cs = GPIO_NUM_MAX;
};

struct Config {
  spi_host_device_t host = SPI2_HOST;

  // The SPI bus (spi_bus_initialize) must be initialized by the caller.
  // This config is copied and then adjusted (e.g. spics_io_num) by the driver.
  spi_device_interface_config_t devcfg = {};

  Pins pins = {};

  // Busy pin behavior: many Good Display panels use BUSY=1 while busy, 0 when idle.
  int busy_active_level = 1;

  // Polling parameters (no infinite tight loops).
  uint32_t busy_poll_delay_ms = 2;
  uint32_t busy_timeout_ms = 8000;

  // How long to wait for exclusive access to the SPI bus.
  // 0 means wait forever.
  uint32_t bus_acquire_timeout_ms = 0;
};

class Device {
 public:
  explicit Device(const Config& cfg);
  ~Device();

  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;

  esp_err_t init();

 protected:
  esp_err_t reset();
  esp_err_t wait_idle(uint32_t timeout_ms = 0);
  bool is_busy() const;

  // Acquire/release exclusive access to the SPI bus for a burst of transfers.
  // Standardized usage: only call these from top-level (public) panel methods,
  // and never hold the bus lock while waiting on BUSY.
  esp_err_t bus_acquire();
  void bus_release();

  esp_err_t write_cmd(uint8_t cmd);
  esp_err_t write_data(const void* data, size_t len);
  esp_err_t write_data_u8(uint8_t data);

  const Config& cfg() const { return _cfg; }

 private:
  esp_err_t _spi_tx(const void* data, size_t len);

  Config _cfg;
  spi_device_handle_t spi_ = nullptr;
  bool _initialized = false;
  bool _bus_acquired = false;
};

}  // namespace ssd1680x

#endif
