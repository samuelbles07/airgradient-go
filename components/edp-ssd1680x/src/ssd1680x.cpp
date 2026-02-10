#include "ssd1680x.h"
#include "edp_err_macros.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ssd1680x {

static uint32_t _clamp_poll_delay_ms(uint32_t v) {
  if (v == 0) {
    return 1;
  }
  return v;
}

Device::Device(const Config& cfg) : _cfg(cfg) {
  _cfg.busy_poll_delay_ms = _clamp_poll_delay_ms(_cfg.busy_poll_delay_ms);
}

Device::~Device() {
  if (_bus_acquired && spi_ != nullptr) {
    spi_device_release_bus(spi_);
    _bus_acquired = false;
  }
  if (spi_ != nullptr) {
    spi_bus_remove_device(spi_);
    spi_ = nullptr;
  }
}

esp_err_t Device::init() {
  if (_initialized) {
    return ESP_OK;
  }

  if (_cfg.pins.busy == GPIO_NUM_MAX || _cfg.pins.rst == GPIO_NUM_MAX || _cfg.pins.dc == GPIO_NUM_MAX ||
      _cfg.pins.cs == GPIO_NUM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  // GPIO config
  gpio_config_t io = {};

  io.intr_type = GPIO_INTR_DISABLE;
  io.mode = GPIO_MODE_INPUT;
  io.pin_bit_mask = (1ULL << _cfg.pins.busy);
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  EDP_RETURN_ON_ERROR(gpio_config(&io));

  io = {};
  io.intr_type = GPIO_INTR_DISABLE;
  io.mode = GPIO_MODE_OUTPUT;
  io.pin_bit_mask = (1ULL << _cfg.pins.rst) | (1ULL << _cfg.pins.dc);
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  EDP_RETURN_ON_ERROR(gpio_config(&io));

  // Default levels
  gpio_set_level(_cfg.pins.dc, 0);
  gpio_set_level(_cfg.pins.rst, 1);

  // SPI device
  spi_device_interface_config_t devcfg = _cfg.devcfg;
  devcfg.spics_io_num = _cfg.pins.cs;
  esp_err_t err = spi_bus_add_device(_cfg.host, &devcfg, &spi_);
  if (err != ESP_OK) {
    spi_ = nullptr;
    return err;
  }

  _initialized = true;
  return ESP_OK;
}

esp_err_t Device::reset() {
  if (!_initialized) {
    EDP_RETURN_ON_ERROR(init());
  }

  gpio_set_level(_cfg.pins.rst, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(_cfg.pins.rst, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
  return ESP_OK;
}

bool Device::is_busy() const {
  const int level = gpio_get_level(_cfg.pins.busy);
  return level == _cfg.busy_active_level;
}

esp_err_t Device::wait_idle(uint32_t timeout_ms) {
  if (!_initialized) {
    EDP_RETURN_ON_ERROR(init());
  }

  if (timeout_ms == 0) {
    timeout_ms = _cfg.busy_timeout_ms;
  }

  const int64_t start_us = esp_timer_get_time();
  while (is_busy()) {
    const int64_t now_us = esp_timer_get_time();
    if (timeout_ms > 0 && (now_us - start_us) >= (int64_t)timeout_ms * 1000) {
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(pdMS_TO_TICKS(_cfg.busy_poll_delay_ms));
  }
  return ESP_OK;
}

esp_err_t Device::write_cmd(uint8_t cmd) {
  if (!_initialized) {
    EDP_RETURN_ON_ERROR(init());
  }
  gpio_set_level(_cfg.pins.dc, 0);

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.flags = SPI_TRANS_USE_TXDATA;
  t.length = 8;
  t.tx_data[0] = cmd;
  return spi_device_transmit(spi_, &t);
}

esp_err_t Device::write_data_u8(uint8_t data) {
  return write_data(&data, 1);
}

esp_err_t Device::_spi_tx(const void* data, size_t len) {
  if (len == 0) {
    return ESP_OK;
  }

  // Chunk transfers to avoid relying on the bus max_transfer_sz.
  // Keep chunks reasonably small to be safe across targets.
  static constexpr size_t CHUNK = 1024;
  const uint8_t* p = static_cast<const uint8_t*>(data);
  size_t remaining = len;

  while (remaining > 0) {
    const size_t n = (remaining > CHUNK) ? CHUNK : remaining;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = n * 8;
    t.tx_buffer = p;
    esp_err_t err = spi_device_transmit(spi_, &t);
    if (err != ESP_OK) {
      return err;
    }
    p += n;
    remaining -= n;
  }
  return ESP_OK;
}

esp_err_t Device::write_data(const void* data, size_t len) {
  if (!_initialized) {
    EDP_RETURN_ON_ERROR(init());
  }
  gpio_set_level(_cfg.pins.dc, 1);
  return _spi_tx(data, len);
}

esp_err_t Device::bus_acquire() {
  if (!_initialized) {
    EDP_RETURN_ON_ERROR(init());
  }
  if (spi_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (_bus_acquired) {
    return ESP_ERR_INVALID_STATE;
  }

  TickType_t wait_ticks = portMAX_DELAY;
  if (_cfg.bus_acquire_timeout_ms != 0) {
    wait_ticks = pdMS_TO_TICKS(_cfg.bus_acquire_timeout_ms);
  }

  const esp_err_t err = spi_device_acquire_bus(spi_, wait_ticks);
  if (err != ESP_OK) {
    return err;
  }

  _bus_acquired = true;
  return ESP_OK;
}

void Device::bus_release() {
  if (!_bus_acquired) {
    return;
  }
  if (spi_ != nullptr) {
    spi_device_release_bus(spi_);
  }
  _bus_acquired = false;
}

}  // namespace ssd1680x
