/**
 * AirGradient
 * https://airgradient.com
 *
 * I2C-to-UART Bridge Implementation using WK2132 chip
 * Embeds WK2132 driver logic from DFRobot library
 *
 * Based on:
 * DFRobot_IICSerial library (MIT License)
 * Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "AirgradientIICSerial.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

AirgradientIICSerial::AirgradientIICSerial(i2c_master_bus_handle_t i2c_bus_handle,
                                           uint8_t subUartChannel, uint8_t IA1, uint8_t IA0,
                                           int iicResetIO)
    : _i2c_bus_handle(i2c_bus_handle), _subSerialChannel(subUartChannel), _rx_buffer_head(0),
      _rx_buffer_tail(0) {

  // Initialize I2C address from DIP switch settings
  _addr = (IA1 << 6) | (IA0 << 5) | WK2132_IIC_ADDR_FIXED;

  // Store reset pin
  if (iicResetIO != -1) {
    _iicResetIO = static_cast<gpio_num_t>(iicResetIO);
  } else {
    _iicResetIO = GPIO_NUM_NC;
  }

  // Clear RX buffer
  memset((void *)_rx_buffer, 0, sizeof(_rx_buffer));

  // Clear device handle map
  memset(_dev_handle_map, 0, sizeof(_dev_handle_map));
}

AirgradientIICSerial::~AirgradientIICSerial() {}

bool AirgradientIICSerial::begin(int baud) {
  if (isInitialized) {
    ESP_LOGW(TAG, "IICSerial already initialized");
    return true;
  }

  // Configure reset GPIO if provided
  if (_iicResetIO != GPIO_NUM_NC) {
    gpio_reset_pin(_iicResetIO);
    gpio_set_direction(_iicResetIO, GPIO_MODE_OUTPUT);
    gpio_set_level(_iicResetIO, 1); // Release reset
  }

  // Retry initialization
  int counter = 0;
  bool opened = false;
  do {
    if (wk2132Begin(baud) == WK2132_ERR_OK) {
      opened = true;
      break;
    }

    ESP_LOGW(TAG, "IICSerial failed to open serial line, retry...");
    counter++;
    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  } while (counter < MAX_RETRY_INIT);

  if (!opened) {
    ESP_LOGE(TAG, "IICSerial failed to open serial line, giving up");
    return false;
  }

  ESP_LOGI(TAG, "IICSerial successfully opened serial line");
  isInitialized = true;
  return true;
}

void AirgradientIICSerial::end() {
  wk2132SubSerialGlobalRegEnable(_subSerialChannel, WK2132_GLOBAL_RESET);
  isInitialized = false;
}

int AirgradientIICSerial::available() {
  // Read FIFO count from hardware
  uint8_t val = 0;
  if (wk2132ReadReg(WK2132_REG_RFCNT, &val, 1) != 1) {
    ESP_LOGV(TAG, "Failed to read FIFO count");
    return 0;
  }

  int fifo_count = (int)val;

  // If FIFO count is 0 but FIFO status shows data available, count is 256
  if (fifo_count == 0) {
    WK2132_FsrReg_t fsr = wk2132ReadFIFOStateReg();
    if (fsr.rDat == 1) {
      fifo_count = 256;
    }
  }

  // Add software buffer count
  int buffer_count =
      (unsigned int)(WK2132_SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail) %
      WK2132_SERIAL_RX_BUFFER_SIZE;

  return fifo_count + buffer_count;
}

void AirgradientIICSerial::print(const char *str) {
  if (isDebug) {
    // Prevent carriage return to stdout
    if (strcmp(str, "\r\n") == 0) {
      printf("\n");
    } else {
      printf("%s", str);
    }
  }

  if (str != nullptr) {
    write((const uint8_t *)str, strlen(str));
  }
}

int AirgradientIICSerial::write(const uint8_t *data, int len) {
  if (data == nullptr || len <= 0) {
    return 0;
  }

  // Check if TX FIFO is full
  WK2132_FsrReg_t fsr = wk2132ReadFIFOStateReg();
  if (fsr.tFull == 1) {
    ESP_LOGV(TAG, "TX FIFO full");
    return 0;
  }

  // Write to FIFO
  wk2132WriteFIFO((void *)data, len);
  return len;
}

int AirgradientIICSerial::read() {
  // Lazy-load data from hardware FIFO to software buffer
  int fifo_bytes =
      available() -
      ((unsigned int)(WK2132_SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail) %
       WK2132_SERIAL_RX_BUFFER_SIZE);

  for (int i = 0; i < fifo_bytes; i++) {
    wk2132_rx_buffer_index_t next_head =
        (wk2132_rx_buffer_index_t)(_rx_buffer_head + 1) % WK2132_SERIAL_RX_BUFFER_SIZE;

    if (next_head != _rx_buffer_tail) {
      // Read one byte from hardware FIFO
      uint8_t val = 0;
      if (wk2132ReadReg(WK2132_REG_FDAT, &val, 1) == 1) {
        _rx_buffer[_rx_buffer_head] = val;
        _rx_buffer_head = next_head;
      }
    } else {
      // Software buffer full
      break;
    }
  }

  // Return data from software buffer
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  }

  unsigned char c = _rx_buffer[_rx_buffer_tail];
  _rx_buffer_tail = (wk2132_rx_buffer_index_t)(_rx_buffer_tail + 1) % WK2132_SERIAL_RX_BUFFER_SIZE;

  if (isDebug) {
    if (c != '\r') {
      printf("%c", c);
    }
  }

  return c;
}

// ========== WK2132 Core Functions ==========

int AirgradientIICSerial::wk2132Begin(unsigned long baud, uint8_t format,
                                      WK2132_CommunicationMode mode, WK2132_LineBreakOutput opt) {
  // Reset RX buffer
  _rx_buffer_head = _rx_buffer_tail;

  // Verify chip communication by reading GENA register
  uint8_t val = 0;
  uint8_t channel = wk2132SubSerialChnnlSwitch(WK2132_SUBUART_CHANNEL_1);
  if (wk2132ReadReg(WK2132_REG_GENA, &val, 1) != 1) {
    ESP_LOGV(TAG, "Failed to read GENA register");
    wk2132SubSerialChnnlSwitch(channel);
    return WK2132_ERR_READ;
  }

#ifndef ARDUINO_ARCH_NRF5
  // Verify GENA has valid value (bit 7 should be set)
  if ((val & 0x80) == 0) {
    ESP_LOGV(TAG, "GENA register validation failed");
    wk2132SubSerialChnnlSwitch(channel);
    return WK2132_ERR_REGDATA;
  }
#endif

  wk2132SubSerialChnnlSwitch(channel);

  // Configure sub UART
  wk2132SubSerialConfig(_subSerialChannel);

  // Set baud rate
  wk2132SetSubSerialBaudRate(baud);

  // Set data format and mode
  wk2132SetSubSerialConfigReg(format, mode, opt);

  return WK2132_ERR_OK;
}

void AirgradientIICSerial::wk2132SubSerialConfig(uint8_t subUartChannel) {
  ESP_LOGV(TAG, "Configuring sub UART channel %d", subUartChannel);

  // Enable sub UART clock
  ESP_LOGV(TAG, "Enable sub UART clock");
  wk2132SubSerialGlobalRegEnable(subUartChannel, WK2132_GLOBAL_CLOCK);

  // Software reset sub UART
  ESP_LOGV(TAG, "Software reset sub UART");
  wk2132SubSerialGlobalRegEnable(subUartChannel, WK2132_GLOBAL_RESET);

  // Enable global interrupt
  ESP_LOGV(TAG, "Enable global interrupt");
  wk2132SubSerialGlobalRegEnable(subUartChannel, WK2132_GLOBAL_INTERRUPT);

  // Switch to page 0
  ESP_LOGV(TAG, "Switch to page 0");
  wk2132SubSerialPageSwitch(WK2132_PAGE0);

  // Configure interrupt enable register
  ESP_LOGV(TAG, "Configure interrupt enable");
  WK2132_SierReg_t sier = {
      .rFTrig = 0x01, .rxOvt = 0x01, .tfTrig = 0x01, .tFEmpty = 0x01, .rsv = 0x00, .fErr = 0x01};
  wk2132SubSerialRegConfig(WK2132_REG_SIER, &sier);

  // Enable and reset TX/RX FIFOs
  ESP_LOGV(TAG, "Configure FIFO");
  WK2132_FcrReg_t fcr = {
      .rfRst = 0x01,  // Reset RX FIFO
      .tfRst = 0x00,  // Don't reset TX FIFO
      .rfEn = 0x01,   // Enable RX FIFO
      .tfEn = 0x01,   // Enable TX FIFO
      .rfTrig = 0x00, // RX trigger: 8 bytes
      .tfTrig = 0x00  // TX trigger: 8 bytes
  };
  wk2132SubSerialRegConfig(WK2132_REG_FCR, &fcr);

  // Enable RX and TX
  ESP_LOGV(TAG, "Enable RX/TX");
  WK2132_ScrReg_t scr = {.rxEn = 0x01, .txEn = 0x01, .sleepEn = 0x01, .rsv = 0x00};
  wk2132SubSerialRegConfig(WK2132_REG_SCR, &scr);
}

void AirgradientIICSerial::wk2132SubSerialGlobalRegEnable(uint8_t subUartChannel,
                                                          WK2132_GlobalRegType type) {
  if (subUartChannel > WK2132_SUBUART_CHANNEL_ALL) {
    ESP_LOGV(TAG, "Invalid sub UART channel");
    return;
  }

  uint8_t val = 0;
  uint8_t regAddr = wk2132GetGlobalRegType(type);
  uint8_t channel = wk2132SubSerialChnnlSwitch(WK2132_SUBUART_CHANNEL_1);

  ESP_LOGV(TAG, "Global reg enable: reg=0x%02x", regAddr);

  if (wk2132ReadReg(regAddr, &val, 1) != 1) {
    ESP_LOGV(TAG, "Failed to read global register");
    wk2132SubSerialChnnlSwitch(channel);
    return;
  }

  ESP_LOGV(TAG, "Before: 0x%02x", val);

  // Set appropriate bit for channel
  switch (subUartChannel) {
  case WK2132_SUBUART_CHANNEL_1:
    val |= 0x01;
    break;
  case WK2132_SUBUART_CHANNEL_2:
    val |= 0x02;
    break;
  default: // Both channels
    val |= 0x03;
    break;
  }

  wk2132WriteReg(regAddr, &val, 1);
  wk2132ReadReg(regAddr, &val, 1);
  ESP_LOGV(TAG, "After: 0x%02x", val);

  wk2132SubSerialChnnlSwitch(channel);
}

void AirgradientIICSerial::wk2132SetSubSerialBaudRate(unsigned long baud) {
  // Disable SCR temporarily
  uint8_t scr = 0x00, clear = 0x00;
  wk2132ReadReg(WK2132_REG_SCR, &scr, 1);
  wk2132SubSerialRegConfig(WK2132_REG_SCR, &clear);

  // Calculate baud rate registers
  // Formula: baudrate = FOSC / (16 * (baud1:baud0 + 1) + pres/16)
  uint16_t valInteger = WK2132_FOSC / (baud * 16) - 1;
  uint16_t valDecimal = (WK2132_FOSC % (baud * 16)) / (baud * 16);

  uint8_t baud1 = (uint8_t)(valInteger >> 8);
  uint8_t baud0 = (uint8_t)(valInteger & 0x00FF);

  // Normalize decimal part
  while (valDecimal > 0x0A) {
    valDecimal /= 0x0A;
  }
  uint8_t baudPres = (uint8_t)(valDecimal);

  // Switch to page 1 to access baud rate registers
  wk2132SubSerialPageSwitch(WK2132_PAGE1);
  wk2132SubSerialRegConfig(WK2132_REG_BAUD1, &baud1);
  wk2132SubSerialRegConfig(WK2132_REG_BAUD0, &baud0);
  wk2132SubSerialRegConfig(WK2132_REG_PRES, &baudPres);

  ESP_LOGV(TAG, "Baud config: baud1=0x%02x, baud0=0x%02x, pres=0x%02x", baud1, baud0, baudPres);

  // Switch back to page 0
  wk2132SubSerialPageSwitch(WK2132_PAGE0);

  // Restore SCR
  wk2132SubSerialRegConfig(WK2132_REG_SCR, &scr);
}

void AirgradientIICSerial::wk2132SetSubSerialConfigReg(uint8_t format,
                                                       WK2132_CommunicationMode mode,
                                                       WK2132_LineBreakOutput opt) {
  uint8_t val = 0;
  _addr = wk2132UpdateAddr(_addr, _subSerialChannel, WK2132_OBJECT_REGISTER);

  if (wk2132ReadReg(WK2132_REG_LCR, &val, 1) != 1) {
    ESP_LOGV(TAG, "Failed to read LCR");
    return;
  }

  ESP_LOGV(TAG, "LCR before: 0x%02x", val);

  WK2132_LcrReg_t lcr = *((WK2132_LcrReg_t *)(&val));
  lcr.format = format;
  lcr.irEn = mode;
  lcr.lBreak = opt;
  val = *(uint8_t *)&lcr;

  wk2132WriteReg(WK2132_REG_LCR, &val, 1);
  wk2132ReadReg(WK2132_REG_LCR, &val, 1);
  ESP_LOGV(TAG, "LCR after: 0x%02x", val);
}

void AirgradientIICSerial::wk2132SubSerialPageSwitch(WK2132_PageNumber page) {
  if (page >= WK2132_PAGE_TOTAL) {
    return;
  }

  uint8_t val = 0;
  if (wk2132ReadReg(WK2132_REG_SPAGE, &val, 1) != 1) {
    ESP_LOGV(TAG, "Failed to read SPAGE");
    return;
  }

  switch (page) {
  case WK2132_PAGE0:
    val &= 0xFE;
    break;
  case WK2132_PAGE1:
    val |= 0x01;
    break;
  default:
    break;
  }

  ESP_LOGV(TAG, "SPAGE before: 0x%02x", val);
  wk2132WriteReg(WK2132_REG_SPAGE, &val, 1);
  wk2132ReadReg(WK2132_REG_SPAGE, &val, 1);
  ESP_LOGV(TAG, "SPAGE after: 0x%02x", val);
}

void AirgradientIICSerial::wk2132SubSerialRegConfig(uint8_t reg, void *pValue) {
  uint8_t val = 0;
  wk2132ReadReg(reg, &val, 1);
  ESP_LOGV(TAG, "Reg 0x%02x before: 0x%02x", reg, val);

  val |= *(uint8_t *)pValue;
  wk2132WriteReg(reg, &val, 1);

  wk2132ReadReg(reg, &val, 1);
  ESP_LOGV(TAG, "Reg 0x%02x after: 0x%02x", reg, val);
}

uint8_t AirgradientIICSerial::wk2132GetGlobalRegType(WK2132_GlobalRegType type) {
  switch (type) {
  case WK2132_GLOBAL_CLOCK:
    return WK2132_REG_GENA;
  case WK2132_GLOBAL_RESET:
    return WK2132_REG_GRST;
  case WK2132_GLOBAL_INTERRUPT:
    return WK2132_REG_GIER;
  default:
    ESP_LOGV(TAG, "Invalid global register type");
    return 0;
  }
}

uint8_t AirgradientIICSerial::wk2132SubSerialChnnlSwitch(uint8_t subUartChannel) {
  uint8_t prev_channel = _subSerialChannel;
  _subSerialChannel = subUartChannel;
  return prev_channel;
}

WK2132_FsrReg_t AirgradientIICSerial::wk2132ReadFIFOStateReg() {
  WK2132_FsrReg_t fsr;
  wk2132ReadReg(WK2132_REG_FSR, &fsr, sizeof(fsr));
  return fsr;
}

uint8_t AirgradientIICSerial::wk2132UpdateAddr(uint8_t pre, uint8_t subUartChannel, uint8_t obj) {
  WK2132_IICAddr_t addr = {
      .type = obj, .uart = subUartChannel, .addrPre = (uint8_t)((int)pre >> 3)};
  return *(uint8_t *)&addr;
}

// ========== I2C Communication Functions ==========

void AirgradientIICSerial::wk2132WriteReg(uint8_t reg, const void *pBuf, size_t size) {
  if (pBuf == nullptr) {
    ESP_LOGV(TAG, "Write register: null pointer");
    return;
  }

  _addr = wk2132UpdateAddr(_addr, _subSerialChannel, WK2132_OBJECT_REGISTER);
  i2c_master_dev_handle_t dev_handle = wk2132GetDeviceHandle(_addr);
  if (!dev_handle) {
    ESP_LOGV(TAG, "Failed to get device handle");
    return;
  }

  const uint8_t *data = (const uint8_t *)pBuf;

  // Compile transmit payload: [register_address][data...]
  uint8_t *txBuf = new uint8_t[size + 1];
  txBuf[0] = reg;
  memcpy(&txBuf[1], data, size);

  // Transmit in single I2C transaction
  esp_err_t err = i2c_master_transmit(dev_handle, txBuf, (size + 1), 500);
  if (err != ESP_OK) {
    ESP_LOGV(TAG, "Register write failed");
  }

  delete[] txBuf;
}

uint8_t AirgradientIICSerial::wk2132ReadReg(uint8_t reg, void *pBuf, size_t size) {
  if (pBuf == nullptr) {
    ESP_LOGV(TAG, "Read register: null pointer");
    return 0;
  }

  _addr = wk2132UpdateAddr(_addr, _subSerialChannel, WK2132_OBJECT_REGISTER);
  _addr &= 0xFE; // Ensure read address
  i2c_master_dev_handle_t dev_handle = wk2132GetDeviceHandle(_addr);
  if (!dev_handle) {
    ESP_LOGV(TAG, "Failed to get device handle");
    return 0;
  }

  // Step 1: Write register address
  esp_err_t err = i2c_master_transmit(dev_handle, &reg, 1, 500);
  if (err != ESP_OK) {
    ESP_LOGV(TAG, "Failed to send register address 0x%02x", reg);
    return 0;
  }

  // Step 2: Read response
  err = i2c_master_receive(dev_handle, (uint8_t *)pBuf, size, 500);
  if (err != ESP_OK) {
    ESP_LOGV(TAG, "Failed to read from device 0x%02x", _addr);
    return 0;
  }

  return size;
}

void AirgradientIICSerial::wk2132WriteFIFO(void *pBuf, size_t size) {
  if (pBuf == nullptr) {
    ESP_LOGV(TAG, "Write FIFO: null pointer");
    return;
  }

  _addr = wk2132UpdateAddr(_addr, _subSerialChannel, WK2132_OBJECT_FIFO);
  i2c_master_dev_handle_t dev_handle = wk2132GetDeviceHandle(_addr);
  if (!dev_handle) {
    return;
  }

  const uint8_t *buf = (const uint8_t *)pBuf;
  size_t left = size;
  size_t chunk = 0;

  // Chunk data into IIC_BUFFER_SIZE segments
  while (left > 0) {
    chunk = (left > WK2132_IIC_BUFFER_SIZE) ? WK2132_IIC_BUFFER_SIZE : left;

    esp_err_t err = i2c_master_transmit(dev_handle, buf, chunk, 500);
    if (err != ESP_OK) {
      ESP_LOGV(TAG, "FIFO write failed to 0x%02x", _addr);
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    buf += chunk;
    left -= chunk;
  }
}

uint8_t AirgradientIICSerial::wk2132ReadFIFO(void *pBuf, size_t size) {
  if (pBuf == nullptr) {
    ESP_LOGV(TAG, "Read FIFO: null pointer");
    return 0;
  }

  _addr = wk2132UpdateAddr(_addr, _subSerialChannel, WK2132_OBJECT_FIFO);
  i2c_master_dev_handle_t dev_handle = wk2132GetDeviceHandle(_addr);
  if (!dev_handle) {
    return 0;
  }

  uint8_t *buf = (uint8_t *)pBuf;
  size_t left = size;
  size_t num = 0;

  // Chunk read into IIC_BUFFER_SIZE segments
  while (left > 0) {
    num = (left > WK2132_IIC_BUFFER_SIZE) ? WK2132_IIC_BUFFER_SIZE : left;

    // Dummy write to initiate FIFO read
    esp_err_t err = i2c_master_transmit(dev_handle, nullptr, 0, 500);
    if (err != ESP_OK) {
      ESP_LOGV(TAG, "Failed to initiate FIFO read from 0x%02x", _addr);
      return 0;
    }

    // Read num bytes
    err = i2c_master_receive(dev_handle, buf, num, 500);
    if (err != ESP_OK) {
      ESP_LOGV(TAG, "Failed to read FIFO data from 0x%02x", _addr);
      return 0;
    }

    buf += num;
    left -= num;
  }

  return size;
}

i2c_master_dev_handle_t AirgradientIICSerial::wk2132GetDeviceHandle(uint8_t addr) {
  // Check if device handle already exists
  for (int i = 0; i < WK2132_MAX_ADDR; i++) {
    if (_dev_handle_map[i].address == addr) {
      return _dev_handle_map[i].dev_handle;
    }
  }

  // Create new device handle
  for (int i = 0; i < WK2132_MAX_ADDR; i++) {
    if (_dev_handle_map[i].address == 0) { // Empty slot
      i2c_device_config_t dev_cfg = {
          .dev_addr_length = I2C_ADDR_BIT_LEN_7,
          .device_address = addr,
          .scl_speed_hz = 100000,
      };

      _dev_handle_map[i].address = addr;

      esp_err_t ret =
          i2c_master_bus_add_device(_i2c_bus_handle, &dev_cfg, &_dev_handle_map[i].dev_handle);
      if (ret != ESP_OK) {
        ESP_LOGV(TAG, "Failed to add I2C device at address 0x%02x", addr);
        _dev_handle_map[i].address = 0; // Clear slot
        return nullptr;
      }

      ESP_LOGV(TAG, "Created I2C device handle for address 0x%02x", addr);
      return _dev_handle_map[i].dev_handle;
    }
  }

  ESP_LOGV(TAG, "No space for new I2C device at address 0x%02x", addr);
  return nullptr;
}
