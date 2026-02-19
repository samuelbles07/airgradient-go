/**
 * AirGradient
 * https://airgradient.com
 *
 * I2C-to-UART Bridge Implementation using WK2132 chip
 * Embeds WK2132 driver logic without external dependencies
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef AIRGRADIENT_IICSERIAL_HPP
#define AIRGRADIENT_IICSERIAL_HPP

#include "AirgradientSerial.hpp"
#include "WK2132Registers.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

/**
 * I2C-to-UART bridge implementation using WK2132 chip.
 * Provides dual-channel UART over I2C interface.
 *
 * Features:
 * - Dual UART channels (channel 1 and 2)
 * - 256-byte hardware FIFO per channel
 * - 32-byte internal RX software buffer
 * - Configurable baud rates (2400-921600)
 * - Hardware reset support via GPIO pin
 * - Retry logic for initialization
 */
class AirgradientIICSerial : public AirgradientSerial {
public:
  /**
   * Constructor for I2C-UART bridge.
   *
   * @param i2c_bus_handle ESP-IDF I2C bus handle
   * @param subUartChannel Sub UART channel (WK2132_SUBUART_CHANNEL_1 or WK2132_SUBUART_CHANNEL_2)
   * @param IA1 DIP switch IA1 setting (0 or 1)
   * @param IA0 DIP switch IA0 setting (0 or 1)
   * @param iicResetIO GPIO pin number for WK2132 reset (-1 for no reset pin)
   */
  AirgradientIICSerial(i2c_master_bus_handle_t i2c_bus_handle,
                       uint8_t subUartChannel = WK2132_SUBUART_CHANNEL_1, uint8_t IA1 = 1,
                       uint8_t IA0 = 1, int iicResetIO = -1);
  ~AirgradientIICSerial();

  /**
   * Initialize I2C-UART bridge with baud rate.
   * Reset pin is configured via constructor.
   *
   * @param baud Baud rate (2400-921600)
   * @return true if successful, false otherwise
   */
  bool begin(int baud) override;

  /**
   * Deinitialize I2C-UART bridge.
   */
  void end() override;

  /**
   * Get number of bytes available to read.
   * Includes both hardware FIFO (256B) and software buffer (32B).
   *
   * @return Number of bytes available
   */
  int available() override;

  /**
   * Print null-terminated string.
   *
   * @param str String to print
   */
  void print(const char *str) override;

  /**
   * Write binary data.
   *
   * @param data Pointer to data buffer
   * @param len Length of data in bytes
   * @return Number of bytes written
   */
  int write(const uint8_t *data, int len) override;

  /**
   * Read single byte.
   * Data is lazily loaded from hardware FIFO to software buffer.
   *
   * @return Byte value (0-255), or -1 if no data available
   */
  int read() override;

private:
  // Initialization constants
  static constexpr int MAX_RETRY_INIT = 3;
  static constexpr int RETRY_DELAY_MS = 500;

  // Device configuration
  i2c_master_bus_handle_t _i2c_bus_handle;
  uint8_t _addr;             // Base I2C address
  uint8_t _subSerialChannel; // Current channel
  gpio_num_t _iicResetIO;    // Reset GPIO pin

  // Device handle cache for different I2C addresses
  struct DeviceInfo {
    uint8_t address;
    i2c_master_dev_handle_t dev_handle;
  };
  DeviceInfo _dev_handle_map[WK2132_MAX_ADDR];

  // RX software buffer (circular buffer)
  volatile wk2132_rx_buffer_index_t _rx_buffer_head;
  volatile wk2132_rx_buffer_index_t _rx_buffer_tail;
  unsigned char _rx_buffer[WK2132_SERIAL_RX_BUFFER_SIZE];

  // ========== WK2132 Core Functions ==========

  /**
   * Initialize WK2132 chip with default configuration.
   * Performs: clock enable, reset, FIFO setup, baud config.
   *
   * @param baud Baud rate
   * @param format Data format (default WK2132_8N1)
   * @param mode Communication mode (default normal)
   * @param opt Line break option (default normal)
   * @return WK2132_ERR_OK if successful, error code otherwise
   */
  int wk2132Begin(unsigned long baud, uint8_t format = WK2132_8N1,
                  WK2132_CommunicationMode mode = WK2132_MODE_NORMAL,
                  WK2132_LineBreakOutput opt = WK2132_LINEBREAK_NORMAL);

  /**
   * Configure sub UART: enable clock, reset, configure FIFO and control registers.
   *
   * @param subUartChannel Channel to configure
   */
  void wk2132SubSerialConfig(uint8_t subUartChannel);

  /**
   * Enable/configure global register for a sub UART.
   *
   * @param subUartChannel Channel to configure
   * @param type Register type (clock, reset, interrupt)
   */
  void wk2132SubSerialGlobalRegEnable(uint8_t subUartChannel, WK2132_GlobalRegType type);

  /**
   * Set baud rate for current sub UART.
   *
   * @param baud Baud rate
   */
  void wk2132SetSubSerialBaudRate(unsigned long baud);

  /**
   * Configure line control register (data format, IR mode, line break).
   *
   * @param format Data format
   * @param mode Communication mode
   * @param opt Line break option
   */
  void wk2132SetSubSerialConfigReg(uint8_t format, WK2132_CommunicationMode mode,
                                   WK2132_LineBreakOutput opt);

  /**
   * Switch register page (page 0 or page 1).
   *
   * @param page Page number
   */
  void wk2132SubSerialPageSwitch(WK2132_PageNumber page);

  /**
   * Configure sub UART register by OR-ing value.
   *
   * @param reg Register address
   * @param pValue Pointer to value to OR with current register value
   */
  void wk2132SubSerialRegConfig(uint8_t reg, void *pValue);

  /**
   * Get global register address from type.
   *
   * @param type Global register type
   * @return Register address
   */
  uint8_t wk2132GetGlobalRegType(WK2132_GlobalRegType type);

  /**
   * Switch active sub UART channel.
   *
   * @param subUartChannel New channel
   * @return Previous channel
   */
  uint8_t wk2132SubSerialChnnlSwitch(uint8_t subUartChannel);

  /**
   * Read FIFO status register.
   *
   * @return FIFO status register value
   */
  WK2132_FsrReg_t wk2132ReadFIFOStateReg();

  /**
   * Update I2C address with channel and object type.
   *
   * @param pre Address prefix (from DIP switches)
   * @param subUartChannel Sub UART channel
   * @param obj Object type (register or FIFO)
   * @return Complete I2C address
   */
  uint8_t wk2132UpdateAddr(uint8_t pre, uint8_t subUartChannel, uint8_t obj);

  // ========== I2C Communication Functions ==========

  /**
   * Write to WK2132 register.
   *
   * @param reg Register address
   * @param pBuf Data to write
   * @param size Number of bytes to write
   */
  void wk2132WriteReg(uint8_t reg, const void *pBuf, size_t size);

  /**
   * Read from WK2132 register.
   *
   * @param reg Register address
   * @param pBuf Buffer to store data
   * @param size Number of bytes to read
   * @return Number of bytes read (0 if failed)
   */
  uint8_t wk2132ReadReg(uint8_t reg, void *pBuf, size_t size);

  /**
   * Write to WK2132 FIFO.
   * Automatically chunks data into IIC_BUFFER_SIZE segments.
   *
   * @param pBuf Data to write
   * @param size Number of bytes to write
   */
  void wk2132WriteFIFO(void *pBuf, size_t size);

  /**
   * Read from WK2132 FIFO.
   * Automatically chunks read into IIC_BUFFER_SIZE segments.
   *
   * @param pBuf Buffer to store data
   * @param size Number of bytes to read
   * @return Number of bytes read (0 if failed)
   */
  uint8_t wk2132ReadFIFO(void *pBuf, size_t size);

  /**
   * Get or create I2C device handle for given address.
   * Caches device handles to avoid repeated initialization.
   *
   * @param addr I2C address
   * @return Device handle, or NULL if failed
   */
  i2c_master_dev_handle_t wk2132GetDeviceHandle(uint8_t addr);
};

#endif // AIRGRADIENT_IICSERIAL_HPP
