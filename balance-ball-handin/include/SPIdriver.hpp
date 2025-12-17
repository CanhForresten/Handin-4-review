#pragma once
#include <com_interface.hpp>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string>

class SPIDriver : public SYSHAT::ICommInterface {
public:
  SPIDriver(const char *spi_device);
  SPIDriver(SPIDriver &&) = default;
  SPIDriver(const SPIDriver &) = default;
  SPIDriver &operator=(SPIDriver &&) = default;
  SPIDriver &operator=(const SPIDriver &) = default;
  ~SPIDriver();

  // read and write
  int8_t write(uint8_t slaveAddress, const uint8_t *buf,
               uint16_t length) override;

  int8_t read(uint8_t slaveAddress, uint8_t *buf, uint16_t length) override;

  int8_t transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer,
                  uint16_t length);

  void spi_delayMicroseconds(uint32_t ms);

private:
  int fd_ = -1;
  uint32_t speed_ = 1000000; // 1MHz
  uint8_t mode_ = SPI_MODE_0;
  uint8_t bits_ = 8;
};
