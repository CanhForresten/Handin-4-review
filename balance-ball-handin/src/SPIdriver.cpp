#include "SPIdriver.hpp"
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

// Your SPIDriver class implementation
SPIDriver::SPIDriver(const char *spi_device) {
 
  /*
   * TODO:
   * Open spidev node
   * Set SPI mode
   * Set bits per word
   * Set SPI speed
 */

}

SPIDriver::~SPIDriver() {

  /*
   * TODO:
   * Close spidev node
   */

}

// This low-level transfer method is the correct way to perform a full-duplex
// SPI transaction.
int8_t SPIDriver::transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer,
                           uint16_t length) {
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = reinterpret_cast<uint64_t>(tx_buffer);
  tr.rx_buf = reinterpret_cast<uint64_t>(rx_buffer);
  tr.len = length;
  tr.delay_usecs = 0;
  tr.speed_hz = speed_;
  tr.bits_per_word = bits_;

  if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
    perror("SPI transfer failed");
    return 1;
  }
  return 0;
}

int8_t SPIDriver::write(const uint8_t slaveAddress, const uint8_t *buf,
                        const uint16_t length) {

  uint8_t reg_addr = slaveAddress;
  std::vector<uint8_t> tx_buffer(length + 1);
  std::vector<uint8_t> rx_buffer(length + 1);

  /*
   * TODO:
   * Implement write
   * - The register address is received via the 'slaveAddress' parameter
   * - The total length of the SPI transaction is the data length + 1 byte for the address
   * - For a write, we can use a dummy rx buffer to receive the data
   */

  // Call transfer with tx/rx buffers and size
  return transfer(tx_buffer.data(), rx_buffer.data(), tx_buffer.size());
}

int8_t SPIDriver::read(const uint8_t slaveAddress, uint8_t *buf,
                       const uint16_t length) {
  uint8_t reg_addr = slaveAddress;
  std::vector<uint8_t> tx_buffer(length + 1, 0x00);
  std::vector<uint8_t> rx_buffer(length + 1);
  int8_t result = 0;

  /*
   * TODO:
   * Implement write
   * - The register address is received via the 'slaveAddress' parameter
   * - The transaction needs a tx buffer with the read address and dummy bytes.
   * - The total length of the SPI transaction is the data length + 1 byte for the address
   * - The rx buffer will store the received data.
   */


  if (result == 0) {
    // Copy the received data (skipping the first dummy byte).
    std::memcpy(buf, rx_buffer.data() + 1, length);
  }
  return result;
}

void SPIDriver::spi_delayMicroseconds(uint32_t ms) {
  // ... (Your delay implementation) ...
}
