#include "I2Cdriver.hpp"
#include <cstdint>
#include <ctime>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

I2CDriver::I2CDriver() {
  if (fd_ >= 0) {
    perror("I2C bus already open");
  }
  const char *i2c_device_file =
      "/dev/i2c-1"; // *** IMPORTANT: CHANGE IF YOUR BUS IS DIFFERENT ***
  fd_ = open(i2c_device_file, O_RDWR);
  if (fd_ < 0) {
    perror("Failed to open I2C device file");
  }
}

I2CDriver::~I2CDriver() {
  close(fd_);
  fd_ = -1;
}

int8_t I2CDriver::write(uint8_t slaveAddress, const uint8_t *buf,
                         uint16_t length) {
  if (ioctl(fd_, I2C_SLAVE, slaveAddress) < 0) {
    perror("Failed setting i2c slave");
    return -1;
  }

  if (::write(fd_, buf, length) != length) {
    perror("Failed writing i2c slave");
    return -2;
  }

  return 0; // Success
}

int8_t I2CDriver::read(uint8_t slaveAddress, uint8_t *buf, uint16_t length) {
  if (ioctl(fd_, I2C_SLAVE, slaveAddress) < 0) {
    perror("Failed setting i2c slave");
    return -1;
  }

  ssize_t bytesRead = ::read(fd_, buf, length);
  if (bytesRead != static_cast<ssize_t>(length)) {
    perror("Failed or incomplete read of i2c slave");
    return -2;
  }

  return 0; // Success
}
