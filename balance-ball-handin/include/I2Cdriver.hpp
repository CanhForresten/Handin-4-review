#pragma once
#include "com_interface.hpp"
#include <string>
class I2CDriver : public SYSHAT::ICommInterface {
public:
  I2CDriver();
  ~I2CDriver();

  // Implement all pure virtual functions
  int8_t write(uint8_t slaveAddress, const uint8_t *buf,
                uint16_t length) override;

  int8_t read(uint8_t slaveAddress, uint8_t *buf, uint16_t length) override;

private:
  int fd_ = -1;
};
