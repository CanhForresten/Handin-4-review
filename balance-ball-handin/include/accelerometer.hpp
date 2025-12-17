#pragma once
#include <linux/spi/spidev.h>
#include "message.hpp"

// For acceleration
#define BMI160_ACCEL_REG      0x12
#define BMI160_ACCEL_SENS     16384.0   // for ±2g
#define BMI160_CMD_REG 0x7E
#define BMI160_CHIP_ID_REG 0x00
#define BMI160_STATUS_REG 0x1B
#define BMI160_READ_BIT 0x80
#define SPI_SPEED 1000000 // 1 MHz
#define SPI_BITS_PER_WORD 8
#define MAXBUFSIZE 32

class Accelerometer
{
    bool isActive;
    int fd;
    uint8_t buffer[MAXBUFSIZE] = {0};
    struct spi_ioc_transfer tx[1] = {0};
    void initSPI(std::string path_name);
    int readReg(uint8_t reg, uint8_t nbytes);
    int writeReg(uint8_t reg, uint8_t data);
    void startAccel(void);
    bool isAccelDataAvailable(void);
    void readAccel(void);

    public:
        explicit Accelerometer(std::string path_name);
        ~Accelerometer();
        void accelerometerThread();
    };
