#include <cstdint>
#include <cstring>
#include <thread>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "broker.hpp"
#include "accelerometer.hpp"

#define ACC_NORMAL_MODE 0x11

#define ACC_X_LSB 0x12
#define ACC_X_MSB 0x13
#define ACC_Y_LSB 0x14
#define ACC_Y_MSB 0x15
#define ACC_Z_LSB 0x16
#define ACC_Z_MSB 0x17

#define SPI_DEVICE "/dev/spidev0.0"

int fd = -1;
struct spi_ioc_transfer tx[1] = {0};

// Initialize SPI bus
void Accelerometer::initSPI(std::string path_name) {
    fd = open(path_name.c_str(), O_RDWR);

    if (fd < 0) {
        perror("open()");
        exit(EXIT_FAILURE);
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED;
    int ret = 0;

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) {
        perror("ioctl(SPI_IOC_WR_MODE)");
    }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) {
        perror("ioctl(SPI_IOC_WR_BITS_PER_WORD)");
    } 

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        perror("ioctl(SPI_IOC_WR_MAX_SPEED_HZ)");
    }  
}

// Read from register
int Accelerometer::readReg(uint8_t reg, uint8_t nbytes) {
    if (nbytes > MAXBUFSIZE) {
        return -1;
    }

    uint8_t buf[nbytes+1] = {0};

    buf[0] = reg | BMI160_READ_BIT;

    tx[0].tx_buf = (__u64)buf;
    tx[0].rx_buf = (__u64)buf;
    tx[0].len = (__u32)nbytes + 1;
    tx[0].cs_change = 0;
    tx[0].delay_usecs = 0;
    tx[0].speed_hz = SPI_SPEED;
    tx[0].bits_per_word = SPI_BITS_PER_WORD;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tx) < 0) {
        perror("ioctl()");
        close(fd);
        exit(EXIT_FAILURE);
    }

    for (uint8_t i = 0; i < nbytes; ++i) {
        buffer[i] = buf[i+1];
    }

  return 0;
}


// Write to register
int Accelerometer::writeReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {0};
    struct spi_ioc_transfer tx[1] = {0}; // Assume tx is declared as this type

    buf[0] = reg;
    buf[1] = data;

    tx[0].tx_buf = (__u64)buf;
    tx[0].rx_buf = 0; 
    tx[0].len = (__u32)sizeof(buf);
    tx[0].cs_change = 0;
    tx[0].delay_usecs = 0;
    tx[0].speed_hz = SPI_SPEED;
    tx[0].bits_per_word = SPI_BITS_PER_WORD;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tx) < 0) {
        perror("ioctl()");
        close(fd);
        exit(EXIT_FAILURE);
    }
  return 0;
}

//Turn on accelerometer
void Accelerometer::startAccel(void) {
    writeReg(BMI160_CMD_REG, ACC_NORMAL_MODE);
    usleep(100000);
}

// Check if accelerometer data is available
bool Accelerometer::isAccelDataAvailable(void) {
    
    if (readReg(BMI160_STATUS_REG, 1) < 0) {
        return false;
    }

    uint8_t status_reg_value = buffer[0]; 

    return (status_reg_value & 0x80) != 0;
}

// Read accelerometer data (6 bytes)
void Accelerometer::readAccel(void) {

    readReg(ACC_X_LSB, 6);

}

Accelerometer::Accelerometer(std::string path_name){
    initSPI(path_name);
    startAccel();
    isActive = true;
}

Accelerometer::~Accelerometer()
{
    isActive = false;
    if (fd >= 0) {
        close(fd);
        fd = -1; 
    }
}

void Accelerometer::accelerometerThread()
{
    char buf[16];
    while (isActive)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        while(isActive && !isAccelDataAvailable()){
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        if (!isActive){
            break;
        }

        readAccel();

        int16_t raw_x = (int16_t)( (buffer[1] << 8) | buffer[0] );
        int16_t raw_y = (int16_t)( (buffer[3] << 8) | buffer[2] );
        int16_t raw_z = (int16_t)( (buffer[5] << 8) | buffer[4] );

        double ax = (double)raw_x / BMI160_ACCEL_SENS;
        double ay = (double)raw_y / BMI160_ACCEL_SENS;
        double az = (double)raw_z / BMI160_ACCEL_SENS;

        // Encode til string
        std::string data = Message::encodeAccelerometerData(ax, ay, az);

        // Lav message med topic "accl"
        auto msg = std::make_unique<Message>("accl", data);

        // Publish til broker
        Broker::getInstance().publish(std::move(msg));
        
        
        // Check if Accelerometer data is available
        // in while loop. Sleep for 100 
        // microseconds, if not

        // Read acceleration

        // Convert to doubles (x,y,z)

        // Encode data

        // Create message & publish
    }
}