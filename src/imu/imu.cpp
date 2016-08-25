#include "imu.h"
#include <wiringPiI2C.h>
//#include <wiringPi.h>
#include <stdlib.h> // for system(), exit()
#include <sys/time.h> // for timing
#include <unistd.h> // for usleep() and read()
#include <stdio.h> // for printf

Imu::Imu(int addr) : address(addr) {
    fdImu = wiringPiI2CSetup(address);
    if (fdImu < 0) {
        printf("IMU not connected, exiting...\n");
        exit(1);
    }
    initialize();
}

Imu::~Imu() {
    close(fdImu);
    close(fdMag);
}

void Imu::reset() {
    initialize();
}

void Imu::initialize() {

    // software reset to ensure that everything is at defaults
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL2_G, 		0x00); // turn off gyroscope
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL3_C, 		0x05); // software reset
    usleep(500);

    // enable pass-through to set up magnetometer
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_MASTER_CONFIG, 	0x04); // enable pass-through mode
    fdMag = wiringPiI2CSetup(0x1E); 				           // 0x1E is the default address for the HMC5883L
    if (fdMag < 0) {
        printf("MAG not connected, exiting...\n");
        exit(1);
    }
    wiringPiI2CWriteReg8(fdMag, HMC5883L_CONFIG_REG_A, 	0x18); // set the HMC5883L to output data at 75Hz
    wiringPiI2CWriteReg8(fdMag, HMC5883L_MODE, 		    0x01); // enable single measurement mode
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_MASTER_CONFIG, 	0x00); // disable the pass-through on the LSM6DS3

    // set up FIFO
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_WAKE_UP_DUR,    0x10); // set timestamp to high resolution (25μs)
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_TAP_CFG,        0x80); // enable the IMU timestamp
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_FIFO_CTRL1,     0xE0); // set watermark level at 104 samples
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_FIFO_CTRL2,     0x84); // enable timestamp as 4th FIFO data set
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_FIFO_CTRL3,     0x09); // enable accel and gryo data in FIFO
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_FIFO_CTRL4,     0x09); // enable timestamp/sensorhub data in FIFO

    // set up sensorhub
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL3_C,        0x44); // enable block data update
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_FUNC_CFG_ACCESS,0x80); // enable access to slave config registers
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_SLV0_ADD, 	    0x3C); // give HMC5883L address to LSM6DS3 (write)
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_SLV0_SUBADD, 	HMC5883L_MODE); // mode register (to make another measurement)
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_SLAVE0_CONFIG, 	0x10); // there are 2 sensors configured (one write, one read)
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_DATAWRITE_SLV0, 0x01); // single measurement on HMC5883L
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_SLV1_ADD, 	    0x3D); // give HMC5883L address to LSM6DS3 (read)
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_SLV1_SUBADD, 	HMC5883L_RA_DATAX_H); // first data output register
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_SLAVE1_CONFIG, 	0x06); // there are 6 data registers to read
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_FUNC_CFG_ACCESS,0x00); // disable access to slave config registers
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL10_C, 		0x3C); // enable the sensor hub on LSM6DS3
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_MASTER_CONFIG, 	0x01); // enable I2C master interface

    // turn on accelerometer and gyroscope
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL1_XL, 		0x40); // turn on accelerometer at 104Hz
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL2_G, 		0x40); // turn on gyroscope at 104Hz
    usleep(100000);
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_MASTER_CONFIG, 	0x11); // use drdy on int2 for external magnetometer
}

void Imu::fifoEnable(bool state) {
    if (state) {
        wiringPiI2CWriteReg8(fdImu, LSM6DS3_FIFO_CTRL5, 0x21); // enable FIFO mode at 104Hz
    } else {
        wiringPiI2CWriteReg8(fdImu, LSM6DS3_FIFO_CTRL5, 0x00); // disable FIFO
    }
}

bool Imu::isFifoEmpty() {
    return wiringPiI2CReadReg8(fdImu, LSM6DS3_FIFO_STATUS2) & 0x10;
}

bool Imu::isFifoFull() {
    return wiringPiI2CReadReg8(fdImu, LSM6DS3_FIFO_STATUS2) & 0x20;
}

bool Imu::isFifoFilled() {
    return wiringPiI2CReadReg8(fdImu, LSM6DS3_FIFO_STATUS2) & 0x80;
}

void Imu::fifoClear() {
    while (fifoSize() > 50) fifoReadBlock(nullptr, fifoSize());
    while (!isFifoEmpty()) {
        fifoRead();
    }
    while (fifoPattern()) fifoRead();
    printf("IMU FIFO cleared\n");
}

uint16_t Imu::fifoRead() {
    return (uint16_t) wiringPiI2CReadReg16(fdImu, LSM6DS3_FIFO_DATA_OUT_L);
}

int Imu::fifoReadBlock(unsigned char* buffer, int size) {
    wiringPiI2CWrite(fdImu, LSM6DS3_FIFO_DATA_OUT_L);
    return read(fdImu, buffer, size);
}

uint16_t Imu::fifoSize() {
    return fifoStatus() & 0x0FFF;
}

uint16_t Imu::fifoStatus() {
    uint8_t dl = wiringPiI2CReadReg8(fdImu, LSM6DS3_FIFO_STATUS1);
    uint16_t dh = wiringPiI2CReadReg8(fdImu, LSM6DS3_FIFO_STATUS2);
    return (dh<<8)|dl;
}

uint16_t Imu::fifoPattern() {
    uint8_t dl = wiringPiI2CReadReg8(fdImu, LSM6DS3_FIFO_STATUS3);
    uint16_t dh = wiringPiI2CReadReg8(fdImu, LSM6DS3_FIFO_STATUS4);
    return (dh<<8)|dl;
}

uint32_t Imu::getTimestamp() {
    uint32_t dl = wiringPiI2CReadReg8(fdImu, LSM6DS3_TIMESTAMP0_REG);
    uint32_t dm = wiringPiI2CReadReg8(fdImu, LSM6DS3_TIMESTAMP1_REG);
    uint32_t dh = wiringPiI2CReadReg8(fdImu, LSM6DS3_TIMESTAMP2_REG);
    return (((dh<<8)|dm)<<8)|dl;
}

void Imu::resetTimestamp() {
    wiringPiI2CWriteReg8(fdImu, LSM6DS3_TIMESTAMP2_REG, 0xAA); // reset IMU timestamp
}

void Imu::fifoParse(unsigned char* buffer, FifoBlock &block) {
    block.gx = *((int16_t*) (buffer+ 0));
    block.gy = *((int16_t*) (buffer+ 2));
    block.gz = *((int16_t*) (buffer+ 4));
    block.ax = *((int16_t*) (buffer+ 6));
    block.ay = *((int16_t*) (buffer+ 8));
    block.az = *((int16_t*) (buffer+10));
    block.mx = (buffer[12] << 8) | buffer[13];
    block.my = (buffer[14] << 8) | buffer[15];
    block.mz = (buffer[16] << 8) | buffer[17];
    block.ts = (((buffer[19] << 8) | buffer[18]) << 8) | buffer[21];
}

void Imu::setGyroResolution(unsigned int res) {
    switch (res) { // 0: 125 dps; 1: 245 dps; 2: 500 dps, 3: 1000 dps; 4: 2000 dps
        case 0:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL2_G, 0x42);
            break;
        case 1:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL2_G, 0x40);
            break;
        case 2:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL2_G, 0x44);
            break;
        case 3:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL2_G, 0x48);
            break;
        case 4:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL2_G, 0x4C);
            break;
        default:
            printf("setGyroResolution failed. Valid inputs are 0-4\n");
    }
}

void Imu::setAccelResolution(unsigned int res) {
    switch (res) { // 0: ±2 g; 1: ±4 g; 2: ±8 g, 3: ±16 g
        case 0:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL1_XL, 0x40);
            break;
        case 1:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL1_XL, 0x48);
            break;
        case 2:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL1_XL, 0x4C);
            break;
        case 3:
            wiringPiI2CWriteReg8(fdImu, LSM6DS3_CTRL1_XL, 0x44);
            break;
        default:
            printf("setAccelResolution failed. Valid inputs are 0-3\n");
    }
}
