#ifndef IMU_H
#define IMU_H

#include <stdint.h> // for uint16_6

// HMC5883L config registers
#define HMC5883L_CONFIG_REG_A	0x00
#define HMC5883L_CONFIG_REG_B	0x01
#define HMC5883L_MODE		    0x02

// HMC5883L Sensor Read registers
#define HMC5883L_RA_DATAX_H	0x03
#define HMC5883L_RA_DATAX_L	0x04
#define HMC5883L_RA_DATAZ_H	0x05
#define HMC5883L_RA_DATAZ_L	0x06
#define HMC5883L_RA_DATAY_H	0x07
#define HMC5883L_RA_DATAY_L	0x08

//---->  LSM6DS3 Info: <-----
//LSM6DS3 gyroscope registers
#define LSM6DS3_OUTX_L_G 	0x22
#define LSM6DS3_OUTX_H_G 	0x23
#define LSM6DS3_OUTY_L_G 	0x24
#define LSM6DS3_OUTY_H_G 	0x25
#define LSM6DS3_OUTZ_L_G 	0x26
#define LSM6DS3_OUTZ_H_G 	0x27

//LSM6DS3 accelerometer registers
#define LSM6DS3_OUTX_L_XL 	0x28
#define LSM6DS3_OUTX_H_XL 	0x29
#define LSM6DS3_OUTY_L_XL 	0x2A
#define LSM6DS3_OUTY_H_XL 	0x2B
#define LSM6DS3_OUTZ_L_XL 	0x2C
#define LSM6DS3_OUTZ_H_XL 	0x2D

//LSM6DS3 config registers
#define LSM6DS3_WHO_AM_I	  0x0F
#define LSM6DS3_CTRL1_XL 	  0x10
#define LSM6DS3_CTRL2_G		  0x11
#define LSM6DS3_CTRL3_C       0x12
#define LSM6DS3_CTRL5_C 	  0x14
#define LSM6DS3_CTRL9_XL	  0x18
#define LSM6DS3_CTRL10_C 	  0x19
#define LSM6DS3_MASTER_CONFIG 0x1A

//LSM6DS3 timestamp registers
#define LSM6DS3_TAP_CFG        0x58
#define LSM6DS3_WAKE_UP_DUR    0x5C
#define LSM6DS3_TIMESTAMP0_REG 0x40
#define LSM6DS3_TIMESTAMP1_REG 0x41
#define LSM6DS3_TIMESTAMP2_REG 0x42

//LSM6DS3 slave configuration registers
#define LSM6DS3_FUNC_CFG_ACCESS 0x01
#define LSM6DS3_SLV0_ADD 	    0x02
#define LSM6DS3_SLV0_SUBADD 	0x03
#define LSM6DS3_SLAVE0_CONFIG 	0x04
#define LSM6DS3_SLV1_ADD 	    0x05
#define LSM6DS3_SLV1_SUBADD 	0x06
#define LSM6DS3_SLAVE1_CONFIG 	0x07
#define LSM6DS3_DATAWRITE_SLV0 	0x0E

//LSM6DS3 sensorhub registers
#define LSM6DS3_SENSORHUB1_REG 0x2E
#define LSM6DS3_SENSORHUB2_REG 0x2F
#define LSM6DS3_SENSORHUB3_REG 0x30
#define LSM6DS3_SENSORHUB4_REG 0x31
#define LSM6DS3_SENSORHUB5_REG 0x32
#define LSM6DS3_SENSORHUB6_REG 0x33

//LSM6DS3 FIFO registers
#define LSM6DS3_FIFO_CTRL1      0x06
#define LSM6DS3_FIFO_CTRL2      0x07
#define LSM6DS3_FIFO_CTRL3      0x08
#define LSM6DS3_FIFO_CTRL4      0x09
#define LSM6DS3_FIFO_CTRL5      0x0A
#define LSM6DS3_FIFO_STATUS1    0x3A
#define LSM6DS3_FIFO_STATUS2    0x3B
#define LSM6DS3_FIFO_STATUS3    0x3C
#define LSM6DS3_FIFO_STATUS4    0x3D
#define LSM6DS3_FIFO_DATA_OUT_L 0x3E
#define LSM6DS3_FIFO_DATA_OUT_H 0x3F

struct FifoBlock {
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t mx;
    int16_t my;
    int16_t mz;
    uint32_t ts;
};

class Imu {
    public:
        Imu(int addr);
        ~Imu();

        void fifoEnable(bool state);
        bool isFifoEmpty();
        bool isFifoFull();
        bool isFifoFilled();

        void fifoClear();
        uint16_t fifoRead();
        int fifoReadBlock(unsigned char* buffer, int size);
        static void fifoParse(unsigned char* buffer, FifoBlock &block);

        uint16_t fifoSize();
        uint16_t fifoStatus();
        uint16_t fifoPattern();

        uint32_t getTimestamp();
        void resetTimestamp();

    private:
        int fdImu, fdMag;
};

#endif
