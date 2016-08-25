/**********************
 * Packet Definitions *
 **********************/
#ifndef PACKETS_H
#define PACKETS_H

#define MOTOR_TIME_PKT_SIZE     18
#define SENSOR_TIME_PKT_SIZE    26
#define IMU_PKT_SIZE            28
#define CAM_PKT_SIZE            76814
#define ENC_PKT_SIZE            18
#define CAM_POWER_SIZE          8
#define GYRO_RES_SIZE           10
#define ACCEL_RES_SIZE          10
#define MOTOR_SET_HOME_SIZE     6
#define MOTOR_SET_SPEED_SIZE    8
#define MOTOR_ABS_POS_SIZE      10
#define MOTOR_REV_POS_SIZE      10
#define MOTOR_GOTO_INDEX_SIZE   6
#define CMD_IMU_RESET_SIZE      8

#define IMU1_PKT_ID             1
#define IMU2_PKT_ID             2
#define MOTOR_TIME_PKT_ID       3
#define CAM_PKT_ID              4
#define ENC_PKT_ID              5
#define CAM_CMD_ID              6
#define GYRO_CMD_ID             7
#define ACCEL_CMD_ID            8
#define MOTOR_SET_HOME_ID       9
#define MOTOR_SET_SPEED_ID      10
#define MOTOR_ABS_POS_ID        11
#define MOTOR_REV_POS_ID        12
#define MOTOR_GOTO_INDEX_ID     13
#define CMD_IMU_RESET_ID        14
#define SENSOR_TIME_PKT_ID      15

#include <cstdint>

inline void endianSwap(float &f) {
    float temp = f;
    unsigned char* pf = (unsigned char*) &f;
    unsigned char* pt = (unsigned char*) &temp;
    pf[0] = pt[3];
    pf[1] = pt[2];
    pf[2] = pt[1];
    pf[3] = pt[0];
}

class Packet {
    public:
        Packet(const uint32_t length, const uint16_t id);
        Packet(const Packet& that);
        Packet& operator=(const Packet& that);
        virtual ~Packet();
        virtual void convert();
        uint32_t length;
        uint16_t id;
        unsigned char* buffer;
};

class MotorTimePacket: public Packet {
    public:
        MotorTimePacket();
        void convert();
        float gpsTime;
        uint32_t sysTimeSeconds;
        uint32_t sysTimeuSeconds;
};

class SensorTimePacket: public Packet {
    public:
        SensorTimePacket();
        void convert();
        float gpsTime;
        uint32_t imuTime1;
        uint32_t imuTime2;
        uint32_t sysTimeSeconds;
        uint32_t sysTimeuSeconds;
};

class ImuPacket: public Packet {
    public:
        ImuPacket(int id);
        void convert();
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

class CameraPacket: public Packet {
    public:
        CameraPacket();
        void convert();
        unsigned char pBuffer[76800];
        uint32_t sysTimeSeconds;
        uint32_t sysTimeuSeconds;
};

class EncoderPacket: public Packet {
    public:
        EncoderPacket();
        void convert();
        uint32_t sysTimeSeconds;
        uint32_t sysTimeuSeconds;
        int32_t raw_cnt;
};

class CameraPowerCmd: public Packet {
    public:
        CameraPowerCmd();
        void convert();
        uint16_t power;
};

class GyroResolutionCmd: public Packet {
    public:
        GyroResolutionCmd();
        void convert();
        uint16_t imu;
        uint16_t resolution;
};

class AccelResolutionCmd: public Packet {
    public:
        AccelResolutionCmd();
        void convert();
        uint16_t imu;
        uint16_t resolution;
};

class SetSpeedCmd: public Packet {
    public:
        SetSpeedCmd();
        void convert();
        int16_t speed;
};

class SetAbsPosCmd: public Packet {
    public:
        SetAbsPosCmd();
        void convert();
        float position;
};

class SetRevPosCmd: public Packet {
    public:
        SetRevPosCmd();
        void convert();
        float position;
};

class CmdImuReset: public Packet {
    public:
        CmdImuReset();
        void convert();
        uint16_t imu;
};

#endif
