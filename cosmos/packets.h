/**********************
 * Packet Definitions *
 **********************/
#ifndef PACKETS_H
#define PACKETS_H

#define TIME_PKT_SIZE       26
#define IMU_PKT_SIZE        28
#define CAM_PKT_SIZE        102414
#define ENC_PKT_SIZE        30

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
        virtual ~Packet();
        virtual void convert();
        uint32_t length;
        uint16_t id;
        unsigned char* buffer;
};

class TimePacket: public Packet {
    public:
        TimePacket();
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
        unsigned char pBuffer[102400];
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
        float motorSpeed;
        float position;
        uint32_t rev_cnt;
};

#endif
