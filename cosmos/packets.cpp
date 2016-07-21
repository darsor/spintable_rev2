#include "packets.h"
#include <arpa/inet.h>
#include <cstring>
#include <cstdio>

Packet::Packet(const uint32_t length, const uint16_t id):
    length(length), id(id) {
    buffer = new unsigned char[length];
}
Packet::Packet(const Packet& that): length(that.length), id(that.id) {
    buffer = new unsigned char[that.length];
}
Packet::~Packet() {
    delete[] buffer;
}
Packet& Packet::operator=(const Packet& that) {
    if (this != &that)
    {
        delete[] buffer;
        buffer = new unsigned char[that.length];
        length = that.length;
        id = that.id;
    }
    return *this;
}
void Packet::convert() { printf("Virtual convert() called. That's a problem\n"); }

MotorTimePacket::MotorTimePacket(): Packet(MOTOR_TIME_PKT_SIZE, MOTOR_TIME_PKT_ID) {}

void MotorTimePacket::convert() {
    uint16_t u16;
    uint32_t u32;
    float f;
    u32 = htonl(length);
    memcpy(buffer+0,  &u32, 4);
    u16 = htons(id);
    memcpy(buffer+4,  &u16, 2);
    f = gpsTime;
    endianSwap(f);
    memcpy(buffer+6,  &f, 4);
    u32 = htonl(sysTimeSeconds);
    memcpy(buffer+10,  &u32, 4);
    u32 = htonl(sysTimeuSeconds);
    memcpy(buffer+14, &u32, 4);
};

SensorTimePacket::SensorTimePacket(): Packet(SENSOR_TIME_PKT_SIZE, SENSOR_TIME_PKT_ID) {}

void SensorTimePacket::convert() {
    uint16_t u16;
    uint32_t u32;
    float f;
    u32 = htonl(length);
    memcpy(buffer+0,  &u32, 4);
    u16 = htons(id);
    memcpy(buffer+4,  &u16, 2);
    f = gpsTime;
    endianSwap(f);
    memcpy(buffer+6,  &f, 4);
    u32 = htonl(imuTime1);
    memcpy(buffer+10,  &u32, 4);
    u32 = htonl(imuTime2);
    memcpy(buffer+14, &u32, 4);
    u32 = htonl(sysTimeSeconds);
    memcpy(buffer+18,  &u32, 4);
    u32 = htonl(sysTimeuSeconds);
    memcpy(buffer+22, &u32, 4);
};

ImuPacket::ImuPacket(int id): Packet(IMU_PKT_SIZE, id) {}

void ImuPacket::convert() {
    int16_t i16;
    uint16_t u16;
    uint32_t u32;
    u32 = htonl(length);
    memcpy(buffer+0,  &u32, 4);
    u16 = htons(id);
    memcpy(buffer+4,  &u16, 2);
    i16= htons(gx);
    memcpy(buffer+6,  &i16, 2);
    i16= htons(gy);
    memcpy(buffer+8,  &i16, 2);
    i16= htons(gz);
    memcpy(buffer+10,  &i16, 2);
    i16= htons(ax);
    memcpy(buffer+12,  &i16, 2);
    i16= htons(ay);
    memcpy(buffer+14,  &i16, 2);
    i16= htons(az);
    memcpy(buffer+16,  &i16, 2);
    i16= htons(mx);
    memcpy(buffer+18,  &i16, 2);
    i16= htons(my);
    memcpy(buffer+20,  &i16, 2);
    i16= htons(mz);
    memcpy(buffer+22,  &i16, 2);
    u32= htonl(ts);
    memcpy(buffer+24,  &u32, 4);
};

CameraPacket::CameraPacket(): Packet(CAM_PKT_SIZE, CAM_PKT_ID) {}

void CameraPacket::convert() {
    uint16_t u16;
    uint32_t u32;
    u32 = htonl(length);
    memcpy(buffer+0, &u32, 4);
    u16 = htons(id);
    memcpy(buffer+4, &u16, 2);
    memcpy(buffer+6, pBuffer, 76800);
    u32 = htonl(sysTimeSeconds);
    memcpy(buffer+76806, &u32, 4);
    u32 = htonl(sysTimeuSeconds);
    memcpy(buffer+76810, &u32, 4);
};

EncoderPacket::EncoderPacket(): Packet(ENC_PKT_SIZE, ENC_PKT_ID) {}

void EncoderPacket::convert() {
    static uint16_t u16;
    static uint32_t u32;
    static int32_t i32;
    u32 = htonl(length);
    memcpy(buffer+0,  &u32, 4);
    u16 = htons(id);
    memcpy(buffer+4,  &u16, 2);
    u32 = htonl(sysTimeSeconds);
    memcpy(buffer+6,  &u32, 4);
    u32 = htonl(sysTimeuSeconds);
    memcpy(buffer+10, &u32, 4);
    i32 = htonl(raw_cnt);
    memcpy(buffer+14, &i32, 4);
};

CameraPowerCmd::CameraPowerCmd() : Packet(CAM_POWER_SIZE, CAM_CMD_ID) {}

void CameraPowerCmd::convert() {
    static uint16_t u16;
    memcpy(&u16, buffer+6, sizeof(u16));
    power = ntohs(u16);
}

GyroResolutionCmd::GyroResolutionCmd() : Packet(GYRO_RES_SIZE, GYRO_CMD_ID) {}

void GyroResolutionCmd::convert() {
    static uint16_t u16;
    memcpy(&u16, buffer+6, sizeof(u16));
    imu = ntohs(u16);
    memcpy(&u16, buffer+8, sizeof(u16));
    resolution = ntohs(u16);
}

AccelResolutionCmd::AccelResolutionCmd() : Packet(ACCEL_RES_SIZE, ACCEL_CMD_ID) {}

void AccelResolutionCmd::convert() {
    static uint16_t u16;
    memcpy(&u16, buffer+6, sizeof(u16));
    imu = ntohs(u16);
    memcpy(&u16, buffer+8, sizeof(u16));
    resolution = ntohs(u16);
}

SetSpeedCmd::SetSpeedCmd() : Packet(MOTOR_SET_SPEED_SIZE, MOTOR_SET_SPEED_ID) {}

void SetSpeedCmd::convert() {
    static uint16_t u16;
    memcpy(&u16, buffer+6, sizeof(u16));
    speed = ntohs(u16);
}

SetAbsPosCmd::SetAbsPosCmd() : Packet(MOTOR_ABS_POS_SIZE, MOTOR_ABS_POS_ID) {}

void SetAbsPosCmd::convert() {
    memcpy(&position, buffer+6, sizeof(position));
    endianSwap(position);
}

SetRevPosCmd::SetRevPosCmd() : Packet(MOTOR_REV_POS_SIZE, MOTOR_REV_POS_ID) {}

void SetRevPosCmd::convert() {
    memcpy(&position, buffer+6, sizeof(position));
    endianSwap(position);
}

CmdImuReset::CmdImuReset() : Packet(CMD_IMU_RESET_SIZE, CMD_IMU_RESET_ID) {}

void CmdImuReset::convert() {
    static uint16_t u16;
    memcpy(&u16, buffer+6, sizeof(u16));
    imu = ntohs(u16);
}
