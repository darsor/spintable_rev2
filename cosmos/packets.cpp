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
void Packet::convert() {}

TimePacket::TimePacket(): Packet(TIME_PKT_SIZE, 3) {}

void TimePacket::convert() {
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

CameraPacket::CameraPacket(): Packet(CAM_PKT_SIZE, 4) {}

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

EncoderPacket::EncoderPacket(): Packet(ENC_PKT_SIZE, 5) {}

void EncoderPacket::convert() {
    static uint16_t u16;
    static uint32_t u32;
    static int32_t i32;
    static float f;
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
    f = motorSpeed;
    endianSwap(f);
    memcpy(buffer+18, &f, 4);
    f = position;
    endianSwap(f);
    memcpy(buffer+22, &f, 4);
    u32 = htonl(rev_cnt);
    memcpy(buffer+26, &u32, 4);
};
