#include "cosmos/cosmosQueue.h"
#include "gps/gps.h"
#include "imu/imu.h"
#include <raspicam/raspicam.h>
#include <wiringPi.h>
#include <sys/time.h>
#include <unistd.h>
#include <sstream>
#include <cstdio>
#include <mutex>
#include <condition_variable>

using namespace raspicam;

// function prototypes
void systemTimestamp(uint32_t &stime, uint32_t &ustime);

// initialize packets
TimePacket* tPacket = NULL;
ImuPacket* iPacket = NULL;

// make the CosmosQueue global (so that all threads can access it)
CosmosQueue queue(4810, 512);

PI_THREAD (cameraControl) {
    CameraPacket* cPacket = NULL;
    raspicam::RaspiCam camera;
    camera.setWidth(320);
    camera.setHeight(240);
    camera.setFormat(RASPICAM_FORMAT_GRAY);
    if (!camera.open()) printf("ERROR: Camera not opened\n");
    else printf("Camera opened\n");
    usleep(3000000);
    printf("size of buffer: %d\n", camera.getImageBufferSize());
    while (true) {
        cPacket = new CameraPacket();
        camera.grab(); //TODO: timestamp before or after this?
        systemTimestamp(cPacket->sysTimeSeconds, cPacket->sysTimeuSeconds);
        camera.retrieve(cPacket->pBuffer);
        queue.push(cPacket);
        usleep(80000);
    }
}

PI_THREAD (cosmosQueue) {
    while (true) {
        queue.connect();
        while (queue.isConnected()) {
            while (queue.pop());
            usleep(10000);
        }
        printf("connection with COSMOS lost\n");
    }
}

int main() {

    // set up wiringPi
    //wiringPiSetup();

    // start threads
    if (piThreadCreate(cosmosQueue) != 0) {
        perror("COSMOS queue thread didn't start");
    }
    if (piThreadCreate(cameraControl) != 0) {
        perror("Camera control thread didn't start");
    }

    // initialize devices
    Gps gps;
    Imu imu1(0x6A);
    Imu imu2(0x6B);

    // set high priority for this thread
    pid_t pid = getpid();
    std::stringstream cmd;
    cmd << "renice -n -2 -p " << pid;
    if (pid > 0)
        system(cmd.str().c_str());

    FifoBlock data;
    unsigned char buffer[24];
    imu1.resetTimestamp(); //TODO: handle timestamp overflow
    imu2.resetTimestamp();
    imu1.fifoEnable(true);
    imu2.fifoEnable(true);
    usleep(1000000);
    imu1.fifoClear(); //TODO: throw away first imu values
    imu2.fifoClear(); // or does this handle that?
    while (true) {

        // get timestamps and send time packet
        /*
         *tPacket = new TimePacket();
         *gps.timestampPPS(tPacket->sysTimeSeconds, tPacket->sysTimeuSeconds);
         *tPacket->imuTime1 = imu1.getTimestamp(); //TODO: test this function
         *tPacket->imuTime2 = imu2.getTimestamp(); //TODO: test this function
         *tPacket->gpsTime = gps.getTime(); //TODO: this might delay: put in other thread?
         */

        //if (imu1.fifoPattern() != 0) printf("PROBLEM: pattern mismatch\n");
        while (imu1.fifoPattern()) imu1.fifoRead();
        if (imu1.fifoSize() >= 24 && imu1.fifoPattern() == 0) {
            //printf("FIFO1 contains %d unread samples and the current pattern is %d\n", imu1.fifoSize(), imu1.fifoPattern());
            if (imu1.fifoReadBlock(buffer, 24) < 24) printf("PROBLEM!!! IMU1 not read correctly\n");
            //if (imu1.fifoReadBlock(buffer, 24) < 0) perror("imu1 read error");
            iPacket = new ImuPacket(1);
            Imu::fifoParse(buffer, data);
            iPacket->gx = data.gx;
            iPacket->gy = data.gy;
            iPacket->gz = data.gz;
            iPacket->ax = data.ax;
            iPacket->ay = data.ay;
            iPacket->az = data.az;
            iPacket->mx = data.mx;
            iPacket->my = data.my;
            iPacket->mz = data.mz;
            iPacket->ts = data.ts;
            queue.push(iPacket);
        }

        //if (imu2.fifoPattern() != 0) printf("PROBLEM: pattern mismatch\n");
        while (imu2.fifoPattern()) imu2.fifoRead();
        if (imu2.fifoSize() >= 24 && imu2.fifoPattern() == 0) {
            //printf("FIFO2 contains %d unread samples and the current pattern is %d\n", imu2.fifoSize(), imu2.fifoPattern());
            if (imu2.fifoReadBlock(buffer, 24) < 24) printf("PROBLEM!!! IMU2 not read correctly\n");
            //if (imu2.fifoReadBlock(buffer, 24) < 0) perror("imu2 read error");
            iPacket = new ImuPacket(2);
            Imu::fifoParse(buffer, data);
            iPacket->gx = data.gx;
            iPacket->gy = data.gy;
            iPacket->gz = data.gz;
            iPacket->ax = data.ax;
            iPacket->ay = data.ay;
            iPacket->az = data.az;
            iPacket->mx = data.mx;
            iPacket->my = data.my;
            iPacket->mz = data.mz;
            iPacket->ts = data.ts;
            queue.push(iPacket);
        }
        usleep(1000);
    }
    return 0;
}

void systemTimestamp(uint32_t &stime, uint32_t &ustime) {
    static struct timeval timeVal;
    gettimeofday(&timeVal, NULL);
    stime =  timeVal.tv_sec;
    ustime = timeVal.tv_usec;
}
