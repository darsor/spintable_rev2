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
#include <atomic>
#include <condition_variable>

using namespace raspicam;

// function prototypes
void systemTimestamp(uint32_t &stime, uint32_t &ustime);

// initialize packets
SensorTimePacket* tPacket = nullptr;
ImuPacket* iPacket = nullptr;

// make the CosmosQueue global (so that all threads can access it)
CosmosQueue queue(4810, 20000, 8);

std::atomic<bool> camera_state;
std::condition_variable camera_cv;
std::mutex camera_mutex;
PI_THREAD (cameraControl) {
    camera_state.store(false);
    CameraPacket* cPacket = nullptr;
    raspicam::RaspiCam camera;
    camera.setWidth(320);
    camera.setHeight(240);
    camera.setFormat(RASPICAM_FORMAT_GRAY);
    camera.setHorizontalFlip(true);
    camera.setVerticalFlip(true);
    while (true) {
        if (camera_state.load()) {
            cPacket = new CameraPacket();
            camera.grab(); //TODO: timestamp before or after this?
            systemTimestamp(cPacket->sysTimeSeconds, cPacket->sysTimeuSeconds);
            camera.retrieve(cPacket->pBuffer);
            queue.push_tlm(cPacket);
            usleep(5000);
        } else {
            camera.release();
            std::unique_lock<std::mutex> lk(camera_mutex);
            camera_cv.wait(lk, []{return camera_state.load();});
            if (!camera.open()) {
                printf("ERROR: Camera not opened\n");
                return nullptr;
            }
            printf("Camera opened\n");
            usleep(2500000);
        }
    }
    return nullptr;
}

int main() {

    // start threads
    if (piThreadCreate(cameraControl) != 0) {
        perror("Camera control thread didn't start");
    }

    // initialize devices
    Gps gps(1, "/dev/ttyS0", 9600);
    Imu imu1(0x6A);
    Imu imu2(0x6B);

    // set high priority for this thread
    pid_t pid = getpid();
    std::stringstream cmd;
    cmd << "renice -n -2 -p " << pid;
    if (pid > 0) system(cmd.str().c_str());

    FifoBlock data;
    unsigned char buffer[24];
    timeval start, end;
    long difference;
    Packet* cmdPacket = nullptr;
    imu1.resetTimestamp();
    imu2.resetTimestamp();
    usleep(1000000);
    imu1.fifoEnable(true);
    imu2.fifoEnable(true);
    while (true) {

        // get timestamps and send time packet
        tPacket = new SensorTimePacket();
        while (!gps.dataAvail()) usleep(100);
        gps.timestampPPS(tPacket->sysTimeSeconds, tPacket->sysTimeuSeconds);
        tPacket->imuTime1 = imu1.getTimestamp();
        tPacket->imuTime2 = imu2.getTimestamp();
        start.tv_sec = tPacket->sysTimeSeconds;
        start.tv_usec = tPacket->sysTimeuSeconds;
        tPacket->gpsTime = gps.getTime(); //TODO: this might delay: put in other thread?
        if (tPacket->imuTime1 > 0xFF0000) imu1.resetTimestamp();
        if (tPacket->imuTime2 > 0xFF0000) imu2.resetTimestamp();
        //printf("pushed time packet with gpstime %f\n", tPacket->gpsTime);
        queue.push_tlm(tPacket);
        difference = 0;

        while (difference < 975000) { // if it's been 900ms since the GPS PPS, break out of this loop
            usleep(1000);

            if (imu1.fifoSize() >= 72) {
                if (imu1.fifoPattern() != 0) {
                    printf("fifo 1 pattern is %d. It should be zero. restarting...\n", imu1.fifoPattern());
                    imu1.fifoEnable(false);
                    usleep(50);
                    imu1.fifoEnable(true);
                    while (imu1.fifoPattern()) imu1.fifoRead();
                    continue;
                }
                //printf("FIFO1 contains %d unread samples and the current pattern is %d\n", imu1.fifoSize(), imu1.fifoPattern());
                if (imu1.fifoReadBlock(buffer, 24) < 24) {
                    printf("PROBLEM!!! IMU1 not read correctly\n");
                }
                //if (imu1.fifoReadBlock(buffer, 24) < 0) perror("imu1 read error");
                iPacket = new ImuPacket(IMU1_PKT_ID);
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
                queue.push_tlm(iPacket);
            }

            if (imu2.fifoSize() >= 72) {
                if (imu2.fifoPattern() != 0) {
                    printf("fifo 2 pattern is %d. It should be zero. restarting...\n", imu2.fifoPattern());
                    imu2.fifoEnable(false);
                    usleep(50);
                    imu2.fifoEnable(true);
                    while (imu2.fifoPattern()) imu2.fifoRead();
                    continue;
                }
                //printf("FIFO2 contains %d unread samples and the current pattern is %d\n", imu2.fifoSize(), imu2.fifoPattern());
                if (imu2.fifoReadBlock(buffer, 24) < 24) {
                    printf("PROBLEM!!! IMU2 not read correctly\n");
                }
                //if (imu2.fifoReadBlock(buffer, 24) < 0) perror("imu2 read error");
                iPacket = new ImuPacket(IMU2_PKT_ID);
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
                queue.push_tlm(iPacket);
            }

            if (queue.pop_cmd(cmdPacket)) {
                switch (cmdPacket->id) {
                    case CAM_CMD_ID:
                        {
                            CameraPowerCmd* camCmd = static_cast<CameraPowerCmd*>(cmdPacket);
                            camCmd->CameraPowerCmd::convert();
                            if (camCmd->power) {
                                std::lock_guard<std::mutex> lk(camera_mutex);
                                camera_state.store(true);
                                camera_cv.notify_one();
                                printf("Camera enabled\n");
                            } else {
                                camera_state.store(false);
                                printf("Camera disabled\n");
                            }
                        }
                        break;
                    case GYRO_CMD_ID:
                        {
                            GyroResolutionCmd* gyroCmd = static_cast<GyroResolutionCmd*>(cmdPacket);
                            gyroCmd->GyroResolutionCmd::convert();
                            if (gyroCmd->imu == 1) imu1.setGyroResolution(gyroCmd->resolution);
                            if (gyroCmd->imu == 2) imu2.setGyroResolution(gyroCmd->resolution);
                            printf("set IMU%u gyro resolution to %u\n", gyroCmd->imu, gyroCmd->resolution);
                        }
                        break;
                    case ACCEL_CMD_ID:
                        {
                            AccelResolutionCmd* accelCmd = static_cast<AccelResolutionCmd*>(cmdPacket);
                            accelCmd->AccelResolutionCmd::convert();
                            if (accelCmd->imu == 1) imu1.setAccelResolution(accelCmd->resolution);
                            if (accelCmd->imu == 2) imu2.setAccelResolution(accelCmd->resolution);
                            printf("set IMU%u accel resolution to %u\n", accelCmd->imu, accelCmd->resolution);
                        }
                        break;
                    case CMD_IMU_RESET_ID:
                        {
                            CmdImuReset* resetCmd = static_cast<CmdImuReset*>(cmdPacket);
                            resetCmd->CmdImuReset::convert();
                            if (resetCmd->imu == 1) {
                                imu1.fifoEnable(false);
                                imu1.reset();
                                usleep(100000);
                                imu1.fifoEnable(true);
                                printf("IMU%u reset\n", resetCmd->imu);
                            } else if (resetCmd->imu == 2) {
                                imu2.fifoEnable(false);
                                imu2.reset();
                                usleep(100000);
                                imu2.fifoEnable(true);
                                printf("IMU%u reset\n", resetCmd->imu);
                            } else printf("IMU%u not reset\n", resetCmd->imu);
                        }
                        break;
                    default:
                        printf("unknown command received\n");
                }
                delete cmdPacket;
                cmdPacket = nullptr;
            }

            gettimeofday(&end, nullptr);
            difference = end.tv_usec - start.tv_usec + (end.tv_sec - start.tv_sec) * 1000000;
        }
    }
    return 0;
}

void systemTimestamp(uint32_t &stime, uint32_t &ustime) {
    static struct timeval timeVal;
    gettimeofday(&timeVal, nullptr);
    stime =  timeVal.tv_sec;
    ustime = timeVal.tv_usec;
}
