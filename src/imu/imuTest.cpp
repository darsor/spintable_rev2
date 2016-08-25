#include "imu.h"
#include <cstdio>
#include <cerrno>
#include <unistd.h>
#include <sys/time.h>

int main() {
    Imu imu1(0x6B);
    Imu imu2(0x6A);
    imu1.fifoEnable(true);
    imu2.fifoEnable(true);
    unsigned char buffer[2496];
    FifoBlock data;
    imu1.resetTimestamp();
    imu2.resetTimestamp();
    imu1.fifoClear();
    imu2.fifoClear();
    timeval start, end;

    for (int i=0; i<10; i++) {

        while (!imu1.isFifoFilled()) usleep(1000);
        gettimeofday(&start, NULL);
        if (imu1.fifoPattern() != 0) printf("PROBLEM: pattern mismatch\n");
        printf("FIFO1 contains %d unread samples and the current pattern is %d\n", imu1.fifoSize(), imu1.fifoPattern());
        if (imu1.fifoReadBlock(buffer, 2496) < 0) perror("imu1 read error");

        for (int j=0; j<104; j++) {
            Imu::fifoParse(buffer + (j*24), data);
            
            printf("gyro1 x: %-7d", data.gx);
            printf("y: %-7d",       data.gy);
            printf("z: %-9d",       data.gz);
            printf("accel1 x: %-7d",data.ax);
            printf("y: %-7d",       data.ay);
            printf("z: %-9d",       data.az);
            printf("mag1 x: %-7d",  data.mx);
            printf("y: %-7d",       data.my);
            printf("z: %-9d",       data.mz);
            printf("timestamp: %u\n",data.ts);
        }
        gettimeofday(&end, NULL);
        printf("time elapsed: %f seconds\n", end.tv_sec + (end.tv_usec/1000000.0) - (start.tv_sec + (start.tv_usec/1000000.0)));

        while (!imu2.isFifoFilled()) usleep(1000);
        gettimeofday(&start, NULL);
        if (imu2.fifoPattern() != 0) printf("PROBLEM: pattern mismatch\n");
        printf("FIFO2 contains %d unread samples and the current pattern is %d\n", imu2.fifoSize(), imu2.fifoPattern());
        if (imu2.fifoReadBlock(buffer, 2496) < 0) perror("imu2 read error");

        for (int j=0; j<104; j++) {
            Imu::fifoParse(buffer + (j*24), data);

            printf("gyro2 x: %-7d", data.gx);
            printf("y: %-7d",       data.gy);
            printf("z: %-9d",       data.gz);
            printf("accel2 x: %-7d",data.ax);
            printf("y: %-7d",       data.ay);
            printf("z: %-9d",       data.az);
            printf("mag2 x: %-7d",  data.mx);
            printf("y: %-7d",       data.my);
            printf("z: %-9d",       data.mz);
            printf("timestamp: %u\n",data.ts);
        }
        gettimeofday(&end, NULL);
        printf("time elapsed: %f seconds\n", end.tv_sec + (end.tv_usec/1000000.0) - (start.tv_sec + (start.tv_usec/1000000.0)));
    }
    return 0;
}
