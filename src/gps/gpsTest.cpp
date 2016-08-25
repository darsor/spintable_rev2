#include "gps.h"
#include <cstdio>
#include <wiringPi.h>

int main() {
    Gps gps;

    while(true) {
        printf("gpstime: %f\n", gps.getTime());
    }
    return 0;
}
