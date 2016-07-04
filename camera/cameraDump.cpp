//#define X_res   320
//#define X_res   1280
#define X_res     2592 
//#define Y_res   240
//#define Y_res   960
#define Y_res     1944

#include <raspicam/raspicam.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>

using namespace raspicam;

int main() {

    unsigned char* pBuffer;
    unsigned int bufferSize;
    {
        raspicam::RaspiCam camera;
        camera.setWidth(X_res);
        camera.setHeight(Y_res);
        camera.setFormat(RASPICAM_FORMAT_GRAY);
        camera.setHorizontalFlip(true);
        camera.setVerticalFlip(true);
        //camera.setContrast(100);
        //camera.setShutterSpeed(1000); // microseconds
        if (!camera.open()) printf("ERROR: Camera not opened\n");
        else printf("Camera opened\n");
        usleep(2000000);
        printf("size of buffer: %d\n", camera.getImageBufferSize());
        bufferSize = camera.getImageBufferSize();
        pBuffer = new unsigned char[bufferSize];
        printf("taking picture...\n");
        camera.grab();
        camera.retrieve(pBuffer);
    }

    std::ofstream ofile("image", std::ios::binary);
    printf("file is%s open\n", ofile.is_open() ? "" : " not");
    printf("writing to file...\n");
    ofile.write((char*) pBuffer, bufferSize);
    ofile.close();
    std::ostringstream cmd;
    printf("converting to png...\n");
    cmd << "convert -depth 8 -size " << X_res << "x" << Y_res << "+0 gray:image out.png";
    std::system(cmd.str().c_str());
    printf("cleaning up...\n");
    std::system("sudo rm image");
    delete pBuffer;
    return 0;
}
