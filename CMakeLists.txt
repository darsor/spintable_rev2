cmake_minimum_required (VERSION 3.0.2 FATAL_ERROR)
project (COSMOS_Server CXX)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o3 -Wall")

set (COMMON_LIBS ${EXTRA_LIBS} wiringPi)

set (SENSOR_LIBS raspicam)

set (MOTOR_LIBS pthread)

set(COMMON_SOURCES
    gps/gps.cpp
    cosmos/cosmos.cpp
    cosmos/packets.cpp
    cosmos/cosmosQueue.cpp
)

set(SENSOR_SOURCES
    sensorpi.cpp
    imu/imu.cpp
)

set(MOTOR_SOURCES
    motorpi.cpp
    motor/dcmotor.cpp
    motor/pwm.cpp
    motor/decoder.cpp
    motor/pid.cpp
)

add_executable (sensor ${COMMON_SOURCES} ${SENSOR_SOURCES})
add_executable (motor ${COMMON_SOURCES} ${MOTOR_SOURCES})

target_link_libraries (sensor ${COMMON_LIBS} ${SENSOR_LIBS})
target_link_libraries (motor ${COMMON_LIBS} ${MOTOR_LIBS})