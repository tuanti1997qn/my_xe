cmake_minimum_required(VERSION 2.8.3)
project(control_car)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DLED_HEARTBEAT)
add_definitions(-DLED_COMM)
add_definitions(-DROSSERIAL_BAUDRATE=57600)
add_definitions(-DTX_BUFFER_SIZE=256)
add_definitions(-DRX_BUFFER_SIZE=256)

# Generate target for TivaC
generate_tivac_firmware(
  SRCS 
    control_car.cpp
    lib/main_c.c
    lib/my_encoder.c
    lib/my_PID.c
    lib/my_pwm.c
    lib/my_timer.c
    lib/my_imu.c
    lib/my_math.c
  INCS 
    .
    inc
  BOARD tm4c123gxl
)