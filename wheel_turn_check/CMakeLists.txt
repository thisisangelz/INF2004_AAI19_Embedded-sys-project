cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(../pico_rssi_tracker/pico_sdk_import.cmake)
project(wheel_turn_check C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(wheel_turn_check
    wheel_turn_check.c
    ../pico_rssi_tracker/imu_mpu9250.c
)
target_include_directories(wheel_turn_check PRIVATE ../pico_rssi_tracker)
target_link_libraries(wheel_turn_check pico_stdlib hardware_i2c hardware_pwm)
pico_enable_stdio_usb(wheel_turn_check 1)
pico_enable_stdio_uart(wheel_turn_check 0)
pico_add_extra_outputs(wheel_turn_check)
