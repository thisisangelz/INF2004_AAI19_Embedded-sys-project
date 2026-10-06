cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(../pico_rssi_tracker/pico_sdk_import.cmake)
project(getting_there C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(getting_there
    getting_there.c motion.c payload_client.c run_report.c
    ../pico_rssi_tracker/imu_mpu9250.c
)
target_include_directories(getting_there PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../pico_rssi_tracker
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer
)
target_compile_options(getting_there PRIVATE -Wall -Wextra)
target_link_libraries(getting_there
    pico_stdlib pico_cyw43_arch_lwip_threadsafe_background
    pico_btstack_cyw43 pico_btstack_ble
    hardware_i2c hardware_pwm hardware_clocks
)
pico_enable_stdio_usb(getting_there 1)
pico_enable_stdio_uart(getting_there 0)
pico_add_extra_outputs(getting_there)
