cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pico_rssi_tracker C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_rssi_tracker
    pico_rssi_tracker.c
)

target_include_directories(pico_rssi_tracker PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer
)

target_link_libraries(pico_rssi_tracker
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_btstack_cyw43
    pico_btstack_ble
    hardware_pwm
    hardware_clocks
)

pico_enable_stdio_usb(pico_rssi_tracker 1)
pico_enable_stdio_uart(pico_rssi_tracker 0)

pico_add_extra_outputs(pico_rssi_tracker)
