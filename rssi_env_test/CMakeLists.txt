cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(../pico_rssi_tracker/pico_sdk_import.cmake)
project(rssi_env_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(rssi_env_test
    rssi_env_test.c
    ../getting_there/run_report.c
)
target_include_directories(rssi_env_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../getting_there
    ${CMAKE_CURRENT_LIST_DIR}/../pico_rssi_tracker
)
target_compile_options(rssi_env_test PRIVATE -Wall -Wextra)
target_link_libraries(rssi_env_test
    pico_stdlib pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm
)
pico_enable_stdio_usb(rssi_env_test 1)
pico_enable_stdio_uart(rssi_env_test 0)
pico_add_extra_outputs(rssi_env_test)
