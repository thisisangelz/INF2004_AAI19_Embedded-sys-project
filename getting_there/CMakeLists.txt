cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(../pico_rssi_tracker/pico_sdk_import.cmake)
project(getting_there C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    pico_sdk_init()
endif()
if (NOT PICO_BOARD STREQUAL "pico_w")
    message(FATAL_ERROR "The log flash layout is for the 2 MiB RP2040 Pico W only")
endif()
add_executable(getting_there
    getting_there.c motion.c payload_client.c run_report.c report_store.c report_flash.c faults.c
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
    hardware_i2c hardware_pwm hardware_clocks hardware_flash pico_flash
)
# Reserve the last 512 KiB of physical 2 MiB flash; overlap fails at link time.
if (NOT COMMAND pico_override_flash_size)
    message(FATAL_ERROR "Persistent logs require pico_override_flash_size (tested SDK: 2.3.0)")
endif()
pico_override_flash_size(getting_there 1572864)
# Core 1 is unused. Re-audit before introducing core 1 or an RTOS.
target_compile_definitions(getting_there PRIVATE PICO_FLASH_ASSUME_CORE1_SAFE=1)
pico_enable_stdio_usb(getting_there 1)
pico_enable_stdio_uart(getting_there 0)
pico_add_extra_outputs(getting_there)
