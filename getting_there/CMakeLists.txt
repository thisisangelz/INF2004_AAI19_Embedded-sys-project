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
    getting_there.c motion.c line_search.c wifi_sampler.c payload_client.c run_report.c report_store.c report_flash.c faults.c
    ../pico_rssi_tracker/imu_mpu9250.c
)
target_include_directories(getting_there PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../pico_rssi_tracker
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer
)
target_compile_options(getting_there PRIVATE -Wall -Wextra)
option(LINE_SEARCH_MOVING_SAMPLES "Enable moving acquisition after hardware timing checks and runtime qualification" OFF)
target_compile_definitions(getting_there PRIVATE LINE_SEARCH_MOVING_SAMPLES=$<BOOL:${LINE_SEARCH_MOVING_SAMPLES}>)
target_link_libraries(getting_there
    pico_stdlib pico_cyw43_arch_lwip_threadsafe_background
    pico_btstack_cyw43 pico_btstack_ble
    hardware_i2c hardware_pwm hardware_clocks hardware_flash pico_flash
)
# Keep the SDK's physical flash bounds at 2 MiB. Shrinking PICO_FLASH_SIZE_BYTES
# to reserve logs makes SDK 2.3.0 hard-assert on the first journal erase.
# Limit only the linker region, leaving 8 KiB for BTstack before the journal.
if (NOT COMMAND pico_add_linker_script_override_path)
    message(FATAL_ERROR "Persistent logs require linker region overrides (tested SDK: 2.3.0)")
endif()
pico_add_linker_script_override_path(getting_there
    ${CMAKE_CURRENT_LIST_DIR}/linker FILES pico_flash_region.ld)
target_compile_definitions(getting_there PRIVATE
    PICO_FLASH_SIZE_BYTES=2097152
    PICO_FLASH_BANK_TOTAL_SIZE=8192
    PICO_FLASH_BANK_STORAGE_OFFSET=1564672
)
# Core 1 is unused. Re-audit before introducing core 1 or an RTOS.
target_compile_definitions(getting_there PRIVATE PICO_FLASH_ASSUME_CORE1_SAFE=1)
pico_enable_stdio_usb(getting_there 1)
pico_enable_stdio_uart(getting_there 0)
pico_add_extra_outputs(getting_there)
