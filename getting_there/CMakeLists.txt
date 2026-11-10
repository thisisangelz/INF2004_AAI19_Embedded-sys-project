cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(../pico_sdk_import.cmake)
project(getting_there C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    pico_sdk_init()
endif()
if (NOT PICO_BOARD STREQUAL "pico_w")
    message(FATAL_ERROR "The log flash layout is for the 2 MiB RP2040 Pico W only")
endif()
option(LINE_SEARCH_MOVING_SAMPLES "Enable moving acquisition after hardware timing checks and runtime qualification" OFF)
# Keep the SDK's physical flash bounds at 2 MiB. Shrinking PICO_FLASH_SIZE_BYTES
# to reserve logs makes SDK 2.3.0 hard-assert on the first journal erase.
# Limit only the linker region, leaving 8 KiB for BTstack before the journal.
if (NOT COMMAND pico_add_linker_script_override_path)
    message(FATAL_ERROR "Persistent logs require linker region overrides (tested SDK: 2.3.0)")
endif()

# Every robot firmware target shares the sources, flash layout and settings.
function(getting_there_firmware target)
    add_executable(${target}
        getting_there.c target_beacon.c motion.c line_search.c wifi_sampler.c payload_client.c run_report.c report_store.c report_flash.c faults.c colour_sensor.c colour_mat.c robot_cues.c
        ../common/buzzer.c
        imu_mpu9250.c
    )
    target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer
        ${CMAKE_CURRENT_LIST_DIR}/../common
    )
    target_compile_options(${target} PRIVATE -Wall -Wextra)
    target_compile_definitions(${target} PRIVATE LINE_SEARCH_MOVING_SAMPLES=$<BOOL:${LINE_SEARCH_MOVING_SAMPLES}>)
    target_link_libraries(${target}
        pico_stdlib pico_cyw43_arch_lwip_threadsafe_background
        pico_btstack_cyw43 pico_btstack_ble
        hardware_i2c hardware_pwm hardware_clocks hardware_flash pico_flash
    )
    pico_add_linker_script_override_path(${target}
        ${CMAKE_CURRENT_LIST_DIR}/linker FILES pico_flash_region.ld)
    target_compile_definitions(${target} PRIVATE
        PICO_FLASH_SIZE_BYTES=2097152
        PICO_FLASH_BANK_TOTAL_SIZE=8192
        PICO_FLASH_BANK_STORAGE_OFFSET=1564672
    )
    # Core 1 is unused. Re-audit before introducing core 1 or an RTOS.
    target_compile_definitions(${target} PRIVATE PICO_FLASH_ASSUME_CORE1_SAFE=1)
    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)
    pico_add_extra_outputs(${target})
endfunction()

getting_there_firmware(getting_there)
# Route variants share navigation, colour qualification and transfer logic.
getting_there_firmware(getting_there_route2)
target_compile_definitions(getting_there_route2 PRIVATE ROUTE_BEACON_COUNT=2)
getting_there_firmware(getting_there_route)
target_compile_definitions(getting_there_route PRIVATE THREE_BEACON_ROUTE=1)
# Fixed targets select their matching Wi-Fi SSID, BLE ID and colour profile.
getting_there_firmware(robot_ap2)
target_compile_definitions(robot_ap2 PRIVATE TARGET_BEACON_ID=2)
getting_there_firmware(robot_ap3)
target_compile_definitions(robot_ap3 PRIVATE TARGET_BEACON_ID=3)
# Test build: after gyro calibration, turns on the spot and logs the colour
# under the sensor at 30-degree steps instead of approaching AP1.
# Short target name: longer ones push SDK object paths past Windows' 260-character limit.
getting_there_firmware(colour_scan)
target_compile_definitions(colour_scan PRIVATE COLOUR_SCAN_TEST=1)
