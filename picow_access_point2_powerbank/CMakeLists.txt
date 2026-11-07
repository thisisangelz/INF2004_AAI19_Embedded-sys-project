cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(picow_access_point2_powerbank C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise the SDK only when this folder is built on its own; a parent
# CMakeLists.txt can initialise it once for multiple targets.
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    pico_sdk_init()
endif()

add_executable(picow_access_point2_powerbank
    ../ble_transfer_powerbank/beacon_server.c
    ../ble_transfer_powerbank/powerbank_load.c
    ../common/buzzer.c
)

target_include_directories(picow_access_point2_powerbank PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer_powerbank
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer
    ${CMAKE_CURRENT_LIST_DIR}/../common
)

target_compile_definitions(picow_access_point2_powerbank PRIVATE
    BEACON_ID=2
    WIFI_SSID="I AM PICO W 2"
)

target_link_libraries(picow_access_point2_powerbank
    pico_stdlib
    pico_multicore
    pico_flash
    pico_cyw43_arch_lwip_threadsafe_background
    pico_btstack_cyw43
    pico_btstack_ble
    hardware_pwm
)

pico_btstack_make_gatt_header(picow_access_point2_powerbank PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer/file_transfer.gatt
)

pico_enable_stdio_usb(picow_access_point2_powerbank 1)
pico_enable_stdio_uart(picow_access_point2_powerbank 0)

pico_add_extra_outputs(picow_access_point2_powerbank)
