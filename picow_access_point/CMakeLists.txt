cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(picow_access_point C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(picow_access_point
    ../ble_transfer/beacon_server.c
)

target_include_directories(picow_access_point PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer
)

target_compile_definitions(picow_access_point PRIVATE
    BEACON_ID=1
    WIFI_SSID="I AM PICO W"
)

target_link_libraries(picow_access_point
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_btstack_cyw43
    pico_btstack_ble
)

pico_btstack_make_gatt_header(picow_access_point PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../ble_transfer/file_transfer.gatt
)

pico_enable_stdio_usb(picow_access_point 1)
pico_enable_stdio_uart(picow_access_point 0)

pico_add_extra_outputs(picow_access_point)
