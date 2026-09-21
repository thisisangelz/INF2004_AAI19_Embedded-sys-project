cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(picow_access_point C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(picow_access_point
    picow_access_point.c
)

target_include_directories(picow_access_point PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(picow_access_point
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(picow_access_point 1)
pico_enable_stdio_uart(picow_access_point 0)

pico_add_extra_outputs(picow_access_point)
