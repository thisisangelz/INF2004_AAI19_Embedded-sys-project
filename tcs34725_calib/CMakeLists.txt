cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(../pico_sdk_import.cmake)
project(tcs34725_calib C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    pico_sdk_init()
endif()

# Rings inside the white edge. tcs34725_calib: pink mat (AP1), 5 rings.
# tcs34725_calib6: blue and purple mats, 6 rings (seven bands counting the
# white edge). Same source; only the list of surfaces to capture differs.
function(tcs34725_calib_target target rings)
    add_executable(${target} tcs34725_calib.c)
    target_compile_options(${target} PRIVATE -Wall -Wextra)
    target_compile_definitions(${target} PRIVATE MAT_RINGS=${rings})
    target_link_libraries(${target} pico_stdlib hardware_i2c hardware_pwm hardware_clocks)
    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)
    pico_add_extra_outputs(${target})
endfunction()

tcs34725_calib_target(tcs34725_calib 5)
tcs34725_calib_target(tcs34725_calib6 6)
