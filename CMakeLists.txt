cmake_minimum_required(VERSION 3.13)

# Builds the robot and all three powerbank beacons, so the project can
# be opened and compiled from the repository root (e.g. by the Raspberry Pi
# Pico VS Code extension). Each subfolder can still be built on its own.

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(aai19_beacon_robot C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(getting_there)
add_subdirectory(picow_access_point_powerbank)
add_subdirectory(picow_access_point2_powerbank)
add_subdirectory(picow_access_point3_powerbank)
add_subdirectory(tcs34725_calib)
