# Install script for directory: C:/Users/thisi/Documents/Pico-v1.5.1/pico-examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/pico_examples")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi/bin/arm-none-eabi-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/pico_extras/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/pico-sdk/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/blink/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/hello_world/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/adc/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/clocks/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/cmake/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/divider/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/dma/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/flash/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/gpio/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/i2c/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/interp/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/multicore/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/picoboard/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/pico_w/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/pio/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/pwm/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/reset/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/rtc/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/spi/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/system/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/timer/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/uart/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/usb/cmake_install.cmake")
  include("C:/Users/thisi/Documents/Pico-v1.5.1/build/watchdog/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "C:/Users/thisi/Documents/Pico-v1.5.1/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
