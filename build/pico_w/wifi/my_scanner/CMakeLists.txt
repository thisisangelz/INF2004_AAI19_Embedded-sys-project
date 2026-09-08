add_executable(my_scanner
        main.c
        )

target_include_directories(my_scanner PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )

target_link_libraries(my_scanner
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        hardware_pwm
        )

pico_enable_stdio_usb(my_scanner 1)
pico_enable_stdio_uart(my_scanner 0)

pico_add_extra_outputs(my_scanner)
