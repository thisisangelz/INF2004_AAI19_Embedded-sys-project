add_executable(buzzer_test
        main.c
        )

target_link_libraries(buzzer_test
        pico_stdlib
        hardware_pwm
        )

pico_enable_stdio_usb(buzzer_test 1)
pico_enable_stdio_uart(buzzer_test 0)

pico_add_extra_outputs(buzzer_test)
