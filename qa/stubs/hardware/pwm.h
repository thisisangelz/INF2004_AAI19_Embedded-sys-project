#ifndef QA_STUB_HARDWARE_PWM_H
#define QA_STUB_HARDWARE_PWM_H
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
void pwm_set_gpio_level(uint pin, uint16_t level);
uint pwm_gpio_to_slice_num(uint pin);
void pwm_set_wrap(uint slice, uint16_t wrap);
void pwm_set_clkdiv(uint slice, float divider);
void pwm_set_enabled(uint slice, bool enabled);
#endif

