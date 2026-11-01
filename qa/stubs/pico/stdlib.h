#ifndef QA_STUB_PICO_STDLIB_H
#define QA_STUB_PICO_STDLIB_H
#include <stdbool.h>
#include <stdint.h>
typedef unsigned int uint;
#define GPIO_IN 0
#define GPIO_FUNC_I2C 3
#define GPIO_FUNC_PWM 4
uint64_t time_us_64(void);
void sleep_ms(uint32_t milliseconds);
void gpio_init(uint pin);
void gpio_set_dir(uint pin, bool output);
void gpio_pull_up(uint pin);
bool gpio_get(uint pin);
void gpio_set_function(uint pin, uint function);
#endif

