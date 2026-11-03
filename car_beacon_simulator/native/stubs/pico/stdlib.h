#ifndef HOST_PICO_STDLIB_H
#define HOST_PICO_STDLIB_H
#include <stdbool.h>
#include <stdint.h>
typedef unsigned int uint;
#define GPIO_FUNC_PWM 4
#define GPIO_FUNC_I2C 3
#define GPIO_IN 0
void gpio_init(uint pin);
void gpio_set_dir(uint pin, bool out);
void gpio_pull_up(uint pin);
bool gpio_get(uint pin);
uint64_t time_us_64(void);
void sleep_ms(uint32_t milliseconds);
void gpio_set_function(uint pin, uint function);
void stdio_init_all(void);
#endif
