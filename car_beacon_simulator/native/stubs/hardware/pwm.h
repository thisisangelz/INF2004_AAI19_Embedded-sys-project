#include "pico/stdlib.h"
typedef struct { uint16_t wrap; } pwm_config;
pwm_config pwm_get_default_config(void);
void pwm_config_set_clkdiv(pwm_config *config, float divider);
void pwm_config_set_wrap(pwm_config *config, uint16_t wrap);
void pwm_init(uint slice, const pwm_config *config, bool enabled);
uint pwm_gpio_to_slice_num(uint pin);
void pwm_set_gpio_level(uint pin, uint16_t level);
void pwm_set_wrap(uint slice, uint16_t wrap);
void pwm_set_clkdiv(uint slice, float divider);
void pwm_set_enabled(uint slice, bool enabled);
