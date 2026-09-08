/**
 * buzzer_test/main.c
 *
 * Standalone test for the Maker Pi Pico's onboard piezo buzzer (GP18).
 * Plays a short beep, pauses, repeats. Uses PWM to generate a tone,
 * since the buzzer is a passive piezo speaker (needs a driven signal,
 * not just an on/off voltage).
 */
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define BUZZER_PIN 18

// Play a tone of the given frequency (Hz) for duration_ms, then stop.
static void beep(uint gpio, uint freq_hz, uint duration_ms) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);                 // 125MHz / 4 = 31.25MHz PWM tick
    uint32_t wrap = (125000000 / 4) / freq_hz - 1;
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(gpio, wrap / 2);  // 50% duty cycle = square wave
    sleep_ms(duration_ms);
    pwm_set_gpio_level(gpio, 0);          // silence
}

int main() {
    stdio_init_all();

    while (true) {
        beep(BUZZER_PIN, 1000, 300);   // 1kHz beep, 300ms
        sleep_ms(700);
    }
}
