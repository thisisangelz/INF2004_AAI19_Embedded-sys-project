/**
 * my_scanner/main.c
 *
 * Adapted from the official pico-examples pico_w/wifi/wifi_scan example.
 * Scans for a specific beacon SSID, tracks its RSSI, and turns the
 * onboard LED on/off based on proximity. Uses two thresholds (enter/exit)
 * plus a miss-counter so minor signal fluctuation doesn't flicker the LED.
 *
 * Also beeps the onboard buzzer (GP18, Maker Pi Pico) on state changes:
 * a short high beep when the beacon enters range, a short low beep when
 * it leaves.
 */
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"

#define TARGET_SSID          "PICO_BEACON_1"
#define RSSI_ENTER_THRESHOLD (-65)   // dBm - must be at least this strong to count as "found"
#define RSSI_EXIT_THRESHOLD  (-75)   // dBm - must drop below this to count as "lost" (hysteresis)
#define MISS_LIMIT           3       // consecutive missed scans before declaring "lost"
#define SCAN_INTERVAL_MS     2000    // how often to (re)start a scan

#define BUZZER_PIN 18

static volatile bool    beacon_seen_this_scan = false;
static volatile int32_t last_rssi = -999;

// Play a tone of the given frequency (Hz) for duration_ms, then stop.
static void beep(uint gpio, uint freq_hz, uint duration_ms) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    uint32_t wrap = (125000000 / 4) / freq_hz - 1;
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(gpio, wrap / 2);  // 50% duty cycle = square wave
    sleep_ms(duration_ms);
    pwm_set_gpio_level(gpio, 0);          // silence
}

static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result && strcmp((const char *)result->ssid, TARGET_SSID) == 0) {
        beacon_seen_this_scan = true;
        last_rssi = result->rssi;
    }
    return 0;
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    bool in_range = false;
    int  miss_count = 0;

    absolute_time_t scan_time = nil_time;
    bool scan_in_progress = false;

    while (true) {
        if (absolute_time_diff_us(get_absolute_time(), scan_time) < 0) {
            if (!scan_in_progress) {
                beacon_seen_this_scan = false;
                cyw43_wifi_scan_options_t scan_options = {0};
                int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
                if (err == 0) {
                    scan_in_progress = true;
                } else {
                    printf("Failed to start scan: %d\n", err);
                    scan_time = make_timeout_time_ms(SCAN_INTERVAL_MS);
                }
            } else if (!cyw43_wifi_scan_active(&cyw43_state)) {
                // this scan cycle just finished - evaluate the result
                if (beacon_seen_this_scan) {
                    miss_count = 0;
                    printf("Beacon RSSI: %d dBm\n", (int)last_rssi);

                    if (!in_range && last_rssi >= RSSI_ENTER_THRESHOLD) {
                        in_range = true;
                        printf(">>> ENTERED RANGE\n");
                        beep(BUZZER_PIN, 1200, 150);   // short high beep
                    } else if (in_range && last_rssi < RSSI_EXIT_THRESHOLD) {
                        in_range = false;
                        printf(">>> LEFT RANGE\n");
                        beep(BUZZER_PIN, 400, 300);    // short low beep
                    }
                } else {
                    miss_count++;
                    printf("Beacon not seen (miss %d)\n", miss_count);
                    if (miss_count >= MISS_LIMIT) {
                        in_range = false;
                    }
                }

                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, in_range);

                scan_time = make_timeout_time_ms(SCAN_INTERVAL_MS);
                scan_in_progress = false;
            }
        }

#if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(scan_time);
#else
        sleep_ms(100);
#endif
    }

    cyw43_arch_deinit();
    return 0;
}
