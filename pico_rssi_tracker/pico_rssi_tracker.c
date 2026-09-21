#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define AP_COUNT 3

static const char *const AP_SSIDS[AP_COUNT] = {
    "I AM PICO W",
    "I AM PICO W 2",
    "I AM PICO W 3",
};

static const uint AP_LED_PINS[AP_COUNT] = {2, 3, 4};

#define BUZZER_PIN 18
#define PRINT_INTERVAL_MS 200

// The buzzer uses one fixed tone. RSSI controls the time between beeps.
#define BUZZER_TONE_HZ 2000
#define BEEP_ON_MS 80
#define BEEP_INTERVAL_FAR_MS 1000
#define BEEP_INTERVAL_NEAR_MS 100

// An access point is unavailable when it has not been seen for this long.
#define RSSI_TIMEOUT_MS 5000

// A new access point must be this much stronger before the selection changes.
#define SWITCH_HYSTERESIS_DB 3

static volatile int ap_rssi[AP_COUNT] = {-127, -127, -127};
static volatile uint32_t ap_last_seen_ms[AP_COUNT] = {0, 0, 0};

static uint buzzer_slice;
static uint16_t buzzer_wrap;
static bool buzzer_enabled = false;
static bool buzzer_is_on = false;
static uint32_t buzzer_interval_ms = BEEP_INTERVAL_FAR_MS;
static uint32_t buzzer_next_event_ms = 0;

// -1 means that no access point is selected; 0..2 identify AP1..AP3.
static int selected_ap = -1;

static bool ssid_matches(const cyw43_ev_scan_result_t *result,
                         const char *target)
{
    size_t target_length = strlen(target);

    return result->ssid_len == target_length &&
           memcmp(result->ssid, target, target_length) == 0;
}

static int scan_result(void *env, const cyw43_ev_scan_result_t *result)
{
    (void)env;

    if (result == NULL) {
        return 0;
    }

    uint32_t now_ms = (uint32_t)(time_us_64() / 1000);

    for (int i = 0; i < AP_COUNT; ++i) {
        if (ssid_matches(result, AP_SSIDS[i])) {
            ap_rssi[i] = result->rssi;
            ap_last_seen_ms[i] = now_ms;
            break;
        }
    }

    return 0;
}

static void leds_init(void)
{
    for (int i = 0; i < AP_COUNT; ++i) {
        gpio_init(AP_LED_PINS[i]);
        gpio_set_dir(AP_LED_PINS[i], GPIO_OUT);
        gpio_put(AP_LED_PINS[i], 0);
    }
}

static void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 16.0f);
    pwm_init(buzzer_slice, &config, true);

    uint32_t clock_hz = clock_get_hz(clk_sys);
    uint32_t wrap =
        (uint32_t)(clock_hz / (16.0f * BUZZER_TONE_HZ)) - 1;

    if (wrap > 65535) {
        wrap = 65535;
    }
    if (wrap < 2) {
        wrap = 2;
    }

    buzzer_wrap = (uint16_t)wrap;
    pwm_set_wrap(buzzer_slice, buzzer_wrap);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

static void buzzer_set_tone(bool enabled)
{
    pwm_set_gpio_level(BUZZER_PIN, enabled ? buzzer_wrap / 2 : 0);
}

static uint32_t rssi_to_beep_interval(int rssi)
{
    if (rssi < -90) {
        rssi = -90;
    }
    if (rssi > -30) {
        rssi = -30;
    }

    int range = rssi + 90;

    return BEEP_INTERVAL_FAR_MS -
           ((uint32_t)range *
            (BEEP_INTERVAL_FAR_MS - BEEP_INTERVAL_NEAR_MS)) /
               60;
}

static void buzzer_configure(bool enabled,
                             uint32_t interval_ms,
                             uint32_t now_ms)
{
    if (!enabled) {
        buzzer_enabled = false;
        buzzer_is_on = false;
        buzzer_set_tone(false);
        return;
    }

    if (interval_ms < BEEP_ON_MS) {
        interval_ms = BEEP_ON_MS;
    }

    buzzer_interval_ms = interval_ms;

    // Start a beep immediately when the first beacon becomes available.
    if (!buzzer_enabled) {
        buzzer_enabled = true;
        buzzer_is_on = true;
        buzzer_set_tone(true);
        buzzer_next_event_ms = now_ms + BEEP_ON_MS;
    }
}

static void buzzer_service(uint32_t now_ms)
{
    if (!buzzer_enabled ||
        (int32_t)(now_ms - buzzer_next_event_ms) < 0) {
        return;
    }

    if (buzzer_is_on) {
        uint32_t off_time_ms =
            buzzer_interval_ms > BEEP_ON_MS
                ? buzzer_interval_ms - BEEP_ON_MS
                : 1;

        buzzer_is_on = false;
        buzzer_set_tone(false);
        buzzer_next_event_ms = now_ms + off_time_ms;
    } else {
        buzzer_is_on = true;
        buzzer_set_tone(true);
        buzzer_next_event_ms = now_ms + BEEP_ON_MS;
    }
}

static int find_strongest_ap(const bool ap_valid[AP_COUNT])
{
    int strongest_ap = -1;

    for (int i = 0; i < AP_COUNT; ++i) {
        if (ap_valid[i] &&
            (strongest_ap < 0 || ap_rssi[i] > ap_rssi[strongest_ap])) {
            strongest_ap = i;
        }
    }

    return strongest_ap;
}

static void update_output(const bool ap_valid[AP_COUNT], uint32_t now_ms)
{
    int strongest_ap = find_strongest_ap(ap_valid);

    if (strongest_ap < 0) {
        selected_ap = -1;
    } else if (selected_ap < 0 || !ap_valid[selected_ap]) {
        selected_ap = strongest_ap;
    } else if (strongest_ap != selected_ap &&
               ap_rssi[strongest_ap] >
                   ap_rssi[selected_ap] + SWITCH_HYSTERESIS_DB) {
        selected_ap = strongest_ap;
    }

    for (int i = 0; i < AP_COUNT; ++i) {
        gpio_put(AP_LED_PINS[i], selected_ap == i);
    }

    if (selected_ap < 0) {
        buzzer_configure(false, 0, now_ms);
    } else {
        buzzer_configure(
            true, rssi_to_beep_interval(ap_rssi[selected_ap]), now_ms);
    }
}

static void print_rssi(const bool ap_valid[AP_COUNT])
{
    for (int i = 0; i < AP_COUNT; ++i) {
        if (ap_valid[i]) {
            printf("AP%d: %d dBm", i + 1, ap_rssi[i]);
        } else {
            printf("AP%d: N/A", i + 1);
        }
        printf(" | ");
    }

    if (selected_ap < 0) {
        printf("Closer: NONE | Buzzer: OFF\n");
    } else {
        printf("Closer: AP%d | Buzzer: BEEP every %lu ms\n",
               selected_ap + 1,
               (unsigned long)rssi_to_beep_interval(ap_rssi[selected_ap]));
    }
}

int main(void)
{
    stdio_init_all();
    leds_init();
    buzzer_init();

    // Give USB serial some time to initialise.
    sleep_ms(2000);

    printf("\n");
    printf("====================================\n");
    printf(" Pico W Three-Beacon RSSI Tracker\n");
    printf("====================================\n");
    for (int i = 0; i < AP_COUNT; ++i) {
        printf("AP%d: %s\n", i + 1, AP_SSIDS[i]);
    }
    printf("Buzzer: slower beeps mean weaker signal\n");
    printf("Buzzer: faster beeps mean stronger signal\n\n");

    if (cyw43_arch_init()) {
        printf("ERROR: Failed to initialise Wi-Fi\n");
        return 1;
    }

    // The tracker scans as a station but does not connect to an AP.
    cyw43_arch_enable_sta_mode();
    printf("Wi-Fi initialised\n");
    printf("Starting continuous scanning...\n\n");

    bool scan_in_progress = false;
    uint32_t next_scan_ms = 0;
    uint32_t next_print_ms = 0;

    while (true) {
        uint32_t now_ms = (uint32_t)(time_us_64() / 1000);

        if (!scan_in_progress &&
            (int32_t)(now_ms - next_scan_ms) >= 0) {
            cyw43_wifi_scan_options_t scan_options = {0};
            int error = cyw43_wifi_scan(
                &cyw43_state, &scan_options, NULL, scan_result);

            if (error == 0) {
                scan_in_progress = true;
            } else {
                printf("Wi-Fi scan start error: %d\n", error);
                next_scan_ms = now_ms + 500;
            }
        }

        if (scan_in_progress &&
            !cyw43_wifi_scan_active(&cyw43_state)) {
            scan_in_progress = false;
            next_scan_ms = now_ms + 100;
        }

        if ((int32_t)(now_ms - next_print_ms) >= 0) {
            bool ap_valid[AP_COUNT];

            for (int i = 0; i < AP_COUNT; ++i) {
                ap_valid[i] =
                    ap_last_seen_ms[i] != 0 &&
                    (uint32_t)(now_ms - ap_last_seen_ms[i]) <
                        RSSI_TIMEOUT_MS;
            }

            update_output(ap_valid, now_ms);
            print_rssi(ap_valid);
            next_print_ms = now_ms + PRINT_INTERVAL_MS;
        }

        buzzer_service(now_ms);
        sleep_ms(5);
    }
}
