#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ---------------------------------------------------------
// ACCESS POINT NAMES
// ---------------------------------------------------------

#define AP1_SSID "I AM PICO W"
#define AP2_SSID "I AM PICO W 2"

// ---------------------------------------------------------
// MAKER PI PICO PINS
// ---------------------------------------------------------

#define AP1_LED_PIN 2
#define AP2_LED_PIN 3
#define BUZZER_PIN 18

// ---------------------------------------------------------
// SETTINGS
// ---------------------------------------------------------

#define PRINT_INTERVAL_MS 200

// If an AP has not been detected for this long,
// treat it as unavailable.
#define RSSI_TIMEOUT_MS 5000

// Prevent the LEDs rapidly switching when both APs
// have almost identical RSSI values.
#define SWITCH_HYSTERESIS_DB 3

// ---------------------------------------------------------
// RSSI STORAGE
// ---------------------------------------------------------

volatile int ap1_rssi = -127;
volatile int ap2_rssi = -127;

volatile uint32_t ap1_last_seen_ms = 0;
volatile uint32_t ap2_last_seen_ms = 0;

uint buzzer_slice;

// 0 = neither
// 1 = AP1
// 2 = AP2
int selected_ap = 0;


// ---------------------------------------------------------
// CHECK WHETHER SCAN RESULT MATCHES AN SSID
// ---------------------------------------------------------

bool ssid_matches(
    const cyw43_ev_scan_result_t *result,
    const char *target
)
{
    size_t target_length = strlen(target);

    if (result->ssid_len != target_length) {
        return false;
    }

    return memcmp(
        result->ssid,
        target,
        target_length
    ) == 0;
}


// ---------------------------------------------------------
// WIFI SCAN CALLBACK
// ---------------------------------------------------------

static int scan_result(
    void *env,
    const cyw43_ev_scan_result_t *result
)
{
    if (result == NULL) {
        return 0;
    }

    uint32_t now_ms =
        (uint32_t)(time_us_64() / 1000);

    // Check AP1
    if (ssid_matches(result, AP1_SSID)) {

        ap1_rssi = result->rssi;
        ap1_last_seen_ms = now_ms;
    }

    // Check AP2
    else if (ssid_matches(result, AP2_SSID)) {

        ap2_rssi = result->rssi;
        ap2_last_seen_ms = now_ms;
    }

    return 0;
}


// ---------------------------------------------------------
// INITIALISE LEDs
// ---------------------------------------------------------

void leds_init()
{
    gpio_init(AP1_LED_PIN);
    gpio_set_dir(AP1_LED_PIN, GPIO_OUT);

    gpio_init(AP2_LED_PIN);
    gpio_set_dir(AP2_LED_PIN, GPIO_OUT);

    gpio_put(AP1_LED_PIN, 0);
    gpio_put(AP2_LED_PIN, 0);
}


// ---------------------------------------------------------
// INITIALISE BUZZER
// ---------------------------------------------------------

void buzzer_init()
{
    gpio_set_function(
        BUZZER_PIN,
        GPIO_FUNC_PWM
    );

    buzzer_slice =
        pwm_gpio_to_slice_num(BUZZER_PIN);

    pwm_config config =
        pwm_get_default_config();

    // Divide system clock so useful audible frequencies
    // fit inside the 16-bit PWM counter.
    pwm_config_set_clkdiv(
        &config,
        16.0f
    );

    pwm_init(
        buzzer_slice,
        &config,
        true
    );

    // Initially silent
    pwm_set_gpio_level(
        BUZZER_PIN,
        0
    );
}


// ---------------------------------------------------------
// SET BUZZER FREQUENCY
// ---------------------------------------------------------

void buzzer_set_frequency(uint32_t frequency_hz)
{
    if (frequency_hz == 0) {

        pwm_set_gpio_level(
            BUZZER_PIN,
            0
        );

        return;
    }

    uint32_t clock_hz =
        clock_get_hz(clk_sys);

    float divider = 16.0f;

    uint32_t wrap =
        (uint32_t)(
            clock_hz /
            (divider * frequency_hz)
        ) - 1;

    if (wrap > 65535) {
        wrap = 65535;
    }

    if (wrap < 2) {
        wrap = 2;
    }

    pwm_set_wrap(
        buzzer_slice,
        wrap
    );

    // 50% duty cycle
    pwm_set_gpio_level(
        BUZZER_PIN,
        wrap / 2
    );
}


// ---------------------------------------------------------
// CONVERT RSSI INTO BUZZER FREQUENCY
// ---------------------------------------------------------

uint32_t rssi_to_frequency(int rssi)
{
    /*
        RSSI examples:

        -90 dBm = very weak
        -70 dBm = weak
        -50 dBm = good
        -30 dBm = very strong

        Map:

        -90 dBm -> 300 Hz
        -30 dBm -> 3000 Hz
    */

    if (rssi < -90) {
        rssi = -90;
    }

    if (rssi > -30) {
        rssi = -30;
    }

    int range =
        rssi + 90;

    uint32_t frequency =
        300 +
        ((uint32_t)range * 2700) / 60;

    return frequency;
}


// ---------------------------------------------------------
// UPDATE WHICH ACCESS POINT IS CLOSER
// ---------------------------------------------------------

void update_output(
    bool ap1_valid,
    bool ap2_valid
)
{
    // Neither AP found
    if (!ap1_valid && !ap2_valid) {

        selected_ap = 0;

        gpio_put(AP1_LED_PIN, 0);
        gpio_put(AP2_LED_PIN, 0);

        buzzer_set_frequency(0);

        return;
    }

    // Only AP1 found
    if (ap1_valid && !ap2_valid) {

        selected_ap = 1;
    }

    // Only AP2 found
    else if (!ap1_valid && ap2_valid) {

        selected_ap = 2;
    }

    // Both found
    else {

        // Initial choice
        if (selected_ap == 0) {

            if (ap1_rssi >= ap2_rssi) {
                selected_ap = 1;
            }
            else {
                selected_ap = 2;
            }
        }

        // Currently following AP1
        else if (selected_ap == 1) {

            if (
                ap2_rssi >
                ap1_rssi + SWITCH_HYSTERESIS_DB
            ) {
                selected_ap = 2;
            }
        }

        // Currently following AP2
        else if (selected_ap == 2) {

            if (
                ap1_rssi >
                ap2_rssi + SWITCH_HYSTERESIS_DB
            ) {
                selected_ap = 1;
            }
        }
    }


    // -----------------------------------------------------
    // LEDs + buzzer
    // -----------------------------------------------------

    if (selected_ap == 1) {

        gpio_put(AP1_LED_PIN, 1);
        gpio_put(AP2_LED_PIN, 0);

        buzzer_set_frequency(
            rssi_to_frequency(ap1_rssi)
        );
    }

    else if (selected_ap == 2) {

        gpio_put(AP1_LED_PIN, 0);
        gpio_put(AP2_LED_PIN, 1);

        buzzer_set_frequency(
            rssi_to_frequency(ap2_rssi)
        );
    }
}


// ---------------------------------------------------------
// PRINT RSSI VALUES
// ---------------------------------------------------------

void print_rssi(
    bool ap1_valid,
    bool ap2_valid
)
{
    if (ap1_valid && ap2_valid) {

        int selected_rssi =
            (selected_ap == 1)
            ? ap1_rssi
            : ap2_rssi;

        uint32_t frequency =
            rssi_to_frequency(selected_rssi);

        printf(
            "AP1: %d dBm | "
            "AP2: %d dBm | "
            "Closer: AP%d | "
            "Buzzer: %lu Hz\n",
            ap1_rssi,
            ap2_rssi,
            selected_ap,
            (unsigned long)frequency
        );
    }

    else if (ap1_valid) {

        printf(
            "AP1: %d dBm | "
            "AP2: N/A | "
            "Closer: AP1 | "
            "Buzzer: %lu Hz\n",
            ap1_rssi,
            (unsigned long)
                rssi_to_frequency(ap1_rssi)
        );
    }

    else if (ap2_valid) {

        printf(
            "AP1: N/A | "
            "AP2: %d dBm | "
            "Closer: AP2 | "
            "Buzzer: %lu Hz\n",
            ap2_rssi,
            (unsigned long)
                rssi_to_frequency(ap2_rssi)
        );
    }

    else {

        printf(
            "AP1: N/A | "
            "AP2: N/A | "
            "Closer: NONE | "
            "Buzzer: OFF\n"
        );
    }
}


// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------

int main()
{
    stdio_init_all();

    leds_init();
    buzzer_init();

    // Give USB serial some time to initialise
    sleep_ms(2000);

    printf("\n");
    printf("==============================\n");
    printf(" Pico W RSSI Beacon Tracker\n");
    printf("==============================\n");
    printf("AP1: %s\n", AP1_SSID);
    printf("AP2: %s\n", AP2_SSID);
    printf("\n");

    // Initialise Wi-Fi
    if (cyw43_arch_init()) {

        printf(
            "ERROR: Failed to initialise Wi-Fi\n"
        );

        return 1;
    }

    // We are scanning for access points,
    // so this Pico operates as a station.
    cyw43_arch_enable_sta_mode();

    printf("Wi-Fi initialised\n");
    printf("Starting continuous scanning...\n\n");


    bool scan_in_progress = false;

    uint32_t next_scan_ms = 0;
    uint32_t next_print_ms = 0;


    while (true) {

        uint32_t now_ms =
            (uint32_t)(time_us_64() / 1000);


        // -------------------------------------------------
        // START A NEW WIFI SCAN
        // -------------------------------------------------

        if (
            !scan_in_progress &&
            (int32_t)(now_ms - next_scan_ms) >= 0
        ) {

            cyw43_wifi_scan_options_t
                scan_options = {0};

            int error =
                cyw43_wifi_scan(
                    &cyw43_state,
                    &scan_options,
                    NULL,
                    scan_result
                );

            if (error == 0) {

                scan_in_progress = true;
            }

            else {

                printf(
                    "Wi-Fi scan start error: %d\n",
                    error
                );

                next_scan_ms =
                    now_ms + 500;
            }
        }


        // -------------------------------------------------
        // CHECK WHETHER SCAN FINISHED
        // -------------------------------------------------

        if (
            scan_in_progress &&
            !cyw43_wifi_scan_active(
                &cyw43_state
            )
        ) {

            scan_in_progress = false;

            // Start another scan shortly afterwards
            next_scan_ms =
                now_ms + 100;
        }


        // -------------------------------------------------
        // PRINT EVERY 200 ms
        // -------------------------------------------------

        if (
            (int32_t)(
                now_ms - next_print_ms
            ) >= 0
        ) {

            bool ap1_valid =
                ap1_last_seen_ms != 0 &&
                (uint32_t)(
                    now_ms -
                    ap1_last_seen_ms
                ) < RSSI_TIMEOUT_MS;

            bool ap2_valid =
                ap2_last_seen_ms != 0 &&
                (uint32_t)(
                    now_ms -
                    ap2_last_seen_ms
                ) < RSSI_TIMEOUT_MS;


            update_output(
                ap1_valid,
                ap2_valid
            );

            print_rssi(
                ap1_valid,
                ap2_valid
            );


            next_print_ms =
                now_ms +
                PRINT_INTERVAL_MS;
        }


        // Small delay so we are not continuously
        // consuming CPU time.
        sleep_ms(5);
    }


    cyw43_arch_deinit();

    return 0;
}