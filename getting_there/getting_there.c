#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "motion.h"
#include "payload_client.h"
#include "run_report.h"

#define AP1_SSID "I AM PICO W"
#define RSSI_MISSING (-127)
#define SCANS_PER_SAMPLE 5
#define MIN_VALID_SCANS 3
#define SCAN_TIMEOUT_MS 15000
#define PROBE_MOVE_MS 2400
#define APPROACH_STEP_MS 750
#define MAX_APPROACH_STEPS 40
#define MAX_RUN_MS 900000
#define MAX_MISSING_SAMPLES 3
#define MAX_PAYLOAD_ATTEMPTS 3
#ifndef BUZZER_PIN
#define BUZZER_PIN 22
#endif
#define BUZZER_HZ 2000
#define COMPLETION_TONE_MS 1000

static volatile int scan_rssi;
static uint8_t scan_bssid[6];
static uint32_t run_started_ms;
static uint16_t buzzer_wrap;
static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static bool run_can_continue(void)
{
    if (motion_stop_requested()) return false;
    if ((uint32_t)(now_ms() - run_started_ms) >= MAX_RUN_MS) {
        motion_stop();
        report_printf("FAILED: 15 minute run limit reached\n");
        return false;
    }
    return true;
}

static void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 16.0f);
    uint32_t wrap = clock_get_hz(clk_sys) / (16u * BUZZER_HZ) - 1u;
    if (wrap > 65535) wrap = 65535;
    buzzer_wrap = (uint16_t)wrap;
    pwm_config_set_wrap(&config, buzzer_wrap);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

static int wifi_result(void *unused, const cyw43_ev_scan_result_t *result)
{
    (void)unused;
    size_t length = strlen(AP1_SSID);
    if (result && result->ssid_len == length && result->rssi < 0 &&
        result->rssi > scan_rssi && memcmp(result->ssid, AP1_SSID, length) == 0) {
        scan_rssi = result->rssi;
        memcpy(scan_bssid, result->bssid, sizeof(scan_bssid));
    }
    return 0;
}

static bool sample_ap1(int *score)
{
    motion_stop();
    int readings[SCANS_PER_SAMPLE];
    int count = 0;
    for (int round = 0; round < SCANS_PER_SAMPLE; ++round) {
        if (!run_can_continue()) return false;
        scan_rssi = RSSI_MISSING;
        cyw43_wifi_scan_options_t options = {0};
        if (cyw43_wifi_scan(&cyw43_state, &options, NULL, wifi_result) != 0) {
            report_printf("FAILED: Wi-Fi scan could not start\n");
            return false;
        }
        uint32_t start = now_ms();
        while (cyw43_wifi_scan_active(&cyw43_state)) {
            if (!run_can_continue() || (uint32_t)(now_ms() - start) >= SCAN_TIMEOUT_MS) {
                report_printf("FAILED: Wi-Fi scan stopped or timed out\n");
                return false;
            }
            sleep_ms(10);
        }
        if (scan_rssi != RSSI_MISSING) {
            readings[count++] = scan_rssi;
            report_printf("AP1 Wi-Fi: %d dBm, BSSID %02x:%02x:%02x:%02x:%02x:%02x\n",
                          scan_rssi, scan_bssid[0], scan_bssid[1], scan_bssid[2],
                          scan_bssid[3], scan_bssid[4], scan_bssid[5]);
        } else {
            report_printf("AP1 Wi-Fi: not seen\n");
        }
    }
    *score = RSSI_MISSING;
    if (count >= MIN_VALID_SCANS) {
        for (int i = 1; i < count; ++i) {
            int value = readings[i], j = i;
            while (j > 0 && readings[j - 1] > value) { readings[j] = readings[j - 1]; --j; }
            readings[j] = value;
        }
        int trim = count / 4, sum = 0, kept = count - 2 * trim;
        for (int i = trim; i < count - trim; ++i) sum += readings[i];
        *score = (sum - kept / 2) / kept;
    }
    if (*score == RSSI_MISSING) report_printf("AP1 filter: %d/5 valid, no score\n", count);
    else report_printf("AP1 filter: %d/5 valid, %d dBm\n", count, *score);
    return !motion_stop_requested();
}

static payload_result_t check_payload(void)
{
    bool was_close = false;
    for (int attempt = 0; attempt < MAX_PAYLOAD_ATTEMPTS; ++attempt) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        payload_result_t result = payload_client_attempt();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (result == PAYLOAD_NOT_CLOSE && !was_close) return result;
        was_close = true;
        report_printf("AP1 was close but transfer was not confirmed; retry %d/%d without moving\n",
                      attempt + 1, MAX_PAYLOAD_ATTEMPTS);
        if (!motion_pause_ms(200)) return PAYLOAD_FATAL;
    }
    report_printf("FAILED: payload could not be confirmed after three stationary attempts\n");
    return PAYLOAD_FATAL;
}

static payload_result_t wait_for_wifi(int *score)
{
    for (int attempt = 0; attempt < MAX_MISSING_SAMPLES; ++attempt) {
        if (!sample_ap1(score)) return PAYLOAD_FATAL;
        if (*score != RSSI_MISSING) return PAYLOAD_NOT_CLOSE;
        report_printf("AP1 Wi-Fi missing; checking BLE while remaining stopped\n");
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (!motion_pause_ms(500)) return PAYLOAD_FATAL;
    }
    report_printf("FAILED: AP1 missing from three fresh Wi-Fi samples; no blind movement\n");
    return PAYLOAD_FATAL;
}

static int strongest_heading(const int score[4])
{
    int best = -1;
    for (int i = 0; i < 4; ++i) {
        if (score[i] == RSSI_MISSING) continue;
        if (best < 0 || score[i] > score[best]) best = i;
    }
    return best;
}

static payload_result_t choose_direction(float *heading)
{
    float origin = motion_heading();
    int score[4];
    report_printf("DIRECTION PASS: four 2400 ms probes from current position\n");
    for (int i = 0; i < 4; ++i) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        float target = origin + i * 90.0f;
        if (!motion_turn_to_heading(target) || !motion_drive(true, PROBE_MOVE_MS, target))
            return PAYLOAD_FATAL;
        // Stop here if a probe has already reached the transfer range.
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (!sample_ap1(&score[i])) return PAYLOAD_FATAL;
        report_printf("Probe %d degrees from pass origin: %s\n", i * 90,
                      score[i] == RSSI_MISSING ? "no valid RSSI" : "score recorded");
        if (!motion_drive(false, PROBE_MOVE_MS, target)) return PAYLOAD_FATAL;
    }
    if (!motion_turn_to_heading(origin)) return PAYLOAD_FATAL;
    int selected = strongest_heading(score);
    if (selected < 0) {
        report_printf("FAILED: no valid AP1 RSSI at any probe heading\n");
        return PAYLOAD_FATAL;
    }
    *heading = origin + selected * 90.0f;
    report_printf("Selected pass direction %d degrees: %d dBm; highest valid reading; ties ordered by probe\n",
                  selected * 90, score[selected]);
    if (!motion_turn_to_heading(*heading)) return PAYLOAD_FATAL;
    return PAYLOAD_NOT_CLOSE;
}

typedef struct {
    float heading;
    float best_heading;
    float path_heading[MAX_APPROACH_STEPS];
    int path_count;
    int best_path_count;
    int best_rssi;
    int alternative_index;
} navigation_t;

static void record_best(navigation_t *navigation, int score)
{
    if (score < navigation->best_rssi) return;
    navigation->best_rssi = score;
    navigation->best_path_count = navigation->path_count;
    navigation->best_heading = navigation->heading;
    navigation->alternative_index = 0;
    report_printf("BEST POSITION: path step %d, %d dBm, heading %d degrees (timed estimate)\n",
                  navigation->best_path_count, score, (int)navigation->best_heading);
}

static payload_result_t return_to_best(navigation_t *navigation)
{
    report_printf("RETURN TO BEST: reverse %d recorded %lu ms steps to path step %d, %d dBm\n",
                  navigation->path_count - navigation->best_path_count,
                  (unsigned long)APPROACH_STEP_MS,
                  navigation->best_path_count, navigation->best_rssi);
    while (navigation->path_count > navigation->best_path_count) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        float heading = navigation->path_heading[navigation->path_count - 1];
        if (!motion_turn_to_heading(heading) ||
            !motion_drive(false, APPROACH_STEP_MS, heading)) return PAYLOAD_FATAL;
        --navigation->path_count;
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
    }
    if (!run_can_continue() || !motion_turn_to_heading(navigation->best_heading))
        return PAYLOAD_FATAL;
    navigation->heading = navigation->best_heading;
    report_printf("RETURN COMPLETE: estimated best position; actual distance is not measured\n");
    return PAYLOAD_NOT_CLOSE;
}

static payload_result_t try_alternative(navigation_t *navigation)
{
    static const int offsets[] = {15, -15, 30, -30, 45, -45};
    if (navigation->alternative_index >= (int)(sizeof(offsets) / sizeof(offsets[0]))) {
        report_printf("FAILED: six nearby headings tried without matching the best RSSI\n");
        return PAYLOAD_FATAL;
    }
    if (!run_can_continue()) return PAYLOAD_FATAL;
    int offset = offsets[navigation->alternative_index++];
    navigation->heading = navigation->best_heading + offset;
    report_printf("TRY NEARBY HEADING: %+d degrees from best heading, target %d degrees\n",
                  offset, (int)navigation->heading);
    if (!motion_turn_to_heading(navigation->heading)) return PAYLOAD_FATAL;
    payload_result_t result = check_payload();
    if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
    int score;
    result = wait_for_wifi(&score);
    if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
    report_printf("NEARBY HEADING RSSI: %d dBm; retained best %d dBm\n",
                  score, navigation->best_rssi);
    return PAYLOAD_NOT_CLOSE;
}

static bool approach_beacon(void)
{
    run_started_ms = now_ms();
    if (!motion_calibrate()) return false;
    payload_result_t result = check_payload();
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    int score;
    result = wait_for_wifi(&score);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL || !motion_learn_direction()) return false;

    navigation_t navigation = {0};
    result = choose_direction(&navigation.heading);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    // Probe endpoints have already been left. Save the actual starting position.
    result = wait_for_wifi(&score);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    navigation.best_rssi = RSSI_MISSING;
    record_best(&navigation, score);
    for (int step = 0; step < MAX_APPROACH_STEPS; ++step) {
        if (!run_can_continue()) return false;
        result = check_payload();
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        report_printf("APPROACH STEP %d/%d: %lu ms towards heading %d degrees\n",
                      step + 1, MAX_APPROACH_STEPS,
                      (unsigned long)APPROACH_STEP_MS, (int)navigation.heading);
        if (!motion_drive(true, APPROACH_STEP_MS, navigation.heading)) return false;
        navigation.path_heading[navigation.path_count++] = navigation.heading;
        // Check BLE before any further movement, including a return movement.
        result = check_payload();
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL || !sample_ap1(&score)) return false;
        if (score == RSSI_MISSING) {
            report_printf("FORWARD SIGNAL LOST: remain stopped and retry detection\n");
            result = wait_for_wifi(&score);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
        }
        report_printf("FORWARD RSSI: %d dBm, best %d dBm\n", score, navigation.best_rssi);
        if (score < navigation.best_rssi) {
            report_printf("SIGNAL DECREASED: confirm while stopped; no further forward step\n");
            result = wait_for_wifi(&score);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            if (score < navigation.best_rssi) {
                report_printf("SIGNAL DECREASE CONFIRMED: %d dBm below recorded best %d dBm\n",
                              score, navigation.best_rssi);
                result = return_to_best(&navigation);
                if (result == PAYLOAD_COMPLETE) return true;
                if (result == PAYLOAD_FATAL) return false;
                result = try_alternative(&navigation);
                if (result == PAYLOAD_COMPLETE) return true;
                if (result == PAYLOAD_FATAL) return false;
                continue;
            }
            report_printf("SIGNAL RECOVERED: keep the current heading\n");
        }
        record_best(&navigation, score);
        report_printf("KEEP HEADING: continue forwards after radio checks\n");
    }
    report_printf("FAILED: %d approach steps without a confirmed payload\n", MAX_APPROACH_STEPS);
    return false;
}

static void completion_sound(void)
{
    motion_stop();
    if (motion_stop_requested()) {
        report_printf("Completion tone cancelled by GP21\n");
        return;
    }
    report_printf("COMPLETION TONE: GP%d, 2000 Hz for 1000 ms\n", BUZZER_PIN);
    pwm_set_gpio_level(BUZZER_PIN, buzzer_wrap / 2);
    bool finished = motion_pause_ms(COMPLETION_TONE_MS);
    pwm_set_gpio_level(BUZZER_PIN, 0);
    report_printf("Completion tone %s; motors stopped\n", finished ? "finished" : "stopped by GP21");
}

int main(void)
{
    stdio_init_all();
    report_init();
    bool ready = motion_init();
    buzzer_init();
    sleep_ms(2000);
    report_printf("Getting there | AP1 only | GP20 start | GP21 stop and latch until reset\n");
    report_printf("Move using Wi-Fi direction; receive payload only after BLE averages >= -50 dBm\n");
    if (ready) {
        if (cyw43_arch_init()) {
            report_printf("FAILED: Wi-Fi/Bluetooth initialisation\n");
            ready = false;
        } else {
            cyw43_arch_enable_sta_mode();
            payload_client_init();
            uint32_t start = now_ms();
            while (!payload_client_ready() && !motion_stop_requested() &&
                   (uint32_t)(now_ms() - start) < 10000) sleep_ms(10);
            ready = payload_client_ready() && !motion_stop_requested();
            if (!ready) report_printf("FAILED: Bluetooth did not become ready or GP21 pressed\n");
        }
    }
    if (ready) {
        report_printf("Waiting for GP20. Keep battery on after the run to read the stored report.\n");
        while (!motion_start_pressed() && !motion_stop_requested()) sleep_ms(10);
        while (motion_start_pressed() && !motion_stop_requested()) sleep_ms(10);
        if (!motion_stop_requested()) {
            bool completed = approach_beacon();
            motion_stop();
            if (completed) completion_sound();
            report_printf("RESULT: %s\n", completed ? "PAYLOAD CONFIRMED" : "STOPPED WITHOUT PAYLOAD");
        }
    }
    motion_stop();
    pwm_set_gpio_level(BUZZER_PIN, 0);
    if (motion_stop_requested()) report_printf("GP21 stop latched\n");
    report_printf("Run ended. Connect Pico USB to read the report. Reset erases RAM.\n");
    bool was_connected = false;
    bool last_start = false;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !was_connected) { sleep_ms(500); report_replay(); }
        bool start = motion_start_pressed();
        if (connected && start && !last_start) report_replay();
        was_connected = stdio_usb_connected();
        last_start = start;
        sleep_ms(20);
    }
}
