#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "faults.h"
#include "motion.h"
#include "packed_bits.h"
#include "payload_client.h"
#include "run_report.h"

#define AP1_SSID "I AM PICO W"
#define RSSI_MISSING rssi_decode(RSSI_MISSING_CODE)
#define SCANS_PER_SAMPLE 5
#define MIN_VALID_SCANS 3
#define SCAN_TIMEOUT_MS 15000
#define APPROACH_STEP_MS 750
#define MAX_TRIAL_SEGMENTS 3
#define MAX_REJECTED_DIRECTIONS 4
#define MIN_PROGRESS_DB 3
#define MAX_APPROACH_STEPS 40
#define MAX_RUN_MS 900000
#define MAX_MISSING_SAMPLES 3
#define MAX_PAYLOAD_ATTEMPTS 3
#ifndef BUZZER_PIN
#define BUZZER_PIN 22
#endif
#define BUZZER_HZ 2000
#define COMPLETION_TONE_MS 1000

// Written by the Wi-Fi scan callback (async context), read by the main loop only
// after the scan has ended. One 7-bit magnitude; see packed_bits.h.
static volatile struct {
    uint8_t rssi_code : RSSI_BITS;
} scan_result;
static uint8_t scan_bssid[6];
static uint8_t target_bssid[6];
static struct { uint8_t locked : 1; } target;
// Both fields have 127 possible codes: valid RSSI -126..-1 plus missing,
// and a within-sample spread of 0..125 dB.
typedef struct {
    uint16_t code : RSSI_BITS;
    uint16_t spread : 7;
} wifi_sample_t;
static uint32_t run_started_ms;
static uint16_t buzzer_wrap;
static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static bool run_can_continue(void)
{
    if (motion_stop_requested()) return false;
    if (!report_ok()) {
        motion_stop();
        fault_raise(FAULT_REPORT);
        return false;
    }
    if ((uint32_t)(now_ms() - run_started_ms) >= MAX_RUN_MS) {
        motion_stop();
        fault_raise(FAULT_RUN_LIMIT);
        report_printf("FAILED: 15 minute run limit reached\n");
        return false;
    }
    return true;
}

static bool save_checkpoint(void)
{
    motion_stop();
    if (report_checkpoint()) return true;
    fault_raise(FAULT_REPORT);
    return false;
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
        result->rssi > rssi_decode(scan_result.rssi_code) &&
        (!target.locked || memcmp(result->bssid, target_bssid, sizeof(target_bssid)) == 0) &&
        memcmp(result->ssid, AP1_SSID, length) == 0) {
        scan_result.rssi_code = rssi_encode(result->rssi);
        memcpy(scan_bssid, result->bssid, sizeof(scan_bssid));
    }
    return 0;
}

static bool sample_ap1(wifi_sample_t *sample)
{
    motion_stop();
    uint8_t readings[BITARRAY_BYTES(SCANS_PER_SAMPLE, RSSI_BITS)] = {0};
    int count = 0;
    for (int round = 0; round < SCANS_PER_SAMPLE; ++round) {
        if (!run_can_continue()) return false;
        scan_result.rssi_code = RSSI_MISSING_CODE;
        cyw43_wifi_scan_options_t options = {0};
        if (cyw43_wifi_scan(&cyw43_state, &options, NULL, wifi_result) != 0) {
            fault_raise(FAULT_WIFI_SCAN);
            report_printf("FAILED: Wi-Fi scan could not start\n");
            return false;
        }
        uint32_t start = now_ms();
        while (cyw43_wifi_scan_active(&cyw43_state)) {
            if (!run_can_continue() || (uint32_t)(now_ms() - start) >= SCAN_TIMEOUT_MS) {
                fault_raise(FAULT_WIFI_SCAN);
                report_printf("FAILED: Wi-Fi scan stopped or timed out\n");
                return false;
            }
            sleep_ms(10);
        }
        if (scan_result.rssi_code != RSSI_MISSING_CODE) {
            bitarray_put(readings, count++, RSSI_BITS, scan_result.rssi_code);
            if (!target.locked) {
                memcpy(target_bssid, scan_bssid, sizeof(target_bssid));
                target.locked = true;
                report_printf("AP1 identity locked to first matching Wi-Fi BSSID\n");
            }
            report_printf("AP1 Wi-Fi: %d dBm, BSSID %02x:%02x:%02x:%02x:%02x:%02x\n",
                          rssi_decode(scan_result.rssi_code), scan_bssid[0], scan_bssid[1], scan_bssid[2],
                          scan_bssid[3], scan_bssid[4], scan_bssid[5]);
        } else {
            report_printf("AP1 Wi-Fi: not seen\n");
        }
        // The scan has completed; persist while stationary, never in its callback.
        if (!save_checkpoint()) return false;
    }
    *sample = (wifi_sample_t){.code = RSSI_MISSING_CODE};
    if (count >= MIN_VALID_SCANS) {
        for (int i = 1; i < count; ++i) {
            unsigned value = bitarray_get(readings, i, RSSI_BITS);
            int j = i;
            while (j > 0 && bitarray_get(readings, j - 1, RSSI_BITS) > value) {
                bitarray_put(readings, j, RSSI_BITS, bitarray_get(readings, j - 1, RSSI_BITS));
                --j;
            }
            bitarray_put(readings, j, RSSI_BITS, value);
        }
        int trim = count / 4, sum = 0, kept = count - 2 * trim;
        for (int i = trim; i < count - trim; ++i)
            sum += bitarray_get(readings, i, RSSI_BITS);
        sample->code = (sum + kept / 2) / kept;
        sample->spread = bitarray_get(readings, count - trim - 1, RSSI_BITS) -
                         bitarray_get(readings, trim, RSSI_BITS);
    }
    if (sample->code == RSSI_MISSING_CODE) fault_raise(FAULT_WIFI_LOST);
    else fault_clear(FAULT_WIFI_LOST);
    if (sample->code == RSSI_MISSING_CODE) report_printf("AP1 filter: %d/5 valid, no score\n", count);
    else report_printf("AP1 filter: %d/5 valid, %d dBm, retained spread %u dB\n",
                       count, rssi_decode(sample->code), (unsigned)sample->spread);
    return save_checkpoint() && !motion_stop_requested();
}

static payload_result_t check_payload(void)
{
    bool was_close = false;
    for (int attempt = 0; attempt < MAX_PAYLOAD_ATTEMPTS; ++attempt) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        if (!save_checkpoint()) return PAYLOAD_FATAL;
        payload_result_t result = payload_client_attempt();
        // A fatal cleanup failure may leave the radio active. Persist its tail
        // only after main shuts the radio down.
        if (result != PAYLOAD_FATAL && !save_checkpoint()) return PAYLOAD_FATAL;
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

static payload_result_t wait_for_wifi(wifi_sample_t *sample)
{
    for (int attempt = 0; attempt < MAX_MISSING_SAMPLES; ++attempt) {
        if (!sample_ap1(sample)) return PAYLOAD_FATAL;
        if (sample->code != RSSI_MISSING_CODE) return PAYLOAD_NOT_CLOSE;
        report_printf("AP1 Wi-Fi missing; checking BLE while remaining stopped\n");
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (!motion_pause_ms(500)) return PAYLOAD_FATAL;
    }
    report_printf("FAILED: AP1 missing from three fresh Wi-Fi samples; no blind movement\n");
    return PAYLOAD_FATAL;
}

// A conservative heuristic, not a statistical confidence interval. Require
// more than the observed retained spread, and at least MIN_PROGRESS_DB.
static int progress_margin(wifi_sample_t baseline, wifi_sample_t current)
{
    int margin = baseline.spread + current.spread + 1;
    return margin > MIN_PROGRESS_DB ? margin : MIN_PROGRESS_DB;
}

static int progress_direction(wifi_sample_t baseline, wifi_sample_t current)
{
    if (baseline.code == RSSI_MISSING_CODE || current.code == RSSI_MISSING_CODE) return 0;
    int delta = (int)baseline.code - (int)current.code;
    int margin = progress_margin(baseline, current);
    return delta >= margin ? 1 : delta <= -margin ? -1 : 0;
}

static payload_result_t retreat_trial(unsigned segments, float heading)
{
    report_printf("REJECT DIRECTION: approximate timed retreat, %u x %u ms; not position feedback\n",
                  segments, APPROACH_STEP_MS);
    while (segments--) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        if (!save_checkpoint()) return PAYLOAD_FATAL;
        if (!motion_drive(false, APPROACH_STEP_MS, heading))
            return PAYLOAD_FATAL;
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
    }
    return PAYLOAD_NOT_CLOSE;
}

static bool approach_beacon(void)
{
    run_started_ms = now_ms();
    target.locked = false;
    if (!motion_calibrate()) return false;
    payload_result_t result = check_payload();
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    wifi_sample_t baseline;
    result = wait_for_wifi(&baseline);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL || !motion_learn_direction()) return false;

    float heading = motion_heading();
    // Counter widths include their terminal values. Every forward segment,
    // including exploratory trials, consumes the global movement budget.
    struct {
        uint16_t step : 6;             // 0..MAX_APPROACH_STEPS
        uint16_t trial_segments : 2;   // 0..MAX_TRIAL_SEGMENTS
        uint16_t rejected : 3;         // 0..MAX_REJECTED_DIRECTIONS
        uint16_t needs_baseline : 1;
    } loop = {.needs_baseline = true};
    _Static_assert(6 == BITS_NEEDED(MAX_APPROACH_STEPS), "step width");
    _Static_assert(2 == BITS_NEEDED(MAX_TRIAL_SEGMENTS), "trial segment width");
    _Static_assert(3 == BITS_NEEDED(MAX_REJECTED_DIRECTIONS), "rejected direction width");
    for (loop.step = 0; loop.step < MAX_APPROACH_STEPS; ++loop.step) {
        if (!run_can_continue()) return false;
        if (loop.needs_baseline) {
            if (!motion_turn_to_heading(heading)) return false;
            result = check_payload();
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            // Take a new baseline AFTER turning. Comparing different antenna
            // orientations or assuming the retreat restored position biases RSSI.
            result = wait_for_wifi(&baseline);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            loop.needs_baseline = false;
            report_printf("DIRECTION TRIAL: heading %d, baseline %d dBm, up to %u x %u ms\n",
                          (int)heading, rssi_decode(baseline.code),
                          MAX_TRIAL_SEGMENTS, APPROACH_STEP_MS);
        }
        result = check_payload();
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        report_printf("APPROACH STEP %d/%d: %u ms, heading %d, trial segment %u/%u\n",
                      loop.step + 1, MAX_APPROACH_STEPS, APPROACH_STEP_MS, (int)heading,
                      (unsigned)loop.trial_segments + 1, MAX_TRIAL_SEGMENTS);
        if (!save_checkpoint() || !run_can_continue()) return false;
        if (!motion_drive(true, APPROACH_STEP_MS, heading)) return false;
        ++loop.trial_segments;
        // Test BLE before taking another Wi-Fi sample or moving again.
        result = check_payload();
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        wifi_sample_t current;
        result = wait_for_wifi(&current);
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        int trend = progress_direction(baseline, current);
        report_printf("PROGRESS: heading %d, base %d, now %d dBm, delta %+d dB, margin %d dB, travel %u ms\n",
                      (int)heading, rssi_decode(baseline.code), rssi_decode(current.code),
                      (int)baseline.code - (int)current.code, progress_margin(baseline, current),
                      (unsigned)loop.trial_segments * APPROACH_STEP_MS);
        if (trend != 0) {
            // A second, fresh stationary batch must agree before accepting a
            // direction or undoing it. An isolated spike is not progress.
            wifi_sample_t confirmation;
            result = wait_for_wifi(&confirmation);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            if (progress_direction(baseline, confirmation) != trend) {
                report_printf("PROGRESS UNCONFIRMED: fresh stationary sample disagrees\n");
                trend = 0;
            } else {
                current = confirmation;
            }
        }
        if (trend > 0) {
            report_printf("PROGRESS CONFIRMED: keep heading; establish new progress baseline\n");
            baseline = current;
            loop.trial_segments = 0;
            loop.rejected = 0;
            continue;
        }
        if (trend == 0 && loop.trial_segments < MAX_TRIAL_SEGMENTS) {
            report_printf("PROGRESS UNCERTAIN: extend the measured trial by one short segment\n");
            continue;
        }
        report_printf("REPLAN: %s\n", trend < 0 ? "signal confirmed weaker" :
                      "no clear improvement within bounded trial");
        result = retreat_trial(loop.trial_segments, heading);
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        loop.trial_segments = 0;
        if (++loop.rejected >= MAX_REJECTED_DIRECTIONS) {
            fault_raise(FAULT_NO_DIRECTION);
            report_printf("FAILED: four directions without confirmed progress; stop for calibration/repositioning\n");
            return false;
        }
        heading += 90.0f;
        if (heading >= 360.0f) heading -= 360.0f;
        loop.needs_baseline = true;
    }
    fault_raise(FAULT_STEP_LIMIT);
    report_printf("FAILED: 40 approach steps without a confirmed payload\n");
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

static void report_faults(void)
{
    report_printf("FAULT MASK: 0x%03x\n", fault_mask());
    for (int f = 0; f < FAULT_COUNT; ++f)
        if (fault_test((fault_t)f)) report_printf("FAULT: %s\n", fault_name((fault_t)f));
}

int main(void)
{
    stdio_init_all();
    report_init();
    bool ready = motion_init();
    buzzer_init();
    sleep_ms(2000);
    report_printf("Getting there | Alvin_2 progress-v2 flash-log | AP1 only\n");
    report_printf("Before GP20: GP21 replays previous run. After GP20: GP21 STOP, latched until reset.\n");
    report_printf("Move using Wi-Fi direction; receive payload only after BLE averages >= -50 dBm\n");
    // GP21 is not armed as STOP here. Boot/replay must not erase the old run,
    // start movement, initialise the radio, or prevent the later GP20 start.
    bool last_replay = false;
    while (true) {
        bool replay = motion_replay_pressed();
        if (replay && !last_replay) report_replay();
        last_replay = replay;
        // Give replay precedence if both buttons are pressed.
        if (!replay && motion_start_pressed()) break;
        sleep_ms(20);
    }
    motion_arm_stop();
    report_printf("GP20 START: preparing persistent log; motors remain stopped.\n");
    bool recording = report_begin();
    if (!recording) { fault_raise(FAULT_REPORT); ready = false; }
    while (motion_start_pressed() && !motion_stop_requested()) sleep_ms(10);
    ready = ready && !motion_stop_requested();
    report_printf("GP20 released. Persistent log %s.\n", recording ? "ready" : "FAILED");
    bool radio_initialised = false;
    if (ready) {
        report_printf("Starting Wi-Fi/Bluetooth hardware.\n");
        if (cyw43_arch_init()) {
            report_printf("FAILED: Wi-Fi/Bluetooth initialisation\n");
            ready = false;
        } else {
            radio_initialised = true;
            report_printf("Wi-Fi hardware initialised; starting Bluetooth.\n");
            cyw43_arch_enable_sta_mode();
            payload_client_init();
            uint32_t start = now_ms();
            while (!payload_client_ready() && !motion_stop_requested() &&
                   (uint32_t)(now_ms() - start) < 10000) sleep_ms(10);
            ready = payload_client_ready() && !motion_stop_requested();
            if (!ready) report_printf("FAILED: Bluetooth did not become ready or GP21 pressed\n");
            else report_printf("Bluetooth ready; beginning AP1 approach.\n");
        }
    }
    if (ready && save_checkpoint() && !motion_stop_requested()) {
        bool completed = approach_beacon();
        motion_stop();
        if (completed) completion_sound();
        report_printf("RESULT: %s\n", completed ? "PAYLOAD CONFIRMED" : "STOPPED WITHOUT PAYLOAD");
    } else {
        report_printf("RESULT: STARTUP FAILED OR STOPPED\n");
    }
    motion_stop();
    pwm_set_gpio_level(BUZZER_PIN, 0);
    // Also quiesces any scan/cleanup still pending after a fatal radio failure.
    if (radio_initialised) cyw43_arch_deinit();
    report_faults();
    if (motion_stop_requested()) report_printf("GP21 stop latched\n");
    report_printf("Run ended. Log survives power-off. Next boot: open serial, press GP21 BEFORE GP20.\n");
    if (!report_finish()) {
        fault_raise(FAULT_REPORT);
        printf("LOG ERROR: saved report is incomplete; no further movement.\n");
    }
    // Never restart automatically. Allow GP21 replay after this run as well.
    last_replay = motion_replay_pressed();
    while (true) {
        bool replay = motion_replay_pressed();
        if (replay && !last_replay) report_replay();
        last_replay = replay;
        sleep_ms(20);
    }
}
