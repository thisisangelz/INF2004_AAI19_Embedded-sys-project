#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "faults.h"
#include "motion.h"
#include "line_search.h"
#include "payload_client.h"
#include "run_report.h"

#define MAX_RUN_MS 900000u
#define MAX_MISSING_SAMPLES 3
#define MAX_PAYLOAD_ATTEMPTS 3
#define START_RELEASE_TIMEOUT_MS 10000u
#define BLUETOOTH_START_TIMEOUT_MS 10000u
// Reserve polling/I2C overrun for every translation block, including returns.
#define BLOCK_RESERVE_MS 25u
#ifndef BUZZER_PIN
#define BUZZER_PIN 22
#endif
#define BUZZER_HZ 2000
#define COMPLETION_TONE_MS 1000

static uint32_t run_started_ms, translation_ms;
static uint16_t buzzer_wrap;
static bool wifi_quiescent = true;
static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static bool run_can_continue(void)
{
    if (motion_stop_requested()) { motion_stop(); return false; }
    if (!report_ok()) { motion_stop(); fault_raise(FAULT_REPORT); return false; }
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
    if (!wifi_quiescent || !wifi_sampler_quiesce()) return false;
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
        if (!wifi_sampler_sample(sample) || !save_checkpoint()) return PAYLOAD_FATAL;
        if (sample->code != RSSI_MISSING_CODE) return PAYLOAD_NOT_CLOSE;
        report_printf("AP1 Wi-Fi missing; checking BLE while remaining stopped\n");
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (!motion_pause_ms(500)) return PAYLOAD_FATAL;
    }
    report_printf("FAILED: AP1 missing from three fresh Wi-Fi samples; no blind movement\n");
    return PAYLOAD_FATAL;
}


static payload_result_t measure_pair(line_pair_t *pair)
{
    payload_result_t result = wait_for_wifi(&pair->first);
    if (result != PAYLOAD_NOT_CLOSE) return result;
    return wait_for_wifi(&pair->second);
}

static uint32_t translation_cost(uint32_t duration)
{
    return duration + ((duration + LINE_BLOCK_MS - 1u) / LINE_BLOCK_MS) * BLOCK_RESERVE_MS;
}

static bool reserve_translation(uint32_t required)
{
    if (translation_ms <= LINE_TRANSLATION_MS && required <= LINE_TRANSLATION_MS - translation_ms)
        return run_can_continue();
    motion_stop();
    fault_raise(FAULT_TRANSLATION_LIMIT);
    report_printf("FAILED: translation budget, used %lu ms, required including return %lu ms, limit %u ms\n",
                  (unsigned long)translation_ms, (unsigned long)required, LINE_TRANSLATION_MS);
    return false;
}

// One powered block, then settle, drain Wi-Fi, persist and check BLE. A fatal
// scan failure keeps wifi_quiescent false until main shuts the radio down.
static payload_result_t translate_block(bool forwards, uint32_t duration, float heading,
                                        uint32_t position, uint32_t reserved_return,
                                        bool acquire_moving, uint32_t *actual)
{
    *actual = 0;
    if (!reserve_translation(translation_cost(duration) + reserved_return) || !save_checkpoint())
        return PAYLOAD_FATAL;
    wifi_sampler_set_position(heading, position, forwards);
    if (!motion_drive_start(forwards, duration, heading)) return PAYLOAD_FATAL;
    bool ok = true;
    wifi_quiescent = false;
    while (motion_drive_active()) {
        if (!motion_drive_update() || !run_can_continue() ||
            (acquire_moving && !wifi_sampler_poll_moving())) { ok = false; break; }
        if (motion_drive_active()) sleep_ms(5);
    }
    motion_stop();
    *actual = motion_drive_elapsed_ms();
    translation_ms += *actual;
    bool finished = motion_drive_finish();
    if (!wifi_sampler_quiesce()) return PAYLOAD_FATAL;
    wifi_quiescent = true;
    if (!ok || !finished || !run_can_continue()) return PAYLOAD_FATAL;
    if (*actual == 0 || *actual > duration + BLOCK_RESERVE_MS || translation_ms > LINE_TRANSLATION_MS) {
        fault_raise(FAULT_TRANSLATION_LIMIT);
        report_printf("FAILED: unexpected powered duration; no further movement\n");
        return PAYLOAD_FATAL;
    }
    return check_payload();
}

static payload_result_t return_along_line(uint32_t from, uint32_t to, float heading)
{
    report_printf("RETURN: approximate reverse from %lu to %lu powered ms, same heading %d\n",
                  (unsigned long)from, (unsigned long)to, (int)heading);
    uint32_t remaining = from - to;
    while (remaining) {
        uint32_t block = remaining > LINE_BLOCK_MS ? LINE_BLOCK_MS : remaining;
        uint32_t actual;
        payload_result_t result = translate_block(false, block, heading, to + remaining,
            translation_cost(remaining - block), false, &actual);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        remaining = actual < remaining ? remaining - actual : 0;
    }
    wifi_sampler_set_position(heading, to, true);
    return PAYLOAD_NOT_CLOSE;
}

typedef struct {
    uint32_t wall_ms, powered_ms, scan_id;
    float heading;
    line_pair_t readings;
} line_point_t;
static line_point_t line_profile[LINE_PROFILE_CAPACITY];
static unsigned profile_count;
static uint32_t profile_id;

static bool record_point(float heading, uint32_t powered, line_pair_t readings)
{
    if (profile_count >= LINE_PROFILE_CAPACITY) {
        motion_stop(); fault_raise(FAULT_PROFILE);
        report_printf("FAILED: line profile buffer full\n");
        return false;
    }
    line_point_t *point = &line_profile[profile_count++];
    *point = (line_point_t){.wall_ms = now_ms(), .powered_ms = powered,
        .scan_id = wifi_sampler_scan_id(), .heading = heading, .readings = readings};
    report_printf("LINE_POINT,line=%lu,point=%u,t_ms=%lu,powered_ms=%lu,heading=%d,scan=%lu,dbm1=%d,dbm2=%d\n",
                  (unsigned long)profile_id, profile_count - 1u, (unsigned long)point->wall_ms,
                  (unsigned long)powered, (int)heading, (unsigned long)point->scan_id,
                  rssi_decode(readings.first.code), rssi_decode(readings.second.code));
    return save_checkpoint();
}

typedef enum { FOLLOW_NO_GAIN, FOLLOW_DECLINE, FOLLOW_STALLED } follow_end_t;
typedef struct {
    float heading;
    bool progress, consistent;
    follow_end_t end;
    uint32_t position;
    line_pair_t baseline;
} follow_result_t;

static payload_result_t follow_direction(float heading, uint32_t trial_ms, follow_result_t *followed)
{
    *followed = (follow_result_t){.heading = line_normalise_heading(heading)};
    profile_count = 0;
    ++profile_id;
    if (!motion_turn_to_heading(heading)) return PAYLOAD_FATAL;
    wifi_sampler_set_position(heading, 0, true);
    line_pair_t best;
    payload_result_t result = measure_pair(&best);
    if (result != PAYLOAD_NOT_CLOSE) return result;
    const line_pair_t baseline = best;
    followed->baseline = baseline;
    if (!record_point(heading, 0, best)) return PAYLOAD_FATAL;
    uint32_t position = 0, best_position = 0, segment_start = 0;
    uint32_t last_decline_position = 0;
    unsigned decline_positions = 0;
    report_printf("FOLLOW: heading %d, blocks at most %u ms; continue on confirmed improvement\n",
                  (int)heading, LINE_FORWARD_BLOCK_MS);
    while (run_can_continue()) {
        uint32_t block = LINE_FORWARD_BLOCK_MS;
        if (!followed->progress && position < trial_ms && block > trial_ms - position)
            block = trial_ms - position;
        wifi_sampler_set_reference(line_reference(best));
        uint32_t actual;
        result = translate_block(true, block, heading, position,
            translation_cost(position + block + BLOCK_RESERVE_MS - best_position), true, &actual);
        position += actual;
        followed->position = position;
        if (result != PAYLOAD_NOT_CLOSE) return result;
        wifi_sampler_set_position(heading, position, true);
        line_pair_t current;
        result = measure_pair(&current);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (!record_point(heading, position, current)) return PAYLOAD_FATAL;
        int margin = line_progress_margin(line_reference(best), line_reference(current));
        int trend = line_compare(best, current);
        report_printf("FOLLOW_POINT,powered_ms=%lu,best_ms=%lu,gain_db=%d,margin_db=%d,baseline_gain_db=%d,baseline_margin_db=%d,trend=%d\n",
                      (unsigned long)position, (unsigned long)best_position,
                      line_confirmed_gain(best, current), margin, line_confirmed_gain(baseline, current),
                      line_progress_margin(line_reference(baseline), line_reference(current)), trend);
        // Once the direction is confirmed, a pair can remain a confirmed gain
        // over its baseline while beating the retained point by less than the
        // noise margin. Keep that stronger two-batch point for a partial return.
        bool stronger = trend > 0 || (followed->progress &&
            line_compare(baseline, current) > 0 && line_confirmed_gain(best, current) > 0);
        if (stronger) {
            if (decline_positions) report_printf("FOLLOW: decline sequence cleared by a new best point\n");
            decline_positions = 0;
            if (!followed->progress)
                report_printf("FOLLOW: first confirmed improvement; continuing without other direction probes\n");
            best = current;
            best_position = position;
            followed->progress = true;
            report_printf("FOLLOW: new confirmed best at %lu powered ms\n", (unsigned long)best_position);
        } else if (trend < 0) {
            if (!decline_positions || position - last_decline_position >= LINE_FORWARD_BLOCK_MS) {
                ++decline_positions;
                last_decline_position = position;
            }
            report_printf("FOLLOW: decline position %u/%u at %lu powered ms; two fresh batches below best\n",
                          decline_positions, LINE_DECLINE_POSITIONS, (unsigned long)position);
            if (decline_positions >= LINE_DECLINE_POSITIONS) {
                followed->end = FOLLOW_DECLINE;
                report_printf("FOLLOW: decline confirmed across %u positions; return only %lu powered ms to best point %lu\n",
                              LINE_DECLINE_POSITIONS,
                              (unsigned long)(position - best_position), (unsigned long)best_position);
                result = return_along_line(position, best_position, heading);
                if (result != PAYLOAD_NOT_CLOSE) return result;
                line_pair_t returned;
                result = measure_pair(&returned);
                if (result != PAYLOAD_NOT_CLOSE) return result;
                followed->consistent = line_return_consistent(best, returned);
                report_printf("RETURN: RSSI %s; timed position remains approximate\n",
                              followed->consistent ? "consistent with best point" : "inconsistent with best point, stopping");
                return PAYLOAD_NOT_CLOSE;
            }
        } else {
            if (decline_positions)
                report_printf("FOLLOW: decline sequence cleared at %lu powered ms; continuing forward\n",
                              (unsigned long)position);
            decline_positions = 0;
        }
        if (!followed->progress && position >= trial_ms && !decline_positions) {
            followed->end = FOLLOW_NO_GAIN;
            report_printf("FOLLOW: no confirmed improvement after %lu powered ms; trial ended at current position\n",
                          (unsigned long)position);
            return PAYLOAD_NOT_CLOSE;
        }
        // If both batches are weaker but the drop is still below the margin,
        // keep sampling for a confirmed decline instead of abandoning the line
        // just before its return decision becomes clear.
        if (followed->progress && position - best_position >= LINE_SWEEP_MS &&
            line_confirmed_gain(current, best) <= 0) {
            followed->end = FOLLOW_STALLED;
            report_printf("FOLLOW: six powered seconds without a new confirmed best; stopping here, no automatic reverse\n");
            return PAYLOAD_NOT_CLOSE;
        }
        if (position - segment_start >= LINE_SWEEP_MS) {
            // Bound each RAM profile while retaining the best point and powered
            // coordinate across intervals. Ending an interval never reverses.
            report_printf("FOLLOW: six-second profile interval complete; continuing same heading, best %lu powered ms\n",
                          (unsigned long)best_position);
            profile_count = 0;
            ++profile_id;
            segment_start = position;
            if (!record_point(heading, position, current)) return PAYLOAD_FATAL;
        }
    }
    return PAYLOAD_FATAL;
}

static payload_result_t check_perpendiculars(float origin, follow_result_t *followed, bool *found)
{
    *found = false;
    report_printf("DIRECTION SEARCH: two perpendicular directions, origin %d degrees; follow first confirmed improvement\n",
                  (int)origin);
    for (unsigned i = 0; i < 2u; ++i) {
        float heading = line_normalise_heading(origin + (i ? 90.0f : -90.0f));
        payload_result_t result = follow_direction(heading, LINE_DIRECTION_TEST_MS, followed);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (followed->progress) {
            *found = true;
            return PAYLOAD_NOT_CLOSE;
        }
        if (followed->end == FOLLOW_NO_GAIN) {
            report_printf("SIDE TRIAL: no confirmed gain; return to its starting best point before checking the other side\n");
            result = return_along_line(followed->position, 0, heading);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            line_pair_t returned;
            result = measure_pair(&returned);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            followed->consistent = line_return_consistent(followed->baseline, returned);
            report_printf("SIDE RETURN: RSSI %s; timed position remains approximate\n",
                          followed->consistent ? "consistent with starting best point" : "inconsistent, stopping");
        }
        // A failed return must stop further movement, including the other side.
        if (!followed->consistent) return PAYLOAD_NOT_CLOSE;
    }
    return PAYLOAD_NOT_CLOSE;
}

static bool approach_beacon(void)
{
    run_started_ms = now_ms();
    translation_ms = profile_id = 0;
    wifi_quiescent = true;
    wifi_sampler_init(run_can_continue);
    payload_result_t result = check_payload();
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    wifi_sample_t initial;
    result = wait_for_wifi(&initial);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL || !motion_learn_direction()) return false;
    follow_result_t followed;
    report_printf("APPROACH: follow current heading; after a confirmed decline return to best and check only perpendicular directions\n");
    result = follow_direction(motion_heading(), LINE_DIRECTION_TEST_MS, &followed);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    while (run_can_continue()) {
        if (followed.end != FOLLOW_DECLINE) {
            fault_raise(FAULT_NO_DIRECTION);
            report_printf("FAILED: current heading ended without a confirmed decline; no further direction search\n");
            return false;
        }
        if (!followed.consistent) {
            fault_raise(FAULT_NO_DIRECTION);
            report_printf("FAILED: return RSSI inconsistent with best point; no further movement\n");
            return false;
        }
        float origin = followed.heading;
        bool found = false;
        result = check_perpendiculars(origin, &followed, &found);
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        if (!found) {
            fault_raise(FAULT_NO_DIRECTION);
            if (!followed.consistent)
                report_printf("FAILED: perpendicular return RSSI inconsistent; no further movement\n");
            else report_printf("FAILED: neither perpendicular direction confirmed improvement\n");
            return false;
        }
    }
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

static bool wait_start_release(void)
{
    report_printf("Release GP20 to continue. Waiting at most 10 seconds; GP21 stops startup.\n");
    uint32_t start = now_ms();
    while (motion_start_pressed()) {
        if (motion_stop_requested()) return false;
        if ((uint32_t)(now_ms() - start) >= START_RELEASE_TIMEOUT_MS) {
            fault_raise(FAULT_START_BUTTON);
            report_printf("FAILED: GP20 still reads pressed after 10 seconds; release the button/check GP20 wiring, then reset\n");
            return false;
        }
        sleep_ms(10);
    }
    return !motion_stop_requested();
}

int main(void)
{
    stdio_init_all();
    report_init();
    bool ready = motion_init();
    buzzer_init();
    sleep_ms(2000);
    report_printf("Getting there | line-search v7 two-directions | AP1 only\n");
    report_printf("Flash layout: 2 MiB physical; 1528 KiB firmware, 8 KiB Bluetooth, 512 KiB saved log\n");
    report_printf("Before GP20: GP21 replays previous run. After GP20: GP21 STOP, latched until reset.\n");
    report_printf("Move using Wi-Fi direction; receive payload only after BLE averages >= -50 dBm\n");
    // GP21 is not armed as STOP here. Boot/replay must not erase the old run,
    // start movement, initialise the radio, or prevent the later GP20 start.
    bool last_replay = false;
    report_printf("Waiting for GP20: press to start; GP21 replays the previous run.\n");
    while (true) {
        report_flush_usb();
        bool replay = motion_replay_pressed();
        if (replay && !last_replay) report_replay();
        last_replay = replay;
        // Give replay precedence if both buttons are pressed.
        if (!replay && motion_start_pressed()) break;
        sleep_ms(20);
    }
    motion_arm_stop();
    bool recording = false;
    if (!motion_stop_requested()) {
        report_printf("GP20 START: preparing persistent log; motors remain stopped.\n");
        recording = report_begin();
        if (!recording) fault_raise(FAULT_REPORT);
    } else report_printf("START CANCELLED: GP21 was pressed; previous log preserved\n");
    bool released = recording && wait_start_release();
    if (released) report_printf("GP20 released. Persistent log ready.\n");
    // An early power-up I2C failure must not permanently reject a sensor which
    // is now ready. Retry once, stopped and recorded, after the explicit start.
    if (released && !ready) {
        report_printf("Retrying IMU initialisation after GP20; motors remain stopped.\n");
        ready = motion_init();
    }
    ready = ready && released && !motion_stop_requested();
    // Calibrate immediately after the new run is safely logged. Radio startup
    // can take several seconds; it should not make the requested calibration
    // appear as if GP20 did nothing.
    if (ready) {
        report_printf("Starting gyro calibration before Wi-Fi/Bluetooth. Keep the robot still.\n");
        ready = motion_calibrate();
        if (!ready) report_printf("FAILED: gyro calibration; check IMU wiring and keep the robot still\n");
    }
    bool radio_initialised = false;
    if (ready) {
        report_printf("Starting Wi-Fi/Bluetooth hardware.\n");
        int radio_error = cyw43_arch_init();
        if (radio_error) {
            fault_raise(FAULT_BLE_RADIO);
            report_printf("FAILED: Wi-Fi/Bluetooth initialisation, error %d\n", radio_error);
            ready = false;
        } else {
            radio_initialised = true;
            report_printf("Wi-Fi hardware initialised; starting Bluetooth.\n");
            cyw43_arch_enable_sta_mode();
            payload_client_init();
            uint32_t start = now_ms();
            uint32_t last_progress = start;
            while (!payload_client_ready() && !payload_client_start_failed() && !motion_stop_requested() &&
                   (uint32_t)(now_ms() - start) < BLUETOOTH_START_TIMEOUT_MS) {
                report_flush_usb();
                if ((uint32_t)(now_ms() - last_progress) >= 1000u) {
                    report_printf("Waiting for Bluetooth: %lu/10000 ms; motors stopped\n",
                                  (unsigned long)(now_ms() - start));
                    last_progress = now_ms();
                }
                sleep_ms(10);
            }
            ready = payload_client_ready() && !payload_client_start_failed() && !motion_stop_requested();
            if (!ready) {
                if (!motion_stop_requested()) fault_raise(FAULT_BLE_RADIO);
                report_printf("FAILED: Bluetooth startup %s\n", motion_stop_requested() ? "stopped by GP21" :
                              payload_client_start_failed() ? "controller power-on failed" : "timed out after 10 seconds");
            }
            else report_printf("Bluetooth ready; beginning AP1 approach.\n");
        }
    }
    // approach_beacon initialises the sampler before its first checkpoint.
    if (ready && !motion_stop_requested()) {
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
    if (recording && !report_finish()) {
        fault_raise(FAULT_REPORT);
        printf("LOG ERROR: saved report is incomplete; no further movement.\n");
    }
    // Never restart automatically. Allow GP21 replay after this run as well.
    last_replay = motion_replay_pressed();
    while (true) {
        report_flush_usb();
        bool replay = motion_replay_pressed();
        if (replay && !last_replay) report_replay();
        last_replay = replay;
        sleep_ms(20);
    }
}
