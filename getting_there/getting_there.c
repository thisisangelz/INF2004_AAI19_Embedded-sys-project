#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "colour_sensor.h"
#include "faults.h"
#include "motion.h"
#include "line_search.h"
#include "payload_client.h"
#include "robot_cues.h"
#include "run_report.h"

#define MAX_RUN_MS 900000u
#define MAX_MISSING_SAMPLES 3
#define MAX_PAYLOAD_ATTEMPTS 3
#define START_RELEASE_TIMEOUT_MS 10000u
#define BLUETOOTH_START_TIMEOUT_MS 10000u
// Reserve polling/I2C overrun for every translation block, including returns.
#define BLOCK_RESERVE_MS 25u

static uint32_t run_started_ms, translation_ms;
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

// Set while the in-range beeps play; they last until the BLE attempt ends.
static bool in_range_cue_on;

static void on_in_range(void)
{
    in_range_cue_on = true;
    cue_play(CUE_IN_RANGE);
}

// Set when the latest check_payload passed colour arrival (ring 4 or darker).
static bool colour_arrived;

static payload_result_t check_payload(void)
{
    bool was_close = false;
    colour_arrived = false;
    for (int attempt = 0; attempt < MAX_PAYLOAD_ATTEMPTS; ++attempt) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        if (!save_checkpoint()) return PAYLOAD_FATAL;
        if (!was_close) {
            // Motors are stopped here. On the mat, the colour sensor decides
            // arrival and BLE only has to show AP1 is in range.
            bool on_mat = colour_arrival_check();
            colour_arrived = on_mat;
            // With a working colour sensor, only the mat allows a transfer:
            // the -58 dBm BLE gate alone passed on ring 1 when the robot
            // drove along the rings (7 November side-on run). This also saves
            // the 5 s BLE check at every stop before the mat. Without the
            // sensor (missing, or disabled after read failures) the strict
            // BLE gate below still applies.
            if (!on_mat && colour_sensor_available()) {
                report_printf("ARRIVAL: not on %s or darker yet; BLE transfer waits for the colour mat\n",
                              colour_zone_name(COLOUR_ARRIVAL_ZONE));
                return PAYLOAD_NOT_CLOSE;
            }
            payload_client_set_close_threshold(on_mat ? COLOUR_ARRIVAL_BLE_DBM : AP1_CLOSE_RSSI_DBM);
            if (on_mat)
                report_printf("ARRIVAL: colour sensor confirms %s or darker; BLE only needs AP1 >= %d dBm\n",
                              colour_zone_name(COLOUR_ARRIVAL_ZONE), COLOUR_ARRIVAL_BLE_DBM);
        }
        payload_result_t result = was_close ? payload_client_retry() : payload_client_attempt();
        if (in_range_cue_on) {
            cue_stop();
            in_range_cue_on = false;
        }
        // A fatal cleanup failure may leave the radio active. Persist its tail
        // only after main shuts the radio down.
        if (result != PAYLOAD_FATAL && !save_checkpoint()) return PAYLOAD_FATAL;
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (result == PAYLOAD_NOT_CLOSE && !was_close) return result;
        was_close = true;
        if (attempt + 1 < MAX_PAYLOAD_ATTEMPTS) {
            report_printf("AP1 transfer not confirmed; stationary connection attempt %d/%d, proximity retained\n",
                          attempt + 2, MAX_PAYLOAD_ATTEMPTS);
            if (!motion_pause_ms(200)) return PAYLOAD_FATAL;
        }
    }
    fault_raise(FAULT_PAYLOAD);
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

// FOLLOW_MAT: a stop saw the mat, so colour steering takes over from Wi-Fi.
typedef enum { FOLLOW_NO_GAIN, FOLLOW_DECLINE, FOLLOW_MAT } follow_end_t;
typedef struct {
    float heading;
    bool progress, consistent;
    follow_end_t end;
    uint32_t position;
    line_pair_t baseline;
} follow_result_t;

static bool log_dip_reading(const char *stage, unsigned probe, uint32_t position,
                            line_pair_t reference, line_pair_t current, bool recovered)
{
    report_printf("DIP_CHECK,stage=%s,probe=%u,powered_ms=%lu,reference_dbm1=%d,reference_dbm2=%d,dbm1=%d,dbm2=%d,recovered=%u\n",
                  stage, probe, (unsigned long)position,
                  rssi_decode(reference.first.code), rssi_decode(reference.second.code),
                  rssi_decode(current.first.code), rssi_decode(current.second.code), recovered);
    return save_checkpoint();
}

// Step sizes. The host tests set the v16 sizes (250/100 ms) to replay runs
// recorded with them; the robot always uses the defaults below.
static uint32_t floor_block_ms = LINE_FORWARD_BLOCK_MS, dip_probe_ms = LINE_DIP_PROBE_MS;
static bool step_on_mat;

// Large steps on the floor, small steps once the latest stop saw the mat.
static uint32_t forward_block_ms(void)
{
    bool on_mat = colour_on_mat();
    if (on_mat != step_on_mat) {
        step_on_mat = on_mat;
        report_printf("STEP: %s; forward blocks now %u ms\n",
                      on_mat ? "colour sensor sees the mat" : "colour sensor no longer sees the mat",
                      on_mat ? LINE_MAT_BLOCK_MS : (unsigned)floor_block_ms);
    }
    return on_mat ? LINE_MAT_BLOCK_MS : floor_block_ms;
}

// ---- Colour scan on the spot (mat steering and the colour_scan test build) ----
#define SCAN_STEPS 12u
#define SCAN_STEP_DEG 30
#define SCAN_READS 3u
_Static_assert(SCAN_STEPS * SCAN_STEP_DEG == 360, "scan covers a full turn");
typedef enum { SCAN_DONE, SCAN_READ_FAILED, SCAN_STOPPED } scan_status_t;

// Three fresh readings while stopped; keep the median by G/R.
static bool scan_read(colour_reading_t *out)
{
    colour_reading_t r[SCAN_READS];
    for (unsigned i = 0; i < SCAN_READS; ++i) {
        if (!colour_sensor_read(&r[i])) return false;
        for (unsigned j = i; j && colour_gr_x1000(&r[j]) < colour_gr_x1000(&r[j - 1]); --j) {
            colour_reading_t swap = r[j]; r[j] = r[j - 1]; r[j - 1] = swap;
        }
    }
    *out = r[SCAN_READS / 2];
    return true;
}

// Turns on the spot through a full circle from origin, logging the colour
// under the sensor (which sits ahead of the wheels) at each 30-degree step,
// then estimates the direction of the mat centre. Ends facing origin + 330.
static scan_status_t colour_scan_turn(float origin, unsigned round, colour_scan_result_t *est,
                                      bool *saw_lighter)
{
    int angles[SCAN_STEPS];
    colour_reading_t readings[SCAN_STEPS];
    *saw_lighter = false;
    *est = (colour_scan_result_t){.valid = false, .max_depth = -1};
    for (unsigned i = 0; i < SCAN_STEPS; ++i) {
        if (!run_can_continue()) return SCAN_STOPPED;
        angles[i] = (int)i * SCAN_STEP_DEG;
        if (!motion_turn_to_heading(origin + (float)angles[i]) || !motion_pause_ms(200)) return SCAN_STOPPED;
        if (!scan_read(&readings[i])) {
            report_printf("COLOUR SCAN: sensor read error at %d degrees\n", angles[i]);
            return SCAN_READ_FAILED;
        }
        colour_zone_t zone = colour_classify(&readings[i]);
        if (colour_lighter_mat_zone(zone)) *saw_lighter = true;
        report_printf("COLOUR_SCAN,round=%u,angle=%d,heading=%d,zone=%s,depth=%d,c=%u,r=%u,g=%u,b=%u,gr=%u\n",
                      round, angles[i], (int)line_normalise_heading(motion_heading()),
                      colour_zone_name(zone), colour_zone_depth(zone), readings[i].c, readings[i].r,
                      readings[i].g, readings[i].b, colour_gr_x1000(&readings[i]));
        if (!report_checkpoint()) { fault_raise(FAULT_REPORT); return SCAN_STOPPED; }
    }
    *est = colour_scan_estimate(angles, readings, SCAN_STEPS);
    report_printf("COLOUR_SCAN_RESULT,round=%u,valid=%u,darkest_angle=%d,vector_angle=%d,depth_range=%d,max_depth=%d\n",
                  round, est->valid, est->darkest_angle, est->vector_angle, est->depth_range, est->max_depth);
    return SCAN_DONE;
}

// ---- Mat steering ----
// The Wi-Fi gradient is too flat near AP1 to find the centre, and a robot
// entering the mat side-on drives along the rings (7 November). On the mat
// the colour scan points at the centre instead (four scan tests, 7 November:
// the same darkest angle in both rounds of every run). After each scan the
// robot turns that way and takes small steps, checking the colour at every
// stop, until arrival or until the surface gets lighter, then scans again.
#define MAT_MAX_SCANS 6u          // per run; then the Wi-Fi search resumes
#define MAT_STEPS_PER_SCAN 4u     // small steps before checking the direction again
#define MAT_ARRIVED_RETRIES 3u    // stationary BLE retries when on ring 4 but AP1 not in range
static unsigned mat_scans_used;

static bool mat_steering_ready(void)
{
    return colour_on_mat() && mat_scans_used < MAT_MAX_SCANS;
}

// Ring 4 or darker but AP1 failed even the loose BLE gate: never drive on
// into the beacon box. Retry while stopped, then stop the run.
static payload_result_t mat_wait_on_arrival(void)
{
    for (unsigned retry = 1; retry <= MAT_ARRIVED_RETRIES; ++retry) {
        report_printf("MAT: on %s or darker but AP1 BLE not in range; stationary retry %u/%u\n",
                      colour_zone_name(COLOUR_ARRIVAL_ZONE), retry, MAT_ARRIVED_RETRIES);
        if (!motion_pause_ms(1000)) return PAYLOAD_FATAL;
        payload_result_t result = check_payload();
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (!colour_arrived) return PAYLOAD_NOT_CLOSE; // reading changed: keep steering
    }
    fault_raise(FAULT_PAYLOAD);
    report_printf("FAILED: on the mat centre but AP1 BLE never came in range; no further movement\n");
    return PAYLOAD_FATAL;
}

// Motors stopped, latest colour check on the mat but not arrived. Returns
// COMPLETE or FATAL, or NOT_CLOSE when the mat is lost or the scans run out,
// so the caller resumes the Wi-Fi search.
static payload_result_t mat_approach(void)
{
    report_printf("MAT: colour sensor sees the mat; steering by colour scans with %u ms steps\n",
                  LINE_MAT_BLOCK_MS);
    cue_play(CUE_PASS);
    while (mat_scans_used < MAT_MAX_SCANS) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        unsigned scan = ++mat_scans_used;
        float origin = motion_heading();
        colour_scan_result_t est;
        bool saw_lighter;
        scan_status_t status = colour_scan_turn(origin, scan, &est, &saw_lighter);
        if (status == SCAN_STOPPED) return PAYLOAD_FATAL;
        if (status == SCAN_READ_FAILED) {
            report_printf("MAT: scan %u could not read the colour sensor; resuming the Wi-Fi search\n", scan);
            return PAYLOAD_NOT_CLOSE;
        }
        // The scan saw the ring pattern, so a robot started on ring 4 can arrive.
        if (saw_lighter) colour_note_lighter_mat();
        if (!est.valid || est.max_depth < colour_zone_depth(COLOUR_RING1)) {
            report_printf("MAT: scan %u found no ring to steer toward; resuming the Wi-Fi search\n", scan);
            return PAYLOAD_NOT_CLOSE;
        }
        float heading = line_normalise_heading(origin + (float)est.vector_angle);
        report_printf("MAT: scan %u points %d degrees from its start, heading %d\n",
                      scan, est.vector_angle, (int)heading);
        if (!motion_turn_to_heading(heading)) return PAYLOAD_FATAL;
        // The sensor may already be over ring 4 after turning.
        payload_result_t result = check_payload();
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (colour_arrived) {
            result = mat_wait_on_arrival();
            if (result != PAYLOAD_NOT_CLOSE) return result;
        }
        int previous = colour_last_depth();
        for (unsigned step = 1; step <= MAT_STEPS_PER_SCAN; ++step) {
            uint32_t actual;
            result = translate_block(true, LINE_MAT_BLOCK_MS, heading, 0, 0, false, &actual);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            int depth = colour_last_depth();
            report_printf("MAT_STEP,scan=%u,step=%u,heading=%d,depth=%d,previous=%d\n",
                          scan, step, (int)heading, depth, previous);
            if (colour_arrived) {
                result = mat_wait_on_arrival();
                if (result != PAYLOAD_NOT_CLOSE) return result;
            }
            // Lighter (or unreadable): off the line to the centre, so scan again.
            if (depth < previous) break;
            previous = depth;
        }
    }
    report_printf("MAT: %u colour scans used without arrival; resuming the Wi-Fi search\n", MAT_MAX_SCANS);
    return PAYLOAD_NOT_CLOSE;
}

static payload_result_t recheck_and_probe_dip(float heading, uint32_t best_position,
                                             line_pair_t reference, uint32_t *position,
                                             line_pair_t *current, bool *recovered)
{
    *recovered = false;
    motion_stop();
    report_printf("DIP: three weak positions; stationary BLE check and two fresh Wi-Fi batches before any extra movement\n");
    payload_result_t result = check_payload();
    if (result != PAYLOAD_NOT_CLOSE) return result;
    result = measure_pair(current);
    if (result != PAYLOAD_NOT_CLOSE) return result;
    *recovered = line_return_consistent(reference, *current);
    if (!log_dip_reading("recheck", 0, *position, reference, *current, *recovered)) return PAYLOAD_FATAL;
    if (*recovered) return PAYLOAD_NOT_CLOSE;

    for (unsigned probe = 1; probe <= LINE_DIP_MAX_PROBES; ++probe) {
        report_printf("DIP: forward probe %u/%u, at most %u ms; frozen reference retained\n",
                      probe, LINE_DIP_MAX_PROBES, (unsigned)dip_probe_ms);
        uint32_t actual;
        result = translate_block(true, dip_probe_ms, heading, *position,
            translation_cost(*position + dip_probe_ms + BLOCK_RESERVE_MS - best_position),
            false, &actual);
        *position += actual;
        if (result != PAYLOAD_NOT_CLOSE) return result;
        wifi_sampler_set_position(heading, *position, true);
        result = measure_pair(current);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        *recovered = line_return_consistent(reference, *current);
        if (!log_dip_reading("probe", probe, *position, reference, *current, *recovered)) return PAYLOAD_FATAL;
        if (*recovered) return PAYLOAD_NOT_CLOSE;
    }
    report_printf("DIP: no recovery after %u probes; returning to retained best point\n", LINE_DIP_MAX_PROBES);
    return PAYLOAD_NOT_CLOSE;
}

// A zero trial_ms keeps the initial heading through inconclusive readings.
// Side probes retain a finite trial until they establish progress. Every mode
// still reserves return travel and checks the run, STOP, radio and fault limits.
static payload_result_t follow_direction(float heading, uint32_t trial_ms, follow_result_t *followed)
{
    *followed = (follow_result_t){.heading = line_normalise_heading(heading)};
    profile_count = 0;
    ++profile_id;
    // One long beep each time a new search direction starts.
    cue_play(CUE_PASS);
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
    line_pair_t recent[LINE_REFERENCE_POSITIONS] = {best};
    unsigned recent_count = 1, recent_next = 1;
    line_pair_t decline_reference = best;
    bool dip_active = false;
    report_printf("FOLLOW: heading %d, blocks at most %u ms on the floor, %u ms on the mat; %s\n",
                  (int)heading, (unsigned)floor_block_ms, LINE_MAT_BLOCK_MS,
                  trial_ms ? "trial until confirmed improvement" :
                  "continue through inconclusive readings within run and travel limits");
    while (run_can_continue()) {
        // Weak positions must be at least one full step apart to count separately.
        const uint32_t step = forward_block_ms();
        uint32_t block = step;
        if (!followed->progress && position < trial_ms && block > trial_ms - position)
            block = trial_ms - position;
        wifi_sampler_set_reference(line_reference(best));
        uint32_t actual;
        result = translate_block(true, block, heading, position,
            translation_cost(position + block + BLOCK_RESERVE_MS - best_position), true, &actual);
        position += actual;
        followed->position = position;
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (mat_steering_ready()) {
            followed->end = FOLLOW_MAT;
            report_printf("FOLLOW: colour sensor sees the mat at %lu powered ms; colour steering takes over\n",
                          (unsigned long)position);
            return PAYLOAD_NOT_CLOSE;
        }
        wifi_sampler_set_position(heading, position, true);
        line_pair_t current;
        result = measure_pair(&current);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (!record_point(heading, position, current)) return PAYLOAD_FATAL;
        int margin = line_pair_margin(best, current);
        int trend = line_compare(best, current);
        report_printf("FOLLOW_POINT,powered_ms=%lu,best_ms=%lu,gain_db=%d,margin_db=%d,baseline_gain_db=%d,baseline_margin_db=%d,trend=%d\n",
                      (unsigned long)position, (unsigned long)best_position,
                      line_confirmed_gain(best, current), margin, line_confirmed_gain(baseline, current),
                      line_pair_margin(baseline, current), trend);
        if (line_reference_declined(decline_reference, current)) {
            if (!dip_active) {
                dip_active = true;
                report_printf("DIP: freezing three-position reference at %d/%d dBm; best return point %lu powered ms\n",
                              rssi_decode(decline_reference.first.code), rssi_decode(decline_reference.second.code),
                              (unsigned long)best_position);
            }
            if (!decline_positions || position - last_decline_position >= step) {
                ++decline_positions;
                last_decline_position = position;
            }
            report_printf("FOLLOW: decline position %u/%u at %lu powered ms; two fresh batches below frozen reference\n",
                          decline_positions, LINE_DECLINE_POSITIONS, (unsigned long)position);
            if (decline_positions >= LINE_DECLINE_POSITIONS) {
                bool recovered;
                result = recheck_and_probe_dip(heading, best_position, decline_reference,
                                               &position, &current, &recovered);
                followed->position = position;
                if (result != PAYLOAD_NOT_CLOSE) return result;
                if (!recovered) {
                    followed->end = FOLLOW_DECLINE;
                    report_printf("FOLLOW: decline persisted after recheck and probes; return only %lu powered ms to best point %lu\n",
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
                report_printf("DIP: recovery confirmed; continuing the same heading\n");
                dip_active = false;
                decline_positions = 0;
            }
        } else if (dip_active) {
            bool recovered = line_return_consistent(decline_reference, current);
            if (decline_positions)
                report_printf("FOLLOW: decline sequence cleared at %lu powered ms; %s\n",
                              (unsigned long)position, recovered ? "signal recovered" : "inconclusive, reference still frozen");
            decline_positions = 0;
            if (recovered) dip_active = false;
        }
        // Keep the actual best return point separate from the median reference.
        // A recheck or probe can also discover a new best at the updated position.
        trend = line_compare(best, current);
        bool stronger = trend > 0 || (followed->progress &&
            line_compare(baseline, current) > 0 && line_confirmed_gain(best, current) > 0);
        if (stronger) {
            dip_active = false;
            decline_positions = 0;
            if (!followed->progress)
                report_printf("FOLLOW: first confirmed improvement; continuing without other direction probes\n");
            best = current;
            best_position = position;
            followed->progress = true;
            report_printf("FOLLOW: new confirmed best at %lu powered ms\n", (unsigned long)best_position);
        }
        if (!dip_active) {
            recent[recent_next] = current;
            recent_next = (recent_next + 1u) % LINE_REFERENCE_POSITIONS;
            if (recent_count < LINE_REFERENCE_POSITIONS) ++recent_count;
            if (recent_count == LINE_REFERENCE_POSITIONS) {
                line_pair_t candidate = line_median_reference(recent);
                // Never let the reference follow a gradual deterioration down.
                if (line_confirmed_gain(decline_reference, candidate) > 0) {
                    decline_reference = candidate;
                    report_printf("DIP: three-position reference strengthened to %d/%d dBm\n",
                                  rssi_decode(candidate.first.code), rssi_decode(candidate.second.code));
                }
            }
        }
        if (trial_ms && !followed->progress && position >= trial_ms && !decline_positions) {
            followed->end = FOLLOW_NO_GAIN;
            report_printf("FOLLOW: no confirmed improvement after %lu powered ms; trial ended at current position\n",
                          (unsigned long)position);
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

// Both perpendiculars, then straight back (v16 tried only the two sides, so a
// robot started facing away from AP1 could never turn round). A return whose
// RSSI disagrees with the best point no longer stops the search: the next
// trial measures its own starting baseline, so it does not depend on it.
#define SEARCH_DIRECTIONS 3u
static payload_result_t check_directions(float origin, follow_result_t *followed, bool *found)
{
    static const float offsets[SEARCH_DIRECTIONS] = {-90.0f, 90.0f, 180.0f};
    *found = false;
    report_printf("DIRECTION SEARCH: both perpendiculars, then straight back, origin %d degrees; follow first confirmed improvement\n",
                  (int)origin);
    for (unsigned i = 0; i < SEARCH_DIRECTIONS; ++i) {
        float heading = line_normalise_heading(origin + offsets[i]);
        payload_result_t result = follow_direction(heading, LINE_DIRECTION_TEST_MS, followed);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (followed->progress || followed->end == FOLLOW_MAT) {
            *found = true;
            return PAYLOAD_NOT_CLOSE;
        }
        if (followed->end == FOLLOW_NO_GAIN) {
            report_printf("TRIAL: no confirmed gain; return to its starting best point before the next direction\n");
            result = return_along_line(followed->position, 0, heading);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            line_pair_t returned;
            result = measure_pair(&returned);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            followed->consistent = line_return_consistent(followed->baseline, returned);
            report_printf("TRIAL RETURN: RSSI %s; timed position remains approximate\n",
                          followed->consistent ? "consistent with starting best point" :
                          "inconsistent with starting best point, continuing from here");
        }
    }
    return PAYLOAD_NOT_CLOSE;
}

static bool approach_beacon(void)
{
    run_started_ms = now_ms();
    translation_ms = profile_id = 0;
    wifi_quiescent = true;
    wifi_sampler_init(run_can_continue);
    colour_arrival_reset();
    step_on_mat = false;
    mat_scans_used = 0;
    payload_result_t result = check_payload();
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    wifi_sample_t initial;
    result = wait_for_wifi(&initial);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL || !motion_learn_direction()) return false;
    follow_result_t followed = {.end = FOLLOW_MAT};
    report_printf("APPROACH: follow current heading; after a confirmed decline return to best, then try both perpendiculars and straight back; on the mat, steer by colour\n");
    // Started on the mat: colour steering first, without any Wi-Fi line.
    if (!mat_steering_ready()) {
        result = follow_direction(motion_heading(), 0, &followed);
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
    }
    while (run_can_continue()) {
        if (followed.end == FOLLOW_MAT) {
            result = mat_approach();
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            report_printf("APPROACH: resuming the Wi-Fi line search at heading %d\n",
                          (int)line_normalise_heading(motion_heading()));
            result = follow_direction(motion_heading(), 0, &followed);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            continue;
        }
        if (followed.end != FOLLOW_DECLINE) {
            fault_raise(FAULT_NO_DIRECTION);
            report_printf("FAILED: current heading ended without a confirmed decline; no further direction search\n");
            return false;
        }
        if (!followed.consistent)
            report_printf("RETURN: RSSI inconsistent with best point; searching from here\n");
        float origin = followed.heading;
        bool found = false;
        result = check_directions(origin, &followed, &found);
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        if (!found) {
            fault_raise(FAULT_NO_DIRECTION);
            report_printf("FAILED: no direction confirmed improvement (both perpendiculars and straight back)\n");
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
    report_printf("COMPLETION BEEPS: GP22, three rising beeps\n");
    // Wait for the beeps to finish so the RESULT line follows them; GP21
    // still cuts them short.
    cue_play(CUE_COLLECTED);
    while (cue_busy()) {
        if (!motion_pause_ms(10)) { cue_stop(); break; }
    }
    report_printf("Completion beeps %s; motors stopped\n", motion_stop_requested() ? "stopped by GP21" : "finished");
}

static void report_faults(void)
{
    report_printf("FAULT MASK: 0x%03x\n", fault_mask());
    for (int f = 0; f < FAULT_COUNT; ++f)
        if (fault_test((fault_t)f)) report_printf("FAULT: %s\n", fault_name((fault_t)f));
}

#if COLOUR_SCAN_TEST
#define SCAN_ROUNDS 2u

// Two full turns on the spot, logging the colour under the sensor at each
// step and the estimated direction of the mat centre.
static bool colour_scan_run(void)
{
    run_started_ms = now_ms();
    if (!colour_sensor_available()) {
        report_printf("COLOUR SCAN FAILED: colour sensor not available\n");
        return false;
    }
    if (!motion_learn_direction()) return false;
    float origin = motion_heading();
    report_printf("COLOUR SCAN: %u rounds, %u readings at %d-degree steps from heading %d\n",
                  SCAN_ROUNDS, SCAN_STEPS, SCAN_STEP_DEG, (int)origin);
    bool all_valid = true;
    for (unsigned round = 1; round <= SCAN_ROUNDS; ++round) {
        cue_play(CUE_PASS);
        colour_scan_result_t est;
        bool saw_lighter;
        scan_status_t status = colour_scan_turn(origin, round, &est, &saw_lighter);
        if (status == SCAN_READ_FAILED) report_printf("COLOUR SCAN FAILED: sensor read error\n");
        if (status != SCAN_DONE) return false;
        if (!est.valid) all_valid = false;
        if (!motion_turn_to_heading(origin)) return false;
    }
    return all_valid;
}
#endif

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
    cues_init();
    sleep_ms(2000);
    report_printf("Getting there | line-search v18 steps500-mat250 back-search mat-steer colour-gate ble-minus58 colour-ring4 cues | AP1 only\n");
#if COLOUR_SCAN_TEST
    report_printf("TEST BUILD: colour scan on the spot after calibration; no radio, no approach\n");
#endif
    report_printf("Flash layout: 2 MiB physical; 1528 KiB firmware, 8 KiB Bluetooth, 512 KiB saved log\n");
    report_printf("Before GP20: GP21 replays previous run. After GP20: GP21 STOP, latched until reset.\n");
    report_printf("Move using Wi-Fi direction; on the mat, steer by colour scans\n");
    report_printf("Transfer only on colour %s or darker after a lighter ring, with BLE >= %d dBm\n",
                  colour_zone_name(COLOUR_ARRIVAL_ZONE), COLOUR_ARRIVAL_BLE_DBM);
    report_printf("Without a working colour sensor: transfer after BLE averages >= %d dBm\n", AP1_CLOSE_RSSI_DBM);
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
        cue_play(CUE_START);
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
    // Optional: without the sensor the robot keeps the BLE-only proximity gate.
    if (ready) colour_sensor_init();
    if (ready) {
        report_printf("Starting gyro calibration before Wi-Fi/Bluetooth. Keep the robot still.\n");
        // A steady tone for as long as the robot must stay still.
        cue_play(CUE_CALIBRATING);
        ready = motion_calibrate();
        cue_stop();
        if (!ready) report_printf("FAILED: gyro calibration; check IMU wiring and keep the robot still\n");
    }
    bool radio_initialised = false;
#if COLOUR_SCAN_TEST
    // Test build: no radio and no approach; scan the colour on the spot.
    (void)approach_beacon; (void)on_in_range;
    if (ready && !motion_stop_requested()) {
        bool scanned = colour_scan_run();
        motion_stop();
        if (scanned) completion_sound();
        else cue_play(CUE_FAULT);
        report_printf("RESULT: %s\n", scanned ? "COLOUR SCAN COMPLETE" : "COLOUR SCAN FAILED");
    } else {
        cue_play(CUE_FAULT);
        report_printf("RESULT: STARTUP FAILED OR STOPPED\n");
    }
#else
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
            payload_client_set_close_callback(on_in_range);
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
        else cue_play(CUE_FAULT);
        report_printf("RESULT: %s\n", completed ? "PAYLOAD CONFIRMED" : "STOPPED WITHOUT PAYLOAD");
    } else {
        cue_play(CUE_FAULT);
        report_printf("RESULT: STARTUP FAILED OR STOPPED\n");
    }
#endif
    motion_stop();
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
