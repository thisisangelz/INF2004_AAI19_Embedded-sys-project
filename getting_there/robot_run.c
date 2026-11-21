#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "colour_sensor.h"
#include "faults.h"
#include "motion.h"
#include "line_search.h"
#include "payload_client.h"
#include "robot_cues.h"
#include "run_report.h"
#include "target_beacon.h"

#include "robot_run.h"
#include "colour_scan.h"

#define MAX_RUN_MS 900000u
#define MAX_MISSING_SAMPLES 3
#define MAX_PAYLOAD_ATTEMPTS 3
// Reserve polling/I2C overrun for every translation block, including returns.
#define BLOCK_RESERVE_MS 25u

static uint32_t run_started_ms, translation_ms;
static bool wifi_quiescent = true;
static uint32_t run_now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static bool run_can_continue(void)
{
    if (motion_stop_requested()) { motion_stop(); return false; }
    if (!report_ok()) { motion_stop(); fault_raise(FAULT_REPORT); return false; }
    if ((uint32_t)(run_now_ms() - run_started_ms) >= MAX_RUN_MS) {
        motion_stop();
        fault_raise(FAULT_RUN_LIMIT);
        report_printf("FAILED: 15 minute run limit reached (%s)\n", target_name());
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
// Set by mat steering when the robot is at the mat centre although no reading
// reached the arrival ring (see mat_centre_reached). Counts as colour arrival.
static bool mat_centre_override;

// Set while the robot backs off a beacon it has finished with: stops still
// read the colour sensor, but never start a transfer.
static bool transfer_paused;

static payload_result_t check_payload(void)
{
    bool was_close = false;
    colour_arrived = false;
    for (int attempt = 0; attempt < MAX_PAYLOAD_ATTEMPTS; ++attempt) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        if (!save_checkpoint()) return PAYLOAD_FATAL;
        if (transfer_paused) {
            (void)colour_arrival_check();
            return PAYLOAD_NOT_CLOSE;
        }
        if (!was_close) {
            // Motors are stopped here. On the mat, the colour sensor decides
            // arrival and BLE only has to show AP1 is in range.
            bool on_mat = colour_arrival_check() || mat_centre_override;
            colour_arrived = on_mat;
            // With a working colour sensor, only the mat allows a transfer:
            // the -58 dBm BLE gate alone passed on ring 1 when the robot
            // drove along the rings (7 November side-on run). This also saves
            // the 5 s BLE check at every stop before the mat. Without the
            // sensor (missing, or disabled after read failures) the strict
            // BLE gate below still applies.
            if (!on_mat && colour_sensor_available()) {
                report_printf("ARRIVAL: not on %s or darker yet; BLE transfer waits for the colour mat\n",
                              COLOUR_ARRIVAL_TEXT);
                return PAYLOAD_NOT_CLOSE;
            }
            payload_client_set_close_threshold(on_mat ? COLOUR_ARRIVAL_BLE_DBM : AP1_CLOSE_RSSI_DBM);
            if (on_mat)
                report_printf("ARRIVAL: colour sensor confirms %s or darker; BLE only needs %s >= %d dBm\n",
                              COLOUR_ARRIVAL_TEXT, target_name(), COLOUR_ARRIVAL_BLE_DBM);
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
            report_printf("%s transfer not confirmed; stationary connection attempt %d/%d, proximity retained\n", target_name(),
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
        report_printf("%s Wi-Fi missing; checking BLE while remaining stopped\n", target_name());
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (!motion_pause_ms(500)) return PAYLOAD_FATAL;
    }
    report_printf("FAILED: %s missing from three fresh Wi-Fi samples; no blind movement\n", target_name());
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

// Set by translate_block when a forward block ended with a ring of the
// avoided mat (the beacon already visited) under the sensor. The block has
// then been driven back, so *actual is 0; the caller must not drive that way
// again.
static bool avoided_mat_hit;

static payload_result_t drive_block(bool forwards, uint32_t duration, float heading,
                                    uint32_t position, uint32_t reserved_return,
                                    bool acquire_moving, uint32_t *actual);

// One powered block, then settle, drain Wi-Fi, persist and check BLE. A fatal
// scan failure keeps wifi_quiescent false until main shuts the radio down.
static payload_result_t translate_block(bool forwards, uint32_t duration, float heading,
                                        uint32_t position, uint32_t reserved_return,
                                        bool acquire_moving, uint32_t *actual)
{
    avoided_mat_hit = false;
    payload_result_t result = drive_block(forwards, duration, heading, position,
                                          reserved_return, acquire_moving, actual);
    if (result != PAYLOAD_NOT_CLOSE || !forwards || !*actual || !colour_on_avoided_mat()) return result;
    // Never drive onto the beacon already visited: its box is still there,
    // and to the colour sensor its rings are floor. Reverse the same block.
    uint32_t forward = *actual, back;
    report_printf("ROUTE: %s mat ahead after %lu ms on heading %d; reversing that block, this way is blocked\n",
                  colour_mat_name(colour_avoid_mat()), (unsigned long)forward, (int)heading);
    result = drive_block(false, forward, heading, position + forward, 0, false, &back);
    avoided_mat_hit = true;
    *actual = back < forward ? forward - back : 0;
    return result;
}

static payload_result_t drive_block(bool forwards, uint32_t duration, float heading,
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
    *point = (line_point_t){.wall_ms = run_now_ms(), .powered_ms = powered,
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
    // early: the line ended at its best point because the signal stopped
    // rising, not after a confirmed decline (see LINE_EARLY_PROBE_GAIN_DB).
    bool progress, consistent, early;
    follow_end_t end;
    uint32_t position;
    line_pair_t baseline;
    // at_best: the reading back at the best point after a declined or early
    // line, facing the line's heading (as the side probes measure).
    line_pair_t at_best;
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
// The tests set 250 to reproduce the 7 November blue-mat run.
static uint32_t mat_near_block_ms = LINE_MAT_NEAR_BLOCK_MS;
static bool step_on_mat;

// On the mat: 250 ms steps on the outer rings, 100 ms near the centre.
static uint32_t mat_step_ms(void)
{
    return colour_near_arrival(LINE_MAT_NEAR_RINGS) ? mat_near_block_ms : LINE_MAT_BLOCK_MS;
}

// Large steps on the floor, small steps once the latest stop saw the mat.
static uint32_t forward_block_ms(void)
{
    bool on_mat = colour_on_mat();
    if (on_mat != step_on_mat) {
        step_on_mat = on_mat;
        report_printf("STEP: %s; forward blocks now %u ms\n",
                      on_mat ? "colour sensor sees the mat" : "colour sensor no longer sees the mat",
                      on_mat ? (unsigned)mat_step_ms() : (unsigned)floor_block_ms);
    }
    return on_mat ? mat_step_ms() : floor_block_ms;
}

// ---- Mat steering ----
// The Wi-Fi gradient is too flat near AP1 to find the centre, and a robot
// entering the mat side-on drives along the rings (7 November). On the mat
// the colour scan points at the centre instead (four scan tests, 7 November:
// the same darkest angle in both rounds of every run). After each scan the
// robot turns that way and takes small steps, checking the colour at every
// stop, until arrival or until the surface gets lighter, then scans again.
#define MAT_MAX_SCANS 6u          // per run; then the Wi-Fi search resumes
// Travel after each scan before checking the direction again: four outer
// steps, or ten near the centre, so small steps do not cause extra scans.
#define MAT_TRAVEL_PER_SCAN_MS 1000u
#define MAT_ARRIVED_RETRIES 3u    // stationary BLE retries when arrived but AP1 not in range
// Small steps within one ring of arrival that read no darker: the robot is
// pushing against the beacon box (7 November blue run: seven 100 ms steps on
// the same reading, heading error growing). A ring took two or three such
// steps in the purple run, so three flat steps means the robot is not moving.
#define MAT_STALL_STEPS 3u
static unsigned mat_scans_used;

static bool mat_steering_ready(void)
{
    return colour_on_mat() && mat_scans_used < MAT_MAX_SCANS;
}

// On the arrival ring but AP1 failed even the loose BLE gate: never drive on
// into the beacon box. Retry while stopped, then stop the run.
static payload_result_t mat_wait_on_arrival(void)
{
    for (unsigned retry = 1; retry <= MAT_ARRIVED_RETRIES; ++retry) {
        report_printf("MAT: on %s or darker but %s BLE not in range; stationary retry %u/%u\n",
                      COLOUR_ARRIVAL_TEXT, target_name(), retry, MAT_ARRIVED_RETRIES);
        if (!motion_pause_ms(1000)) return PAYLOAD_FATAL;
        payload_result_t result = check_payload();
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (!colour_arrived) return PAYLOAD_NOT_CLOSE; // reading changed: keep steering
    }
    fault_raise(FAULT_PAYLOAD);
    report_printf("FAILED: on the mat centre but %s BLE never came in range; no further movement\n", target_name());
    return PAYLOAD_FATAL;
}

// The robot is at the mat centre although no check reached the arrival ring.
// On 7 November the inner rings of the blue and purple mats read about one
// ring lighter than calibrated (the sensor saw roughly half the calibration
// brightness), so arrival never triggered and the robot inched on into the
// beacon box. Stop here for good: transfer with the loose BLE gate, retry in
// place if AP1 is not in range, and never move again from this point.
static payload_result_t mat_centre_reached(const char *why)
{
    report_printf("MAT: %s; at the mat centre, treating as arrived with no further movement\n", why);
    mat_centre_override = true;
    payload_result_t result = check_payload();
    if (result != PAYLOAD_NOT_CLOSE) return result;
    return mat_wait_on_arrival(); // FATAL after its retries: never drives again
}

// Motors stopped, latest colour check on the mat but not arrived. Returns
// COMPLETE or FATAL, or NOT_CLOSE when the mat is lost or the scans run out,
// so the caller resumes the Wi-Fi search.
static payload_result_t mat_approach(void)
{
    report_printf("MAT: colour sensor sees the mat; steering by colour scans with %u ms steps, %u ms within %u rings of arrival\n",
                  LINE_MAT_BLOCK_MS, (unsigned)mat_near_block_ms, LINE_MAT_NEAR_RINGS);
    cue_play(CUE_PASS);
    while (mat_scans_used < MAT_MAX_SCANS) {
        if (!run_can_continue()) return PAYLOAD_FATAL;
        unsigned scan = ++mat_scans_used;
        float origin = motion_heading();
        colour_scan_result_t est;
        unsigned lighter_mats;
        int farthest_rings;
        scan_status_t status = colour_scan_turn(origin, scan, &est, &lighter_mats,
                                               &farthest_rings, run_can_continue, save_checkpoint);
        if (status == SCAN_STOPPED) return PAYLOAD_FATAL;
        if (status == SCAN_READ_FAILED) {
            report_printf("MAT: scan %u could not read the colour sensor; resuming the Wi-Fi search\n", scan);
            return PAYLOAD_NOT_CLOSE;
        }
        // The scan saw the ring pattern, so a robot started on the arrival
        // ring can arrive.
        colour_note_lighter_mats(lighter_mats);
        // Inner rings in every direction: there is nowhere darker to go. On
        // 7 November this scan (all blue ring 3 readings) instead resumed the
        // Wi-Fi search, which drove the robot into the beacon box.
        if (farthest_rings <= (int)LINE_MAT_NEAR_RINGS)
            return mat_centre_reached("every scan direction is within two rings of arrival");
        if (!est.valid || est.max_depth < colour_zone_depth(COLOUR_RING1)) {
            report_printf("MAT: scan %u found no ring to steer toward; resuming the Wi-Fi search\n", scan);
            return PAYLOAD_NOT_CLOSE;
        }
        float heading = line_normalise_heading(origin + (float)est.vector_angle);
        report_printf("MAT: scan %u points %d degrees from its start, heading %d\n",
                      scan, est.vector_angle, (int)heading);
        if (!motion_turn_to_heading(heading)) return PAYLOAD_FATAL;
        // The sensor may already be over the arrival ring after turning.
        payload_result_t result = check_payload();
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (colour_arrived) {
            result = mat_wait_on_arrival();
            if (result != PAYLOAD_NOT_CLOSE) return result;
        }
        int previous = colour_last_depth();
        uint32_t travelled = 0;
        unsigned flat_steps = 0;
        for (unsigned step = 1; travelled < MAT_TRAVEL_PER_SCAN_MS; ++step) {
            uint32_t actual, block = mat_step_ms();
            result = translate_block(true, block, heading, 0, 0, false, &actual);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            // A stop that moved nothing still counts a full block, so the loop ends.
            travelled += actual ? actual : block;
            int depth = colour_last_depth();
            report_printf("MAT_STEP,scan=%u,step=%u,heading=%d,ms=%lu,depth=%d,previous=%d\n",
                          scan, step, (int)heading, (unsigned long)block, depth, previous);
            if (colour_arrived) {
                result = mat_wait_on_arrival();
                if (result != PAYLOAD_NOT_CLOSE) return result;
            }
            // Lighter (or unreadable): off the line to the centre, so scan again.
            if (depth < previous) break;
            if (depth > previous || !colour_near_arrival(1)) flat_steps = 0;
            else if (++flat_steps >= MAT_STALL_STEPS)
                return mat_centre_reached("three small steps one ring from arrival read no darker; the robot is against the beacon box");
            previous = depth;
        }
    }
    // Out of scans close to the centre: never hand back to Wi-Fi driving here.
    if (colour_near_arrival(LINE_MAT_NEAR_RINGS))
        return mat_centre_reached("colour scans used up within two rings of arrival");
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
    // Let the caller hand over at this position before any more Wi-Fi or travel.
    if (mat_steering_ready()) return PAYLOAD_NOT_CLOSE;
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
        if (avoided_mat_hit) {
            report_printf("DIP: probe blocked by the visited beacon's mat; returning to retained best point\n");
            return PAYLOAD_NOT_CLOSE;
        }
        if (mat_steering_ready()) return PAYLOAD_NOT_CLOSE;
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

// Ends a line as declined: drive back to its best point and check the signal
// there. The caller has logged why.
static payload_result_t end_line_at_best(follow_result_t *followed, float heading, uint32_t position,
                                         uint32_t best_position, line_pair_t best)
{
    followed->end = FOLLOW_DECLINE;
    payload_result_t result = return_along_line(position, best_position, heading);
    if (result != PAYLOAD_NOT_CLOSE) return result;
    line_pair_t returned;
    result = measure_pair(&returned);
    if (result != PAYLOAD_NOT_CLOSE) return result;
    followed->at_best = returned;
    followed->consistent = line_return_consistent(best, returned);
    report_printf("RETURN: RSSI %s; timed position remains approximate\n",
                  followed->consistent ? "consistent with best point" : "inconsistent with best point, stopping");
    return PAYLOAD_NOT_CLOSE;
}

// Early side probe (17 November): each time a line passed beside a mat, the
// signal rose, went flat for two to five stops, then fell. Waiting for the
// confirmed decline (three weak stops, a recheck, two probes and the drive
// back) cost about 1-1.5 minutes per line. Once the line has risen at least
// LINE_EARLY_PROBE_GAIN_DB and then found no new best for
// LINE_EARLY_PROBE_FLAT_POINTS stops, it ends at its best point and the side
// probe runs. Set side_probe_enabled to use it (the shipped robot does).
// skip_early_probe: the next line must not end early (it continues a line
// whose early probe found no clear side).
static bool skip_early_probe;
// Side probe after a line ends (see "Side probe" below). Tests replaying
// earlier runs turn it off; the robot always uses it.
static bool side_probe_enabled = true;

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
    // Stops since the last new best; the early probe needs a few in a row.
    unsigned since_best = 0;
    const bool early_allowed = side_probe_enabled && !skip_early_probe;
    skip_early_probe = false;
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
        if (avoided_mat_hit) {
            if (trial_ms && !followed->progress) {
                followed->end = FOLLOW_NO_GAIN;
                report_printf("FOLLOW: trial ended at the visited beacon's mat, %lu powered ms\n",
                              (unsigned long)position);
                return PAYLOAD_NOT_CLOSE;
            }
            report_printf("FOLLOW: the visited beacon's mat blocks this line; return %lu powered ms to best point %lu\n",
                          (unsigned long)(position - best_position), (unsigned long)best_position);
            return end_line_at_best(followed, heading, position, best_position, best);
        }
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
                // Avoidance takes precedence: a corrective reverse can see
                // the target edge, but the blocked line must still return.
                if (!avoided_mat_hit && mat_steering_ready()) {
                    followed->end = FOLLOW_MAT;
                    report_printf("DIP: colour sensor sees the mat at %lu powered ms; colour steering takes over\n",
                                  (unsigned long)position);
                    return PAYLOAD_NOT_CLOSE;
                }
                if (!recovered) {
                    report_printf("FOLLOW: decline persisted after recheck and probes; return only %lu powered ms to best point %lu\n",
                                  (unsigned long)(position - best_position), (unsigned long)best_position);
                    return end_line_at_best(followed, heading, position, best_position, best);
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
            since_best = 0;
            report_printf("FOLLOW: new confirmed best at %lu powered ms\n", (unsigned long)best_position);
        } else {
            ++since_best;
        }
        if (early_allowed && since_best >= LINE_EARLY_PROBE_FLAT_POINTS &&
            line_confirmed_gain(baseline, best) >= LINE_EARLY_PROBE_GAIN_DB) {
            followed->early = true;
            report_printf("FOLLOW: no new best for %u stops after rising %d dB; the beacon is beside this line, return %lu powered ms to best point %lu and probe the sides now\n",
                          since_best, line_confirmed_gain(baseline, best),
                          (unsigned long)(position - best_position), (unsigned long)best_position);
            return end_line_at_best(followed, heading, position, best_position, best);
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
// first_side (-90 or +90) is the perpendicular tried first: a side probe's
// stronger side, or -90 by default.
// last: 180 (straight back) after a confirmed decline, 0 (straight on) after
// an early side probe, when the line had not declined.
static payload_result_t check_directions_from(float origin, float first_side, float last,
                                              follow_result_t *followed, bool *found)
{
    const float offsets[SEARCH_DIRECTIONS] = {first_side, -first_side, last};
    *found = false;
    report_printf("DIRECTION SEARCH: perpendicular %+d first, then %+d, then straight %s, origin %d degrees; follow first confirmed improvement\n",
                  (int)first_side, (int)-first_side, last == 0.0f ? "on" : "back", (int)origin);
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

// The default order (-90 first); kept for the host tests of that order.
static inline payload_result_t check_directions(float origin, follow_result_t *followed, bool *found)
{
    return check_directions_from(origin, -90.0f, 180.0f, followed, found);
}

// ---- Side probe ----
// After a line declines, the robot is back at its strongest point on that
// line, so AP1 lies roughly to one side of it. The fixed order then tried
// -90 first: on 17 November that turned away from the mat on the left. The
// heading itself cannot pick the side: the robot's body changes the reading
// by up to 17 dB (same spot: -61 dBm facing 0, -78 dBm facing 270, the side
// AP1 was on). So each probe drives a short way to one side, turns back to
// the line's heading, and measures there; both sides are compared facing the
// same way. The stronger side is tried first; a tie keeps the old order.
// Tests replaying earlier runs turn it off (reset); the robot always uses it
// (side_probe_enabled, declared with the early probe above).
typedef enum { PROBE_UNDECIDED, PROBE_DECIDED, PROBE_MAT } probe_outcome_t;

// Drives LINE_SIDE_PROBE_MS along side_heading, measures facing origin, and
// drives back. Stops at the mat (mat set) without returning, so colour
// steering can take over from there.
static payload_result_t probe_one_side(float origin, float side_heading, line_pair_t *reading, bool *mat)
{
    *mat = false;
    if (!motion_turn_to_heading(side_heading)) return PAYLOAD_FATAL;
    uint32_t moved = 0;
    while (moved < LINE_SIDE_PROBE_MS) {
        uint32_t block = LINE_SIDE_PROBE_MS - moved;
        if (block > LINE_BLOCK_MS) block = LINE_BLOCK_MS;
        uint32_t actual;
        // Reserve the whole drive back, including this block.
        payload_result_t result = translate_block(true, block, side_heading, moved,
            translation_cost(moved + block + BLOCK_RESERVE_MS), false, &actual);
        moved += actual;
        if (result != PAYLOAD_NOT_CLOSE) return result;
        // Measure where the visited beacon's mat stopped the probe.
        if (avoided_mat_hit) break;
        if (mat_steering_ready()) {
            *mat = true;
            report_printf("SIDE PROBE: colour sensor sees the mat at %lu ms on heading %d; colour steering takes over\n",
                          (unsigned long)moved, (int)side_heading);
            return PAYLOAD_NOT_CLOSE;
        }
    }
    if (!motion_turn_to_heading(origin)) return PAYLOAD_FATAL;
    wifi_sampler_set_position(origin, 0, true);
    payload_result_t result = measure_pair(reading);
    if (result != PAYLOAD_NOT_CLOSE) return result;
    report_printf("SIDE_PROBE,side_heading=%d,moved_ms=%lu,facing=%d,dbm1=%d,dbm2=%d\n",
                  (int)side_heading, (unsigned long)moved, (int)origin,
                  rssi_decode(reading->first.code), rssi_decode(reading->second.code));
    if (!save_checkpoint()) return PAYLOAD_FATAL;
    if (!motion_turn_to_heading(side_heading)) return PAYLOAD_FATAL;
    return return_along_line(moved, 0, side_heading);
}

// first_reading: the reading of the side tried first, when one is decided.
static payload_result_t side_probe(float origin, float *first_side, line_pair_t *first_reading,
                                   probe_outcome_t *outcome)
{
    static const float sides[2] = {-90.0f, 90.0f};
    line_pair_t readings[2];
    *first_side = -90.0f;
    *outcome = PROBE_UNDECIDED;
    report_printf("SIDE PROBE: from the best point on heading %d, drive %u ms to each side and measure facing %d\n",
                  (int)origin, LINE_SIDE_PROBE_MS, (int)origin);
    for (unsigned i = 0; i < 2; ++i) {
        bool mat;
        payload_result_t result = probe_one_side(origin, line_normalise_heading(origin + sides[i]),
                                                 &readings[i], &mat);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (mat) {
            *outcome = PROBE_MAT;
            return PAYLOAD_NOT_CLOSE;
        }
    }
    if (!motion_turn_to_heading(origin)) return PAYLOAD_FATAL;
    // Same noise margin as every other gain decision (at least 3 dB), or the
    // average when the two batches disagree.
    bool by_average;
    int verdict = line_side_compare(readings[0], readings[1], &by_average);
    if (verdict != 0) {
        *first_side = verdict > 0 ? sides[1] : sides[0];
        *first_reading = verdict > 0 ? readings[1] : readings[0];
        *outcome = PROBE_DECIDED;
    }
    report_printf("SIDE PROBE: %+d side %d/%d dBm, %+d side %d/%d dBm; %s%s\n",
                  (int)sides[0], rssi_decode(readings[0].first.code), rssi_decode(readings[0].second.code),
                  (int)sides[1], rssi_decode(readings[1].first.code), rssi_decode(readings[1].second.code),
                  *outcome == PROBE_DECIDED ? (*first_side > 0 ? "try +90 first" : "try -90 first")
                                            : "no clear side, keep the usual order",
                  by_average ? " (batches disagree; decided on the average)" : "");
    return PAYLOAD_NOT_CLOSE;
}

// Each beacon of a route starts afresh: its own 15-minute limit and travel
// budget, and no colour memory or mat scans from the previous beacon.
static void begin_leg(void)
{
    run_started_ms = run_now_ms();
    translation_ms = 0;
    colour_arrival_reset();
    step_on_mat = false;
    mat_scans_used = 0;
    mat_centre_override = false;
}

static bool approach_loop(follow_result_t followed);

static bool approach_beacon(void)
{
    profile_id = 0;
    wifi_quiescent = true;
    wifi_sampler_init(run_can_continue);
    begin_leg();
    payload_result_t result = check_payload();
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    wifi_sample_t initial;
    result = wait_for_wifi(&initial);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL || !motion_learn_direction()) return false;
    follow_result_t followed = {.end = FOLLOW_MAT};
    report_printf("APPROACH: follow current heading; after a confirmed decline return to best, probe both sides, try the stronger side first, then the other and straight back; on the mat, steer by colour\n");
    // Started on the mat: colour steering first, without any Wi-Fi line.
    if (!mat_steering_ready()) {
        result = follow_direction(motion_heading(), 0, &followed);
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
    }
    return approach_loop(followed);
}

// From the end of a line (or a hand-over to colour steering) until the
// transfer completes or the run stops.
static bool approach_loop(follow_result_t followed)
{
    payload_result_t result;
    while (run_can_continue()) {
        if (followed.end == FOLLOW_MAT) {
            result = mat_approach();
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            report_printf("APPROACH: resuming the Wi-Fi line search at heading %d\n",
                          (int)line_normalise_heading(motion_heading()));
            // Normalised: after several scans the gyro total can exceed 360,
            // which logged lines like "FOLLOW: heading 519".
            result = follow_direction(line_normalise_heading(motion_heading()), 0, &followed);
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
        const bool early = followed.early; // the trials below overwrite followed
        bool found = false;
        float first_side = -90.0f;
        if (side_probe_enabled) {
            probe_outcome_t outcome;
            line_pair_t side_reading = followed.at_best; // set when a side is decided
            result = side_probe(origin, &first_side, &side_reading, &outcome);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            if (outcome == PROBE_MAT) {
                followed.end = FOLLOW_MAT;
                continue;
            }
            // The line had not declined, so the beacon may still be ahead.
            // Turn off it only for a side that reads clearly stronger than
            // the best point itself (side_compare: 3 dB). On 17 November both
            // sides read weaker than the best point three times, and each
            // time AP2 was straight on: the side lines cost 2.5-3 minutes.
            // When a side was stronger (-55/-56 against -59/-60), the mat
            // was on that side.
            bool beside = false;
            if (outcome == PROBE_DECIDED && early) {
                bool by_average;
                beside = line_side_compare(followed.at_best, side_reading, &by_average) > 0;
                report_printf("APPROACH: %+d side %d/%d dBm against %d/%d dBm at the best point; %s\n",
                              (int)first_side, rssi_decode(side_reading.first.code),
                              rssi_decode(side_reading.second.code),
                              rssi_decode(followed.at_best.first.code),
                              rssi_decode(followed.at_best.second.code),
                              beside ? "stronger, try that side first" :
                                       "not stronger than the best point, straight on first");
            }
            if (early && !beside) {
                report_printf("APPROACH: %s after the early probe; continue heading %d from the best point\n",
                              outcome == PROBE_DECIDED ? "no side stronger than the best point" : "no clear side",
                              (int)origin);
                skip_early_probe = true;
                result = follow_direction(origin, 0, &followed);
                if (result == PAYLOAD_COMPLETE) return true;
                if (result == PAYLOAD_FATAL) return false;
                continue;
            }
        }
        // After an early probe the line had not declined, so the last try is
        // straight on rather than straight back.
        result = check_directions_from(origin, first_side, early ? 0.0f : 180.0f,
                                       &followed, &found);
        if (result == PAYLOAD_COMPLETE) return true;
        if (result == PAYLOAD_FATAL) return false;
        if (!found) {
            fault_raise(FAULT_NO_DIRECTION);
            report_printf("FAILED: no direction confirmed improvement (both perpendiculars and straight %s)\n",
                          early ? "on" : "back");
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

// ---- Route: the first beacon, then the next one, from one flash ----
// ROUTE_NEXT_BEACON (CMake, robot_route and robot_leg2): after a verified
// transfer at TARGET_BEACON_ID the robot goes on to this beacon; 0 ends the
// run at the first beacon, as before. ROUTE_START_ON_MAT (robot_leg2) is a
// test build: the robot starts on the first beacon's arrival ring, facing the
// beacon as if it had just arrived, and skips that transfer.
#ifndef ROUTE_NEXT_BEACON
#define ROUTE_NEXT_BEACON 0
#endif
#ifndef ROUTE_START_ON_MAT
#define ROUTE_START_ON_MAT 0
#endif
// Variables so the host tests can run either build.
static uint8_t route_next = ROUTE_NEXT_BEACON;
static bool route_start_on_mat = ROUTE_START_ON_MAT;

// Backing off the visited mat: small blocks until two stops in a row read
// floor, then a little more so the whole robot is clear before it turns.
// Pink ring 4 to the edge took about 0.5 s of driving on 17 November.
#define LEAVE_MAT_BLOCK_MS 250u
#define LEAVE_MAT_FLOOR_CHECKS 2u
#define LEAVE_MAT_MAX_MS 3000u
#define LEAVE_MAT_CLEARANCE_MS 500u
#define LEAVE_MAT_BLIND_MS 1500u   // without a colour sensor
// Directions tried from just off the visited mat, relative to facing it:
// straight back first, then each side. Never forward, through the beacon.
#define LEG_DIRECTIONS 3u

static void announce_target(const char *what)
{
    report_printf("ROUTE: %s %s (Wi-Fi \"%s\", %s mat)%s%s%s\n", what, target_name(), target_ssid(),
                  colour_mat_name(target_mat()),
                  colour_avoid_mat() != COLOUR_MAT_NONE ? "; keeping off the " : "",
                  colour_avoid_mat() != COLOUR_MAT_NONE ? colour_mat_name(colour_avoid_mat()) : "",
                  colour_avoid_mat() != COLOUR_MAT_NONE ? " mat" : "");
}

static void select_target(uint8_t id, colour_mat_t avoid)
{
    target_select(id);
    colour_set_target_mat(target_mat());
    colour_set_avoid_mat(avoid);
    colour_arrival_reset();
    wifi_sampler_retarget();
}

// Reverses straight out along the heading the robot arrived on. Mat steering
// drove in along it from lighter rings to darker, so reversing retraces that
// path, and the robot never turns next to the beacon box. Stops only read
// the colour: the transfer here is already done.
static payload_result_t leave_mat(float heading)
{
    const bool sensor = colour_sensor_available();
    const uint32_t limit = sensor ? LEAVE_MAT_MAX_MS : LEAVE_MAT_BLIND_MS;
    if (sensor)
        report_printf("ROUTE: leaving the %s mat: reverse on heading %d until %u stops read floor (at most %u ms), then %u ms more\n",
                      colour_mat_name(target_mat()), (int)heading, LEAVE_MAT_FLOOR_CHECKS,
                      (unsigned)limit, LEAVE_MAT_CLEARANCE_MS);
    else
        report_printf("ROUTE: leaving the %s mat without a colour sensor: reverse %u ms on heading %d\n",
                      colour_mat_name(target_mat()), (unsigned)limit, (int)heading);
    transfer_paused = true;
    payload_result_t result = PAYLOAD_NOT_CLOSE;
    uint32_t moved = 0;
    unsigned floor_checks = 0;
    while (moved < limit && (!sensor || floor_checks < LEAVE_MAT_FLOOR_CHECKS)) {
        uint32_t actual, block = limit - moved < LEAVE_MAT_BLOCK_MS ? limit - moved : LEAVE_MAT_BLOCK_MS;
        result = translate_block(false, block, heading, 0, 0, false, &actual);
        if (result != PAYLOAD_NOT_CLOSE) break;
        moved += actual ? actual : block;
        floor_checks = colour_on_mat() ? 0u : floor_checks + 1u;
    }
    if (result == PAYLOAD_NOT_CLOSE && sensor) {
        if (floor_checks < LEAVE_MAT_FLOOR_CHECKS)
            report_printf("ROUTE: the colour sensor still sees the mat after %lu ms; continuing anyway\n",
                          (unsigned long)moved);
        uint32_t actual;
        result = translate_block(false, LEAVE_MAT_CLEARANCE_MS, heading, 0, 0, false, &actual);
        moved += actual;
    }
    transfer_paused = false;
    if (result == PAYLOAD_NOT_CLOSE)
        report_printf("ROUTE: off the mat after %lu ms reversing\n", (unsigned long)moved);
    return result;
}

// From just off the visited mat, facing it. The next beacon may be anywhere
// except beyond the visited one, so probe straight back and both sides, the
// same way as the side probe: drive out, turn to face the same heading every
// time (the robot body changes the reading by up to 17 dB with heading),
// measure, and drive back. Then follow the strongest.
static payload_result_t start_next_leg(float facing, follow_result_t *followed)
{
    static const float offsets[LEG_DIRECTIONS] = {180.0f, 90.0f, -90.0f};
    line_pair_t readings[LEG_DIRECTIONS];
    unsigned best = 0;
    report_printf("ROUTE: looking for %s: probe %+d, %+d and %+d from heading %d, %u ms each, measuring facing %d\n",
                  target_name(), (int)offsets[0], (int)offsets[1], (int)offsets[2], (int)facing,
                  LINE_SIDE_PROBE_MS, (int)facing);
    for (unsigned i = 0; i < LEG_DIRECTIONS; ++i) {
        float heading = line_normalise_heading(facing + offsets[i]);
        bool mat;
        payload_result_t result = probe_one_side(facing, heading, &readings[i], &mat);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (mat) {
            followed->end = FOLLOW_MAT;
            report_printf("ROUTE: reached the %s mat while probing heading %d\n",
                          colour_mat_name(target_mat()), (int)heading);
            return PAYLOAD_NOT_CLOSE;
        }
        if (line_pair_sum(readings[i]) > line_pair_sum(readings[best])) best = i;
    }
    // Clear when the strongest beats both others by the usual 3 dB margin, or
    // on average when the batches disagree (side_compare).
    bool clear = true;
    for (unsigned i = 0; i < LEG_DIRECTIONS; ++i) {
        bool by_average;
        if (i != best && line_side_compare(readings[i], readings[best], &by_average) <= 0) clear = false;
    }
    float heading = line_normalise_heading(facing + offsets[best]);
    report_printf("ROUTE: %s %+d: %d/%d dBm, %+d: %d/%d dBm, %+d: %d/%d dBm; follow %+d, heading %d (%s)\n",
                  target_name(),
                  (int)offsets[0], rssi_decode(readings[0].first.code), rssi_decode(readings[0].second.code),
                  (int)offsets[1], rssi_decode(readings[1].first.code), rssi_decode(readings[1].second.code),
                  (int)offsets[2], rssi_decode(readings[2].first.code), rssi_decode(readings[2].second.code),
                  (int)offsets[best], (int)heading, clear ? "clearly strongest" : "strongest, not by 3 dB");
    return follow_direction(heading, 0, followed);
}

// The robot is off the visited mat, facing it, with the next target selected.
static bool approach_next_beacon(float facing)
{
    report_printf("APPROACH: probe three directions away from the visited beacon, follow the strongest; then as before\n");
    wifi_sample_t initial;
    payload_result_t result = wait_for_wifi(&initial);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    follow_result_t followed = {.end = FOLLOW_MAT};
    result = start_next_leg(facing, &followed);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    return approach_loop(followed);
}

// The whole run: the first beacon, then (route builds) the next one.
static bool run_route(void)
{
    const uint8_t first = target_id();
    if (!route_next) return approach_beacon();
    if (route_start_on_mat) {
        // Test build: set up what approach_beacon would have, without its transfer.
        profile_id = 0;
        wifi_quiescent = true;
        wifi_sampler_init(run_can_continue);
        begin_leg();
        report_printf("ROUTE: test start on the %s mat's arrival ring, facing %s; its transfer is skipped\n",
                      colour_mat_name(target_mat()), target_name());
        if (!motion_learn_direction()) return false;
    } else {
        announce_target("leg 1/2:");
        if (!approach_beacon()) return false;
        // Three rising beeps for the first beacon; the run goes on.
        completion_sound();
        if (motion_stop_requested()) return false;
    }
    const float facing = line_normalise_heading(motion_heading());
    begin_leg(); // the move off the mat counts toward the next beacon's budget
    if (leave_mat(facing) != PAYLOAD_NOT_CLOSE) return false;
    select_target(route_next, target_mat_of(first));
    announce_target("leg 2/2:");
    return approach_next_beacon(facing);
}

#if COLOUR_SCAN_TEST
#define SCAN_ROUNDS 2u

// Two full turns on the spot, logging the colour under the sensor at each
// step and the estimated direction of the mat centre.
static bool colour_scan_run(void)
{
    run_started_ms = run_now_ms();
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
        unsigned lighter_mats;
        int farthest_rings;
        // Diagnostic firmware never initializes the radio: checkpoint directly.
        scan_status_t status = colour_scan_turn(origin, round, &est, &lighter_mats,
                                               &farthest_rings, run_can_continue, report_checkpoint);
        if (status == SCAN_READ_FAILED) report_printf("COLOUR SCAN FAILED: sensor read error\n");
        if (status != SCAN_DONE) return false;
        if (!est.valid) all_valid = false;
        if (!motion_turn_to_heading(origin)) return false;
    }
    return all_valid;
}
#endif


uint8_t robot_run_next_beacon(void) { return route_next; }
bool robot_run_starts_on_mat(void) { return route_start_on_mat; }
void robot_run_in_range(void) { on_in_range(); }
void robot_run_completion_cue(void) { completion_sound(); }
bool robot_run_execute(void)
{
    bool completed = run_route();
    motion_stop();
    return completed;
}
#if COLOUR_SCAN_TEST
bool robot_run_colour_scan(void) { return colour_scan_run(); }
#endif
