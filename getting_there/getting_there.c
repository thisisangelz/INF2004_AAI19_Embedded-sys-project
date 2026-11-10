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
#include "target_beacon.h"

#define MAX_RUN_MS 900000u
#define MAX_MISSING_SAMPLES 3
#define MAX_PAYLOAD_ATTEMPTS 3
#define START_RELEASE_TIMEOUT_MS 10000u
#define BLUETOOTH_START_TIMEOUT_MS 10000u
// Reserve polling/I2C overrun for every translation block, including returns.
#define BLOCK_RESERVE_MS 25u

static uint32_t run_started_ms, translation_ms;
static bool wifi_quiescent = true;
static struct { uint8_t learned : 1; } direction;
#if ROUTE_BEACON_COUNT
#define ROUTE_DEPARTURE_MS 3000u
#define ROUTE_CLEARANCE_MS 250u
#define ROUTE_COMPLETE_MASK ((1u << ROUTE_BEACON_COUNT) - 1u)
static struct {
    uint8_t completed_mask : ROUTE_BEACON_COUNT;
    uint8_t departing : 1;
    uint8_t incoming_valid : 1;
    uint8_t incoming_forwards : 1;
} route;
_Static_assert(sizeof(route) == 1, "route state fits in one byte");
static float incoming_heading;
#endif
static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static bool run_can_continue(void)
{
    if (motion_stop_requested()) { motion_stop(); return false; }
#if ROUTE_BEACON_COUNT
    if (!colour_sensor_available()) {
        motion_stop(); fault_raise(FAULT_COLOUR_SENSOR); return false;
    }
#endif
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
// Single-target compatibility: mat steering inferred the centre without a
// verified arrival ring. Route builds stop instead of applying this override.
static bool mat_centre_override;

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
            bool on_mat = colour_arrival_check() || mat_centre_override;
#if ROUTE_BEACON_COUNT
            if (!colour_sensor_available()) {
                fault_raise(FAULT_COLOUR_SENSOR);
                report_printf("FAILED: route colour sensor unavailable; BLE fallback is disabled\n");
                return PAYLOAD_FATAL;
            }
#endif
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
                report_printf("ARRIVAL: %s; BLE only needs %s >= %d dBm\n",
                              mat_centre_override ? "inferred centre override (arrival ring unverified)" :
                              "colour arrival ring verified after lighter ring",
                              target_beacon_name(), COLOUR_ARRIVAL_BLE_DBM);
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
            report_printf("%s transfer not confirmed; stationary connection attempt %d/%d, proximity retained\n", target_beacon_name(),
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
        report_printf("%s Wi-Fi missing; checking BLE while remaining stopped\n", target_beacon_name());
        payload_result_t result = check_payload();
        if (result == PAYLOAD_COMPLETE || result == PAYLOAD_FATAL) return result;
        if (!motion_pause_ms(500)) return PAYLOAD_FATAL;
    }
    report_printf("FAILED: %s missing from three fresh Wi-Fi samples; no blind movement\n", target_beacon_name());
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
#if ROUTE_BEACON_COUNT
    // Retain enough of this leg's 60 s budget to depart after its transfer.
    if (!route.departing && target_beacon_id() < ROUTE_BEACON_COUNT)
        required += translation_cost(ROUTE_DEPARTURE_MS);
#endif
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
#if ROUTE_BEACON_COUNT
    // Arrival can be confirmed on a reverse return too. Departure must undo
    // the most recent powered approach direction, never repeat it inward.
    incoming_heading = heading;
    route.incoming_forwards = forwards;
    route.incoming_valid = true;
#endif
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

// A foreign mat rejects this trial; it is not evidence that the target signal
// declined. Its caller retreats before choosing a different heading.
typedef enum { FOLLOW_NO_GAIN, FOLLOW_DECLINE, FOLLOW_MAT, FOLLOW_FOREIGN_MAT } follow_end_t;
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
// farthest_rings: the most rings any reading was from its own mat's arrival
// ring (SCAN_OFF_MAT if a reading was not on a mat). Small means every
// direction around the robot is inner rings: the robot is at the centre.
#define SCAN_OFF_MAT 1000
static scan_status_t colour_scan_turn(float origin, unsigned round, colour_scan_result_t *est,
                                      unsigned *lighter_mats, int *farthest_rings)
{
    int angles[SCAN_STEPS];
    colour_reading_t readings[SCAN_STEPS];
    *lighter_mats = 0;
    *farthest_rings = 0;
    *est = (colour_scan_result_t){.valid = false, .max_depth = -1};
    for (unsigned i = 0; i < SCAN_STEPS; ++i) {
        if (!run_can_continue()) return SCAN_STOPPED;
        angles[i] = (int)i * SCAN_STEP_DEG;
        if (!motion_turn_to_heading(origin + (float)angles[i]) || !motion_pause_ms(200)) return SCAN_STOPPED;
        if (!scan_read(&readings[i])) {
            report_printf("COLOUR SCAN: sensor read error at %d degrees\n", angles[i]);
            return SCAN_READ_FAILED;
        }
        colour_mat_t mat;
        colour_zone_t zone = colour_classify_mat(&readings[i], &mat);
        if (colour_lighter_zone(mat, zone)) *lighter_mats |= 1u << mat;
        int rings = mat == COLOUR_MAT_NONE ? SCAN_OFF_MAT :
            colour_zone_depth(colour_arrival_zone(mat)) - colour_zone_depth(zone);
        if (rings > *farthest_rings) *farthest_rings = rings;
        report_printf("COLOUR_SCAN,round=%u,angle=%d,heading=%d,zone=%s,mat=%s,depth=%d,c=%u,r=%u,g=%u,b=%u,gr=%u,br=%u\n",
                      round, angles[i], (int)line_normalise_heading(motion_heading()),
                      colour_zone_name(zone), colour_mat_name(mat), colour_zone_depth(zone),
                      readings[i].c, readings[i].r, readings[i].g, readings[i].b,
                      colour_gr_x1000(&readings[i]), colour_br_x1000(&readings[i]));
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
    // White edges are deliberately ambiguous in the classifier. Require a
    // coloured ring of the active target before steering toward a mat centre.
    return colour_target_ring_seen() && !colour_foreign_mat_seen() && mat_scans_used < MAT_MAX_SCANS;
}

// On the arrival ring but AP1 failed even the loose BLE gate: never drive on
// into the beacon box. Retry while stopped, then stop the run.
static payload_result_t mat_wait_on_arrival(void)
{
    for (unsigned retry = 1; retry <= MAT_ARRIVED_RETRIES; ++retry) {
        report_printf("MAT: on %s or darker but %s BLE not in range; stationary retry %u/%u\n",
                      COLOUR_ARRIVAL_TEXT, target_beacon_name(), retry, MAT_ARRIVED_RETRIES);
        if (!motion_pause_ms(1000)) return PAYLOAD_FATAL;
        payload_result_t result = check_payload();
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if (!colour_arrived) return PAYLOAD_NOT_CLOSE; // reading changed: keep steering
    }
    fault_raise(FAULT_PAYLOAD);
    report_printf("FAILED: on the mat centre but %s BLE never came in range; no further movement\n", target_beacon_name());
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
    motion_stop();
    report_printf("MAT: %s; inferred centre, arrival ring is NOT proven\n", why);
#if ROUTE_BEACON_COUNT
    fault_raise(FAULT_COLOUR_SENSOR);
    report_printf("FAILED: route requires genuine colour arrival; stationary stop at inferred centre\n");
    return PAYLOAD_FATAL;
#else
    mat_centre_override = true;
    payload_result_t result = check_payload();
    if (result != PAYLOAD_NOT_CLOSE) return result;
    return mat_wait_on_arrival(); // FATAL after its retries: never drives again
#endif
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
        scan_status_t status = colour_scan_turn(origin, scan, &est, &lighter_mats, &farthest_rings);
        if (status == SCAN_STOPPED) return PAYLOAD_FATAL;
        if (status == SCAN_READ_FAILED) {
            report_printf("MAT: scan %u could not read the colour sensor; restoring entry heading\n", scan);
            return motion_turn_to_heading(origin) ? PAYLOAD_NOT_CLOSE : PAYLOAD_FATAL;
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
            report_printf("MAT: scan %u found no ring to steer toward; restoring entry heading\n", scan);
            return motion_turn_to_heading(origin) ? PAYLOAD_NOT_CLOSE : PAYLOAD_FATAL;
        }
        float heading = line_normalise_heading(origin + (float)est.vector_angle);
        report_printf("MAT: scan %u points %d degrees from its start, heading %d\n",
                      scan, est.vector_angle, (int)heading);
        if (!motion_turn_to_heading(heading)) return PAYLOAD_FATAL;
#if ROUTE_BEACON_COUNT
        // A fresh valid colour gradient followed by a successful turn gives
        // an observed inward heading even if arrival occurs by rotation.
        // Reverse outward without another turn; this is not exact path replay.
        incoming_heading = heading;
        route.incoming_forwards = true;
        route.incoming_valid = true;
        report_printf("DEPARTURE BASIS: fresh colour gradient, inward heading=%d\n", (int)heading);
#endif
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
    if (colour_foreign_mat_seen()) {
        report_printf("DIP: foreign coloured mat at stopped recheck; no forward probes, returning to retained best\n");
        return PAYLOAD_NOT_CLOSE;
    }
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
        if (colour_foreign_mat_seen()) {
            // This dip already requires a return unless recovery is proven.
            // Do not spend the second forward probe entering another mat.
            report_printf("DIP: foreign coloured mat during probe; no further probing, returning to retained best\n");
            return PAYLOAD_NOT_CLOSE;
        }
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

// Every new heading measures its own baseline. A zero argument uses the same
// bounded acquisition as any other trial; it never permits flat RSSI drift.
static payload_result_t follow_direction(float heading, uint32_t trial_ms, follow_result_t *followed)
{
    if (!trial_ms) trial_ms = LINE_DIRECTION_TEST_MS;
    *followed = (follow_result_t){.heading = line_normalise_heading(heading)};
    profile_count = 0;
    ++profile_id;
    // One long beep each time a new search direction starts.
    cue_play(CUE_PASS);
    if (!motion_turn_to_heading(heading)) return PAYLOAD_FATAL;
#if ROUTE_BEACON_COUNT
    // The front sensor moved during the turn. Refresh its evidence before
    // driving; a previous heading's colour reading is not a clearance check.
    payload_result_t turned = check_payload();
    if (turned != PAYLOAD_NOT_CLOSE) return turned;
#endif
    wifi_sampler_set_position(heading, 0, true);
    line_pair_t best;
    payload_result_t result = measure_pair(&best);
    if (result != PAYLOAD_NOT_CLOSE) return result;
    const line_pair_t baseline = best;
    followed->baseline = baseline;
    if (!record_point(heading, 0, best)) return PAYLOAD_FATAL;
    if (colour_foreign_mat_seen()) {
        followed->end = FOLLOW_FOREIGN_MAT;
        report_printf("FOLLOW: foreign coloured mat under sensor; rejecting heading without forward movement\n");
        return PAYLOAD_NOT_CLOSE;
    }
    uint32_t position = 0, best_position = 0, segment_start = 0;
    uint32_t last_decline_position = 0, best_wall_ms = now_ms();
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
        if (followed->progress && (position - best_position >= LINE_NO_BEST_MS ||
            (uint32_t)(now_ms() - best_wall_ms) >= LINE_NO_BEST_WALL_MS)) {
            report_printf("FOLLOW: no-new-best watchdog; return to retained best before reacquiring\n");
            followed->end = FOLLOW_DECLINE;
            result = return_along_line(position, best_position, heading);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            followed->position = best_position;
            line_pair_t returned;
            result = measure_pair(&returned);
            if (result != PAYLOAD_NOT_CLOSE) return result;
            followed->consistent = line_return_consistent(best, returned);
            return PAYLOAD_NOT_CLOSE;
        }
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
        if (colour_foreign_mat_seen()) {
            followed->end = FOLLOW_FOREIGN_MAT;
            report_printf("FOLLOW: foreign coloured mat at %lu powered ms; stop and retreat before trying another heading\n",
                          (unsigned long)position);
            return PAYLOAD_NOT_CLOSE;
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
                                  followed->consistent ? "consistent with best point" : "inconsistent with best point; reacquiring from actual return");
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
        bool stronger = (trend > 0 && line_compare(baseline, current) > 0) || (followed->progress &&
            line_compare(baseline, current) > 0 && line_confirmed_gain(best, current) > 0);
        if (stronger) {
            dip_active = false;
            decline_positions = 0;
            if (!followed->progress)
                report_printf("FOLLOW: first confirmed improvement; continuing without other direction probes\n");
            best = current;
            best_position = position;
            best_wall_ms = now_ms();
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
        if (!followed->progress && position >= trial_ms) {
            if (trial_ms == LINE_DIRECTION_TEST_MS && line_confirmed_gain(baseline, current) > 0) {
                trial_ms = LINE_DIRECTION_EXTENDED_MS;
                report_printf("TRIAL: positive but unconfirmed gain; bounded extension to %lu powered ms\n", (unsigned long)trial_ms);
                continue;
            }
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

// Complete the cardinal search, then try the four intervening diagonals. The
// same short displacement can show <3 dB gain on both axes while showing a
// supported gain toward a diagonal. Do not lower the confirmation margin.
// Every candidate has its own measured baseline and reserves return travel.
#define SEARCH_DIRECTIONS 7u
static payload_result_t check_directions(float origin, follow_result_t *followed, bool *found)
{
    static const float offsets[SEARCH_DIRECTIONS] = {-90.0f, 90.0f, 180.0f, 45.0f, -45.0f, 135.0f, -135.0f};
    *found = false;
    report_printf("DIRECTION SEARCH: both perpendiculars, straight back, then four diagonals; origin %d degrees; unchanged confirmation margin\n",
                  (int)origin);
    for (unsigned i = 0; i < SEARCH_DIRECTIONS; ++i) {
        float heading = line_normalise_heading(origin + offsets[i]);
        report_printf("DIRECTION TRIAL: %u/%u, offset=%d, heading=%d\n",
                      i + 1u, SEARCH_DIRECTIONS, (int)offsets[i], (int)heading);
        payload_result_t result = follow_direction(heading, LINE_DIRECTION_TEST_MS, followed);
        if (result != PAYLOAD_NOT_CLOSE) return result;
        if ((followed->progress && followed->end != FOLLOW_FOREIGN_MAT) || followed->end == FOLLOW_MAT) {
            *found = true;
            return PAYLOAD_NOT_CLOSE;
        }
        if (followed->end == FOLLOW_NO_GAIN || followed->end == FOLLOW_FOREIGN_MAT) {
            report_printf("TRIAL: %s; return to its starting point before the next direction\n",
                          followed->end == FOLLOW_FOREIGN_MAT ? "foreign mat rejected" : "no confirmed gain");
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
    translation_ms = profile_id = 0;
    profile_count = 0;
#if ROUTE_BEACON_COUNT
    route.incoming_valid = false;
#endif
    wifi_quiescent = true;
    wifi_sampler_init(run_can_continue);
    colour_arrival_reset();
    step_on_mat = false;
    mat_scans_used = 0;
    mat_centre_override = colour_arrived = false;
    payload_result_t result = check_payload();
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    wifi_sample_t initial;
    result = wait_for_wifi(&initial);
    if (result == PAYLOAD_COMPLETE) return true;
    if (result == PAYLOAD_FATAL) return false;
    if (!direction.learned) {
        if (!motion_learn_direction()) return false;
        direction.learned = true;
    }
    follow_result_t followed = {.end = FOLLOW_MAT};
    report_printf("APPROACH: bounded initial heading, cardinal then diagonal trials; reject foreign mats, return before each new baseline; steer only on the target's coloured rings\n");
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
        if (followed.end == FOLLOW_NO_GAIN || followed.end == FOLLOW_FOREIGN_MAT) {
            report_printf("TRIAL: %s heading returns to start before trying others\n",
                          followed.end == FOLLOW_FOREIGN_MAT ? "foreign-mat" : "initial/current no-gain");
            result = return_along_line(followed.position, 0, followed.heading);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            line_pair_t returned;
            result = measure_pair(&returned);
            if (result == PAYLOAD_COMPLETE) return true;
            if (result == PAYLOAD_FATAL) return false;
            followed.consistent = line_return_consistent(followed.baseline, returned);
        } else if (followed.end != FOLLOW_DECLINE) {
            fault_raise(FAULT_NO_DIRECTION);
            report_printf("FAILED: unexpected follow termination\n");
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
            report_printf("FAILED: no direction confirmed improvement after cardinal and diagonal trials within the travel/run limits\n");
            return false;
        }
    }
    return false;
}

#if ROUTE_BEACON_COUNT
// Drive opposite the observed approach direction, without another turn.
// This path does NOT use translate_block: no payload checks or BLE scan are
// permitted, and the old identity remains active throughout departure.
static bool depart_previous_mat(void)
{
    motion_stop();
    if (!run_can_continue() || !wifi_sampler_quiesce() || !payload_client_quiescent()) return false;
    float error = line_normalise_heading(motion_heading() - incoming_heading + 180.0f) - 180.0f;
    if (!route.incoming_valid || error > 5.0f || error < -5.0f) {
        fault_raise(FAULT_HEADING);
        report_printf("FAILED: no aligned incoming path for departure; no turn beside previous box\n");
        return false;
    }
    route.departing = true;
    uint32_t powered = 0, clearance = 0;
    bool floor_seen = false;
    report_printf("DEPART: target=%u, direction=%s, observed approach heading=%d, limit=%u powered ms, no BLE\n",
                  target_beacon_id(), route.incoming_forwards ? "reverse" : "forward",
                  (int)incoming_heading, ROUTE_DEPARTURE_MS);
    while (powered < ROUTE_DEPARTURE_MS && run_can_continue()) {
        uint32_t remaining = ROUTE_DEPARTURE_MS - powered;
        uint32_t block = remaining > LINE_MAT_BLOCK_MS ? LINE_MAT_BLOCK_MS : remaining;
        // Reserve polling overrun inside the departure's hard powered limit.
        if (block <= BLOCK_RESERVE_MS) break;
        if (block + BLOCK_RESERVE_MS > remaining) block = remaining - BLOCK_RESERVE_MS;
        if (!reserve_translation(translation_cost(block)) || !save_checkpoint() ||
            !motion_drive_start(!route.incoming_forwards, block, incoming_heading)) break;
        bool ok = true;
        while (motion_drive_active()) {
            if (!motion_drive_update() || !run_can_continue()) { ok = false; break; }
            if (motion_drive_active()) sleep_ms(5);
        }
        motion_stop();
        uint32_t actual = motion_drive_elapsed_ms();
        powered += actual;
        translation_ms += actual;
        bool finished = motion_drive_finish();
        if (!ok || !finished || !actual || actual > block + BLOCK_RESERVE_MS ||
            powered > ROUTE_DEPARTURE_MS || translation_ms > LINE_TRANSLATION_MS) break;
        bool floor;
        if (!colour_floor_check(&floor) || !run_can_continue()) break;
        // Each floor check is three fresh readings classified across ALL mats;
        // a different mat and a white edge do not count as clear floor.
        if (!floor) { floor_seen = false; clearance = 0; }
        else if (!floor_seen) { floor_seen = true; clearance = 0; }
        else clearance += actual;
        report_printf("DEPART: powered_ms=%lu,floor=%u,clearance_ms=%lu\n",
                      (unsigned long)powered, floor ? 1u : 0u, (unsigned long)clearance);
        if (floor_seen && clearance >= ROUTE_CLEARANCE_MS) {
            route.departing = false;
            return save_checkpoint() && run_can_continue();
        }
    }
    motion_stop();
    route.departing = false;
    if (!fault_mask()) fault_raise(FAULT_COLOUR_SENSOR);
    report_printf("FAILED: old mat not safely cleared within departure bound; target unchanged\n");
    return false;
}

static bool approach_route(void)
{
    route.completed_mask = route.departing = route.incoming_valid = 0;
    if (!run_can_continue()) return false;
    // Sampler has no scan at startup; initialise it before the first quiesce.
    wifi_sampler_init(run_can_continue);
    if (!wifi_sampler_quiesce() || !payload_client_reset_target_qualification(1)) return false;
    for (;;) {
        uint8_t id = target_beacon_id();
        colour_set_target_mat(target_beacon_mat());
        report_printf("ROUTE: target=%u\n", id);
        report_printf("ROUTE: leg=%s,mat=%s,translation_limit_ms=%u,mission_limit_ms=%u\n",
                      target_beacon_name(), colour_mat_name(target_beacon_mat()), LINE_TRANSLATION_MS, MAX_RUN_MS);
        if (!save_checkpoint() || !approach_beacon()) return false;
        motion_stop();
        // approach_beacon returns success only from PAYLOAD_COMPLETE. Refuse
        // advancement if cleanup or persistence subsequently failed.
        if (!payload_client_quiescent() || !save_checkpoint() || !run_can_continue()) return false;
        route.completed_mask |= (uint8_t)(1u << (id - 1u));
        report_printf("ROUTE: confirmed=%u,mask=%u\n", id, (unsigned)route.completed_mask);
        report_printf("ROUTE: leg_result=%s,CRC+finalACK verified,translation_ms=%lu\n",
                      target_beacon_name(), (unsigned long)translation_ms);
        // The callback already logged the full verified reply; persist it and
        // the confirmation before the next target can overwrite that buffer.
        if (!save_checkpoint()) return false;
        if (route.completed_mask == ROUTE_COMPLETE_MASK) return true;
        if (!depart_previous_mat() || !run_can_continue() || !wifi_sampler_quiesce() ||
            !payload_client_reset_target_qualification((uint8_t)(id + 1u))) return false;
        // No reset of gyro bias, integrated heading, STOP latch, mission clock
        // or report journal. approach_beacon resets only per-leg navigation.
    }
}
#endif

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
        unsigned lighter_mats;
        int farthest_rings;
        scan_status_t status = colour_scan_turn(origin, round, &est, &lighter_mats, &farthest_rings);
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
    report_printf("Getting there | bounded eight-heading acquisition, foreign-mat guard, no-new-best watchdog | %s\n",
                  ROUTE_BEACON_COUNT == 2 ? "AP1 -> AP2 route" :
                  ROUTE_BEACON_COUNT == 3 ? "AP1 -> AP2 -> AP3 route" : target_beacon_name());
#if COLOUR_SCAN_TEST
    report_printf("TEST BUILD: colour scan on the spot after calibration; no radio, no approach\n");
    report_printf("Colour mats: pink, blue and purple all recognised\n");
#else
    // Each beacon has its own mat; the other two read as floor.
    colour_set_target_mat(target_beacon_mat());
    report_printf("Colour mat: %s (%s); other mats read as floor\n",
                  colour_mat_name(target_beacon_mat()), target_beacon_name());
#endif
    report_printf("Flash layout: 2 MiB physical; 1528 KiB firmware, 8 KiB Bluetooth, 512 KiB saved log\n");
    report_printf("Before GP20: GP21 replays previous run. After GP20: GP21 STOP, latched until reset.\n");
    report_printf("Move using Wi-Fi direction; on the mat, steer by colour scans\n");
    report_printf("Transfer only on %s or darker after a lighter ring, with BLE >= %d dBm\n",
                  COLOUR_ARRIVAL_TEXT, COLOUR_ARRIVAL_BLE_DBM);
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
            else report_printf("Bluetooth ready; beginning %s approach.\n", target_beacon_name());
        }
    }
    // approach_beacon initialises the sampler before its first checkpoint.
    if (ready && !motion_stop_requested()) {
        run_started_ms = now_ms();
        direction.learned = false;
        motion_set_run_limit(run_started_ms, MAX_RUN_MS);
        payload_client_set_continue_callback(run_can_continue);
#if ROUTE_BEACON_COUNT
        bool completed = approach_route();
#else
        bool completed = approach_beacon();
#endif
        motion_stop();
        if (completed) completion_sound();
        else cue_play(CUE_FAULT);
#if ROUTE_BEACON_COUNT
        report_printf("RESULT: %s\n", completed && route.completed_mask == ROUTE_COMPLETE_MASK ?
                      "ROUTE COMPLETE" : "ROUTE STOPPED");
        report_printf("ROUTE: final_mask=%u\n", (unsigned)route.completed_mask);
#else
        report_printf("RESULT: %s\n", completed ? "PAYLOAD CONFIRMED" : "STOPPED WITHOUT PAYLOAD");
#endif
    } else {
        cue_play(CUE_FAULT);
        report_printf("RESULT: STARTUP FAILED OR STOPPED\n");
    }
#endif
    motion_stop();
    // Also quiesces any scan/cleanup still pending after a fatal radio failure.
    if (radio_initialised) cyw43_arch_deinit();
    report_faults();
    if (fault_test(FAULT_STOP_LATCHED)) report_printf("GP21 stop latched\n");
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
