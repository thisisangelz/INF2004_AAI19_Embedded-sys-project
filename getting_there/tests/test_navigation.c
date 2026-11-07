#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#define main firmware_main
#include "../getting_there.c"
#undef main
#include "fixtures/v14_wifi_dip.h"

static uint64_t sim_time, scan_finish_us;
static bool stopped, motors_on, wifi_missing, fail_drive, fail_turn, stop_during_pause;
static bool complete_first, fail_scan, scan_active, storage_failed;
static bool irregular_scans, stop_on_reverse, raw_script_mode;
static const uint32_t *powered_script;
static unsigned powered_script_length;
static int wifi_calls, forwards_count, backwards_count, ble_calls, sign_checks;
static int tone_starts, tone_stops, retry_mode, complete_after_step, fail_at_ble_call, retry_calls;
static int complete_at_ble_call;
static unsigned scan_delay_ms, ble_delay_ms, stop_at_scan, stop_at_step, fail_at_step, checkpoint_calls;
static float current_heading;
static const int *script;
static size_t script_length;
static int default_rssi;
static bool model;
static double robot_x, robot_y, beacon_x, beacon_y;
// Model weak patch: AP1 reads 12 dB lower while dead_from <= x < dead_to.
static double dead_from, dead_to;
// Closest the colour sensor came to the mat centre while driving (model).
static double model_min_sensor_d;
static double model_sensor_d(void);
// 7 November run conditions: inner rings read one ring lighter, so arrival
// never shows; and the beacon box is solid, so the robot cannot drive the
// sensor closer than MODEL_BOX_BLOCK to the centre (it pushes in place).
static bool model_dim_inner, model_box_solid;
static unsigned model_blocked_blocks;
#define MODEL_BOX_BLOCK 0.045
static struct { bool forwards; float heading; uint32_t ms; unsigned scans_before; } moves[512];
static unsigned move_count;
static uint32_t mock_duration, mock_elapsed;
static bool mock_forwards, mock_pending;
static char log_text[262144];
static size_t log_length;
static bool ui_test, ui_armed, ui_finished, ui_radio, ui_log_failure;
static unsigned ui_stage, ui_replays, ui_begins, ui_calibrations;
static unsigned ui_init_calls, ui_fail_init_count;
static bool ui_hold_start, ui_cal_failure, ui_stop_calibration, ui_stop_on_arm;
static bool ui_bt_failure, ui_bt_timeout;
static int ui_radio_error;
static jmp_buf ui_exit;
static int (*scan_callback)(void *, const cyw43_ev_scan_result_t *);
static void *scan_context;
static int scan_rssi;
int cyw43_state;

uint64_t time_us_64(void) { return sim_time; }
void sleep_ms(uint32_t ms)
{
    sim_time += (uint64_t)ms * 1000;
    if (ui_test && ui_finished) longjmp(ui_exit, 1);
    if (ui_test && sim_time > 120000000u) longjmp(ui_exit, 2);
    if (ui_test && ui_stage == 1 && ui_replays) ui_stage = 2;
}
void report_printf(const char *format, ...)
{
    assert(!motors_on);
    va_list args;
    va_start(args, format);
    int count = vsnprintf(log_text + log_length, sizeof(log_text) - log_length, format, args);
    va_end(args);
    if (count > 0 && (size_t)count < sizeof(log_text) - log_length) log_length += (size_t)count;
    if (ui_test && strstr(format, "Run ended.")) ui_finished = true;
}
void report_flush_usb(void) { assert(!motors_on); }
void report_init(void) { }
void report_replay(void)
{
    if (ui_test) { assert(!ui_armed && !ui_radio && !ui_begins); ++ui_replays; }
}
bool report_begin(void)
{
    if (ui_test) { assert(ui_armed && !ui_radio && ui_replays == 1); ++ui_begins; ui_stage = 3; }
    return !ui_log_failure;
}
bool report_checkpoint(void) { assert(!motors_on && !scan_active); ++checkpoint_calls; return !storage_failed; }
bool report_finish(void)
{
    if (ui_test) { assert(!motors_on && !ui_radio); ui_finished = true; }
    return !ui_log_failure;
}
bool report_ok(void) { return !storage_failed; }
void stdio_init_all(void) { }
bool stdio_usb_connected(void) { return false; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
// Sound cues: tone_starts counts completion cues, tone_stops counts cue_stop().
static unsigned cue_counts[CUE_FAULT + 1];
static uint64_t cue_end;
void cues_init(void) { }
void cue_play(robot_cue_t cue)
{
    assert(!motors_on);
    ++cue_counts[cue];
    if (cue == CUE_COLLECTED) ++tone_starts;
    // The completion pattern lasts 910 ms; looping cues last until stopped.
    uint64_t length = cue == CUE_COLLECTED ? 910000 :
                      cue == CUE_CALIBRATING || cue == CUE_IN_RANGE ? UINT64_MAX / 2 : 100000;
    cue_end = sim_time + length;
}
void cue_stop(void) { ++tone_stops; cue_end = sim_time; }
bool cue_busy(void) { return sim_time < cue_end; }
static void (*close_callback)(void);
void payload_client_set_close_callback(void (*callback)(void)) { close_callback = callback; }
bool motion_init(void)
{
    if (ui_test && ++ui_init_calls <= ui_fail_init_count) { fault_raise(FAULT_IMU); return false; }
    fault_clear(FAULT_IMU);
    return true;
}
bool motion_calibrate(void)
{
    if (ui_test) { assert(ui_armed && !ui_radio); ++ui_calibrations; }
    sleep_ms(2000);
    if (ui_stop_calibration) { stopped = true; fault_raise(FAULT_STOP_LATCHED); }
    if (ui_cal_failure) fault_raise(FAULT_IMU);
    return !stopped && !ui_cal_failure;
}
bool motion_learn_direction(void) { ++sign_checks; return !stopped; }
bool motion_turn_to_heading(float heading)
{ assert(!motors_on && !scan_active); current_heading = heading; return !stopped && !fail_turn; }
float motion_heading(void) { return current_heading; }
bool motion_pause_ms(uint32_t ms)
{
    if (stop_during_pause) { sleep_ms(100); stopped = true; return false; }
    sleep_ms(ms); return !stopped;
}
bool motion_stop_requested(void) { return stopped; }
bool motion_start_pressed(void) { return ui_test && (ui_stage == 2 || (ui_hold_start && ui_begins)); }
bool motion_replay_pressed(void) { return ui_test && ui_stage == 1; }
void motion_arm_stop(void)
{
    ui_armed = true;
    if (ui_stop_on_arm) { stopped = true; fault_raise(FAULT_STOP_LATCHED); }
}
void cyw43_arch_deinit(void) { ui_radio = scan_active = false; }
void motion_stop(void) { motors_on = false; }
bool motion_drive_start(bool forwards, uint32_t ms, float heading)
{
    assert(!motors_on && !stopped && ms <= LINE_BLOCK_MS && move_count < 512);
    moves[move_count].forwards = forwards;
    moves[move_count].heading = heading;
    moves[move_count].scans_before = (unsigned)wifi_calls;
    moves[move_count++].ms = ms;
    if (forwards) ++forwards_count; else ++backwards_count;
    mock_duration = move_count <= powered_script_length ? powered_script[move_count-1u] : ms;
    mock_elapsed = 0; mock_forwards = forwards;
    mock_pending = motors_on = true;
    return true;
}
bool motion_drive_update(void)
{
    if (stop_at_step && (unsigned)forwards_count >= stop_at_step) stopped = true;
    if (fail_at_step && (unsigned)forwards_count >= fail_at_step) {
        fail_drive = true;
        fault_raise(FAULT_IMU);
    }
    if (stop_on_reverse && !mock_forwards) stopped = true;
    if (fail_drive || stopped) { motors_on = false; return false; }
    uint32_t delta = mock_duration - mock_elapsed;
    if (delta > 5) delta = 5;
    mock_elapsed += delta;
    if (model) {
        double distance = (mock_forwards ? 1 : -1) * 0.20 * delta / 750.0;
        double radians = current_heading * 3.141592653589793 / 180.0;
        double old_x = robot_x, old_y = robot_y;
        robot_x += distance * cos(radians); robot_y += distance * sin(radians);
        if (model_box_solid && model_sensor_d() < MODEL_BOX_BLOCK) {
            robot_x = old_x; robot_y = old_y; // against the box: wheels slip
            if (mock_elapsed + delta >= mock_duration) ++model_blocked_blocks;
        }
        if (model_sensor_d() < model_min_sensor_d) model_min_sensor_d = model_sensor_d();
    }
    if (mock_elapsed >= mock_duration) motors_on = false;
    return true;
}
bool motion_drive_active(void) { return motors_on; }
uint32_t motion_drive_elapsed_ms(void) { return mock_elapsed; }
bool motion_drive_finish(void)
{
    motors_on = false;
    if (mock_pending) sleep_ms(300);
    mock_pending = false;
    return !fail_drive && !stopped;
}
bool motion_drive(bool forwards, uint32_t ms, float heading)
{
    if (!motion_drive_start(forwards, ms, heading)) return false;
    while (motors_on) { if (!motion_drive_update()) break; if (motors_on) sleep_ms(5); }
    return motion_drive_finish();
}
// Colour sensor: arrival is scripted by check number (0 = never).
static unsigned colour_checks, colour_arrive_at_check, colour_inits, colour_resets;
static int close_threshold = AP1_CLOSE_RSSI_DBM;
static unsigned strict_attempts, loose_attempts;
static bool complete_on_loose;
// colour_present: a working sensor, so only colour arrival allows a transfer.
// Off by default: the v16 regressions use the BLE-only gate, as before.
static bool colour_present;
static bool mat_stub, model_lighter_seen;
static unsigned mat_from_check, colour_reads;
static int model_last_depth;
bool colour_sensor_init(void) { ++colour_inits; return true; }
bool colour_sensor_available(void) { return colour_present; }
void colour_arrival_reset(void) { ++colour_resets; model_lighter_seen = false; model_last_depth = -1; }
// Model mat: pink (white edge plus five rings) by default, or blue (white edge
// plus six rings), radius 0.35 m around the beacon, so blue rings are narrower.
// The sensor is 0.08 m ahead of the wheel axis; readings are calibrated means.
static bool model_blue;
#define MODEL_MAT_RADIUS 0.35
#define MODEL_SENSOR_AHEAD 0.08
#define MODEL_BOX_RADIUS 0.03     // the beacon box: the sensor must stay outside
static int model_bands(void) { return model_blue ? 7 : 6; }     // white edge + rings
static int model_arrival_depth(void) { return model_blue ? 6 : 5; } // blue ring 5, pink ring 4
static double model_sensor_d(void)
{
    double radians = current_heading * 3.141592653589793 / 180.0;
    return hypot(robot_x + MODEL_SENSOR_AHEAD * cos(radians) - beacon_x,
                 robot_y + MODEL_SENSOR_AHEAD * sin(radians) - beacon_y);
}
static int model_depth(void)
{
    double d = model_sensor_d();
    if (d > MODEL_MAT_RADIUS) return 0;
    // Another beacon's mat reads as floor once the robot's own mat is set.
    colour_mat_t model_mat = model_blue ? COLOUR_MAT_BLUE : COLOUR_MAT_PINK;
    if (colour_target_mat() != COLOUR_MAT_NONE && colour_target_mat() != model_mat) return 0;
    int bands = model_bands();
    int depth = bands - (int)(d / (MODEL_MAT_RADIUS / bands));
    depth = depth < 1 ? 1 : depth > bands ? bands : depth;
    if (model_dim_inner && depth >= model_arrival_depth()) depth = model_arrival_depth() - 1;
    return depth;
}
void colour_note_lighter_mats(unsigned mask)
{
    if (mask & (1u << (model_blue ? COLOUR_MAT_BLUE : COLOUR_MAT_PINK))) model_lighter_seen = true;
}
bool colour_sensor_read(colour_reading_t *reading)
{
    static const colour_reading_t pink[6] = {
        {7800, 1000, 1015, 970}, {6450, 1000, 927, 910},
        {5150, 1000, 785, 800}, {3770, 1000, 628, 800}, {2700, 1000, 475, 800},
        {1930, 1000, 344, 800},
    };
    // Blue: 7 November calibration means, white edge to ring 6.
    static const colour_reading_t blue[7] = {
        {14078, 4000, 4820, 4738}, {8493, 2290, 2898, 3026}, {7105, 1765, 2410, 2715},
        {6650, 1493, 2235, 2739}, {4685, 942, 1541, 2091}, {3355, 658, 1074, 1560},
        {2577, 518, 799, 1222},
    };
    static const colour_reading_t floor_reading = {2900, 1000, 922, 760};
    assert(!motors_on);
    ++colour_reads;
    int depth = model ? model_depth() : 0;
    *reading = depth == 0 ? floor_reading : model_blue ? blue[depth - 1] : pink[depth - 1];
    return colour_present;
}
// Scripted arrival by check number (0 = never), or the model's mat rule: the
// arrival ring or darker after a lighter surface of the mat at an earlier check.
bool colour_arrival_check(void)
{
    assert(!motors_on && !scan_active);
    ++colour_checks;
    if (!model) return colour_arrive_at_check && colour_checks >= colour_arrive_at_check;
    if (!colour_present) return false;
    int depth = model_depth();
    bool seen_before = model_lighter_seen;
    if (depth >= 1 && depth < model_arrival_depth()) model_lighter_seen = true;
    model_last_depth = depth;
    return depth >= model_arrival_depth() && seen_before;
}
int colour_last_depth(void) { return model ? model_last_depth : -1; }
bool colour_near_arrival(unsigned rings)
{
    return model && colour_present && model_last_depth >= 1 &&
           model_arrival_depth() - model_last_depth <= (int)rings;
}
// On the mat: scripted, or in the model the last check saw the mat.
bool colour_on_mat(void)
{
    if (model) return colour_present && model_last_depth >= 1;
    return mat_stub || (mat_from_check && colour_checks >= mat_from_check);
}
void payload_client_set_close_threshold(int dbm) { assert(!motors_on); close_threshold = dbm; }
int payload_client_close_threshold(void) { return close_threshold; }
void payload_client_init(void) { }bool payload_client_ready(void) { return !ui_bt_failure && !ui_bt_timeout; }
bool payload_client_start_failed(void) { return ui_bt_failure; }
const uint8_t *payload_client_data(uint16_t *length) { *length = 0; return NULL; }
static bool ble_never; // AP1 never passes any BLE gate
static payload_result_t scripted_attempt(void)
{
    assert(!motors_on && !scan_active);
    sleep_ms(ble_delay_ms);
    ++ble_calls;
    if (ble_never) return PAYLOAD_NOT_CLOSE;
    if (close_threshold == COLOUR_ARRIVAL_BLE_DBM) ++loose_attempts; else ++strict_attempts;
    if (complete_on_loose && close_threshold == COLOUR_ARRIVAL_BLE_DBM) return PAYLOAD_COMPLETE;
    if (fail_at_ble_call == ble_calls) return PAYLOAD_FATAL;
    if (complete_first || complete_at_ble_call == ble_calls ||
        (complete_after_step > 0 && forwards_count >= complete_after_step) ||
        (model && hypot(robot_x - beacon_x, robot_y - beacon_y) < 0.25)) return PAYLOAD_COMPLETE;
    if (retry_mode == 1) return PAYLOAD_RETRY;
    if (retry_mode == 2) return ble_calls == 1 ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE;
    if (retry_mode == 3) return ble_calls == 1 ? PAYLOAD_RETRY : PAYLOAD_COMPLETE;
    if (retry_mode == 4 && forwards_count>=16) return retry_calls ? PAYLOAD_COMPLETE : PAYLOAD_RETRY;
    return PAYLOAD_NOT_CLOSE;
}
// Like the real client: once close range is confirmed the callback fires.
payload_result_t payload_client_attempt(void)
{
    payload_result_t result = scripted_attempt();
    if ((result == PAYLOAD_COMPLETE || result == PAYLOAD_RETRY) && close_callback) close_callback();
    return result;
}
payload_result_t payload_client_retry(void)
{
    assert(!motors_on && !scan_active && ble_calls>0);
    ++retry_calls;
    return payload_client_attempt();
}
int cyw43_arch_init(void)
{
    if (ui_test) assert(ui_armed && ui_begins == 1 && ui_calibrations == 1);
    if (ui_radio_error) return ui_radio_error;
    ui_radio = true; return 0;
}
void cyw43_arch_enable_sta_mode(void) { }
static void deliver_scan(void)
{
    scan_active = false;
    if (wifi_missing || scan_rssi == -127) return;
    int rssi = scan_rssi;
    if (model) {
        double distance = fmax(0.10, hypot(robot_x - beacon_x, robot_y - beacon_y));
        double bias = 10 * sin(current_heading * 3.141592653589793 / 180.0);
        rssi = (int)lround(-50 - 20 * log10(distance) + bias);
        if (robot_x >= dead_from && robot_x < dead_to) rssi -= 12;
    }
    cyw43_ev_scan_result_t result = {.rssi = (int16_t)rssi, .ssid_len = 11, .bssid = {1,2,3,4,5,6}};
    memcpy(result.ssid, "I AM PICO W", 11);
    // A stronger network with the same SSID must not replace a locked target.
    scan_callback(scan_context, &result);
    memset(result.bssid, 9, 6); result.rssi = -10;
    // Inject this only after the first scan has locked AP1.
    if (wifi_calls > 1) scan_callback(scan_context, &result);
}
int cyw43_wifi_scan(int *state, cyw43_wifi_scan_options_t *options, void *context,
                   int (*callback)(void *, const cyw43_ev_scan_result_t *))
{
    (void)state; (void)options;
    assert(!scan_active);
    unsigned reading = (unsigned)wifi_calls++;
    unsigned group = raw_script_mode ? reading : reading / SCANS_PER_SAMPLE;
    if (fail_scan) return -1;
    if (stop_at_scan && (unsigned)wifi_calls == stop_at_scan) stopped = true;
    scan_rssi = group < script_length ? script[group] : default_rssi;
    scan_callback = callback; scan_context = context;
    unsigned delay = scan_delay_ms + (irregular_scans ? ((unsigned)wifi_calls % 3u) * 7u : 0u);
    scan_active = true; scan_finish_us = sim_time + (uint64_t)delay * 1000;
    if (!scan_delay_ms) deliver_scan();
    return 0;
}
bool cyw43_wifi_scan_active(int *state)
{
    (void)state;
    if (scan_active && sim_time >= scan_finish_us) deliver_scan();
    return scan_active;
}

static void reset(void)
{
    ui_test = ui_armed = ui_finished = ui_radio = ui_log_failure = false;
    ui_stage = ui_replays = ui_begins = ui_calibrations = 0;
    ui_init_calls = ui_fail_init_count = 0;
    ui_hold_start = ui_cal_failure = ui_stop_calibration = ui_stop_on_arm = false;
    ui_bt_failure = ui_bt_timeout = false;
    ui_radio_error = 0;
    sim_time = scan_finish_us = 0;
    stopped = motors_on = wifi_missing = fail_drive = fail_turn = stop_during_pause = false;
    complete_first = fail_scan = model = scan_active = storage_failed = false;
    irregular_scans = stop_on_reverse = raw_script_mode = false;
    powered_script = NULL; powered_script_length = 0;
    wifi_calls = forwards_count = backwards_count = ble_calls = sign_checks = 0;
    tone_starts = tone_stops = retry_mode = complete_after_step = fail_at_ble_call = retry_calls = 0;
    complete_at_ble_call = 0;
    script = NULL; script_length = 0; default_rssi = -70;
    current_heading = 0; robot_x = robot_y = beacon_x = beacon_y = dead_from = dead_to = 0;
    log_length = 0; log_text[0] = 0; move_count = 0;
    ble_delay_ms = stop_at_scan = stop_at_step = fail_at_step = checkpoint_calls = mock_duration = mock_elapsed = 0;
    scan_delay_ms = 200; // Ordinary control tests deliberately use stationary fallback.
    mock_pending = false;
    colour_checks = colour_arrive_at_check = colour_inits = colour_resets = 0;
    close_threshold = AP1_CLOSE_RSSI_DBM; strict_attempts = loose_attempts = 0; complete_on_loose = false;
    memset(cue_counts, 0, sizeof(cue_counts)); cue_end = 0; close_callback = NULL; in_range_cue_on = false;
    run_started_ms = now_ms(); translation_ms = profile_id = 0; wifi_quiescent = true;
    // Recorded-run and v16 regression checks keep their 250/100 ms steps;
    // shipped_steps() selects the robot's v18 sizes.
    floor_block_ms = 250; dip_probe_ms = 100; step_on_mat = mat_stub = false; mat_from_check = 0;
    colour_present = model_lighter_seen = colour_arrived = false; colour_reads = mat_scans_used = 0;
    model_last_depth = -1; ble_never = false;
    model_blue = false; model_min_sensor_d = 1e9; mat_near_block_ms = LINE_MAT_NEAR_BLOCK_MS;
    model_dim_inner = model_box_solid = false; model_blocked_blocks = 0;
    // Any mat, unless a test sets one (firmware_main sets AP1's pink mat).
    colour_set_target_mat(COLOUR_MAT_NONE);
    for (int f = 0; f < FAULT_COUNT; ++f) fault_clear((fault_t)f);
    wifi_sampler_init(run_can_continue);
}
// The step sizes the robot actually uses (reset() selects v16's).
static void shipped_steps(void) { floor_block_ms = LINE_FORWARD_BLOCK_MS; dip_probe_ms = LINE_DIP_PROBE_MS; }
static line_pair_t pair(int first, int second)
{
    return (line_pair_t){.first = {.code = rssi_encode(first)}, .second = {.code = rssi_encode(second)}};
}
static void check_decisions(void)
{
    assert(line_compare(pair(-70,-70), pair(-67,-67)) == 1);
    assert(line_compare(pair(-70,-70), pair(-73,-73)) == -1);
    assert(line_compare(pair(-70,-70), pair(-45,-70)) == 0);
    assert(line_compare(pair(-70,-70), pair(-75,-70)) == 0);
    assert(line_compare(pair(-70,-70), pair(-127,-60)) == 0);
    assert(line_return_consistent(pair(-60,-60),pair(-61,-60)));
    assert(!line_return_consistent(pair(-60,-60),pair(-70,-60)));
    assert(!line_return_consistent(pair(-60,-60),pair(-65,-65)));
    line_pair_t noisy = pair(-65,-65); noisy.first.spread = 5;
    assert(line_compare(pair(-70,-70), noisy) == 0);
    // Recorded start and point 9: a repeatable 6 dB gain must exceed the margin.
    line_pair_t baseline = pair(-69,-73), improved = pair(-63,-62);
    baseline.first.spread=1; baseline.second.spread=3;
    improved.first.spread=improved.second.spread=2;
    assert(line_pair_margin(baseline,improved)==4);
    assert(line_confirmed_gain(baseline,improved)==6 && line_compare(baseline,improved)==1);
    assert(line_compare(improved,baseline)==-1);
    // Batch disagreement still prevents a single strong batch establishing gain.
    assert(line_compare(baseline,pair(-50,-70))==0);
    assert(line_compare(baseline,pair(-66,-65))==0);
    assert(line_compare(baseline,pair(-65,-65))==1);
    // Return tolerance is independent of the less restrictive progress margin.
    assert(line_return_consistent(baseline,baseline));
    uint8_t raw[] = {70,70,10,70,70};
    assert(wifi_filter(raw,5).code == 70);
    raw[2] = 126; assert(wifi_filter(raw,5).code == 70);
    uint8_t missing[] = {127,127,127,70,70};
    assert(wifi_filter(missing,5).code == 127);
    assert(line_normalise_heading(-45) == 315 && line_normalise_heading(405) == 45);
}
static void check_return_tolerance(void)
{
    // Recorded v9 return: -62/-62 with spread 1, then -65/-65 with spreads 4/0.
    // The second returned batch is exactly at its 3 dB tolerance.
    line_pair_t baseline = pair(-62,-62), returned = pair(-65,-65);
    baseline.first.spread = baseline.second.spread = 1;
    returned.first.spread = 4;
    assert(line_return_consistent(baseline,returned));
    returned.first.code = returned.second.code = rssi_encode(-66);
    assert(!line_return_consistent(baseline,returned));
    // Swapping the batches must apply the same boundary to the first batch.
    returned.first.spread = 0; returned.second.spread = 4;
    assert(!line_return_consistent(baseline,returned));
    returned.first.code = returned.second.code = rssi_encode(-65);
    assert(line_return_consistent(baseline,returned));

    // Equality also applies when measured spreads raise the tolerance to 5 dB.
    baseline.first.spread = baseline.second.spread = 2;
    returned = pair(-67,-67);
    returned.first.spread = returned.second.spread = 2;
    assert(line_return_consistent(baseline,returned));
    returned.first.code = returned.second.code = rssi_encode(-68);
    assert(!line_return_consistent(baseline,returned));

    // The two returned batches may differ by the tolerance, but not exceed it.
    assert(line_return_consistent(pair(-70,-70),pair(-60,-63)));
    assert(!line_return_consistent(pair(-70,-70),pair(-60,-64)));
    assert(!line_return_consistent(pair(-127,-62),pair(-65,-65)));
    assert(!line_return_consistent(pair(-62,-62),pair(-65,-127)));
}
static void check_profiles(void)
{
    follow_result_t followed;
    static const int curve[] = {
        -70,-70, -65,-65, -60,-60, -60,-60,
        -66,-66, -67,-67, -68,-68, // Three weak positions after a sustained peak.
        -68,-68, -68,-68, -69,-69, // Stationary recheck and two short probes.
        -60,-60                    // Return to the actual best, not the median.
    };
    reset(); script=curve; script_length=sizeof(curve)/sizeof(*curve); irregular_scans=true;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE);
    assert(followed.progress && followed.consistent && followed.end==FOLLOW_DECLINE);
    assert(forwards_count==8 && backwards_count==2 && moves[8].ms==750 && moves[9].ms==450);
    assert(moves[6].ms==100 && moves[7].ms==100);
    assert(translation_ms==2900 && profile_count==7 && line_profile[2].powered_ms==500);
    assert(wifi_calls==110 && line_profile[1].wall_ms>line_profile[1].powered_ms);
    assert(strstr(log_text,"return only 1200 powered ms to best point 500"));
    static const int spike[] = {-70,-70,-45,-70,-66,-66,-66,-66,-72,-72,-73,-73,-74,-74,
                               -75,-75,-75,-75,-76,-76,-66,-66};
    reset(); script=spike; script_length=sizeof(spike)/sizeof(*spike);
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE && followed.progress && followed.consistent);
    assert(translation_ms==2900 && backwards_count==2 && moves[8].ms==750);
    // A stronger two-batch point can remain confirmed against the original
    // baseline even when its increase over the previous best is only 1 dB.
    static const int incremental[] = {-70,-70,-65,-65,-64,-64,-64,-64,-68,-68,-69,-69,-70,-70,
                                     -70,-70,-71,-71,-72,-72,-64,-64};
    reset(); script=incremental; script_length=sizeof(incremental)/sizeof(*incremental);
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE && followed.consistent);
    assert(moves[8].ms==750 && moves[9].ms==450 && strstr(log_text,"to best point 500"));
    // A flat side direction still has a bounded trial; its caller returns it.
    reset(); assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE);
    assert(!followed.progress && followed.end==FOLLOW_NO_GAIN && !backwards_count);
    assert(forwards_count==10 && translation_ms==2400 && moves[9].ms==150);
    assert(strstr(log_text,"trial ended at current position"));
    // After progress, a plateau continues beyond six powered seconds to BLE.
    static const int plateau[] = {-70,-70,-65,-65};
    reset(); script=plateau; script_length=sizeof(plateau)/sizeof(*plateau); default_rssi=-65;
    complete_after_step=30;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_COMPLETE);
    assert(followed.progress && !backwards_count);
    assert(translation_ms==7500 && strstr(log_text,"six-second profile interval complete"));
    // A weak decline below the margin should keep being sampled until it is
    // confirmed, rather than changing direction because a profile timer ended.
    int gradual[66];
    for (unsigned i=0;i<66;++i) gradual[i]=-66;
    gradual[0]=gradual[1]=-70; gradual[2]=gradual[3]=-65;
    gradual[52]=gradual[53]=-70; gradual[54]=gradual[55]=-71;
    gradual[56]=gradual[57]=-72; gradual[58]=gradual[59]=-72;
    gradual[60]=gradual[61]=-73; gradual[62]=gradual[63]=-74;
    gradual[64]=gradual[65]=-65;
    reset(); script=gradual; script_length=sizeof(gradual)/sizeof(*gradual);
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE);
    assert(followed.end==FOLLOW_DECLINE && followed.consistent && forwards_count==30);
    assert(translation_ms==14150 && strstr(log_text,"return only 6950 powered ms to best point 250"));
    static const int bad_return[] = {-70,-70,-65,-65,-65,-65,-72,-72,-73,-73,-74,-74,
                                    -75,-75,-76,-76,-77,-77,-80,-80};
    reset(); script=bad_return; script_length=sizeof(bad_return)/sizeof(*bad_return);
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE && !followed.consistent);
    reset(); translation_ms=LINE_TRANSLATION_MS-500;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_FATAL && !forwards_count && fault_test(FAULT_TRANSLATION_LIMIT));
    reset(); profile_count=LINE_PROFILE_CAPACITY;
    assert(!record_point(0,0,pair(-70,-70)) && fault_test(FAULT_PROFILE));
}

static void check_follow_first(void)
{
    // Follow rising RSSI past both the old 2400 ms probe limit and the
    // six-second profile interval. No other direction or reverse is needed.
    int rising[3+26*2];
    rising[0]=rising[1]=rising[2]=-115;
    for (unsigned i=0;i<26;++i) rising[3+i*2]=rising[4+i*2]=-112+(int)i*3;
    reset(); script=rising; script_length=sizeof(rising)/sizeof(*rising); complete_after_step=27;
    assert(approach_beacon() && forwards_count==27 && !backwards_count);
    assert(translation_ms==6750 && !strstr(log_text,"DIRECTION SEARCH"));
    assert(strstr(log_text,"first confirmed improvement") && strstr(log_text,"continuing same heading"));
    for (unsigned i=0;i<move_count;++i) assert(moves[i].forwards && moves[i].heading==0 && moves[i].ms==250);

    // Only the two perpendicular headings are available. Try the other side
    // after a consistent failed first trial, and follow its confirmed gain.
    static const int failed_side[] = {
        -70,-70,-74,-74,-75,-75,-76,-76,-76,-76,-77,-77,-78,-78,-70,-70,
        -70,-70,-65,-65,-65,-65,-71,-71,-72,-72,-73,-73,-73,-73,-74,-74,-75,-75,-65,-65
    };
    reset(); script=failed_side; script_length=sizeof(failed_side)/sizeof(*failed_side);
    follow_result_t followed; bool found=false;
    assert(check_directions(0,&followed,&found)==PAYLOAD_NOT_CLOSE && found);
    assert(followed.heading==90 && followed.progress && followed.consistent);
    assert(forwards_count==12 && backwards_count==4 && current_heading==90);
    for (unsigned i=0;i<move_count;++i) assert(moves[i].heading==270 || moves[i].heading==90);
    static const int side[] = {-70,-70,-65,-65,-65,-65,-71,-71,-72,-72,-73,-73,
                              -73,-73,-74,-74,-75,-75,-65,-65};
    reset(); script=side; script_length=sizeof(side)/sizeof(*side);
    assert(check_directions(0,&followed,&found)==PAYLOAD_NOT_CLOSE && found);
    assert(followed.heading==270 && current_heading==270 && forwards_count==7 && backwards_count==2);
    reset(); stop_at_scan=11;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_FATAL && stopped && forwards_count==1 && !backwards_count);
}

static void check_direction_policy(void)
{
    // An improving forward line ends on sustained decline. Both sides then
    // decline without improving, and the flat trial straight back finds no
    // gain either: stop after these three checks, with no recovery pass.
    static const int no_side[] = {
        -70, -70,-70,-65,-65,-65,-65,-71,-71,-72,-72,-73,-73,-73,-73,-74,-74,-75,-75,-65,-65,
        -65,-65,-71,-71,-72,-72,-73,-73,-73,-73,-74,-74,-75,-75,-65,-65,
        -65,-65,-71,-71,-72,-72,-73,-73,-73,-73,-74,-74,-75,-75,-65,-65
    };
    reset(); script=no_side; script_length=sizeof(no_side)/sizeof(*no_side);
    assert(!approach_beacon() && fault_test(FAULT_NO_DIRECTION));
    // v16's 17 forward/6 reverse blocks, plus a 2400 ms back trial and its return.
    assert(forwards_count==27 && backwards_count==10 && translation_ms==11250);
    assert(strstr(log_text,"no direction confirmed improvement (both perpendiculars and straight back)"));
    assert(!strstr(log_text,"FALLBACK:") && !strstr(log_text,"four fallback"));
    bool went_back=false;
    for (unsigned i=0;i<move_count;++i) {
        assert(moves[i].heading==0 || moves[i].heading==270 || moves[i].heading==90 || moves[i].heading==180);
        if (moves[i].heading==180 && moves[i].forwards) went_back=true;
    }
    assert(went_back);
    // A return that disagrees with the best point no longer ends the run: the
    // same three-direction search follows from where the robot stopped.
    int inconsistent[sizeof(no_side)/sizeof(*no_side)];
    memcpy(inconsistent,no_side,sizeof(no_side)); inconsistent[19]=inconsistent[20]=-90;
    reset(); script=inconsistent; script_length=sizeof(inconsistent)/sizeof(*inconsistent);
    assert(!approach_beacon() && forwards_count==27 && backwards_count==10);
    assert(strstr(log_text,"inconsistent with best point; searching from here") && strstr(log_text,"DIRECTION SEARCH"));
    // Even if the initial direction never improved, a consistent decline return
    // checks the other directions, starting with the first perpendicular.
    static const int rejected_forward[] = {-70,-70,-70,-74,-74,-75,-75,-76,-76,
                                          -76,-76,-77,-77,-78,-78,-70,-70};
    reset(); script=rejected_forward; script_length=sizeof(rejected_forward)/sizeof(*rejected_forward);
    complete_after_step=6;
    assert(approach_beacon() && forwards_count==6 && backwards_count==2 && current_heading==270);
    // Flat trials must return to the same starting best point before testing
    // the next direction. This differs from ending an improving line.
    reset(); follow_result_t followed; bool found;
    assert(check_directions(0,&followed,&found)==PAYLOAD_NOT_CLOSE && !found);
    assert(forwards_count==30 && backwards_count==12 && translation_ms==14400 && move_count==42);
    assert(moves[10].heading==270 && !moves[10].forwards);
    assert(moves[14].heading==90 && moves[14].forwards);
    assert(moves[28].heading==180 && moves[28].forwards && followed.consistent);
    assert(strstr(log_text,"TRIAL RETURN: RSSI consistent"));
    // An inconsistent return after the first side still tries the other two.
    int bad_flat[24];
    for (unsigned i=0;i<24;++i) bad_flat[i]=i<22 ? -70 : -90;
    reset(); script=bad_flat; script_length=24;
    assert(check_directions(0,&followed,&found)==PAYLOAD_NOT_CLOSE && !found);
    assert(strstr(log_text,"inconsistent with starting best point, continuing from here"));
    assert(forwards_count==30 && backwards_count==12 && current_heading==180);
}

static void check_recorded_run(void)
{
    // Actual v5 forward observations through its first declining position.
    // Later readings below are synthetic alternatives, not measured predictions.
    static const int prefix[] = {
        -73,-74,-76,-75,-72,-72,-72,-70,-71,-72,
        -73,-73,-73,-73,-73,-73,-127,-70,-71,-70,
        -71,-70,-73,-68,-67,-82,-78,-127,-127,-78,
        -80,-77,-79,-80,-127,-69,-65,-66,-66,-71,
        -66,-68,-66,-68,-68,-66,-68,-69,-69,-71,
        -71,-65,-67,-70,-61,-68,-66,-66,-67,-67,
        -69,-66,-68,-67,-66,-60,-58,-59,-60,-63,
        -58,-60,-60,-59,-58,-62,-60,-61,-61,-61,
        -64,-62,-64,-62,-61,-69,-65,-65,-66,-65,
        -67,-66,-69,-65,-65,-62,-60,-62,-61,-60,
        -60,-59,-61,-61,-64,-64,-61,-62,-61,-61,
        -61,-62,-62,-61,-62,-62,-62,-63,-61,-62,
        -63,-62,-61,-63,-62,-58,-59,-57,-57,-57,
        -57,-57,-57,-57,-57,-63,-64,-62,-65,-63,
        -62,-62,-64,-65,-65,
    };
    static const uint32_t durations[] = {251,250,251,250,250,251,251,250,251,251,250,254,250};
    int raw[215];
    memcpy(raw,prefix,sizeof(prefix));
    for (unsigned i=145;i<155;++i) raw[i]=-56; // The signal recovers after one weak patch.
    reset(); script=raw; script_length=155; raw_script_mode=true;
    powered_script=durations; powered_script_length=sizeof(durations)/sizeof(*durations);
    complete_after_step=15;
    assert(approach_beacon() && forwards_count==15 && !backwards_count);
    assert(strstr(log_text,"new confirmed best at 3010 powered ms"));
    assert(!strstr(log_text,"DIRECTION SEARCH"));
    // Synthetic sustained decline, recheck and two probes still return only
    // the excess over the recorded best, including both short probe movements.
    for (unsigned i=145;i<175;++i) raw[i]=-68;
    for (unsigned i=175;i<185;++i) raw[i]=-69;
    for (unsigned i=185;i<205;++i) raw[i]=-70;
    for (unsigned i=205;i<215;++i) raw[i]=-57;
    reset(); script=raw; script_length=215; raw_script_mode=true;
    powered_script=durations; powered_script_length=sizeof(durations)/sizeof(*durations);
    complete_after_step=19;
    assert(approach_beacon() && forwards_count==19 && backwards_count==2);
    assert(!moves[18].forwards && moves[18].ms==750 && moves[18].heading==0);
    assert(!moves[19].forwards && moves[19].ms==450);
    assert(moves[20].forwards && moves[20].heading==270);
    assert(strstr(log_text,"return only 1200 powered ms to best point 3010"));
    assert(!strstr(log_text,"FALLBACK:") && strstr(log_text,"both perpendiculars"));
}

static void check_decline_positions(void)
{
    // One or two weak positions followed by recovery must not cause a reverse.
    static const int patch[] = {-70,-70,-65,-65,-72,-72,-65,-65,-73,-73,-74,-74,-65,-65};
    reset(); script=patch; script_length=sizeof(patch)/sizeof(*patch); complete_after_step=7;
    follow_result_t followed;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_COMPLETE && !backwards_count);
    assert(strstr(log_text,"decline position 2/3") && !strstr(log_text,"decline position 3/3"));
    // Brief IMU/moving-scan stops cannot count nearby observations as separate
    // 250 ms positions, even when every pair is below the best.
    int short_steps[26];
    for (unsigned i=0;i<26;++i) short_steps[i]=-75;
    short_steps[0]=short_steps[1]=-70; short_steps[2]=short_steps[3]=-65;
    short_steps[24]=short_steps[25]=-65;
    static const uint32_t hundred_ms[] = {100,100,100,100,100,100,100,100};
    reset(); script=short_steps; script_length=26;
    powered_script=hundred_ms; powered_script_length=8;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE && followed.consistent);
    assert(forwards_count==10 && backwards_count==2 && moves[10].ms==750 && moves[11].ms==150);
    assert(strstr(log_text,"1/3 at 200") && strstr(log_text,"2/3 at 500") && strstr(log_text,"3/3 at 800"));
    // A first decline at the trial limit must still get two more positions.
    int boundary[34];
    for (unsigned i=0;i<34;++i) boundary[i]=-70;
    boundary[20]=boundary[21]=-75; boundary[22]=boundary[23]=-76; boundary[24]=boundary[25]=-77;
    boundary[26]=boundary[27]=-77; boundary[28]=boundary[29]=-78; boundary[30]=boundary[31]=-79;
    reset(); script=boundary; script_length=34;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE);
    assert(followed.end==FOLLOW_DECLINE && !followed.progress && forwards_count==14 && translation_ms==6200);
    // No return time is spent if the next confirming position cannot be reserved.
    reset(); script=patch; script_length=sizeof(patch)/sizeof(*patch); translation_ms=LINE_TRANSLATION_MS-800;
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_FATAL && !backwards_count);
    assert(fault_test(FAULT_TRANSLATION_LIMIT));
}

static void check_dip_reference(void)
{
    line_pair_t samples[] = {pair(-70,-71),pair(-45,-45),pair(-66,-67)};
    samples[2].first.spread=2;
    line_pair_t median=line_median_reference(samples);
    assert(median.first.code==66 && median.second.code==67 && median.first.spread==2);
    samples[0]=pair(-127,-70);
    assert(line_reference(line_median_reference(samples)).code==RSSI_MISSING_CODE);
    assert(!line_reference_declined(pair(-60,-60),pair(-63,-63)));
    assert(line_reference_declined(pair(-60,-60),pair(-64,-64)));
    assert(!line_reference_declined(pair(-60,-60),pair(-70,-60)));
    assert(!line_reference_declined(pair(-127,-60),pair(-70,-70)));

    // An isolated strong POSITION (both batches) remains the best return point,
    // but must not become the decline reference and force a reverse.
    static const int peak[] = {-70,-70,-65,-65,-65,-65,-45,-45,
                              -65,-65,-65,-65,-65,-65,-65,-65};
    follow_result_t followed;
    reset(); script=peak; script_length=sizeof(peak)/sizeof(*peak); complete_after_step=8;
    assert(follow_direction(0,0,&followed)==PAYLOAD_COMPLETE && !backwards_count);
    assert(strstr(log_text,"new confirmed best at 750 powered ms"));
    assert(!strstr(log_text,"decline position") && !strstr(log_text,"forward probe"));

    // A gradual decline must not pull the three-position reference down with it.
    static const int drift[] = {-70,-70,-60,-60,-60,-60,-61,-61,-62,-62,-63,-63,
        -64,-64,-65,-65,-66,-66,-67,-67,-68,-68,-69,-69,-60,-60};
    reset(); script=drift; script_length=sizeof(drift)/sizeof(*drift);
    assert(follow_direction(0,0,&followed)==PAYLOAD_NOT_CLOSE && followed.consistent);
    assert(forwards_count==10 && backwards_count==3 && followed.end==FOLLOW_DECLINE);
    assert(strstr(log_text,"reference at -60/-60") && strstr(log_text,"1/3 at 1500"));
    assert(strstr(log_text,"return only 1950 powered ms to best point 250"));
}

static void check_dip_recovery(void)
{
    // Recover at the stationary recheck, first probe, or second probe. A new
    // normal 250 ms block must then be possible on the same heading.
    for (unsigned probes=0;probes<=LINE_DIP_MAX_PROBES;++probes) {
        int readings[] = {-70,-70,-60,-60,-60,-60,-65,-65,-66,-66,-67,-67,
                          -68,-68,-69,-69,-70,-70};
        unsigned recovery=12+2*probes;
        readings[recovery]=readings[recovery+1]=-63; // Exactly the tolerance.
        reset(); script=readings; script_length=recovery+2; complete_after_step=6+(int)probes;
        follow_result_t followed;
        assert(follow_direction(0,0,&followed)==PAYLOAD_COMPLETE);
        assert(!backwards_count && forwards_count==6+(int)probes && !motors_on);
        assert(strstr(log_text,"DIP: recovery confirmed; continuing the same heading"));
        assert(moves[5+probes].forwards && moves[5+probes].ms==250 && moves[5+probes].heading==0);
        for (unsigned i=0;i<probes;++i) assert(moves[5+i].ms==100);
        assert(wifi_calls==(int)(recovery+2)*5);
        assert(ble_calls==7+(int)probes); // Five normal blocks, recheck, probes, next block.
    }
    // Uncertain batch disagreement clears the consecutive count but must leave
    // the reference frozen. Repeated intermediate values cannot erase the peak.
    static const int uncertain[] = {-70,-70,-60,-60,-60,-60,-65,-65,-60,-70,
        -66,-66,-67,-67,-68,-68,-69,-69,-70,-70,-71,-71,-60,-60};
    reset(); script=uncertain; script_length=sizeof(uncertain)/sizeof(*uncertain);
    follow_result_t followed;
    assert(follow_direction(0,0,&followed)==PAYLOAD_NOT_CLOSE && followed.consistent);
    assert(forwards_count==9 && strstr(log_text,"inconclusive, reference still frozen"));
    assert(strstr(log_text,"3/3 at 1750") && strstr(log_text,"reference_dbm1=-60,reference_dbm2=-60"));
}

static void check_v14_dip(void)
{
    int raw[195];
    memcpy(raw,v14_wifi_raw,sizeof(v14_wifi_raw));
    // Actual evidence ends at 3273 ms, where v14 reversed. The new median
    // reference must let the robot reach one more forward checkpoint first.
    reset(); script=raw; script_length=145; raw_script_mode=true;
    powered_script=v14_forward_ms; powered_script_length=13; complete_after_step=14;
    scan_delay_ms=720; ble_delay_ms=5000;
    assert(approach_beacon() && forwards_count==14 && !backwards_count && wifi_calls==145);
    assert(strstr(log_text,"reference at -52/-52") && strstr(log_text,"2/3 at 3273"));
    assert(!strstr(log_text,"3/3 at 3273"));

    // Synthetic continuation: one more weak position, a weak stopped recheck,
    // then recovery at the second probe. BLE success is also synthetic.
    for (unsigned i=145;i<175;++i) raw[i]=-60;
    for (unsigned i=175;i<185;++i) raw[i]=-50;
    reset(); script=raw; script_length=185; raw_script_mode=true;
    powered_script=v14_forward_ms; powered_script_length=13; complete_after_step=17;
    scan_delay_ms=720; ble_delay_ms=5000;
    assert(approach_beacon() && forwards_count==17 && !backwards_count && wifi_calls==185);
    assert(moves[14].ms==100 && moves[15].ms==100 && moves[16].ms==250);
    assert(strstr(log_text,"stage=probe,probe=2,powered_ms=3723") && !strstr(log_text,"DIRECTION SEARCH"));

    // Alternative synthetic continuation: no recovery. Return only to the
    // actual recorded best at 2517 ms, including the 200 ms spent probing.
    for (unsigned i=175;i<185;++i) raw[i]=-61;
    for (unsigned i=185;i<195;++i) raw[i]=-50;
    reset(); script=raw; script_length=195; raw_script_mode=true;
    powered_script=v14_forward_ms; powered_script_length=13; complete_after_step=17;
    assert(approach_beacon() && forwards_count==17 && backwards_count==2);
    assert(!moves[16].forwards && moves[16].ms==750);
    assert(!moves[17].forwards && moves[17].ms==456);
    assert(moves[18].forwards && moves[18].heading==270);
    assert(strstr(log_text,"return only 1206 powered ms to best point 2517"));
}

static void check_dip_safety(void)
{
    uint32_t position;
    line_pair_t current;
    bool recovered;
    // BLE success or failure at the stopped recheck and after either probe
    // prevents all subsequent scans and movements.
    for (int call=1;call<=3;++call) {
        reset(); complete_at_ble_call=call; position=1500;
        assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_COMPLETE);
        assert(forwards_count==call-1 && !backwards_count && !motors_on);
        assert(position==1500u+100u*(unsigned)(call-1) && wifi_calls==(call-1)*10);
        reset(); fail_at_ble_call=call; position=1500;
        assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
        assert(forwards_count==call-1 && !backwards_count && !motors_on);
    }
    for (unsigned probe=1;probe<=2;++probe) {
        reset(); stop_at_step=probe; position=1500;
        assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
        assert(stopped && !motors_on && forwards_count==(int)probe && ble_calls==(int)probe);
        reset(); fail_at_step=probe; position=1500;
        assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
        assert(fault_test(FAULT_IMU) && !motors_on && forwards_count==(int)probe && ble_calls==(int)probe);
    }
    // Missing Wi-Fi or STOP at any new scan stage must not start a later probe.
    for (unsigned stage=0;stage<=2;++stage) {
        int missing[10];
        for (unsigned i=0;i<10;++i) missing[i]=i<stage*2 ? -70 : -127;
        reset(); script=missing; script_length=10; default_rssi=-127; position=1500;
        assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
        assert(forwards_count==(int)stage && !backwards_count && !motors_on);
        reset(); stop_at_scan=1+10*stage; position=1500;
        assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
        assert(stopped && forwards_count==(int)stage && !motors_on);
    }
    reset(); fail_scan=true; position=1500;
    assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
    assert(fault_test(FAULT_WIFI_SCAN) && !forwards_count);
    reset(); storage_failed=true; position=1500;
    assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
    assert(fault_test(FAULT_REPORT) && !forwards_count && !ble_calls);
    // Before each probe, reserve the entire possible return, including overrun.
    for (unsigned probes=0;probes<=1;++probes) {
        reset(); translation_ms=LINE_TRANSLATION_MS-(probes ? 1399u : 1299u); position=1500;
        assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
        assert(fault_test(FAULT_TRANSLATION_LIMIT) && forwards_count==(int)probes && !backwards_count);
    }
    reset(); sim_time=(uint64_t)(MAX_RUN_MS-1000u)*1000; position=1500;
    assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_FATAL);
    assert(fault_test(FAULT_RUN_LIMIT) && !forwards_count && !motors_on);
    // Shortened powered blocks do not permit additional probes beyond two.
    static const uint32_t short_probes[]={40,50};
    reset(); powered_script=short_probes; powered_script_length=2; position=1500;
    assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_NOT_CLOSE);
    assert(!recovered && forwards_count==2 && position==1590 && translation_ms==90);
}

static void check_forward_continuation(void)
{
    // A valid but flat initial signal must pass both former stopping points.
    reset(); complete_after_step=30;
    assert(approach_beacon() && forwards_count==30 && !backwards_count);
    assert(translation_ms==7500 && !fault_mask() && !motors_on);
    assert(ble_calls==31 && strstr(log_text,"six-second profile interval complete"));
    assert(!strstr(log_text,"first confirmed improvement") && !strstr(log_text,"DIRECTION SEARCH"));
    for (unsigned i=0;i<move_count;++i) assert(moves[i].forwards && moves[i].ms==250);

    // Replay all 115 raw Wi-Fi readings and ten powered durations from the
    // user's 3 November v7 run. BLE completion on step 11 is synthetic: it
    // proves we reach the next checkpoint, not that hardware will connect there.
    static const int raw[] = {
        -73,-74,-69,-71,-70,-66,-69,-69,-71,-70,
        -70,-74,-73,-71,-74,-69,-74,-71,-76,-74,
        -68,-127,-72,-127,-78,-67,-65,-67,-69,-67,
        -127,-65,-67,-71,-65,-72,-73,-72,-72,-70,
        -72,-68,-71,-71,-72,-67,-68,-68,-66,-67,
        -71,-61,-68,-67,-70,-69,-71,-73,-69,-73,
        -71,-71,-70,-70,-127,-69,-73,-68,-71,-66,
        -127,-71,-63,-63,-68,-61,-67,-69,-65,-66,
        -60,-61,-62,-64,-63,-70,-73,-72,-70,-66,
        -72,-127,-71,-73,-72,-62,-62,-64,-66,-63,
        -61,-63,-66,-61,-61,-64,-66,-68,-65,-65,
        -64,-65,-67,-67,-67,
    };
    static const uint32_t durations[] = {252,252,251,251,252,252,251,251,252,141};
    reset(); script=raw; script_length=sizeof(raw)/sizeof(*raw); raw_script_mode=true;
    powered_script=durations; powered_script_length=sizeof(durations)/sizeof(*durations);
    scan_delay_ms=720; ble_delay_ms=5000; complete_after_step=11;
    assert(approach_beacon() && forwards_count==11 && !backwards_count && !fault_mask());
    assert(wifi_calls==115 && ble_calls==12 && translation_ms==2655);
    assert(strstr(log_text,"powered_ms=2264,best_ms=0,gain_db=6,margin_db=4"));
    assert(strstr(log_text,"new confirmed best at 2264 powered ms"));
    assert(!strstr(log_text,"trial ended") && !strstr(log_text,"DIRECTION SEARCH"));

    // With no BLE success, continuation still reserves the entire possible
    // return. It must stop before combined outward/return travel exceeds 60 s.
    reset(); assert(!approach_beacon() && fault_test(FAULT_TRANSLATION_LIMIT));
    assert(forwards_count>30 && !backwards_count && !motors_on);
    assert(translation_ms+translation_cost(translation_ms)<=LINE_TRANSLATION_MS);
    assert(!fault_test(FAULT_NO_DIRECTION) && !fault_test(FAULT_PROFILE));

    // Realistic stationary scan/BLE delays reach the wall limit first.
    reset(); scan_delay_ms=720; ble_delay_ms=5000;
    assert(!approach_beacon() && fault_test(FAULT_RUN_LIMIT) && !motors_on);
    assert(translation_ms>6000 && translation_ms<LINE_TRANSLATION_MS && !backwards_count);
    assert(!fault_test(FAULT_PROFILE));

    // STOP and an IMU fault after prolonged inconclusive travel prevent any
    // later movement or BLE attempt, just as they do during the first step.
    reset(); stop_at_step=31;
    assert(!approach_beacon() && stopped && !motors_on && forwards_count==31);
    assert(translation_ms==7500 && ble_calls==31 && !backwards_count);
    reset(); fail_at_step=31;
    assert(!approach_beacon() && fault_test(FAULT_IMU) && !motors_on && forwards_count==31);
    assert(translation_ms==7500 && ble_calls==31 && !backwards_count);
}

static void check_radio_and_limits(void)
{
    wifi_sample_t sample;
    reset(); scan_delay_ms = 200;
    assert(wifi_sampler_sample(&sample) && sample.code == 70);
    assert(strstr(log_text,"fallback=1") && !wifi_sampler_moving_enabled());
    reset(); scan_delay_ms = 20;
    for (unsigned i=0;i<3;++i) assert(wifi_sampler_sample(&sample));
    assert(wifi_sampler_moving_enabled());
    // A moving decline only requests an early stop; decisions stay stationary.
    wifi_sampler_set_reference((wifi_sample_t){.code=60});
    uint32_t actual;
    assert(translate_block(true,750,0,0,translation_cost(750),true,&actual) == PAYLOAD_NOT_CLOSE);
    assert(actual < 750 && !motors_on && !scan_active && ble_calls == 1);
    assert(strstr(log_text,"stopped=0"));
    // A scan that outlasts travel is drained before BLE and permanently disables moving acquisition.
    reset(); scan_delay_ms = 10;
    for (unsigned i=0;i<3;++i) assert(wifi_sampler_sample(&sample));
    scan_delay_ms = 900;
    assert(translate_block(true,750,0,0,translation_cost(750),true,&actual) == PAYLOAD_NOT_CLOSE);
    assert(!scan_active && !wifi_sampler_moving_enabled() && ble_calls == 1);
    reset(); scan_delay_ms = SCAN_TIMEOUT_MS + 1;
    assert(!wifi_sampler_sample(&sample) && fault_test(FAULT_WIFI_SCAN));
    assert(!forwards_count && !ble_calls);
    sim_time += (uint64_t)SCAN_TIMEOUT_MS * 1000;
    assert(!save_checkpoint() && !checkpoint_calls); // radio failure stays latched
    reset(); scan_delay_ms = 0;
    for (unsigned i=0;i<3;++i) assert(wifi_sampler_sample(&sample));
    assert(translate_block(true,750,0,0,translation_cost(750),true,&actual) == PAYLOAD_FATAL);
    assert(fault_test(FAULT_PROFILE) && !motors_on && !ble_calls);
    reset(); wifi_missing = true; assert(!approach_beacon() && wifi_calls == 15 && !forwards_count);
    reset(); fail_scan = true; assert(!approach_beacon() && fault_test(FAULT_WIFI_SCAN));
    reset(); retry_mode = 1; assert(!approach_beacon() && ble_calls == 3 && !forwards_count);
    assert(retry_calls==2 && fault_test(FAULT_PAYLOAD));
    reset(); retry_mode = 2; assert(!approach_beacon() && ble_calls == 3 && !forwards_count);
    assert(retry_calls==2 && fault_test(FAULT_PAYLOAD));
    reset(); retry_mode = 3; assert(approach_beacon() && ble_calls==2 && retry_calls==1 && !forwards_count);
    // Arrival after step 16, then one failed transfer: stationary retry must
    // complete without issuing steps 17 or 18. RSSI here is synthetic and flat.
    reset(); retry_mode = 4;
    assert(approach_beacon() && forwards_count==16 && retry_calls==1 && ble_calls==18);
    assert(!backwards_count && !motors_on);
    reset(); stopped = true; assert(!approach_beacon() && !forwards_count && !ble_calls);
    reset(); storage_failed = true; assert(!approach_beacon() && fault_test(FAULT_REPORT));
    reset(); stop_at_scan = 6; assert(!approach_beacon() && !forwards_count);
    reset(); complete_after_step=1; assert(approach_beacon() && forwards_count==1 && !backwards_count);
    reset(); fail_at_ble_call=2; assert(!approach_beacon() && forwards_count==1 && !backwards_count);
    reset(); stop_on_reverse=true;
    assert(return_along_line(750,0,0)==PAYLOAD_FATAL && !motors_on && !ble_calls);
    reset(); run_started_ms = 0; sim_time = (uint64_t)MAX_RUN_MS * 1000;
    assert(!run_can_continue() && fault_test(FAULT_RUN_LIMIT));
    // Unsigned wall-clock wrap does not consume the movement budget or cause a timeout.
    reset(); sim_time = (uint64_t)(UINT32_MAX-30u)*1000; run_started_ms=now_ms(); scan_delay_ms=20;
    assert(wifi_sampler_sample(&sample) && !fault_test(FAULT_RUN_LIMIT));
}
static void run_startup_test(void)
{
    ui_test = true; ui_stage = 1;
    int outcome = setjmp(ui_exit);
    if (!outcome) firmware_main();
    assert(outcome == 1 && ui_finished && !motors_on && !ui_radio);
    ui_test = false;
}
static void check_startup(void)
{
    reset(); complete_after_step=1; run_startup_test();
    assert(sign_checks==1 && forwards_count==1 && ui_calibrations==1 && ui_init_calls==1);
    reset(); ui_fail_init_count=1; complete_after_step=1; run_startup_test();
    assert(ui_init_calls==2 && forwards_count==1 && !fault_test(FAULT_IMU));
    assert(strstr(log_text,"Retrying IMU initialisation"));
    reset(); ui_fail_init_count=2; run_startup_test();
    assert(ui_init_calls==2 && !ui_calibrations && !forwards_count && !ble_calls && fault_test(FAULT_IMU));
    reset(); ui_hold_start=true; run_startup_test();
    assert(fault_test(FAULT_START_BUTTON) && !ui_calibrations && !forwards_count && !ble_calls);
    reset(); ui_cal_failure=true; run_startup_test();
    assert(fault_test(FAULT_IMU) && !forwards_count && !ble_calls);
    reset(); ui_radio_error=-5; run_startup_test();
    assert(fault_test(FAULT_BLE_RADIO) && !forwards_count && !ble_calls && strstr(log_text,"error -5"));
    reset(); ui_bt_failure=true; run_startup_test();
    assert(fault_test(FAULT_BLE_RADIO) && !forwards_count && !ble_calls);
    reset(); ui_bt_timeout=true; run_startup_test();
    assert(fault_test(FAULT_BLE_RADIO) && !forwards_count && !ble_calls && strstr(log_text,"timed out after 10 seconds"));
    reset(); ui_stop_calibration=true; run_startup_test();
    assert(fault_test(FAULT_STOP_LATCHED) && !forwards_count && !ble_calls);
    reset(); ui_stop_on_arm=true; run_startup_test();
    assert(!ui_begins && !ui_calibrations && !forwards_count && !ble_calls);
    reset(); wifi_missing=true; run_startup_test();
    assert(!forwards_count && !sign_checks && fault_test(FAULT_WIFI_LOST));
    reset(); stop_at_scan=1; run_startup_test();
    assert(!forwards_count && !sign_checks);
    reset(); retry_mode=1; run_startup_test();
    assert(fault_test(FAULT_PAYLOAD) && !tone_starts && !forwards_count);
    reset(); retry_mode=3; run_startup_test();
    assert(tone_starts==1 && !forwards_count && strstr(log_text,"RESULT: PAYLOAD CONFIRMED"));
}
static void check_cues(void)
{
    // In range: the beeps start when close range is confirmed and stop when
    // each attempt ends, including a stationary retry.
    reset(); close_callback = on_in_range; retry_mode = 3;
    assert(approach_beacon() && cue_counts[CUE_IN_RANGE] == 2 && tone_stops == 2 && !in_range_cue_on);
    // Earlier attempts are not close, so only the final, completed one beeps.
    reset(); close_callback = on_in_range; complete_after_step = 3;
    assert(approach_beacon() && cue_counts[CUE_IN_RANGE] == 1 && !in_range_cue_on);
    // One long beep per search direction: a single straight line here.
    assert(cue_counts[CUE_PASS] == 1 && !cue_counts[CUE_COLLECTED] && !cue_counts[CUE_FAULT]);
}

static void check_colour_arrival(void){
    // On the mat after two steps: the loose BLE gate replaces -58 at once.
    reset(); colour_arrive_at_check = 3; complete_on_loose = true;
    assert(approach_beacon() && colour_resets == 1 && forwards_count == 2);
    assert(strict_attempts == 2 && loose_attempts == 1 && colour_checks == 3);
    assert(strstr(log_text, "ARRIVAL: colour sensor confirms the arrival ring (pink ring 4, blue or purple ring 5) or darker"));
    // Never on the mat: every attempt keeps the strict gate, one colour check each.
    reset(); complete_after_step = 3;
    assert(approach_beacon() && !loose_attempts && strict_attempts == colour_checks);
    assert(!strstr(log_text, "ARRIVAL: colour"));
    // Stationary retries keep the loose gate and do not repeat the colour check.
    reset(); colour_arrive_at_check = 1; retry_mode = 3;
    assert(approach_beacon() && colour_checks == 1 && loose_attempts == 2 && !strict_attempts);
    assert(retry_calls == 1 && !forwards_count);
    // On the mat but AP1 too weak even for the loose gate: navigation continues
    // and each later check uses whatever the colour sensor reports then.
    reset(); colour_arrive_at_check = 2; complete_after_step = 3;
    assert(approach_beacon() && loose_attempts >= 1 && strict_attempts == 1);
}

static void print_model_failure(unsigned target)
{
    fprintf(stderr,"Model target %u failed at (%.2f, %.2f), translation %u ms, faults %x\n",
            target,robot_x,robot_y,translation_ms,fault_mask());
    for (char *row=strtok(log_text,"\n");row;row=strtok(NULL,"\n"))
        if (!strncmp(row,"FOLLOW:",7) || !strncmp(row,"RETURN:",7) || !strncmp(row,"TRIAL",5) ||
            !strncmp(row,"DIRECTION",9) || !strncmp(row,"DIP:",4) || !strncmp(row,"FAILED",6))
            fprintf(stderr,"%s\n",row);
}

static void check_v18_steps(void)
{
    // Floor: 500 ms steps. The second stop's colour check sees the mat, so the
    // third step and later ones are 250 ms; the change is logged once. Colour
    // steering is spent here, so the Wi-Fi line keeps driving on the mat.
    follow_result_t followed;
    reset(); shipped_steps(); mat_scans_used=MAT_MAX_SCANS; mat_from_check=2; complete_after_step=5;
    assert(follow_direction(0,0,&followed)==PAYLOAD_COMPLETE && forwards_count==5 && !backwards_count);
    assert(moves[0].ms==500 && moves[1].ms==500 && moves[2].ms==250 && moves[4].ms==250);
    assert(translation_ms==1750 && strstr(log_text,"STEP: colour sensor sees the mat; forward blocks now 250 ms"));
    assert(strstr(log_text,"500 ms on the floor, 250 ms on the mat"));
    // Leaving the mat again restores the floor step.
    mat_from_check=0; assert(forward_block_ms()==500 && strstr(log_text,"no longer sees the mat"));
    // A flat side trial still covers 2400 ms, now in five stops instead of ten.
    reset(); shipped_steps();
    assert(follow_direction(0,LINE_DIRECTION_TEST_MS,&followed)==PAYLOAD_NOT_CLOSE);
    assert(followed.end==FOLLOW_NO_GAIN && forwards_count==5 && moves[4].ms==400 && translation_ms==2400);
    // Probes are 250 ms each, so the dip check looks 0.5 s past the third weak position.
    uint32_t position=1500; line_pair_t current; bool recovered;
    reset(); shipped_steps();
    assert(recheck_and_probe_dip(0,500,pair(-60,-60),&position,&current,&recovered)==PAYLOAD_NOT_CLOSE);
    assert(!recovered && forwards_count==2 && position==2000 && moves[0].ms==250);

    // A 12 dB weak patch about 1.1 s of travel long, 0.8 m along a line
    // straight at AP1, like the dips in the 7 November three-chair logs
    // (sideways run: still weak 0.7 s after it began). With v16 steps, three
    // weak positions and both 100 ms probes all fall inside it, so the robot
    // reverses off a good line. With v18 steps the third position is already
    // past the patch and the robot drives on to AP1.
    for (unsigned shipped=0;shipped<2;++shipped) {
        reset(); if (shipped) shipped_steps();
        model=true; beacon_x=2; dead_from=0.80; dead_to=1.10;
        bool arrived=approach_beacon();
        if (shipped) {
            if (!arrived) print_model_failure(0);
            assert(arrived && !backwards_count && !strstr(log_text,"DIRECTION SEARCH"));
        } else {
            assert(backwards_count && strstr(log_text,"DIRECTION SEARCH"));
            assert(strstr(log_text,"stage=probe,probe=2") && !strstr(log_text,"recovered=1"));
        }
    }
}

static int sensor_depth_now(void) { return model_depth(); }

static void check_mat_steering(void)
{
    // Colour gate: with a working sensor, BLE is not even tried before
    // arrival. In the 7 November side-on run v16 transferred on ring 1
    // because BLE alone read -58 dBm; here BLE would succeed from step 1.
    reset(); colour_present=true; complete_after_step=1; colour_arrive_at_check=5; complete_on_loose=true;
    assert(approach_beacon() && forwards_count==4 && ble_calls==1 && !strict_attempts && loose_attempts==1);
    assert(strstr(log_text,"BLE transfer waits for the colour mat"));
    // A missing sensor keeps the strict BLE gate.
    reset(); complete_after_step=1;
    assert(approach_beacon() && forwards_count==1 && strict_attempts==2);

    // Started on ring 1 driving along the ring, like the side-on run: scan,
    // turn toward the centre and arrive at ring 4 with no Wi-Fi line at all.
    // Then started on ring 2 facing straight out, and on the white edge.
    static const struct { double x, y, heading; const char *name; } starts[] = {
        {0.0, -0.26, 0, "ring 1, along the ring"},
        {0.20, 0.0, 0, "ring 2, facing out"},
        {-0.30, 0.10, 90, "white edge, side-on"},
    };
    for (unsigned i=0;i<sizeof(starts)/sizeof(*starts);++i) {
        reset(); shipped_steps(); model=true; colour_present=true;
        robot_x=starts[i].x; robot_y=starts[i].y; current_heading=starts[i].heading;
        bool arrived=approach_beacon();
        if (!arrived || sensor_depth_now()<5 || strict_attempts || strstr(log_text,"LINE_POINT,line=1,point=1")) {
            fprintf(stderr,"Mat start '%s': arrived %d, sensor depth %d, robot (%.2f, %.2f)\n",
                    starts[i].name,arrived,sensor_depth_now(),robot_x,robot_y);
            for (char *row=strtok(log_text,"\n");row;row=strtok(NULL,"\n"))
                if (!strncmp(row,"MAT",3) || !strncmp(row,"COLOUR_SCAN_RESULT",18) || !strncmp(row,"FAILED",6))
                    fprintf(stderr,"%s\n",row);
            assert(false);
        }
        assert(!strict_attempts && loose_attempts==1 && strstr(log_text,"MAT: scan 1 points"));
        // One scan was enough from each start (ring 1 along the ring: 90
        // degrees left; ring 2 facing out: 180; white edge: within 4 degrees
        // of the true centre direction).
        // One scan was enough from each start (ring 1 along the ring: 90
        // degrees left; ring 2 facing out: 180; white edge: within 4 degrees
        // of the true centre direction). Outer steps are 250 ms; within two
        // rings of arrival they are 100 ms.
        assert(mat_scans_used==1 && move_count<=4 && !backwards_count);
        for (unsigned m=0;m<move_count;++m)
            assert(moves[m].forwards && (moves[m].ms==250 || moves[m].ms==100));
        assert(moves[move_count-1].ms==100);
    }

    // On ring 4 but AP1 never passes even the loose gate: stop there with a
    // payload fault rather than driving on into the beacon box.
    reset(); shipped_steps(); model=true; colour_present=true; ble_never=true; robot_y=-0.26;
    assert(!approach_beacon() && fault_test(FAULT_PAYLOAD) && sensor_depth_now()>=5);
    assert(strstr(log_text,"AP1 BLE never came in range") && strstr(log_text,"stationary retry 3/3"));

    // The colour reads floor all round (no ring pattern): give up colour
    // steering after its scan budget and never transfer without arrival.
    reset(); shipped_steps(); colour_present=true; mat_stub=true; sim_time=0;
    assert(!approach_beacon() && mat_scans_used==MAT_MAX_SCANS && !ble_calls);
    assert(strstr(log_text,"found no ring to steer toward") && strstr(log_text,"resuming the Wi-Fi line search"));
}

// On 7 November the robot drove into the beacon box on the blue mat and
// nearly on the purple one. The 6-ring mats fit one more ring in the same
// radius, and in this model a 250 ms step (6.7 cm) is wider than a blue ring
// (5 cm). From five starts per mat, 100 ms steps within two rings of arrival
// must stop on the arrival ring itself (pink ring 4, blue ring 5) without the
// sensor ever reaching the ring inside it. With 250 ms steps the blue runs
// reach ring 6, next to the box.
static void check_near_centre_steps(void)
{
    static const struct { double x, y, heading; const char *name; } starts[] = {
        {0.0, -0.26, 0, "ring 1, along the ring"},
        {0.20, 0.0, 0, "ring 2, facing out"},
        {-0.30, 0.10, 90, "white edge, side-on"},
        {-0.33, 0.0, 0, "white edge, facing the centre"},
        {0.0, -0.30, 90, "white edge, facing the centre from below"},
    };
    for (unsigned blue=0;blue<2;++blue) {
        unsigned old_overshoots=0;
        for (unsigned old=0;old<2;++old)
        for (unsigned i=0;i<sizeof(starts)/sizeof(*starts);++i) {
            reset(); shipped_steps(); model=true; colour_present=true; model_blue=blue;
            if (old) mat_near_block_ms=LINE_MAT_BLOCK_MS;
            robot_x=starts[i].x; robot_y=starts[i].y; current_heading=starts[i].heading;
            model_min_sensor_d=model_sensor_d();
            bool arrived=approach_beacon();
            // Inner edge of the arrival ring: the ring inside it starts here.
            double inner_edge=MODEL_MAT_RADIUS/model_bands();
            bool overshot=model_min_sensor_d<inner_edge;
            if (old) { old_overshoots+=overshot; continue; }
            if (!arrived || overshot || model_depth()!=model_arrival_depth() || mat_scans_used!=1) {
                fprintf(stderr,"%s mat '%s': arrived %d, closest %.3f m (arrival ring inner edge %.3f), depth %d, scans %u\n",
                        blue?"Blue":"Pink",starts[i].name,arrived,model_min_sensor_d,inner_edge,model_depth(),mat_scans_used);
                assert(false);
            }
            assert(model_min_sensor_d>MODEL_BOX_RADIUS && !strict_attempts);
            // Normal readings arrive by colour; the centre stop rules stay out of it.
            assert(!strstr(log_text,"at the mat centre"));
        }
        // The old 250 ms steps reach the innermost ring on the blue mat.
        if (blue) assert(old_overshoots>=3);
    }

    // 7 November blue and purple runs: the inner rings read one ring lighter
    // than calibrated, so arrival never triggered and the robot pushed into
    // the beacon box. With a solid box and no arrival reading, the robot must
    // stop at the box on its own within a few blocked steps, transfer, and
    // never resume Wi-Fi driving there.
    for (unsigned i=0;i<sizeof(starts)/sizeof(*starts);++i) {
        reset(); shipped_steps(); model=true; colour_present=true; model_blue=true;
        model_dim_inner=model_box_solid=true;
        robot_x=starts[i].x; robot_y=starts[i].y; current_heading=starts[i].heading;
        bool arrived=approach_beacon();
        if (!arrived || model_blocked_blocks>MAT_STALL_STEPS || !strstr(log_text,"at the mat centre")) {
            fprintf(stderr,"Dim blue '%s': arrived %d, blocked steps %u, scans %u, robot (%.3f, %.3f)\n",
                    starts[i].name,arrived,model_blocked_blocks,mat_scans_used,robot_x,robot_y);
            for (char *row=strtok(log_text,"\n");row;row=strtok(NULL,"\n"))
                if (!strncmp(row,"MAT",3) || !strncmp(row,"COLOUR_SCAN_RESULT",18) || !strncmp(row,"APPROACH",8))
                    fprintf(stderr,"%s\n",row);
            assert(false);
        }
        assert(!strict_attempts && !strstr(log_text,"resuming the Wi-Fi line search"));
        assert(strstr(log_text,"read no darker; the robot is against the beacon box"));
    }
    // Already surrounded by inner rings (every scan direction within two rings
    // of arrival, like the all-ring-3 scan in the blue run): stop without moving.
    reset(); shipped_steps(); model=true; colour_present=true; model_blue=true; model_dim_inner=true;
    robot_x=0.02;
    assert(approach_beacon() && !move_count && strstr(log_text,"every scan direction is within two rings"));

    // Each beacon has its own mat. The AP1 robot (pink) started on the blue
    // mat never steers by colour or transfers there; the AP2 robot (blue)
    // does, from the same start.
    reset(); shipped_steps(); model=true; colour_present=true; model_blue=true;
    colour_set_target_mat(COLOUR_MAT_PINK); robot_y=-0.26;
    assert(!approach_beacon() && !ble_calls && !strstr(log_text,"MAT: colour sensor sees the mat"));
    reset(); shipped_steps(); model=true; colour_present=true; model_blue=true;
    colour_set_target_mat(COLOUR_MAT_BLUE); robot_y=-0.26;
    assert(approach_beacon() && loose_attempts==1 && strstr(log_text,"MAT: colour sensor sees the mat"));
    // The same stop rules without a solid box: arrival still never reads, so
    // the robot must stop once nothing around it is darker, not drive on.
    reset(); shipped_steps(); model=true; colour_present=true; model_blue=true; model_dim_inner=true;
    robot_y=-0.26;
    assert(approach_beacon() && strstr(log_text,"at the mat centre") && !strict_attempts);
    // BLE never in range at the centre: retries in place, then stops for good.
    reset(); shipped_steps(); model=true; colour_present=true; model_blue=true;
    model_dim_inner=model_box_solid=true; ble_never=true; robot_y=-0.26;
    assert(!approach_beacon() && fault_test(FAULT_PAYLOAD) && strstr(log_text,"stationary retry 3/3"));
    unsigned moves_at_stop=move_count;
    assert(strstr(log_text,"at the mat centre") && moves_at_stop>0);
}

int main(void)
{
    check_decisions(); check_return_tolerance(); check_profiles(); check_follow_first(); check_direction_policy(); check_recorded_run(); check_decline_positions();
    check_dip_reference(); check_dip_recovery(); check_v14_dip(); check_dip_safety();
    check_forward_continuation(); check_radio_and_limits(); check_startup(); check_colour_arrival(); check_cues();
    check_v18_steps(); check_mat_steering(); check_near_centre_steps();
    reset(); complete_first=true; assert(approach_beacon() && !wifi_calls && !forwards_count && !sign_checks);
    tone_stops=0;
    uint64_t before=sim_time; completion_sound();
    assert(tone_starts==1 && tone_stops==0 && sim_time-before==910000);
    reset(); stopped=true; completion_sound(); assert(!tone_starts);
    reset(); stop_during_pause=true; completion_sound(); assert(tone_starts==1 && tone_stops==1);
    reset(); fail_drive=true; assert(!approach_beacon() && forwards_count==1 && !backwards_count);
    reset(); fail_turn=true; assert(!approach_beacon() && !forwards_count);
    // Ideal model, beacon 2 m ahead, left, behind and right. Continuing the
    // first line lets a side target establish a decline and then be reached
    // by a perpendicular; a rear target (robot facing away) is now reached
    // by the trial straight back, which v16 never tried. Both step sizes.
    // Third pass: v18 steps with the colour sensor, so the last part is colour
    // steering and only ring 4 allows the transfer.
    const double targets[4][2]={{2,0},{0,2},{-2,0},{0,-2}};
    for (unsigned shipped=0;shipped<3;++shipped)
    for (unsigned i=0;i<4;++i) {
        reset(); if (shipped) shipped_steps();
        model=true; beacon_x=targets[i][0]; beacon_y=targets[i][1]; colour_present=shipped==2;
        bool arrived=approach_beacon();
        if (!arrived) print_model_failure(i);
        assert(arrived && hypot(robot_x-beacon_x,robot_y-beacon_y)<0.25);
        assert(translation_ms<=LINE_TRANSLATION_MS);
        if (i==2) assert(strstr(log_text,"straight back"));
        if (colour_present) assert(strstr(log_text,"colour steering takes over") && !strict_attempts && model_depth()>=5);
    }
    for (int fail=0;fail<2;++fail) {
        reset(); ui_test=true; ui_stage=1; complete_first=true; ui_log_failure=fail;
        if (!setjmp(ui_exit)) firmware_main();
        assert(ui_replays==1 && ui_begins==1 && ui_finished && !motors_on);
        if (fail) assert(!ble_calls && !forwards_count && fault_test(FAULT_REPORT));
        else assert(strstr(log_text,"RESULT: PAYLOAD CONFIRMED") && colour_inits == 1);
        // The AP1 build keeps to the pink mat.
        assert(colour_target_mat() == COLOUR_MAT_PINK && strstr(log_text,"Colour mat: PINK (AP1)"));
        // Start beep, calibration tone and completion cue on success; the
        // falling fault cue when the log cannot be prepared.
        if (fail) assert(cue_counts[CUE_FAULT] == 1 && !cue_counts[CUE_CALIBRATING] && !tone_starts);
        else assert(cue_counts[CUE_START] == 1 && cue_counts[CUE_CALIBRATING] == 1 && tone_starts == 1 &&
                    cue_counts[CUE_IN_RANGE] == 1 && !cue_counts[CUE_FAULT]);
    }
    puts("Navigation: three-position median, frozen decline reference, stationary recheck, bounded probes, recorded v14 dip and synthetic continuations, inclusive return tolerance, recorded v5/v7/v9 regressions, three-direction search with straight back, v18 500/250 ms steps, weak patch, colour gate on the transfer, mat steering from side-on, outward and edge starts, 100 ms steps stopping on the arrival ring of the pink and blue model mats, all four model targets at both step sizes and with colour steering, travel/run limits, radio fallback, BSSID, STOP, startup, arrival, colour-arrival BLE gate and sound cue checks passed");
    return 0;
}
