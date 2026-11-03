#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#define main firmware_main
#include "../getting_there.c"
#undef main

static uint64_t sim_time, scan_finish_us;
static bool stopped, motors_on, wifi_missing, fail_drive, fail_turn, stop_during_pause;
static bool complete_first, fail_scan, scan_active, storage_failed;
static bool irregular_scans, stop_on_reverse, raw_script_mode;
static const uint32_t *powered_script;
static unsigned powered_script_length;
static int wifi_calls, forwards_count, backwards_count, ble_calls, sign_checks;
static int tone_starts, tone_stops, retry_mode, complete_after_step, fail_at_ble_call, retry_calls;
static int complete_at_ble_call;
static unsigned near_after_step, stop_at_move, fail_at_move;
static bool nearby;
static unsigned scan_delay_ms, ble_delay_ms, stop_at_scan, stop_at_step, fail_at_step, checkpoint_calls;
static float current_heading;
static const int *script;
static size_t script_length;
static int default_rssi;
static bool model;
static double robot_x, robot_y, beacon_x, beacon_y;
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
uint32_t clock_get_hz(int clock) { (void)clock; return 125000000; }
pwm_config pwm_get_default_config(void) { return (pwm_config){0}; }
void pwm_config_set_clkdiv(pwm_config *config, float divider) { (void)config; (void)divider; }
void pwm_config_set_wrap(pwm_config *config, uint16_t wrap) { config->wrap = wrap; }
void pwm_init(uint slice, const pwm_config *config, bool enabled)
{ (void)slice; (void)config; (void)enabled; }
uint pwm_gpio_to_slice_num(uint pin) { return pin / 2; }
void pwm_set_gpio_level(uint pin, uint16_t level)
{
    assert(pin == 22 && !motors_on);
    if (level) ++tone_starts; else ++tone_stops;
}
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
    if (stop_at_move && move_count >= stop_at_move) stopped = true;
    if (fail_at_move && move_count >= fail_at_move) { fail_drive = true; fault_raise(FAULT_IMU); }
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
        robot_x += distance * cos(radians); robot_y += distance * sin(radians);
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
void payload_client_init(void) { }
bool payload_client_ready(void) { return !ui_bt_failure && !ui_bt_timeout; }
bool payload_client_start_failed(void) { return ui_bt_failure; }
const uint8_t *payload_client_data(uint16_t *length) { *length = 0; return NULL; }
payload_result_t payload_client_attempt(void)
{
    assert(!motors_on && !scan_active);
    sleep_ms(ble_delay_ms);
    ++ble_calls;
    if (near_after_step && (unsigned)forwards_count >= near_after_step) nearby = true;
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
payload_result_t payload_client_retry(void)
{
    assert(!motors_on && !scan_active && ble_calls>0);
    ++retry_calls;
    return payload_client_attempt();
}
bool payload_client_nearby(void) { return nearby; }
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
    near_after_step = stop_at_move = fail_at_move = 0; nearby = false;
    script = NULL; script_length = 0; default_rssi = -70;
    current_heading = 0; robot_x = robot_y = beacon_x = beacon_y = 0;
    log_length = 0; log_text[0] = 0; move_count = 0;
    ble_delay_ms = stop_at_scan = stop_at_step = fail_at_step = checkpoint_calls = mock_duration = mock_elapsed = 0;
    scan_delay_ms = 200; // Ordinary control tests deliberately use stationary fallback.
    mock_pending = false;
    run_started_ms = now_ms(); translation_ms = profile_id = 0; wifi_quiescent = true;
    for (int f = 0; f < FAULT_COUNT; ++f) fault_clear((fault_t)f);
    wifi_sampler_init(run_can_continue);
}
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
#include "test_coarse_fine_cases.h"

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
    assert(tone_starts==3 && !forwards_count && strstr(log_text,"RESULT: PAYLOAD CONFIRMED"));
}
int main(void)
{
    check_decisions(); check_return_tolerance(); check_coarse_tracking(); check_coarse_follow();
    check_refine_return(); check_coarse_faults(); check_radio_and_limits(); check_startup();
    reset(); complete_first=true; assert(approach_beacon() && !wifi_calls && !forwards_count && !sign_checks);
    buzzer_init(); tone_stops=0;
    uint64_t before=sim_time; completion_sound();
    assert(tone_starts==3 && tone_stops==4 && sim_time-before==900000);
    reset(); stopped=true; completion_sound(); assert(!tone_starts);
    reset(); stop_during_pause=true; completion_sound(); assert(tone_starts==1 && tone_stops==2);
    reset(); fail_drive=true; assert(!approach_beacon() && forwards_count==1 && !backwards_count);
    reset(); fail_turn=true; assert(!approach_beacon() && !forwards_count);
    const double targets[4][2]={{2,0},{0,2},{-2,0},{0,-2}};
    for (unsigned i=0;i<4;++i) {
        reset(); model=true; beacon_x=targets[i][0]; beacon_y=targets[i][1];
        bool arrived=approach_beacon();
        if (!arrived && i!=2) {
            fprintf(stderr,"Model target %u failed at (%.2f, %.2f), translation %u ms, faults %x\n",
                    i,robot_x,robot_y,translation_ms,fault_mask());
            for (char *row=strtok(log_text,"\n");row;row=strtok(NULL,"\n"))
                if (!strncmp(row,"FOLLOW:",7) || !strncmp(row,"RETURN:",7) ||
                    !strncmp(row,"DIRECTION",9) || !strncmp(row,"FALLBACK",8) || !strncmp(row,"FAILED",6))
                    fprintf(stderr,"%s\n",row);
        }
        if (i!=2) assert(arrived && hypot(robot_x-beacon_x,robot_y-beacon_y)<0.25);
        else {
            // Continuing the initial line allows a side target to establish a
            // decline and then be reached by a perpendicular. A rear target
            // still has no improving perpendicular in this ideal model.
            assert(!arrived && fault_test(FAULT_NO_DIRECTION));
            assert(strstr(log_text,"neither perpendicular direction confirmed improvement"));
        }
        assert(translation_ms<=LINE_TRANSLATION_MS);
    }
    for (int fail=0;fail<2;++fail) {
        reset(); ui_test=true; ui_stage=1; complete_first=true; ui_log_failure=fail;
        if (!setjmp(ui_exit)) firmware_main();
        assert(ui_replays==1 && ui_begins==1 && ui_finished && !motors_on);
        if (fail) assert(!ble_calls && !forwards_count && fault_test(FAULT_REPORT));
        else assert(strstr(log_text,"RESULT: PAYLOAD CONFIRMED"));
    }
    puts("Navigation v17: coarse steps, strict recovery, three-of-four decline, recent strong regions, bounded exploration, fine return selection, BLE completion, STOP, faults, budgets, radio and startup checks passed");
    return 0;
}
