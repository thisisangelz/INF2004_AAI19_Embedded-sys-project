#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <setjmp.h>
#define main firmware_main
#include "../getting_there.c"
#undef main

static uint64_t sim_time;
static bool stopped, motors_on, wifi_missing, fail_drive, fail_turn, stop_during_pause;
static bool complete_first, stop_on_confirmation, expire_on_confirmation, fail_scan;
static int wifi_calls, forwards_count, backwards_count, ble_calls, sign_checks;
static int tone_starts, tone_stops, retry_mode, complete_after_step, fail_at_ble_call;
static float current_heading;
static const int *script;
static size_t script_length;
static int default_rssi;
static bool model;
static double robot_x, robot_y, beacon_x, beacon_y;
static struct { bool forwards; float heading; } moves[128];
static unsigned move_count;
static char log_text[32768];
static size_t log_length;
static bool ui_test, ui_armed, ui_finished, ui_radio, ui_log_failure;
static unsigned ui_stage, ui_replays, ui_begins;
static unsigned ui_calibrations;
static jmp_buf ui_exit;
int cyw43_state;

uint64_t time_us_64(void) { return sim_time; }
void sleep_ms(uint32_t ms)
{
    sim_time += (uint64_t)ms * 1000;
    if (ui_test && ui_finished) longjmp(ui_exit, 1);
    if (ui_test && ui_stage == 1 && ui_replays) ui_stage = 2;
}
void report_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int count = vsnprintf(log_text + log_length, sizeof(log_text) - log_length, format, args);
    va_end(args);
    if (count > 0 && (size_t)count < sizeof(log_text) - log_length) log_length += count;
}
void report_init(void) { }
void report_replay(void)
{
    if (ui_test) { assert(!ui_armed && !ui_radio && !ui_begins); ++ui_replays; }
}
bool report_begin(void)
{
    if (ui_test) {
        assert(ui_armed && !ui_radio && ui_replays == 1);
        ++ui_begins; ui_stage = 3;
    }
    return !ui_log_failure;
}
bool report_checkpoint(void) { assert(!motors_on); return true; }
bool report_finish(void)
{
    if (ui_test) { assert(!motors_on && !ui_radio); ui_finished = true; }
    return !ui_log_failure;
}
bool report_ok(void) { return true; }
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
bool motion_init(void) { return true; }
bool motion_calibrate(void)
{
    if (ui_test) { assert(ui_armed && !ui_radio); ++ui_calibrations; }
    sleep_ms(2000);
    return !stopped;
}
bool motion_learn_direction(void) { ++sign_checks; return !stopped; }
bool motion_turn_to_heading(float heading)
{ assert(!motors_on); current_heading = heading; return !stopped && !fail_turn; }
float motion_heading(void) { return current_heading; }
bool motion_pause_ms(uint32_t ms)
{
    if (stop_during_pause) { sleep_ms(100); stopped = true; return false; }
    sleep_ms(ms); return !stopped;
}
bool motion_stop_requested(void) { return stopped; }
bool motion_start_pressed(void) { return ui_test && ui_stage == 2; }
bool motion_replay_pressed(void) { return ui_test && ui_stage == 1; }
void motion_arm_stop(void) { ui_armed = true; }
void cyw43_arch_deinit(void) { ui_radio = false; }
void motion_stop(void) { motors_on = false; }
bool motion_drive(bool forwards, uint32_t ms, float heading)
{
    assert(!stopped && ms == APPROACH_STEP_MS && move_count < 128);
    moves[move_count].forwards = forwards;
    moves[move_count++].heading = heading;
    if (forwards) ++forwards_count; else ++backwards_count;
    motors_on = true;
    if (model) {
        double distance = (forwards ? 1 : -1) * 0.20;
        double radians = heading * 3.141592653589793 / 180.0;
        robot_x += distance * cos(radians);
        robot_y += distance * sin(radians);
    }
    sleep_ms(ms + 300);
    motors_on = false;
    return !fail_drive;
}
void payload_client_init(void) { }
bool payload_client_ready(void) { return true; }
const uint8_t *payload_client_data(uint16_t *length) { *length = 0; return NULL; }
payload_result_t payload_client_attempt(void)
{
    assert(!motors_on);
    ++ble_calls;
    if (fail_at_ble_call == ble_calls) return PAYLOAD_FATAL;
    if (complete_first || (complete_after_step > 0 && forwards_count >= complete_after_step) ||
        (model && hypot(robot_x - beacon_x, robot_y - beacon_y) < 0.25))
        return PAYLOAD_COMPLETE;
    if (retry_mode == 1) return PAYLOAD_RETRY;
    if (retry_mode == 2) return ble_calls == 1 ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE;
    return PAYLOAD_NOT_CLOSE;
}
int cyw43_arch_init(void)
{
    if (ui_test) assert(ui_armed && ui_begins == 1 && ui_calibrations == 1);
    ui_radio = true;
    return 0;
}
void cyw43_arch_enable_sta_mode(void) { }
int cyw43_wifi_scan(int *state, cyw43_wifi_scan_options_t *options, void *context,
                   int (*callback)(void *, const cyw43_ev_scan_result_t *))
{
    (void)state; (void)options;
    assert(!motors_on);
    unsigned group = wifi_calls++ / SCANS_PER_SAMPLE;
    if (fail_scan) return -1;
    if (stop_on_confirmation && group == 3) stopped = true;
    if (expire_on_confirmation && group == 3) sim_time = (uint64_t)MAX_RUN_MS * 1000;
    if (wifi_missing) return 0;
    int rssi = group < script_length ? script[group] : default_rssi;
    if (model) {
        double distance = fmax(0.10, hypot(robot_x - beacon_x, robot_y - beacon_y));
        // Large orientation bias; paired readings must cancel it.
        double orientation_bias = 10 * sin(current_heading * 3.141592653589793 / 180.0);
        rssi = (int)lround(-50 - 20 * log10(distance) + orientation_bias);
    }
    if (rssi == -127) return 0;
    cyw43_ev_scan_result_t result = {.rssi = (int16_t)rssi, .ssid_len = 11,
                                   .bssid = {1, 2, 3, 4, 5, 6}};
    memcpy(result.ssid, "I AM PICO W", 11);
    callback(context, &result);
    return 0;
}
bool cyw43_wifi_scan_active(int *state) { (void)state; return false; }

static void reset(void)
{
    ui_test = ui_armed = ui_finished = ui_radio = ui_log_failure = false;
    ui_stage = ui_replays = ui_begins = 0;
    ui_calibrations = 0;
    sim_time = 0;
    stopped = motors_on = wifi_missing = fail_drive = fail_turn = stop_during_pause = false;
    complete_first = stop_on_confirmation = expire_on_confirmation = fail_scan = model = false;
    wifi_calls = forwards_count = backwards_count = ble_calls = sign_checks = 0;
    tone_starts = tone_stops = retry_mode = complete_after_step = fail_at_ble_call = 0;
    script = NULL; script_length = 0; default_rssi = -70;
    current_heading = 0; robot_x = robot_y = beacon_x = beacon_y = 0;
    log_length = 0; log_text[0] = 0; move_count = 0; target.locked = false;
    for (int f = 0; f < FAULT_COUNT; ++f) fault_clear((fault_t)f);
}

static void check_packing_and_identity(void)
{
    uint8_t packed[BITARRAY_BYTES(12, RSSI_BITS)] = {0};
    for (unsigned slot = 0; slot < 12; ++slot) {
        for (int dbm = -127; dbm < 0; ++dbm) {
            unsigned previous[12];
            for (unsigned k = 0; k < 12; ++k) previous[k] = bitarray_get(packed, k, RSSI_BITS);
            bitarray_put(packed, slot, RSSI_BITS, rssi_encode(dbm));
            assert(rssi_decode(bitarray_get(packed, slot, RSSI_BITS)) == dbm);
            for (unsigned k = 0; k < 12; ++k)
                if (k != slot) assert(bitarray_get(packed, k, RSSI_BITS) == previous[k]);
        }
    }
    assert(sizeof(packed) == 11 && sizeof(wifi_sample_t) == 2);
    assert(rssi_encode(0) == 127 && rssi_encode(128) == 127 && rssi_encode(-128) == 127);
    wifi_sample_t baseline = {.code = 70}, current = {.code = 67};
    assert(progress_direction(baseline, current) == 1);
    current.code = 73; assert(progress_direction(baseline, current) == -1);
    current.code = 68; assert(progress_direction(baseline, current) == 0);
    current.code = 65; current.spread = 5;
    assert(progress_direction(baseline, current) == 0);
    current.code = 127; assert(progress_direction(baseline, current) == 0);

    reset(); wifi_sample_t measured;
    assert(sample_ap1(&measured) && measured.code == 70 && target.locked);
    cyw43_ev_scan_result_t other = {.rssi = -20, .ssid_len = 11, .bssid = {9, 9, 9, 9, 9, 9}};
    memcpy(other.ssid, "I AM PICO W", 11);
    wifi_result(NULL, &other);
    assert(scan_result.rssi_code == 70);
    memcpy(other.bssid, target_bssid, sizeof(target_bssid));
    wifi_result(NULL, &other);
    assert(scan_result.rssi_code == 20);
}

int main(void)
{
    reset(); complete_first = true;
    assert(approach_beacon());
    assert(!wifi_calls && !forwards_count && !sign_checks && !tone_starts);
    buzzer_init(); tone_stops = 0;
    uint64_t before = sim_time;
    completion_sound();
    assert(tone_starts == 1 && tone_stops == 1 && sim_time - before == 1000000);
    reset(); stopped = true; completion_sound(); assert(!tone_starts);
    reset(); stop_during_pause = true; completion_sound();
    assert(tone_starts == 1 && tone_stops == 1 && sim_time == 100000);

    // Confirmed weakening must trigger a retreat/replan after ONE step.
    static const int weakening[] = {-60, -60, -70, -71, -65, -60, -61};
    reset(); script = weakening; script_length = 7; complete_after_step = 3;
    assert(approach_beacon());
    assert(backwards_count == 1 && move_count == 4 && moves[1].forwards == false);
    assert(moves[0].heading == 0 && moves[2].heading == 90 && moves[3].heading == 90);
    assert(strstr(log_text, "REPLAN: signal confirmed weaker"));

    // Small gains accumulate against the same anchor until they are measurable.
    static const int gradual[] = {-70, -70, -69, -68, -67, -66};
    reset(); script = gradual; script_length = 6; complete_after_step = 4;
    assert(approach_beacon() && !backwards_count && current_heading == 0);
    assert(strstr(log_text, "PROGRESS CONFIRMED"));

    // One bad batch must not cause a turn if a fresh stationary batch disagrees.
    static const int spike[] = {-60, -60, -65, -60, -56, -56};
    reset(); script = spike; script_length = 6; complete_after_step = 3;
    assert(approach_beacon() && !backwards_count && current_heading == 0);
    assert(strstr(log_text, "PROGRESS UNCONFIRMED"));

    // Flat signal exhausts a bounded search, instead of choosing an arbitrary tie.
    reset(); assert(!approach_beacon());
    assert(forwards_count == 12 && backwards_count == 12 && fault_test(FAULT_NO_DIRECTION));
    assert(moves[0].heading == 0 && moves[6].heading == 90 &&
           moves[12].heading == 180 && moves[18].heading == 270);

    // A worse direction is rejected even when it would be the best of bad scores.
    static const int bad[] = {-40, -40, -60, -60, -40, -60, -60,
                             -40, -60, -60, -40, -60, -60};
    reset(); script = bad; script_length = 13;
    assert(!approach_beacon() && forwards_count == 4 && backwards_count == 4);

    reset(); wifi_missing = true;
    assert(!approach_beacon() && wifi_calls == 15 && !forwards_count && !sign_checks);
    reset(); fail_scan = true;
    assert(!approach_beacon() && !forwards_count && fault_test(FAULT_WIFI_SCAN));
    reset(); retry_mode = 1;
    assert(!approach_beacon() && ble_calls == 3 && !forwards_count);
    reset(); retry_mode = 2;
    assert(!approach_beacon() && ble_calls == 3 && !forwards_count);
    reset(); fail_drive = true;
    assert(!approach_beacon() && forwards_count == 1 && !backwards_count);
    reset(); fail_turn = true;
    assert(!approach_beacon() && !forwards_count);
    reset(); stopped = true;
    assert(!approach_beacon() && !forwards_count && !ble_calls);

    reset(); script = weakening; script_length = 7; stop_on_confirmation = true;
    assert(!approach_beacon() && forwards_count == 1 && !backwards_count);
    reset(); script = weakening; script_length = 7; expire_on_confirmation = true;
    assert(!approach_beacon() && forwards_count == 1 && !backwards_count);
    assert(fault_test(FAULT_RUN_LIMIT));
    reset(); fail_at_ble_call = 4; // Cleanup failure after first movement.
    assert(!approach_beacon() && forwards_count == 1 && !backwards_count);
    static const int lost[] = {-60, -60, -127, -127, -127};
    reset(); script = lost; script_length = 5;
    assert(!approach_beacon() && forwards_count == 1 && !backwards_count);

    // Continuous progress still obeys the total forward movement budget.
    int improving[82] = {-126, -126};
    for (int i = 0; i < 40; ++i) improving[2 + 2 * i] = improving[3 + 2 * i] = -123 + 3 * i;
    reset(); script = improving; script_length = 82;
    assert(!approach_beacon() && forwards_count == 40 && !backwards_count);
    assert(fault_test(FAULT_STEP_LIMIT));

    // Integration model: position-dependent path loss + large antenna bias.
    // This validates control logic only, not physical RSSI or wheel accuracy.
    const double targets[4][2] = {{1,0},{0,1},{-1,0},{0,-1}};
    for (unsigned i = 0; i < 4; ++i) {
        reset(); model = true; beacon_x = targets[i][0]; beacon_y = targets[i][1];
        assert(approach_beacon());
        assert(hypot(robot_x - beacon_x, robot_y - beacon_y) < 0.25);
        assert(forwards_count <= MAX_APPROACH_STEPS);
    }
    check_packing_and_identity();
    // Exercise the real main loop: replay before start, arm only at GP20,
    // persist before radio/motors, shut radio down before final save.
    for (int fail = 0; fail < 2; ++fail) {
        reset(); ui_test = true; ui_stage = 1; complete_first = true;
        ui_log_failure = fail;
        if (!setjmp(ui_exit)) firmware_main();
        assert(ui_replays == 1 && ui_begins == 1 && ui_finished && !motors_on);
        if (fail) assert(!ble_calls && !forwards_count && fault_test(FAULT_REPORT));
        else assert(strstr(log_text, "RESULT: PAYLOAD CONFIRMED"));
    }
    ui_test = false;
    puts("Navigation: weakening, cumulative gains, noise rejection, bounded search, STOP, limits, BSSID, packing and four-direction model passed");
    return 0;
}
