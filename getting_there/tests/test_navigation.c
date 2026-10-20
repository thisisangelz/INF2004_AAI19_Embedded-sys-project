#include <assert.h>
#include <stdarg.h>
#include <stdlib.h>
#define main firmware_main
#include "../getting_there.c"
#undef main

static uint64_t sim_time;
static bool stopped;
static bool motors_on;
static int wifi_calls;
static int probe_forwards;
static int probe_backwards;
static int approach_steps;
static int ble_calls;
static int sign_checks;
static int cue_counts[CUE_BUTTON + 1];
static int cue_stops;
static uint64_t cue_end;
static uint64_t stop_at;
static int saves;
static bool wifi_missing;
static bool direction_fixture;
static bool fail_drive;
static bool stop_during_pause;
static int complete_after_step;
static int retry_mode;
static bool complete_first;
static bool complete_probe;
static float current_heading;
static float last_approach_heading;
static char log_text[32768];
static size_t log_length;
int cyw43_state;
uint64_t time_us_64(void) { return sim_time; }
void sleep_ms(uint32_t ms)
{
    sim_time += (uint64_t)ms * 1000;
    if (stop_at && sim_time >= stop_at) stopped = true;
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
void report_replay(void) { }
void report_begin_run(void) { }
void report_save(void) { assert(!motors_on); ++saves; }
bool report_print_saved(void) { return true; }
void cues_init(void) { }
void cue_play(robot_cue_t cue)
{
    ++cue_counts[cue];
    uint64_t length = cue == CUE_COLLECTED ? 1000000 :
                      cue == CUE_CALIBRATING || cue == CUE_IN_RANGE ? UINT64_MAX / 2 : 100000;
    cue_end = sim_time + length;
}
void cue_stop(void) { ++cue_stops; cue_end = sim_time; }
bool cue_busy(void) { return sim_time < cue_end; }
void payload_client_set_close_callback(void (*callback)(void)) { (void)callback; }
void stdio_init_all(void) { }
bool stdio_usb_connected(void) { return false; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
bool motion_init(void) { return true; }
bool motion_calibrate(void) { sleep_ms(2000); return !stopped; }
bool motion_learn_direction(void) { ++sign_checks; return !stopped; }
bool motion_turn_to_heading(float heading) { current_heading = heading; return !stopped; }
float motion_heading(void) { return current_heading; }
bool motion_pause_ms(uint32_t ms)
{
    if (stop_during_pause) { sleep_ms(100); stopped = true; return false; }
    sleep_ms(ms); return !stopped;
}
bool motion_stop_requested(void) { return stopped; }
bool motion_start_pressed(void) { return false; }
void motion_stop(void) { motors_on = false; }
bool motion_drive(bool forwards, uint32_t ms, float heading)
{
    assert(!stopped);
    if (ms == 2400) { if (forwards) ++probe_forwards; else ++probe_backwards; }
    else { assert(forwards && ms == 750); ++approach_steps; last_approach_heading = heading; }
    sleep_ms(ms + 300);
    return !fail_drive;
}
void payload_client_init(void) { }
bool payload_client_ready(void) { return true; }
const uint8_t *payload_client_data(uint16_t *length) { *length = 0; return NULL; }
payload_result_t payload_client_attempt(void)
{
    assert(!motors_on);
    ++ble_calls;
    payload_result_t result = PAYLOAD_NOT_CLOSE;
    if (complete_first || (complete_probe && probe_forwards) ||
        (complete_after_step > 0 && approach_steps >= complete_after_step)) result = PAYLOAD_COMPLETE;
    else if (retry_mode == 1) result = PAYLOAD_RETRY;
    else if (retry_mode == 2) result = ble_calls == 1 ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE;
    // The real client calls the close callback once BLE range is confirmed.
    if (result != PAYLOAD_NOT_CLOSE) on_in_range();
    return result;
}
int cyw43_arch_init(void) { return 0; }
void cyw43_arch_enable_sta_mode(void) { }
int cyw43_wifi_scan(int *state, cyw43_wifi_scan_options_t *options, void *context,
                   int (*callback)(void *, const cyw43_ev_scan_result_t *))
{
    (void)state; (void)options;
    assert(!motors_on);
    int group = wifi_calls++ / 5;
    if (wifi_missing) return 0;
    const int fixture[] = {-74, -70, -70, -127, -69};
    int rssi = direction_fixture && group < 5 ? fixture[group] : -70;
    if (rssi == -127) return 0;
    cyw43_ev_scan_result_t result = {.rssi = (int16_t)rssi, .ssid_len = 11};
    memcpy(result.ssid, "I AM PICO W", 11);
    callback(context, &result);
    return 0;
}
bool cyw43_wifi_scan_active(int *state) { (void)state; return false; }

static void reset(void)
{
    sim_time = 0; stopped = motors_on = false;
    wifi_calls = probe_forwards = probe_backwards = approach_steps = ble_calls = sign_checks = 0;
    retry_mode = complete_after_step = 0;
    memset(cue_counts, 0, sizeof(cue_counts));
    cue_stops = saves = 0; cue_end = stop_at = 0;
    wifi_missing = direction_fixture = fail_drive = complete_first = complete_probe = false;
    stop_during_pause = false;
    current_heading = last_approach_heading = 0;
    log_length = 0; log_text[0] = 0;
}

int main(void)
{
    reset(); complete_first = true;
    assert(approach_beacon());
    assert(!wifi_calls && !probe_forwards && !approach_steps && !sign_checks);
    // The calibration tone and the in-range beeps both stop when their stage ends.
    assert(cue_counts[CUE_CALIBRATING] == 1 && cue_counts[CUE_IN_RANGE] == 1 && cue_stops == 2);
    assert(!in_range_cue_on && !cue_counts[CUE_COLLECTED] && saves == 1);
    uint64_t before = sim_time;
    completion_sound();
    assert(cue_counts[CUE_COLLECTED] == 1 && sim_time - before == 1000000);
    reset(); stopped = true; completion_sound(); assert(!cue_counts[CUE_COLLECTED]);
    reset(); stop_at = 100000; completion_sound();
    assert(cue_counts[CUE_COLLECTED] == 1 && cue_stops == 1 && sim_time == 100000);

    reset(); direction_fixture = true; complete_after_step = 1;
    assert(approach_beacon());
    assert(probe_forwards == 4 && probe_backwards == 4 && approach_steps == 1);
    assert(last_approach_heading == 270 && sign_checks == 1 && !cue_counts[CUE_COLLECTED]);
    assert(cue_counts[CUE_PASS] == 1);

    reset(); complete_probe = true;
    assert(approach_beacon());
    assert(probe_forwards == 1 && !probe_backwards && !approach_steps);

    reset(); complete_after_step = 5;
    assert(approach_beacon());
    assert(probe_forwards == 8 && probe_backwards == 8 && approach_steps == 5);
    // Two direction passes; checkpoints after calibration, each pass and each step.
    assert(cue_counts[CUE_PASS] == 2 && saves == 1 + 2 + 4);

    reset(); wifi_missing = true;
    assert(!approach_beacon());
    assert(wifi_calls == 15 && !probe_forwards && !approach_steps && !sign_checks && !cue_counts[CUE_COLLECTED]);

    reset(); retry_mode = 1;
    assert(!approach_beacon());
    assert(ble_calls == 3 && !wifi_calls && !approach_steps && !cue_counts[CUE_COLLECTED]);
    // Each close attempt starts the in-range beeps and stops them when it ends.
    assert(cue_counts[CUE_IN_RANGE] == 3 && cue_stops == 1 + 3 && !in_range_cue_on);
    reset(); retry_mode = 2;
    assert(!approach_beacon());
    assert(ble_calls == 3 && !approach_steps);

    reset(); fail_drive = true;
    assert(!approach_beacon());
    assert(probe_forwards == 1 && !probe_backwards && !approach_steps && !cue_counts[CUE_COLLECTED]);

    reset(); stopped = true;
    assert(!approach_beacon() && !ble_calls && !probe_forwards && !cue_counts[CUE_COLLECTED]);

    reset(); assert(!approach_beacon());
    assert(approach_steps == 40 && !cue_counts[CUE_COLLECTED]);

    const int supplied[] = {-70, -70, -127, -69};
    const int tied[] = {-71, -71, -71, -71};
    const int one[] = {-127, -127, -90, -127};
    const int missing[] = {-127, -127, -127, -127};
    uint8_t packed[BITARRAY_BYTES(PROBE_COUNT, RSSI_BITS)];
    const int *cases[] = {supplied, tied, one, missing};
    const int expected[] = {3, 0, 2, -1};
    for (int c = 0; c < 4; ++c) {
        memset(packed, 0, sizeof(packed));
        for (int i = 0; i < PROBE_COUNT; ++i)
            bitarray_put(packed, i, RSSI_BITS, rssi_encode(cases[c][i]));
        assert(strongest_heading(packed) == expected[c]);
    }
    assert(sizeof(packed) == 4);
    reset(); run_started_ms = 0; sim_time = (uint64_t)MAX_RUN_MS * 1000;
    assert(!run_can_continue());
    puts("Navigation, stationary retries, limits, completion cues and flash checkpoints: passed");
    return 0;
}
