#include <assert.h>
#include <stdarg.h>
#include <stdlib.h>
#define main firmware_main
#include "../getting_there.c"
#undef main

typedef struct {
    bool forwards;
    uint32_t ms;
    float heading;
    int samples_before;
    int position_after;
} drive_event_t;
typedef struct { uint32_t ms; float heading; } position_leg_t;
static uint64_t sim_time;
static bool stopped, motors_on, wifi_missing, fail_drive, fail_reverse;
static bool stop_during_pause, stop_during_reverse, complete_first, complete_probe;
static int wifi_calls, probe_forwards, probe_backwards, approach_steps, reverse_steps;
static int ble_calls, sign_checks, tone_starts, tone_stops, retry_mode;
static int complete_after_step, complete_after_reverse;
static const int *scan_script;
static size_t scan_script_length;
static float current_heading, last_approach_heading;
static position_leg_t position[MAX_APPROACH_STEPS + 4];
static int position_count;
static drive_event_t drive_events[2 * MAX_APPROACH_STEPS + 16];
static int drive_event_count;
static float turn_events[3 * MAX_APPROACH_STEPS + 16];
static int turn_event_count;
static char log_text[65536];
static size_t log_length;
int cyw43_state;

uint64_t time_us_64(void) { return sim_time; }
void sleep_ms(uint32_t ms) { sim_time += (uint64_t)ms * 1000; }
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
void stdio_init_all(void) { }
bool stdio_usb_connected(void) { return false; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
uint32_t clock_get_hz(int clock) { (void)clock; return 125000000; }
pwm_config pwm_get_default_config(void) { return (pwm_config){0}; }
void pwm_config_set_clkdiv(pwm_config *config, float divider) { (void)config; (void)divider; }
void pwm_config_set_wrap(pwm_config *config, uint16_t wrap) { config->wrap = wrap; }
void pwm_init(uint slice, const pwm_config *config, bool enabled) { (void)slice; (void)config; (void)enabled; }
uint pwm_gpio_to_slice_num(uint pin) { return pin / 2; }
void pwm_set_gpio_level(uint pin, uint16_t level)
{
    assert(pin == 22 && !motors_on);
    if (level) ++tone_starts; else ++tone_stops;
}
bool motion_init(void) { return true; }
bool motion_calibrate(void) { sleep_ms(2000); return !stopped; }
bool motion_learn_direction(void) { ++sign_checks; return !stopped; }
bool motion_turn_to_heading(float heading)
{
    if (stopped) return false;
    assert(turn_event_count < (int)(sizeof(turn_events) / sizeof(*turn_events)));
    turn_events[turn_event_count++] = heading;
    current_heading = heading;
    return true;
}
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
    assert(!stopped && heading == current_heading);
    assert(drive_event_count < (int)(sizeof(drive_events) / sizeof(*drive_events)));
    drive_event_t *event = &drive_events[drive_event_count++];
    *event = (drive_event_t){forwards, ms, heading, wifi_calls / SCANS_PER_SAMPLE, position_count};
    if (ms == PROBE_MOVE_MS) {
        if (forwards) ++probe_forwards; else ++probe_backwards;
    } else {
        assert(ms == 300 && ms == APPROACH_STEP_MS);
        if (forwards) { ++approach_steps; last_approach_heading = heading; }
        else if (stop_during_reverse) { stopped = true; return false; }
    }
    sleep_ms(ms + 300);
    if (fail_drive || (!forwards && ms == APPROACH_STEP_MS && fail_reverse)) return false;
    if (forwards) {
        assert(position_count < (int)(sizeof(position) / sizeof(*position)));
        position[position_count++] = (position_leg_t){ms, heading};
    } else {
        // Each reverse must retrace the most recent successful forward leg.
        // This detects returning in the current heading instead of the stored route.
        assert(position_count > 0);
        assert(position[position_count - 1].ms == ms);
        assert(position[position_count - 1].heading == heading);
        --position_count;
        if (ms == APPROACH_STEP_MS) ++reverse_steps;
    }
    event->position_after = position_count;
    motors_on = false;
    return true;
}
void payload_client_init(void) { }
bool payload_client_ready(void) { return true; }
const uint8_t *payload_client_data(uint16_t *length) { *length = 0; return NULL; }
payload_result_t payload_client_attempt(void)
{
    assert(!motors_on);
    ++ble_calls;
    if (complete_first || (complete_probe && probe_forwards) ||
        (complete_after_step > 0 && approach_steps >= complete_after_step) ||
        (complete_after_reverse > 0 && reverse_steps >= complete_after_reverse)) return PAYLOAD_COMPLETE;
    if (retry_mode == 1) return PAYLOAD_RETRY;
    if (retry_mode == 2) return ble_calls == 1 ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE;
    return PAYLOAD_NOT_CLOSE;
}
int cyw43_arch_init(void) { return 0; }
void cyw43_arch_enable_sta_mode(void) { }
int cyw43_wifi_scan(int *state, cyw43_wifi_scan_options_t *options, void *context,
                   int (*callback)(void *, const cyw43_ev_scan_result_t *))
{
    (void)state; (void)options;
    assert(!motors_on);
    int group = wifi_calls++ / SCANS_PER_SAMPLE;
    if (wifi_missing) return 0;
    int rssi = -70;
    if (scan_script) {
        assert((size_t)group < scan_script_length);
        rssi = scan_script[group];
    }
    if (rssi == RSSI_MISSING) return 0;
    cyw43_ev_scan_result_t result = {.rssi = (int16_t)rssi, .ssid_len = 11};
    memcpy(result.ssid, "I AM PICO W", 11);
    callback(context, &result);
    return 0;
}
bool cyw43_wifi_scan_active(int *state) { (void)state; return false; }

static void reset(void)
{
    sim_time = 0; stopped = motors_on = false;
    wifi_calls = probe_forwards = probe_backwards = approach_steps = reverse_steps = 0;
    ble_calls = sign_checks = tone_starts = tone_stops = retry_mode = 0;
    complete_after_step = complete_after_reverse = 0;
    wifi_missing = fail_drive = fail_reverse = complete_first = complete_probe = false;
    scan_script = NULL; scan_script_length = 0;
    stop_during_pause = stop_during_reverse = false;
    current_heading = last_approach_heading = 0;
    position_count = drive_event_count = turn_event_count = 0;
    log_length = 0; log_text[0] = 0;
    run_started_ms = 0;
}
static void scans(const int *script, size_t length)
{
    scan_script = script;
    scan_script_length = length;
}
#define USE_SCANS(script) scans(script, sizeof(script) / sizeof(*(script)))

static void seed_route(navigation_t *navigation)
{
    *navigation = (navigation_t){.heading = -15, .best_heading = 0,
                                .path_count = 3, .best_path_count = 1, .best_rssi = -60};
    const float route[] = {0, 15, -15};
    for (int i = 0; i < 3; ++i) {
        navigation->path_heading[i] = route[i];
        assert(motion_turn_to_heading(route[i]));
        assert(motion_drive(true, APPROACH_STEP_MS, route[i]));
    }
    drive_event_count = turn_event_count = 0;
}

int main(void)
{
    reset(); complete_first = true;
    assert(approach_beacon());
    assert(!wifi_calls && !probe_forwards && !approach_steps && !sign_checks && !tone_starts);
    buzzer_init(); tone_stops = 0;
    uint64_t before = sim_time;
    completion_sound();
    assert(tone_starts == 1 && tone_stops == 1 && sim_time - before == 1000000);
    reset(); stopped = true; completion_sound(); assert(!tone_starts);
    reset(); stop_during_pause = true; completion_sound();
    assert(tone_starts == 1 && tone_stops == 1 && sim_time == 100000);

    const int supplied[] = {-71, -71, -75, -65};
    const int tied[] = {-71, -71, -71, -71};
    const int one[] = {-127, -127, -90, -127};
    const int missing[] = {-127, -127, -127, -127};
    const int distinct[] = {-72, -65, -70, -60};
    assert(second_strongest_heading(supplied) == 0);
    assert(second_strongest_heading(tied) == 1);
    assert(second_strongest_heading(one) == 2);
    assert(second_strongest_heading(missing) == -1);
    assert(second_strongest_heading(distinct) == 1);

    // The reported strongest rightward score must now select the next-ranked probe.
    const int report_fixture[] = {-74, -71, -71, -75, -65, -74};
    reset(); USE_SCANS(report_fixture); complete_after_step = 1;
    assert(approach_beacon());
    assert(probe_forwards == 4 && probe_backwards == 4 && approach_steps == 1);
    assert(last_approach_heading == 0 && sign_checks == 1 && !tone_starts);
    assert(position_count == 1 && strstr(log_text, "second-highest valid reading"));

    reset(); complete_probe = true;
    assert(approach_beacon());
    assert(probe_forwards == 1 && !probe_backwards && !approach_steps);

    // Equal scores retain the heading for ten short steps without another pass.
    reset(); complete_after_step = 10;
    assert(approach_beacon());
    assert(probe_forwards == 4 && probe_backwards == 4 && approach_steps == 10);
    assert(last_approach_heading == 90 && !reverse_steps && position_count == 10);

    // Probe scores are from endpoints already left; compare progress to the origin.
    const int improving[] = {-80, -60, -65, -70, -75, -80,
                             -79, -78, -77, -76, -75, -74, -73};
    reset(); USE_SCANS(improving); complete_after_step = 8;
    assert(approach_beacon());
    assert(probe_forwards == 4 && approach_steps == 8 && last_approach_heading == 90);
    assert(!reverse_steps && position_count == 8);

    // A decline is confirmed at the same position. Recovery keeps the current route.
    const int brief_dip[] = {-75, -55, -60, -70, -80, -75,
                             -70, -71, -70, -69};
    reset(); USE_SCANS(brief_dip); complete_after_step = 4;
    assert(approach_beacon());
    assert(approach_steps == 4 && !reverse_steps && position_count == 4);
    assert(last_approach_heading == 90 && strstr(log_text, "SIGNAL RECOVERED"));
    assert(drive_events[10].forwards && drive_events[10].samples_before == 9);

    // Even a 1 dBm confirmed decline reverses before another forward movement.
    const int confirmed_drop[] = {-75, -55, -60, -70, -80, -75,
                                  -70, -71, -72, -70};
    reset(); USE_SCANS(confirmed_drop); complete_after_step = 3;
    assert(approach_beacon());
    assert(probe_forwards == 4 && approach_steps == 3 && reverse_steps == 1);
    assert(position_count == 2 && last_approach_heading == 105);
    assert(!drive_events[10].forwards && drive_events[10].heading == 90);
    assert(drive_events[10].samples_before == 9 && drive_events[10].position_after == 1);
    assert(drive_events[11].forwards && drive_events[11].heading == 105);
    assert(drive_events[11].samples_before == 10);
    assert(strstr(log_text, "SIGNAL DECREASE CONFIRMED") && strstr(log_text, "RETURN COMPLETE"));

    // Missing readings wait without moving. Recovery does not repeat four probes.
    const int recovered_signal[] = {-75, -55, -60, -70, -80, -75, -127, -127, -70};
    reset(); USE_SCANS(recovered_signal); complete_after_step = 2;
    assert(approach_beacon());
    assert(probe_forwards == 4 && approach_steps == 2 && !reverse_steps);
    assert(last_approach_heading == 90 && drive_events[9].samples_before == 9);

    const int lost_signal[] = {-75, -55, -60, -70, -80, -75,
                               -127, -127, -127, -127};
    reset(); USE_SCANS(lost_signal);
    assert(!approach_beacon());
    assert(probe_forwards == 4 && approach_steps == 1 && !reverse_steps && !tone_starts);

    // Return a bent route in reverse order, stopping exactly at the recorded best.
    navigation_t navigation;
    reset(); seed_route(&navigation);
    assert(return_to_best(&navigation) == PAYLOAD_NOT_CLOSE);
    assert(position_count == 1 && navigation.path_count == navigation.best_path_count);
    assert(drive_event_count == 2 && reverse_steps == 2);
    assert(drive_events[0].heading == -15 && drive_events[1].heading == 15);
    assert(turn_event_count == 3 && turn_events[2] == 0 && navigation.heading == 0);

    // A verified payload during the return prevents the remaining reverse legs.
    reset(); seed_route(&navigation); complete_after_reverse = 1;
    assert(return_to_best(&navigation) == PAYLOAD_COMPLETE);
    assert(position_count == 2 && navigation.path_count == 2 && drive_event_count == 1);

    reset(); seed_route(&navigation); fail_reverse = true;
    assert(return_to_best(&navigation) == PAYLOAD_FATAL);
    assert(position_count == 3 && navigation.path_count == 3 && drive_event_count == 1);
    reset(); seed_route(&navigation); stop_during_reverse = true;
    assert(return_to_best(&navigation) == PAYLOAD_FATAL);
    assert(stopped && navigation.path_count == 3 && drive_event_count == 1);

    // An interrupted return cannot turn towards an alternative or drive forwards.
    reset(); USE_SCANS(confirmed_drop); fail_reverse = true;
    assert(!approach_beacon());
    assert(approach_steps == 2 && !reverse_steps && drive_event_count == 11);
    assert(!strstr(log_text, "TRY NEARBY HEADING"));
    reset(); USE_SCANS(confirmed_drop); stop_during_reverse = true;
    assert(!approach_beacon());
    assert(stopped && approach_steps == 2 && !strstr(log_text, "TRY NEARBY HEADING"));

    // Alternatives are relative to the best heading and stop after six attempts.
    reset(); navigation = (navigation_t){.heading = 90, .best_heading = 90, .best_rssi = -60};
    const float alternatives[] = {105, 75, 120, 60, 135, 45};
    for (int i = 0; i < 6; ++i) {
        assert(try_alternative(&navigation) == PAYLOAD_NOT_CLOSE);
        assert(navigation.heading == alternatives[i] && navigation.best_heading == 90);
        assert(navigation.best_rssi == -60 && navigation.alternative_index == i + 1);
    }
    int turns_before = turn_event_count, samples_before = wifi_calls;
    assert(try_alternative(&navigation) == PAYLOAD_FATAL);
    assert(turn_event_count == turns_before && wifi_calls == samples_before && !approach_steps);

    // Tied readings save the latest position. Forward best readings reset offsets.
    reset(); navigation = (navigation_t){.heading = 15, .best_heading = 0,
                                        .path_count = 2, .best_path_count = 1,
                                        .best_rssi = -60, .alternative_index = 4};
    record_best(&navigation, -60);
    assert(navigation.best_path_count == 2 && navigation.best_heading == 15);
    assert(navigation.alternative_index == 0);
    navigation.path_count = 3; navigation.heading = 30; navigation.alternative_index = 1;
    record_best(&navigation, -61);
    assert(navigation.best_path_count == 2 && navigation.best_heading == 15);
    assert(navigation.best_rssi == -60 && navigation.alternative_index == 1);

    reset(); wifi_missing = true;
    assert(!approach_beacon());
    assert(wifi_calls == 15 && !probe_forwards && !approach_steps && !sign_checks && !tone_starts);
    reset(); retry_mode = 1;
    assert(!approach_beacon());
    assert(ble_calls == 3 && !wifi_calls && !approach_steps && !tone_starts);
    reset(); retry_mode = 2;
    assert(!approach_beacon());
    assert(ble_calls == 3 && !approach_steps);
    reset(); fail_drive = true;
    assert(!approach_beacon());
    assert(probe_forwards == 1 && !probe_backwards && !approach_steps && !tone_starts);
    reset(); stopped = true;
    assert(!approach_beacon() && !ble_calls && !probe_forwards && !tone_starts);
    navigation = (navigation_t){.path_count = 1, .best_path_count = 0};
    assert(return_to_best(&navigation) == PAYLOAD_FATAL);
    assert(!drive_event_count && !turn_event_count);
    reset(); assert(!approach_beacon());
    assert(approach_steps == MAX_APPROACH_STEPS && probe_forwards == 4 && !reverse_steps && !tone_starts);
    reset(); sim_time = (uint64_t)MAX_RUN_MS * 1000;
    assert(!run_can_continue());
    puts("Second-ranked direction, short forward steps, confirmed decline, route return, nearby alternatives and completion: passed");
    return 0;
}
