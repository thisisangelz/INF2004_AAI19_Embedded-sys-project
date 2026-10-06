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
static int tone_starts;
static int tone_stops;
static bool wifi_missing;
static bool direction_fixture;
static const int *scan_script;
static size_t scan_script_length;
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
    if (complete_first || (complete_probe && probe_forwards) ||
        (complete_after_step > 0 && approach_steps >= complete_after_step)) return PAYLOAD_COMPLETE;
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
    int group = wifi_calls++ / 5;
    if (wifi_missing) return 0;
    const int fixture[] = {-74, -70, -70, -127, -69};
    int rssi = direction_fixture && group < 5 ? fixture[group] : -70;
    if (scan_script) {
        assert((size_t)group < scan_script_length);
        rssi = scan_script[group];
    }
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
    tone_starts = tone_stops = retry_mode = complete_after_step = 0;
    wifi_missing = direction_fixture = fail_drive = complete_first = complete_probe = false;
    scan_script = NULL; scan_script_length = 0;
    stop_during_pause = false;
    current_heading = last_approach_heading = 0;
    log_length = 0; log_text[0] = 0;
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

    reset(); direction_fixture = true; complete_after_step = 1;
    assert(approach_beacon());
    assert(probe_forwards == 4 && probe_backwards == 4 && approach_steps == 1);
    assert(last_approach_heading == 270 && sign_checks == 1 && !tone_starts);

    reset(); complete_probe = true;
    assert(approach_beacon());
    assert(probe_forwards == 1 && !probe_backwards && !approach_steps);

    // Stable readings must keep the heading well beyond the former four-step limit.
    reset(); direction_fixture = true; complete_after_step = 10;
    assert(approach_beacon());
    assert(probe_forwards == 4 && probe_backwards == 4 && approach_steps == 10);
    assert(last_approach_heading == 270 && !strstr(log_text, "SIGNAL DECLINED"));

    // The selected probe endpoint is stronger than the starting position.
    // Forward progress must use the new stationary baseline, then follow improvement.
    const int improving[] = {-80, -55, -70, -75, -78, -80,
                             -79, -78, -77, -76, -75, -74, -73};
    reset(); scan_script = improving; scan_script_length = sizeof(improving) / sizeof(*improving);
    complete_after_step = 8;
    assert(approach_beacon());
    assert(probe_forwards == 4 && approach_steps == 8 && last_approach_heading == 0);
    assert(strstr(log_text, "FORWARD BASELINE: -80 dBm") && !strstr(log_text, "SIGNAL DECLINED"));

    // Isolated dips, including two consecutive dips, must not cause another pass.
    const int temporary_dips[] = {-74, -70, -73, -76, -78, -74,
                                  -80, -74, -80, -80, -74};
    reset(); scan_script = temporary_dips; scan_script_length = sizeof(temporary_dips) / sizeof(*temporary_dips);
    complete_after_step = 6;
    assert(approach_beacon());
    assert(probe_forwards == 4 && approach_steps == 6 && !strstr(log_text, "SIGNAL DECLINED"));

    // Three readings 4 dBm below the improved best trigger a new pass before step five.
    const int sustained_drop[] = {-74, -70, -73, -76, -78, -74,
                                  -68, -72, -72, -72, -75, -65, -73, -74, -72};
    reset(); scan_script = sustained_drop; scan_script_length = sizeof(sustained_drop) / sizeof(*sustained_drop);
    complete_after_step = 5;
    assert(approach_beacon());
    assert(probe_forwards == 8 && probe_backwards == 8 && approach_steps == 5);
    assert(last_approach_heading == 90 && strstr(log_text, "FORWARD SIGNAL DECLINED"));

    // A lost signal prevents the next step until detection and direction selection finish.
    const int recovered_signal[] = {-74, -70, -73, -76, -78, -74,
                                    -127, -127, -74, -75, -65, -73, -74, -72};
    reset(); scan_script = recovered_signal; scan_script_length = sizeof(recovered_signal) / sizeof(*recovered_signal);
    complete_after_step = 2;
    assert(approach_beacon());
    assert(probe_forwards == 8 && probe_backwards == 8 && approach_steps == 2);
    assert(last_approach_heading == 90 && strstr(log_text, "FORWARD SIGNAL LOST"));

    const int lost_signal[] = {-74, -70, -73, -76, -78, -74,
                               -127, -127, -127, -127};
    reset(); scan_script = lost_signal; scan_script_length = sizeof(lost_signal) / sizeof(*lost_signal);
    assert(!approach_beacon());
    assert(probe_forwards == 4 && approach_steps == 1 && !tone_starts);

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

    reset(); assert(!approach_beacon());
    assert(approach_steps == 40 && probe_forwards == 4 && !tone_starts);

    const int supplied[] = {-70, -70, -127, -69};
    const int tied[] = {-71, -71, -71, -71};
    const int one[] = {-127, -127, -90, -127};
    const int missing[] = {-127, -127, -127, -127};
    assert(strongest_heading(supplied) == 3 && strongest_heading(tied) == 0);
    assert(strongest_heading(one) == 2 && strongest_heading(missing) == -1);
    reset(); run_started_ms = 0; sim_time = (uint64_t)MAX_RUN_MS * 1000;
    assert(!run_can_continue());
    puts("Continued forward navigation, RSSI decline, lost-signal recovery, retries, limits and completion tone: passed");
    return 0;
}
