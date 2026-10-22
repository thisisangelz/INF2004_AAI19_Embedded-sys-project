// Wi-Fi RSSI hill-climb for comparing environments. Same approach as getting_there
// (4-direction probe pass, then 750 ms steps towards the strongest heading), but
// with no IMU: no gyro calibration, turn-sign learning or heading hold, and no BLE
// payload transfer. Turns and drives are timed and open-loop. Every raw reading and
// every filtered sample is logged as CSV so runs in different places can be compared.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "run_report.h"

#define AP1_SSID "I AM PICO W"
#define RSSI_MISSING (-127)

// Robo Pico motor driver, same wiring and polarity as getting_there.
#define M1_A 8
#define M1_B 9
#define M2_A 10
#define M2_B 11
#define START_BUTTON 20
#define STOP_BUTTON 21
#define M1_FORWARD_USES_A 1
#define M2_FORWARD_USES_A 0

#define DRIVE_DUTY_PERCENT 35
#define TURN_DUTY_PERCENT 25
// Open-loop: tune on your floor and battery level until a "90 degree" turn is 90.
#define TURN_90_MS 450
#define MOTOR_SETTLE_MS 300
#define PROBE_COUNT 4
#define PROBE_MOVE_MS 2400
#define APPROACH_STEP_MS 750
#define REPASS_EVERY_STEPS 4
#define MAX_APPROACH_STEPS 40
#define MAX_RUN_MS 900000
#define SCANS_PER_SAMPLE 5
#define MIN_VALID_SCANS 3
#define SCAN_TIMEOUT_MS 15000
// Stop once a filtered sample is at least this strong. Set to 0 to run all steps.
#define STOP_AT_DBM (-35)

static volatile int scan_best_dbm;
static bool stop_latched;
static uint32_t run_started_ms;
static int nominal_heading; // commanded clockwise degrees; open-loop, drifts in reality
static const char *phase = "start";
static int step_number;

static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static bool stop_pressed(void)
{
    if (!gpio_get(STOP_BUTTON)) stop_latched = true;
    return stop_latched;
}

static void motor_write(uint pin_a, uint pin_b, int speed, bool forward_uses_a)
{
    bool use_a = speed >= 0 ? forward_uses_a : !forward_uses_a;
    uint16_t level = (uint16_t)((speed < 0 ? -speed : speed) * 100u);
    pwm_set_gpio_level(pin_a, speed != 0 && use_a ? level : 0);
    pwm_set_gpio_level(pin_b, speed != 0 && !use_a ? level : 0);
}

static void motors_set(int left, int right)
{
    if (stop_pressed()) left = right = 0;
    motor_write(M1_A, M1_B, left, M1_FORWARD_USES_A);
    motor_write(M2_A, M2_B, right, M2_FORWARD_USES_A);
}

static void motors_init(void)
{
    const uint pins[] = {M1_A, M1_B, M2_A, M2_B};
    for (uint i = 0; i < 4; ++i) {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pins[i]);
        pwm_set_wrap(slice, 9999);
        pwm_set_clkdiv(slice, 1.25f);
        pwm_set_gpio_level(pins[i], 0);
        pwm_set_enabled(slice, true);
    }
    gpio_init(START_BUTTON); gpio_set_dir(START_BUTTON, GPIO_IN); gpio_pull_up(START_BUTTON);
    gpio_init(STOP_BUTTON); gpio_set_dir(STOP_BUTTON, GPIO_IN); gpio_pull_up(STOP_BUTTON);
    motors_set(0, 0);
}

static bool start_pressed(void) { return !gpio_get(START_BUTTON); }

static bool pause_checked(uint32_t duration_ms)
{
    uint32_t start = now_ms();
    while ((uint32_t)(now_ms() - start) < duration_ms) {
        if (stop_pressed()) { motors_set(0, 0); return false; }
        sleep_ms(5);
    }
    return true;
}

static bool run_can_continue(void)
{
    if (stop_pressed()) return false;
    if ((uint32_t)(now_ms() - run_started_ms) >= MAX_RUN_MS) {
        motors_set(0, 0);
        report_printf("EVENT,%lu,FAILED,15 minute run limit\n", (unsigned long)now_ms());
        return false;
    }
    return true;
}

// Timed move, then motors off and a settle pause so the next scan is taken at rest.
static bool timed_move(int left, int right, uint32_t duration_ms)
{
    motors_set(left, right);
    bool ok = pause_checked(duration_ms);
    motors_set(0, 0);
    return ok && pause_checked(MOTOR_SETTLE_MS);
}

static bool drive(bool forwards, uint32_t duration_ms)
{
    int duty = forwards ? DRIVE_DUTY_PERCENT : -DRIVE_DUTY_PERCENT;
    return timed_move(duty, duty, duration_ms);
}

static bool turn_clockwise_90(void)
{
    if (!timed_move(TURN_DUTY_PERCENT, -TURN_DUTY_PERCENT, TURN_90_MS)) return false;
    nominal_heading = (nominal_heading + 90) % 360;
    return true;
}

static int wifi_result(void *unused, const cyw43_ev_scan_result_t *result)
{
    (void)unused;
    size_t length = strlen(AP1_SSID);
    if (result && result->ssid_len == length && result->rssi < 0 &&
        result->rssi > scan_best_dbm && memcmp(result->ssid, AP1_SSID, length) == 0)
        scan_best_dbm = result->rssi;
    return 0;
}

// Five scans, trimmed mean (lowest and highest quarter dropped), as in getting_there.
// Every raw scan and the sample summary are logged as CSV.
static bool sample_ap1(int *score)
{
    int readings[SCANS_PER_SAMPLE], count = 0;
    for (int round = 0; round < SCANS_PER_SAMPLE; ++round) {
        if (!run_can_continue()) return false;
        scan_best_dbm = RSSI_MISSING;
        cyw43_wifi_scan_options_t options = {0};
        if (cyw43_wifi_scan(&cyw43_state, &options, NULL, wifi_result) != 0) {
            report_printf("EVENT,%lu,FAILED,Wi-Fi scan could not start\n", (unsigned long)now_ms());
            return false;
        }
        uint32_t start = now_ms();
        while (cyw43_wifi_scan_active(&cyw43_state)) {
            if (!run_can_continue() || (uint32_t)(now_ms() - start) >= SCAN_TIMEOUT_MS) {
                report_printf("EVENT,%lu,FAILED,Wi-Fi scan stopped or timed out\n", (unsigned long)now_ms());
                return false;
            }
            sleep_ms(10);
        }
        int dbm = scan_best_dbm;
        if (dbm != RSSI_MISSING) readings[count++] = dbm;
        report_printf("RAW,%lu,%d,%d,%s,%d,%d\n", (unsigned long)now_ms(), step_number,
                      nominal_heading, phase, round + 1, dbm);
    }
    *score = RSSI_MISSING;
    int lowest = 0, highest = 0;
    if (count) {
        for (int i = 1; i < count; ++i) {
            int value = readings[i], j = i;
            while (j > 0 && readings[j - 1] > value) { readings[j] = readings[j - 1]; --j; }
            readings[j] = value;
        }
        lowest = readings[0];
        highest = readings[count - 1];
    }
    if (count >= MIN_VALID_SCANS) {
        int trim = count / 4, sum = 0, kept = count - 2 * trim;
        for (int i = trim; i < count - trim; ++i) sum += readings[i];
        *score = (sum - kept / 2) / kept;
    }
    report_printf("SAMPLE,%lu,%d,%d,%s,%d,%d,%d,%d\n", (unsigned long)now_ms(), step_number,
                  nominal_heading, phase, count, *score, lowest, highest);
    return !stop_pressed();
}

// Probe four headings 90 degrees apart, then turn to the strongest one.
static bool choose_direction(void)
{
    phase = "probe";
    int scores[PROBE_COUNT], best = -1;
    report_printf("EVENT,%lu,PASS,four %d ms probes\n", (unsigned long)now_ms(), PROBE_MOVE_MS);
    for (int i = 0; i < PROBE_COUNT; ++i) {
        if (!run_can_continue()) return false;
        if (i > 0 && !turn_clockwise_90()) return false;
        if (!drive(true, PROBE_MOVE_MS) || !sample_ap1(&scores[i])) return false;
        if (scores[i] != RSSI_MISSING && (best < 0 || scores[i] > scores[best])) best = i;
        if (!drive(false, PROBE_MOVE_MS)) return false;
    }
    if (!turn_clockwise_90()) return false; // 270 -> 360: back to the pass origin
    if (best < 0) {
        report_printf("EVENT,%lu,FAILED,no valid RSSI at any probe heading\n", (unsigned long)now_ms());
        return false;
    }
    report_printf("EVENT,%lu,SELECTED,%d degrees,%d dBm\n", (unsigned long)now_ms(), best * 90, scores[best]);
    for (int i = 0; i < best; ++i)
        if (!turn_clockwise_90()) return false;
    return true;
}

static bool approach(void)
{
    run_started_ms = now_ms();
    int score = RSSI_MISSING;
    step_number = 0;
    phase = "start";
    if (!sample_ap1(&score)) return false;
    if (score == RSSI_MISSING) {
        report_printf("EVENT,%lu,FAILED,AP1 not seen at start\n", (unsigned long)now_ms());
        return false;
    }
    bool need_pass = true;
    int since_pass = 0;
    for (step_number = 1; step_number <= MAX_APPROACH_STEPS; ++step_number) {
        if (!run_can_continue()) return false;
        if (need_pass) {
            if (!choose_direction()) return false;
            need_pass = false;
            since_pass = 0;
        }
        phase = "step";
        if (!drive(true, APPROACH_STEP_MS) || !sample_ap1(&score)) return false;
        if (score == RSSI_MISSING) { need_pass = true; continue; }
        if (STOP_AT_DBM && score >= STOP_AT_DBM) {
            report_printf("EVENT,%lu,DONE,reached %d dBm\n", (unsigned long)now_ms(), score);
            return true;
        }
        if (++since_pass >= REPASS_EVERY_STEPS) need_pass = true;
    }
    report_printf("EVENT,%lu,DONE,step limit reached\n", (unsigned long)now_ms());
    return true;
}

int main(void)
{
    stdio_init_all();
    report_init();
    motors_init();
    sleep_ms(2000);
    bool ready = !cyw43_arch_init();
    if (ready) cyw43_arch_enable_sta_mode();
    report_printf("rssi_env_test | no IMU, no calibration | GP20 start | GP21 stop and latch\n");
    report_printf("CSV: RAW,t_ms,step,heading,phase,scan,dbm | SAMPLE,t_ms,step,heading,phase,valid,mean,min,max | dbm -127 = not seen\n");
    if (!ready) report_printf("FAILED: Wi-Fi initialisation\n");
    else {
        report_printf("Waiting for GP20.\n");
        while (!start_pressed() && !stop_pressed()) sleep_ms(10);
        while (start_pressed() && !stop_pressed()) sleep_ms(10);
        if (!stop_pressed()) {
            bool ok = approach();
            motors_set(0, 0);
            report_printf("RESULT: %s\n", ok ? "RUN FINISHED" : "STOPPED EARLY");
        }
    }
    motors_set(0, 0);
    report_printf("Run ended. Connect Pico USB to read the report; GP20 prints it again.\n");
    bool was_connected = false, last_start = false;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !was_connected) { sleep_ms(500); report_replay(); }
        bool start = start_pressed();
        if (connected && start && !last_start) report_replay();
        was_connected = connected;
        last_start = start;
        sleep_ms(20);
    }
}
