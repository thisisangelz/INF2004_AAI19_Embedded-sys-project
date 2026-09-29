#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/pwm.h"
#include "imu_mpu9250.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

// Robo Pico's built-in motor driver. M1 is assumed to be the left wheel.
#define M1_A 8
#define M1_B 9
#define M2_A 10
#define M2_B 11
#define IMU_SDA 6
#define IMU_SCL 7
#define START_BUTTON 20
#define STOP_BUTTON 21

// M1 is left and M2 is right. This polarity matches the reported right spin
// during the former forward command. Verify both wheels with the chassis lifted.
#define M1_FORWARD_USES_A 1
#define M2_FORWARD_USES_A 0

#define DRIVE_DUTY_PERCENT 35
#define TURN_DUTY_PERCENT 35
#define PROBE_MOVE_MS 1200
#define FINAL_MOVE_MS 250
#define MOTOR_SETTLE_MS 300
#define SCAN_TIMEOUT_MS 15000
#define TURN_TIMEOUT_MS 6000
#define TURN_START_TIMEOUT_MS 1000
#define MIN_RSSI_GAIN_DB 4
#define MIN_BEST_MARGIN_DB 3
#define MAX_STRAIGHT_YAW_DEGREES 25.0f

#define BEACON_COUNT 1
#define HEADING_COUNT 4
#define SCANS_PER_SAMPLE 5
#define MIN_VALID_SCANS 3
#define RSSI_MISSING (-127)
#define REPORT_CAPACITY 8192

static const char *const ssids[BEACON_COUNT] = {"I AM PICO W"};
static volatile int scan_rssi[BEACON_COUNT];
static uint8_t scan_bssid[6];
static bool stopped;
static mpu9250_t imu;
static char report[REPORT_CAPACITY];
static size_t report_length;
static bool report_truncated;

static void report_printf(const char *format, ...)
{
    char line[256];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (written <= 0) return;

    size_t length = (size_t)written;
    if (length >= sizeof(line)) {
        length = sizeof(line) - 1;
        report_truncated = true;
    }
    size_t remaining = sizeof(report) - 1 - report_length;
    if (length > remaining) {
        length = remaining;
        report_truncated = true;
    }
    memcpy(report + report_length, line, length);
    report_length += length;
    report[report_length] = '\0';

    if (stdio_usb_connected()) printf("%s", line);
}

static void replay_report(void)
{
    if (!stdio_usb_connected()) return;
    printf("\r\n========== STORED WHEEL TEST REPORT ==========\r\n");
    printf("%s", report);
    if (report_truncated) printf("REPORT TRUNCATED: buffer full\r\n");
    printf("========== END OF REPORT ==========\r\n");
    printf("Press GP20 to print this report again. Reset erases it.\r\n");
}

static uint32_t now_ms(void)
{
    return (uint32_t)(time_us_64() / 1000);
}

static bool stop_pressed(void)
{
    if (!gpio_get(STOP_BUTTON)) stopped = true;
    return stopped;
}

static void motor_write(uint pin_a, uint pin_b, int speed, bool forward_uses_a)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
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

static void motors_stop(void)
{
    motor_write(M1_A, M1_B, 0, M1_FORWARD_USES_A);
    motor_write(M2_A, M2_B, 0, M2_FORWARD_USES_A);
}

static void motors_init(void)
{
    const uint pins[] = {M1_A, M1_B, M2_A, M2_B};
    for (uint i = 0; i < 4; ++i) {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pins[i]);
        pwm_set_wrap(slice, 9999);
        pwm_set_clkdiv(slice, 1.25f); // 10 kHz at the normal 125 MHz clock
        pwm_set_gpio_level(pins[i], 0);
        pwm_set_enabled(slice, true);
    }
    motors_stop();
}

static bool pause_checked(uint32_t duration_ms)
{
    uint32_t start = now_ms();
    while ((uint32_t)(now_ms() - start) < duration_ms) {
        if (stop_pressed()) {
            motors_stop();
            return false;
        }
        sleep_ms(5);
    }
    return true;
}

static bool drive_for(int speed, uint32_t duration_ms)
{
    motors_set(speed, speed);
    uint32_t start = now_ms();
    uint64_t previous_us = time_us_64();
    float yaw_change = 0.0f;
    while ((uint32_t)(now_ms() - start) < duration_ms) {
        if (stop_pressed()) {
            motors_stop();
            return false;
        }
        float gyro[3];
        if (!mpu9250_read_gyro_dps(&imu, gyro)) {
            motors_stop();
            report_printf("DRIVE FAILED: IMU read error\n");
            return false;
        }
        uint64_t current_us = time_us_64();
        yaw_change += gyro[2] * (float)(current_us - previous_us) / 1000000.0f;
        previous_us = current_us;
        if (yaw_change > MAX_STRAIGHT_YAW_DEGREES ||
            yaw_change < -MAX_STRAIGHT_YAW_DEGREES) {
            motors_stop();
            report_printf("DRIVE FAILED: turned %d degrees during straight pulse; check M1/M2 direction\n",
                   (int)yaw_change);
            return false;
        }
        sleep_ms(10);
    }
    motors_stop();
    return pause_checked(MOTOR_SETTLE_MS);
}

// Relative heading only: the Z gyro controls a turn, but gives no distance.
static bool turn_degrees(int degrees)
{
    if (degrees == 0) return true;
    int direction = degrees > 0 ? 1 : -1;
    float target = (float)(degrees > 0 ? degrees : -degrees);
    float angle = 0.0f;
    float gyro_sign = 0.0f;
    float peak_rate = 0.0f;
    uint32_t start = now_ms();
    uint64_t previous_us = time_us_64();
    motors_set(-direction * TURN_DUTY_PERCENT,
                direction * TURN_DUTY_PERCENT);

    while (angle < target) {
        if (stop_pressed() || (uint32_t)(now_ms() - start) > TURN_TIMEOUT_MS) {
            motors_stop();
            report_printf("TURN FAILED: %s; net gyro angle %d of %d degrees\n",
                   stopped ? "stop pressed" : "6 s timeout",
                   (int)angle, (int)target);
            return false;
        }
        if (gyro_sign == 0.0f &&
            (uint32_t)(now_ms() - start) > TURN_START_TIMEOUT_MS) {
            motors_stop();
            report_printf("TURN FAILED: no sustained Z-axis rotation within 1 s; check motor directions and IMU mounting\n");
            return false;
        }
        float gyro[3];
        if (!mpu9250_read_gyro_dps(&imu, gyro)) {
            motors_stop();
            report_printf("TURN FAILED: IMU read error\n");
            return false;
        }
        uint64_t current_us = time_us_64();
        if (gyro_sign == 0.0f && (gyro[2] > 10.0f || gyro[2] < -10.0f)) {
            gyro_sign = gyro[2] > 0.0f ? 1.0f : -1.0f;
        }
        float rate = gyro[2] < 0.0f ? -gyro[2] : gyro[2];
        if (rate > peak_rate) peak_rate = rate;
        if (gyro_sign != 0.0f) {
            // Opposite rotation subtracts from progress instead of adding.
            angle += gyro_sign * gyro[2] *
                     (float)(current_us - previous_us) / 1000000.0f;
        }
        previous_us = current_us;
        sleep_ms(10);
    }
    motors_stop();
    report_printf("TURN OK: commanded %d degrees, net gyro measured %d degrees, elapsed %lu ms, peak gyro Z %d dps\n",
                  degrees, (int)angle, (unsigned long)(now_ms() - start),
                  (int)peak_rate);
    return pause_checked(MOTOR_SETTLE_MS);
}

static int wifi_result(void *unused, const cyw43_ev_scan_result_t *result)
{
    (void)unused;
    if (!result || result->rssi >= 0) return 0;
    for (int i = 0; i < BEACON_COUNT; ++i) {
        size_t length = strlen(ssids[i]);
        if (result->ssid_len == length &&
            memcmp(result->ssid, ssids[i], length) == 0 &&
            result->rssi > scan_rssi[i]) {
            scan_rssi[i] = result->rssi;
            memcpy(scan_bssid, result->bssid, sizeof(scan_bssid));
        }
    }
    return 0;
}

static bool scan_once(int results[BEACON_COUNT])
{
    for (int i = 0; i < BEACON_COUNT; ++i) scan_rssi[i] = RSSI_MISSING;
    cyw43_wifi_scan_options_t options = {0};
    if (cyw43_wifi_scan(&cyw43_state, &options, NULL, wifi_result) != 0) {
        report_printf("Wi-Fi scan could not start\n");
        return false;
    }
    uint32_t start = now_ms();
    while (cyw43_wifi_scan_active(&cyw43_state)) {
        if (stop_pressed() || (uint32_t)(now_ms() - start) > SCAN_TIMEOUT_MS) {
            motors_stop();
            report_printf("Wi-Fi scan stopped or timed out\n");
            return false;
        }
        sleep_ms(10);
    }
    for (int i = 0; i < BEACON_COUNT; ++i) results[i] = scan_rssi[i];
    if (results[0] == RSSI_MISSING) {
        report_printf("Wi-Fi scan: AP1 not seen\n");
    } else {
        report_printf("Wi-Fi scan: AP1 %d dBm, BSSID %02x:%02x:%02x:%02x:%02x:%02x\n",
               results[0], scan_bssid[0], scan_bssid[1], scan_bssid[2],
               scan_bssid[3], scan_bssid[4], scan_bssid[5]);
    }
    return !stop_pressed();
}

// Match the tracker filter: discard the lowest and highest quarter of the
// valid scan readings, then average the remaining RSSI values.
static int rssi_robust_average(const int *values, int count)
{
    int sorted[SCANS_PER_SAMPLE];
    for (int i = 0; i < count; ++i) {
        int value = values[i];
        int j = i;
        while (j > 0 && sorted[j - 1] > value) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = value;
    }
    int trim = count / 4;
    int sum = 0;
    for (int i = trim; i < count - trim; ++i) sum += sorted[i];
    int kept = count - 2 * trim;
    return (sum - kept / 2) / kept;
}

static bool sample_beacons(int values[BEACON_COUNT])
{
    int readings[BEACON_COUNT][SCANS_PER_SAMPLE];
    int counts[BEACON_COUNT] = {0};
    for (int round = 0; round < SCANS_PER_SAMPLE; ++round) {
        int scan[BEACON_COUNT];
        if (!scan_once(scan)) return false;
        for (int i = 0; i < BEACON_COUNT; ++i) {
            if (scan[i] != RSSI_MISSING)
                readings[i][counts[i]++] = scan[i];
        }
    }
    for (int i = 0; i < BEACON_COUNT; ++i) {
        values[i] = counts[i] >= MIN_VALID_SCANS
                        ? rssi_robust_average(readings[i], counts[i])
                        : RSSI_MISSING;
        if (values[i] == RSSI_MISSING) {
            report_printf("AP%d filter: only %d/%d valid scans; no RSSI score\n",
                          i + 1, counts[i], SCANS_PER_SAMPLE);
        } else {
            report_printf("AP%d filter: %d/%d valid scans, filtered RSSI %d dBm\n",
                          i + 1, counts[i], SCANS_PER_SAMPLE, values[i]);
        }
    }
    return true;
}

static bool calibrate_gyro(void)
{
    report_printf("Keep robot still: calibrating gyro for 2 seconds\n");
    float sum = 0.0f;
    for (int i = 0; i < 200; ++i) {
        float gyro[3];
        if (!mpu9250_read_gyro_dps(&imu, gyro)) return false;
        sum += gyro[2];
        if (!pause_checked(10)) return false;
    }
    imu.gyro_bias_dps[2] = sum / 200.0f;
    report_printf("Gyro Z bias calibrated\n");
    return true;
}

static bool run_test(void)
{
    if (!calibrate_gyro()) return false;
    int baseline[BEACON_COUNT];
    report_printf("Sampling AP1 only (%s) at start\n", ssids[0]);
    if (!sample_beacons(baseline)) return false;
    if (baseline[0] == RSSI_MISSING) {
        report_printf("Baseline AP1: not seen\n");
        report_printf("FAILED: AP1 was not found in at least three of five scans; no movement\n");
        return false;
    }
    const int target = 0;
    report_printf("Baseline AP1=%d dBm; target locked to AP1\n", baseline[0]);

    int score[HEADING_COUNT];
    for (int heading = 0; heading < HEADING_COUNT; ++heading) {
        report_printf("Heading %d degrees: forward probe\n", heading * 90);
        if (!drive_for(DRIVE_DUTY_PERCENT, PROBE_MOVE_MS)) return false;
        int at_probe[BEACON_COUNT];
        if (!sample_beacons(at_probe)) return false;
        score[heading] = at_probe[target];
        report_printf("Heading %d target RSSI=%d dBm; returning\n",
               heading * 90, score[heading]);
        if (!drive_for(-DRIVE_DUTY_PERCENT, PROBE_MOVE_MS)) return false;
        if (!turn_degrees(90)) return false;
    }
    motors_stop();
    int best = -1, second = -1;
    for (int i = 0; i < HEADING_COUNT; ++i) {
        if (score[i] == RSSI_MISSING) continue;
        if (best < 0 || score[i] > score[best]) {
            second = best; best = i;
        } else if (second < 0 || score[i] > score[second]) {
            second = i;
        }
    }
    if (best < 0 || second < 0 ||
        score[best] < baseline[target] + MIN_RSSI_GAIN_DB ||
        score[best] - score[second] < MIN_BEST_MARGIN_DB) {
        report_printf("UNCERTAIN: no clear RSSI direction; motors remain stopped\n");
        return false;
    }
    report_printf("Selected heading %d degrees: %d dBm (next %d dBm)\n",
           best * 90, score[best], score[second]);
    int final_turn = best == 3 ? -90 : best * 90;
    if (!turn_degrees(final_turn)) return false;
    report_printf("Final short forward movement\n");
    if (!drive_for(DRIVE_DUTY_PERCENT, FINAL_MOVE_MS)) return false;
    motors_stop();
    report_printf("TEST COMPLETE: motors stopped; target AP%d\n", target + 1);
    return true;
}

int main(void)
{
    stdio_init_all();
    gpio_init(START_BUTTON);
    gpio_set_dir(START_BUTTON, GPIO_IN);
    gpio_pull_up(START_BUTTON);
    gpio_init(STOP_BUTTON);
    gpio_set_dir(STOP_BUTTON, GPIO_IN);
    gpio_pull_up(STOP_BUTTON);
    motors_init();
    sleep_ms(2000);
    report_printf("\nWheel RSSI test | GP20 start | GP21 stop\n");
    report_printf("M1=left, M2=right | IMU GP6/GP7 | movement is timed, not measured\n");

    bool ready = mpu9250_init(&imu, i2c1, IMU_SDA, IMU_SCL, 400000);
    if (!ready) {
        report_printf("FAILED: MPU-9250/6500 not found at I2C address 0x68\n");
    } else {
        report_printf("IMU WHO_AM_I=0x%02x\n", imu.who_am_i);
        if (cyw43_arch_init()) {
            report_printf("FAILED: Wi-Fi initialisation\n");
            ready = false;
        } else {
            cyw43_arch_enable_sta_mode();
        }
    }

    if (ready) {
        report_printf("Waiting for GP20. GP21 stops and latches until reset.\n");
        while (gpio_get(START_BUTTON) && !stop_pressed()) sleep_ms(10);
        if (stopped) {
            report_printf("STOP pressed before start\n");
        } else {
            while (!gpio_get(START_BUTTON) && !stop_pressed()) sleep_ms(10);
            if (!stopped) (void)run_test();
        }
    }
    motors_stop();
    if (stopped) report_printf("Test stopped by GP21\n");
    report_printf("Test ended. Connect Pico USB to read stored report.\n");

    bool was_connected = false;
    bool last_start = true;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !was_connected) {
            sleep_ms(500);
            replay_report();
        }
        was_connected = stdio_usb_connected();
        bool start = gpio_get(START_BUTTON);
        if (was_connected && !start && last_start) replay_report();
        last_start = start;
        sleep_ms(20);
    }
}
