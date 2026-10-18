#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/pwm.h"
#include "imu_mpu9250.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#define START_BUTTON 20
#define STOP_BUTTON 21
#define M1_A 8
#define M1_B 9
#define M2_A 10
#define M2_B 11
#define TURN_DUTY 35
#define TURN_TIMEOUT_MS 6000
#define TARGET_DEGREES 90.0f

static mpu9250_t imu;
static char report[1024];
static size_t report_length;
static bool stopped;

static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static void log_line(const char *message)
{
    size_t size = strlen(message);
    if (size < sizeof(report) - report_length) {
        memcpy(report + report_length, message, size + 1);
        report_length += size;
    }
    if (stdio_usb_connected()) printf("%s", message);
}

static void replay_report(void)
{
    if (!stdio_usb_connected()) return;
    printf("\r\n========== STORED TURN CHECK ==========\r\n%s", report);
    printf("========== END OF REPORT ==========\r\n");
}

static bool stop_pressed(void)
{
    if (!gpio_get(STOP_BUTTON)) stopped = true;
    return stopped;
}

static void motor_pin(uint pin, bool on)
{
    pwm_set_gpio_level(pin, on ? TURN_DUTY * 100 : 0);
}

static void motors_stop(void)
{
    motor_pin(M1_A, false);
    motor_pin(M1_B, false);
    motor_pin(M2_A, false);
    motor_pin(M2_B, false);
}

static void motors_turn(void)
{
    // Same motor polarity and turn direction as wheel_rssi_test.
    motor_pin(M1_A, false);
    motor_pin(M1_B, true);
    motor_pin(M2_A, false);
    motor_pin(M2_B, true);
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
    motors_stop();
}

static bool calibrate_gyro(void)
{
    float sum = 0.0f;
    for (int i = 0; i < 200; ++i) {
        if (stop_pressed()) return false;
        float gyro[3];
        if (!mpu9250_read_gyro_dps(&imu, gyro)) return false;
        sum += gyro[2];
        sleep_ms(10);
    }
    imu.gyro_bias_dps[2] = sum / 200.0f;
    char line[80];
    snprintf(line, sizeof(line), "Gyro Z bias %d centi-dps\n",
             (int)(imu.gyro_bias_dps[2] * 100.0f));
    log_line(line);
    return true;
}

static void check_turn(void)
{
    log_line("Starting one gyro-controlled 90 degree turn\n");
    uint32_t start_ms = now_ms();
    uint64_t last_us = time_us_64();
    float angle = 0.0f;
    float sign = 0.0f;
    float peak_rate = 0.0f;
    float peak_x = 0.0f;
    float peak_y = 0.0f;
    motors_turn();
    while (angle < TARGET_DEGREES) {
        if (stop_pressed()) { log_line("Stopped by GP21\n"); break; }
        if ((uint32_t)(now_ms() - start_ms) >= TURN_TIMEOUT_MS) {
            log_line("Turn timed out after 6 seconds\n");
            break;
        }
        if (sign == 0.0f && (uint32_t)(now_ms() - start_ms) >= 1000) {
            log_line("No sustained Z rotation in 1 second; motors stopped\n");
            break;
        }
        float gyro[3];
        if (!mpu9250_read_gyro_dps(&imu, gyro)) {
            log_line("IMU read failed\n");
            break;
        }
        uint64_t current_us = time_us_64();
        if (sign == 0.0f && (gyro[2] > 10.0f || gyro[2] < -10.0f))
            sign = gyro[2] > 0.0f ? 1.0f : -1.0f;
        if (sign != 0.0f)
            angle += sign * gyro[2] * (float)(current_us - last_us) / 1000000.0f;
        float rate = gyro[2] < 0.0f ? -gyro[2] : gyro[2];
        if (rate > peak_rate) peak_rate = rate;
        float rate_x = gyro[0] < 0.0f ? -gyro[0] : gyro[0];
        float rate_y = gyro[1] < 0.0f ? -gyro[1] : gyro[1];
        if (rate_x > peak_x) peak_x = rate_x;
        if (rate_y > peak_y) peak_y = rate_y;
        last_us = current_us;
        sleep_ms(10);
    }
    motors_stop();
    char line[128];
    snprintf(line, sizeof(line), "Result: integrated gyro %d degrees, elapsed %lu ms, peak X/Y/Z %d/%d/%d dps\n",
             (int)angle, (unsigned long)(now_ms() - start_ms),
             (int)peak_x, (int)peak_y, (int)peak_rate);
    log_line(line);
    log_line("Compare the robot's final physical heading with the 90 degree floor mark.\n");
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
    log_line("Single-turn check | GP20 start | GP21 stop | M1 left, M2 right\n");
    if (!mpu9250_init(&imu, i2c1, 6, 7, 400000)) {
        log_line("IMU not found at 0x68; motors remain stopped\n");
    } else {
        char line[80];
        snprintf(line, sizeof(line), "IMU WHO_AM_I=0x%02x\n", imu.who_am_i);
        log_line(line);
        log_line("Press GP20, then keep the robot still for 2 seconds\n");
        while (gpio_get(START_BUTTON) && !stop_pressed()) sleep_ms(10);
        while (!gpio_get(START_BUTTON) && !stop_pressed()) sleep_ms(10);
        if (!stopped) {
            if (calibrate_gyro()) check_turn();
            else log_line("Gyro calibration failed or GP21 was pressed\n");
        }
    }
    motors_stop();
    log_line("Finished. Keep battery on and connect Pico USB to read report.\n");
    bool was_connected = false;
    bool last_start = true;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !was_connected) {
            sleep_ms(500);
            replay_report();
        }
        was_connected = connected;
        bool start = gpio_get(START_BUTTON);
        if (connected && !start && last_start) replay_report();
        last_start = start;
        sleep_ms(20);
    }
}
