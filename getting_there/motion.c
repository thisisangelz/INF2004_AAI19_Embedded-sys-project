#include "motion.h"
#include "faults.h"
#include "packed_bits.h"
#include "run_report.h"
#include "hardware/pwm.h"
#include "imu_mpu9250.h"
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
#define TURN_DUTY_PERCENT 25
#define TURN_SLOW_DUTY_PERCENT 20
#define TURN_SLOW_ZONE_DEGREES 25.0f
#define TURN_TOLERANCE_DEGREES 2.0f
#define TURN_COAST_SECONDS 0.03f
#define TURN_MAX_CORRECTIONS 2
#define TURN_CORRECTION_MAX_MS 200
#define GYRO_DIRECTION_CHECK_DEGREES 15.0f
#define GYRO_RANGE_DPS 1000
#define GYRO_SATURATION_DPS 980.0f
#define MOTION_SAMPLE_MS 5
#define STEERING_YAW_GAIN 0.65f
#define STEERING_RATE_GAIN 0.10f
#define MAX_STEERING_ADJUSTMENT 8


#define MOTOR_SETTLE_MS 300

#define TURN_TIMEOUT_MS 6000
#define TURN_START_TIMEOUT_MS 1000
#define MAX_STRAIGHT_YAW_DEGREES 25.0f

static mpu9250_t imu;
// Sensor coordinates are kept across movement and motor-off settling.
static float gyro_heading_degrees;
// Gyro sign for motors_set(-duty, +duty), learnt before the first probe.
// Three states (unknown, +1, -1) fit in two bits. The stop latch is the
// FAULT_STOP_LATCHED bit, because GP21 can be polled from any context.
enum { TURN_SIGN_UNKNOWN, TURN_SIGN_POSITIVE, TURN_SIGN_NEGATIVE, TURN_SIGN_MAX = TURN_SIGN_NEGATIVE };
_Static_assert(2 == BITS_NEEDED(TURN_SIGN_MAX), "turn sign field is not minimal");
static volatile struct {
    uint8_t turn_sign : 2;
    uint8_t stop_armed : 1;
} motion_state;

static float turn_sign(void)
{
    return motion_state.turn_sign == TURN_SIGN_POSITIVE ? 1.0f :
           motion_state.turn_sign == TURN_SIGN_NEGATIVE ? -1.0f : 0.0f;
}
static void set_turn_sign(float sign)
{
    motion_state.turn_sign = sign > 0.0f ? TURN_SIGN_POSITIVE :
                             sign < 0.0f ? TURN_SIGN_NEGATIVE : TURN_SIGN_UNKNOWN;
}
static uint32_t now_ms(void)
{
    return (uint32_t)(time_us_64() / 1000);
}

static bool stop_pressed(void)
{
    if (motion_state.stop_armed && !gpio_get(STOP_BUTTON)) fault_raise(FAULT_STOP_LATCHED);
    return fault_test(FAULT_STOP_LATCHED);
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
    if (!motion_state.stop_armed || stop_pressed()) left = right = 0;
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
        pwm_set_gpio_level(pins[i], 0);
    }
    // Each motor's two pins share one slice. Configure each slice once,
    // deriving it from a constant pin (also avoids SDK/GCC range warnings).
    _Static_assert(M1_A / 2 == M1_B / 2 && M2_A / 2 == M2_B / 2,
                   "motor pins must share their PWM slice");
    const uint slices[] = {pwm_gpio_to_slice_num(M1_A), pwm_gpio_to_slice_num(M2_A)};
    for (uint i = 0; i < 2; ++i) {
        uint slice = slices[i];
        pwm_set_wrap(slice, 9999);
        pwm_set_clkdiv(slice, 1.25f); // 10 kHz at the normal 125 MHz clock
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

static float normalise_degrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int steering_adjustment(int speed, float yaw_error, float yaw_rate,
                               float turn_gyro_sign)
{
    float correction = turn_gyro_sign *
                       (STEERING_YAW_GAIN * yaw_error +
                        STEERING_RATE_GAIN * yaw_rate);
    int adjustment = (int)(correction + (correction >= 0.0f ? 0.5f : -0.5f));
    int limit = (speed < 0 ? -speed : speed) - 1;
    if (limit < 0) limit = 0;
    if (limit > MAX_STEERING_ADJUSTMENT) limit = MAX_STEERING_ADJUSTMENT;
    if (adjustment > limit) adjustment = limit;
    if (adjustment < -limit) adjustment = -limit;
    return adjustment;
}

static bool read_motion_gyro(float gyro[3], uint64_t *previous_us,
                             const char *motion)
{
    if (stop_pressed()) {
        motors_stop();
        return false;
    }
    if (!mpu9250_read_gyro_dps(&imu, gyro)) {
        motors_stop();
        fault_raise(FAULT_IMU);
        report_printf("%s FAILED: IMU read error\n", motion);
        return false;
    }
    if (gyro[2] >= GYRO_SATURATION_DPS ||
        gyro[2] <= -GYRO_SATURATION_DPS) {
        motors_stop();
        fault_raise(FAULT_GYRO_SATURATED);
        report_printf("%s FAILED: gyro Z near %d dps range limit; angle cannot be trusted\n",
                      motion, GYRO_RANGE_DPS);
        return false;
    }
    uint64_t current_us = time_us_64();
    gyro_heading_degrees += gyro[2] *
                            (float)(current_us - *previous_us) / 1000000.0f;
    *previous_us = current_us;
    return true;
}

static bool settle_motion(const char *motion, uint64_t *previous_us)
{
    uint32_t start = now_ms();
    while ((uint32_t)(now_ms() - start) < MOTOR_SETTLE_MS) {
        float gyro[3];
        if (!read_motion_gyro(gyro, previous_us, motion)) return false;
        sleep_ms(MOTION_SAMPLE_MS);
    }
    // Include the final interval before the next movement or scan.
    float gyro[3];
    return read_motion_gyro(gyro, previous_us, motion);
}

static bool drive_for(int speed, uint32_t duration_ms, float heading_degrees)
{
    if (turn_sign() == 0.0f) {
        motors_stop();
        report_printf("DRIVE FAILED: gyro direction has not been learnt\n");
        return false;
    }
    const char *direction = speed >= 0 ? "forwards" : "backwards";
    report_printf("DRIVE: %s, base duty %d%%, duration %lu ms, heading %d degrees, gyro correction on\n",
                  direction, speed < 0 ? -speed : speed,
                  (unsigned long)duration_ms, (int)heading_degrees);
    float start_heading = gyro_heading_degrees;
    float target_heading = heading_degrees * turn_sign();
    int peak_adjustment = 0;
    uint32_t start = now_ms();
    uint64_t previous_us = time_us_64();
    motors_set(speed, speed);
    while ((uint32_t)(now_ms() - start) < duration_ms) {
        float gyro[3];
        if (!read_motion_gyro(gyro, &previous_us, "DRIVE")) return false;
        float yaw_error = normalise_degrees(gyro_heading_degrees - target_heading);
        if (yaw_error > MAX_STRAIGHT_YAW_DEGREES ||
            yaw_error < -MAX_STRAIGHT_YAW_DEGREES) {
            motors_stop();
            fault_raise(FAULT_HEADING);
            report_printf("DRIVE FAILED: heading error %+d degrees; check M1/M2 direction\n",
                          (int)(turn_sign() * yaw_error));
            return false;
        }
        int adjustment = steering_adjustment(speed, yaw_error, gyro[2],
                                             turn_sign());
        int magnitude = adjustment < 0 ? -adjustment : adjustment;
        if (magnitude > peak_adjustment) peak_adjustment = magnitude;
        // A signed wheel differential counters the same yaw in either travel direction.
        motors_set(speed + adjustment, speed - adjustment);
        sleep_ms(MOTION_SAMPLE_MS);
    }
    motors_stop();
    uint32_t elapsed = now_ms() - start;
    if (!settle_motion("DRIVE", &previous_us)) return false;
    float yaw_error = normalise_degrees(gyro_heading_degrees - target_heading);
    if (yaw_error > MAX_STRAIGHT_YAW_DEGREES ||
        yaw_error < -MAX_STRAIGHT_YAW_DEGREES) {
        fault_raise(FAULT_HEADING);
        report_printf("DRIVE FAILED: excessive rotation while settling\n");
        return false;
    }
    report_printf("DRIVE OK: %s, powered %lu ms, settled gyro Z rotation %+d degrees, heading error %+d degrees, peak correction %d duty points\n",
                  direction, (unsigned long)elapsed,
                  (int)(gyro_heading_degrees - start_heading),
                  (int)(turn_sign() * yaw_error), peak_adjustment);
    return true;
}

// Relative heading only: the Z gyro controls a turn, but gives no distance.
static bool turn_degrees(float degrees)
{
    if (stop_pressed()) { motors_stop(); return false; }
    if (degrees == 0.0f) return true;
    float start_heading = gyro_heading_degrees;
    float peak_rate = 0.0f;
    uint32_t start = now_ms();
    uint64_t previous_us = time_us_64();

    for (int attempt = 0; attempt <= TURN_MAX_CORRECTIONS; ++attempt) {
        float progress = turn_sign() *
                         (gyro_heading_degrees - start_heading);
        float error = degrees - progress;
        if (turn_sign() != 0.0f &&
            error <= TURN_TOLERANCE_DEGREES &&
            error >= -TURN_TOLERANCE_DEGREES) {
            return true;
        }
        int direction = error > 0.0f ? 1 : -1;
        float segment_heading = gyro_heading_degrees;
        uint32_t segment_start = now_ms();
        float sign_candidate = 0.0f;
        int sign_samples = 0;
        int duty = direction * error <= TURN_SLOW_ZONE_DEGREES
                       ? TURN_SLOW_DUTY_PERCENT : TURN_DUTY_PERCENT;
        motors_set(-direction * duty, direction * duty);

        while (true) {
            if ((uint32_t)(now_ms() - start) >= TURN_TIMEOUT_MS) {
                motors_stop();
                fault_raise(FAULT_TURN);
                report_printf("TURN FAILED: 6 s timeout\n");
                return false;
            }
            float gyro[3];
            if (!read_motion_gyro(gyro, &previous_us, "TURN")) return false;
            float rate = gyro[2] < 0.0f ? -gyro[2] : gyro[2];
            if (rate > peak_rate) peak_rate = rate;
            if (turn_sign() == 0.0f) {
                float candidate = gyro[2] > 5.0f ? 1.0f :
                                  gyro[2] < -5.0f ? -1.0f : 0.0f;
                if (candidate == 0.0f) {
                    sign_samples = 0;
                } else if (candidate == sign_candidate) {
                    ++sign_samples;
                } else {
                    sign_candidate = candidate;
                    sign_samples = 1;
                }
                if (sign_samples >= 3) {
                    set_turn_sign(direction * sign_candidate);
                }
            }
            progress = turn_sign() *
                       (gyro_heading_degrees - start_heading);
            error = degrees - progress;
            float remaining = direction * error;
            float projected_rate = direction * turn_sign() * gyro[2];
            if (projected_rate < 0.0f) projected_rate = 0.0f;
            float coast_angle = projected_rate * TURN_COAST_SECONDS;
            if (turn_sign() != 0.0f &&
                (remaining <= TURN_TOLERANCE_DEGREES ||
                 (attempt == 0 && remaining <= coast_angle))) {
                break;
            }
            float segment_progress = direction * turn_sign() *
                                     (gyro_heading_degrees - segment_heading);
            if (segment_progress < -3.0f) {
                motors_stop();
                fault_raise(FAULT_HEADING);
                report_printf("TURN FAILED: rotation opposite to learnt motor direction\n");
                return false;
            }
            if ((uint32_t)(now_ms() - segment_start) >= TURN_START_TIMEOUT_MS &&
                segment_progress < 1.0f) {
                motors_stop();
                fault_raise(FAULT_TURN);
                report_printf("TURN FAILED: no sustained Z rotation within 1 s at %d%% duty\n", duty);
                return false;
            }
            if (attempt > 0 &&
                (uint32_t)(now_ms() - segment_start) >= TURN_CORRECTION_MAX_MS) {
                break;
            }
            duty = remaining <= TURN_SLOW_ZONE_DEGREES
                       ? TURN_SLOW_DUTY_PERCENT : TURN_DUTY_PERCENT;
            motors_set(-direction * duty, direction * duty);
            sleep_ms(MOTION_SAMPLE_MS);
        }
        motors_stop();
        if (!settle_motion("TURN", &previous_us)) return false;
        progress = turn_sign() *
                   (gyro_heading_degrees - start_heading);
        error = degrees - progress;
        if (error <= TURN_TOLERANCE_DEGREES && error >= -TURN_TOLERANCE_DEGREES) {
            report_printf("TURN OK: commanded %d degrees, settled gyro rotation %d degrees, error %+d degrees, elapsed %lu ms, peak gyro Z %d dps, corrections %d\n",
                          (int)degrees, (int)progress, (int)error,
                          (unsigned long)(now_ms() - start), (int)peak_rate, attempt);
            return true;
        }
        report_printf("TURN: settled error %+d degrees; %s\n", (int)error,
                      attempt < TURN_MAX_CORRECTIONS ? "correcting" : "correction limit reached");
    }
    motors_stop();
    fault_raise(FAULT_TURN);
    report_printf("TURN FAILED: settled heading outside %d degree tolerance\n",
                  (int)TURN_TOLERANCE_DEGREES);
    return false;
}

static bool turn_to_heading(float heading_degrees)
{
    if (turn_sign() == 0.0f) return false;
    float error = normalise_degrees(heading_degrees -
                                    turn_sign() * gyro_heading_degrees);
    return turn_degrees(error);
}

static bool calibrate_gyro(void)
{
    report_printf("Keep robot still: calibrating gyro for 2 seconds\n");
    float sum = 0.0f;
    for (int i = 0; i < 200; ++i) {
        float gyro[3];
        if (!mpu9250_read_gyro_dps(&imu, gyro)) {
            fault_raise(FAULT_IMU);
            return false;
        }
        sum += gyro[2];
        if (!pause_checked(10)) return false;
    }
    imu.gyro_bias_dps[2] = sum / 200.0f;
    report_printf("Gyro Z bias calibrated\n");
    return true;
}


bool motion_init(void)
{
    gpio_init(START_BUTTON);
    gpio_set_dir(START_BUTTON, GPIO_IN);
    gpio_pull_up(START_BUTTON);
    gpio_init(STOP_BUTTON);
    gpio_set_dir(STOP_BUTTON, GPIO_IN);
    gpio_pull_up(STOP_BUTTON);
    motors_init();
    if (!mpu9250_init(&imu, i2c1, IMU_SDA, IMU_SCL, 400000) ||
        !mpu9250_set_gyro_range(&imu, GYRO_RANGE_DPS)) {
        fault_raise(FAULT_IMU);
        report_printf("FAILED: IMU initialisation or gyro range configuration\n");
        return false;
    }
    report_printf("IMU WHO_AM_I=0x%02x | gyro +/- %d dps | turns %d%% (slow %d%%), drive %d%%\n",
                  imu.who_am_i, GYRO_RANGE_DPS, TURN_DUTY_PERCENT, TURN_SLOW_DUTY_PERCENT,
                  DRIVE_DUTY_PERCENT);
    return true;
}

bool motion_calibrate(void)
{
    gyro_heading_degrees = 0.0f;
    set_turn_sign(0.0f);
    return calibrate_gyro();
}

bool motion_learn_direction(void)
{
    report_printf("Learning gyro direction: 15 degree turn, then return to original heading\n");
    bool ok = turn_degrees(GYRO_DIRECTION_CHECK_DEGREES) && turn_to_heading(0.0f);
    // USB output can block: report only after the motors are stopped.
    motors_stop();
    if (ok) report_printf("Gyro direction learnt: positive motor turn gives gyro Z sign %+d\n",
                          (int)turn_sign());
    return ok;
}

bool motion_drive(bool forwards, uint32_t duration_ms, float heading_degrees)
{
    return drive_for(forwards ? DRIVE_DUTY_PERCENT : -DRIVE_DUTY_PERCENT,
                     duration_ms, heading_degrees);
}

bool motion_turn_to_heading(float heading_degrees)
{
    return turn_to_heading(heading_degrees);
}

float motion_heading(void)
{
    return turn_sign() * gyro_heading_degrees;
}

bool motion_pause_ms(uint32_t duration_ms) { return pause_checked(duration_ms); }
bool motion_stop_requested(void) { return stop_pressed(); }
bool motion_start_pressed(void) { return !gpio_get(START_BUTTON); }
bool motion_replay_pressed(void) { return !gpio_get(STOP_BUTTON); }
void motion_arm_stop(void)
{
    uint32_t irq = save_and_disable_interrupts();
    motion_state.stop_armed = true;
    restore_interrupts(irq);
    (void)stop_pressed();
}
void motion_stop(void) { motors_stop(); }
