#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

static uint64_t simulated_us;
static uint32_t stop_event_ms = UINT32_MAX;
static uint16_t pwm_levels[32];
static uint16_t raised_faults;
static bool imu_read_ok = true;

#include "getting_there/motion.c"

static i2c_inst_t mock_i2c;
i2c_inst_t *i2c0 = &mock_i2c;
i2c_inst_t *i2c1 = &mock_i2c;
uint64_t time_us_64(void) { return simulated_us; }
void sleep_ms(uint32_t milliseconds) { simulated_us += (uint64_t)milliseconds * 1000; }
void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin, bool output) { (void)pin; (void)output; }
void gpio_pull_up(uint pin) { (void)pin; }
bool gpio_get(uint pin)
{
    return pin == STOP_BUTTON ? (simulated_us / 1000 < stop_event_ms) : true;
}
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
void pwm_set_gpio_level(uint pin, uint16_t level) { assert(pin < 32); pwm_levels[pin] = level; }
uint pwm_gpio_to_slice_num(uint pin) { return pin / 2; }
void pwm_set_wrap(uint slice, uint16_t wrap) { (void)slice; (void)wrap; }
void pwm_set_clkdiv(uint slice, float divider) { (void)slice; (void)divider; }
void pwm_set_enabled(uint slice, bool enabled) { (void)slice; (void)enabled; }
uint32_t clock_get_hz(uint clock) { (void)clock; return 125000000; }
uint i2c_init(i2c_inst_t *i2c, uint baudrate) { (void)i2c; return baudrate; }
int i2c_write_blocking(i2c_inst_t *i2c, uint8_t address, const uint8_t *src,
                       size_t length, bool nostop)
{ (void)i2c; (void)address; (void)src; (void)nostop; return (int)length; }
int i2c_read_blocking(i2c_inst_t *i2c, uint8_t address, uint8_t *dst,
                      size_t length, bool nostop)
{ (void)i2c; (void)address; (void)dst; (void)nostop; return (int)length; }
bool mpu9250_init(mpu9250_t *value, i2c_inst_t *bus, uint sda, uint scl, uint32_t hz)
{ (void)sda; (void)scl; (void)hz; value->i2c = bus; value->who_am_i = 0x71; return true; }
bool mpu9250_set_gyro_range(mpu9250_t *value, uint16_t range)
{ value->gyro_range_dps = range; value->gyro_lsb_per_dps = 32.8f; return true; }
bool mpu9250_read_gyro_dps(const mpu9250_t *value, float output[3])
{ (void)value; output[0] = output[1] = output[2] = 0.0f; return imu_read_ok; }
void fault_raise(fault_t fault) { raised_faults |= (uint16_t)(1u << fault); }
void fault_clear(fault_t fault) { raised_faults &= (uint16_t)~(1u << fault); }
bool fault_test(fault_t fault) { return ((raised_faults >> fault) & 1u) != 0; }
uint16_t fault_mask(void) { return raised_faults; }
const char *fault_name(fault_t fault) { (void)fault; return "test"; }
void report_printf(const char *format, ...) { (void)format; }

static bool all_motor_pwm_off(void)
{
    return pwm_levels[M1_A] == 0 && pwm_levels[M1_B] == 0 &&
           pwm_levels[M2_A] == 0 && pwm_levels[M2_B] == 0;
}

int main(void)
{
    stop_event_ms = UINT32_MAX;
    motors_set(35, 35);
    assert(!all_motor_pwm_off());
    stop_event_ms = 7;
    assert(!pause_checked(100));
    assert(all_motor_pwm_off());
    assert(simulated_us / 1000 - stop_event_ms <= 20);
    assert(fault_test(FAULT_STOP_LATCHED));

    raised_faults = 0;
    stop_event_ms = UINT32_MAX;
    motors_set(35, 35);
    imu_read_ok = false;
    uint64_t previous = simulated_us;
    float values[3];
    assert(!read_motion_gyro(values, &previous, "TEST"));
    assert(all_motor_pwm_off());
    assert(fault_test(FAULT_IMU));
    puts("STOP polling model and IMU-fault PWM shutdown: passed [HOST; hardware unverified]");
    return 0;
}

