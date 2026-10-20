#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../motion.c"
#include "../../pico_rssi_tracker/imu_mpu9250.c"

static uint64_t clock_us, press_at_us;
static bool gp21, write_failure, read_failure;
static uint16_t levels[30];
static uint8_t registers[256], selected_register;
static i2c_inst_t bus;
i2c_inst_t *i2c1 = &bus;
static bool powered(void)
{
    return levels[M1_A] || levels[M1_B] || levels[M2_A] || levels[M2_B];
}
void report_printf(const char *format, ...) { (void)format; assert(!powered()); }
uint64_t time_us_64(void) { return clock_us; }
void sleep_ms(uint32_t ms)
{
    clock_us += ms * 1000;
    if (press_at_us && clock_us >= press_at_us) gp21 = true;
}
void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin, bool out) { (void)pin; (void)out; }
void gpio_pull_up(uint pin) { (void)pin; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
bool gpio_get(uint pin) { return pin == STOP_BUTTON ? !gp21 : true; }
uint pwm_gpio_to_slice_num(uint pin) { return (pin >> 1) & 7u; }
void pwm_set_gpio_level(uint pin, uint16_t level) { assert(pin < 30); levels[pin] = level; }
void pwm_set_wrap(uint slice, uint16_t wrap) { assert(slice < 8); (void)wrap; }
void pwm_set_clkdiv(uint slice, float divider) { assert(slice < 8); (void)divider; }
void pwm_set_enabled(uint slice, bool enabled) { assert(slice < 8); (void)enabled; }
uint i2c_init(i2c_inst_t *i2c, uint baud) { (void)i2c; return baud; }
int i2c_write_timeout_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src,
                         size_t len, bool nostop, uint timeout)
{
    (void)i2c; (void)addr; (void)nostop;
    assert(timeout == 5000);
    if (write_failure) { clock_us += timeout; return -1; }
    selected_register = src[0];
    if (len == 2) registers[src[0]] = src[1];
    return (int)len;
}
int i2c_read_timeout_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst,
                        size_t len, bool nostop, uint timeout)
{
    (void)i2c; (void)addr; (void)nostop;
    assert(timeout == 5000);
    if (read_failure) { clock_us += timeout; return -1; }
    memcpy(dst, registers + selected_register, len);
    return (int)len;
}
static void reset(void)
{
    for (int f = 0; f < FAULT_COUNT; ++f) fault_clear((fault_t)f);
    motion_state.turn_sign = motion_state.stop_armed = 0;
    gp21 = write_failure = read_failure = false;
    clock_us = press_at_us = 0;
    gyro_heading_degrees = 0;
    memset(registers, 0, sizeof(registers));
    registers[0x75] = 0x71;
    assert(motion_init());
}
int main(void)
{
    reset();
    gp21 = true;
    assert(motion_replay_pressed() && !motion_stop_requested());
    assert(!fault_test(FAULT_STOP_LATCHED));
    motors_set(35, 35); assert(!powered()); // disarmed means physically no PWM
    gp21 = false;
    motion_arm_stop();
    motors_set(35, 35); assert(powered());
    gp21 = true;
    motors_set(35, 35); assert(!powered() && motion_stop_requested());
    gp21 = false;
    assert(motion_stop_requested()); // release never unlatches STOP

    reset(); gp21 = true; motion_arm_stop();
    assert(motion_stop_requested()); // holding STOP at arm time is safe
    reset(); motion_arm_stop(); set_turn_sign(1);
    press_at_us = 10000;
    assert(!motion_drive(true, 750, 0));
    assert(!powered() && clock_us <= 15000 && fault_test(FAULT_STOP_LATCHED));

    reset(); motion_arm_stop(); set_turn_sign(1); write_failure = true;
    assert(!motion_drive(true, 750, 0));
    assert(!powered() && fault_test(FAULT_IMU) && clock_us == 5000);
    reset(); motion_arm_stop(); set_turn_sign(1); read_failure = true;
    assert(!motion_drive(true, 750, 0));
    assert(!powered() && fault_test(FAULT_IMU) && clock_us == 5000);
    reset(); motion_arm_stop(); set_turn_sign(1);
    assert(motion_drive(true, 750, 0) && !powered());
    assert(motion_drive(false, 750, 0) && !powered());
    puts("Motion: GP21 replay vs armed STOP, PWM interlock, latched stop, bounded I2C failures and stationary logging passed");
}
