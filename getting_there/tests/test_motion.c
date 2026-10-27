#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "../motion.c"
#include "../../pico_rssi_tracker/imu_mpu9250.c"

static uint64_t clock_us, press_at_us;
static bool gp21, write_failure, read_failure;
static bool gyro_model, gyro_stalled, change_range_on_reset;
static unsigned read_failures_remaining, write_failures_remaining, controller_resets;
static bool short_gyro_read;
static uint32_t recovery_log_delay_us;
static char motion_log[8192];
static size_t motion_log_length;
static int gyro_sensor_sign;
static uint16_t levels[30];
static uint8_t registers[256], selected_register;
static i2c_inst_t bus;
i2c_inst_t *i2c1 = &bus;
static bool powered(void)
{
    return levels[M1_A] || levels[M1_B] || levels[M2_A] || levels[M2_B];
}
static void capture_log(const char *format, va_list args)
{
    assert(!powered());
    int n=vsnprintf(motion_log+motion_log_length,sizeof(motion_log)-motion_log_length,format,args);
    if (n>0 && (size_t)n<sizeof(motion_log)-motion_log_length) motion_log_length+=(size_t)n;
    if (!strncmp(format,"IMU RECOVERED:",14)) clock_us+=recovery_log_delay_us;
}
void report_printf(const char *format, ...)
{ va_list args; va_start(args,format); capture_log(format,args); va_end(args); }
void report_defer_printf(const char *format, ...)
{ va_list args; va_start(args,format); capture_log(format,args); va_end(args); }
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
bool gpio_get(uint pin)
{
    if (press_at_us && clock_us>=press_at_us) gp21=true;
    return pin==STOP_BUTTON ? !gp21 : true;
}
uint pwm_gpio_to_slice_num(uint pin) { return (pin >> 1) & 7u; }
void pwm_set_gpio_level(uint pin, uint16_t level) { assert(pin < 30); levels[pin] = level; }
void pwm_set_wrap(uint slice, uint16_t wrap) { assert(slice < 8); (void)wrap; }
void pwm_set_clkdiv(uint slice, float divider) { assert(slice < 8); (void)divider; }
void pwm_set_enabled(uint slice, bool enabled) { assert(slice < 8); (void)enabled; }
uint i2c_init(i2c_inst_t *i2c, uint baud) { (void)i2c; assert(baud==100000); return baud; }
void i2c_deinit(i2c_inst_t *i2c)
{
    (void)i2c; assert(!powered()); ++controller_resets;
    if (change_range_on_reset) registers[0x1b]=0;
}
int i2c_write_timeout_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src,
                         size_t len, bool nostop, uint timeout)
{
    (void)i2c; (void)addr; (void)nostop;
    assert(timeout == 5000);
    if (write_failure || write_failures_remaining) {
        if (write_failures_remaining) --write_failures_remaining;
        clock_us+=timeout; return -2;
    }
    selected_register = src[0];
    if (len == 2) registers[src[0]] = src[1];
    return (int)len;
}
int i2c_read_timeout_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst,
                        size_t len, bool nostop, uint timeout)
{
    (void)i2c; (void)addr; (void)nostop;
    assert(timeout == 5000);
    if (read_failure || read_failures_remaining) {
        if (read_failures_remaining) --read_failures_remaining;
        clock_us+=timeout; return -2;
    }
    if (short_gyro_read && selected_register==0x43 && len==6) {
        short_gyro_read=false; return 3;
    }
    if (gyro_model && selected_register == 0x43 && len == 6) {
        int left = ((int)levels[M1_A] - (int)levels[M1_B]) / 100;
        int right = ((int)levels[M2_B] - (int)levels[M2_A]) / 100;
        int16_t raw = gyro_stalled ? 0 : (int16_t)((right-left) * 4.0f * 32.8f * gyro_sensor_sign);
        registers[0x47] = (uint8_t)((uint16_t)raw >> 8);
        registers[0x48] = (uint8_t)raw;
    }
    memcpy(dst, registers + selected_register, len);
    return (int)len;
}
static void reset(void)
{
    for (int f = 0; f < FAULT_COUNT; ++f) fault_clear((fault_t)f);
    motion_state.turn_sign = motion_state.stop_armed = 0;
    gp21 = write_failure = read_failure = change_range_on_reset = short_gyro_read = false;
    read_failures_remaining=write_failures_remaining=controller_resets=0;
    recovery_log_delay_us=0;
    motion_log_length=0; motion_log[0]=0;
    gyro_model = gyro_stalled = false; gyro_sensor_sign = 1;
    clock_us = press_at_us = 0;
    gyro_heading_degrees = 0;
    memset(&drive,0,sizeof(drive));
    memset(registers, 0, sizeof(registers));
    registers[0x75] = 0x71;
    assert(motion_init());
}
int main(void)
{
    reset(); read_failure=true;
    assert(!motion_init() && fault_test(FAULT_IMU));
    read_failure=false; assert(motion_init() && !fault_test(FAULT_IMU));
    reset(); motion_arm_stop(); registers[0x47]=1; registers[0x48]=0x48; // 328 / 32.8 = 10 dps bias
    assert(motion_calibrate() && imu.gyro_bias_dps[2] > 9.9f);
    assert(motion_calibrate() && imu.gyro_bias_dps[2] > 9.9f);
    for (int sign=-1;sign<=1;sign+=2) {
        reset(); motion_arm_stop(); gyro_model=true; gyro_sensor_sign=sign;
        assert(motion_learn_direction() && turn_sign() == (float)sign && !powered());
        assert(motion_heading() <= 2.0f && motion_heading() >= -2.0f);
        assert(motion_drive(true,750,0) && !powered());
    }
    reset(); motion_arm_stop(); gyro_model=gyro_stalled=true;
    assert(!motion_learn_direction() && fault_test(FAULT_TURN) && !powered());
    assert(clock_us<=1010000);
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
    assert(!powered() && fault_test(FAULT_IMU) && clock_us == 15000 && controller_resets==2);
    reset(); motion_arm_stop(); set_turn_sign(1); read_failure = true;
    assert(!motion_drive(true, 750, 0));
    assert(!powered() && fault_test(FAULT_IMU) && clock_us == 15000 && controller_resets==2);
    // A transient preflight error is repaired before either wheel is powered.
    reset(); motion_arm_stop(); set_turn_sign(1); read_failures_remaining=1;
    assert(motion_drive(true,250,0) && !powered() && !fault_test(FAULT_IMU));
    assert(controller_resets==1 && moving_recoveries==0 && strstr(motion_log,"IMU RECOVERED"));
    reset(); motion_arm_stop(); set_turn_sign(1); write_failures_remaining=1;
    assert(motion_drive(true,250,0) && controller_resets==1 && !fault_test(FAULT_IMU));
    // A live read failure stops PWM immediately and ends the block. Controller
    // resets and validation happen only while off; the pause is not powered time.
    reset(); motion_arm_stop(); set_turn_sign(1);
    assert(motion_drive_start(true,250,0)); read_failures_remaining=1;
    assert(motion_drive_update() && !powered() && !motion_drive_active());
    assert(motion_drive_elapsed_ms()==5 && moving_recoveries==1 && gyro_uncertainty_degrees==5.0f);
    assert(motion_drive_finish() && motion_drive_elapsed_ms()==5 && clock_us==305000);
    assert(motion_drive(true,250,0) && !fault_test(FAULT_IMU));
    assert(strstr(motion_log,"reg 0x43, select 1, read -2"));
    // Queueing diagnostics after receipt must not change the sample timestamp
    // or make the motor-off logging interval appear as powered travel.
    reset(); motion_arm_stop(); set_turn_sign(1); registers[0x47]=1; registers[0x48]=0x48;
    assert(motion_drive_start(true,250,0)); read_failures_remaining=1; recovery_log_delay_us=10000;
    assert(motion_drive_update() && !powered() && clock_us==15000 && drive.previous_us==5000);
    assert(gyro_heading_degrees>0.049f && gyro_heading_degrees<0.051f);
    assert(motion_drive_elapsed_ms()==5 && motion_drive_finish());
    // A short transfer is not decoded as a valid gyro sample.
    reset(); motion_arm_stop(); set_turn_sign(1);
    assert(motion_drive_start(true,250,0)); short_gyro_read=true;
    assert(motion_drive_update() && !powered() && motion_drive_elapsed_ms()==1);
    assert(motion_drive_finish() && strstr(motion_log,"select 1, read 3"));
    // Do not resume after a long blind interval, repeated moving failures,
    // or a sensor reset that changes the range behind the driver's back.
    reset(); motion_arm_stop(); set_turn_sign(1);
    assert(motion_drive_start(true,250,0)); sleep_ms(30); read_failures_remaining=1;
    assert(!motion_drive_update() && !powered() && fault_test(FAULT_IMU));
    reset(); motion_arm_stop(); set_turn_sign(1);
    for (unsigned i=0;i<2;++i) {
        assert(motion_drive_start(true,250,0)); read_failures_remaining=1;
        assert(motion_drive_update() && !powered() && motion_drive_finish());
    }
    assert(motion_drive_start(true,250,0)); read_failures_remaining=1;
    assert(!motion_drive_update() && !powered() && moving_recoveries==2 && fault_test(FAULT_IMU));
    reset(); motion_arm_stop(); set_turn_sign(1); read_failures_remaining=1; change_range_on_reset=true;
    assert(!motion_drive_start(true,250,0) && !powered() && fault_test(FAULT_IMU));
    reset(); motion_arm_stop(); set_turn_sign(1); read_failure=true; press_at_us=1000;
    assert(!motion_drive_start(true,250,0) && !powered() && fault_test(FAULT_STOP_LATCHED));
    assert(controller_resets==0 && clock_us==5000);
    reset(); motion_arm_stop(); set_turn_sign(1);
    motors_set(-25,25); read_failures_remaining=1;
    float gyro[3]; uint64_t previous=time_us_64();
    assert(!read_motion_gyro(gyro,&previous,"TURN") && !powered() && fault_test(FAULT_IMU));
    assert(controller_resets==0);
    // Calibration bias must survive a controller-only recovery.
    reset(); motion_arm_stop(); set_turn_sign(1); imu.gyro_bias_dps[2]=1.5f; read_failures_remaining=1;
    assert(motion_drive_start(true,250,0) && imu.gyro_bias_dps[2]==1.5f);
    motion_stop();
    reset(); motion_arm_stop(); set_turn_sign(1);
    assert(motion_drive(true, 750, 0) && !powered());
    assert(motion_drive(false, 750, 0) && !powered());
    reset(); motion_arm_stop(); set_turn_sign(1);
    assert(motion_drive_start(true, 750, 0));
    assert(powered() && motion_drive_active() && motion_drive_elapsed_ms() == 0);
    sleep_ms(125); assert(motion_drive_update() && motion_drive_active());
    motion_stop();
    assert(!powered() && motion_drive_elapsed_ms() == 125);
    assert(motion_drive_finish() && clock_us == 425000);
    assert(motion_drive_elapsed_ms() == 125); // settling adds no translation
    assert(motion_drive_start(false, 200, 0));
    sleep_ms(200); assert(motion_drive_update() && !motion_drive_active());
    assert(motion_drive_finish() && motion_drive_elapsed_ms() == 200);

    reset(); motion_arm_stop(); set_turn_sign(1);
    assert(motion_drive_start(true,750,0));
    sleep_ms(25); gp21 = true;
    assert(!motion_drive_update() && !powered());
    assert(!motion_drive_finish() && motion_drive_elapsed_ms() == 25);

    reset(); motion_arm_stop(); set_turn_sign(1);
    registers[0x47] = 0x7f; registers[0x48] = 0xff;
    assert(!motion_drive(true,750,0) && !powered() && fault_test(FAULT_GYRO_SATURATED));
    reset(); motion_arm_stop(); set_turn_sign(1);
    gyro_heading_degrees = 30;
    assert(!motion_drive(true,750,0) && !powered() && fault_test(FAULT_HEADING));
    puts("Motion: stopped I2C recovery, preflight, bounded yaw uncertainty, powered-time accounting, range verification, STOP and persistent failures passed");
}
