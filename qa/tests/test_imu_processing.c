#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t mock_register;
static uint8_t gyro_bytes[6];
static uint8_t gyro_config;
static bool fail_read;
static bool fail_write;

#include "pico_rssi_tracker/imu_mpu9250.c"

static i2c_inst_t mock_i2c;
i2c_inst_t *i2c0 = &mock_i2c;
i2c_inst_t *i2c1 = &mock_i2c;
uint i2c_init(i2c_inst_t *i2c, uint baudrate) { (void)i2c; return baudrate; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
void gpio_pull_up(uint pin) { (void)pin; }
uint64_t time_us_64(void) { return 0; }
void sleep_ms(uint32_t milliseconds) { (void)milliseconds; }
void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin, bool output) { (void)pin; (void)output; }
bool gpio_get(uint pin) { (void)pin; return true; }

int i2c_write_blocking(i2c_inst_t *i2c, uint8_t address, const uint8_t *src,
                       size_t length, bool nostop)
{
    (void)i2c; (void)address; (void)nostop;
    if (fail_write) return -1;
    mock_register = src[0];
    if (length == 2 && src[0] == MPU9250_GYRO_CONFIG) gyro_config = src[1];
    return (int)length;
}

int i2c_read_blocking(i2c_inst_t *i2c, uint8_t address, uint8_t *dst,
                      size_t length, bool nostop)
{
    (void)i2c; (void)address; (void)nostop;
    if (fail_read) return -1;
    if (mock_register == MPU9250_GYRO_XOUT_H && length == sizeof(gyro_bytes))
        memcpy(dst, gyro_bytes, length);
    else if (mock_register == MPU9250_GYRO_CONFIG && length == 1)
        dst[0] = gyro_config;
    else if (mock_register == MPU9250_WHO_AM_I && length == 1)
        dst[0] = 0x71;
    else
        memset(dst, 0, length);
    return (int)length;
}

int main(void)
{
    mpu9250_t imu = {.i2c = &mock_i2c, .address = 0x68,
                     .gyro_range_dps = 250, .gyro_lsb_per_dps = 131.0f};
    const uint8_t fixture[6] = {0x80, 0x00, 0x7f, 0xff, 0xff, 0x7d};
    memcpy(gyro_bytes, fixture, sizeof(fixture));
    float values[3];
    assert(mpu9250_read_gyro_dps(&imu, values));
    assert(values[0] < -250.0f && values[1] > 250.0f);
    assert(fabsf(values[2] + 1.0f) < 0.001f);

    imu.gyro_bias_dps[2] = 0.25f;
    assert(mpu9250_read_gyro_dps(&imu, values));
    assert(fabsf(values[2] + 1.25f) < 0.001f);
    assert(mpu9250_set_gyro_range(&imu, 1000));
    assert(imu.gyro_range_dps == 1000 && fabsf(imu.gyro_lsb_per_dps - 32.8f) < 0.001f);
    assert(imu.gyro_bias_dps[2] == 0.0f);
    assert(!mpu9250_set_gyro_range(&imu, 123));
    fail_read = true;
    assert(!mpu9250_read_gyro_dps(&imu, values));
    puts("IMU signed decoding, scaling, bias and failures: passed [HOST mocked I2C]");
    return 0;
}

