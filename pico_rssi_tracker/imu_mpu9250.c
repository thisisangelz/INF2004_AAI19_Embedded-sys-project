#include "imu_mpu9250.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define MPU9250_ADDRESS 0x68
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_GYRO_RANGE_MASK 0x18
#define MPU9250_GYRO_XOUT_H 0x43
#define IMU_I2C_TIMEOUT_US 5000

static bool write_reg(const mpu9250_t *imu, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_write_timeout_us(imu->i2c, imu->address, data, 2, false,
                              IMU_I2C_TIMEOUT_US) == 2;
}

static bool read_regs_status(const mpu9250_t *imu, uint8_t reg, uint8_t *data,
                            size_t length, mpu9250_io_status_t *status)
{
    mpu9250_io_status_t result = {.reg = reg, .read_result = 0};
    result.select_result = i2c_write_timeout_us(imu->i2c, imu->address, &reg, 1, true,
                                               IMU_I2C_TIMEOUT_US);
    if (result.select_result == 1)
        result.read_result = i2c_read_timeout_us(imu->i2c, imu->address, data, length, false,
                                                IMU_I2C_TIMEOUT_US);
    if (status) *status = result;
    return result.select_result == 1 && result.read_result == (int)length;
}

static bool read_regs(const mpu9250_t *imu, uint8_t reg, uint8_t *data, size_t length)
{
    return read_regs_status(imu, reg, data, length, NULL);
}

bool mpu9250_init(mpu9250_t *imu, i2c_inst_t *i2c, uint sda_pin,
                  uint scl_pin, uint32_t bus_hz)
{
    *imu = (mpu9250_t){
        .i2c = i2c,
        .address = MPU9250_ADDRESS,
        .gyro_range_dps = 250,
        .gyro_lsb_per_dps = 131.0f
    };
    i2c_init(i2c, bus_hz);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    uint8_t id = 0;
    if (!read_regs(imu, MPU9250_WHO_AM_I, &id, 1) ||
        (id != 0x71 && id != 0x73 && id != 0x70)) {
        return false;
    }
    if (!write_reg(imu, MPU9250_PWR_MGMT_1, 0x00) ||
        !write_reg(imu, MPU9250_GYRO_CONFIG, 0x00) ||
        !mpu9250_set_gyro_range(imu, 250)) {
        return false;
    }
    imu->who_am_i = id;
    return true;
}

bool mpu9250_set_gyro_range(mpu9250_t *imu, uint16_t range_dps)
{
    uint8_t range_bits;
    float scale;
    // MPU-6500/9250 GYRO_CONFIG bits 4:3 and documented scale factors.
    switch (range_dps) {
        case 250: range_bits = 0x00; scale = 131.0f; break;
        case 500: range_bits = 0x08; scale = 65.5f; break;
        case 1000: range_bits = 0x10; scale = 32.8f; break;
        case 2000: range_bits = 0x18; scale = 16.4f; break;
        default: return false;
    }
    if (!imu || !imu->i2c) return false;

    uint8_t config;
    if (!read_regs(imu, MPU9250_GYRO_CONFIG, &config, 1)) return false;
    config = (uint8_t)((config & ~MPU9250_GYRO_RANGE_MASK) | range_bits);
    if (!write_reg(imu, MPU9250_GYRO_CONFIG, config)) return false;
    uint8_t verified;
    if (!read_regs(imu, MPU9250_GYRO_CONFIG, &verified, 1) ||
        (verified & MPU9250_GYRO_RANGE_MASK) != range_bits) {
        return false;
    }

    bool changed = imu->gyro_range_dps != range_dps;
    imu->gyro_range_dps = range_dps;
    imu->gyro_lsb_per_dps = scale;
    if (changed) {
        for (int axis = 0; axis < 3; ++axis) imu->gyro_bias_dps[axis] = 0.0f;
    }
    return true;
}

bool mpu9250_read_gyro_dps(const mpu9250_t *imu, float gyro_dps[3])
{
    return mpu9250_read_gyro_dps_status(imu, gyro_dps, NULL);
}

bool mpu9250_verify_config(const mpu9250_t *imu, mpu9250_io_status_t *status)
{
    uint8_t value;
    if (!read_regs_status(imu, MPU9250_WHO_AM_I, &value, 1, status) || value != imu->who_am_i)
        return false;
    if (!read_regs_status(imu, MPU9250_PWR_MGMT_1, &value, 1, status) || (value & 0xc0u))
        return false;
    unsigned expected = imu->gyro_range_dps == 1000 ? 0x10u :
                        imu->gyro_range_dps == 2000 ? 0x18u : imu->gyro_range_dps == 500 ? 0x08u : 0;
    return read_regs_status(imu, MPU9250_GYRO_CONFIG, &value, 1, status) &&
           (value & MPU9250_GYRO_RANGE_MASK) == expected;
}

bool mpu9250_read_gyro_dps_status(const mpu9250_t *imu, float gyro_dps[3],
                                mpu9250_io_status_t *status)
{
    uint8_t data[6];
    if (!read_regs_status(imu, MPU9250_GYRO_XOUT_H, data, sizeof(data), status)) return false;
    for (int axis = 0; axis < 3; ++axis) {
        int16_t raw = (int16_t)((data[axis * 2] << 8) | data[axis * 2 + 1]);
        gyro_dps[axis] = (float)raw / imu->gyro_lsb_per_dps -
                         imu->gyro_bias_dps[axis];
    }
    return true;
}
