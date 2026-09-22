#include "imu_mpu9250.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define MPU9250_ADDRESS 0x68
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_GYRO_XOUT_H 0x43

static bool write_reg(const mpu9250_t *imu, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_write_blocking(imu->i2c, imu->address, data, 2, false) == 2;
}

static bool read_regs(const mpu9250_t *imu, uint8_t reg, uint8_t *data,
                     size_t length)
{
    if (i2c_write_blocking(imu->i2c, imu->address, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(imu->i2c, imu->address, data, length, false) ==
           (int)length;
}

bool mpu9250_init(mpu9250_t *imu, i2c_inst_t *i2c, uint sda_pin,
                  uint scl_pin, uint32_t bus_hz)
{
    *imu = (mpu9250_t){.i2c = i2c, .address = MPU9250_ADDRESS};
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
        !write_reg(imu, MPU9250_GYRO_CONFIG, 0x00)) {
        return false;
    }
    imu->who_am_i = id;
    return true;
}

bool mpu9250_read_gyro_dps(const mpu9250_t *imu, float gyro_dps[3])
{
    uint8_t data[6];
    if (!read_regs(imu, MPU9250_GYRO_XOUT_H, data, sizeof(data))) return false;
    for (int axis = 0; axis < 3; ++axis) {
        int16_t raw = (int16_t)((data[axis * 2] << 8) | data[axis * 2 + 1]);
        gyro_dps[axis] = (float)raw / 131.0f - imu->gyro_bias_dps[axis];
    }
    return true;
}
