#ifndef IMU_MPU9250_H
#define IMU_MPU9250_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    uint8_t who_am_i;
    uint16_t gyro_range_dps;
    float gyro_lsb_per_dps;
    float gyro_bias_dps[3];
} mpu9250_t;

bool mpu9250_init(mpu9250_t *imu, i2c_inst_t *i2c, uint sda_pin,
                  uint scl_pin, uint32_t bus_hz);
// Accepts 250, 500, 1000 or 2000 degrees per second. A verified range change
// clears the gyro biases, so calibrate again before integrating readings.
// On failure, stop movement and reinitialise before using more gyro readings.
bool mpu9250_set_gyro_range(mpu9250_t *imu, uint16_t range_dps);
bool mpu9250_read_gyro_dps(const mpu9250_t *imu, float gyro_dps[3]);

#endif
