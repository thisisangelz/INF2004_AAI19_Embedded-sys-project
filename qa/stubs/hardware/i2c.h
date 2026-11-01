#ifndef QA_STUB_HARDWARE_I2C_H
#define QA_STUB_HARDWARE_I2C_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "pico/stdlib.h"
typedef struct i2c_inst { int unused; } i2c_inst_t;
extern i2c_inst_t *i2c0;
extern i2c_inst_t *i2c1;
uint i2c_init(i2c_inst_t *i2c, uint baudrate);
int i2c_write_blocking(i2c_inst_t *i2c, uint8_t address, const uint8_t *src,
                       size_t length, bool nostop);
int i2c_read_blocking(i2c_inst_t *i2c, uint8_t address, uint8_t *dst,
                      size_t length, bool nostop);
#endif

