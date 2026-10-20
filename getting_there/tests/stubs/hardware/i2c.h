#ifndef HOST_I2C_H
#define HOST_I2C_H
#include <stddef.h>
#include "pico/stdlib.h"
typedef struct { int unused; } i2c_inst_t;
extern i2c_inst_t *i2c1;
uint i2c_init(i2c_inst_t *i2c, uint baud);
int i2c_write_timeout_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src,
                         size_t len, bool nostop, uint timeout);
int i2c_read_timeout_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst,
                        size_t len, bool nostop, uint timeout);
#endif
