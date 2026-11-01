/* TCS34725 hardware diagnostic. Standalone: no motors, no radio, no other GPIOs.
 * Wiring: I2C0 SDA=GP0, SCL=GP1, addr 0x29, 100 kHz.
 * Stage 1 (identify): probe 0x29, read ID register (0x12).
 * Stage 2 (measure): only after stage 1 passes; init sensor, read clear/R/G/B.
 * Results live in volatile globals (see tcs_diag) for GDB/OpenOCD inspection. */
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define TCS_I2C        i2c0
#define TCS_SDA_PIN    0
#define TCS_SCL_PIN    1
#define TCS_ADDR       0x29
#define TCS_BAUD       100000
#define TCS_TIMEOUT_US 5000
#define USB_WAIT_MS    3000

#define TCS_CMD        0x80
#define TCS_CMD_INC    0xA0
#define REG_ENABLE     0x00
#define REG_ATIME      0x01
#define REG_CONTROL    0x0F
#define REG_ID         0x12
#define REG_STATUS     0x13
#define REG_CDATAL     0x14
#define ENABLE_PON     0x01
#define ENABLE_AEN     0x02
#define STATUS_AVALID  0x01
#define ATIME_101MS    0xD5   /* 43 cycles, ~103 ms */
#define GAIN_4X        0x01

typedef enum {
    STAGE_BOOT = 0, STAGE_PROBE, STAGE_ID, STAGE_INIT, STAGE_MEASURE
} tcs_stage_t;

/* ID_OK: 0x44 (TCS34725/21) or 0x4D (TCS34723/27, same register map). */
typedef struct {
    volatile uint32_t stage;          /* tcs_stage_t reached */
    volatile int32_t  probe_result;   /* >=0 ACK, <0 pico error code */
    volatile uint8_t  probe_ack;      /* 1 if 0x29 ACKed */
    volatile uint8_t  id_value;       /* raw ID register */
    volatile uint8_t  id_ok;          /* 1 if 0x44 or 0x4D */
    volatile uint8_t  init_ok;        /* 1 if stage 2 init succeeded */
    volatile int32_t  last_error;     /* last failing I2C return code, 0 if none */
    volatile uint32_t i2c_error_count;
    volatile uint16_t clear, red, green, blue;
    volatile uint8_t  data_valid;     /* 1 if last read completed with AVALID */
    volatile uint32_t sample_count;
    volatile uint32_t sample_time_ms; /* time of last good sample */
} tcs_diag_t;

volatile tcs_diag_t tcs_diag;

static absolute_time_t deadline(void) { return make_timeout_time_us(TCS_TIMEOUT_US); }

/* Bounded register read: write cmd byte (no stop), repeated-start read. 0 = ok. */
static int tcs_read(uint8_t reg, uint8_t *buf, size_t len) {
    uint8_t cmd = TCS_CMD_INC | reg;
    int w = i2c_write_blocking_until(TCS_I2C, TCS_ADDR, &cmd, 1, true, deadline());
    if (w != 1) { tcs_diag.last_error = w; tcs_diag.i2c_error_count++; return w < 0 ? w : -1; }
    int r = i2c_read_blocking_until(TCS_I2C, TCS_ADDR, buf, len, false, deadline());
    if (r != (int)len) { tcs_diag.last_error = r; tcs_diag.i2c_error_count++; return r < 0 ? r : -1; }
    return 0;
}

static int tcs_write(uint8_t reg, uint8_t val) {
    uint8_t b[2] = { (uint8_t)(TCS_CMD | reg), val };
    int w = i2c_write_blocking_until(TCS_I2C, TCS_ADDR, b, 2, false, deadline());
    if (w != 2) { tcs_diag.last_error = w; tcs_diag.i2c_error_count++; return w < 0 ? w : -1; }
    return 0;
}

static void i2c_setup(void) {
    i2c_init(TCS_I2C, TCS_BAUD);
    gpio_set_function(TCS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TCS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(TCS_SDA_PIN);
    gpio_pull_up(TCS_SCL_PIN);
}

static bool stage1_identify(void) {
    tcs_diag.stage = STAGE_PROBE;
    uint8_t cmd = TCS_CMD | REG_ID;
    int p = i2c_write_blocking_until(TCS_I2C, TCS_ADDR, &cmd, 1, true, deadline());
    tcs_diag.probe_result = p;
    tcs_diag.probe_ack = (p == 1);
    if (p != 1) { tcs_diag.last_error = p; tcs_diag.i2c_error_count++; return false; }
    tcs_diag.stage = STAGE_ID;
    uint8_t id = 0;
    int r = i2c_read_blocking_until(TCS_I2C, TCS_ADDR, &id, 1, false, deadline());
    if (r != 1) { tcs_diag.last_error = r; tcs_diag.i2c_error_count++; return false; }
    tcs_diag.id_value = id;
    tcs_diag.id_ok = (id == 0x44 || id == 0x4D);
    return tcs_diag.id_ok;
}

static bool stage2_init(void) {
    tcs_diag.stage = STAGE_INIT;
    if (tcs_write(REG_ATIME, ATIME_101MS)) return false;
    if (tcs_write(REG_CONTROL, GAIN_4X)) return false;
    if (tcs_write(REG_ENABLE, ENABLE_PON)) return false;
    sleep_ms(3);                               /* datasheet: 2.4 ms after PON */
    if (tcs_write(REG_ENABLE, ENABLE_PON | ENABLE_AEN)) return false;
    sleep_ms(120);                             /* first integration */
    tcs_diag.init_ok = 1;
    return true;
}

static bool stage2_sample(void) {
    uint8_t st = 0, d[8];
    if (tcs_read(REG_STATUS, &st, 1)) { tcs_diag.data_valid = 0; return false; }
    if (!(st & STATUS_AVALID)) { tcs_diag.data_valid = 0; return false; }
    if (tcs_read(REG_CDATAL, d, 8)) { tcs_diag.data_valid = 0; return false; }
    tcs_diag.clear = (uint16_t)(d[0] | d[1] << 8);
    tcs_diag.red   = (uint16_t)(d[2] | d[3] << 8);
    tcs_diag.green = (uint16_t)(d[4] | d[5] << 8);
    tcs_diag.blue  = (uint16_t)(d[6] | d[7] << 8);
    tcs_diag.data_valid = 1;
    tcs_diag.sample_count++;
    tcs_diag.sample_time_ms = to_ms_since_boot(get_absolute_time());
    return true;
}

int main(void) {
    stdio_init_all();
    for (int i = 0; i < USB_WAIT_MS / 100 && !stdio_usb_connected(); i++) sleep_ms(100);
    i2c_setup();

    printf("TCS34725 diag: I2C0 SDA=GP0 SCL=GP1 addr=0x%02X @%d Hz\n", TCS_ADDR, TCS_BAUD);
    bool id_pass = stage1_identify();
    printf("probe=%ld ack=%u id=0x%02X id_ok=%u err=%ld\n",
           (long)tcs_diag.probe_result, tcs_diag.probe_ack, tcs_diag.id_value,
           tcs_diag.id_ok, (long)tcs_diag.last_error);

    bool measuring = false;
    for (;;) {
        if (!id_pass) {
            printf("STAGE1 FAIL: re-trying (probe=%ld err=%ld)\n",
                   (long)tcs_diag.probe_result, (long)tcs_diag.last_error);
            sleep_ms(1000);
            id_pass = stage1_identify();
            continue;
        }
        if (!measuring) {
            measuring = stage2_init();
            printf(measuring ? "STAGE2 init ok\n" : "STAGE2 init FAIL err=%ld\n",
                   (long)tcs_diag.last_error);
            if (!measuring) sleep_ms(1000);
            continue;
        }
        tcs_diag.stage = STAGE_MEASURE;
        if (stage2_sample())
            printf("C=%u R=%u G=%u B=%u n=%lu\n", tcs_diag.clear, tcs_diag.red,
                   tcs_diag.green, tcs_diag.blue, (unsigned long)tcs_diag.sample_count);
        else
            printf("sample invalid err=%ld errs=%lu\n", (long)tcs_diag.last_error,
                   (unsigned long)tcs_diag.i2c_error_count);
        sleep_ms(500);
    }
}
