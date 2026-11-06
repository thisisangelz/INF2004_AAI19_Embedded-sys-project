/* TCS34725 colour-mat calibration. Standalone: no motors, no radio.
 *
 * Sensor: I2C0 SDA=GP0, SCL=GP1 (Robo Pico Grove 1), address 0x29.
 * Buttons: GP20 captures the current surface, GP21 steps back one surface.
 * Buzzer: GP22 beeps when a capture starts and ends.
 *
 * Put the robot so the sensor is over each surface in turn, press GP20 and
 * keep it still for about 2 s. After the last surface the programme prints a
 * summary, a CSV block and whether neighbouring rings can be told apart.
 *
 * I2C access follows tcs34725_diag on dev/tooling-setup: every transfer has a
 * 5 ms deadline, so a loose cable reports an error instead of hanging. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define TCS_I2C        i2c0
#define TCS_SDA_PIN    0
#define TCS_SCL_PIN    1
#define TCS_ADDR       0x29
#define TCS_BAUD       100000
#define TCS_TIMEOUT_US 5000

#define CAPTURE_BUTTON 20
#define BACK_BUTTON    21
#define BUZZER_PIN     22

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

/* 43 integration cycles of 2.4 ms: ~103 ms, full scale 43 * 1024 counts. */
#define ATIME_VALUE    0xD5
#define INTEGRATION_MS 103
#define FULL_SCALE     (43u * 1024u)
/* CONTROL register: 0 = 1x, 1 = 4x, 2 = 16x, 3 = 60x. */
#ifndef TCS_GAIN
#define TCS_GAIN       1
#endif

#define SAMPLES_PER_CAPTURE 20
#define LIVE_PERIOD_MS      500

typedef struct {
    uint16_t c, r, g, b;
} rgbc_t;

typedef struct {
    uint32_t n;
    uint32_t sum_c, sum_r, sum_g, sum_b;
    uint16_t min_c, max_c;
    uint32_t sum_gr, sum_br;   /* ratios x1000 */
    uint16_t min_gr, max_gr, min_br, max_br;
    bool saturated;
} stats_t;

/* Lightest to darkest; the beacon box sits on the darkest centre. */
static const char *const LABELS[] = {
    "FLOOR",
    "WHITE EDGE",
    "RING 1 (lightest pink)",
    "RING 2",
    "RING 3",
    "RING 4",
    "RING 5 (darkest, beside beacon box)",
    "OTHER (shadow / chair leg / tape)",
};
#define LABEL_COUNT    (int)(sizeof(LABELS) / sizeof(LABELS[0]))
#define FIRST_MAT      1   /* WHITE EDGE */
#define LAST_MAT       6   /* RING 5 */
#define OTHER_LABEL    7

static stats_t results[LABEL_COUNT];
static uint32_t i2c_errors;
static uint16_t buzzer_wrap;

static absolute_time_t deadline(void) { return make_timeout_time_us(TCS_TIMEOUT_US); }

static bool tcs_read(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t cmd = TCS_CMD_INC | reg;
    if (i2c_write_blocking_until(TCS_I2C, TCS_ADDR, &cmd, 1, true, deadline()) != 1 ||
        i2c_read_blocking_until(TCS_I2C, TCS_ADDR, buf, len, false, deadline()) != (int)len) {
        i2c_errors++;
        return false;
    }
    return true;
}

static bool tcs_write(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {(uint8_t)(TCS_CMD | reg), value};
    if (i2c_write_blocking_until(TCS_I2C, TCS_ADDR, bytes, 2, false, deadline()) != 2) {
        i2c_errors++;
        return false;
    }
    return true;
}

static bool tcs_start(void)
{
    uint8_t id = 0;
    if (!tcs_read(REG_ID, &id, 1)) {
        printf("SENSOR NOT FOUND at 0x29 on GP0/GP1 (Grove 1). Check the cable.\n");
        return false;
    }
    if (id != 0x44 && id != 0x4D) {
        printf("Unexpected sensor ID 0x%02X (expected 0x44 or 0x4D)\n", id);
        return false;
    }
    if (!tcs_write(REG_ATIME, ATIME_VALUE) || !tcs_write(REG_CONTROL, TCS_GAIN) ||
        !tcs_write(REG_ENABLE, ENABLE_PON)) {
        return false;
    }
    sleep_ms(3); /* datasheet: 2.4 ms after power-on */
    if (!tcs_write(REG_ENABLE, ENABLE_PON | ENABLE_AEN)) return false;
    sleep_ms(INTEGRATION_MS + 20);
    printf("Sensor ID 0x%02X ready | integration %d ms | gain setting %d\n",
           id, INTEGRATION_MS, TCS_GAIN);
    return true;
}

/* Waits one full integration so every call returns a fresh reading. */
static bool tcs_sample(rgbc_t *out)
{
    sleep_ms(INTEGRATION_MS + 10);
    uint8_t status = 0, d[8];
    if (!tcs_read(REG_STATUS, &status, 1) || !(status & STATUS_AVALID) ||
        !tcs_read(REG_CDATAL, d, sizeof(d))) {
        return false;
    }
    out->c = (uint16_t)(d[0] | d[1] << 8);
    out->r = (uint16_t)(d[2] | d[3] << 8);
    out->g = (uint16_t)(d[4] | d[5] << 8);
    out->b = (uint16_t)(d[6] | d[7] << 8);
    return true;
}

static uint16_t ratio_x1000(uint16_t num, uint16_t den)
{
    uint32_t value = (uint32_t)num * 1000u / (den ? den : 1u);
    return value > 65535u ? 65535u : (uint16_t)value;
}

static void beep(uint32_t ms)
{
    pwm_set_gpio_level(BUZZER_PIN, buzzer_wrap / 2);
    sleep_ms(ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

static void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 16.0f);
    buzzer_wrap = (uint16_t)(clock_get_hz(clk_sys) / (16u * 2000u) - 1u);
    pwm_config_set_wrap(&config, buzzer_wrap);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

static void button_init(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

/* True once per press-and-release (active low). */
static bool pressed(uint pin)
{
    if (gpio_get(pin)) return false;
    sleep_ms(30);
    if (gpio_get(pin)) return false;
    while (!gpio_get(pin)) sleep_ms(10);
    return true;
}

static void print_reading(const char *prefix, const rgbc_t *s)
{
    printf("%s C=%5u R=%5u G=%5u B=%5u | G/R=%u.%03u B/R=%u.%03u%s\n", prefix,
           s->c, s->r, s->g, s->b,
           ratio_x1000(s->g, s->r) / 1000, ratio_x1000(s->g, s->r) % 1000,
           ratio_x1000(s->b, s->r) / 1000, ratio_x1000(s->b, s->r) % 1000,
           s->c >= FULL_SCALE - 1 ? " | SATURATED: rebuild with TCS_GAIN=0" : "");
}

static void capture(int label)
{
    stats_t *st = &results[label];
    *st = (stats_t){.min_c = 0xFFFF, .min_gr = 0xFFFF, .min_br = 0xFFFF};
    printf("\nCAPTURING %s: keep the robot still...\n", LABELS[label]);
    beep(60);
    int attempts = 0;
    while (st->n < SAMPLES_PER_CAPTURE && attempts++ < SAMPLES_PER_CAPTURE * 3) {
        rgbc_t s;
        if (!tcs_sample(&s)) continue;
        uint16_t gr = ratio_x1000(s.g, s.r), br = ratio_x1000(s.b, s.r);
        st->n++;
        st->sum_c += s.c; st->sum_r += s.r; st->sum_g += s.g; st->sum_b += s.b;
        st->sum_gr += gr; st->sum_br += br;
        if (s.c < st->min_c) st->min_c = s.c;
        if (s.c > st->max_c) st->max_c = s.c;
        if (gr < st->min_gr) st->min_gr = gr;
        if (gr > st->max_gr) st->max_gr = gr;
        if (br < st->min_br) st->min_br = br;
        if (br > st->max_br) st->max_br = br;
        if (s.c >= FULL_SCALE - 1) st->saturated = true;
    }
    beep(150);
    if (st->n == 0) {
        printf("CAPTURE FAILED for %s: no valid readings (I2C errors %lu)\n",
               LABELS[label], (unsigned long)i2c_errors);
        return;
    }
    printf("CAPTURED %-36s n=%2lu | C %5lu [%5u..%5u] | R %5lu G %5lu B %5lu | "
           "G/R %4lu [%4u..%4u] | B/R %4lu [%4u..%4u] (ratios x1000)%s\n",
           LABELS[label], (unsigned long)st->n,
           (unsigned long)(st->sum_c / st->n), st->min_c, st->max_c,
           (unsigned long)(st->sum_r / st->n), (unsigned long)(st->sum_g / st->n),
           (unsigned long)(st->sum_b / st->n),
           (unsigned long)(st->sum_gr / st->n), st->min_gr, st->max_gr,
           (unsigned long)(st->sum_br / st->n), st->min_br, st->max_br,
           st->saturated ? " | SATURATED" : "");
}

/* Reports the gap between [lo_a..hi_a] (lighter) and [lo_b..hi_b] (darker):
 * positive means every lighter reading was above every darker one. */
static void print_gap(const char *what, int lighter_min, int darker_max)
{
    int gap = lighter_min - darker_max;
    printf("  %-4s gap %+6d  %s\n", what, gap, gap > 0 ? "separate" : "OVERLAP");
}

static void compare(int lighter, int darker)
{
    const stats_t *a = &results[lighter], *b = &results[darker];
    if (!a->n || !b->n) return;
    printf("%s -> %s\n", LABELS[lighter], LABELS[darker]);
    print_gap("C", a->min_c, b->max_c);
    print_gap("G/R", a->min_gr, b->max_gr);
}

static void print_summary(void)
{
    printf("\n========== CALIBRATION SUMMARY (gain setting %d, %d ms) ==========\n",
           TCS_GAIN, INTEGRATION_MS);
    printf("CSV,label,n,c_mean,c_min,c_max,r_mean,g_mean,b_mean,"
           "gr_mean,gr_min,gr_max,br_mean,br_min,br_max,saturated\n");
    for (int i = 0; i < LABEL_COUNT; ++i) {
        const stats_t *st = &results[i];
        if (!st->n) {
            printf("CSV,%s,0\n", LABELS[i]);
            continue;
        }
        printf("CSV,%s,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,%u,%u,%lu,%u,%u,%d\n", LABELS[i],
               (unsigned long)st->n, (unsigned long)(st->sum_c / st->n), st->min_c, st->max_c,
               (unsigned long)(st->sum_r / st->n), (unsigned long)(st->sum_g / st->n),
               (unsigned long)(st->sum_b / st->n), (unsigned long)(st->sum_gr / st->n),
               st->min_gr, st->max_gr, (unsigned long)(st->sum_br / st->n),
               st->min_br, st->max_br, st->saturated);
    }
    printf("\nNeighbour check: each step inward should get darker (lower C) and\n"
           "deeper pink (lower G/R). 'separate' means no reading overlapped.\n");
    compare(0, FIRST_MAT);       /* floor vs white edge */
    compare(0, FIRST_MAT + 1);   /* floor vs ring 1 */
    for (int i = FIRST_MAT; i < LAST_MAT; ++i) compare(i, i + 1);
    printf("Compare OTHER against the rings by eye: it must not look like the\n"
           "target ring, or a shadow could trigger a false arrival.\n");
    printf("========== END OF SUMMARY ==========\n");
}

int main(void)
{
    stdio_init_all();
    buzzer_init();
    button_init(CAPTURE_BUTTON);
    button_init(BACK_BUTTON);
    for (int i = 0; i < 30 && !stdio_usb_connected(); ++i) sleep_ms(100);

    i2c_init(TCS_I2C, TCS_BAUD);
    gpio_set_function(TCS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TCS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(TCS_SDA_PIN);
    gpio_pull_up(TCS_SCL_PIN);

    printf("\n===== TCS34725 colour-mat calibration =====\n");
    printf("GP20: capture current surface | GP21: go back one surface\n");
    while (!tcs_start()) sleep_ms(1000);

    int label = 0;
    uint32_t next_live_ms = 0;
    printf("\nPlace the sensor over: %s, then press GP20.\n", LABELS[label]);
    while (true) {
        if (pressed(CAPTURE_BUTTON)) {
            if (label >= LABEL_COUNT) {
                for (int i = 0; i < LABEL_COUNT; ++i) results[i].n = 0;
                label = 0;
                printf("\nNew round started.\n");
            } else {
                capture(label);
                if (results[label].n) label++;
                if (label >= LABEL_COUNT) {
                    print_summary();
                    printf("Copy everything above into the team notes.\n"
                           "GP20: new round | GP21: redo the last surface\n");
                    beep(400);
                }
            }
            if (label < LABEL_COUNT)
                printf("\nNext: place the sensor over %s, then press GP20.\n", LABELS[label]);
        }
        if (pressed(BACK_BUTTON) && label > 0) {
            label--;
            results[label].n = 0;
            printf("\nBack: redo %s, then press GP20.\n", LABELS[label]);
        }
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (label < LABEL_COUNT && (int32_t)(now - next_live_ms) >= 0) {
            rgbc_t s;
            char prefix[48];
            snprintf(prefix, sizeof(prefix), "LIVE [next: %.20s]", LABELS[label]);
            if (tcs_sample(&s)) print_reading(prefix, &s);
            else printf("LIVE read failed (I2C errors %lu)\n", (unsigned long)i2c_errors);
            next_live_ms = now + LIVE_PERIOD_MS;
        }
        sleep_ms(10);
    }
}
