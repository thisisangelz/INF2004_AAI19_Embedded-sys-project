#include "colour_sensor.h"
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "run_report.h"

#define TCS_I2C i2c0
#define TCS_SDA_PIN 0
#define TCS_SCL_PIN 1
#define TCS_ADDR 0x29
#define TCS_BAUD 100000
#define TCS_TIMEOUT_US 5000

#define TCS_CMD 0x80
#define TCS_CMD_INC 0xA0
#define REG_ENABLE 0x00
#define REG_ATIME 0x01
#define REG_CONTROL 0x0F
#define REG_ID 0x12
#define REG_STATUS 0x13
#define REG_CDATAL 0x14
#define ENABLE_PON 0x01
#define ENABLE_AEN 0x02
#define STATUS_AVALID 0x01
// Must match the calibration: 43 cycles (~103 ms) at 4x gain.
#define ATIME_VALUE 0xD5
#define GAIN_4X 0x01
#define INTEGRATION_MS 103u

// Classification thresholds: midpoints between the pooled calibration ranges.
// G/R is the main cue; brightness C only separates the floor from ring 1.
#define GR_WHITE_MIN 968u
#define GR_RING1_MIN 856u
#define GR_RING2_MIN 707u
#define GR_RING3_MIN 552u
#define GR_RING4_MIN 409u
#define C_RING1_MIN 4660u
// Far below every calibrated surface (lowest: 1729): sensor lifted or covered.
#define C_USABLE_MIN 600u
// Consecutive failed checks before arrival is disabled for the rest of the run.
#define MAX_FAILED_CHECKS 3u
_Static_assert(COLOUR_ARRIVAL_SAMPLES == 3, "COLOUR log line prints exactly three zones");

static bool available;
static bool lighter_mat_seen;
static unsigned failed_checks;

static int tcs_write(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {(uint8_t)(TCS_CMD | reg), value};
    return i2c_write_timeout_us(TCS_I2C, TCS_ADDR, bytes, 2, false, TCS_TIMEOUT_US) == 2;
}

static int tcs_read(uint8_t reg, uint8_t *data, size_t length)
{
    uint8_t cmd = (uint8_t)(TCS_CMD_INC | reg);
    return i2c_write_timeout_us(TCS_I2C, TCS_ADDR, &cmd, 1, true, TCS_TIMEOUT_US) == 1 &&
           i2c_read_timeout_us(TCS_I2C, TCS_ADDR, data, length, false, TCS_TIMEOUT_US) == (int)length;
}

uint16_t colour_gr_x1000(const colour_reading_t *reading)
{
    if (!reading->r) return 0xFFFFu;
    uint32_t ratio = (uint32_t)reading->g * 1000u / reading->r;
    return ratio > 0xFFFFu ? 0xFFFFu : (uint16_t)ratio;
}

colour_zone_t colour_classify(const colour_reading_t *reading)
{
    if (reading->c < C_USABLE_MIN || !reading->r) return COLOUR_UNKNOWN;
    uint16_t gr = colour_gr_x1000(reading);
    if (gr >= GR_WHITE_MIN) return COLOUR_WHITE;
    if (gr >= GR_RING1_MIN) return reading->c > C_RING1_MIN ? COLOUR_RING1 : COLOUR_FLOOR;
    if (gr >= GR_RING2_MIN) return COLOUR_RING2;
    if (gr >= GR_RING3_MIN) return COLOUR_RING3;
    if (gr >= GR_RING4_MIN) return COLOUR_RING4;
    return COLOUR_RING5;
}

const char *colour_zone_name(colour_zone_t zone)
{
    switch (zone) {
    case COLOUR_FLOOR: return "FLOOR";
    case COLOUR_WHITE: return "WHITE";
    case COLOUR_RING1: return "RING1";
    case COLOUR_RING2: return "RING2";
    case COLOUR_RING3: return "RING3";
    case COLOUR_RING4: return "RING4";
    case COLOUR_RING5: return "RING5";
    default: return "UNKNOWN";
    }
}

bool colour_sensor_init(void)
{
    available = false;
    i2c_init(TCS_I2C, TCS_BAUD);
    gpio_set_function(TCS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TCS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(TCS_SDA_PIN);
    gpio_pull_up(TCS_SCL_PIN);
    uint8_t id = 0;
    if (!tcs_read(REG_ID, &id, 1) || (id != 0x44 && id != 0x4D)) {
        report_printf("COLOUR SENSOR: not found on GP0/GP1 (Grove 1); arrival uses the BLE proximity gate only\n");
        return false;
    }
    if (!tcs_write(REG_ATIME, ATIME_VALUE) || !tcs_write(REG_CONTROL, GAIN_4X) ||
        !tcs_write(REG_ENABLE, ENABLE_PON)) {
        report_printf("COLOUR SENSOR: configuration failed; arrival uses the BLE proximity gate only\n");
        return false;
    }
    sleep_ms(3); // datasheet: 2.4 ms after power-on
    if (!tcs_write(REG_ENABLE, ENABLE_PON | ENABLE_AEN)) {
        report_printf("COLOUR SENSOR: enable failed; arrival uses the BLE proximity gate only\n");
        return false;
    }
    available = true;
    report_printf("COLOUR SENSOR: TCS34725 ID 0x%02x ready on GP0/GP1; arrival at %s or darker after a lighter mat ring\n",
                  id, colour_zone_name(COLOUR_ARRIVAL_ZONE));
    return true;
}

bool colour_sensor_available(void) { return available; }

void colour_arrival_reset(void)
{
    lighter_mat_seen = false;
    failed_checks = 0;
}

// Waits one full integration so every reading is fresh, not a repeat.
static bool read_fresh(colour_reading_t *reading)
{
    sleep_ms(INTEGRATION_MS + 12u);
    uint8_t status = 0, d[8];
    if (!tcs_read(REG_STATUS, &status, 1) || !(status & STATUS_AVALID) ||
        !tcs_read(REG_CDATAL, d, sizeof(d))) return false;
    reading->c = (uint16_t)(d[0] | d[1] << 8);
    reading->r = (uint16_t)(d[2] | d[3] << 8);
    reading->g = (uint16_t)(d[4] | d[5] << 8);
    reading->b = (uint16_t)(d[6] | d[7] << 8);
    return true;
}

static bool lighter_mat(colour_zone_t zone)
{
    return zone >= COLOUR_WHITE && zone < COLOUR_ARRIVAL_ZONE;
}

bool colour_arrival_check(void)
{
    if (!available) return false;
    colour_zone_t zones[COLOUR_ARRIVAL_SAMPLES];
    colour_reading_t last = {0};
    bool all_dark = true, read_ok = true;
    for (unsigned i = 0; i < COLOUR_ARRIVAL_SAMPLES; ++i) {
        if (!read_fresh(&last)) { read_ok = false; zones[i] = COLOUR_UNKNOWN; break; }
        zones[i] = colour_classify(&last);
        if (zones[i] < COLOUR_ARRIVAL_ZONE) all_dark = false;
    }
    if (!read_ok) {
        if (++failed_checks >= MAX_FAILED_CHECKS) {
            available = false;
            report_printf("COLOUR SENSOR: %u failed checks; disabled, arrival uses the BLE proximity gate only\n",
                          failed_checks);
        } else {
            report_printf("COLOUR SENSOR: read failed (%u/%u)\n", failed_checks, MAX_FAILED_CHECKS);
        }
        return false;
    }
    failed_checks = 0;
    // Seen before this check's verdict, so one check cannot both introduce and
    // satisfy the lighter-ring requirement.
    bool seen_before = lighter_mat_seen;
    for (unsigned i = 0; i < COLOUR_ARRIVAL_SAMPLES; ++i)
        if (lighter_mat(zones[i])) lighter_mat_seen = true;
    bool arrived = all_dark && seen_before;
    report_printf("COLOUR,zones=%s/%s/%s,c=%u,r=%u,g=%u,b=%u,gr=%u,lighter_seen=%u,arrived=%u\n",
                  colour_zone_name(zones[0]), colour_zone_name(zones[1]), colour_zone_name(zones[2]),
                  last.c, last.r, last.g, last.b, colour_gr_x1000(&last), seen_before, arrived);
    return arrived;
}

bool colour_sensor_read(colour_reading_t *reading)
{
    return available && read_fresh(reading);
}

int colour_zone_depth(colour_zone_t zone)
{
    switch (zone) {
    case COLOUR_FLOOR: return 0;
    case COLOUR_WHITE: return 1;
    case COLOUR_RING1: return 2;
    case COLOUR_RING2: return 3;
    case COLOUR_RING3: return 4;
    case COLOUR_RING4: return 5;
    case COLOUR_RING5: return 6;
    default: return -1;
    }
}

colour_scan_result_t colour_scan_estimate(const int *angles, const colour_reading_t *readings,
                                          unsigned count)
{
    colour_scan_result_t result = {.valid = false};
    int min_depth = 7, max_depth = -1;
    uint16_t darkest_gr = 0xFFFFu;
    for (unsigned i = 0; i < count; ++i) {
        int depth = colour_zone_depth(colour_classify(&readings[i]));
        if (depth < 0) continue;
        if (depth < min_depth) min_depth = depth;
        uint16_t gr = colour_gr_x1000(&readings[i]);
        if (depth > max_depth || (depth == max_depth && gr < darkest_gr)) {
            max_depth = depth;
            darkest_gr = gr;
            result.darkest_angle = angles[i];
        }
    }
    if (max_depth < 0) return result;
    result.depth_range = max_depth - min_depth;
    if (result.depth_range < 1) return result;
    // Weight each direction by how much deeper it is than the shallowest.
    float x = 0.0f, y = 0.0f;
    for (unsigned i = 0; i < count; ++i) {
        int depth = colour_zone_depth(colour_classify(&readings[i]));
        if (depth <= min_depth) continue;
        float radians = (float)angles[i] * 3.14159265f / 180.0f;
        x += (float)(depth - min_depth) * cosf(radians);
        y += (float)(depth - min_depth) * sinf(radians);
    }
    int angle = (int)lroundf(atan2f(y, x) * 180.0f / 3.14159265f);
    result.vector_angle = (angle % 360 + 360) % 360;
    result.valid = true;
    return result;
}