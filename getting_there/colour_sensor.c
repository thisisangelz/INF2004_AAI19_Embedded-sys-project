// TCS34725 hardware access and the arrival rule. Classification and the scan
// estimate are in colour_mat.c.
#include "colour_sensor.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "run_report.h"
#include "motion.h"

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

// Consecutive failed checks before arrival is disabled for the rest of the run.
#define MAX_FAILED_CHECKS 3u
_Static_assert(COLOUR_ARRIVAL_SAMPLES == 3, "COLOUR log line prints exactly three zones");

static bool available;
// Bit (1u << colour_mat_t) per mat whose lighter surface has been seen.
static unsigned lighter_mats;
static bool on_mat;
static int last_depth = -1;
// Fewest rings between any reading of the latest check and its mat's arrival
// ring (0 or less: on it or inside). NOT_ON_MAT when no reading was on a mat.
#define NOT_ON_MAT 1000
static int last_rings_to_arrival = NOT_ON_MAT;
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
    report_printf("COLOUR SENSOR: TCS34725 ID 0x%02x ready on GP0/GP1; pink, blue and purple mats; arrival on %s or darker after a lighter ring of the same mat\n",
                  id, COLOUR_ARRIVAL_TEXT);
    return true;
}

bool colour_sensor_available(void) { return available; }

void colour_arrival_reset(void)
{
    lighter_mats = 0;
    on_mat = false;
    last_depth = -1;
    last_rings_to_arrival = NOT_ON_MAT;
    failed_checks = 0;
}

bool colour_on_mat(void) { return available && on_mat; }
int colour_last_depth(void) { return available ? last_depth : -1; }
bool colour_near_arrival(unsigned rings)
{
    return available && last_rings_to_arrival <= (int)rings;
}
void colour_note_lighter_mats(unsigned mat_mask) { lighter_mats |= mat_mask; }

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

bool colour_floor_check(bool *floor)
{
    *floor = false;
    if (!available) return false;
    bool clear = true;
    colour_mat_t previous = colour_target_mat();
    for (unsigned i = 0; i < COLOUR_ARRIVAL_SAMPLES; ++i) {
        colour_reading_t reading;
        if (motion_stop_requested() || !read_fresh(&reading)) return false;
        colour_set_target_mat(COLOUR_MAT_NONE);
        colour_zone_t zone = colour_classify(&reading);
        colour_set_target_mat(previous);
        if (zone == COLOUR_UNKNOWN) return false;
        if (zone != COLOUR_FLOOR) clear = false;
    }
    *floor = clear;
    return true;
}

bool colour_arrival_check(void)
{
    if (!available) return false;
    on_mat = false;
    last_depth = -1;
    last_rings_to_arrival = NOT_ON_MAT;
    colour_zone_t zones[COLOUR_ARRIVAL_SAMPLES];
    colour_mat_t mats[COLOUR_ARRIVAL_SAMPLES];
    colour_reading_t last = {0};
    bool all_dark = true, read_ok = true;
    for (unsigned i = 0; i < COLOUR_ARRIVAL_SAMPLES; ++i) {
        if (!read_fresh(&last)) {
            read_ok = false;
            zones[i] = COLOUR_UNKNOWN;
            mats[i] = COLOUR_MAT_NONE;
            break;
        }
        zones[i] = colour_classify_mat(&last, &mats[i]);
        // Every reading on the same mat, each at its arrival ring or darker.
        if (!colour_zone_arrived(mats[i], zones[i]) || mats[i] != mats[0]) all_dark = false;
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
    // The lighter surface must belong to the mat the robot arrives on.
    bool seen_before = (lighter_mats >> mats[0]) & 1u;
    for (unsigned i = 0; i < COLOUR_ARRIVAL_SAMPLES; ++i) {
        if (colour_lighter_zone(mats[i], zones[i])) lighter_mats |= 1u << mats[i];
        if (zones[i] >= COLOUR_WHITE) on_mat = true;
        if (mats[i] != COLOUR_MAT_NONE) {
            int rings = colour_zone_depth(colour_arrival_zone(mats[i])) - colour_zone_depth(zones[i]);
            if (rings < last_rings_to_arrival) last_rings_to_arrival = rings;
        }
    }
    last_depth = colour_zone_depth(zones[COLOUR_ARRIVAL_SAMPLES - 1]);
    bool arrived = all_dark && seen_before;
    report_printf("COLOUR,zones=%s/%s/%s,mats=%s/%s/%s,c=%u,r=%u,g=%u,b=%u,gr=%u,br=%u,lighter_seen=%u,arrived=%u\n",
                  colour_zone_name(zones[0]), colour_zone_name(zones[1]), colour_zone_name(zones[2]),
                  colour_mat_name(mats[0]), colour_mat_name(mats[1]), colour_mat_name(mats[2]),
                  last.c, last.r, last.g, last.b, colour_gr_x1000(&last), colour_br_x1000(&last),
                  seen_before, arrived);
    return arrived;
}

bool colour_sensor_read(colour_reading_t *reading)
{
    return available && read_fresh(reading);
}
