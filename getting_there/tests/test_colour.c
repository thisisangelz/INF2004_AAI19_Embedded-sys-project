#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "colour_sensor.h"
#include "hardware/i2c.h"

// Fake TCS34725 on I2C0: returns the scripted readings in order.
static i2c_inst_t bus0;
i2c_inst_t *i2c0 = &bus0;
static uint8_t selected_reg, sensor_id = 0x44;
static colour_reading_t script[32];
static unsigned script_length, script_next;
static bool bus_dead;
static unsigned config_writes, reads;
static char log_text[8192];
static size_t log_length;

void report_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = vsnprintf(log_text + log_length, sizeof(log_text) - log_length, format, args);
    va_end(args);
    if (n > 0) log_length += (size_t)n;
}
void sleep_ms(uint32_t ms) { (void)ms; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
void gpio_pull_up(uint pin) { (void)pin; }
uint i2c_init(i2c_inst_t *i2c, uint baud) { assert(i2c == i2c0 && baud == 100000); return baud; }
int i2c_write_timeout_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len,
                         bool nostop, uint timeout)
{
    (void)nostop;
    assert(i2c == i2c0 && addr == 0x29 && timeout == 5000);
    if (bus_dead) return -1;
    selected_reg = src[0] & 0x1F;
    if (len == 2) ++config_writes;
    return (int)len;
}
int i2c_read_timeout_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len,
                        bool nostop, uint timeout)
{
    (void)nostop;
    assert(i2c == i2c0 && addr == 0x29 && timeout == 5000);
    if (bus_dead) return -2;
    if (selected_reg == 0x12) { dst[0] = sensor_id; return 1; }
    if (selected_reg == 0x13) { dst[0] = 0x01; return 1; } // AVALID
    assert(selected_reg == 0x14 && len == 8 && script_next < script_length);
    const colour_reading_t *r = &script[script_next++];
    const uint16_t v[4] = {r->c, r->r, r->g, r->b};
    for (int i = 0; i < 4; ++i) { dst[2 * i] = (uint8_t)v[i]; dst[2 * i + 1] = (uint8_t)(v[i] >> 8); }
    ++reads;
    return 8;
}

// Builds a reading with the given brightness and G/R ratio (x1000).
static colour_reading_t at(uint16_t c, uint16_t gr)
{
    return (colour_reading_t){.c = c, .r = 1000, .g = gr, .b = 800};
}

static void queue3(colour_reading_t a, colour_reading_t b, colour_reading_t c)
{
    script_length = script_next = 0;
    script[script_length++] = a;
    script[script_length++] = b;
    script[script_length++] = c;
}

static void check_calibration_data(void)
{
    // Means and range ends from all three calibration rounds (6 November).
    static const struct { uint16_t c, gr; colour_zone_t zone; } points[] = {
        {3067, 918, COLOUR_FLOOR}, {2616, 923, COLOUR_FLOOR}, {2902, 922, COLOUR_FLOOR},
        {2611, 918, COLOUR_FLOOR}, {3068, 924, COLOUR_FLOOR},
        {2379, 908, COLOUR_FLOOR}, {2813, 930, COLOUR_FLOOR}, // shadows on the floor
        {7808, 1013, COLOUR_WHITE}, {7101, 1010, COLOUR_WHITE}, {8301, 1025, COLOUR_WHITE},
        {6792, 1006, COLOUR_WHITE}, {8400, 1029, COLOUR_WHITE},
        {6274, 930, COLOUR_RING1}, {6450, 927, COLOUR_RING1}, {6603, 924, COLOUR_RING1},
        {6255, 924, COLOUR_RING1}, {6638, 931, COLOUR_RING1},
        {4707, 781, COLOUR_RING2}, {5150, 785, COLOUR_RING2}, {5268, 788, COLOUR_RING2},
        {4694, 781, COLOUR_RING2}, {5294, 788, COLOUR_RING2},
        {3651, 631, COLOUR_RING3}, {3768, 622, COLOUR_RING3}, {4135, 629, COLOUR_RING3},
        {3640, 622, COLOUR_RING3}, {4153, 632, COLOUR_RING3},
        {2638, 480, COLOUR_RING4}, {2640, 471, COLOUR_RING4}, {2906, 474, COLOUR_RING4},
        {2634, 471, COLOUR_RING4}, {2908, 481, COLOUR_RING4},
        {1892, 343, COLOUR_RING5}, {1927, 344, COLOUR_RING5}, {1988, 344, COLOUR_RING5},
        {1889, 343, COLOUR_RING5}, {1991, 345, COLOUR_RING5},
        {1732, 344, COLOUR_RING5}, // beacon holder's shadow lying on ring 5
    };
    for (size_t i = 0; i < sizeof(points) / sizeof(points[0]); ++i) {
        colour_reading_t r = at(points[i].c, points[i].gr);
        if (colour_classify(&r) != points[i].zone) {
            fprintf(stderr, "C=%u G/R=%u classified %s, expected %s\n", points[i].c, points[i].gr,
                    colour_zone_name(colour_classify(&r)), colour_zone_name(points[i].zone));
            assert(false);
        }
    }
    colour_reading_t covered = at(400, 900), no_red = {.c = 3000, .r = 0, .g = 500};
    assert(colour_classify(&covered) == COLOUR_UNKNOWN && colour_classify(&no_red) == COLOUR_UNKNOWN);
}

static void check_arrival_rules(void)
{
    const colour_reading_t floor = at(2900, 922), ring2 = at(5150, 785), ring3 = at(3768, 622);
    const colour_reading_t ring4 = at(2640, 474), ring5 = at(1927, 344);

    assert(colour_sensor_init() && colour_sensor_available() && config_writes == 4);
    colour_arrival_reset();
    // Dark readings with no lighter ring first: not arrival (a dark object on the floor).
    queue3(ring4, ring4, ring4); assert(!colour_arrival_check());
    queue3(floor, floor, floor); assert(!colour_arrival_check());
    // A lighter ring and dark readings in the same check do not count yet.
    queue3(ring3, ring4, ring4); assert(!colour_arrival_check());
    queue3(ring4, ring4, ring4); assert(colour_arrival_check());
    assert(strstr(log_text, "COLOUR,zones=RING4/RING4/RING4") && strstr(log_text, "arrived=1"));
    // One lighter reading among three blocks arrival.
    queue3(ring4, ring3, ring4); assert(!colour_arrival_check());
    queue3(ring5, ring5, ring4); assert(colour_arrival_check());

    // A new run forgets the lighter ring.
    colour_arrival_reset();
    queue3(ring5, ring5, ring5); assert(!colour_arrival_check());
    queue3(ring2, ring2, ring2); assert(!colour_arrival_check());
    queue3(ring4, ring5, ring4); assert(colour_arrival_check());
    // Floor shadows never count as a lighter mat ring.
    colour_arrival_reset();
    queue3(at(2379, 908), at(2813, 930), floor); assert(!colour_arrival_check());
    queue3(ring4, ring4, ring4); assert(!colour_arrival_check());

    // Read failures: two are tolerated, the third disables arrival for the run.
    colour_arrival_reset();
    bus_dead = true;
    assert(!colour_arrival_check() && colour_sensor_available());
    assert(!colour_arrival_check() && colour_sensor_available());
    assert(!colour_arrival_check() && !colour_sensor_available());
    assert(strstr(log_text, "disabled, arrival uses the BLE proximity gate only"));
    bus_dead = false;
    unsigned before = reads;
    queue3(ring4, ring4, ring4); assert(!colour_arrival_check() && reads == before);

    // A missing or wrong sensor leaves arrival off without any reads.
    sensor_id = 0x00;
    assert(!colour_sensor_init() && !colour_sensor_available());
    assert(strstr(log_text, "COLOUR SENSOR: not found on GP0/GP1"));
    assert(!colour_arrival_check() && reads == before);
    sensor_id = 0x4D; // TCS34727 shares the register map
    assert(colour_sensor_init());
}

int main(void)
{
    check_calibration_data();
    check_arrival_rules();
    puts("Colour: all three calibration rounds classified, shadows, lighter-ring-first arrival, three-sample agreement, run reset, read-failure fallback and missing sensor passed");
    return 0;
}
