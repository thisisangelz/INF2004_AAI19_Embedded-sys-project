#ifndef GETTING_THERE_COLOUR_SENSOR_H
#define GETTING_THERE_COLOUR_SENSOR_H
#include <stdbool.h>
#include <stdint.h>

// TCS34725 on Robo Pico Grove 1 (I2C0, SDA GP0, SCL GP1). Thresholds come from
// docs/calibration/2026-11-06-tcs34725-mat.md (three rounds, gain 4x, 103 ms).
typedef enum {
    COLOUR_UNKNOWN, // no reading, sensor error or unusable value
    COLOUR_FLOOR,
    COLOUR_WHITE,   // white mat edge
    COLOUR_RING1,   // lightest pink
    COLOUR_RING2,
    COLOUR_RING3,
    COLOUR_RING4,
    COLOUR_RING5,   // darkest, under and beside the beacon box
} colour_zone_t;

// Arrival: this ring or darker in every reading of one stationary check, after
// a lighter mat surface (white edge or rings 1-3) was seen earlier in the run.
#define COLOUR_ARRIVAL_ZONE COLOUR_RING4
#define COLOUR_ARRIVAL_SAMPLES 3

typedef struct {
    uint16_t c, r, g, b;
} colour_reading_t;

// Pure helpers, no hardware access.
colour_zone_t colour_classify(const colour_reading_t *reading);
const char *colour_zone_name(colour_zone_t zone);
uint16_t colour_gr_x1000(const colour_reading_t *reading);

// Probes and configures the sensor. False leaves arrival disabled, so the
// robot keeps the BLE-only proximity gate.
bool colour_sensor_init(void);
bool colour_sensor_available(void);
// Clears the "lighter mat surface seen" memory at the start of a run.
void colour_arrival_reset(void);
// Call with motors stopped. Takes COLOUR_ARRIVAL_SAMPLES fresh readings
// (about 115 ms each), logs them and returns true when arrival is confirmed.
bool colour_arrival_check(void);
// True when the latest arrival check saw the mat (white edge or any ring) in
// at least one reading. False before the first check, after a failed read,
// or without a sensor. Navigation uses it to take smaller steps on the mat.
bool colour_on_mat(void);
// One fresh reading (about 115 ms). False on an I2C error or missing sensor.
bool colour_sensor_read(colour_reading_t *reading);

// Mat depth: 0 floor, 1 white edge, 2-6 rings 1-5; -1 unknown. Unlike G/R, it
// orders the floor below the mat (floor G/R lies between white and ring 1).
int colour_zone_depth(colour_zone_t zone);

// Direction to the mat centre from readings taken while turning on the spot:
// the sensor sits ahead of the wheels, so each angle samples a different
// point. Angles are degrees from the scan origin, in the same direction as
// the HEADING values in the log (set by the learnt gyro sign).
typedef struct {
    bool valid;         // at least two depths seen, so a direction exists
    int darkest_angle;  // angle of the deepest reading (lowest G/R on ties)
    int vector_angle;   // depth-weighted mean direction, 0..359
    int depth_range;    // deepest minus shallowest depth seen
} colour_scan_result_t;
colour_scan_result_t colour_scan_estimate(const int *angles, const colour_reading_t *readings,
                                          unsigned count);

#endif
