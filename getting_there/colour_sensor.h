#ifndef GETTING_THERE_COLOUR_SENSOR_H
#define GETTING_THERE_COLOUR_SENSOR_H
#include <stdbool.h>
#include <stdint.h>

// TCS34725 on Robo Pico Grove 1 (I2C0, SDA GP0, SCL GP1). Thresholds come from
// docs/calibration/ (gain 4x, 103 ms): the 5-ring pink mat (6 November) and the
// 6-ring blue and purple mats (7 November). Each reading identifies its mat,
// so any beacon can sit on any of the three mats.
typedef enum {
    COLOUR_UNKNOWN, // no reading, sensor error or unusable value
    COLOUR_FLOOR,
    COLOUR_WHITE,   // white mat edge
    COLOUR_RING1,   // lightest ring
    COLOUR_RING2,
    COLOUR_RING3,
    COLOUR_RING4,
    COLOUR_RING5,   // darkest on the pink mat
    COLOUR_RING6,   // darkest on the blue and purple mats
} colour_zone_t;

typedef enum {
    COLOUR_MAT_NONE,   // floor or unknown
    COLOUR_MAT_PINK,   // 5 rings
    COLOUR_MAT_BLUE,   // 6 rings
    COLOUR_MAT_PURPLE, // 6 rings
    COLOUR_MAT_COUNT
} colour_mat_t;

// Arrival: one ring short of the darkest (pink ring 4, blue and purple ring 5)
// or darker, on the same mat, in every reading of one stationary check, after
// a lighter surface of that mat (white edge or an outer ring) was seen earlier.
#define COLOUR_ARRIVAL_SAMPLES 3
#define COLOUR_ARRIVAL_TEXT "the arrival ring (pink ring 4, blue or purple ring 5)"

typedef struct {
    uint16_t c, r, g, b;
} colour_reading_t;

// Pure helpers, no hardware access (colour_mat.c).
colour_zone_t colour_classify_mat(const colour_reading_t *reading, colour_mat_t *mat);
colour_zone_t colour_classify(const colour_reading_t *reading);
const char *colour_zone_name(colour_zone_t zone);
const char *colour_mat_name(colour_mat_t mat);
colour_zone_t colour_arrival_zone(colour_mat_t mat);
bool colour_zone_arrived(colour_mat_t mat, colour_zone_t zone);
// White edge or an outer ring of that mat: must be seen before arrival.
bool colour_lighter_zone(colour_mat_t mat, colour_zone_t zone);
uint16_t colour_gr_x1000(const colour_reading_t *reading);
uint16_t colour_br_x1000(const colour_reading_t *reading);

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
// Mat depth (see colour_zone_depth) of the latest check's last reading;
// -1 before a check, after a failed read or without a sensor.
int colour_last_depth(void);
// True when a reading of the latest check was within this many rings of its
// own mat's arrival ring (pink ring 4, blue or purple ring 5), or past it.
// 0: on the arrival ring; 2: pink ring 2 or blue/purple ring 3 and inward.
bool colour_near_arrival(unsigned rings);
// Records that lighter surfaces of these mats (bit 1u << colour_mat_t) were
// seen outside an arrival check, for example during a colour scan, so a robot
// started on the arrival ring can arrive.
void colour_note_lighter_mats(unsigned mat_mask);
// One fresh reading (about 115 ms). False on an I2C error or missing sensor.
bool colour_sensor_read(colour_reading_t *reading);

// Mat depth: 0 floor, 1 white edge, 2-7 rings 1-6; -1 unknown. It orders the
// surfaces inward on every mat, whichever ratio the mat's colour changes.
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
    int max_depth;      // deepest depth seen; -1 if every reading was unknown
} colour_scan_result_t;
colour_scan_result_t colour_scan_estimate(const int *angles, const colour_reading_t *readings,
                                          unsigned count);

#endif
