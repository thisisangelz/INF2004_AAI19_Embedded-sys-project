// Colour mat maths with no hardware access: surface classification, mat depth
// and the turn-on-the-spot direction estimate. Shared by the robot firmware,
// the colour tests and the navigation tests.
#include "colour_sensor.h"
#include <math.h>

// Classification thresholds: midpoints between the pooled calibration ranges.
// G/R is the main cue; brightness C only separates the floor from ring 1.
#define GR_WHITE_MIN 968u
#define GR_RING1_MIN 856u
#define GR_RING2_MIN 707u
#define GR_RING3_MIN 552u
#define GR_RING4_MIN 409u
#define C_RING1_MIN 4660u
// Blue/red x1000. The floor's G/R lies between ring 1 and the white edge,
// and under the brighter 7 November lighting its C reached 5012, above
// C_RING1_MIN, so four floor stops in the side-on run read as ring 1. B/R,
// like G/R, does not depend on brightness: floor 724-813 (53 readings),
// ring 1 879-950, white edge 871-986 in that day's logs. White and ring 1
// must also reach this midpoint; otherwise the reading is floor.
#define BR_MAT_MIN 842u
// Far below every calibrated surface (lowest: 1729): sensor lifted or covered.
#define C_USABLE_MIN 600u

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
    bool mat_blue = (uint32_t)reading->b * 1000u / reading->r >= BR_MAT_MIN;
    if (gr >= GR_WHITE_MIN) return mat_blue ? COLOUR_WHITE : COLOUR_FLOOR;
    if (gr >= GR_RING1_MIN) return reading->c > C_RING1_MIN && mat_blue ? COLOUR_RING1 : COLOUR_FLOOR;
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
    result.max_depth = max_depth;
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
