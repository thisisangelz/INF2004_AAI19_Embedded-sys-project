// Colour mat maths with no hardware access: surface classification, mat depth
// and the turn-on-the-spot direction estimate. Shared by the robot firmware,
// the colour tests and the navigation tests.
#include "colour_sensor.h"
#include <math.h>

// Far below every calibrated surface (lowest: 1729): sensor lifted or covered.
#define C_USABLE_MIN 600u

// ---- Which mat ----
// Ratios x1000, from docs/calibration (6 November pink, 7 November blue and
// purple). The floor and every pink surface have B/R at most 987 (pink white
// edge); blue and purple surfaces at least 1082 (purple white edge). G/R then
// splits purple (at most 1124, its white edge) from blue (at least 1202, its
// white edge).
#define BR_BLUE_PURPLE_MIN 1035u
#define GR_BLUE_MIN 1163u

// ---- Pink mat (5 rings): G/R falls inward; C and B/R separate the floor ----
#define PINK_GR_WHITE_MIN 968u
#define PINK_GR_RING1_MIN 856u
#define PINK_GR_RING2_MIN 707u
#define PINK_GR_RING3_MIN 552u
#define PINK_GR_RING4_MIN 409u
#define PINK_C_RING1_MIN 4660u
// The floor's G/R lies between ring 1 and the white edge, and under the
// brighter 7 November lighting its C reached 5012, above PINK_C_RING1_MIN, so
// four floor stops in the side-on run read as ring 1. B/R does not depend on
// brightness: floor 724-813, ring 1 879-950 that day (6 November: 747-768 and
// 898-906). Ring 1 needs at least the midpoint.
#define PINK_BR_RING1_MIN 842u
// The white edge needs more: a person's shadow on the floor read G/R 975 with
// B/R up to 842 (7 November, purple round 1); pink white edge 871-986, the
// lowest values only where the sensor was half over the edge.
#define PINK_BR_WHITE_MIN 857u

// ---- Blue mat (6 rings): B/R rises inward ----
#define BLUE_BR_RING1_MIN 1262u
#define BLUE_BR_RING2_MIN 1418u
#define BLUE_BR_RING3_MIN 1661u
#define BLUE_BR_RING4_MIN 1993u
#define BLUE_BR_RING5_MIN 2283u
// Rings 5 and 6 overlap in B/R; ring 6 has the lower G/R (1541-1588 against
// 1622-1642).
#define BLUE_GR_RING6_MAX 1604u

// ---- Purple mat (6 rings): B/R rises inward ----
#define PURPLE_BR_RING2_MIN 1151u
#define PURPLE_BR_RING3_MIN 1217u
#define PURPLE_BR_RING4_MIN 1280u
#define PURPLE_BR_RING5_MIN 1353u
#define PURPLE_BR_RING6_MIN 1494u
// The white edge and ring 1 are 4 apart in B/R; G/R separates them (white
// 1107-1124, ring 1 1081-1086).
#define PURPLE_GR_WHITE_MIN 1097u

uint16_t colour_gr_x1000(const colour_reading_t *reading)
{
    if (!reading->r) return 0xFFFFu;
    uint32_t ratio = (uint32_t)reading->g * 1000u / reading->r;
    return ratio > 0xFFFFu ? 0xFFFFu : (uint16_t)ratio;
}

uint16_t colour_br_x1000(const colour_reading_t *reading)
{
    if (!reading->r) return 0xFFFFu;
    uint32_t ratio = (uint32_t)reading->b * 1000u / reading->r;
    return ratio > 0xFFFFu ? 0xFFFFu : (uint16_t)ratio;
}

static colour_zone_t classify_pink(uint16_t c, uint16_t gr, uint16_t br)
{
    if (gr >= PINK_GR_WHITE_MIN) return br >= PINK_BR_WHITE_MIN ? COLOUR_WHITE : COLOUR_FLOOR;
    if (gr >= PINK_GR_RING1_MIN)
        return c > PINK_C_RING1_MIN && br >= PINK_BR_RING1_MIN ? COLOUR_RING1 : COLOUR_FLOOR;
    if (gr >= PINK_GR_RING2_MIN) return COLOUR_RING2;
    if (gr >= PINK_GR_RING3_MIN) return COLOUR_RING3;
    if (gr >= PINK_GR_RING4_MIN) return COLOUR_RING4;
    return COLOUR_RING5;
}

static colour_zone_t classify_blue(uint16_t gr, uint16_t br)
{
    if (br < BLUE_BR_RING1_MIN) return COLOUR_WHITE;
    if (br < BLUE_BR_RING2_MIN) return COLOUR_RING1;
    if (br < BLUE_BR_RING3_MIN) return COLOUR_RING2;
    if (br < BLUE_BR_RING4_MIN) return COLOUR_RING3;
    if (br < BLUE_BR_RING5_MIN) return COLOUR_RING4;
    return gr <= BLUE_GR_RING6_MAX ? COLOUR_RING6 : COLOUR_RING5;
}

static colour_zone_t classify_purple(uint16_t gr, uint16_t br)
{
    if (br < PURPLE_BR_RING2_MIN) return gr >= PURPLE_GR_WHITE_MIN ? COLOUR_WHITE : COLOUR_RING1;
    if (br < PURPLE_BR_RING3_MIN) return COLOUR_RING2;
    if (br < PURPLE_BR_RING4_MIN) return COLOUR_RING3;
    if (br < PURPLE_BR_RING5_MIN) return COLOUR_RING4;
    if (br < PURPLE_BR_RING6_MIN) return COLOUR_RING5;
    return COLOUR_RING6;
}

colour_zone_t colour_classify_mat(const colour_reading_t *reading, colour_mat_t *mat)
{
    *mat = COLOUR_MAT_NONE;
    if (reading->c < C_USABLE_MIN || !reading->r) return COLOUR_UNKNOWN;
    uint16_t gr = colour_gr_x1000(reading), br = colour_br_x1000(reading);
    if (br < BR_BLUE_PURPLE_MIN) {
        colour_zone_t zone = classify_pink(reading->c, gr, br);
        if (zone != COLOUR_FLOOR) *mat = COLOUR_MAT_PINK;
        return zone;
    }
    if (gr >= GR_BLUE_MIN) {
        *mat = COLOUR_MAT_BLUE;
        return classify_blue(gr, br);
    }
    *mat = COLOUR_MAT_PURPLE;
    return classify_purple(gr, br);
}

colour_zone_t colour_classify(const colour_reading_t *reading)
{
    colour_mat_t mat;
    return colour_classify_mat(reading, &mat);
}

const char *colour_mat_name(colour_mat_t mat)
{
    switch (mat) {
    case COLOUR_MAT_PINK: return "PINK";
    case COLOUR_MAT_BLUE: return "BLUE";
    case COLOUR_MAT_PURPLE: return "PURPLE";
    default: return "NONE";
    }
}

colour_zone_t colour_arrival_zone(colour_mat_t mat)
{
    switch (mat) {
    case COLOUR_MAT_PINK: return COLOUR_RING4;                    // of 5 rings
    case COLOUR_MAT_BLUE: case COLOUR_MAT_PURPLE: return COLOUR_RING5; // of 6
    default: return COLOUR_UNKNOWN;
    }
}

bool colour_zone_arrived(colour_mat_t mat, colour_zone_t zone)
{
    return mat != COLOUR_MAT_NONE && zone >= colour_arrival_zone(mat);
}

bool colour_lighter_zone(colour_mat_t mat, colour_zone_t zone)
{
    return mat != COLOUR_MAT_NONE && zone >= COLOUR_WHITE && zone < colour_arrival_zone(mat);
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
    case COLOUR_RING6: return "RING6";
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
    case COLOUR_RING6: return 7;
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
