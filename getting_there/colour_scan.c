#include "colour_scan.h"
#include "faults.h"
#include "line_search.h"
#include "motion.h"
#include "run_report.h"

_Static_assert(SCAN_STEPS * SCAN_STEP_DEG == 360, "scan covers a full turn");

// Three fresh readings while stopped; keep the median by G/R.
static bool scan_read(colour_reading_t *out)
{
    colour_reading_t r[SCAN_READS];
    for (unsigned i = 0; i < SCAN_READS; ++i) {
        if (!colour_sensor_read(&r[i])) return false;
        for (unsigned j = i; j && colour_gr_x1000(&r[j]) < colour_gr_x1000(&r[j - 1]); --j) {
            colour_reading_t swap = r[j]; r[j] = r[j - 1]; r[j - 1] = swap;
        }
    }
    *out = r[SCAN_READS / 2];
    return true;
}

// Turns on the spot through a full circle from origin, logging the colour
// under the sensor (which sits ahead of the wheels) at each 30-degree step,
// then estimates the direction of the mat centre. Ends facing origin + 330.
// farthest_rings: the most rings any reading was from its own mat's arrival
// ring (SCAN_OFF_MAT if a reading was not on a mat). Small means every
// direction around the robot is inner rings: the robot is at the centre.
scan_status_t colour_scan_turn(float origin, unsigned round, colour_scan_result_t *est,
                                      unsigned *lighter_mats, int *farthest_rings,
                                      bool (*can_continue)(void), bool (*checkpoint)(void))
{
    int angles[SCAN_STEPS];
    colour_reading_t readings[SCAN_STEPS];
    *lighter_mats = 0;
    *farthest_rings = 0;
    *est = (colour_scan_result_t){.valid = false, .max_depth = -1};
    for (unsigned i = 0; i < SCAN_STEPS; ++i) {
        if (!can_continue()) return SCAN_STOPPED;
        angles[i] = (int)i * SCAN_STEP_DEG;
        if (!motion_turn_to_heading(origin + (float)angles[i]) || !motion_pause_ms(200)) return SCAN_STOPPED;
        if (!scan_read(&readings[i])) {
            report_printf("COLOUR SCAN: sensor read error at %d degrees\n", angles[i]);
            return SCAN_READ_FAILED;
        }
        colour_mat_t mat;
        colour_zone_t zone = colour_classify_mat(&readings[i], &mat);
        if (colour_lighter_zone(mat, zone)) *lighter_mats |= 1u << mat;
        int rings = mat == COLOUR_MAT_NONE ? SCAN_OFF_MAT :
            colour_zone_depth(colour_arrival_zone(mat)) - colour_zone_depth(zone);
        if (rings > *farthest_rings) *farthest_rings = rings;
        report_printf("COLOUR_SCAN,round=%u,angle=%d,heading=%d,zone=%s,mat=%s,depth=%d,c=%u,r=%u,g=%u,b=%u,gr=%u,br=%u\n",
                      round, angles[i], (int)line_normalise_heading(motion_heading()),
                      colour_zone_name(zone), colour_mat_name(mat), colour_zone_depth(zone),
                      readings[i].c, readings[i].r, readings[i].g, readings[i].b,
                      colour_gr_x1000(&readings[i]), colour_br_x1000(&readings[i]));
        if (!checkpoint()) { fault_raise(FAULT_REPORT); return SCAN_STOPPED; }
    }
    *est = colour_scan_estimate(angles, readings, SCAN_STEPS);
    report_printf("COLOUR_SCAN_RESULT,round=%u,valid=%u,darkest_angle=%d,vector_angle=%d,depth_range=%d,max_depth=%d\n",
                  round, est->valid, est->darkest_angle, est->vector_angle, est->depth_range, est->max_depth);
    return SCAN_DONE;
}

