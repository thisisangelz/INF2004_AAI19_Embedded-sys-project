#ifndef GETTING_THERE_COLOUR_SCAN_H
#define GETTING_THERE_COLOUR_SCAN_H

#include "colour_sensor.h"

#define SCAN_STEPS 12u
#define SCAN_STEP_DEG 30
#define SCAN_READS 3u
#define SCAN_OFF_MAT 1000

typedef enum { SCAN_DONE, SCAN_READ_FAILED, SCAN_STOPPED } scan_status_t;

// Stopped, main-core operation. Hooks enforce the caller's run deadline and
// safe flash checkpoint policy. All pointers/hooks must be non-null.
// Ends facing origin + 330 degrees on success; never translates the robot.
// lighter_mats is a mat bitmask; farthest_rings is distance in rings from
// arrival (SCAN_OFF_MAT when any sample is off a mat).
scan_status_t colour_scan_turn(float origin, unsigned round, colour_scan_result_t *estimate,
                              unsigned *lighter_mats, int *farthest_rings,
                              bool (*can_continue)(void), bool (*checkpoint)(void));

#endif
