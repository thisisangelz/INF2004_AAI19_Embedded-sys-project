#ifndef GETTING_THERE_TARGET_BEACON_H
#define GETTING_THERE_TARGET_BEACON_H

// The beacon the robot is approaching. CMake builds one firmware per first
// beacon: getting_there (AP1), robot_ap2 and robot_ap3, and the route builds
// robot_route and robot_leg2 (AP1, then AP2). Each beacon has its own colour
// mat, and the robot ignores the other two (they read as floor).
#include <stdint.h>
#include "colour_sensor.h"
#ifndef TARGET_BEACON_ID
#define TARGET_BEACON_ID 1
#endif
#if TARGET_BEACON_ID < 1 || TARGET_BEACON_ID > 3
#error "TARGET_BEACON_ID must be 1, 2 or 3"
#endif
#define TARGET_BEACON_COUNT 3u

// The current target. It starts as TARGET_BEACON_ID; a route selects the
// next beacon after each verified transfer.
void target_select(uint8_t id);       // 1..3; other values are ignored
uint8_t target_id(void);              // 1..3
const char *target_name(void);        // "AP1", "AP2" or "AP3" in log lines
// Wi-Fi SSIDs set in the picow_access_point*_powerbank beacon builds.
const char *target_ssid(void);
// AP1 pink (red) mat, AP2 blue, AP3 purple.
colour_mat_t target_mat(void);
colour_mat_t target_mat_of(uint8_t id);
const char *target_name_of(uint8_t id);

#endif
