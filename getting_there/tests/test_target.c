// Compiled once per beacon by run_host_checks.py, which reads EXPECTED_ID and
// EXPECTED_SSID from that beacon's picow_access_point*_powerbank/CMakeLists.txt.
// TARGET_BEACON_ID is the build's first target (robot_ap2 for AP2), and the
// route builds select the same beacon later with target_select.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "target_beacon.h"

int main(void)
{
    char name[8];
    snprintf(name, sizeof(name), "AP%d", EXPECTED_ID);
    // AP1 pink (red) mat, AP2 blue, AP3 purple.
    static const colour_mat_t mats[] = {COLOUR_MAT_NONE, COLOUR_MAT_PINK, COLOUR_MAT_BLUE, COLOUR_MAT_PURPLE};
    // The build's starting target.
    assert(TARGET_BEACON_ID == EXPECTED_ID && target_id() == EXPECTED_ID);
    assert(strcmp(target_ssid(), EXPECTED_SSID) == 0);
    assert(strcmp(target_name(), name) == 0);
    assert(target_mat() == mats[EXPECTED_ID] && target_mat_of(EXPECTED_ID) == mats[EXPECTED_ID]);
    assert(strcmp(target_name_of(EXPECTED_ID), name) == 0);
    // Selected at run time from another beacon, as a route does.
    target_select(EXPECTED_ID == 1 ? 2 : 1);
    assert(target_id() != EXPECTED_ID);
    target_select(EXPECTED_ID);
    assert(target_id() == EXPECTED_ID && strcmp(target_ssid(), EXPECTED_SSID) == 0 &&
           strcmp(target_name(), name) == 0 && target_mat() == mats[EXPECTED_ID]);
    // Out-of-range IDs change nothing.
    target_select(0);
    target_select(4);
    assert(target_id() == EXPECTED_ID);
    printf("Target AP%d: robot SSID \"%s\", beacon ID and %s mat match the beacon build, also when selected at run time\n",
           EXPECTED_ID, target_ssid(), EXPECTED_ID == 1 ? "pink" : EXPECTED_ID == 2 ? "blue" : "purple");
    return 0;
}
