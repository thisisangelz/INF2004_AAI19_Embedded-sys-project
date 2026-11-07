// Compiled once per beacon by run_host_checks.py, which reads EXPECTED_ID and
// EXPECTED_SSID from that beacon's picow_access_point*_powerbank/CMakeLists.txt.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "target_beacon.h"

int main(void)
{
    char name[8];
    snprintf(name, sizeof(name), "AP%d", EXPECTED_ID);
    assert(TARGET_BEACON_ID == EXPECTED_ID);
    assert(strcmp(TARGET_SSID, EXPECTED_SSID) == 0);
    assert(strcmp(TARGET_NAME, name) == 0);
    // AP1 pink (red) mat, AP2 blue, AP3 purple.
    static const colour_mat_t mats[] = {COLOUR_MAT_NONE, COLOUR_MAT_PINK, COLOUR_MAT_BLUE, COLOUR_MAT_PURPLE};
    assert(TARGET_MAT == mats[EXPECTED_ID]);
    printf("Target AP%d: robot SSID \"%s\", beacon ID and %s mat match the beacon build\n",
           EXPECTED_ID, TARGET_SSID, EXPECTED_ID == 1 ? "pink" : EXPECTED_ID == 2 ? "blue" : "purple");
    return 0;
}
