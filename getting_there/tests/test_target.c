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
    printf("Target AP%d: robot SSID \"%s\" and beacon ID match the beacon build\n", EXPECTED_ID, TARGET_SSID);
    return 0;
}
