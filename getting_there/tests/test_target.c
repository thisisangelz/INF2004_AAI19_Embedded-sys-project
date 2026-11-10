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
    assert(target_beacon_id() == EXPECTED_ID);
    assert(strcmp(target_beacon_ssid(), EXPECTED_SSID) == 0);
    assert(strcmp(target_beacon_name(), name) == 0);
#if ROUTE_BEACON_COUNT == 2
    static const colour_mat_t mats[] = {COLOUR_MAT_NONE, COLOUR_MAT_BLUE, COLOUR_MAT_PINK};
#else
    static const colour_mat_t mats[] = {COLOUR_MAT_NONE, COLOUR_MAT_PINK, COLOUR_MAT_BLUE, COLOUR_MAT_PURPLE};
#endif
    assert(target_beacon_mat() == mats[EXPECTED_ID]);
    printf("Target AP%d: robot SSID \"%s\", beacon ID and %s mat match the beacon build\n",
           EXPECTED_ID, target_beacon_ssid(), target_beacon_mat() == COLOUR_MAT_BLUE ? "blue" : target_beacon_mat() == COLOUR_MAT_PINK ? "pink" : "purple");
    assert(target_beacon_ssid_length() == strlen(target_beacon_ssid()));
    assert(!target_beacon_select(0) && !target_beacon_select(4));
#if ROUTE_BEACON_COUNT
    for (uint8_t id=1; id<=ROUTE_BEACON_COUNT; ++id) {
        assert(target_beacon_select(id) && target_beacon_id()==id && target_beacon_mat()==mats[id]);
        assert(target_beacon_name()[2] == '0'+id);
        assert(target_beacon_ssid_length()==strlen(target_beacon_ssid()));
    }
    assert(!target_beacon_select(ROUTE_BEACON_COUNT + 1));
#else
    for (uint8_t id=1; id<=3; ++id) assert(target_beacon_select(id) == (id==EXPECTED_ID));
    assert(target_beacon_id()==EXPECTED_ID);
#endif
    return 0;
}
