#include "target_beacon.h"

typedef struct {
    const char *name, *ssid;
    colour_mat_t mat;
} beacon_info_t;

// Index 0 is unused so the table reads by beacon ID. The SSIDs must match
// WIFI_SSID in picow_access_point*_powerbank/CMakeLists.txt (checked by the
// host tests).
static const beacon_info_t beacons[TARGET_BEACON_COUNT + 1u] = {
    {"AP?", "", COLOUR_MAT_NONE},
    {"AP1", "I AM PICO W", COLOUR_MAT_PINK},
    {"AP2", "I AM PICO W 2", COLOUR_MAT_BLUE},
    {"AP3", "I AM PICO W 3", COLOUR_MAT_PURPLE},
};

static uint8_t current = TARGET_BEACON_ID;

static uint8_t valid_id(uint8_t id) { return id >= 1u && id <= TARGET_BEACON_COUNT ? id : 0u; }

void target_select(uint8_t id)
{
    if (valid_id(id)) current = id;
}

uint8_t target_id(void) { return current; }
const char *target_name(void) { return beacons[current].name; }
const char *target_ssid(void) { return beacons[current].ssid; }
colour_mat_t target_mat(void) { return beacons[current].mat; }
colour_mat_t target_mat_of(uint8_t id) { return beacons[valid_id(id)].mat; }
const char *target_name_of(uint8_t id) { return beacons[valid_id(id)].name; }
