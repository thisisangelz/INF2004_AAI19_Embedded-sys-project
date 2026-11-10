#include "target_beacon.h"
#include "packed_bits.h"
#if ROUTE_BEACON_COUNT
// IDs 1..3 need two bits. Only main writes, while holding the radio lock;
// callbacks read only while their target is immutable for the whole attempt.
static struct { uint8_t id : 2; } target = {TARGET_BEACON_ID};
_Static_assert(2 == BITS_NEEDED(3), "target ID width");
uint8_t target_beacon_id(void) { return target.id; }
#else
uint8_t target_beacon_id(void) { return TARGET_BEACON_ID; }
#endif
bool target_beacon_select(uint8_t id)
{
#if ROUTE_BEACON_COUNT
    if (id < 1 || id > ROUTE_BEACON_COUNT) return false;
    target.id = id;
    return true;
#else
    return id == TARGET_BEACON_ID;
#endif
}
const char *target_beacon_name(void)
{
    static const char *const names[] = {"AP1", "AP2", "AP3"};
    return names[target_beacon_id() - 1u];
}
const char *target_beacon_ssid(void)
{
    static const char *const ssids[] = {"I AM PICO W", "I AM PICO W 2", "I AM PICO W 3"};
    return ssids[target_beacon_id() - 1u];
}
size_t target_beacon_ssid_length(void) { return target_beacon_id() == 1 ? 11u : 13u; }
colour_mat_t target_beacon_mat(void)
{
    static const colour_mat_t mats[] = {COLOUR_MAT_PINK, COLOUR_MAT_BLUE, COLOUR_MAT_PURPLE};
    return mats[target_beacon_id() - 1u];
}
