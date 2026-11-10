#ifndef GETTING_THERE_TARGET_BEACON_H
#define GETTING_THERE_TARGET_BEACON_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "colour_sensor.h"
#ifndef TARGET_BEACON_ID
#define TARGET_BEACON_ID 1
#endif
#ifndef THREE_BEACON_ROUTE
#define THREE_BEACON_ROUTE 0
#endif
#ifndef ROUTE_BEACON_COUNT
// Keep the existing three-beacon build flag for older simulator builds.
#define ROUTE_BEACON_COUNT (THREE_BEACON_ROUTE ? 3 : 0)
#endif
#if TARGET_BEACON_ID < 1 || TARGET_BEACON_ID > 3
#error "TARGET_BEACON_ID must be 1, 2 or 3"
#endif
#if ROUTE_BEACON_COUNT != 0 && ROUTE_BEACON_COUNT != 2 && ROUTE_BEACON_COUNT != 3
#error "ROUTE_BEACON_COUNT must be 0 (single target), 2 or 3"
#endif
#if ROUTE_BEACON_COUNT && TARGET_BEACON_ID != 1
#error "Multi-beacon routes start at AP1"
#endif
uint8_t target_beacon_id(void);
const char *target_beacon_name(void);
const char *target_beacon_ssid(void);
size_t target_beacon_ssid_length(void); // excludes the terminating NUL
colour_mat_t target_beacon_mat(void);
// Low-level selection: main thread only, with BLE idle/disconnected and Wi-Fi
// quiesced. Hardware uses payload_client_reset_target_qualification(id) to
// select and reset under the CYW43/BTstack lock. No hardware access here.
// IDs outside the selected route (or fixed target) leave it unchanged.
bool target_beacon_select(uint8_t id);
#endif
