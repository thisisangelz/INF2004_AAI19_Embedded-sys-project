#ifndef GETTING_THERE_WIFI_SAMPLER_H
#define GETTING_THERE_WIFI_SAMPLER_H
#include <stdbool.h>
#include <stdint.h>
#include "packed_bits.h"

#define SCANS_PER_SAMPLE 5u
#define MIN_VALID_SCANS 3u
#define SCAN_TIMEOUT_MS 15000u
#define WIFI_MOVING_INTERVAL_MS 750u
#define WIFI_RECORD_CAPACITY 64u
// Enable only after measuring motor/STOP timing with the assembled robot.
#ifndef LINE_SEARCH_MOVING_SAMPLES
#define LINE_SEARCH_MOVING_SAMPLES 0
#endif

typedef struct {
    uint16_t code : RSSI_BITS;
    uint16_t spread : 7;
} wifi_sample_t;

void wifi_sampler_init(bool (*can_continue)(void));
void wifi_sampler_set_position(float heading, uint32_t powered_ms, bool forwards);
void wifi_sampler_set_reference(wifi_sample_t reference);
bool wifi_sampler_sample(wifi_sample_t *sample);
// Always stop before quiescing. Failure forbids further BLE or flash work.
bool wifi_sampler_quiesce(void);
bool wifi_sampler_poll_moving(void);
bool wifi_sampler_moving_enabled(void);
uint32_t wifi_sampler_scan_id(void);
wifi_sample_t wifi_filter(const uint8_t *codes, unsigned count);
#endif
