#ifndef GETTING_THERE_LINE_SEARCH_H
#define GETTING_THERE_LINE_SEARCH_H
#include "wifi_sampler.h"
#include "target_beacon.h"

#define LINE_BLOCK_MS 750u
// Two-beacon floor trials cover more distance between stationary readings.
#if ROUTE_BEACON_COUNT == 2
#define LINE_FORWARD_BLOCK_MS 750u
#else
#define LINE_FORWARD_BLOCK_MS 500u
#endif
#define LINE_MAT_BLOCK_MS 250u
// Near the centre: 100 ms steps once a stop is within two rings of the mat's
// arrival ring. The blue and purple mats fit six rings where pink has five,
// so their inner rings are narrower. With 250 ms steps on 7 November the robot
// drove into the beacon box on the blue mat and nearly on the purple one; in
// the host model a 250 ms step is wider than a blue ring. 100 ms matches the
// v16 dip probes, so the motors are known to handle it.
#define LINE_MAT_NEAR_BLOCK_MS 100u
#define LINE_MAT_NEAR_RINGS 2u
#define LINE_DECLINE_POSITIONS 3u
#define LINE_REFERENCE_POSITIONS 3u
// v16: 100 ms. Each probe now looks a full mat step further past a weak patch.
#define LINE_DIP_PROBE_MS 250u
#define LINE_DIP_MAX_PROBES 2u
#define LINE_DIRECTION_TEST_MS 2400u
#define LINE_DIRECTION_EXTENDED_MS 4800u
#define LINE_NO_BEST_MS 6000u
#define LINE_NO_BEST_WALL_MS 120000u
#define LINE_SWEEP_MS 6000u
#define LINE_TRANSLATION_MS 60000u
#define LINE_PROFILE_CAPACITY 32u
#if ROUTE_BEACON_COUNT == 2
#define MIN_PROGRESS_DB 2
#else
#define MIN_PROGRESS_DB 3
#endif
#define MIN_RETURN_DB 3
#define MIN_DECLINE_DB 3

typedef struct { wifi_sample_t first, second; } line_pair_t;
// Cruise evidence: the newest 2 x LINE_MOVING_HALF_SCANS valid scans of one
// heading, oldest first.
#define LINE_MOVING_HALF_SCANS 3u
float line_normalise_heading(float heading);
int line_pair_margin(line_pair_t previous, line_pair_t current);
int line_compare(line_pair_t previous, line_pair_t current);
int line_confirmed_gain(line_pair_t previous, line_pair_t current);
line_pair_t line_median_reference(const line_pair_t samples[LINE_REFERENCE_POSITIONS]);
bool line_reference_declined(line_pair_t reference, line_pair_t current);
bool line_return_consistent(line_pair_t best, line_pair_t returned);
wifi_sample_t line_reference(line_pair_t pair);
line_pair_t line_moving_pair(const uint8_t codes[2u * LINE_MOVING_HALF_SCANS]);
#endif
