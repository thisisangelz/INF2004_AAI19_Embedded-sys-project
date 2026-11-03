#ifndef GETTING_THERE_LINE_SEARCH_H
#define GETTING_THERE_LINE_SEARCH_H
#include "wifi_sampler.h"

#define LINE_BLOCK_MS 750u
#define LINE_FORWARD_BLOCK_MS 500u
#define LINE_FAST_BLOCK_MS 750u
#define LINE_NEAR_BLOCK_MS 250u
#define LINE_DECLINE_POSITIONS 3u
#define LINE_DECLINE_WINDOW 4u
#define LINE_REFERENCE_POSITIONS 3u
#define LINE_EVIDENCE_SPACING_MS 250u
#define LINE_EXPLORE_MS 2000u
#define LINE_REGION_DB 2
#define LINE_STABLE_SPREAD_DB 3u
#define LINE_FINE_STEP_MS 125u
#define LINE_FINE_RADIUS_MS 250u
#define LINE_FINE_MAX_STEPS 8u
#define LINE_DIRECTION_TEST_MS 2400u
#define LINE_SWEEP_MS 6000u
#define LINE_TRANSLATION_MS 60000u
#define LINE_PROFILE_CAPACITY 32u
#define MIN_PROGRESS_DB 3

typedef struct { wifi_sample_t first, second; } line_pair_t;
float line_normalise_heading(float heading);
int line_pair_margin(line_pair_t previous, line_pair_t current);
int line_compare(line_pair_t previous, line_pair_t current);
int line_confirmed_gain(line_pair_t previous, line_pair_t current);
line_pair_t line_median_reference(const line_pair_t samples[LINE_REFERENCE_POSITIONS]);
bool line_reference_declined(line_pair_t reference, line_pair_t current);
bool line_return_consistent(line_pair_t best, line_pair_t returned);
wifi_sample_t line_reference(line_pair_t pair);
bool line_pair_stable(line_pair_t pair);
bool line_region_matches(line_pair_t reference, line_pair_t current);
typedef struct {
    line_pair_t baseline, reference, region_readings;
    line_pair_t recent[LINE_REFERENCE_POSITIONS];
    uint32_t recent_positions[LINE_REFERENCE_POSITIONS];
    uint32_t region_position, last_evidence_position;
    unsigned recent_count, recent_next, weak_bits, good_count;
    bool dip_active, progress;
} line_track_t;
void line_track_init(line_track_t *track, line_pair_t baseline);
void line_track_observe(line_track_t *track, uint32_t position, line_pair_t current);
unsigned line_track_weak_count(const line_track_t *track);
bool line_track_reassess(const line_track_t *track, uint32_t position);
uint32_t line_track_step(const line_track_t *track, uint32_t position, bool nearby);
bool line_track_recovered(line_track_t *track, uint32_t position, line_pair_t first, line_pair_t second);
#endif
