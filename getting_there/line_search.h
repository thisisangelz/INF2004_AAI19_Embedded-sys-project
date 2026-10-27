#ifndef GETTING_THERE_LINE_SEARCH_H
#define GETTING_THERE_LINE_SEARCH_H
#include "wifi_sampler.h"

#define LINE_BLOCK_MS 750u
#define LINE_FORWARD_BLOCK_MS 250u
#define LINE_DECLINE_POSITIONS 3u
#define LINE_DIRECTION_TEST_MS 2400u
#define LINE_SWEEP_MS 6000u
#define LINE_TRANSLATION_MS 60000u
#define LINE_PROFILE_CAPACITY 32u
#define MIN_PROGRESS_DB 3

typedef struct { wifi_sample_t first, second; } line_pair_t;
float line_normalise_heading(float heading);
int line_progress_margin(wifi_sample_t previous, wifi_sample_t current);
int line_compare(line_pair_t previous, line_pair_t current);
int line_confirmed_gain(line_pair_t previous, line_pair_t current);
bool line_return_consistent(line_pair_t best, line_pair_t returned);
wifi_sample_t line_reference(line_pair_t pair);
#endif
