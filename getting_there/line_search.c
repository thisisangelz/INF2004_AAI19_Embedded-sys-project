#include "line_search.h"
#include <math.h>

float line_normalise_heading(float heading)
{
    heading = fmodf(heading, 360.0f);
    return heading < 0 ? heading + 360.0f : heading;
}

int line_progress_margin(wifi_sample_t previous, wifi_sample_t current)
{
    int margin = previous.spread + current.spread + 1;
    return margin > MIN_PROGRESS_DB ? margin : MIN_PROGRESS_DB;
}

static bool valid_pair(line_pair_t pair)
{
    return pair.first.code != RSSI_MISSING_CODE && pair.second.code != RSSI_MISSING_CODE;
}

wifi_sample_t line_reference(line_pair_t pair)
{
    if (!valid_pair(pair)) return (wifi_sample_t){.code = RSSI_MISSING_CODE};
    unsigned low = pair.first.code < pair.second.code ? pair.first.code : pair.second.code;
    unsigned high = pair.first.code > pair.second.code ? pair.first.code : pair.second.code;
    unsigned spread = pair.first.spread > pair.second.spread ? pair.first.spread : pair.second.spread;
    spread += high - low;
    return (wifi_sample_t){.code = high, .spread = spread > 125u ? 125u : spread};
}

int line_confirmed_gain(line_pair_t previous, line_pair_t current)
{
    if (!valid_pair(previous) || !valid_pair(current)) return 0;
    // Compare the strongest old batch against the weakest new batch. Neither a
    // single strong batch nor two overlapping rolling averages establish a peak.
    int old_best = previous.first.code < previous.second.code ? previous.first.code : previous.second.code;
    int new_worst = current.first.code > current.second.code ? current.first.code : current.second.code;
    return old_best - new_worst;
}

int line_compare(line_pair_t previous, line_pair_t current)
{
    if (!valid_pair(previous) || !valid_pair(current)) return 0;
    int margin = line_progress_margin(line_reference(previous), line_reference(current));
    if (line_confirmed_gain(previous, current) >= margin) return 1;
    if (line_confirmed_gain(current, previous) >= margin) return -1;
    return 0;
}

bool line_return_consistent(line_pair_t best, line_pair_t returned)
{
    if (!valid_pair(best) || !valid_pair(returned)) return false;
    int difference = (int)returned.first.code - (int)returned.second.code;
    if (difference < 0) difference = -difference;
    if (difference >= line_progress_margin(returned.first, returned.second)) return false;
    wifi_sample_t reference = line_reference(best);
    return (int)returned.first.code - (int)reference.code < line_progress_margin(reference, returned.first) &&
           (int)returned.second.code - (int)reference.code < line_progress_margin(reference, returned.second);
}
