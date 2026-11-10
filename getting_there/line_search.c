#include "line_search.h"
#include <math.h>

float line_normalise_heading(float heading)
{
    heading = fmodf(heading, 360.0f);
    return heading < 0 ? heading + 360.0f : heading;
}

// Preserve the existing tolerance for checking an approximate timed return.
static int return_margin(wifi_sample_t previous, wifi_sample_t current)
{
    int margin = previous.spread + current.spread + 1;
    return margin > MIN_RETURN_DB ? margin : MIN_RETURN_DB;
}

int line_pair_margin(line_pair_t previous, line_pair_t current)
{
    // The strongest-old/weakest-new comparison already accounts for disagreement
    // between the two batches. Use the largest retained within-batch range once,
    // plus 1 dB for rounding, instead of adding the same uncertainty again.
    unsigned spread = previous.first.spread;
    if (previous.second.spread > spread) spread = previous.second.spread;
    if (current.first.spread > spread) spread = current.first.spread;
    if (current.second.spread > spread) spread = current.second.spread;
    int margin = (int)spread + 1;
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
    int margin = line_pair_margin(previous, current);
    if (line_confirmed_gain(previous, current) >= margin) return 1;
    if (line_confirmed_gain(current, previous) >= margin) return -1;
    return 0;
}

line_pair_t line_median_reference(const line_pair_t samples[LINE_REFERENCE_POSITIONS])
{
    _Static_assert(LINE_REFERENCE_POSITIONS == 3u, "Median reference requires three positions");
    line_pair_t sorted[LINE_REFERENCE_POSITIONS];
    for (unsigned i = 0; i < LINE_REFERENCE_POSITIONS; ++i) {
        if (!valid_pair(samples[i]))
            return (line_pair_t){.first.code = RSSI_MISSING_CODE, .second.code = RSSI_MISSING_CODE};
        sorted[i] = samples[i];
        for (unsigned j = i; j && line_reference(sorted[j]).code < line_reference(sorted[j - 1]).code; --j) {
            line_pair_t swap = sorted[j]; sorted[j] = sorted[j - 1]; sorted[j - 1] = swap;
        }
    }
    // Keep both batches from the actual median-strength position, not a mix
    // of batches from different places. The weaker batch ranks each position.
    return sorted[1];
}

bool line_reference_declined(line_pair_t reference, line_pair_t current)
{
    // Equality remains inside the recovery tolerance. Both current batches
    // must be strictly weaker than the reference by more than the noise margin.
    int margin = line_pair_margin(reference, current);
    if (margin < MIN_DECLINE_DB) margin = MIN_DECLINE_DB;
    return valid_pair(reference) && valid_pair(current) &&
           line_confirmed_gain(current, reference) > margin;
}

bool line_return_consistent(line_pair_t best, line_pair_t returned)
{
    if (!valid_pair(best) || !valid_pair(returned)) return false;
    int difference = (int)returned.first.code - (int)returned.second.code;
    if (difference < 0) difference = -difference;
    // Accept equality at the tolerance for batch agreement and loss from best.
    if (difference > return_margin(returned.first, returned.second)) return false;
    wifi_sample_t reference = line_reference(best);
    return (int)returned.first.code - (int)reference.code <= return_margin(reference, returned.first) &&
           (int)returned.second.code - (int)reference.code <= return_margin(reference, returned.second);
}
