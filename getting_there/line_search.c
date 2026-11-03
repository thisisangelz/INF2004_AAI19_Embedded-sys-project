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
    return margin > MIN_PROGRESS_DB ? margin : MIN_PROGRESS_DB;
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
    return valid_pair(reference) && valid_pair(current) &&
           line_confirmed_gain(current, reference) > line_pair_margin(reference, current);
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

bool line_pair_stable(line_pair_t pair)
{
    int difference = (int)pair.first.code - (int)pair.second.code;
    return valid_pair(pair) && pair.first.spread <= LINE_STABLE_SPREAD_DB &&
           pair.second.spread <= LINE_STABLE_SPREAD_DB &&
           difference >= -LINE_REGION_DB && difference <= LINE_REGION_DB;
}

bool line_region_matches(line_pair_t reference, line_pair_t current)
{
    if (!valid_pair(reference) || !line_pair_stable(current)) return false;
    // A fixed loss tolerance: noise cannot widen the definition of recovery.
    int weakest = line_reference(reference).code + LINE_REGION_DB;
    return current.first.code <= weakest && current.second.code <= weakest;
}

void line_track_init(line_track_t *track, line_pair_t baseline)
{
    *track = (line_track_t){.baseline = baseline, .reference = baseline,
        .region_readings = baseline, .recent = {baseline}, .recent_count = 1,
        .recent_next = 1};
}

unsigned line_track_weak_count(const line_track_t *track)
{
    unsigned count = 0;
    for (unsigned bits = track->weak_bits; bits; bits >>= 1) count += bits & 1u;
    return count;
}

void line_track_observe(line_track_t *track, uint32_t position, line_pair_t current)
{
    // Early motor stops cannot turn nearby observations into independent positions.
    if (position - track->last_evidence_position < LINE_EVIDENCE_SPACING_MS) return;
    track->last_evidence_position = position;
    bool weak = line_reference_declined(track->reference, current);
    track->weak_bits = ((track->weak_bits << 1) | (unsigned)weak) & ((1u << LINE_DECLINE_WINDOW) - 1u);
    bool good = line_region_matches(track->reference, current);
    track->good_count = good ? (track->good_count < 2 ? track->good_count + 1 : 2) : 0;
    if (!good) track->dip_active = true;
    if (line_pair_stable(current) && line_compare(track->baseline, current) > 0) track->progress = true;
    if (track->good_count >= 2) {
        track->dip_active = false;
        track->weak_bits = 0;
        track->region_position = position;
        track->region_readings = current;
    }
    if (!track->dip_active && line_pair_stable(current)) {
        track->recent[track->recent_next] = current;
        track->recent_positions[track->recent_next] = position;
        track->recent_next = (track->recent_next + 1) % LINE_REFERENCE_POSITIONS;
        if (track->recent_count < LINE_REFERENCE_POSITIONS) ++track->recent_count;
        if (track->recent_count == LINE_REFERENCE_POSITIONS) {
            line_pair_t candidate = line_median_reference(track->recent);
            if (line_pair_stable(candidate) && line_confirmed_gain(track->reference, candidate) > 0) {
                track->reference = candidate; // Never follow a weakening signal down.
                // Select the latest supported part of the strong region, even
                // if it is slightly weaker than an older isolated maximum.
                unsigned matches = 0, latest = 0;
                for (unsigned i = 0; i < LINE_REFERENCE_POSITIONS; ++i) {
                    if (!line_region_matches(candidate, track->recent[i])) continue;
                    if (!matches || track->recent_positions[i] > track->recent_positions[latest]) latest = i;
                    ++matches;
                }
                if (matches >= 2) {
                    track->region_position = track->recent_positions[latest];
                    track->region_readings = track->recent[latest];
                }
            }
        }
    }
}

bool line_track_reassess(const line_track_t *track, uint32_t position)
{
    return line_track_weak_count(track) >= LINE_DECLINE_POSITIONS ||
           position - track->region_position >= LINE_EXPLORE_MS;
}

uint32_t line_track_step(const line_track_t *track, uint32_t position, bool nearby)
{
    uint32_t travelled = position - track->region_position;
    if (travelled >= LINE_EXPLORE_MS) return 0;
    uint32_t step = nearby ? LINE_NEAR_BLOCK_MS :
        track->progress && !track->dip_active ? LINE_FAST_BLOCK_MS : LINE_FORWARD_BLOCK_MS;
    uint32_t remaining = LINE_EXPLORE_MS - travelled;
    return step < remaining ? step : remaining;
}

bool line_track_recovered(line_track_t *track, uint32_t position, line_pair_t first, line_pair_t second)
{
    if (!line_region_matches(track->reference, first) || !line_region_matches(track->reference, second)) return false;
    track->weak_bits = 0;
    track->good_count = 2;
    track->dip_active = false;
    track->region_position = track->last_evidence_position = position;
    track->region_readings = second;
    // Two stopped pairs confirm this point but are not two spatial positions.
    track->recent[0] = second;
    track->recent_positions[0] = position;
    track->recent_count = track->recent_next = 1;
    if (line_compare(track->baseline, second) > 0) track->progress = true;
    return true;
}
