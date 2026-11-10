#include <assert.h>
#include "wifi_sampler.h"

int main(void)
{
    _Static_assert(SCANS_PER_SAMPLE == 9 && MIN_VALID_SCANS == 7,
                   "comparison builds must use the same sampling budget");
    const uint8_t missing[] = {70, 70, 70, 70, 70, 70, 0, 127, 127};
    const uint8_t accepted[] = {70, 70, 70, 70, 70, 70, 70, 0, 127};
    assert(wifi_filter(missing, SCANS_PER_SAMPLE).code == RSSI_MISSING_CODE);
    wifi_sample_t sample = wifi_filter(accepted, SCANS_PER_SAMPLE);
    assert(sample.code == 70 && sample.spread == 0);

#if WIFI_RSSI_FILTER == WIFI_RSSI_FILTER_KALMAN
    const uint8_t late_spike[] = {70, 70, 70, 70, 70, 70, 70, 70, 40};
    const uint8_t early_spike[] = {40, 70, 70, 70, 70, 70, 70, 70, 70};
    sample = wifi_filter(late_spike, SCANS_PER_SAMPLE);
    assert(sample.code == 63 && sample.spread == 7);
    sample = wifi_filter(early_spike, SCANS_PER_SAMPLE);
    assert(sample.code == 68 && sample.spread == 2);
    assert(wifi_filter(accepted, SCANS_PER_SAMPLE).code == 70);
#else
    const uint8_t varying[] = {60, 90, 30, 60, 110, 60, 40, 60, 60};
    sample = wifi_filter(varying, SCANS_PER_SAMPLE);
    assert(sample.code == 61 && sample.spread == 50);
#endif
    return 0;
}
