#include "wifi_sampler.h"
#include "faults.h"
#include "motion.h"
#include "run_report.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <string.h>

#define AP1_SSID "I AM PICO W"
#define QUALIFICATION_WINDOWS 3u
#define SCAN_START_LIMIT_MS 5u

typedef struct {
    uint32_t wall_ms, powered_ms, scan_id;
    float heading;
    uint8_t code;
    bool stopped;
} wifi_record_t;

static wifi_record_t records[WIFI_RECORD_CAPACITY];
static unsigned record_count;
// One callback-owned result per scan. Consume only after scan_active is false.
static volatile wifi_record_t pending_result;
static uint8_t pending_bssid[6], target_bssid[6];
static bool identity_locked, scan_open, moving_scan, fallback, sampler_failed;
static unsigned qualification_windows;
static uint32_t scan_started_ms, scan_id, position_ms, max_start_ms;
static float position_heading;
static bool position_forwards;
static bool (*sampler_can_continue)(void);
static wifi_sample_t moving_reference;
static uint8_t moving_codes[SCANS_PER_SAMPLE];
static unsigned moving_count;
static uint32_t moving_window_started_ms;

static uint32_t sampler_now_ms(void) { return (uint32_t)(time_us_64() / 1000); }
static uint32_t sampler_position(void)
{
    // During callbacks this is an estimate at RECEIPT time, not a hardware
    // timestamp of the radio measurement. Turns and stopped time add no travel.
    uint32_t elapsed = motion_drive_elapsed_ms();
    if (!moving_scan) elapsed = 0;
    return position_forwards ? position_ms + elapsed :
           elapsed < position_ms ? position_ms - elapsed : 0;
}

wifi_sample_t wifi_filter(const uint8_t *codes, unsigned count)
{
    uint8_t sorted[SCANS_PER_SAMPLE];
    unsigned valid = 0;
    if (count > SCANS_PER_SAMPLE) count = SCANS_PER_SAMPLE;
    for (unsigned i = 0; i < count; ++i) {
        if (codes[i] >= RSSI_MISSING_CODE || codes[i] == 0) continue;
        unsigned j = valid++;
        while (j && sorted[j - 1] > codes[i]) { sorted[j] = sorted[j - 1]; --j; }
        sorted[j] = codes[i];
    }
    if (valid < MIN_VALID_SCANS) return (wifi_sample_t){.code = RSSI_MISSING_CODE};
    unsigned trim = valid / 4u, kept = valid - 2u * trim, sum = 0;
    for (unsigned i = trim; i < valid - trim; ++i) sum += sorted[i];
    return (wifi_sample_t){.code = (sum + kept / 2u) / kept,
        .spread = sorted[valid - trim - 1u] - sorted[trim]};
}

static int sampler_result(void *unused, const cyw43_ev_scan_result_t *result)
{
    (void)unused;
    if (!result || result->ssid_len != sizeof(AP1_SSID) - 1u ||
        result->rssi >= 0 || result->rssi <= -127 ||
        memcmp(result->ssid, AP1_SSID, sizeof(AP1_SSID) - 1u) != 0 ||
        (identity_locked && memcmp(result->bssid, target_bssid, 6) != 0)) return 0;
    uint8_t code = (uint8_t)rssi_encode(result->rssi);
    if (code >= pending_result.code) return 0;
    pending_result.wall_ms = sampler_now_ms();
    pending_result.powered_ms = sampler_position();
    pending_result.stopped = !motion_drive_active();
    pending_result.code = code;
    memcpy(pending_bssid, result->bssid, sizeof(pending_bssid));
    return 0;
}

void wifi_sampler_init(bool (*can_continue)(void))
{
    sampler_can_continue = can_continue;
    record_count = qualification_windows = moving_count = 0;
    scan_id = position_ms = max_start_ms = 0;
    position_heading = 0;
    position_forwards = true;
    identity_locked = scan_open = moving_scan = fallback = sampler_failed = false;
    moving_reference = (wifi_sample_t){.code = RSSI_MISSING_CODE};
    report_printf("RSSI sampling: stationary decisions; moving acquisition %s, qualification requires 3 full five-scan windows <= 750 ms\n",
                  LINE_SEARCH_MOVING_SAMPLES ? "awaiting timing qualification" : "disabled pending hardware timing checks");
}

void wifi_sampler_set_position(float heading, uint32_t powered_ms, bool forwards)
{
    // Caller changes the frame only after quiescing the previous scan.
    position_heading = heading;
    position_ms = powered_ms;
    position_forwards = forwards;
    moving_count = 0;
    moving_window_started_ms = sampler_now_ms();
}

void wifi_sampler_set_reference(wifi_sample_t reference) { moving_reference = reference; }
uint32_t wifi_sampler_scan_id(void) { return scan_id; }
bool wifi_sampler_moving_enabled(void)
{
    return LINE_SEARCH_MOVING_SAMPLES && !fallback && qualification_windows >= QUALIFICATION_WINDOWS;
}

static bool sampler_begin(bool moving)
{
    if (sampler_failed || scan_open || !sampler_can_continue()) return false;
    if (record_count >= WIFI_RECORD_CAPACITY) {
        motion_stop();
        fault_raise(FAULT_PROFILE);
        sampler_failed = true;
        return false;
    }
    pending_result = (wifi_record_t){.code = RSSI_MISSING_CODE,
        .scan_id = ++scan_id, .heading = position_heading};
    scan_started_ms = sampler_now_ms();
    scan_open = true;
    moving_scan = moving;
    if (moving && moving_count == 0) moving_window_started_ms = scan_started_ms;
    cyw43_wifi_scan_options_t options = {0};
    int error = cyw43_wifi_scan(&cyw43_state, &options, NULL, sampler_result);
    uint32_t elapsed = sampler_now_ms() - scan_started_ms;
    if (elapsed > max_start_ms) max_start_ms = elapsed;
    if (error) {
        scan_open = false;
        sampler_failed = true;
        motion_stop();
        fault_raise(FAULT_WIFI_SCAN);
        report_printf("FAILED: Wi-Fi scan could not start, error %d\n", error);
        return false;
    }
    if (moving && elapsed > SCAN_START_LIMIT_MS) {
        fallback = true;
        motion_stop();
    }
    return true;
}

static bool sampler_collect(void)
{
    if (!scan_open || cyw43_wifi_scan_active(&cyw43_state)) return true;
    wifi_record_t record = pending_result;
    if (record.code == RSSI_MISSING_CODE) {
        record.wall_ms = sampler_now_ms();
        record.powered_ms = sampler_position();
        record.stopped = !motion_drive_active();
    } else if (!identity_locked) {
        memcpy(target_bssid, pending_bssid, sizeof(target_bssid));
        identity_locked = true;
    }
    if (record_count >= WIFI_RECORD_CAPACITY) {
        motion_stop();
        fault_raise(FAULT_PROFILE);
        sampler_failed = true;
        return false;
    }
    records[record_count++] = record;
    scan_open = false;
    if (moving_scan) {
        if (record.code == RSSI_MISSING_CODE ||
            sampler_now_ms() - moving_window_started_ms > WIFI_MOVING_INTERVAL_MS) {
            fallback = true;
            motion_stop();
        } else if (record.stopped) {
            // A scheduled BLE pause may finish the last scan while stationary.
            // Keep it in the trace, but exclude it from the moving window.
            moving_count = 0;
        } else {
            moving_codes[moving_count++] = record.code;
            if (moving_count == SCANS_PER_SAMPLE) {
                wifi_sample_t sample = wifi_filter(moving_codes, moving_count);
                unsigned margin = moving_reference.spread + sample.spread + 1u;
                if (margin < 3u) margin = 3u;
                if (moving_reference.code != RSSI_MISSING_CODE &&
                    (int)sample.code - (int)moving_reference.code >= (int)margin)
                    motion_stop(); // hint only; two stationary batches decide
                moving_count = 0;
                moving_window_started_ms = sampler_now_ms();
            }
        }
    }
    return true;
}

static bool sampler_flush(void)
{
    if (motion_drive_active() || scan_open || cyw43_wifi_scan_active(&cyw43_state)) return false;
    for (unsigned i = 0; i < record_count; ++i) {
        const wifi_record_t *r = &records[i];
        report_printf("RSSI_RAW,scan=%lu,t_ms=%lu,powered_ms=%lu,heading=%d,dbm=%d,stopped=%u\n",
                      (unsigned long)r->scan_id, (unsigned long)r->wall_ms,
                      (unsigned long)r->powered_ms, (int)r->heading,
                      rssi_decode(r->code), r->stopped ? 1u : 0u);
    }
    record_count = 0;
    return report_ok();
}

bool wifi_sampler_quiesce(void)
{
    motion_stop();
    if (sampler_failed) return false;
    while (scan_open && cyw43_wifi_scan_active(&cyw43_state)) {
        if (!sampler_can_continue() || sampler_now_ms() - scan_started_ms >= SCAN_TIMEOUT_MS) {
            fault_raise(FAULT_WIFI_SCAN);
            sampler_failed = true;
            report_printf("FAILED: Wi-Fi scan stopped or timed out; BLE/flash forbidden until radio shutdown\n");
            return false;
        }
        sleep_ms(5);
    }
    if (!sampler_collect()) return false;
    return sampler_flush();
}

bool wifi_sampler_poll_moving(void)
{
    if (!sampler_can_continue()) { motion_stop(); return false; }
    if (!wifi_sampler_moving_enabled()) return true;
    if (scan_open && sampler_now_ms() - scan_started_ms >= WIFI_MOVING_INTERVAL_MS) {
        fallback = true;
        motion_stop();
        return true;
    }
    if (!sampler_collect()) return false;
    if (motion_drive_active() && wifi_sampler_moving_enabled() && !scan_open)
        return sampler_begin(true);
    return true;
}

bool wifi_sampler_sample(wifi_sample_t *sample)
{
    if (!wifi_sampler_quiesce()) return false;
    report_printf("AP1 Wi-Fi sample: collecting 5 fresh scans (3 required); motors stopped\n");
    uint8_t readings[SCANS_PER_SAMPLE];
    unsigned valid = 0;
    uint32_t start = sampler_now_ms();
    for (unsigned i = 0; i < SCANS_PER_SAMPLE; ++i) {
        if (!sampler_begin(false) || !wifi_sampler_quiesce()) return false;
        readings[i] = pending_result.code;
        if (readings[i] != RSSI_MISSING_CODE) ++valid;
    }
    *sample = wifi_filter(readings, SCANS_PER_SAMPLE);
    uint32_t elapsed = sampler_now_ms() - start;
    if (!fallback && valid == SCANS_PER_SAMPLE && elapsed <= WIFI_MOVING_INTERVAL_MS &&
        max_start_ms <= SCAN_START_LIMIT_MS) {
        if (qualification_windows < QUALIFICATION_WINDOWS) ++qualification_windows;
    } else {
        qualification_windows = 0;
        if (LINE_SEARCH_MOVING_SAMPLES) fallback = true;
    }
    if (sample->code == RSSI_MISSING_CODE) fault_raise(FAULT_WIFI_LOST);
    else fault_clear(FAULT_WIFI_LOST);
    report_printf("RSSI_SAMPLE,scan=%lu,valid=%u,dbm=%d,spread=%u,duration_ms=%lu,moving=%u,fallback=%u\n",
                  (unsigned long)scan_id, valid, rssi_decode(sample->code), (unsigned)sample->spread,
                  (unsigned long)elapsed, wifi_sampler_moving_enabled() ? 1u : 0u, fallback ? 1u : 0u);
    return sampler_can_continue() && report_ok();
}
