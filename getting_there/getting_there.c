#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "colour_sensor.h"
#include "faults.h"
#include "motion.h"
#include "payload_client.h"
#include "robot_cues.h"
#include "run_report.h"
#include "target_beacon.h"

#include "robot_run.h"

#define START_RELEASE_TIMEOUT_MS 10000u
#define BLUETOOTH_START_TIMEOUT_MS 10000u

static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }

static void report_faults(void)
{
    report_printf("FAULT MASK: 0x%03x\n", fault_mask());
    for (int f = 0; f < FAULT_COUNT; ++f)
        if (fault_test((fault_t)f)) report_printf("FAULT: %s\n", fault_name((fault_t)f));
}

static bool wait_start_release(void)
{
    report_printf("Release GP20 to continue. Waiting at most 10 seconds; GP21 stops startup.\n");
    uint32_t start = now_ms();
    while (motion_start_pressed()) {
        if (motion_stop_requested()) return false;
        if ((uint32_t)(now_ms() - start) >= START_RELEASE_TIMEOUT_MS) {
            fault_raise(FAULT_START_BUTTON);
            report_printf("FAILED: GP20 still reads pressed after 10 seconds; release the button/check GP20 wiring, then reset\n");
            return false;
        }
        sleep_ms(10);
    }
    return !motion_stop_requested();
}

int main(void)
{
    stdio_init_all();
    report_init();
    bool ready = motion_init();
    cues_init();
    sleep_ms(2000);
    if (robot_run_next_beacon())
        report_printf("Getting there | line-search v18 steps500-mat250 back-search side-probe early-probe straight-first mat-steer colour-gate own-mat ble-minus58 cues route | %s -> %s%s\n",
                      target_name(), target_name_of(robot_run_next_beacon()),
                      robot_run_starts_on_mat() ? " (test: start on the first mat)" : "");
    else
        report_printf("Getting there | line-search v18 steps500-mat250 back-search side-probe early-probe straight-first mat-steer colour-gate own-mat ble-minus58 cues | %s only\n",
                      target_name());
#if COLOUR_SCAN_TEST
    report_printf("TEST BUILD: colour scan on the spot after calibration; no radio, no approach\n");
    report_printf("Colour mats: pink, blue and purple all recognised\n");
#else
    // Each beacon has its own mat; the other two read as floor.
    colour_set_target_mat(target_mat());
    report_printf("Colour mat: %s (%s); other mats read as floor\n",
                  colour_mat_name(target_mat()), target_name());
    if (robot_run_next_beacon())
        report_printf("Route: %s mat at %s, then %s mat at %s; each beacon has its own 15 minute limit and travel budget\n",
                      colour_mat_name(target_mat()), target_name(),
                      colour_mat_name(target_mat_of(robot_run_next_beacon())), target_name_of(robot_run_next_beacon()));
#endif
    report_printf("Flash layout: 2 MiB physical; 1528 KiB firmware, 8 KiB Bluetooth, 512 KiB saved log\n");
    report_printf("Before GP20: GP21 replays previous run. After GP20: GP21 STOP, latched until reset.\n");
    report_printf("Move using Wi-Fi direction; on the mat, steer by colour scans\n");
    report_printf("Transfer only on %s or darker after a lighter ring, with BLE >= %d dBm\n",
                  COLOUR_ARRIVAL_TEXT, COLOUR_ARRIVAL_BLE_DBM);
    report_printf("Without a working colour sensor: transfer after BLE averages >= %d dBm\n", AP1_CLOSE_RSSI_DBM);
    // GP21 is not armed as STOP here. Boot/replay must not erase the old run,
    // start movement, initialise the radio, or prevent the later GP20 start.
    bool last_replay = false;
    report_printf("Waiting for GP20: press to start; GP21 replays the previous run.\n");
    while (true) {
        report_flush_usb();
        bool replay = motion_replay_pressed();
        if (replay && !last_replay) report_replay();
        last_replay = replay;
        // Give replay precedence if both buttons are pressed.
        if (!replay && motion_start_pressed()) break;
        sleep_ms(20);
    }
    motion_arm_stop();
    bool recording = false;
    if (!motion_stop_requested()) {
        cue_play(CUE_START);
        report_printf("GP20 START: preparing persistent log; motors remain stopped.\n");
        recording = report_begin();
        if (!recording) fault_raise(FAULT_REPORT);
    } else report_printf("START CANCELLED: GP21 was pressed; previous log preserved\n");
    bool released = recording && wait_start_release();
    if (released) report_printf("GP20 released. Persistent log ready.\n");
    // An early power-up I2C failure must not permanently reject a sensor which
    // is now ready. Retry once, stopped and recorded, after the explicit start.
    if (released && !ready) {
        report_printf("Retrying IMU initialisation after GP20; motors remain stopped.\n");
        ready = motion_init();
    }
    ready = ready && released && !motion_stop_requested();
    // Calibrate immediately after the new run is safely logged. Radio startup
    // can take several seconds; it should not make the requested calibration
    // appear as if GP20 did nothing.
    // Optional: without the sensor the robot keeps the BLE-only proximity gate.
    if (ready) colour_sensor_init();
    if (ready) {
        report_printf("Starting gyro calibration before Wi-Fi/Bluetooth. Keep the robot still.\n");
        // A steady tone for as long as the robot must stay still.
        cue_play(CUE_CALIBRATING);
        ready = motion_calibrate();
        cue_stop();
        if (!ready) report_printf("FAILED: gyro calibration; check IMU wiring and keep the robot still\n");
    }
    bool radio_initialised = false;
#if COLOUR_SCAN_TEST
    // Test build: no radio and no approach; scan the colour on the spot.
    if (ready && !motion_stop_requested()) {
        bool scanned = robot_run_colour_scan();
        motion_stop();
        if (scanned) robot_run_completion_cue();
        else cue_play(CUE_FAULT);
        report_printf("RESULT: %s\n", scanned ? "COLOUR SCAN COMPLETE" : "COLOUR SCAN FAILED");
    } else {
        cue_play(CUE_FAULT);
        report_printf("RESULT: STARTUP FAILED OR STOPPED\n");
    }
#else
    if (ready) {
        report_printf("Starting Wi-Fi/Bluetooth hardware.\n");
        int radio_error = cyw43_arch_init();
        if (radio_error) {
            fault_raise(FAULT_BLE_RADIO);
            report_printf("FAILED: Wi-Fi/Bluetooth initialisation, error %d\n", radio_error);
            ready = false;
        } else {
            radio_initialised = true;
            report_printf("Wi-Fi hardware initialised; starting Bluetooth.\n");
            cyw43_arch_enable_sta_mode();
            payload_client_set_close_callback(robot_run_in_range);
            payload_client_init();
            uint32_t start = now_ms();
            uint32_t last_progress = start;
            while (!payload_client_ready() && !payload_client_start_failed() && !motion_stop_requested() &&
                   (uint32_t)(now_ms() - start) < BLUETOOTH_START_TIMEOUT_MS) {
                report_flush_usb();
                if ((uint32_t)(now_ms() - last_progress) >= 1000u) {
                    report_printf("Waiting for Bluetooth: %lu/10000 ms; motors stopped\n",
                                  (unsigned long)(now_ms() - start));
                    last_progress = now_ms();
                }
                sleep_ms(10);
            }
            ready = payload_client_ready() && !payload_client_start_failed() && !motion_stop_requested();
            if (!ready) {
                if (!motion_stop_requested()) fault_raise(FAULT_BLE_RADIO);
                report_printf("FAILED: Bluetooth startup %s\n", motion_stop_requested() ? "stopped by GP21" :
                              payload_client_start_failed() ? "controller power-on failed" : "timed out after 10 seconds");
            }
            else report_printf("Bluetooth ready; beginning %s approach.\n", target_name());
        }
    }
    // approach_beacon (or the route test start) initialises the sampler
    // before its first checkpoint.
    if (ready && !motion_stop_requested()) {
        bool completed = robot_run_execute();
        motion_stop();
        if (completed) robot_run_completion_cue();
        else cue_play(CUE_FAULT);
        if (!robot_run_next_beacon())
            report_printf("RESULT: %s\n", completed ? "PAYLOAD CONFIRMED" : "STOPPED WITHOUT PAYLOAD");
        else if (completed && robot_run_starts_on_mat())
            report_printf("RESULT: ROUTE COMPLETE, %s payload confirmed (test start on the %s mat)\n",
                          target_name(), target_name_of(TARGET_BEACON_ID));
        else if (completed)
            report_printf("RESULT: ROUTE COMPLETE, %s and %s payloads confirmed\n",
                          target_name_of(TARGET_BEACON_ID), target_name());
        else
            report_printf("RESULT: ROUTE STOPPED before the %s payload\n", target_name());
    } else {
        cue_play(CUE_FAULT);
        report_printf("RESULT: STARTUP FAILED OR STOPPED\n");
    }
#endif
    motion_stop();
    // Also quiesces any scan/cleanup still pending after a fatal radio failure.
    if (radio_initialised) cyw43_arch_deinit();
    report_faults();
    if (motion_stop_requested()) report_printf("GP21 stop latched\n");
    report_printf("Run ended. Log survives power-off. Next boot: open serial, press GP21 BEFORE GP20.\n");
    if (recording && !report_finish()) {
        fault_raise(FAULT_REPORT);
        printf("LOG ERROR: saved report is incomplete; no further movement.\n");
    }
    // Never restart automatically. Allow GP21 replay after this run as well.
    last_replay = motion_replay_pressed();
    while (true) {
        report_flush_usb();
        bool replay = motion_replay_pressed();
        if (replay && !last_replay) report_replay();
        last_replay = replay;
        sleep_ms(20);
    }
}
