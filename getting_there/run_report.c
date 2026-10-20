#include "run_report.h"
#include "report_store.h"
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "pico/critical_section.h"
#include "pico/stdio_usb.h"

#define REPORT_CAPACITY 32768
static char pending[REPORT_CAPACITY];
static char snapshot[REPORT_CAPACITY];
static uint16_t pending_length; // 0..32768
static volatile struct {
    uint8_t active : 1;
    uint8_t failed : 1;
    uint8_t closed : 1;
} report_state;
static critical_section_t report_lock;

void report_init(void)
{
    critical_section_init(&report_lock);
    pending_length = 0;
    report_state.active = report_state.failed = report_state.closed = false;
}

static void report_failed(void)
{
    critical_section_enter_blocking(&report_lock);
    report_state.failed = true;
    critical_section_exit(&report_lock);
}

void report_printf(const char *format, ...)
{
    char line[512];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (written <= 0) return;
    size_t length = (size_t)written;
    if (length >= sizeof(line)) length = sizeof(line) - 1;
    critical_section_enter_blocking(&report_lock);
    if (!report_state.closed) {
        // Never silently drop old lines. Stop at the next safe run boundary.
        if ((size_t)written >= sizeof(line) || pending_length + length > sizeof(pending))
            report_state.failed = true;
        else if (!report_state.failed) {
            memcpy(pending + pending_length, line, length);
            pending_length += length;
        }
    }
    critical_section_exit(&report_lock);
    if (stdio_usb_connected()) printf("%s", line);
}

bool report_ok(void) { return !report_state.failed; }

bool report_begin(void)
{
    // Keep boot/config diagnostics. Call before Wi-Fi/BLE or movement starts.
    bool active = report_store_begin();
    critical_section_enter_blocking(&report_lock);
    report_state.active = active;
    if (!active) report_state.failed = true;
    critical_section_exit(&report_lock);
    return report_checkpoint();
}

bool report_checkpoint(void)
{
    if (!report_state.active) return !report_state.failed;
    // Foreground only: motors OFF, outside radio operations. Never in callbacks.
    critical_section_enter_blocking(&report_lock);
    size_t length = pending_length;
    memcpy(snapshot, pending, length);
    pending_length = 0;
    critical_section_exit(&report_lock);
    if (length && !report_store_append(snapshot, length)) report_failed();
    return !report_state.failed;
}

bool report_finish(void)
{
    critical_section_enter_blocking(&report_lock);
    report_state.closed = true;
    critical_section_exit(&report_lock);
    bool ok = report_checkpoint();
    if (!ok || !report_store_finish()) report_failed();
    critical_section_enter_blocking(&report_lock);
    report_state.active = false;
    critical_section_exit(&report_lock);
    return !report_state.failed;
}

static void usb_emit(const char *data, size_t length)
{
    fwrite(data, 1, length, stdout);
}

void report_replay(void)
{
    if (!stdio_usb_connected()) return;
    // Reading/replaying flash never gets appended to the new run's report.
    report_store_replay(usb_emit);
    printf("Before a run: GP21 replays; GP20 starts a new run and replaces this log.\n");
}
