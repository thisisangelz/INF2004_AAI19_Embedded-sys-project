#include "run_report.h"
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "pico/critical_section.h"
#include "pico/stdio_usb.h"

#define REPORT_CAPACITY 32768
static char report[REPORT_CAPACITY];
static char replay_snapshot[REPORT_CAPACITY];
static size_t report_length;
static bool earlier_lines_discarded;
static critical_section_t report_lock;

void report_init(void) { critical_section_init(&report_lock); }

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
    size_t drop = 0;
    while (report_length - drop + length >= sizeof(report)) {
        const char *newline = memchr(report + drop, '\n', report_length - drop);
        drop = newline ? (size_t)(newline - report) + 1 : report_length;
    }
    if (drop) {
        memmove(report, report + drop, report_length - drop);
        report_length -= drop;
        earlier_lines_discarded = true;
    }
    memcpy(report + report_length, line, length);
    report_length += length;
    report[report_length] = '\0';
    critical_section_exit(&report_lock);
    if (stdio_usb_connected()) printf("%s", line);
}

void report_replay(void)
{
    if (!stdio_usb_connected()) return;
    // A late cleanup event can still log after a radio fault. Snapshot before USB I/O.
    critical_section_enter_blocking(&report_lock);
    memcpy(replay_snapshot, report, report_length + 1);
    bool discarded = earlier_lines_discarded;
    critical_section_exit(&report_lock);
    printf("\n========== STORED APPROACH REPORT ==========\n");
    if (discarded) printf("Earlier lines discarded; most recent report retained.\n");
    printf("%s", replay_snapshot);
    printf("========== END OF REPORT ==========\n");
    printf("Press GP20 to print again. Reset erases the report and payload.\n");
}
