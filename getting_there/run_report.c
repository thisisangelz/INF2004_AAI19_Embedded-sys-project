#include "run_report.h"
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hardware/flash.h"
#include "pico/btstack_flash_bank.h"
#include "pico/critical_section.h"
#include "pico/flash.h"
#include "pico/stdio_usb.h"

#define REPORT_CAPACITY 32768
static char report[REPORT_CAPACITY];
// Used for USB replay and for building flash checkpoints; both run on the
// main loop only, never at the same time.
static char snapshot[REPORT_CAPACITY];
static size_t report_length;
static bool earlier_lines_discarded;
static critical_section_t report_lock;

// Saved log layout, just below BTstack's own flash bank at the end of flash:
// a 16-byte header, then the report text, then erased 0xFF bytes.
#define LOG_MAGIC 0x474F4C52u // "RLOG"
#define LOG_HEADER_SIZE 16u
#define LOG_SECTORS ((LOG_HEADER_SIZE + REPORT_CAPACITY + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE)
#define LOG_REGION_SIZE (LOG_SECTORS * FLASH_SECTOR_SIZE)
#define LOG_OFFSET (PICO_FLASH_BANK_STORAGE_OFFSET - LOG_REGION_SIZE)
#define FLASH_TIMEOUT_MS 100

typedef struct {
    uint32_t magic;
    uint32_t run_number;
    uint32_t run_check; // ~run_number, guards against a half-written header
    uint32_t reserved;
} log_header_t;
_Static_assert(sizeof(log_header_t) == LOG_HEADER_SIZE, "log header size");

extern char __flash_binary_end;
static bool flash_log_fits;
static bool saving_enabled;
static bool save_failed;
static uint32_t saved_run_number; // 0 = nothing valid in flash
static uint32_t current_run_number;
static uint8_t sector_buffer[FLASH_SECTOR_SIZE];

typedef struct {
    uint32_t offset;
    const uint8_t *data;
} flash_job_t;

static const uint8_t *log_flash(void)
{
    return (const uint8_t *)(XIP_BASE + LOG_OFFSET);
}

static void read_saved_header(void)
{
    log_header_t header;
    memcpy(&header, log_flash(), sizeof(header));
    saved_run_number = header.magic == LOG_MAGIC &&
                       header.run_check == ~header.run_number
                           ? header.run_number : 0;
}

void report_init(void)
{
    critical_section_init(&report_lock);
    // The programme must end below the log region, or saving would erase it.
    flash_log_fits = (uintptr_t)&__flash_binary_end - XIP_BASE <= LOG_OFFSET;
    if (flash_log_fits) read_saved_header();
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

static size_t take_snapshot(bool *discarded)
{
    // A late cleanup event can still log after a radio fault. Snapshot before
    // slow USB or flash I/O.
    critical_section_enter_blocking(&report_lock);
    size_t length = report_length;
    memcpy(snapshot, report, length + 1);
    if (discarded) *discarded = earlier_lines_discarded;
    critical_section_exit(&report_lock);
    return length;
}

void report_replay(void)
{
    if (!stdio_usb_connected()) return;
    bool discarded;
    take_snapshot(&discarded);
    printf("\n========== STORED APPROACH REPORT ==========\n");
    if (discarded) printf("Earlier lines discarded; most recent report retained.\n");
    printf("%s", snapshot);
    printf("========== END OF REPORT ==========\n");
    printf("Press GP20 to print again. A copy is saved in flash and survives reset.\n");
}

void report_begin_run(void)
{
    if (!flash_log_fits) {
        report_printf("FLASH LOG: disabled; programme overlaps the log area\n");
        return;
    }
    current_run_number = saved_run_number + 1;
    saving_enabled = true;
    save_failed = false;
}

static void erase_and_program(void *param)
{
    const flash_job_t *job = param;
    flash_range_erase(job->offset, FLASH_SECTOR_SIZE);
    flash_range_program(job->offset, job->data, FLASH_SECTOR_SIZE);
}

// Fills sector_buffer with bytes [sector * 4096, +4096) of the saved image.
static void build_sector(unsigned sector, size_t text_length)
{
    const log_header_t header = {
        .magic = LOG_MAGIC,
        .run_number = current_run_number,
        .run_check = ~current_run_number,
        .reserved = 0xFFFFFFFFu,
    };
    memset(sector_buffer, 0xFF, sizeof(sector_buffer));
    size_t base = (size_t)sector * FLASH_SECTOR_SIZE;
    for (size_t i = 0; i < FLASH_SECTOR_SIZE; ++i) {
        size_t position = base + i;
        if (position < LOG_HEADER_SIZE) {
            sector_buffer[i] = ((const uint8_t *)&header)[position];
        } else if (position - LOG_HEADER_SIZE < text_length) {
            sector_buffer[i] = (uint8_t)snapshot[position - LOG_HEADER_SIZE];
        }
    }
}

void report_save(void)
{
    if (!saving_enabled || save_failed) return;
    size_t length = take_snapshot(NULL);
    for (unsigned sector = 0; sector < LOG_SECTORS; ++sector) {
        build_sector(sector, length);
        const uint8_t *current = log_flash() + sector * FLASH_SECTOR_SIZE;
        // The log only grows, so usually just the header sector and the
        // last text sector change. Unchanged sectors are not rewritten.
        if (memcmp(current, sector_buffer, FLASH_SECTOR_SIZE) == 0) continue;
        flash_job_t job = {LOG_OFFSET + sector * FLASH_SECTOR_SIZE, sector_buffer};
        int rc = flash_safe_execute(erase_and_program, &job, FLASH_TIMEOUT_MS);
        if (rc != PICO_OK) {
            save_failed = true;
            report_printf("FLASH LOG: save failed (error %d); later checkpoints skipped\n", rc);
            return;
        }
    }
    saved_run_number = current_run_number;
}

bool report_print_saved(void)
{
    if (!stdio_usb_connected()) return false;
    if (!flash_log_fits) {
        printf("\nFLASH LOG: unavailable; programme overlaps the log area\n");
        return true;
    }
    read_saved_header();
    if (saved_run_number == 0) {
        printf("\n========== NO SAVED RUN LOG IN FLASH ==========\n");
        return true;
    }
    const char *text = (const char *)log_flash() + LOG_HEADER_SIZE;
    size_t length = 0;
    while (length < REPORT_CAPACITY && text[length] != '\0' &&
           (uint8_t)text[length] != 0xFF) {
        ++length;
    }
    printf("\n========== SAVED LOG: RUN #%lu (from flash) ==========\n",
           (unsigned long)saved_run_number);
    printf("%.*s", (int)length, text);
    printf("========== END OF SAVED LOG ==========\n");
    printf("Hold GP20 for 2 s to print again. The next run overwrites it.\n");
    return true;
}
