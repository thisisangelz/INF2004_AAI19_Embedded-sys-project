#include "beacon_log.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hardware/sync.h"
#include "pico/stdio_usb.h"

typedef struct {
    uint16_t length;
    char text[BEACON_LOG_LINE_BYTES];
} log_line_t;

static log_line_t queue[BEACON_LOG_QUEUE_LINES];
static unsigned head, count;
static uint32_t dropped, truncated;
// Main owns this copy, so producers may reuse a queue slot during USB output.
static log_line_t output;
static size_t output_offset;

static void increment_saturating(uint32_t *counter)
{
    if (*counter != UINT32_MAX) ++*counter;
}

void beacon_log_init(void)
{
    uint32_t irq = save_and_disable_interrupts();
    head = count = 0;
    dropped = truncated = 0;
    output.length = 0;
    output_offset = 0;
    restore_interrupts(irq);
}

void beacon_log_printf(const char *format, ...)
{
    char text[BEACON_LOG_LINE_BYTES];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (written == 0) return;
    bool cut = written >= (int)sizeof(text);
    size_t length = cut ? sizeof(text) - 1u : written > 0 ? (size_t)written : 0;
    if (cut) text[length - 1u] = '\n';

    uint32_t irq = save_and_disable_interrupts();
    if (cut) increment_saturating(&truncated);
    if (written < 0 || count == BEACON_LOG_QUEUE_LINES) {
        increment_saturating(&dropped);
    } else {
        log_line_t *line = &queue[(head + count) % BEACON_LOG_QUEUE_LINES];
        memcpy(line->text, text, length);
        line->length = (uint16_t)length;
        ++count;
    }
    restore_interrupts(irq);
}

// Reserve the next output under the same IRQ mask as queue/counter updates.
// A loss summary is kept outside the ring and cannot itself be dropped on full.
static bool take_output(bool allow_summary)
{
    uint32_t irq = save_and_disable_interrupts();
    uint32_t lost = allow_summary ? dropped : 0;
    uint32_t cut = allow_summary ? truncated : 0;
    if (lost || cut) {
        dropped = truncated = 0;
    } else if (count) {
        const log_line_t *line = &queue[head];
        output.length = line->length;
        memcpy(output.text, line->text, line->length);
        head = (head + 1u) % BEACON_LOG_QUEUE_LINES;
        --count;
    }
    restore_interrupts(irq);
    if (lost || cut) {
        // Fixed-size integers and constant strings fit well inside 320 bytes.
        output.length = (uint16_t)snprintf(output.text, sizeof(output.text),
            "\nBEACON LOG: dropped=%s%lu truncated=%s%lu lines since previous summary\n",
            lost == UINT32_MAX ? ">=" : "", (unsigned long)lost,
            cut == UINT32_MAX ? ">=" : "", (unsigned long)cut);
    }
    output_offset = 0;
    return output.length != 0;
}

void beacon_log_flush(void)
{
    if (!stdio_usb_connected()) return;
    for (unsigned i = 0; i < BEACON_LOG_FLUSH_LINES; ++i) {
        // At most one summary per flush: even continuous overflow must leave
        // an output slot for a queued line rather than starving the old text.
        if (!output.length && !take_output(i == 0)) break;
        size_t remaining = output.length - output_offset;
        size_t written = fwrite(output.text + output_offset, 1, remaining, stdout);
        output_offset += written;
        if (written < remaining) return;
        output.length = 0;
    }
}
