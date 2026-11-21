#ifndef BEACON_LOG_H
#define BEACON_LOG_H

// Core 0 only: initialise before enabling radio callbacks; enqueue from those
// callbacks and flush from main. No heap, RTOS, or core 1 access is supported.
#define BEACON_LOG_LINE_BYTES 320u
#define BEACON_LOG_QUEUE_LINES 8u
#define BEACON_LOG_FLUSH_LINES 2u

void beacon_log_init(void);
// Trusted constant formats, bounded widths/precisions only. Every %s argument
// must have a bounded precision so formatting work in an IRQ is also bounded.
// A full queue drops the NEW line; an overlong line retains 318 bytes + '\n'.
// Dropped and truncated calls are counted independently (one may count both).
void beacon_log_printf(const char *format, ...)
    __attribute__((format(__printf__, 1, 2)));
// At most two output attempts per call, including loss summaries. Disconnected
// USB preserves the bounded queue. Short writes retain their unsent suffix.
// Copies/counter snapshots mask IRQs; formatting and USB writes never do.
// Loss counters saturate and are reported as >=UINT32_MAX, never wrap.
void beacon_log_flush(void);

#endif
