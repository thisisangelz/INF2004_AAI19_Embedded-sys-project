#ifndef GETTING_THERE_RUN_REPORT_H
#define GETTING_THERE_RUN_REPORT_H
#include <stdbool.h>
void report_init(void);
bool report_begin(void);
bool report_checkpoint(void);
bool report_finish(void);
bool report_ok(void);
void report_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
// Radio callbacks queue text only. Drain USB from main with motors stopped.
void report_defer_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
void report_flush_usb(void);
void report_replay(void);
#endif
