#ifndef GETTING_THERE_RUN_REPORT_H
#define GETTING_THERE_RUN_REPORT_H
#include <stdbool.h>
void report_init(void);
bool report_begin(void);
bool report_checkpoint(void);
bool report_finish(void);
bool report_ok(void);
void report_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
void report_replay(void);
#endif
