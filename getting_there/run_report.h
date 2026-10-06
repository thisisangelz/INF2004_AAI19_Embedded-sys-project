#ifndef GETTING_THERE_RUN_REPORT_H
#define GETTING_THERE_RUN_REPORT_H
void report_init(void);
void report_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
void report_replay(void);
#endif
