#ifndef GETTING_THERE_RUN_REPORT_H
#define GETTING_THERE_RUN_REPORT_H
#include <stdbool.h>
void report_init(void);
void report_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
// Prints this boot's report from RAM.
void report_replay(void);

// Flash copy of the report, kept across reset and power-off. The previous
// run stays readable until a new run makes its first checkpoint.
// Starts a new saved run; later checkpoints overwrite the previous run's log.
void report_begin_run(void);
// Checkpoint to flash. Flash writes pause the whole chip for tens of
// milliseconds, so call only with motors stopped and no radio operation.
void report_save(void);
// Prints the saved log over USB. Returns false if USB is not connected.
bool report_print_saved(void);
#endif
