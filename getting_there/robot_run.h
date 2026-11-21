#ifndef GETTING_THERE_ROBOT_RUN_H
#define GETTING_THERE_ROBOT_RUN_H

#include <stdbool.h>
#include <stdint.h>

// Run orchestration is owned by the main core. Drivers/radio must be ready,
// STOP armed, and the report open before execute; it returns true only after
// every requested payload is verified. All exits leave the motors stopped.
bool robot_run_execute(void);
uint8_t robot_run_next_beacon(void); // 0 for a single-beacon build
bool robot_run_starts_on_mat(void); // leg-2 diagnostic build

// Main-loop callbacks; these must not be invoked from an IRQ.
void robot_run_in_range(void);
void robot_run_completion_cue(void);

#if COLOUR_SCAN_TEST
// Diagnostic build: run the colour scan with no radio or translation.
bool robot_run_colour_scan(void);
#endif

#endif
