#ifndef GETTING_THERE_MOTION_H
#define GETTING_THERE_MOTION_H
#include <stdbool.h>
#include <stdint.h>
bool motion_init(void);
bool motion_calibrate(void);
bool motion_learn_direction(void);
bool motion_drive(bool forwards, uint32_t duration_ms, float heading_degrees);
// Cooperative drive: call update at least every 5 ms, then finish once stopped.
// Elapsed time excludes the motor-off settling period and remains available
// after STOP or an error. A recovered straight-line read fault can end a
// block early; finish must settle and validate heading before the next start.
// These functions never write logs while PWM is on.
bool motion_drive_start(bool forwards, uint32_t duration_ms, float heading_degrees);
bool motion_drive_update(void);
bool motion_drive_active(void);
uint32_t motion_drive_elapsed_ms(void);
bool motion_drive_finish(void);
bool motion_turn_to_heading(float heading_degrees);
float motion_heading(void);
bool motion_pause_ms(uint32_t duration_ms);
bool motion_stop_requested(void);
bool motion_start_pressed(void);
bool motion_replay_pressed(void);
void motion_arm_stop(void);
void motion_stop(void);
#endif
