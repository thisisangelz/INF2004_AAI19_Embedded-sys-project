#ifndef GETTING_THERE_MOTION_H
#define GETTING_THERE_MOTION_H
#include <stdbool.h>
#include <stdint.h>
bool motion_init(void);
bool motion_calibrate(void);
bool motion_learn_direction(void);
bool motion_drive(bool forwards, uint32_t duration_ms, float heading_degrees);
bool motion_turn_to_heading(float heading_degrees);
float motion_heading(void);
bool motion_pause_ms(uint32_t duration_ms);
bool motion_stop_requested(void);
bool motion_start_pressed(void);
void motion_stop(void);
#endif
