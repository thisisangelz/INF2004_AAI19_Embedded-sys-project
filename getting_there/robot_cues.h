#ifndef GETTING_THERE_ROBOT_CUES_H
#define GETTING_THERE_ROBOT_CUES_H
#include <stdbool.h>

// Robot status sounds on the Robo Pico buzzer (GP22). All non-blocking.
typedef enum {
    CUE_START,       // GP20 accepted: hands off
    CUE_CALIBRATING, // steady tone while the gyro calibrates: keep still
    CUE_PASS,        // a direction pass is starting
    CUE_IN_RANGE,    // BLE close-range check passed: beeps speed up, then
                     // stay fast until cue_stop() at the end of the transfer
    CUE_COLLECTED,   // payload verified, run complete: three rising beeps
    CUE_FAULT,       // stopped without a payload: three falling beeps
    CUE_BUTTON,      // GP20 held long enough to print the saved log
} robot_cue_t;

void cues_init(void);
void cue_play(robot_cue_t cue);
void cue_stop(void);
bool cue_busy(void);

#endif
