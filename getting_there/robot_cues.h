#ifndef GETTING_THERE_ROBOT_CUES_H
#define GETTING_THERE_ROBOT_CUES_H
#include <stdbool.h>

// Robot status sounds on the Robo Pico buzzer (GP22). All non-blocking.
typedef enum {
    CUE_START,       // GP20 accepted: hands off
    CUE_CALIBRATING, // repeating tick while the gyro calibrates: keep still
    CUE_READY,       // calibration done; the robot will soon move
    CUE_PASS,        // a direction pass is starting
    CUE_IN_RANGE,    // BLE close-range check passed; transfer starting
    CUE_COLLECTED,   // this beacon's payload verified
    CUE_ALL_DONE,    // every target beacon collected
    CUE_FAULT,       // run stopped without a payload, or GP21 pressed
    CUE_BUTTON,      // GP20 held long enough to print the saved log
} robot_cue_t;

void cues_init(void);
void cue_play(robot_cue_t cue);
void cue_stop(void);
bool cue_busy(void);

#endif
