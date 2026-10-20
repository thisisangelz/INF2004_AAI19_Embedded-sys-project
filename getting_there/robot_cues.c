#include "robot_cues.h"

#include "buzzer.h"

#ifndef BUZZER_PIN
#define BUZZER_PIN 22
#endif

#define N(hz, ms) {(hz), (ms)}
#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

static const buzzer_note_t start[] = {N(2000, 80)};
// One note repeated with no gap: a steady tone until cue_stop().
static const buzzer_note_t calibrating[] = {N(1500, 200)};
static const buzzer_note_t pass[] = {N(1800, 600)};
// Gaps shrink from 300 ms to 60 ms, then the last beep and gap repeat.
static const buzzer_note_t in_range[] = {
    N(3000, 60), N(0, 300), N(3000, 60), N(0, 220), N(3000, 60), N(0, 160),
    N(3000, 60), N(0, 110), N(3000, 60), N(0, 80), N(3000, 50), N(0, 60),
};
#define IN_RANGE_LOOP_FROM 10
static const buzzer_note_t collected[] = {
    N(1500, 200), N(0, 80), N(2000, 200), N(0, 80), N(2700, 350),
};
static const buzzer_note_t fault[] = {
    N(1500, 250), N(0, 120), N(1000, 250), N(0, 120), N(700, 500),
};
static const buzzer_note_t button[] = {N(2500, 40), N(0, 40), N(2500, 40)};

void cues_init(void)
{
    buzzer_init(BUZZER_PIN);
}

void cue_play(robot_cue_t cue)
{
    switch (cue) {
    case CUE_START: buzzer_play(start, COUNT(start), false); break;
    case CUE_CALIBRATING: buzzer_play(calibrating, COUNT(calibrating), true); break;
    case CUE_PASS: buzzer_play(pass, COUNT(pass), false); break;
    case CUE_IN_RANGE:
        buzzer_play_looping_tail(in_range, COUNT(in_range), IN_RANGE_LOOP_FROM);
        break;
    case CUE_COLLECTED: buzzer_play(collected, COUNT(collected), false); break;
    case CUE_FAULT: buzzer_play(fault, COUNT(fault), false); break;
    case CUE_BUTTON: buzzer_play(button, COUNT(button), false); break;
    }
}

void cue_stop(void) { buzzer_stop(); }
bool cue_busy(void) { return buzzer_busy(); }
