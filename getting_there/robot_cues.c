#include "robot_cues.h"

#include "buzzer.h"

#ifndef BUZZER_PIN
#define BUZZER_PIN 22
#endif

#define N(hz, ms) {(hz), (ms)}
#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

static const buzzer_note_t start[] = {N(2000, 80)};
static const buzzer_note_t calibrating[] = {N(2500, 20), N(0, 230)};
static const buzzer_note_t ready[] = {N(1500, 100), N(0, 60), N(2500, 150)};
static const buzzer_note_t pass[] = {N(1800, 60)};
static const buzzer_note_t in_range[] = {
    N(3000, 80), N(0, 60), N(3000, 80), N(0, 60), N(3000, 80),
};
static const buzzer_note_t collected[] = {N(2000, 1000)};
static const buzzer_note_t all_done[] = {
    N(2500, 400), N(0, 100), N(2500, 120), N(0, 100), N(2500, 400),
};
static const buzzer_note_t fault[] = {
    N(800, 300), N(0, 200), N(800, 300), N(0, 200), N(800, 300),
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
    case CUE_READY: buzzer_play(ready, COUNT(ready), false); break;
    case CUE_PASS: buzzer_play(pass, COUNT(pass), false); break;
    case CUE_IN_RANGE: buzzer_play(in_range, COUNT(in_range), false); break;
    case CUE_COLLECTED: buzzer_play(collected, COUNT(collected), false); break;
    case CUE_ALL_DONE: buzzer_play(all_done, COUNT(all_done), false); break;
    case CUE_FAULT: buzzer_play(fault, COUNT(fault), false); break;
    case CUE_BUTTON: buzzer_play(button, COUNT(button), false); break;
    }
}

void cue_stop(void) { buzzer_stop(); }
bool cue_busy(void) { return buzzer_busy(); }
