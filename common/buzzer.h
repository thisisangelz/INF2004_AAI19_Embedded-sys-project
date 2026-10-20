#ifndef COMMON_BUZZER_H
#define COMMON_BUZZER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Background tone player for the Robo Pico's piezo buzzer (GP22). A pattern
// plays from a hardware timer alarm, so callers never block and motors, scans
// and BLE keep running. Safe to call from the main loop or BTstack callbacks.

typedef struct {
    uint16_t freq_hz; // 0 = silence
    uint16_t ms;      // must be at least 1
} buzzer_note_t;

void buzzer_init(unsigned pin);
// Replaces whatever is playing. The notes must stay valid while playing
// (use static const arrays). repeat loops the pattern until buzzer_stop().
void buzzer_play(const buzzer_note_t *notes, size_t count, bool repeat);
void buzzer_stop(void);
bool buzzer_busy(void);

#endif
