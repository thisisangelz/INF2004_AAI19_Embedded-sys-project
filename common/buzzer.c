#include "buzzer.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/time.h"

#define BUZZER_CLKDIV 16.0f

static unsigned buzzer_pin;
static unsigned buzzer_slice;
static bool initialised;

// Pattern state. Changed only with interrupts disabled (buzzer_play/stop) or
// from the alarm callback, which cannot be pre-empted by those callers.
static const buzzer_note_t *pattern;
static size_t pattern_count;
static size_t pattern_index;
static bool pattern_repeat;
static size_t pattern_loop_from;
static alarm_id_t alarm;
static volatile bool playing;

static void set_tone(uint16_t freq_hz)
{
    if (freq_hz == 0) {
        pwm_set_gpio_level(buzzer_pin, 0);
        return;
    }
    uint32_t wrap = (uint32_t)(clock_get_hz(clk_sys) /
                               (BUZZER_CLKDIV * freq_hz)) - 1u;
    if (wrap > 65535u) wrap = 65535u;
    if (wrap < 2u) wrap = 2u;
    pwm_set_wrap(buzzer_slice, (uint16_t)wrap);
    pwm_set_gpio_level(buzzer_pin, (uint16_t)(wrap / 2u));
}

static int64_t next_note(alarm_id_t id, void *user_data)
{
    (void)id;
    (void)user_data;
    if (++pattern_index >= pattern_count) {
        if (!pattern_repeat) {
            set_tone(0);
            playing = false;
            alarm = 0;
            return 0;
        }
        pattern_index = pattern_loop_from;
    }
    set_tone(pattern[pattern_index].freq_hz);
    // A positive return reschedules relative to this alarm's due time, so
    // pattern timing does not drift.
    return (int64_t)pattern[pattern_index].ms * 1000;
}

void buzzer_init(unsigned pin)
{
    buzzer_pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, BUZZER_CLKDIV);
    pwm_init(buzzer_slice, &config, true);
    pwm_set_gpio_level(pin, 0);
    initialised = true;
}

static void start_pattern(const buzzer_note_t *notes, size_t count,
                          bool repeat, size_t loop_from)
{
    if (!initialised) return;
    uint32_t irq = save_and_disable_interrupts();
    if (alarm > 0) cancel_alarm(alarm);
    alarm = 0;
    pattern = notes;
    pattern_count = count;
    pattern_index = 0;
    pattern_repeat = repeat;
    pattern_loop_from = loop_from < count ? loop_from : 0;
    playing = count > 0;
    if (playing) {
        set_tone(notes[0].freq_hz);
        // Notes are at least 1 ms, so the alarm is never already in the past.
        alarm = add_alarm_in_ms(notes[0].ms, next_note, NULL, false);
        if (alarm <= 0) {
            // No free alarm: stay silent rather than hold a tone forever.
            set_tone(0);
            playing = false;
            alarm = 0;
        }
    } else {
        set_tone(0);
    }
    restore_interrupts(irq);
}

void buzzer_play(const buzzer_note_t *notes, size_t count, bool repeat)
{
    start_pattern(notes, count, repeat, 0);
}

void buzzer_play_looping_tail(const buzzer_note_t *notes, size_t count,
                              size_t loop_from)
{
    start_pattern(notes, count, true, loop_from);
}

void buzzer_stop(void)
{
    start_pattern(NULL, 0, false, 0);
}

bool buzzer_busy(void)
{
    return playing;
}
