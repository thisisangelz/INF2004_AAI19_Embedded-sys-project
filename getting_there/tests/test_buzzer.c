#include <assert.h>
#include <stdio.h>

// RP2040 has eight PWM slices. Keep the hardware-only constant local to this
// test, which compiles the production driver just like test_motion.c does.
#define NUM_PWM_SLICES 8
#define HOST_TEST_IRQ_HOOKS
#include "../../common/buzzer.c"

static uint64_t fake_now_us, fake_deadline_us;
static uint16_t fake_level, fake_wrap;
static bool fake_irq_disabled, fake_active, fake_no_slots;
static unsigned fake_cancels, fake_adds;
static alarm_id_t fake_id;
static alarm_callback_t fake_callback;
static void *fake_user_data;

uint32_t host_test_save_and_disable_interrupts(void)
{
    uint32_t previous = fake_irq_disabled;
    fake_irq_disabled = true;
    return previous;
}

void host_test_restore_interrupts(uint32_t state)
{
    fake_irq_disabled = state;
}

uint32_t clock_get_hz(int clock)
{
    assert(clock == clk_sys);
    return 125000000;
}

void gpio_set_function(uint pin, uint function)
{
    assert(pin == 22 && function == GPIO_FUNC_PWM);
}

uint pwm_gpio_to_slice_num(uint pin)
{
    assert(pin == 22);
    return (pin >> 1) & 7u;
}

pwm_config pwm_get_default_config(void)
{
    return (pwm_config){.wrap = 65535};
}

void pwm_config_set_clkdiv(pwm_config *config, float divider)
{
    (void)config;
    assert(divider == 16.0f);
}

void pwm_init(uint slice, const pwm_config *config, bool enabled)
{
    assert(slice == 3 && enabled);
    fake_wrap = config->wrap;
}

void pwm_set_wrap(uint slice, uint16_t wrap)
{
    assert(slice == 3);
    fake_wrap = wrap;
}

void pwm_set_gpio_level(uint pin, uint16_t level)
{
    assert(pin == 22);
    fake_level = level;
}

alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t callback,
                          void *user_data, bool fire_if_past)
{
    assert(fake_irq_disabled && !fake_active && ms && !fire_if_past);
    ++fake_adds;
    if (fake_no_slots) return -1;
    fake_active = true;
    fake_deadline_us = fake_now_us + (uint64_t)ms * 1000;
    fake_callback = callback;
    fake_user_data = user_data;
    return ++fake_id;
}

bool cancel_alarm(alarm_id_t id)
{
    assert(fake_irq_disabled && fake_active && id == fake_id);
    ++fake_cancels;
    fake_active = false;
    return true;
}

// Mirror the SDK's signed rescheduling semantics, including callback latency.
// A deadline left in the past is dispatched again by the alarm IRQ loop.
static void fire_at(uint64_t now_us, uint32_t callback_cost_us)
{
    assert(fake_active && !fake_irq_disabled && now_us >= fake_deadline_us);
    fake_now_us = now_us;
    uint64_t previous_deadline = fake_deadline_us;
    int64_t delta = fake_callback(fake_id, fake_user_data);
    fake_now_us += callback_cost_us;
    if (delta < 0) fake_deadline_us = previous_deadline + (uint64_t)-delta;
    else if (delta > 0) fake_deadline_us = fake_now_us + (uint64_t)delta;
    else fake_active = false;
}

static void expect_tone(uint16_t hz)
{
    if (!hz) {
        assert(fake_level == 0);
    } else {
        assert(fake_wrap == (uint16_t)(125000000u / (16u * hz) - 1u));
        assert(fake_level == fake_wrap / 2u && fake_level != 0);
    }
}

int main(void)
{
    static const buzzer_note_t notes[] = {{1000, 10}, {0, 20}, {2000, 30}};
    static const buzzer_note_t steady[] = {{1500, 20}};
    buzzer_init(22);
    assert(!buzzer_busy());
    expect_tone(0);

    // Late callbacks and their execution time must not accumulate in the
    // pattern: transitions remain at 10, 30 and 60 ms from its start.
    buzzer_play(notes, 3, false);
    assert(buzzer_busy() && fake_active && fake_deadline_us == 10000);
    expect_tone(1000);
    fire_at(13500, 700);
    expect_tone(0);
    assert(fake_deadline_us == 30000);
    fire_at(34000, 900);
    expect_tone(2000);
    assert(fake_deadline_us == 60000);
    fire_at(60000, 0);
    assert(!fake_active && !buzzer_busy());
    expect_tone(0);
    buzzer_stop(); // A finished alarm must not be cancelled using its old ID.
    assert(fake_cancels == 0);

    // A delay longer than a note catches up using the original deadlines.
    uint64_t origin = fake_now_us;
    buzzer_play(notes, 3, false);
    fire_at(origin + 45000, 0);
    assert(fake_deadline_us == origin + 30000);
    fire_at(fake_now_us, 0);
    assert(fake_deadline_us == origin + 60000);
    expect_tone(2000);
    fire_at(origin + 60000, 0);
    assert(!buzzer_busy() && !fake_active);

    // Full repetition returns to the intro, while a looping tail skips it.
    origin = fake_now_us;
    buzzer_play(notes, 3, true);
    fire_at(origin + 10000, 0);
    fire_at(origin + 30000, 0);
    fire_at(origin + 60000, 0);
    expect_tone(1000);
    assert(buzzer_busy() && fake_deadline_us == origin + 70000);
    buzzer_stop();
    assert(!fake_active && !buzzer_busy() && fake_cancels == 1);
    expect_tone(0);

    origin = fake_now_us;
    buzzer_play_looping_tail(notes, 3, 1);
    fire_at(origin + 10000, 0);
    fire_at(origin + 30000, 0);
    fire_at(origin + 65000, 500);
    expect_tone(0);
    assert(fake_deadline_us == origin + 80000);
    fire_at(origin + 80000, 0);
    expect_tone(2000);
    assert(fake_deadline_us == origin + 110000);

    // Replacing an active pattern cancels it and starts a new timeline.
    origin = fake_now_us;
    buzzer_play(steady, 1, true);
    assert(fake_cancels == 2 && fake_deadline_us == origin + 20000);
    fire_at(origin + 23000, 400);
    expect_tone(1500);
    assert(buzzer_busy() && fake_deadline_us == origin + 40000);
    buzzer_stop();
    assert(fake_cancels == 3 && !fake_active && !buzzer_busy());
    expect_tone(0);

    // An empty pattern is the supported stop operation, not a zero-ms note.
    unsigned adds_before = fake_adds;
    buzzer_play(NULL, 0, false);
    assert(fake_adds == adds_before && !buzzer_busy());
    fake_no_slots = true;
    buzzer_play(steady, 1, true);
    assert(!fake_active && !buzzer_busy() && !fake_irq_disabled);
    expect_tone(0);
    buzzer_stop();
    assert(fake_cancels == 3);
    puts("Buzzer: SDK alarm timing, delayed callback catch-up, repeat/tail, replacement, stop and allocation failure passed");
    return 0;
}
