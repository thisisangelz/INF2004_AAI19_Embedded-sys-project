// Exercise the scanner's real classification and motion/checkpoint boundaries.
#include <assert.h>
#include <stdio.h>
#include "colour_scan.h"
#include "faults.h"

static unsigned turns, reads, checkpoints;
static unsigned fail_read, fail_checkpoint, stop_at_turn;
static bool fail_turn;
static float heading;

void report_printf(const char *format, ...) { (void)format; }
bool motion_turn_to_heading(float target)
{
    assert(target == 10.0f + turns * SCAN_STEP_DEG);
    ++turns;
    heading = target;
    return !fail_turn;
}
bool motion_pause_ms(uint32_t ms) { assert(ms == 200); return true; }
float motion_heading(void) { return heading; }

bool colour_sensor_read(colour_reading_t *reading)
{
    ++reads;
    if (reads == fail_read) return false;
    // Every stop has two ring-2 samples and a floor outlier. The scanner
    // must classify the median rather than using the latest sample.
    *reading = (colour_reading_t){.c = 5000, .r = 1000,
        .g = reads % 3 == 0 ? 920 : 780, .b = 900};
    return true;
}

static bool can_continue(void) { return turns < stop_at_turn; }
static bool checkpoint(void)
{
    ++checkpoints;
    assert(reads == checkpoints * SCAN_READS);
    return checkpoints != fail_checkpoint;
}
static void reset(void)
{
    turns = reads = checkpoints = fail_read = fail_checkpoint = 0;
    stop_at_turn = SCAN_STEPS + 1;
    fail_turn = false;
    for (int f = 0; f < FAULT_COUNT; ++f) fault_clear((fault_t)f);
    colour_set_target_mat(COLOUR_MAT_PINK);
}
static scan_status_t scan(colour_scan_result_t *estimate, unsigned *lighter, int *rings)
{
    return colour_scan_turn(10.0f, 1, estimate, lighter, rings, can_continue, checkpoint);
}

int main(void)
{
    colour_scan_result_t estimate;
    unsigned lighter;
    int rings;
    reset();
    assert(scan(&estimate, &lighter, &rings) == SCAN_DONE);
    assert(turns == SCAN_STEPS && reads == SCAN_STEPS * SCAN_READS);
    assert(checkpoints == SCAN_STEPS && heading == 340.0f);
    assert(lighter == (1u << COLOUR_MAT_PINK) && rings == 2);
    assert(!estimate.valid); // All directions equally dark: no invented bearing.

    reset();
    stop_at_turn = 0;
    assert(scan(&estimate, &lighter, &rings) == SCAN_STOPPED);
    assert(!turns && !reads && !checkpoints && !estimate.valid);

    reset();
    stop_at_turn = 2;
    assert(scan(&estimate, &lighter, &rings) == SCAN_STOPPED);
    assert(turns == 2 && checkpoints == 2 && !estimate.valid);

    reset();
    fail_turn = true;
    assert(scan(&estimate, &lighter, &rings) == SCAN_STOPPED);
    assert(turns == 1 && !reads && !checkpoints);

    reset();
    fail_read = 2;
    assert(scan(&estimate, &lighter, &rings) == SCAN_READ_FAILED);
    assert(turns == 1 && reads == 2 && !checkpoints && !estimate.valid);

    reset();
    fail_checkpoint = 1;
    assert(scan(&estimate, &lighter, &rings) == SCAN_STOPPED);
    assert(turns == 1 && checkpoints == 1 && fault_test(FAULT_REPORT));
    assert(!estimate.valid);
    puts("Colour scan: median, flat surface, STOP/deadline, turn/read failure and checkpoint failure passed");
}
