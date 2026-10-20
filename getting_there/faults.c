#include "faults.h"
#include "packed_bits.h"

_Static_assert(FAULT_COUNT <= 16, "fault mask no longer fits uint16_t");
_Static_assert(FAULT_COUNT > 8, "fault mask fits in a byte; shrink it");

static volatile uint16_t faults;

void fault_raise(fault_t fault)
{
    uint32_t irq = save_and_disable_interrupts();
    faults |= (uint16_t)(1u << fault);
    restore_interrupts(irq);
}

void fault_clear(fault_t fault)
{
    uint32_t irq = save_and_disable_interrupts();
    faults &= (uint16_t)~(1u << fault);
    restore_interrupts(irq);
}

bool fault_test(fault_t fault) { return (faults >> fault) & 1u; }
uint16_t fault_mask(void) { return faults; }

const char *fault_name(fault_t fault)
{
    static const char *const names[FAULT_COUNT] = {
        "IMU", "GYRO_SATURATED", "HEADING", "TURN", "WIFI_LOST", "WIFI_SCAN",
        "BLE_RADIO", "PAYLOAD", "STOP_LATCHED", "RUN_LIMIT", "NO_DIRECTION", "STEP_LIMIT",
    };
    return fault < FAULT_COUNT ? names[fault] : "?";
}
