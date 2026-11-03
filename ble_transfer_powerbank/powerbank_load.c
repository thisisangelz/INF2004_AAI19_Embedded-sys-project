#include <stdint.h>

#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "powerbank_load.h"

static semaphore_t core1_ready;
static bool core1_flash_safe;

// Scratch X also holds core 1's stack. Core 0's stack is in scratch Y.
// Keeping the load here avoids repeatedly fetching instructions from flash
// or the main SRAM banks used by Wi-Fi and BLE.
static void __noinline __scratch_x("powerbank_cpu_load") powerbank_load_core1(void)
{
    // BTstack may write flash during CYW43 initialisation or later operation.
    // Leave interrupts enabled so the SDK can pause this core for those writes.
    core1_flash_safe = flash_safe_execute_core_init();
    sem_release(&core1_ready);

    uint32_t a = 0x12345678u;
    uint32_t b = 0x9e3779b9u;
    while (true) {
        a ^= a << 13;
        a ^= a >> 17;
        a ^= a << 5;
        b = (b + a) * 33u;
        a ^= b;
        // Retain the calculations without volatile memory traffic. There are
        // no radio calls, shared-state accesses or waits in the running loop.
        __asm volatile("" : "+r"(a), "+r"(b));
    }
}

bool powerbank_load_start(void)
{
    sem_init(&core1_ready, 0, 1);
    multicore_launch_core1(powerbank_load_core1);
    // This synchronises both the ready flag and the SDK's flash setup before
    // core 0 starts CYW43/BTstack. The FIFO is reserved for SDK flash lockout.
    sem_acquire_blocking(&core1_ready);
    return core1_flash_safe;
}
