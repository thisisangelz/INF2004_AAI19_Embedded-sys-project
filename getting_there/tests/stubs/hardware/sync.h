#ifndef HOST_HARDWARE_SYNC_H
#define HOST_HARDWARE_SYNC_H
#include <stdint.h>
// Host tests are single-threaded, so there is no interrupt to mask.
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t state) { (void)state; }
#endif
