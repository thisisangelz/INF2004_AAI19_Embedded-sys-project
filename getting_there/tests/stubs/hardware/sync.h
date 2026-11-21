#ifndef HOST_HARDWARE_SYNC_H
#define HOST_HARDWARE_SYNC_H
#include <stdint.h>
#ifdef HOST_TEST_IRQ_HOOKS
uint32_t host_test_save_and_disable_interrupts(void);
void host_test_restore_interrupts(uint32_t state);
static inline uint32_t save_and_disable_interrupts(void)
{ return host_test_save_and_disable_interrupts(); }
static inline void restore_interrupts(uint32_t state)
{ host_test_restore_interrupts(state); }
#else
// Host tests are single-threaded, so there is no interrupt to mask.
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t state) { (void)state; }
#endif
#endif
