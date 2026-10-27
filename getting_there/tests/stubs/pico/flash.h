#ifndef HOST_PICO_FLASH_H
#define HOST_PICO_FLASH_H
#include <stdint.h>
#define PICO_OK 0
int flash_safe_execute(void (*callback)(void *), void *context, uint32_t timeout_ms);
#endif
