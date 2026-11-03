#ifndef HOST_HARDWARE_FLASH_H
#define HOST_HARDWARE_FLASH_H
#include <stddef.h>
#include <stdint.h>
#define FLASH_PAGE_SIZE 256u
#define FLASH_SECTOR_SIZE 4096u
void flash_range_erase(uint32_t offset, size_t count);
void flash_range_program(uint32_t offset, const uint8_t *data, size_t count);
#endif
