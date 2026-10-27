#ifndef GETTING_THERE_REPORT_STORE_H
#define GETTING_THERE_REPORT_STORE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Pico W has 2 MiB flash. The linker reserves its final 512 KiB for logs.
#define REPORT_FLASH_OFFSET (1536u * 1024u)
#define REPORT_FLASH_BYTES (512u * 1024u)
// BTstack's two banks stay immediately before the journal, outside firmware.
#define REPORT_BT_FLASH_BYTES (8u * 1024u)
#define REPORT_BT_FLASH_OFFSET (REPORT_FLASH_OFFSET - REPORT_BT_FLASH_BYTES)
#define REPORT_PAGE_BYTES 256u
#define REPORT_PAGE_DATA 244u
typedef void (*report_emit_t)(const char *data, size_t length);
bool report_store_begin(void);
bool report_store_append(const char *data, size_t length);
bool report_store_finish(void);
void report_store_replay(report_emit_t emit);

// Hardware adapter. All writes are foreground-only, with motors OFF.
void report_flash_read(uint32_t offset, void *data, size_t length);
bool report_flash_erase(void);
bool report_flash_program(uint32_t offset, const void *page);
#endif
