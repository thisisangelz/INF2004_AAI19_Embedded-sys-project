#include "report_store.h"
#include "motion.h"
#include <string.h>
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "pico/flash.h"
#include "pico/stdlib.h"
#include "run_report.h"

_Static_assert(PICO_FLASH_SIZE_BYTES == REPORT_FLASH_OFFSET + REPORT_FLASH_BYTES,
               "SDK flash bounds must describe the full physical chip");
_Static_assert(PICO_FLASH_BANK_STORAGE_OFFSET == REPORT_BT_FLASH_OFFSET,
               "Bluetooth storage must remain before the journal");
_Static_assert(PICO_FLASH_BANK_TOTAL_SIZE == REPORT_BT_FLASH_BYTES,
               "Bluetooth storage reservation mismatch");
_Static_assert(REPORT_PAGE_BYTES == FLASH_PAGE_SIZE, "journal/flash page mismatch");
_Static_assert(REPORT_FLASH_OFFSET + REPORT_FLASH_BYTES == 2u * 1024u * 1024u,
               "journal must end at physical Pico W flash limit");

typedef struct {
    uint32_t offset;
    const uint8_t *page;
} flash_operation_t;

void report_flash_read(uint32_t offset, void *data, size_t length)
{
    memcpy(data, (const void *)(XIP_BASE + REPORT_FLASH_OFFSET + offset), length);
}

static void erase_sector(void *arg)
{
    flash_operation_t *op = arg;
    flash_range_erase(REPORT_FLASH_OFFSET + op->offset, FLASH_SECTOR_SIZE);
}

bool report_flash_erase(void)
{
    // Before radio initialisation and movement. Yield between sectors for USB.
    const uint32_t sectors = REPORT_FLASH_BYTES / FLASH_SECTOR_SIZE;
    report_printf("Preparing persistent log: erasing %lu sectors in the reserved 512 KiB region\n",
                  (unsigned long)sectors);
    for (uint32_t offset = 0; offset < REPORT_FLASH_BYTES; offset += FLASH_SECTOR_SIZE) {
        flash_operation_t op = {.offset = offset};
        if (flash_safe_execute(erase_sector, &op, 1000) != PICO_OK) {
            report_printf("LOG PREPARATION FAILED at flash sector %lu/%lu\n",
                          (unsigned long)(offset / FLASH_SECTOR_SIZE + 1),
                          (unsigned long)sectors);
            return false;
        }
        // Motors are OFF, but remember a STOP pressed during log preparation.
        (void)motion_stop_requested();
        sleep_ms(1);
        if ((offset / FLASH_SECTOR_SIZE + 1) % 16 == 0)
            report_printf("Preparing persistent log: %lu/%lu flash sectors erased\n",
                          (unsigned long)(offset / FLASH_SECTOR_SIZE + 1),
                          (unsigned long)sectors);
    }
    return true;
}

static void program_page(void *arg)
{
    flash_operation_t *op = arg;
    flash_range_program(REPORT_FLASH_OFFSET + op->offset, op->page, FLASH_PAGE_SIZE);
}

bool report_flash_program(uint32_t offset, const void *page)
{
    if (offset % FLASH_PAGE_SIZE || offset > REPORT_FLASH_BYTES - FLASH_PAGE_SIZE)
        return false;
    flash_operation_t op = {.offset = offset, .page = page};
    return flash_safe_execute(program_page, &op, 1000) == PICO_OK;
}
