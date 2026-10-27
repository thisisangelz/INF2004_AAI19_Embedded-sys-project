#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint8_t flash_image[PICO_FLASH_SIZE_BYTES];
#define XIP_BASE ((uintptr_t)flash_image)
#include "../report_flash.c"

static unsigned erases, programs, safe_calls, stop_polls;
static bool fail_safe;
static char diagnostics[4096];
static size_t diagnostic_length;

void report_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int length = vsnprintf(diagnostics + diagnostic_length,
        sizeof(diagnostics) - diagnostic_length, format, args);
    va_end(args);
    assert(length >= 0 && (size_t)length < sizeof(diagnostics) - diagnostic_length);
    diagnostic_length += (size_t)length;
}
bool motion_stop_requested(void) { ++stop_polls; return false; }
void sleep_ms(uint32_t ms) { assert(ms == 1); }
int flash_safe_execute(void (*callback)(void *), void *context, uint32_t timeout_ms)
{
    assert(timeout_ms == 1000);
    ++safe_calls;
    if (fail_safe) return -1;
    callback(context);
    return PICO_OK;
}
void flash_range_erase(uint32_t offset, size_t count)
{
    // Enforce the SDK 2.3.0 physical-chip check which the report-store mock
    // previously missed. The first erase used to exceed the configured size.
    assert(offset + count <= PICO_FLASH_SIZE_BYTES);
    assert(offset >= REPORT_FLASH_OFFSET && count == FLASH_SECTOR_SIZE);
    assert(offset % FLASH_SECTOR_SIZE == 0);
    memset(flash_image + offset, 0xff, count);
    ++erases;
}
void flash_range_program(uint32_t offset, const uint8_t *data, size_t count)
{
    assert(offset + count <= PICO_FLASH_SIZE_BYTES);
    assert(offset >= REPORT_FLASH_OFFSET && count == FLASH_PAGE_SIZE);
    assert(offset % FLASH_PAGE_SIZE == 0);
    for (size_t i = 0; i < count; ++i) flash_image[offset + i] &= data[i];
    ++programs;
}
int main(void)
{
    assert(REPORT_FLASH_OFFSET + FLASH_SECTOR_SIZE > 1572864u);
    assert(REPORT_BT_FLASH_OFFSET + REPORT_BT_FLASH_BYTES == REPORT_FLASH_OFFSET);
    memset(flash_image, 0x5a, sizeof(flash_image));
    assert(report_flash_erase());
    assert(erases == 128 && stop_polls == 128 && safe_calls == 128);
    for (unsigned i = 0; i < REPORT_FLASH_OFFSET; ++i) assert(flash_image[i] == 0x5a);
    for (unsigned i = REPORT_FLASH_OFFSET; i < sizeof(flash_image); ++i) assert(flash_image[i] == 0xff);
    uint8_t page[REPORT_PAGE_BYTES], readback[REPORT_PAGE_BYTES];
    memset(page, 0xa5, sizeof(page));
    assert(report_flash_program(0, page));
    assert(report_flash_program(REPORT_FLASH_BYTES - REPORT_PAGE_BYTES, page));
    report_flash_read(0, readback, sizeof(readback));
    assert(!memcmp(page, readback, sizeof(page)));
    report_flash_read(REPORT_FLASH_BYTES - REPORT_PAGE_BYTES, readback, sizeof(readback));
    assert(!memcmp(page, readback, sizeof(page)) && programs == 2);
    assert(!report_flash_program(1, page));
    assert(!report_flash_program(REPORT_FLASH_BYTES, page));
    assert(!report_flash_program(UINT32_MAX, page) && programs == 2);
    fail_safe = true;
    assert(!report_flash_program(0, page));
    assert(!report_flash_erase() && erases == 128);
    assert(strstr(diagnostics, "LOG PREPARATION FAILED at flash sector 1/128"));
    puts("Flash adapter: full physical bounds, all 128 journal sectors, protected firmware/Bluetooth, boundary pages and errors passed");
}
