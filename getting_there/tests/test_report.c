#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../report_store.c"
static size_t capture_usb_write(const void *data, size_t size, size_t count, FILE *stream);
#define fwrite capture_usb_write
#include "../run_report.c"
#undef fwrite

static uint8_t flash_image[REPORT_FLASH_BYTES];
static uint8_t saved_image[REPORT_FLASH_BYTES];
static char output[REPORT_FLASH_BYTES + 1024];
static size_t output_length;
static int torn_after = -1;
static bool erase_fails;
static bool inject_callback;
static unsigned erase_calls, write_calls;
static bool usb_connected, callback_active;
static unsigned usb_queries;
static char live_usb[16384];
static size_t live_usb_length;

bool stdio_usb_connected(void) { assert(!callback_active); ++usb_queries; return usb_connected; }
static size_t capture_usb_write(const void *data, size_t size, size_t count, FILE *stream)
{
    (void)stream;
    assert(!callback_active && size * count < sizeof(live_usb) - live_usb_length);
    memcpy(live_usb + live_usb_length, data, size * count);
    live_usb_length += size * count; live_usb[live_usb_length]=0;
    return count;
}
void report_flash_read(uint32_t offset, void *data, size_t length)
{
    assert(offset + length <= sizeof(flash_image));
    memcpy(data, flash_image + offset, length);
}
bool report_flash_erase(void)
{
    ++erase_calls;
    if (erase_fails) return false;
    memset(flash_image, 0xff, sizeof(flash_image));
    return true;
}
bool report_flash_program(uint32_t offset, const void *page)
{
    ++write_calls;
    if (inject_callback) {
        inject_callback = false;
        report_defer_printf("callback during checkpoint\n");
    }
    assert(offset % REPORT_PAGE_BYTES == 0 && offset + REPORT_PAGE_BYTES <= sizeof(flash_image));
    size_t size = torn_after < 0 ? REPORT_PAGE_BYTES : (size_t)torn_after;
    const uint8_t *p = page;
    for (size_t i = 0; i < size; ++i) {
        assert((flash_image[offset + i] & p[i]) == p[i]);
        flash_image[offset + i] &= p[i];
    }
    return torn_after < 0;
}
static void capture(const char *data, size_t length)
{
    assert(output_length + length < sizeof(output));
    memcpy(output + output_length, data, length);
    output_length += length;
    output[output_length] = 0;
}
static void replay(void)
{
    output_length = 0;
    output[0] = 0;
    report_store_replay(capture);
}
static void reboot(void)
{
    memset(&store, 0, sizeof(store)); // RAM gone; flash unchanged
    report_init();
}

int main(void)
{
    memset(flash_image, 0xff, sizeof(flash_image));
    reboot(); usb_connected=true; callback_active=true; usb_queries=0;
    report_defer_printf("Bluetooth callback queued\n");
    assert(!usb_queries && !live_usb_length);
    callback_active=false; report_flush_usb();
    assert(strstr(live_usb,"Bluetooth callback queued"));
    assert(report_begin() && report_finish()); replay();
    assert(strstr(output,"Bluetooth callback queued"));
    reboot(); live_usb_length=0; live_usb[0]=0;
    char expected_usb[8192]={0}; size_t expected_length=0;
    for (unsigned i=0;i<400;++i) {
        char line[32]; int n=snprintf(line,sizeof(line),"deferred %03u\n",i);
        memcpy(expected_usb+expected_length,line,(size_t)n); expected_length+=(size_t)n;
        callback_active=true; report_defer_printf("%s",line); callback_active=false;
        if (i%17u==16u) report_flush_usb();
    }
    while (usb_length) report_flush_usb();
    assert(live_usb_length==expected_length && !memcmp(live_usb,expected_usb,expected_length));
    assert(report_begin() && report_finish()); replay();
    assert(strstr(output,expected_usb));
    usb_connected=false;
    memset(flash_image, 0xff, sizeof(flash_image));
    reboot();
    replay();
    assert(strstr(output, "No saved run"));
    report_printf("boot configuration\n");
    assert(report_begin());
    report_printf("RUN ONE\n");
    assert(report_finish());
    memcpy(saved_image, flash_image, sizeof(saved_image));
    unsigned writes = write_calls, erases = erase_calls;
    for (int n = 0; n < 3; ++n) {
        reboot();
        report_printf("new boot, must not change old saved text\n");
        replay();
        assert(strstr(output, "boot configuration\nRUN ONE\n"));
        assert(strstr(output, "[COMPLETE:"));
        assert(!strstr(output, "new boot"));
        assert(!memcmp(saved_image, flash_image, sizeof(saved_image)));
    }
    assert(writes == write_calls && erases == erase_calls);
    assert(report_begin()); // new GP20 start replaces old run
    report_printf("RUN TWO\n");
    assert(report_finish());
    replay();
    assert(strstr(output, "RUN TWO") && !strstr(output, "RUN ONE"));

    reboot(); assert(report_begin());
    report_printf("foreground\n");
    inject_callback = true;
    assert(report_checkpoint() && report_finish());
    replay();
    assert(strstr(output, "foreground\ncallback during checkpoint\n"));
    assert(strstr(output, "[COMPLETE:"));

    // Entire long report, not just the old 32 KiB tail.
    reboot();
    assert(report_begin());
    char line[301];
    memset(line, 'X', 299); line[299] = '\n'; line[300] = 0;
    for (unsigned i = 0; i < 400; ++i) {
        report_printf("%s", line);
        if (i % 50 == 49) assert(report_checkpoint());
    }
    assert(report_finish());
    reboot(); replay();
    char *text = strstr(output, "========== SAVED RUN ==========\n");
    assert(text);
    text += strlen("========== SAVED RUN ==========\n");
    for (unsigned i = 0; i < 400; ++i) assert(!memcmp(text + i * 300, line, 300));
    assert(strstr(text + 120000, "[COMPLETE:"));

    // Power loss discards unflushed RAM only, retains explicit incomplete prefix.
    reboot(); assert(report_begin());
    report_printf("saved prefix\n"); assert(report_checkpoint());
    report_printf("unsaved tail\n");
    reboot(); replay();
    assert(strstr(output, "saved prefix") && !strstr(output, "unsaved tail"));
    assert(strstr(output, "[INCOMPLETE:"));

    // Cut power at every byte boundary of a fully occupied data page.
    assert(report_store_begin());
    assert(report_store_append("prefix\n", 7));
    memcpy(saved_image, flash_image, sizeof(saved_image));
    char data[REPORT_PAGE_DATA];
    memset(data, 'Q', sizeof(data));
    for (int cut = 0; cut < 256; ++cut) {
        memcpy(flash_image, saved_image, sizeof(saved_image));
        store.next = 2; store.active = true; torn_after = cut;
        assert(!report_store_append(data, sizeof(data)));
        replay();
        assert(strstr(output, "prefix\n") && !strstr(output, "QQ"));
        assert(strstr(output, "[INCOMPLETE:"));
    }
    torn_after = -1;
    assert(report_store_begin());
    assert(report_store_append("intact\n", 7));
    assert(report_store_finish());
    flash_image[REPORT_PAGE_BYTES + 12] ^= 1; // CRC detects corrupt data
    replay();
    assert(!strstr(output, "intact") && strstr(output, "[INCOMPLETE:"));

    // Overlong printf / RAM overflow never claims a complete run.
    reboot(); assert(report_begin());
    char oversized[600]; memset(oversized, 'L', 599); oversized[599] = 0;
    report_printf("%s", oversized);
    assert(!report_ok() && !report_finish());
    replay(); assert(strstr(output, "[INCOMPLETE:"));
    reboot(); assert(report_begin());
    for (unsigned i = 0; i < 110; ++i) report_printf("%s", line);
    assert(!report_ok() && !report_finish());
    replay(); assert(strstr(output, "[INCOMPLETE:"));

    // Finite flash journal fills without wrapping into firmware/old entries.
    assert(report_store_begin());
    for (unsigned i = 0; i < PAGE_COUNT - 2; ++i)
        assert(report_store_append(data, sizeof(data)));
    assert(!report_store_append("extra", 5) && !report_store_finish());
    replay(); assert(strstr(output, "[INCOMPLETE:"));
    erase_fails = true;
    reboot(); assert(!report_begin() && !report_ok());
    puts("Report: reboot persistence, previous-run isolation, 120 KB full replay, power cuts, CRC, overflow and write errors passed");
    return 0;
}
