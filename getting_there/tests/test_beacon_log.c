#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define HOST_TEST_IRQ_HOOKS
static size_t capture_write(const void *, size_t, size_t, FILE *);
#define fwrite capture_write
#include "../../ble_transfer_powerbank/beacon_log.c"
#undef fwrite

static bool connected, callback_active;
static uint32_t masked;
static unsigned writes, usb_queries;
static size_t write_limit, captured_length;
static char captured[16384];
static void (*after_unlock)(void), (*during_write)(void);

uint32_t host_test_save_and_disable_interrupts(void)
{ uint32_t old = masked; masked = 1; return old; }
void host_test_restore_interrupts(uint32_t old)
{
    masked = old;
    if (!masked && after_unlock) {
        void (*callback)(void) = after_unlock;
        after_unlock = NULL;
        callback_active = true; callback(); callback_active = false;
    }
}
bool stdio_usb_connected(void)
{ assert(!masked && !callback_active); ++usb_queries; return connected; }
static size_t capture_write(const void *data, size_t size, size_t length, FILE *stream)
{
    (void)stream;
    assert(!masked && !callback_active && size == 1);
    assert(length < BEACON_LOG_LINE_BYTES);
    ++writes;
    if (during_write) { callback_active = true; during_write(); callback_active = false; }
    if (length > write_limit) length = write_limit;
    assert(captured_length + length < sizeof(captured));
    memcpy(captured + captured_length, data, length);
    captured_length += length; captured[captured_length] = 0;
    return length;
}
static void reset(void)
{
    after_unlock = during_write = NULL;
    connected = true; callback_active = false; masked = 0;
    writes = usb_queries = 0; captured_length = 0; captured[0] = 0;
    write_limit = SIZE_MAX;
    beacon_log_init();
}
static void flood(void)
{
    for (unsigned i = 0; i < BEACON_LOG_QUEUE_LINES; ++i)
        beacon_log_printf("new %u\n", i);
}
static void drain(void)
{
    for (unsigned i = 0; i < 32 && (count || output.length || dropped || truncated); ++i)
        beacon_log_flush();
    assert(!count && !output.length && !dropped && !truncated);
}

int main(void)
{
    reset();
    char content[256]; memset(content, 'X', sizeof(content));
    callback_active = true;
    beacon_log_printf("AP3 FILE CONTENT: %.*s\n", (int)sizeof(content), content);
    callback_active = false;
    assert(!usb_queries && !writes && count == 1 && !truncated);
    beacon_log_flush();
    char expected[320];
    snprintf(expected, sizeof(expected), "AP3 FILE CONTENT: %.*s\n", (int)sizeof(content), content);
    assert(!strcmp(captured, expected));

    // Full queue drops complete new lines, counts truncation independently,
    // retains old lines offline and never spends more than two writes/flush.
    reset(); connected = false;
    for (unsigned i = 0; i < BEACON_LOG_QUEUE_LINES; ++i) beacon_log_printf("old %u\n", i);
    char long_text[500]; memset(long_text, 'L', sizeof(long_text));
    beacon_log_printf("%.*s", (int)sizeof(long_text), long_text);
    beacon_log_printf("dropped too\n");
    beacon_log_flush();
    assert(!writes && count == BEACON_LOG_QUEUE_LINES && dropped == 2 && truncated == 1);
    connected = true; beacon_log_flush();
    assert(writes == 2 && strstr(captured, "dropped=2 truncated=1") && strstr(captured, "old 0\n"));
    drain();
    assert(strstr(captured, "old 7\n") && !strstr(captured, "dropped too") && !strstr(captured, "LLLL"));

    reset(); beacon_log_printf("%.*s", (int)sizeof(long_text), long_text);
    assert(truncated == 1 && queue[head].length == BEACON_LOG_LINE_BYTES - 1);
    assert(queue[head].text[BEACON_LOG_LINE_BYTES - 2] == '\n');
    drain(); assert(strstr(captured, "dropped=0 truncated=1"));

    // Reuse a just-popped ring slot immediately after IRQs are restored. The
    // USB writer must see the original private snapshot, with IRQs enabled.
    reset();
    for (unsigned i = 0; i < BEACON_LOG_QUEUE_LINES; ++i) beacon_log_printf("old %u\n", i);
    after_unlock = flood;
    beacon_log_flush();
    assert(writes == 2 && !strncmp(captured, "old 0\nold 1\n", 12));
    assert(dropped == BEACON_LOG_QUEUE_LINES - 1);
    drain(); assert(strstr(captured, "dropped=7 truncated=0"));

    // Producers can keep adding work while output runs: one call still ends.
    reset(); beacon_log_printf("first\n"); during_write = flood;
    beacon_log_flush(); assert(writes == BEACON_LOG_FLUSH_LINES);
    unsigned before = writes;
    beacon_log_flush(); assert(writes - before == BEACON_LOG_FLUSH_LINES);
    during_write = NULL; drain();

    // Short/zero writes and disconnected USB retain the exact unsent suffix.
    reset(); beacon_log_printf("abcdef\n"); write_limit = 3;
    beacon_log_flush(); assert(!strcmp(captured, "abc") && output_offset == 3);
    write_limit = 0; beacon_log_flush(); assert(output_offset == 3);
    connected = false; before = writes; beacon_log_flush(); assert(writes == before);
    connected = true; write_limit = SIZE_MAX; drain(); assert(!strcmp(captured, "abcdef\n"));

    // A partly emitted summary must also survive, without losing later losses.
    reset(); dropped = 2; truncated = 1; write_limit = 5;
    beacon_log_flush(); assert(!dropped && !truncated && output_offset == 5);
    dropped = 3; write_limit = SIZE_MAX; drain();
    assert(strstr(captured, "dropped=2 truncated=1") && strstr(captured, "dropped=3 truncated=0"));
    reset(); flood(); dropped = truncated = UINT32_MAX;
    beacon_log_printf("%.*s", (int)sizeof(long_text), long_text);
    assert(dropped == UINT32_MAX && truncated == UINT32_MAX);
    drain(); assert(strstr(captured, "dropped=>=4294967295 truncated=>=4294967295"));
    puts("Beacon log: bounded IRQ queue/flush, 256-byte content, overflow/truncation, saturation, interrupted copying and short writes passed");
    return 0;
}
