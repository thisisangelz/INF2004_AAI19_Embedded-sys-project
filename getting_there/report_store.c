#include "report_store.h"
#include <string.h>

// Versioned on-flash records. A page is programmed once; a torn write fails CRC.
#define START_MAGIC 0x31474f4cu
#define DATA_MAGIC  0x31415444u
#define END_MAGIC   0x31444e45u
#define PAGE_COUNT (REPORT_FLASH_BYTES / REPORT_PAGE_BYTES)
typedef struct {
    uint32_t magic;
    uint16_t index;
    uint16_t length;
    uint32_t crc;
    char data[REPORT_PAGE_DATA];
} log_page_t;
_Static_assert(sizeof(log_page_t) == REPORT_PAGE_BYTES, "flash record size");
static struct {
    uint16_t next : 12; // 0..2048 including full
    uint16_t active : 1;
} store;
_Static_assert(PAGE_COUNT == 2048, "review journal counter width");

static uint32_t crc_bytes(uint32_t crc, const void *bytes, size_t length)
{
    const uint8_t *p = bytes;
    while (length--) {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static uint32_t page_crc(const log_page_t *p)
{
    uint32_t crc = crc_bytes(UINT32_MAX, p, offsetof(log_page_t, crc));
    return ~crc_bytes(crc, p->data, p->length);
}

static bool write_page(uint32_t magic, const char *data, size_t length)
{
    if (store.next >= PAGE_COUNT || length > REPORT_PAGE_DATA) return false;
    log_page_t page;
    memset(&page, 0xff, sizeof(page));
    page.magic = magic;
    page.index = store.next;
    page.length = (uint16_t)length;
    if (length) memcpy(page.data, data, length);
    page.crc = page_crc(&page);
    uint32_t offset = store.next * REPORT_PAGE_BYTES;
    if (!report_flash_program(offset, &page)) return false;
    log_page_t verify;
    report_flash_read(offset, &verify, sizeof(verify));
    if (memcmp(&page, &verify, sizeof(page))) return false;
    ++store.next;
    return true;
}

bool report_store_begin(void)
{
    store.active = false;
    store.next = 0;
    // Previous run is erased ONLY on explicit GP20 start, never on boot/replay.
    if (!report_flash_erase() || !write_page(START_MAGIC, NULL, 0)) return false;
    store.active = true;
    return true;
}

bool report_store_append(const char *data, size_t length)
{
    if (!store.active) return false;
    while (length) {
        size_t chunk = length > REPORT_PAGE_DATA ? REPORT_PAGE_DATA : length;
        // Reserve the final page for the completed-run marker.
        if (store.next >= PAGE_COUNT - 1 || !write_page(DATA_MAGIC, data, chunk)) {
            store.active = false;
            return false;
        }
        data += chunk;
        length -= chunk;
    }
    return true;
}

bool report_store_finish(void)
{
    if (!store.active) return false;
    bool ok = write_page(END_MAGIC, NULL, 0);
    store.active = false;
    return ok;
}

static void emit_text(report_emit_t emit, const char *text)
{
    emit(text, strlen(text));
}

void report_store_replay(report_emit_t emit)
{
    log_page_t page;
    bool complete = false;
    for (unsigned i = 0; i < PAGE_COUNT; ++i) {
        report_flash_read(i * REPORT_PAGE_BYTES, &page, sizeof(page));
        bool valid = page.index == i && page.length <= REPORT_PAGE_DATA &&
                     page.crc == page_crc(&page);
        if (!i) {
            if (!valid || page.magic != START_MAGIC || page.length) {
                emit_text(emit, "\nNo saved run found (or its start record is damaged).\n");
                return;
            }
            emit_text(emit, "\n========== SAVED RUN ==========\n");
        } else {
            if (!valid) break;
            if (page.magic == END_MAGIC && !page.length) { complete = true; break; }
            if (page.magic != DATA_MAGIC || !page.length) break;
            emit(page.data, page.length);
        }
    }
    emit_text(emit, complete ? "\n[COMPLETE: all recorded run text saved]\n" :
        "\n[INCOMPLETE: recovered valid prefix only; power loss, storage limit or write error. Unsaved tail unavailable.]\n");
    emit_text(emit, "========== END OF SAVED RUN ==========\n");
}
