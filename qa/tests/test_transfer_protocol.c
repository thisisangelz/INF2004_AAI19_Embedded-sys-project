#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ble_transfer/transfer_protocol.h"

int main(void)
{
    uint8_t bytes[8] = {0};
    transfer_write_u16(bytes, 0xabcd);
    assert(bytes[0] == 0xcd && bytes[1] == 0xab);
    assert(transfer_read_u16(bytes) == 0xabcd);

    transfer_write_u32(bytes, 0x89abcdefu);
    assert(bytes[0] == 0xef && bytes[1] == 0xcd && bytes[2] == 0xab && bytes[3] == 0x89);
    assert(transfer_read_u32(bytes) == 0x89abcdefu);

    const uint8_t vector[] = "123456789";
    assert(transfer_crc32(vector, strlen((const char *)vector)) == 0xcbf43926u);
    assert(TRANSFER_DATA_BYTES + 3 == TRANSFER_PACKET_SIZE);
    assert(TRANSFER_MAX_FILE_SIZE == 256);
    puts("wire encoding and CRC reference vector: passed [HOST]");
    return 0;
}

