#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "getting_there/packed_bits.h"

int main(void)
{
    assert(BITS_NEEDED(0) == 1);
    assert(BITS_NEEDED(1) == 1);
    assert(BITS_NEEDED(2) == 2);
    assert(BITS_NEEDED(127) == 7);
    assert(BITS_NEEDED(128) == 8);
    assert(BITS_NEEDED(511) == 9);
    assert(FIELD_MAX(7) == 127);

    uint8_t data[BITARRAY_BYTES(12, RSSI_BITS)];
    memset(data, 0xa5, sizeof(data));
    for (unsigned index = 0; index < 12; ++index)
        bitarray_put(data, index, RSSI_BITS, index * 11);
    for (unsigned index = 0; index < 12; ++index)
        assert(bitarray_get(data, index, RSSI_BITS) == index * 11);
    assert(sizeof(data) == 11);

    for (int dbm = -126; dbm <= -1; ++dbm)
        assert(rssi_decode(rssi_encode(dbm)) == dbm);
    assert(rssi_decode(RSSI_MISSING_CODE) == -127);
    puts("packed field boundaries and RSSI round trips: passed [HOST]");
    return 0;
}

