#include <assert.h>
#include <stdio.h>
#include "getting_there/packed_bits.h"

int main(void)
{
    volatile uint32_t word = 0;
    packed_put(&word, 0, 5, 17);
    packed_put(&word, 5, 3, 6);
    packed_set_bits(&word, 1u << 12);
    assert(packed_get(&word, 0, 5) == 17);
    assert(packed_get(&word, 5, 3) == 6);
    assert((word & (1u << 12)) != 0);

    packed_put(&word, 0, 5, 3);
    assert(packed_get(&word, 0, 5) == 3);
    assert(packed_get(&word, 5, 3) == 6);
    packed_clear_bits(&word, 1u << 12);
    assert((word & (1u << 12)) == 0);
    puts("packed update independence: passed [HOST single-thread model]");
    return 0;
}

