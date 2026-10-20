#ifndef GETTING_THERE_PACKED_BITS_H
#define GETTING_THERE_PACKED_BITS_H
#include <stdbool.h>
#include <stdint.h>
#include "hardware/sync.h"

// Smallest field width that holds 0..max. Pair it with _Static_assert so a
// field can neither be too narrow nor wider than the data needs.
#define BITS_NEEDED(max) ((max) < 2u ? 1u : (max) < 4u ? 2u : (max) < 8u ? 3u : \
                          (max) < 16u ? 4u : (max) < 32u ? 5u : (max) < 64u ? 6u : \
                          (max) < 128u ? 7u : (max) < 256u ? 8u : (max) < 512u ? 9u : 16u)
#define FIELD_MAX(bits) ((1u << (bits)) - 1u)

// A status word is shared between the main loop and BTstack callbacks (which
// run in the async-context IRQ). A read-modify-write on it must not be split
// by that IRQ, so every update masks interrupts around the whole operation.
// Aligned 32-bit reads are single instructions on the RP2040 and need no lock.
static inline uint32_t packed_get(const volatile uint32_t *word, unsigned shift, unsigned bits)
{
    return (*word >> shift) & FIELD_MAX(bits);
}

static inline void packed_put(volatile uint32_t *word, unsigned shift, unsigned bits,
                              uint32_t value)
{
    uint32_t mask = FIELD_MAX(bits) << shift;
    uint32_t irq = save_and_disable_interrupts();
    *word = (*word & ~mask) | ((value << shift) & mask);
    restore_interrupts(irq);
}

static inline void packed_set_bits(volatile uint32_t *word, uint32_t mask)
{
    uint32_t irq = save_and_disable_interrupts();
    *word |= mask;
    restore_interrupts(irq);
}

static inline void packed_clear_bits(volatile uint32_t *word, uint32_t mask)
{
    uint32_t irq = save_and_disable_interrupts();
    *word &= ~mask;
    restore_interrupts(irq);
}

// Fixed-width fields packed back to back with no padding between elements.
#define BITARRAY_BYTES(count, width) (((count) * (width) + 7u) / 8u)

static inline uint32_t bitarray_get(const uint8_t *buffer, unsigned index, unsigned width)
{
    uint32_t value = 0;
    unsigned position = index * width;
    for (unsigned bit = 0; bit < width; ++bit, ++position)
        value |= (uint32_t)((buffer[position >> 3] >> (position & 7u)) & 1u) << bit;
    return value;
}

static inline void bitarray_put(uint8_t *buffer, unsigned index, unsigned width, uint32_t value)
{
    unsigned position = index * width;
    for (unsigned bit = 0; bit < width; ++bit, ++position) {
        uint8_t mask = (uint8_t)(1u << (position & 7u));
        if ((value >> bit) & 1u) buffer[position >> 3] |= mask;
        else buffer[position >> 3] &= (uint8_t)~mask;
    }
}

// RSSI is stored as the magnitude of a negative dBm value: 1..127 needs 7 bits.
// Code 127 is the "no reading" marker and decodes to the firmware's -127 sentinel.
#define RSSI_BITS 7u
#define RSSI_MISSING_CODE 127u
_Static_assert(RSSI_BITS == BITS_NEEDED(RSSI_MISSING_CODE), "RSSI field is not minimal");

static inline uint32_t rssi_encode(int dbm) { return (uint32_t)(-dbm); }
static inline int rssi_decode(uint32_t code) { return -(int)code; }

#endif
