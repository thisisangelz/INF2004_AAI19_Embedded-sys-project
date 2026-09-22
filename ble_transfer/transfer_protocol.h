#ifndef PICO_BEACON_TRANSFER_PROTOCOL_H
#define PICO_BEACON_TRANSFER_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define BEACON_SERVICE_UUID 0xFF20
#define BEACON_CHARACTERISTIC_UUID 0xFF21
#define BEACON_PROTOCOL_VERSION 1

#define TRANSFER_MAX_FILE_SIZE 256
#define TRANSFER_PACKET_SIZE 20
#define TRANSFER_DATA_BYTES 17

enum transfer_message_type {
    MSG_HELLO = 0x01,
    MSG_HELLO_ACK = 0x02,
    MSG_FILE_START = 0x03,
    MSG_FILE_READY = 0x04,
    MSG_FILE_DATA = 0x05,
    MSG_FILE_END = 0x06,
    MSG_FILE_RECEIVED = 0x07,
    MSG_REPLY_REQUEST = 0x08,
    MSG_REPLY_START = 0x09,
    MSG_REPLY_NEXT = 0x0A,
    MSG_REPLY_DATA = 0x0B,
    MSG_REPLY_END = 0x0C,
    MSG_COMPLETE = 0x0D,
    MSG_COMPLETE_ACK = 0x0E,
    MSG_ERROR = 0x7F,
};

static inline void transfer_write_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static inline uint16_t transfer_read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void transfer_write_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static inline uint32_t transfer_read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline uint32_t transfer_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

#endif
