#ifndef GETTING_THERE_PAYLOAD_CLIENT_H
#define GETTING_THERE_PAYLOAD_CLIENT_H
#include <stdbool.h>
#include <stdint.h>
typedef enum {
    PAYLOAD_NOT_CLOSE,
    PAYLOAD_RETRY,
    PAYLOAD_COMPLETE,
    PAYLOAD_FATAL
} payload_result_t;
void payload_client_init(void);
bool payload_client_ready(void);
bool payload_client_start_failed(void);
// Call with motors stopped and no Wi-Fi scan running. Returns after BLE cleanup.
payload_result_t payload_client_attempt(void);
const uint8_t *payload_client_data(uint16_t *length);
#endif
