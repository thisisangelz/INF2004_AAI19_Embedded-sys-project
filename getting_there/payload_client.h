#ifndef GETTING_THERE_PAYLOAD_CLIENT_H
#define GETTING_THERE_PAYLOAD_CLIENT_H
#include <stdbool.h>
#include <stdint.h>
// Stronger trial threshold after v11 qualified at -66 dBm after one step.
// This is a signal requirement, not a measured stopping distance.
#ifndef AP1_CLOSE_RSSI_DBM
#define AP1_CLOSE_RSSI_DBM (-58)
#endif
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
// Retry the same qualified AP without moving or repeating the RSSI gate.
// Valid only immediately after PAYLOAD_RETRY, with motors still stopped.
payload_result_t payload_client_retry(void);
const uint8_t *payload_client_data(uint16_t *length);
#endif
