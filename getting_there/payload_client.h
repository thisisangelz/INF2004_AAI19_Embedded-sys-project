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
// Once the colour sensor confirms the robot is on the mat, BLE only has to show
// that AP1 is in range. On-mat filtered readings were -64 to -73 dBm.
#ifndef COLOUR_ARRIVAL_BLE_DBM
#define COLOUR_ARRIVAL_BLE_DBM (-85)
#endif
void payload_client_init(void);
// Main-thread helpers synchronised with BTstack callbacks using the CYW43
// async-context lock. Quiescent means idle, disconnected, no listener, no
// queued start/abort or unacknowledged scan commands, and no cleanup fault.
bool payload_client_quiescent(void);
// Atomically select ID and clear all old target qualification, addresses,
// service/characteristic, counters and reply state. Also refuses active Wi-Fi.
// Caller stops motors and drains wifi_sampler before calling. No BLE commands.
bool payload_client_reset_target_qualification(uint8_t id);
// Main-thread only; checked during attempts so the shared mission deadline
// aborts BLE and still waits for cleanup before any movement or target change.
void payload_client_set_continue_callback(bool (*callback)(void));
bool payload_client_ready(void);
bool payload_client_start_failed(void);
// Filtered BLE average that counts as close. Ignored while an attempt runs.
void payload_client_set_close_threshold(int dbm);
int payload_client_close_threshold(void);
// Called from the main loop once per attempt when the close-range check has
// passed (immediately for a retry of an already qualified beacon).
void payload_client_set_close_callback(void (*callback)(void));
// Call with motors stopped and no Wi-Fi scan running. Returns after BLE cleanup.
payload_result_t payload_client_attempt(void);
// Retry the same qualified AP without moving or repeating the RSSI gate.
// Valid only immediately after PAYLOAD_RETRY, with motors still stopped.
payload_result_t payload_client_retry(void);
const uint8_t *payload_client_data(uint16_t *length);
#endif
