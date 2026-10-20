#ifndef GETTING_THERE_FAULTS_H
#define GETTING_THERE_FAULTS_H
#include <stdbool.h>
#include <stdint.h>

// One bit per fault condition, all packed in a single shared mask. Faults may be
// raised from the main loop, BTstack callbacks and button polling, so raise and
// clear are atomic read-modify-writes.
typedef enum {
    FAULT_IMU,            // IMU initialisation or read failed
    FAULT_GYRO_SATURATED, // gyro Z near its range limit; heading untrusted
    FAULT_HEADING,        // straight-line or turn heading error out of bounds
    FAULT_TURN,           // turn timed out, did not start or missed tolerance
    FAULT_WIFI_LOST,      // AP1 missing from the latest Wi-Fi sample
    FAULT_WIFI_SCAN,      // Wi-Fi scan could not start, stopped or timed out
    FAULT_BLE_RADIO,      // BLE scan command or cleanup failed
    FAULT_PAYLOAD,        // BLE payload transfer failed
    FAULT_STOP_LATCHED,   // GP21 stop pressed; latched until reset
    FAULT_RUN_LIMIT,      // 15 minute run limit reached
    FAULT_NO_DIRECTION,   // no probe heading had a valid RSSI
    FAULT_STEP_LIMIT,     // approach steps exhausted without a payload
    FAULT_COUNT
} fault_t;

void fault_raise(fault_t fault);
void fault_clear(fault_t fault);
bool fault_test(fault_t fault);
uint16_t fault_mask(void);
const char *fault_name(fault_t fault);

#endif
