#ifndef SIM_BEACON_ADAPTER_H
#define SIM_BEACON_ADAPTER_H
#include <stdint.h>
#include <stdbool.h>
void native_beacon_boot(void);
void native_beacon_connect(bool connected);
int native_beacon_enable_notifications(void);
int native_beacon_write(uint8_t *data,uint16_t length);
uint16_t native_beacon_take_notification(uint8_t *out);
uint16_t native_beacon_value_handle(void);
const uint8_t *native_beacon_advertisement(uint8_t *length);
const char *native_beacon_ssid(void);
#endif
