#ifndef SIM_BEACON_ADAPTER_H
#define SIM_BEACON_ADAPTER_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    void (*boot)(void);
    void (*connect)(bool);
    int (*enable_notifications)(void);
    int (*write)(uint8_t *, uint16_t);
    uint16_t (*take_notification)(uint8_t *);
    uint16_t (*value_handle)(void);
    uint16_t (*service_start)(void);
    uint16_t (*service_end)(void);
    const uint8_t *(*advertisement)(uint8_t *);
    const char *(*ssid)(void);
} NativeBeacon;
#define SIM_DECLARE_BEACON(P) \
    void P##native_beacon_boot(void); \
    void P##native_beacon_connect(bool); \
    int P##native_beacon_enable_notifications(void); \
    int P##native_beacon_write(uint8_t *,uint16_t); \
    uint16_t P##native_beacon_take_notification(uint8_t *); \
    uint16_t P##native_beacon_value_handle(void); \
    uint16_t P##native_beacon_service_start(void); \
    uint16_t P##native_beacon_service_end(void); \
    const uint8_t *P##native_beacon_advertisement(uint8_t *); \
    const char *P##native_beacon_ssid(void)
#define SIM_BEACON_ENTRY(P) {P##native_beacon_boot,P##native_beacon_connect, \
    P##native_beacon_enable_notifications,P##native_beacon_write,P##native_beacon_take_notification, \
    P##native_beacon_value_handle,P##native_beacon_service_start,P##native_beacon_service_end, \
    P##native_beacon_advertisement,P##native_beacon_ssid}
extern const NativeBeacon native_beacons[3];
SIM_DECLARE_BEACON();
#endif
