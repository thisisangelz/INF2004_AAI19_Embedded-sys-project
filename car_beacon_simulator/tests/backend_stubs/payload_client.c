// Event spy standing in for payload_client.c. No firmware procedures or decisions.
#include "btstack.h"
#include <stdio.h>
#include <string.h>
#define TRANSFER_PACKET_SIZE 32
#define BEACON_SERVICE_UUID 0xff20
#define MSG_REPLY_DATA 9
#define RSSI_SAMPLE_COUNT 12
#define CLOSE_RSSI_DBM (-58)
#define F_PAYLOAD_VERIFIED 1
static struct { unsigned sample_count,close_count; } counters;
static hci_con_handle_t connection_handle=42;
static gatt_client_characteristic_t characteristic;
extern int mock_threshold;
int payload_client_close_threshold(void) { return mock_threshold; }
static unsigned current_state(void) { return 0; }
static int filtered_rssi(void) { return -127; }
static bool flag(int f) { (void)f; return false; }
static void transfer_write_u16(uint8_t *p,uint16_t n) { p[0]=(uint8_t)n; p[1]=(uint8_t)(n>>8); }
extern void mock_hci(uint8_t *,uint16_t);
extern void mock_gatt(uint8_t *,uint16_t);
static void hci_handler(uint8_t type,uint16_t channel,uint8_t *p,uint16_t n) { (void)type;(void)channel; mock_hci(p,n); }
static void gatt_handler(uint8_t type,uint16_t channel,uint8_t *p,uint16_t n) { (void)type;(void)channel; mock_gatt(p,n); }
