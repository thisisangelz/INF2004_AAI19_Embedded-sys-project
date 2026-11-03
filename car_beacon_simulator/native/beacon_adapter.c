// Isolated translation unit: the selected branch's beacon firmware owns its
// protocol state, advertisement, reply bytes, validation and completion.
#include <stdarg.h>
#include <stdio.h>
#include <setjmp.h>
#include "sim.h"
#include "beacon_adapter.h"
#include "btstack.h"

static int beacon_printf(const char *format, ...);
static void beacon_sleep(uint32_t ms);
static void beacon_att_init(const uint8_t *,att_read_callback_t,att_write_callback_t);
static void beacon_hci_register(btstack_packet_callback_registration_t *);
static int beacon_power(HCI_POWER_MODE);
#define main beacon_firmware_main
#define printf beacon_printf
#define sleep_ms beacon_sleep
#define att_server_init beacon_att_init
#define hci_add_event_handler beacon_hci_register
#define hci_power_control beacon_power
#include "beacon_server.c"
#undef main
#undef printf
#undef sleep_ms
#undef att_server_init
#undef hci_add_event_handler
#undef hci_power_control

static jmp_buf boot_return;
static bool booting, stack_started, advertised;
static att_write_callback_t server_write;
static btstack_packet_handler_t server_hci,server_att;
static const uint8_t *radio_ad;
static uint8_t radio_ad_length;
static char radio_ssid[33];
static uint8_t notification_out[TRANSFER_PACKET_SIZE];
static uint16_t notification_length;

static int beacon_printf(const char *fmt, ...) {
    char text[4096]; va_list a; va_start(a,fmt);
    int n=vsnprintf(text,sizeof(text),fmt,a); va_end(a);
    sim_log("beacon",text); return n;
}
static void beacon_sleep(uint32_t ms) {
    (void)ms;
    // Beacon boots before the robot run. Yield after startup reaches its idle
    // loop; its registered event callbacks remain live throughout the run.
    if(booting && stack_started) longjmp(boot_return,1);
}
static void beacon_att_init(const uint8_t *db,att_read_callback_t read,att_write_callback_t write) {
    (void)db;(void)read; server_write=write;
}
static void beacon_hci_register(btstack_packet_callback_registration_t *r) { server_hci=r->callback; }
static int beacon_power(HCI_POWER_MODE mode) {
    (void)mode; stack_started=true;
    uint8_t packet[]={BTSTACK_EVENT_STATE,1,HCI_STATE_WORKING};
    if(server_hci) server_hci(HCI_EVENT_PACKET,0,packet,sizeof(packet));
    return 0;
}
void att_server_register_packet_handler(btstack_packet_handler_t handler) { server_att=handler; }
void gap_advertisements_set_params(uint16_t lo,uint16_t hi,uint8_t type,uint8_t own,
                                  bd_addr_t direct,uint8_t channels,uint8_t filter) {
    (void)lo;(void)hi;(void)type;(void)own;(void)direct;(void)channels;(void)filter;
}
void gap_advertisements_set_data(uint8_t length,uint8_t *data) { radio_ad=data; radio_ad_length=length; }
void gap_advertisements_enable(int enabled) { advertised=enabled!=0; }
void att_server_request_can_send_now_event(hci_con_handle_t handle) {
    uint8_t packet[]={ATT_EVENT_CAN_SEND_NOW,2,(uint8_t)handle,(uint8_t)(handle>>8)};
    if(server_att) server_att(HCI_EVENT_PACKET,0,packet,sizeof(packet));
}
uint8_t att_server_notify(hci_con_handle_t handle,uint16_t attribute,const uint8_t *data,uint16_t length) {
    (void)handle;(void)attribute;
    if(length>sizeof(notification_out)) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    memcpy(notification_out,data,length); notification_length=length;
    if(length>=3 && data[0]==MSG_REPLY_DATA) sim.tx_bytes+=data[2];
    return 0;
}
void cyw43_arch_enable_ap_mode(const char *ssid,const char *password,uint32_t auth) {
    (void)password;(void)auth; snprintf(radio_ssid,sizeof(radio_ssid),"%s",ssid);
}
void cyw43_arch_gpio_put(uint pin,bool enabled) { (void)pin;(void)enabled; }
int cyw43_wifi_pm(int *state,uint32_t mode) { (void)state;(void)mode; return 0; }
uint32_t cyw43_pm_value(uint8_t mode,uint16_t sleep,uint8_t li,uint8_t dtim,uint8_t assoc) {
    (void)mode;(void)sleep;(void)li;(void)dtim;(void)assoc; return 0;
}
bool powerbank_load_start(void) { return true; } // physical power load is outside simulation

static void update_stats(void) {
    sim.peer_state=beacon_status_name(beacon_status);
    if(beacon_status==BEACON_STATUS_FILE_VERIFIED) sim.peer_verified=true;
    if(beacon_status==BEACON_STATUS_COMPLETE) sim.peer_complete=true;
    if(beacon_status==BEACON_STATUS_HANDSHAKE) sim.peer_verified=sim.peer_complete=false;
}
void native_beacon_boot(void) {
    booting=true;
    if(!setjmp(boot_return)) (void)beacon_firmware_main();
    booting=false; update_stats();
}
void native_beacon_connect(bool connected) {
    if(!server_hci) return;
    if(connected) {
        uint8_t p[21]={HCI_EVENT_LE_META,19,HCI_SUBEVENT_LE_CONNECTION_COMPLETE,0,42,0};
        server_hci(HCI_EVENT_PACKET,0,p,sizeof(p));
    } else {
        uint8_t p[]={HCI_EVENT_DISCONNECTION_COMPLETE,4,0,42,0,0x13};
        server_hci(HCI_EVENT_PACKET,0,p,sizeof(p));
    }
    update_stats();
}
int native_beacon_enable_notifications(void) {
    uint8_t config[]={1,0};
    if(!server_write) return ATT_ERROR_UNLIKELY_ERROR;
    int result=server_write(42,ATT_CHARACTERISTIC_0xFF21_01_CLIENT_CONFIGURATION_HANDLE,0,0,config,2);
    update_stats(); return result;
}
int native_beacon_write(uint8_t *data,uint16_t length) {
    if(!server_write) return ATT_ERROR_UNLIKELY_ERROR;
    unsigned before=received_file_length;
    int result=server_write(42,ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE,0,0,data,length);
    if(received_file_length>before) sim.rx_bytes+=received_file_length-before;
    sim.received_length=received_file_length; update_stats(); return result;
}
uint16_t native_beacon_take_notification(uint8_t *out) {
    uint16_t length=notification_length;
    memcpy(out,notification_out,length); notification_length=0; return length;
}
uint16_t native_beacon_value_handle(void) { return ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE; }
const uint8_t *native_beacon_advertisement(uint8_t *length) {
    *length=advertised?radio_ad_length:0; return radio_ad;
}
const char *native_beacon_ssid(void) { return radio_ssid; }
