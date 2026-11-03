// Compile the real BLE client and connect it to the selected beacon firmware.
// All proximity filtering, packet parsing, CRC checks and final acknowledgement
// decisions below this boundary belong to the selected firmware source.
#include "sim.h"
#include "beacon_adapter.h"
#include <math.h>
#include "payload_client.c"

const hci_cmd_t hci_le_set_scan_enable={0x200c,"11"};
static enum { EV_NONE,EV_SCAN,EV_CONNECT,EV_SERVICE,EV_CHAR,EV_CCC,EV_WRITE,EV_DISCONNECT } pending;
static uint64_t due,next_ad;
static bool scanning;
static uint8_t written[TRANSFER_PACKET_SIZE];
static uint16_t written_length;
static void schedule(int event) { pending=event; due=sim.us+20000; }
int sim_ble_threshold(void) { return CLOSE_RSSI_DBM; }
void sim_ble_stats(void) {
    static const char *names[]={"Idle","Starting","Scanning","Stopping scan","Connecting","Finding service",
        "Finding characteristic","Enabling notifications","Sending HELLO","Waiting for HELLO ACK",
        "Sending file metadata","Waiting for file ready","Sending file","Sending file end","Waiting for CRC",
        "Requesting reply","Waiting for reply metadata","Requesting chunk","Receiving reply",
        "Sending COMPLETE","Waiting for COMPLETE ACK","Stopping scan","Disconnecting"};
    unsigned state=(unsigned)current_state();
    fputs(",\"ble_state\":",stdout); sim_json_string(state<sizeof(names)/sizeof(*names)?names[state]:"Unknown");
    printf(",\"ble_samples\":%u,\"ble_close_count\":%u,\"ble_filtered\":%d,\"ble_threshold\":%d,\"payload_verified\":%s",
        counters.sample_count,counters.close_count,counters.sample_count>=RSSI_SAMPLE_COUNT?filtered_rssi():-127,
        CLOSE_RSSI_DBM,flag(F_PAYLOAD_VERIFIED)?"true":"false");
}
void hci_dump_log(int level,const char *fmt,...) { (void)level;(void)fmt; }
void l2cap_init(void) { }
void sm_init(void) { }
void sm_set_io_capabilities(io_capability_t v) { (void)v; }
void att_server_init(const uint8_t *db,att_read_callback_t r,att_write_callback_t w) { (void)db;(void)r;(void)w; }
void gatt_client_init(void) { }
void hci_add_event_handler(btstack_packet_callback_registration_t *r) { (void)r; }
int hci_power_control(HCI_POWER_MODE m) {
    (void)m; uint8_t p[]={BTSTACK_EVENT_STATE,1,HCI_STATE_WORKING};
    hci_handler(HCI_EVENT_PACKET,0,p,sizeof(p)); return 0;
}
void btstack_run_loop_execute_on_main_thread(btstack_context_callback_registration_t *r) { r->callback(r->context); }
void gap_set_scan_parameters(uint8_t t,uint16_t i,uint16_t w) { (void)t;(void)i;(void)w; }
void gap_start_scan(void) { scanning=true; ++sim.ble_attempts; next_ad=sim.us+100000; schedule(EV_SCAN); }
void gap_stop_scan(void) { scanning=false; schedule(EV_SCAN); }
uint8_t gap_connect(const bd_addr_t a,bd_addr_type_t t) { (void)a;(void)t; schedule(EV_CONNECT); return 0; }
uint8_t gap_connect_cancel(void) { pending=EV_NONE; return 0; }
uint8_t gap_disconnect(hci_con_handle_t h) { (void)h; schedule(EV_DISCONNECT); return 0; }
uint8_t gatt_client_write_value_of_characteristic(btstack_packet_handler_t cb,hci_con_handle_t h,uint16_t vh,uint16_t len,uint8_t *v) {
    (void)cb;(void)h;(void)vh;
    if(len>sizeof(written)) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    memcpy(written,v,len); written_length=len; schedule(EV_WRITE); return 0;
}
uint8_t gatt_client_discover_primary_services_by_uuid16(btstack_packet_handler_t cb,hci_con_handle_t h,uint16_t u) {
    (void)cb;(void)h;(void)u; schedule(EV_SERVICE); return 0;
}
uint8_t gatt_client_discover_characteristics_for_service_by_uuid16(btstack_packet_handler_t cb,hci_con_handle_t h,gatt_client_service_t *s,uint16_t u) {
    (void)cb;(void)h;(void)s;(void)u; schedule(EV_CHAR); return 0;
}
uint8_t gatt_client_write_client_characteristic_configuration(btstack_packet_handler_t cb,hci_con_handle_t h,gatt_client_characteristic_t *c,uint16_t conf) {
    (void)cb;(void)h;(void)c;(void)conf; schedule(EV_CCC); return 0;
}
void gatt_client_listen_for_characteristic_value_updates(gatt_client_notification_t *n,btstack_packet_handler_t cb,hci_con_handle_t h,gatt_client_characteristic_t *c) { (void)n;(void)cb;(void)h;(void)c; }
void gatt_client_stop_listening_for_characteristic_value_updates(gatt_client_notification_t *n) { (void)n; }
void gatt_client_deserialize_service(const uint8_t *p,int o,gatt_client_service_t *s) {
    (void)p;(void)o; *s=(gatt_client_service_t){.uuid16=BEACON_SERVICE_UUID};
}
void gatt_client_deserialize_characteristic(const uint8_t *p,int o,gatt_client_characteristic_t *c) {
    (void)p;(void)o; *c=(gatt_client_characteristic_t){.value_handle=native_beacon_value_handle()};
}
static void notify_peer(const uint8_t *v,uint16_t len) {
#ifdef SIM_EXTENDED_GATT
    const unsigned extra=4;
#else
    const unsigned extra=0;
#endif
    uint8_t p[64]={GATT_EVENT_NOTIFICATION,(uint8_t)(len+6+extra)};
    transfer_write_u16(p+2,connection_handle); transfer_write_u16(p+4+extra,characteristic.value_handle);
    transfer_write_u16(p+6+extra,len); memcpy(p+8+extra,v,len);
    gatt_handler(HCI_EVENT_PACKET,0,p,(uint16_t)(8+extra+len));
}
static void query_done(uint8_t status) {
#ifdef SIM_EXTENDED_GATT
    uint8_t p[]={GATT_EVENT_QUERY_COMPLETE,7,(uint8_t)connection_handle,(uint8_t)(connection_handle>>8),0,0,0,0,status};
#else
    uint8_t p[]={GATT_EVENT_QUERY_COMPLETE,3,(uint8_t)connection_handle,(uint8_t)(connection_handle>>8),status};
#endif
    gatt_handler(HCI_EVENT_PACKET,0,p,sizeof(p));
}
static void exchange(void) {
    uint8_t request[TRANSFER_PACKET_SIZE];
    uint16_t length=written_length;
    memcpy(request,written,length);
    int status=native_beacon_write(request,length);
    uint8_t response[TRANSFER_PACKET_SIZE];
    uint16_t response_length=native_beacon_take_notification(response);
    if(sim.corrupt_reply && response_length>3 && response[0]==MSG_REPLY_DATA) response[3]^=1;
    query_done((uint8_t)status);
    if(!status && response_length) notify_peer(response,response_length);
}
void sim_ble_tick(void) {
    if(pending && sim.us>=due) {
        int event=pending; pending=EV_NONE;
        switch(event) {
        case EV_SCAN: {
            uint8_t p[]={HCI_EVENT_COMMAND_COMPLETE,4,1,0x0c,0x20,0};
            hci_handler(HCI_EVENT_PACKET,0,p,sizeof(p)); break;
        }
        case EV_CONNECT: {
            native_beacon_connect(true);
            uint8_t p[21]={HCI_EVENT_LE_META,19,HCI_SUBEVENT_LE_CONNECTION_COMPLETE,0,42,0};
            hci_handler(HCI_EVENT_PACKET,0,p,sizeof(p)); break;
        }
        case EV_SERVICE: {
            uint8_t p[32]={GATT_EVENT_SERVICE_QUERY_RESULT,30};
            gatt_handler(HCI_EVENT_PACKET,0,p,sizeof(p)); query_done(0); break;
        }
        case EV_CHAR: {
            uint8_t p[40]={GATT_EVENT_CHARACTERISTIC_QUERY_RESULT,38};
            gatt_handler(HCI_EVENT_PACKET,0,p,sizeof(p)); query_done(0); break;
        }
        case EV_CCC: query_done((uint8_t)native_beacon_enable_notifications()); break;
        case EV_WRITE: exchange(); break;
        case EV_DISCONNECT: {
            uint8_t p[]={HCI_EVENT_DISCONNECTION_COMPLETE,4,0,42,0,0x13};
            hci_handler(HCI_EVENT_PACKET,0,p,sizeof(p));
            native_beacon_connect(false); break;
        }
        default: break;
        }
    }
    if(scanning && sim.us>=next_ad) {
        next_ad=sim.us+100000;
        uint8_t length; const uint8_t *ad=native_beacon_advertisement(&length);
        if(!length || length>31) return;
        ++sim.ads; sim.ble_observed=(int)lround(sim_ble());
        uint8_t p[43]={GAP_EVENT_ADVERTISING_REPORT,(uint8_t)(10+length),0,0,1,2,3,4,5,6,
            (uint8_t)sim.ble_observed,length};
        memcpy(p+12,ad,length);
        hci_handler(HCI_EVENT_PACKET,0,p,(uint16_t)(12+length));
    }
}
