// Test radio dispatch with spies only. There is no actual firmware in this binary.
#include "sim.h"
#include "beacon_adapter.h"
#include "btstack.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
Sim sim;
int sim_device;
int mock_threshold=-58;
void sim_json_string(const char *s) { (void)s; }
void reverse_bytes(const uint8_t *src,uint8_t *dest,int len) { for(int i=0;i<len;++i) dest[i]=src[len-1-i]; }
void reverse_bd_addr(const bd_addr_t src,bd_addr_t dest) { for(unsigned i=0;i<6;++i) dest[i]=src[5-i]; }
static uint8_t ads[3][2]={{1,1},{1,2},{1,3}};
static const uint8_t *ad1(uint8_t *n) { *n=2; return ads[0]; }
static const uint8_t *ad2(uint8_t *n) { *n=2; return ads[1]; }
static const uint8_t *ad3(uint8_t *n) { *n=2; return ads[2]; }
static unsigned connections[3],writes[3],seen,ad_count,cancelled,disconnects;
static uint8_t last_status;
static void c1(bool on) { connections[0]+=on; }
static void c2(bool on) { connections[1]+=on; }
static void c3(bool on) { connections[2]+=on; }
static int w1(uint8_t *p,uint16_t n) { (void)p;(void)n;++writes[0];return 0; }
static int w2(uint8_t *p,uint16_t n) { (void)p;(void)n;++writes[1];return 0; }
static int w3(uint8_t *p,uint16_t n) { (void)p;(void)n;++writes[2];return 0; }
static uint16_t notification(uint8_t *p) { (void)p;return 0; }
static void boot(void) { }
const NativeBeacon native_beacons[3]={
 {.boot=boot,.advertisement=ad1,.connect=c1,.write=w1,.take_notification=notification},
 {.boot=boot,.advertisement=ad2,.connect=c2,.write=w2,.take_notification=notification},
 {.boot=boot,.advertisement=ad3,.connect=c3,.write=w3,.take_notification=notification}};
void mock_hci(uint8_t *p,uint16_t n) {
    (void)n;
    if(p[0]==GAP_EVENT_ADVERTISING_REPORT) {
        bd_addr_t address; gap_event_advertising_report_get_address(p,address);
        int id=sim_radio_identity(address); assert(id>=1 && id<=3);
        assert(gap_event_advertising_report_get_data(p)[1]==id);
        seen|=1u<<id; ++ad_count;
    } else if(p[0]==HCI_EVENT_LE_META) { last_status=p[3]; cancelled+=p[3]==ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER; }
    else if(p[0]==HCI_EVENT_DISCONNECTION_COMPLETE) ++disconnects;
}
void mock_gatt(uint8_t *p,uint16_t n) { (void)p;(void)n; }
int main(void) {
    assert(sim_scenario_json("{\"x\":0,\"y\":0,\"beacons\":[{\"id\":1,\"x\":1,\"y\":0},{\"id\":2,\"x\":2,\"y\":0},{\"id\":3,\"x\":3,\"y\":0}]}"));
    double signal=sim_ble_at(2); assert(sim_ble_threshold()==-58); mock_threshold=-85;
    assert(sim_ble_threshold()==-85 && sim_ble_at(2)==signal);
    gap_start_scan(); sim.us+=100000; sim_ble_tick(); assert(ad_count==3 && seen==14);
    assert(sim_beacon(1)->ble_observed!=sim_beacon(3)->ble_observed);
    sim.active_target=3; sim.us+=100000; sim_ble_tick(); assert(ad_count==6);
    sim.radio_range=2.5; sim.us+=100000; sim_ble_tick(); assert(ad_count==8);
    gap_stop_scan(); sim.us+=20000; sim_ble_tick();
    bd_addr_t address; sim_radio_address(2,address);
    assert(!gap_connect(address,BD_ADDR_TYPE_LE_PUBLIC));
    sim.us+=20000; sim_ble_tick(); assert(sim.connected_id==2 && connections[1]==1 && !connections[0] && !last_status);
    uint8_t request[]={1,2};
    gatt_client_write_value_of_characteristic(NULL,42,1,sizeof(request),request);
    sim.us+=20000; sim_ble_tick(); assert(writes[1]==1 && writes[0]==0 && writes[2]==0);
    gap_disconnect(42); sim.us+=20000; sim_ble_tick(); assert(!sim.connected_id && disconnects==1);
    assert(!gap_connect(address,BD_ADDR_TYPE_LE_PUBLIC)); gap_connect_cancel();
    sim.us+=20000; sim_ble_tick(); assert(cancelled==1 && !sim.connected_id && connections[1]==1);
    // Same address maps to same radio despite a target change; visibility can fail it.
    sim_radio_address(3,address); assert(!gap_connect(address,BD_ADDR_TYPE_LE_PUBLIC));
    sim.us+=20000; sim_ble_tick(); assert(last_status==ERROR_CODE_CONNECTION_FAILED_TO_BE_ESTABLISHED && !sim.connected_id);
    memset(address,7,sizeof(address)); assert(gap_connect(address,BD_ADDR_TYPE_LE_PUBLIC)!=0);
    return 0;
}
