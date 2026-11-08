// Only the adapter/world is exercised. No selected firmware sources are linked.
#include "sim.h"
#include "beacon_adapter.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
float motion_heading(void) { return (float)sim.heading; }
unsigned fault_mask(void) { return 0; }
int firmware_main(void) { abort(); } // An adapter test must never enter firmware.
void sim_ble_tick(void) { }
void sim_ble_stats(void) { }
static const char *ssid1(void) { return "mock AP1"; }
static const char *ssid2(void) { return "mock AP2"; }
static const char *ssid3(void) { return "mock AP3"; }
const NativeBeacon native_beacons[3]={{.ssid=ssid1},{.ssid=ssid2},{.ssid=ssid3}};
static unsigned reports,ids;
static int scan(void *ctx,const cyw43_ev_scan_result_t *result) {
    assert(ctx==&reports); ++reports;
    int id=result->bssid[5]; assert(id>=1 && id<=3);
    ids|=1u<<id;
    const char *ssid=native_beacons[id-1].ssid();
    assert(result->ssid_len==strlen(ssid)); assert(!memcmp(result->ssid,ssid,result->ssid_len));
    return 0;
}
static int fired,owner;
static int64_t alarm_mock(alarm_id_t id,void *data) {
    (void)id; assert(data==&fired); ++fired; owner=sim_device;
    pwm_set_gpio_level(22,400); return fired==1?10000:0;
}
static void reg(uint8_t r,uint8_t *out,size_t n) {
    assert(i2c_write_timeout_us(i2c0,0x29,&r,1,true,5000)==1);
    assert(i2c_read_timeout_us(i2c0,0x29,out,n,false,5000)==(int)n);
}
int main(int argc,char **argv) {
    if(argc==2 && !strcmp(argv[1],"console")) { sim_console_printf("LOG ERROR: saved report incomplete; %d pages\n",2048); return 0; }
    if(argc==3 && !strcmp(argv[1],"validate")) return sim_scenario_json(argv[2])?0:2;
    assert(sim_scenario_json("{\"x\":0,\"y\":0,\"beacons\":[{\"id\":3,\"x\":3,\"y\":0},{\"id\":1,\"x\":1,\"y\":0},{\"id\":2,\"x\":2,\"y\":0}],\"mats\":[{\"id\":2,\"x\":0.08,\"y\":0,\"mat\":\"blue\"}]}"));
    assert(sim.beacons[0].wifi_observed==-127 && sim.beacons[0].ble_observed==-127);
    assert(sim.beacon_count==3 && sim.mat_count==1 && sim.beacons[0].id==3);
    uint8_t address[6]; sim_radio_address(2,address); assert(sim_radio_identity(address)==2);
    assert(sim_radio_visible(3)); sim.radio_range=2.5; assert(!sim_radio_visible(3)); sim.radio_range=5;
    double wifi=sim_wifi_at(2),ble=sim_ble_at(2);
    sim.active_target=3; assert(sim_wifi_at(2)==wifi && sim_ble_at(2)==ble);
    sim.completed_mask=7; assert(sim_radio_visible(1)); sim.completed_mask=0;
    uint16_t rgb[4]; double sx,sy; int mat,band;
    sim_colour_raw(rgb,&sx,&sy,&mat,&band);
    assert(fabs(sx-.08)<1e-12 && sy==0 && mat==2 && band==6);
    assert(rgb[0]==2577 && rgb[1]==518 && rgb[2]==799 && rgb[3]==1222);
    sim.heading=180; sim_colour_raw(rgb,&sx,&sy,&mat,&band); assert(band==3 && mat==2);
    // Outside every mat is the measured floor, even next to a radio.
    sim.x=10; sim_colour_raw(rgb,&sx,&sy,&mat,&band); assert(!mat && band==-1 && rgb[0]==3067);
    sim.x=0; sim.heading=0;
    uint8_t id=0; reg(0xb2,&id,1); assert(id==0x44);
    uint8_t enable[]={0x80,3}; assert(i2c_write_blocking(i2c0,0x29,enable,2,false)==2);
    uint8_t status=0; reg(0xb3,&status,1); assert(status==1);
    uint8_t raw[8]; reg(0xb4,raw,8); assert((raw[0]|raw[1]<<8)==2577);
    sim.colour_enabled=false; assert(i2c_write_blocking(i2c0,0x29,enable,2,false)==-1); sim.colour_enabled=true;
    // Separate bus/register selection preserves the IMU when colour is read.
    uint8_t imu[]={0x75,0x71},read=0,sel=0x75;
    assert(i2c_write_blocking(i2c1,0x68,imu,2,false)==2);
    assert(i2c_write_blocking(i2c1,0x68,&sel,1,true)==1);
    reg(0xb2,&id,1); assert(i2c_read_blocking(i2c1,0x68,&read,1,false)==1 && read==0x71);
    assert(cyw43_wifi_scan(NULL,NULL,&reports,scan)==0); sim.us+=100000;
    assert(!cyw43_wifi_scan_active(NULL)); assert(reports==3 && ids==14);
    assert(sim_beacon(1)->wifi_observed==(int)lround(sim_wifi_at(1)));
    assert(sim_beacon(3)->wifi_observed==(int)lround(sim_wifi_at(3)));
    sim.radio_range=1.5; assert(cyw43_wifi_scan(NULL,NULL,&reports,scan)==0); sim.us+=100000;
    assert(!cyw43_wifi_scan_active(NULL)); assert(reports==4);
    sim_device=2; alarm_id_t timer=add_alarm_in_ms(10,alarm_mock,&fired,false); assert(timer>0);
    sim_device=0; sim.us+=10000; sim_alarms_tick();
    assert(fired==1 && owner==2 && sim.pwm[22]==0 && sim_beacon(2)->pwm[22]==400);
    sim.us+=10000; sim_alarms_tick(); assert(fired==2 && !cancel_alarm(timer));
    timer=add_alarm_in_ms(1,alarm_mock,&fired,false); assert(cancel_alarm(timer));
    // Swept collision stops translation, retains motor commands and gyro turns.
    sim.x=0; sim.y=0; sim.heading=0; sim.pwm[8]=sim.pwm[11]=10000;
    sim_physics(2000000); assert(sim.x<1-.069 && sim.collision && sim.pwm[8]==10000);
    // Telemetry alone never synthesises success from connections or markers.
    sim_beacon(1)->connections=1; sim_report_observe("AP1 BLE connected; discovering payload service\n"); assert(!sim.success);
    sim_report_observe("ROUTE: target=2\n"); assert(sim_active_target()==2);
    sim_report_observe("ROUTE: target=+3\n"); assert(sim_active_target()==2);
    sim_report_observe("ROUTE: target=3 extra\n"); assert(sim_active_target()==2);
    sim_report_observe("ROUTE: confirmed=1,mask=1\n"); assert(sim.completed_mask==1);
    sim_report_observe("AP1 PAYLOAD COMPLETE: verified reply and final acknowledgement\n");
    assert(sim_beacon(1)->payload_events==1 && !sim.success);
    sim_report_observe("ROUTE: confirmed=3,mask=7\n"); assert(sim.completed_mask==1);
    sim_report_observe("ROUTE: confirmed=2,mask=3\n");
    sim_report_observe("ROUTE: confirmed=3,mask=7\n"); assert(sim.completed_mask==7 && !sim.success);
    sim_report_observe("prefix RESULT: ROUTE COMPLETE\n"); assert(!sim.success);
    sim_report_observe("RESULT: ROUTE COMPLETE\n"); assert(sim.success);
    sim_frame(); return 0;
}
