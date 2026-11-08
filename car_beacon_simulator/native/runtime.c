// Hardware boundary only: navigation, motor control and the BLE client are
// compiled from an immutable snapshot of the selected firmware commit.
#include "sim.h"
#include "beacon_adapter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "motion.h"
#include "faults.h"

#define PI 3.14159265358979323846
Sim sim;
static uint8_t imu_regs[256], selected_reg, colour_regs[32], colour_reg;
static i2c_inst_t buses[2];
i2c_inst_t *i2c0=&buses[0], *i2c1=&buses[1];
int sim_device;
int cyw43_state;
static uint64_t scan_due;
static void *scan_context;
static int (*scan_callback)(void *, const cyw43_ev_scan_result_t *);
static double wheel_l(void) { return ((double)sim.pwm[8] - sim.pwm[9]) / 10000 * sim.speed; }
static double wheel_r(void) { return ((double)sim.pwm[11] - sim.pwm[10]) / 10000 * sim.speed; }
void sim_json_string(const char *s) {
    putchar('"');
    for (; *s; ++s) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') { putchar('\\'); putchar(c); }
        else if (c == '\n') fputs("\\n", stdout);
        else if (c == '\r') fputs("\\r", stdout);
        else if (c == '\t') fputs("\\t", stdout);
        else if (c < 32) printf("\\u%04x", c);
        else putchar(c);
    }
    putchar('"');
}
void sim_log(const char *source, const char *message) {
    printf("{\"type\":\"log\",\"t\":%.3f,\"source\":", sim.us / 1e6);
    sim_json_string(source); fputs(",\"message\":", stdout); sim_json_string(message); puts("}");
}
int sim_console_printf(const char *fmt,...) {
    char text[4096]; va_list args; va_start(args,fmt);
    int n=vsnprintf(text,sizeof(text),fmt,args); va_end(args);
    sim_log("firmware-console",text); return n;
}
static void sim_world_frame(void) {
    printf(",\"active_target\":%d,\"completed_mask\":%u,\"collision\":%s,\"beacons\":[",sim_active_target(),sim.completed_mask,sim.collision?"true":"false");
    for(unsigned i=0;i<sim.beacon_count;++i) {
        SimBeacon *b=&sim.beacons[i]; const NativeBeacon *n=&native_beacons[b->id-1];
        if(i) putchar(',');
        printf("{\"id\":%d,\"x\":%.6f,\"y\":%.6f,\"mat\":",b->id,b->x,b->y); sim_json_string(b->mat);
        fputs(",\"ssid\":",stdout); sim_json_string(n->ssid());
        printf(",\"bssid\":\"02:00:00:00:00:%02x\",\"wifi\":%.6f,\"ble\":%.6f,\"visible\":%s,\"peer_state\":",
            b->id,sim_wifi_at(b->id),sim_ble_at(b->id),sim_radio_visible(b->id)?"true":"false"); sim_json_string(b->peer_state);
        printf(",\"wifi_observed\":%d,\"ble_observed\":%d",b->wifi_observed,b->ble_observed);
        printf(",\"verified\":%s,\"complete\":%s,\"rx\":%u,\"tx\":%u,\"rx_bytes\":%u,\"tx_bytes\":%u,"
               "\"connections\":%u,\"verified_events\":%u,\"complete_events\":%u,\"payload_events\":%u,\"buzzer\":%s}",
            b->verified?"true":"false",b->complete?"true":"false",b->rx_bytes,b->tx_bytes,b->rx_bytes,b->tx_bytes,
            b->connections,b->verified_events,b->complete_events,b->payload_events,b->pwm[22]?"true":"false");
    }
    uint16_t rgb[4]; double sx,sy; int mat,band; sim_colour_raw(rgb,&sx,&sy,&mat,&band);
    printf("],\"sensor\":{\"x\":%.6f,\"y\":%.6f,\"enabled\":%s,\"mat_id\":%d,\"band\":%d,\"c\":%u,\"r\":%u,\"g\":%u,\"b\":%u}",
        sx,sy,sim.colour_enabled?"true":"false",mat,band,rgb[0],rgb[1],rgb[2],rgb[3]);
}
void sim_frame(void) {
    printf("{\"type\":\"frame\",\"t\":%.3f,\"x\":%.6f,\"y\":%.6f,\"heading\":%.5f,"
           "\"gyro_heading\":%.5f,\"distance\":%.6f,\"closest\":%.6f,\"path\":%.6f,"
           "\"wifi\":%.6f,\"ble\":%.6f,\"wifi_observed\":%d,\"ble_observed\":%d,"
           "\"left\":%.6f,\"right\":%.6f,\"left_pwm\":%.2f,\"right_pwm\":%.2f,"
           "\"wifi_scans\":%u,\"ble_reports\":%u,\"ble_attempts\":%u,\"faults\":%u,"
           "\"buzzer\":%s,\"wifi_active\":%s,\"rx_bytes\":%u,\"tx_bytes\":%u,"
           "\"peer_verified\":%s,\"peer_complete\":%s,\"peer_state\":",
           sim.us/1e6, sim.x, sim.y, sim.heading, (double)motion_heading(), sim_distance(), sim.closest, sim.path,
           sim_wifi(), sim_ble(), sim.wifi_observed, sim.ble_observed,
           wheel_l(), wheel_r(), ((double)sim.pwm[8]-sim.pwm[9])/100, ((double)sim.pwm[11]-sim.pwm[10])/100,
           sim.scans, sim.ads, sim.ble_attempts, (unsigned)fault_mask(), sim.pwm[22] ? "true":"false",
           sim.wifi_active ? "true":"false", sim.rx_bytes, sim.tx_bytes,
           sim.peer_verified ? "true":"false", sim.peer_complete ? "true":"false");
    sim_json_string(sim.peer_state); sim_world_frame(); printf(",\"wifi_reports\":%u",sim.wifi_reports); sim_ble_stats(); puts("}");
}
static void finish(const char *reason) {
    sim_frame();
    printf("{\"type\":\"result\",\"success\":%s,\"t\":%.3f,\"reason\":", sim.success ? "true":"false", sim.us/1e6);
    sim_json_string(reason); puts("}"); fflush(stdout); exit(0);
}
static void wifi_tick(void) {
    if (!sim.wifi_active || sim.us<scan_due) return;
    sim.wifi_active=false;
    // Deliver every visible physical radio. Firmware chooses which SSID/BSSID
    // to accept; changing its build target never creates/removes an AP.
    for(unsigned i=0;i<sim.beacon_count;++i) {
        SimBeacon *b=&sim.beacons[i]; if(!sim_radio_visible(b->id)) continue;
        const char *ssid=native_beacons[b->id-1].ssid(); if(!*ssid) continue;
        cyw43_ev_scan_result_t result={.rssi=(int16_t)lround(sim_wifi_at(b->id)),.ssid_len=(uint8_t)strlen(ssid)};
        sim_radio_address(b->id,result.bssid); memcpy(result.ssid,ssid,result.ssid_len);
        b->wifi_observed=sim.wifi_observed=result.rssi; ++sim.wifi_reports; scan_callback(scan_context,&result);
    }
}
uint64_t time_us_64(void) { return sim.us; }
void sleep_ms(uint32_t ms) {
    uint64_t target=sim.us+(uint64_t)ms*1000;
    while (sim.us<target) {
        uint64_t remaining=target-sim.us;
        uint64_t step=remaining>5000?5000:remaining, alarm=sim_next_alarm();
        if(alarm<=sim.us) { sim_alarms_tick(); continue; }
        if(alarm-sim.us<step) step=alarm-sim.us;
        sim_physics((uint32_t)step);
        sim_alarms_tick(); wifi_tick(); sim_ble_tick();
        if (sim.us>=sim.next_frame) { sim_frame(); sim.next_frame=sim.us+100000; }
        if (sim.finished) finish(sim.success ? "Firmware confirmed payload" : "Firmware stopped without payload");
        if (sim.us/1e6>=sim.horizon) { sim_log("simulator","Simulation time limit reached; firmware did not finish."); finish("Simulator time limit"); }
    }
}
void stdio_init_all(void) { }
bool stdio_usb_connected(void) { return false; }
void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin,bool out) { (void)pin;(void)out; }
void gpio_pull_up(uint pin) { (void)pin; }
void gpio_set_function(uint pin,uint fn) { (void)pin;(void)fn; }
bool gpio_get(uint pin) {
    if(pin==20) {
        if(!sim.auto_start) sim.auto_start=sim.us+100000;
        return !(sim.us>=sim.auto_start && sim.us<sim.auto_start+50000);
    }
    if(pin==21) return !(sim.stop_at>=0 && sim.us/1e6>=sim.stop_at);
    return true;
}
uint32_t clock_get_hz(int clock) { (void)clock; return 125000000; }
pwm_config pwm_get_default_config(void) { return (pwm_config){0}; }
void pwm_config_set_clkdiv(pwm_config *c,float d) { (void)c;(void)d; }
void pwm_config_set_wrap(pwm_config *c,uint16_t w) { c->wrap=w; }
void pwm_init(uint s,const pwm_config *c,bool e) { (void)s;(void)c;(void)e; }
uint pwm_gpio_to_slice_num(uint pin) { return (pin>>1)&7u; }
void pwm_set_gpio_level(uint pin,uint16_t level) {
    if(pin>=32) return;
    if(sim_device) { SimBeacon *b=sim_beacon(sim_device); if(b) b->pwm[pin]=level; }
    else sim.pwm[pin]=level;
}
void pwm_set_wrap(uint s,uint16_t w) { (void)s;(void)w; }
void pwm_set_clkdiv(uint s,float d) { (void)s;(void)d; }
void pwm_set_enabled(uint s,bool e) { (void)s;(void)e; }
uint i2c_init(i2c_inst_t *i,uint baud) { (void)i; return baud; }
void i2c_deinit(i2c_inst_t *i) { (void)i; }
int i2c_write_timeout_us(i2c_inst_t *i,uint8_t addr,const uint8_t *src,size_t len,bool ns,uint timeout) {
    (void)ns;(void)timeout;
    if(!len) return 0;
    if(i==i2c0) {
        if(addr!=0x29 || !sim.colour_enabled) return -1;
        colour_reg=src[0]&0x1f;
        for(size_t k=1;k<len;++k) colour_regs[(colour_reg+k-1)&0x1f]=src[k];
        return (int)len;
    }
    if(i!=i2c1 || addr!=0x68) return -1;
    selected_reg=src[0];
    for(size_t k=1;k<len;k++) imu_regs[(uint8_t)(selected_reg+k-1)]=src[k];
    return (int)len;
}
int i2c_read_timeout_us(i2c_inst_t *i,uint8_t addr,uint8_t *dst,size_t len,bool ns,uint timeout) {
    (void)ns;(void)timeout;
    if(i==i2c0) {
        if(addr!=0x29 || !sim.colour_enabled) return -1;
        uint16_t rgb[4]; double sx,sy; int mat,band; sim_colour_raw(rgb,&sx,&sy,&mat,&band);
        colour_regs[0x12]=0x44;
        colour_regs[0x13]=(colour_regs[0]&3)==3?1:0;
        for(unsigned k=0;k<4;++k) { colour_regs[0x14+k*2]=(uint8_t)rgb[k]; colour_regs[0x15+k*2]=(uint8_t)(rgb[k]>>8); }
        for(size_t k=0;k<len;++k) dst[k]=colour_regs[(colour_reg+k)&0x1f];
        return (int)len;
    }
    if(i!=i2c1 || addr!=0x68) return -1;
    if(selected_reg==0x43 && len>=6) {
        double rate=sim.us==sim.gyro_last_us ? (wheel_r()-wheel_l())/sim.wheelbase*180/PI :
            (sim.gyro_angle-sim.gyro_last_angle)/((sim.us-sim.gyro_last_us)/1e6);
        sim.gyro_last_angle=sim.gyro_angle; sim.gyro_last_us=sim.us;
        static const double sensitivity[]={131,65.5,32.8,16.4};
        int raw=(int)lround(rate*sensitivity[(imu_regs[0x1b]>>3)&3]);
        if(raw>32767) raw=32767; if(raw< -32768) raw=-32768;
        imu_regs[0x47]=(uint8_t)((uint16_t)raw>>8); imu_regs[0x48]=(uint8_t)raw;
    }
    for(size_t k=0;k<len;k++) dst[k]=imu_regs[(uint8_t)(selected_reg+k)];
    return (int)len;
}
int i2c_write_blocking(i2c_inst_t *i,uint8_t addr,const uint8_t *src,size_t len,bool ns) {
    return i2c_write_timeout_us(i,addr,src,len,ns,5000);
}
int i2c_read_blocking(i2c_inst_t *i,uint8_t addr,uint8_t *dst,size_t len,bool ns) {
    return i2c_read_timeout_us(i,addr,dst,len,ns,5000);
}
int cyw43_arch_init(void) { return 0; }
void cyw43_arch_deinit(void) { sim.wifi_active=false; }
void cyw43_arch_enable_sta_mode(void) { }
int cyw43_wifi_scan(int *s,cyw43_wifi_scan_options_t *o,void *ctx,int (*cb)(void *,const cyw43_ev_scan_result_t *)) {
    (void)s;(void)o;
    if(sim.wifi_active) return -1;
    sim.wifi_active=true; ++sim.scans; scan_due=sim.us+100000;
    scan_context=ctx; scan_callback=cb; return 0;
}
bool cyw43_wifi_scan_active(int *s) { (void)s; wifi_tick(); return sim.wifi_active; }
int firmware_main(void);
int main(int argc,char **argv) {
    char *allocated=NULL;
    if(argc==3 && !strcmp(argv[1],"--scenario")) {
        FILE *file=fopen(argv[2],"rb"); if(!file) { perror("scenario"); return 2; }
        allocated=malloc(65537); if(!allocated) { fclose(file); return 2; }
        size_t n=fread(allocated,1,65536,file); bool oversized=fgetc(file)!=EOF; fclose(file);
        allocated[n]=0;
        if(oversized || !sim_scenario_json(allocated)) { free(allocated); fputs("Invalid JSON scenario\n",stderr); return 2; }
        free(allocated);
    } else if(argc==3 && !strcmp(argv[1],"--scenario-json")) {
        if(!sim_scenario_json(argv[2])) { fputs("Invalid JSON scenario\n",stderr); return 2; }
    } else if(argc==14) {
        char json[4096];
        // Legacy positional CLI is translated through the same validated parser.
        snprintf(json,sizeof(json),"{\"x\":%s,\"y\":%s,\"heading\":%s,\"bx\":%s,\"by\":%s,"
            "\"speed\":%s,\"wheelbase\":%s,\"wifi_ref\":%s,\"exponent\":%s,\"ble_radius\":%s,"
            "\"stop_at\":%s,\"horizon\":%s,\"corrupt_reply\":%s}",
            argv[1],argv[2],argv[3],argv[4],argv[5],argv[6],argv[7],argv[8],argv[9],argv[10],argv[11],argv[12],argv[13]);
        if(!sim_scenario_json(json)) { fputs("Invalid numeric scenario\n",stderr); return 2; }
    } else { fputs("Use --scenario PATH, --scenario-json JSON, or 13 legacy numeric parameters\n",stderr); return 2; }
    for(unsigned i=0;i<sim.beacon_count;++i) if(!native_beacons[sim.beacons[i].id-1].boot) {
        fputs("Physical beacon has no selected source configuration\n",stderr); return 2;
    }
    sim.closest=sim_distance(); imu_regs[0x75]=0x71;
    setvbuf(stdout,NULL,_IOFBF,65536);
    for(unsigned i=0;i<sim.beacon_count;++i) native_beacons[sim.beacons[i].id-1].boot();
    sim_frame(); firmware_main(); finish("Firmware returned"); return 0;
}
