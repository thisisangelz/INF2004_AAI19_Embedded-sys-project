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
static uint8_t imu_regs[256], selected_reg;
static i2c_inst_t bus;
i2c_inst_t *i2c1 = &bus;
int cyw43_state;
static uint64_t scan_due;
static void *scan_context;
static int (*scan_callback)(void *, const cyw43_ev_scan_result_t *);
static double wheel_l(void) { return ((double)sim.pwm[8] - sim.pwm[9]) / 10000 * sim.speed; }
static double wheel_r(void) { return ((double)sim.pwm[11] - sim.pwm[10]) / 10000 * sim.speed; }
double sim_distance(void) { return hypot(sim.x - sim.bx, sim.y - sim.by); }
double sim_wifi(void) {
    return fmax(-126, fmin(-1, sim.wifi_ref - 10 * sim.exponent * log10(fmax(.01, sim_distance()))));
}
double sim_ble(void) {
    // The environment calibration is fixed across branches. A firmware change
    // to its acceptance threshold must not silently move the radio's reference.
    return fmax(-126, fmin(-1, -50.0 - 10 * sim.exponent * log10(fmax(.01, sim_distance()) / sim.ble_radius)));
}
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
    sim_json_string(sim.peer_state); printf(",\"wifi_reports\":%u",sim.wifi_reports); sim_ble_stats(); puts("}");
}
static void finish(const char *reason) {
    sim_frame();
    printf("{\"type\":\"result\",\"success\":%s,\"t\":%.3f,\"reason\":", sim.success ? "true":"false", sim.us/1e6);
    sim_json_string(reason); puts("}"); fflush(stdout); exit(0);
}
static void physics(uint32_t us) {
    double dt=us/1e6, left=wheel_l(), right=wheel_r();
    double v=(left+right)/2, omega=(right-left)/sim.wheelbase;
    double angle=sim.heading*PI/180, da=omega*dt;
    if (fabs(omega)<1e-9) { sim.x+=v*dt*cos(angle); sim.y+=v*dt*sin(angle); }
    else { sim.x+=v/omega*(sin(angle+da)-sin(angle)); sim.y-=v/omega*(cos(angle+da)-cos(angle)); }
    sim.heading+=da*180/PI; sim.gyro_angle+=da*180/PI;
    sim.path+=fabs(v)*dt; sim.closest=fmin(sim.closest,sim_distance()); sim.us+=us;
}
static void wifi_tick(void) {
    if (!sim.wifi_active || sim.us<scan_due) return;
    sim.wifi_active=false;
    const char *ssid=native_beacon_ssid();
    if(!*ssid) return;
    cyw43_ev_scan_result_t result={.rssi=(int16_t)lround(sim_wifi()), .ssid_len=(uint8_t)strlen(ssid),.bssid={2,0,0,0,0,1}};
    memcpy(result.ssid,ssid,result.ssid_len); sim.wifi_observed=result.rssi; ++sim.wifi_reports;
    scan_callback(scan_context,&result);
}
uint64_t time_us_64(void) { return sim.us; }
void sleep_ms(uint32_t ms) {
    uint64_t target=sim.us+(uint64_t)ms*1000;
    while (sim.us<target) {
        uint64_t remaining=target-sim.us;
        physics(remaining>5000 ? 5000 : (uint32_t)remaining);
        wifi_tick(); sim_ble_tick();
        if (sim.us>=sim.next_frame) { sim_frame(); sim.next_frame=sim.us+100000; }
        if (sim.finished) finish(sim.success ? "Firmware confirmed payload" : "Firmware stopped without payload");
        if (sim.us/1e6>=sim.horizon) { sim_log("simulator","Simulation time limit reached; firmware did not finish."); finish("Simulator time limit"); }
    }
}
static void report(const char *format, va_list args) {
    char text[4096]; vsnprintf(text,sizeof(text),format,args);
    if (strstr(text,"Waiting for GP20") && !sim.auto_start) sim.auto_start=sim.us+100000;
    if (strstr(text,"RESULT: PAYLOAD CONFIRMED")) sim.success=true;
    if (strstr(text,"Run ended.")) sim.finished=true;
    sim_log("firmware",text);
}
void report_printf(const char *fmt,...) { va_list a; va_start(a,fmt); report(fmt,a); va_end(a); }
void report_defer_printf(const char *fmt,...) { va_list a; va_start(a,fmt); report(fmt,a); va_end(a); }
void report_init(void) { }
void report_replay(void) { }
void report_flush_usb(void) { }
bool report_begin(void) { sim.started=true; return true; }
bool report_checkpoint(void) { ++sim.checkpoints; return true; }
bool report_finish(void) { return true; }
bool report_ok(void) { return true; }
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
void pwm_set_gpio_level(uint pin,uint16_t level) { if(pin<32) sim.pwm[pin]=level; }
void pwm_set_wrap(uint s,uint16_t w) { (void)s;(void)w; }
void pwm_set_clkdiv(uint s,float d) { (void)s;(void)d; }
void pwm_set_enabled(uint s,bool e) { (void)s;(void)e; }
uint i2c_init(i2c_inst_t *i,uint baud) { (void)i; return baud; }
void i2c_deinit(i2c_inst_t *i) { (void)i; }
int i2c_write_timeout_us(i2c_inst_t *i,uint8_t addr,const uint8_t *src,size_t len,bool ns,uint timeout) {
    (void)i;(void)addr;(void)ns;(void)timeout;
    if(!len) return 0;
    selected_reg=src[0];
    for(size_t k=1;k<len;k++) imu_regs[(uint8_t)(selected_reg+k-1)]=src[k];
    return (int)len;
}
int i2c_read_timeout_us(i2c_inst_t *i,uint8_t addr,uint8_t *dst,size_t len,bool ns,uint timeout) {
    (void)i;(void)addr;(void)ns;(void)timeout;
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
    if(argc!=14) { fprintf(stderr,"Expected 13 numeric scenario parameters\n"); return 2; }
    sim=(Sim){.x=atof(argv[1]),.y=atof(argv[2]),.heading=atof(argv[3]),.bx=atof(argv[4]),.by=atof(argv[5]),
        .speed=atof(argv[6]),.wheelbase=atof(argv[7]),.wifi_ref=atof(argv[8]),.exponent=atof(argv[9]),
        .ble_radius=atof(argv[10]),.stop_at=atof(argv[11]),.horizon=atof(argv[12]),.corrupt_reply=atoi(argv[13]),
        .wifi_observed=-127,.ble_observed=-127,.peer_state="Advertising"};
    sim.closest=sim_distance(); imu_regs[0x75]=0x71;
    setvbuf(stdout,NULL,_IOFBF,65536); native_beacon_boot(); sim_frame();
    firmware_main(); finish("Firmware returned"); return 0;
}
