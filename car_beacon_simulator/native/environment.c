// Physical world only: no target selection, navigation or arrival decisions.
#include "sim.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846
SimBeacon *sim_beacon(int id) {
    for(unsigned i=0;i<sim.beacon_count;++i) if(sim.beacons[i].id==id) return &sim.beacons[i];
    return NULL;
}
double sim_distance_to(int id) {
    SimBeacon *b=sim_beacon(id);
    return b ? hypot(sim.x-b->x,sim.y-b->y) : INFINITY;
}
double sim_distance(void) { return hypot(sim.x-sim.bx,sim.y-sim.by); }
static double signal(double reference,double distance) {
    return fmax(-126,fmin(-1,reference-10*sim.exponent*log10(fmax(.01,distance))));
}
double sim_wifi_at(int id) { return signal(sim.wifi_ref,sim_distance_to(id)); }
double sim_ble_at(int id) { return signal(-50.0,sim_distance_to(id)/sim.ble_radius); }
double sim_wifi(void) { return signal(sim.wifi_ref,sim_distance()); }
double sim_ble(void) { return signal(-50.0,sim_distance()/sim.ble_radius); }
bool sim_radio_visible(int id) { return sim_beacon(id) && sim_distance_to(id)<=sim.radio_range; }
void sim_radio_address(int id,uint8_t address[6]) {
    const uint8_t base[]={2,0,0,0,0,(uint8_t)id}; memcpy(address,base,6);
}
int sim_radio_identity(const uint8_t address[6]) {
    for(unsigned i=0;i<sim.beacon_count;++i) {
        uint8_t candidate[6]; sim_radio_address(sim.beacons[i].id,candidate);
        if(!memcmp(address,candidate,6)) return sim.beacons[i].id;
    }
    return 0;
}
// Measured C,R,G,B means, never synthesised from classification thresholds.
// docs/calibration/raw/2026-11-06-pink-round1.txt and
// docs/calibration/raw/2026-11-07-{blue,purple}-round2.txt.
// Index 0 is white edge; subsequent indices are numbered inward rings.
static const uint16_t pink[6][4]={
    {7808,2550,2585,2410},{6274,2148,1999,1930},{4707,1792,1402,1401},
    {3651,1573,993,1025},{2638,1320,635,668},{1892,1116,384,408}};
static const uint16_t blue[7][4]={
    {14078,4000,4820,4738},{8493,2290,2898,3026},{7105,1765,2410,2715},
    {6650,1493,2235,2739},{4685,942,1541,2091},{3355,658,1074,1560},{2577,518,799,1222}};
static const uint16_t purple[7][4]={
    {15831,4694,5276,5197},{8368,2514,2731,2843},{6783,2029,2134,2415},
    {5264,1570,1589,1967},{3183,954,912,1250},{2323,685,640,958},{2137,595,564,944}};
void sim_colour_raw(uint16_t out[4],double *sx,double *sy,int *mat_id,int *band) {
    *sx=sim.x+sim.sensor_offset*cos(sim.heading*PI/180);
    *sy=sim.y+sim.sensor_offset*sin(sim.heading*PI/180);
    const uint16_t floor[]={3067,1113,1023,833}; memcpy(out,floor,sizeof(floor));
    *mat_id=0; *band=-1;
    double nearest=INFINITY;
    for(unsigned i=0;i<sim.mat_count;++i) {
        SimMat *m=&sim.mats[i]; double d=hypot(*sx-m->x,*sy-m->y);
        // Nearest centre wins overlap, then lowest mat ID. Independent of target.
        if(d>sim.mat_radius || d>nearest || (d==nearest && *mat_id && m->id>*mat_id)) continue;
        const uint16_t (*table)[4]=!strcmp(m->mat,"pink")?pink:!strcmp(m->mat,"blue")?blue:purple;
        int count=!strcmp(m->mat,"pink")?6:7;
        // Equal radial band widths are illustrative geometry, not measured sizes.
        int ring=(int)((1-d/sim.mat_radius)*count); if(ring>=count) ring=count-1;
        memcpy(out,table[ring],sizeof(floor)); *mat_id=m->id; *band=ring; nearest=d;
    }
}
void sim_physics(uint32_t us) {
    double dt=us/1e6;
    double left=((double)sim.pwm[8]-sim.pwm[9])/10000*sim.speed;
    double right=((double)sim.pwm[11]-sim.pwm[10])/10000*sim.speed;
    double v=(left+right)/2, omega=(right-left)/sim.wheelbase;
    double angle=sim.heading*PI/180, da=omega*dt, x=sim.x,y=sim.y;
    if(fabs(omega)<1e-9) { x+=v*dt*cos(angle); y+=v*dt*sin(angle); }
    else { x+=v/omega*(sin(angle+da)-sin(angle)); y-=v/omega*(cos(angle+da)-cos(angle)); }
    sim.collision=false;
    // The wheel midpoint is a point body. Swept segments stop at each circular
    // beacon box (box_radius); no unmeasured robot footprint is invented.
    double fraction=1, dx=x-sim.x,dy=y-sim.y, a=dx*dx+dy*dy;
    for(unsigned i=0;i<sim.beacon_count && sim.box_radius>0 && a>0;++i) {
        double ox=sim.x-sim.beacons[i].x,oy=sim.y-sim.beacons[i].y;
        double c=ox*ox+oy*oy-sim.box_radius*sim.box_radius, b=2*(ox*dx+oy*dy);
        if(c<0) { if(b<0) fraction=0; continue; } // Allow an overlapping start to move outward.
        double disc=b*b-4*a*c;
        if(disc>=0) { double t=(-b-sqrt(disc))/(2*a); if(t>=0 && t<fraction) fraction=fmax(0,t-1e-8); }
    }
    if(fraction<1) sim.collision=true;
    sim.x+=dx*fraction; sim.y+=dy*fraction;
    sim.heading+=da*180/PI; sim.gyro_angle+=da*180/PI;
    sim.path+=hypot(dx,dy)*fraction; sim.closest=fmin(sim.closest,sim_distance()); sim.us+=us;
}
