// Deterministic Pico alarm boundary; callbacks retain their device's PWM owner.
#include "sim.h"
#include "pico/time.h"
#include <limits.h>
static struct { alarm_id_t id; uint64_t due; alarm_callback_t callback; void *data; int device; } alarms[32];
static alarm_id_t next_id;
alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t callback, void *data, bool fire_if_past) {
    if(!ms && !fire_if_past) return 0;
    for(unsigned i=0;i<32;++i) if(!alarms[i].id) {
        if(next_id==INT32_MAX) next_id=0;
        alarms[i].id=++next_id; alarms[i].due=sim.us+(uint64_t)ms*1000;
        alarms[i].callback=callback; alarms[i].data=data; alarms[i].device=sim_device;
        return alarms[i].id;
    }
    return -1;
}
bool cancel_alarm(alarm_id_t id) {
    for(unsigned i=0;i<32;++i) if(alarms[i].id==id) { alarms[i].id=0; return true; }
    return false;
}
uint64_t sim_next_alarm(void) {
    uint64_t next=UINT64_MAX;
    for(unsigned i=0;i<32;++i) if(alarms[i].id && alarms[i].due<next) next=alarms[i].due;
    return next;
}
void sim_alarms_tick(void) {
    for(unsigned i=0;i<32;++i) if(alarms[i].id && alarms[i].due<=sim.us) {
        alarm_id_t id=alarms[i].id;
        int owner=sim_device; sim_device=alarms[i].device;
        int64_t delay=alarms[i].callback(id,alarms[i].data);
        sim_device=owner;
        if(alarms[i].id!=id) continue;
        if(!delay) alarms[i].id=0;
        else if(delay>0) alarms[i].due=sim.us+(uint64_t)delay;
        else alarms[i].due+=(uint64_t)(-delay);
    }
}
