#include "sim.h"
#include <stdio.h>
#include <string.h>
#ifdef SIM_TARGET_API
#include "target_beacon.h"
#endif
int sim_active_target(void) {
#if ROUTE_BEACON_COUNT || THREE_BEACON_ROUTE
    return sim.active_target; // Only an explicit firmware marker changes route progress.
#elif defined(SIM_DYNAMIC_TARGET_API)
    return target_beacon_id();
#elif defined(SIM_TARGET_API)
    return TARGET_BEACON_ID;
#else
    return 1;
#endif
}
void sim_report_observe(const char *text) {
    if(strstr(text,"Waiting for GP20") && !sim.auto_start) sim.auto_start=sim.us+100000;
    if(!strcmp(text,"RESULT: PAYLOAD CONFIRMED\n") || !strcmp(text,"RESULT: ROUTE COMPLETE\n")) sim.success=true;
    if(!strncmp(text,"Run ended.",10)) sim.finished=true;
    int id=0,mask=0,n=0; char marker[64];
    if(sscanf(text,"ROUTE: target=%d%n",&id,&n)==1 && id>=1 && id<=3 &&
       (!strcmp(text+n,"\n") || !text[n])) {
        snprintf(marker,sizeof(marker),"ROUTE: target=%d%s",id,text[n]?"\n":"");
        if(!strcmp(text,marker)) sim.active_target=id;
    }
    n=0;
    if(sscanf(text,"ROUTE: confirmed=%d,mask=%d%n",&id,&mask,&n)==2 && id>=1 && id<=3 &&
       (!strcmp(text+n,"\n") || !text[n]) && mask==(int)(sim.completed_mask | (1u<<(id-1)))) {
        snprintf(marker,sizeof(marker),"ROUTE: confirmed=%d,mask=%d%s",id,mask,text[n]?"\n":"");
        if(!strcmp(text,marker)) sim.completed_mask=(unsigned)mask;
    }
    // Fixed-target firmware reports verified client completion explicitly.
    for(id=1;id<=3;++id) {
        char expected[96]; snprintf(expected,sizeof(expected),"AP%d PAYLOAD COMPLETE: verified reply and final acknowledgement\n",id);
        if(!strcmp(text,expected)) { SimBeacon *b=sim_beacon(id); if(b) ++b->payload_events; }
    }
    sim_log("firmware",text);
}
