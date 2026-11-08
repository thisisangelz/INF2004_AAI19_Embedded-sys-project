// Small JSON reader for scenario objects. No firmware or controller code.
#include "sim.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
typedef struct { const char *start, *end; int next; char type; } Token;
static Token tokens[512];
static int used;
static const char *cursor;
static void whitespace(void) { while(isspace((unsigned char)*cursor)) ++cursor; }
static int parse(unsigned depth) {
    whitespace(); if(depth>16 || used>=512) return -1;
    int n=used++; tokens[n].start=cursor; tokens[n].type=*cursor;
    if(*cursor=='{' || *cursor=='[') {
        char close=*cursor++=='{'?'}':']'; whitespace();
        if(*cursor!=close) for(;;) {
            if(close=='}') {
                whitespace(); if(*cursor!='"' || parse(depth+1)<0) return -1;
                whitespace(); if(*cursor++!=':') return -1;
            }
            if(parse(depth+1)<0) return -1;
            whitespace(); if(*cursor==close) break;
            if(*cursor++!=',') return -1;
        }
        ++cursor;
    } else if(*cursor=='"') {
        ++cursor;
        while(*cursor && *cursor!='"') {
            if((unsigned char)*cursor<32) return -1;
            if(*cursor=='\\') {
                ++cursor;
                if(*cursor=='u') { for(int i=0;i<4;++i) { ++cursor; if(!isxdigit((unsigned char)*cursor)) return -1; } }
                else if(!*cursor || !strchr("\"\\/bfnrt",*cursor)) return -1;
            }
            ++cursor;
        }
        if(*cursor++!='"') return -1;
    } else if(!strncmp(cursor,"true",4)) cursor+=4;
    else if(!strncmp(cursor,"false",5)) cursor+=5;
    else if(!strncmp(cursor,"null",4)) cursor+=4;
    else {
        // JSON number syntax, deliberately reject NaN/Infinity/hex and trailing '.' .
        if(*cursor=='-') ++cursor;
        if(*cursor=='0') ++cursor;
        else { if(*cursor<'1' || *cursor>'9') return -1; while(isdigit((unsigned char)*cursor)) ++cursor; }
        if(*cursor=='.') { ++cursor; if(!isdigit((unsigned char)*cursor)) return -1; while(isdigit((unsigned char)*cursor)) ++cursor; }
        if(*cursor=='e' || *cursor=='E') { ++cursor; if(*cursor=='+' || *cursor=='-') ++cursor; if(!isdigit((unsigned char)*cursor)) return -1; while(isdigit((unsigned char)*cursor)) ++cursor; }
    }
    tokens[n].end=cursor; tokens[n].next=used; return n;
}
static bool equal(int t,const char *s) {
    return tokens[t].type=='"' && tokens[t].end-tokens[t].start==(int)strlen(s)+2 && !memcmp(tokens[t].start+1,s,strlen(s));
}
static int field(int object,const char *key) {
    for(int k=object+1;k<tokens[object].next;) {
        int v=k+1; if(equal(k,key)) return v; k=tokens[v].next;
    }
    return -1;
}
static bool number(int t,double *out) {
    if(t<0) return true;
    if(tokens[t].type=='t') { *out=1; return true; }
    if(tokens[t].type=='f') { *out=0; return true; }
    if(tokens[t].type!='-' && !isdigit((unsigned char)tokens[t].type)) return false;
    char *end; *out=strtod(tokens[t].start,&end); return end==tokens[t].end && isfinite(*out);
}
static bool value(int object,const char *key,double *out,double lo,double hi) {
    return number(field(object,key),out) && *out>=lo && *out<=hi;
}
void sim_scenario_defaults(void) {
    sim=(Sim){.x=-2,.y=-1,.bx=1,.by=1,.speed=.76,.wheelbase=.14,.wifi_ref=-50,.exponent=2,
        .ble_radius=.25,.stop_at=-1,.horizon=1000,.mat_radius=.35,.sensor_offset=.08,.radio_range=5,
        .box_radius=.07,.colour_enabled=true,.wifi_observed=-127,.ble_observed=-127,.peer_state="ADVERTISING"};
}
static bool placements(int array,bool mats) {
    if(tokens[array].type!='[') return false;
    unsigned count=0,mask=0;
    for(int t=array+1;t<tokens[array].next;t=tokens[t].next) {
        if(++count>3 || tokens[t].type!='{') return false;
        double id=0,x=NAN,y=NAN;
        int identity=field(t,"id");
        if(identity<0 || tokens[identity].end-tokens[identity].start!=1 ||
           *tokens[identity].start<'1' || *tokens[identity].start>'3') return false;
        if(!value(t,"id",&id,1,3) || floor(id)!=id || (mask&(1u<<(int)id)) ||
           !value(t,"x",&x,-50,50) || !value(t,"y",&y,-50,50)) return false;
        mask|=1u<<(int)id;
        const char *mat=(const char *[]){"pink","blue","purple"}[(int)id-1];
        int m=field(t,"mat");
        if(m>=0) {
            if(equal(m,"pink")) mat="pink"; else if(equal(m,"blue")) mat="blue";
            else if(equal(m,"purple")) mat="purple"; else return false;
        }
        if(mats) { SimMat *b=&sim.mats[count-1]; b->id=(int)id; b->x=x; b->y=y; strcpy(b->mat,mat); }
        else { SimBeacon *b=&sim.beacons[count-1]; b->id=(int)id; b->x=x; b->y=y; strcpy(b->mat,mat); b->peer_state="ADVERTISING"; b->wifi_observed=b->ble_observed=-127; }
    }
    if(mats) sim.mat_count=count; else sim.beacon_count=count;
    return count>0;
}
bool sim_scenario_json(const char *json) {
    sim_scenario_defaults(); cursor=json; used=0;
    if(parse(0)!=0) return false;
    whitespace(); if(*cursor || tokens[0].type!='{') return false;
#define V(key,lo,hi) if(!value(0,#key,&sim.key,lo,hi)) return false
    V(x,-50,50); V(y,-50,50); V(heading,-360,360); V(bx,-50,50); V(by,-50,50);
    V(speed,.05,2); V(wheelbase,.05,1); V(wifi_ref,-100,-10); V(exponent,.5,5);
    V(ble_radius,.05,3); V(stop_at,-1,1100); V(horizon,5,1100); V(mat_radius,.01,5);
    V(sensor_offset,0,1); V(radio_range,.01,100); V(box_radius,0,1);
#undef V
    double corrupt=0,colour=1;
    if(!value(0,"corrupt_reply",&corrupt,0,1) || (corrupt!=0 && corrupt!=1) ||
       !value(0,"colour_enabled",&colour,0,1) || (colour!=0 && colour!=1)) return false;
    sim.corrupt_reply=corrupt!=0; sim.colour_enabled=colour!=0;
    int b=field(0,"beacons");
    if(b>=0) { if(!placements(b,false)) return false; }
    else { sim.beacon_count=1; sim.beacons[0]=(SimBeacon){.id=1,.x=sim.bx,.y=sim.by,.mat="pink",.peer_state="ADVERTISING",.wifi_observed=-127,.ble_observed=-127}; }
    for(unsigned i=0;i<sim.beacon_count;++i) if(sim.beacons[i].id==1) { sim.bx=sim.beacons[i].x; sim.by=sim.beacons[i].y; }
    int m=field(0,"mats");
    if(m>=0) { if(!placements(m,true)) return false; }
    else {
        sim.mat_count=sim.beacon_count;
        for(unsigned i=0;i<sim.mat_count;++i) {
            SimBeacon *bcn=&sim.beacons[i]; SimMat *mat=&sim.mats[i];
            mat->id=bcn->id; mat->x=bcn->x; mat->y=bcn->y; strcpy(mat->mat,bcn->mat);
        }
    }
    return true;
}
