// Deterministic main-loop route fixtures; BLE wire/CRC/finalACK are exercised
// separately against the actual client by test_payload_client_route.
#define NAVIGATION_TEST_MAIN legacy_navigation_checks
#include "test_navigation.c"

static unsigned leg_steps[3], seen_target, confirms;
static bool fail_second, lose_cleanup;
static payload_result_t route_peer(void)
{
    unsigned id = target_beacon_id();
    if (route.departing) assert(false); // no BLE calls are permitted here
    if (id != seen_target) { seen_target=id; leg_steps[id-1u]=(unsigned)forwards_count; }
    if ((unsigned)forwards_count == leg_steps[id-1u]) return PAYLOAD_NOT_CLOSE;
    if (id==2 && fail_second) return PAYLOAD_FATAL;
    if (lose_cleanup) payload_idle=false;
    ++confirms;
    return PAYLOAD_COMPLETE;
}
static void route_reset(void)
{
    reset(); shipped_steps(); colour_present=true; colour_arrive_at_check=1;
    attempt_override=route_peer;
    memset(leg_steps,0,sizeof(leg_steps)); seen_target=confirms=0;
    fail_second=lose_cleanup=false; route.incoming_forwards=true;
}
static void check_full_route(void)
{
    route_reset();
    assert(approach_route());
    assert(route.completed_mask==7 && target_beacon_id()==3 && confirms==3);
    assert(forwards_count==3 && backwards_count==4 && departure_floor_checks==4);
    assert(sign_checks==1 && !motors_on && colour_resets==3);
    assert(strstr(log_text,"ROUTE: target=1") && strstr(log_text,"ROUTE: target=2") &&
           strstr(log_text,"ROUTE: target=3"));
    assert(strstr(log_text,"ROUTE: confirmed=1,mask=1") &&
           strstr(log_text,"ROUTE: confirmed=2,mask=3") &&
           strstr(log_text,"ROUTE: confirmed=3,mask=7"));
    assert(!strstr(log_text,"RESULT: PAYLOAD CONFIRMED"));
    assert(translation_ms==500 && !fault_mask()); // leg budget resets, mission clock does not
}
static void check_partial_and_gates(void)
{
    route_reset(); fail_second=true;
    assert(!approach_route() && route.completed_mask==1 && target_beacon_id()==2 && !motors_on);
    assert(!strstr(log_text,"ROUTE: target=3"));
    route_reset(); lose_cleanup=true;
    assert(!approach_route() && !route.completed_mask && target_beacon_id()==1 && !backwards_count);
    route_reset(); floor_always_mat=true;
    assert(!approach_route() && route.completed_mask==1 && target_beacon_id()==1 && !motors_on);
    assert(translation_ms<=500+ROUTE_DEPARTURE_MS && !strstr(log_text,"ROUTE: target=2"));
    route_reset(); floor_read_error=true;
    assert(!approach_route() && route.completed_mask==1 && target_beacon_id()==1 && !motors_on);
    route_reset(); stop_on_reverse=true;
    assert(!approach_route() && stopped && route.completed_mask==1 && target_beacon_id()==1 && !motors_on);
    route_reset(); colour_present=false;
    assert(!approach_route() && fault_test(FAULT_COLOUR_SENSOR) && !forwards_count);
    route_reset(); run_started_ms=0; sim_time=(uint64_t)MAX_RUN_MS*1000;
    assert(!approach_route() && fault_test(FAULT_RUN_LIMIT) && !forwards_count);
    route_reset(); storage_failed=true;
    assert(!approach_route() && !route.completed_mask && !forwards_count);
}
static void check_departure_heading_and_budget(void)
{
    route_reset(); incoming_heading=0; route.incoming_valid=true; current_heading=359;
    assert(depart_previous_mat()); // signed shortest angle: -1, not +359
    assert(!forwards_count && backwards_count==2 && !ble_calls);
    route_reset(); incoming_heading=350; route.incoming_valid=true; current_heading=20;
    assert(!depart_previous_mat() && !move_count && fault_test(FAULT_HEADING));
    route_reset(); route.incoming_valid=false;
    assert(!depart_previous_mat() && !move_count); // no observed incoming path
    // Arrival by rotation after a colour scan also stops conservatively if it
    // no longer faces the last forward approach heading; no turn at the box.
    route_reset(); incoming_heading=90; current_heading=120; route.incoming_valid=true;
    assert(!depart_previous_mat() && !move_count);
    // A transfer can complete during a timed reverse return. Depart forward,
    // opposite to that actual approach, with no BLE calls during departure.
    route_reset(); attempt_override=NULL; complete_first=true;
    uint32_t actual;
    assert(translate_block(false,250,0,250,0,false,&actual)==PAYLOAD_COMPLETE);
    assert(route.incoming_valid && !route.incoming_forwards);
    int calls=ble_calls;
    assert(depart_previous_mat() && forwards_count==2 && backwards_count==1 && ble_calls==calls);
    route_reset(); incoming_heading=0; route.incoming_valid=true;
    translation_ms=LINE_TRANSLATION_MS-100;
    assert(!depart_previous_mat() && fault_test(FAULT_TRANSLATION_LIMIT) && !move_count);
}
static void check_mat_fallback_and_inference(void)
{
    route_reset(); current_heading=27; mat_stub=true;
    assert(mat_approach()==PAYLOAD_NOT_CLOSE && current_heading==27);
    assert(strstr(log_text,"restoring entry heading"));
    route_reset(); current_heading=42; mat_stub=true; fail_colour_read_at=7;
    assert(mat_approach()==PAYLOAD_NOT_CLOSE && current_heading==42);
    assert(strstr(log_text,"could not read") && strstr(log_text,"restoring entry heading"));
    // With the sensor 8 cm ahead of the axle, a valid scan/turn can put it
    // on the arrival ring before any forward driving. This supplies an inward
    // observed departure heading and still requires genuine colour arrival.
    route_reset(); model=true; robot_y=-0.19; attempt_override=NULL; complete_first=true;
    assert(mat_approach()==PAYLOAD_COMPLETE && !forwards_count && route.incoming_valid);
    assert(route.incoming_forwards && strstr(log_text,"DEPARTURE BASIS: fresh colour gradient"));
    int calls=ble_calls;
    assert(depart_previous_mat() && backwards_count==2 && ble_calls==calls);
    route_reset();
    assert(mat_centre_reached("synthetic inconclusive centre")==PAYLOAD_FATAL);
    assert(fault_test(FAULT_COLOUR_SENSOR) && !mat_centre_override && !ble_calls && !move_count);
    assert(strstr(log_text,"arrival ring is NOT proven"));
}
static void check_bounded_trials_and_watchdog(void)
{
    reset(); shipped_steps(); colour_present=true; follow_result_t followed;
    assert(follow_direction(0,0,&followed)==PAYLOAD_NOT_CLOSE);
    assert(followed.end==FOLLOW_NO_GAIN && followed.position==2400 && !backwards_count);
    // Repeated small positive gain may extend once, never past 4800 ms.
    static const int small_gain[]={-70,-70,-69,-69};
    reset(); shipped_steps(); colour_present=true; script=small_gain; script_length=4; default_rssi=-69;
    assert(follow_direction(0,0,&followed)==PAYLOAD_NOT_CLOSE);
    assert(followed.end==FOLLOW_NO_GAIN && followed.position==4800 && !followed.progress);
    assert(strstr(log_text,"bounded extension"));
    // A supported initial best followed by flat readings returns to that best.
    static const int gain[]={-70,-70,-65,-65};
    reset(); shipped_steps(); colour_present=true; script=gain; script_length=4; default_rssi=-65;
    assert(follow_direction(0,0,&followed)==PAYLOAD_NOT_CLOSE);
    assert(followed.progress && followed.end==FOLLOW_DECLINE && followed.position==500);
    assert(backwards_count && strstr(log_text,"no-new-best watchdog"));
    assert(!strstr(log_text,"decline persisted"));
}
int main(void)
{
    check_full_route(); check_partial_and_gates(); check_departure_heading_and_budget();
    check_bounded_trials_and_watchdog(); check_mat_fallback_and_inference();
    return 0;
}
