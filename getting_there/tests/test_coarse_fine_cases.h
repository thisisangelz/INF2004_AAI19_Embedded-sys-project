// Included after the navigation hardware mocks. Tests exercise production code.
static void check_coarse_tracking(void)
{
    line_track_t track;
    line_track_init(&track,pair(-60,-60));
    line_track_observe(&track,500,pair(-65,-65));
    line_track_observe(&track,1000,pair(-60,-70)); // Inconclusive, not a reset.
    line_track_observe(&track,1500,pair(-66,-66));
    assert(line_track_weak_count(&track)==2 && !line_track_reassess(&track,1500));
    line_track_observe(&track,1750,pair(-67,-67));
    assert(line_track_weak_count(&track)==3 && line_track_reassess(&track,1750));
    assert(track.region_position==0 && track.dip_active);

    line_track_init(&track,pair(-60,-60));
    line_track_observe(&track,500,pair(-65,-65));
    line_track_observe(&track,1000,pair(-60,-60));
    assert(line_track_weak_count(&track)==1 && track.region_position==0);
    line_track_observe(&track,1500,pair(-61,-61));
    assert(!line_track_weak_count(&track) && track.region_position==1500 && !track.dip_active);

    // Recorded v16: return tolerance accepted -62/-61 against -56/-57.
    // Navigation recovery must reject it, even with the original noise spreads.
    line_pair_t reference=pair(-56,-57), weak=pair(-62,-61);
    reference.first.spread=reference.second.spread=1;
    weak.first.spread=2; weak.second.spread=1;
    assert(line_return_consistent(reference,weak));
    line_track_init(&track,reference);
    assert(!line_track_recovered(&track,6802,weak,weak));
    assert(track.region_position==0);
    assert(!line_track_recovered(&track,6802,weak,pair(-57,-58)));
    assert(line_track_recovered(&track,6802,pair(-58,-59),pair(-57,-58)));
    assert(track.region_position==6802 && track.recent_count==1);
    line_pair_t noisy=pair(-57,-57); noisy.first.spread=4;
    assert(!line_region_matches(reference,noisy));
    assert(!line_region_matches(reference,pair(-57,-60)));
    assert(!line_region_matches(reference,pair(-127,-57)));

    // Later near-equal readings can advance the return region, but the signal
    // reference must not become weaker and allow gradual drift.
    line_track_init(&track,pair(-70,-70));
    line_track_observe(&track,500,pair(-60,-60));
    line_track_observe(&track,1250,pair(-60,-60));
    assert(track.reference.first.code==60 && track.region_position==1250 && track.progress);
    line_track_observe(&track,2000,pair(-61,-61));
    line_track_observe(&track,2750,pair(-61,-61));
    assert(track.region_position==2750 && track.reference.first.code==60);
    line_track_observe(&track,3500,pair(-64,-64));
    line_track_observe(&track,4000,pair(-60,-60));
    line_track_observe(&track,4500,pair(-65,-65));
    assert(line_track_step(&track,4500,false)==250);
    line_track_observe(&track,4750,pair(-60,-60));
    assert(track.region_position==2750 && line_track_weak_count(&track)==2);
    assert(line_track_reassess(&track,4750) && !line_track_step(&track,4750,false));

    // An isolated strong position must not promote the median reference.
    line_track_init(&track,pair(-65,-65));
    line_track_observe(&track,500,pair(-65,-65));
    line_track_observe(&track,1000,pair(-45,-45));
    line_track_observe(&track,1750,pair(-65,-65));
    assert(track.reference.first.code==65 && !track.dip_active);
    // Short blocks cannot manufacture multiple spatial votes.
    line_track_init(&track,pair(-60,-60));
    for (unsigned pos=100;pos<=600;pos+=100) line_track_observe(&track,pos,pair(-70,-70));
    assert(line_track_weak_count(&track)==2);
    // A new heading gets its own baseline and no inherited decline votes.
    line_track_init(&track,pair(-80,-80));
    assert(!track.progress && !track.weak_bits && track.reference.first.code==80);
}

static void check_coarse_follow(void)
{
    follow_result_t followed;
    static const int rising[]={-70,-70,-65,-65,-64,-64,-63,-63};
    reset(); script=rising; script_length=8; complete_after_step=4;
    assert(follow_direction(0,0,&followed)==PAYLOAD_COMPLETE && !backwards_count);
    assert(moves[0].ms==500 && moves[1].ms==750 && moves[2].ms==750 && moves[3].ms==750);
    assert(translation_ms==2750 && wifi_calls==40 && ble_calls==4);
    reset(); script=rising; script_length=8; complete_after_step=4; near_after_step=2;
    assert(follow_direction(0,0,&followed)==PAYLOAD_COMPLETE);
    assert(moves[0].ms==500 && moves[1].ms==750 && moves[2].ms==250 && moves[3].ms==250);
    assert(translation_ms==1750);
    // Flat signals can continue, with fewer pauses per unit of powered travel.
    reset(); complete_after_step=16;
    assert(approach_beacon() && forwards_count==16 && !backwards_count && translation_ms==8000);
    assert(strstr(log_text,"six-second profile interval complete"));
    reset(); scan_delay_ms=720; ble_delay_ms=5000; complete_after_step=6;
    assert(approach_beacon() && translation_ms==3000 && !backwards_count);
    assert(sim_time<110000000u); // V16 required twelve scan/BLE cycles for this travel.
    // Side trials retain their 2400 ms limit and return before the other side.
    reset(); bool found;
    assert(check_perpendiculars(0,&followed,&found)==PAYLOAD_NOT_CLOSE && !found && followed.consistent);
    assert(forwards_count==10 && backwards_count==8 && translation_ms==9600);
    assert(moves[0].heading==270 && moves[9].heading==90);
    // Invalid returns still prevent a turn to the other side.
    int bad_side[14]; for (unsigned i=0;i<14;++i) bad_side[i]=i<12 ? -70 : -90;
    reset(); script=bad_side; script_length=14;
    assert(check_perpendiculars(0,&followed,&found)==PAYLOAD_NOT_CLOSE && !found && !followed.consistent);
    assert(current_heading==270 && forwards_count==5);
}

static const int refinement_readings[]={-62,-62,-60,-60,-58,-58,-60,-60,-62,-62,-58,-58};

static void check_refine_return(void)
{
    line_track_t track;
    line_track_init(&track,pair(-60,-60)); track.region_position=1250;
    bool consistent; uint32_t position;
    reset(); script=refinement_readings; script_length=12;
    assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_NOT_CLOSE && consistent);
    assert(backwards_count==6 && forwards_count==2 && position==1250 && wifi_calls==60);
    assert(moves[0].ms==750 && moves[1].ms==750);
    for (unsigned i=2;i<8;++i) assert(moves[i].ms==125);
    assert(strstr(log_text,"selected 1250") && strstr(log_text,"ready for perpendicular turn"));
    assert(translation_ms==2250);
    // Full approach -> three weak positions -> stopped failed recheck -> local
    // sweep and selected turning point. Later scans are synthetic alternatives.
    static const int decline[]={-70,-70,-60,-60,-60,-60,-66,-66,-67,-67,-68,-68,
        -68,-68,-69,-69,-62,-62,-60,-60,-58,-58,-60,-60,-62,-62,-58,-58};
    reset(); script=decline; script_length=sizeof(decline)/sizeof(*decline);
    follow_result_t followed;
    assert(follow_direction(0,0,&followed)==PAYLOAD_NOT_CLOSE && followed.consistent && followed.progress);
    assert(followed.end==FOLLOW_DECLINE && followed.position==1250);
    assert(forwards_count==7 && backwards_count==6 && translation_ms==5250);
    assert(strstr(log_text,"weak positions 3/4") && strstr(log_text,"REGION RETURN: from 3000"));
    // The stopped recheck can recover without any extra small forward probes.
    int recover[16]; memcpy(recover,decline,sizeof(recover));
    for (unsigned i=12;i<16;++i) recover[i]=-61;
    reset(); script=recover; script_length=16; complete_after_step=6;
    assert(follow_direction(0,0,&followed)==PAYLOAD_COMPLETE && !backwards_count);
    assert(strstr(log_text,"two stopped pairs confirmed recovery") && moves[5].ms==750);
    // A stable but weak region cannot be accepted simply because the sweep ended.
    reset(); default_rssi=-75;
    assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_NOT_CLOSE && !consistent);
    assert(!forwards_count && backwards_count==6);
    // The best sample is confirmed again after repositioning.
    int failed_verify[12]; memcpy(failed_verify,refinement_readings,sizeof(failed_verify));
    failed_verify[10]=failed_verify[11]=-80;
    reset(); script=failed_verify; script_length=12;
    assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_NOT_CLOSE && !consistent);
    assert(strstr(log_text,"inconsistent; stopping"));
}

static void check_coarse_faults(void)
{
    line_track_t track;
    line_track_init(&track,pair(-60,-60)); track.region_position=1250;
    bool consistent; uint32_t position;
    // Stop on BLE success/failure during bulk reverse, fine sweep or reposition.
    for (int call=1;call<=8;++call) {
        reset(); script=refinement_readings; script_length=12; complete_at_ble_call=call;
        assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_COMPLETE);
        assert(ble_calls==call && move_count==(unsigned)call && !motors_on);
        reset(); script=refinement_readings; script_length=12; fail_at_ble_call=call;
        assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL);
        assert(ble_calls==call && move_count==(unsigned)call && !motors_on);
        reset(); script=refinement_readings; script_length=12; stop_at_move=(unsigned)call;
        assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL);
        assert(stopped && move_count==(unsigned)call && !motors_on && ble_calls==call-1);
        reset(); script=refinement_readings; script_length=12; fail_at_move=(unsigned)call;
        assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL);
        assert(fault_test(FAULT_IMU) && move_count==(unsigned)call && !motors_on);
    }
    // Missing samples or STOP at any Wi-Fi stage prevent later movement.
    for (unsigned stage=0;stage<6;++stage) {
        int missing[16];
        for (unsigned i=0;i<16;++i) missing[i]=i<stage*2 ? refinement_readings[i] : -127;
        reset(); script=missing; script_length=16; default_rssi=-127;
        assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL && !motors_on);
        assert(move_count==(stage<5 ? 2+stage : 8));
        reset(); script=refinement_readings; script_length=12; stop_at_scan=stage*10+1;
        assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL && stopped && !motors_on);
        assert(move_count==(stage<5 ? 2+stage : 8));
    }
    reset(); storage_failed=true;
    assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL && !move_count);
    reset(); translation_ms=LINE_TRANSLATION_MS-3949;
    assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL && !move_count);
    assert(fault_test(FAULT_TRANSLATION_LIMIT));
    reset(); sim_time=(uint64_t)(MAX_RUN_MS-1000u)*1000; script=refinement_readings; script_length=12;
    assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL && !motors_on);
    assert(fault_test(FAULT_RUN_LIMIT));
    // Excessively short blocks have a finite attempt count, including return refinement.
    static const uint32_t short_blocks[]={750,750,1,1,1,1,1,1,1,1};
    reset(); powered_script=short_blocks; powered_script_length=10; default_rssi=-60;
    assert(return_and_refine(0,3000,&track,&consistent,&position)==PAYLOAD_FATAL && move_count==10);
    assert(fault_test(FAULT_PROFILE));

    bool recovered;
    for (unsigned scan=1;scan<=20;scan+=5) {
        reset(); stop_at_scan=scan;
        assert(recheck_region(&track,3000,&recovered)==PAYLOAD_FATAL && !move_count && stopped);
    }
    reset(); default_rssi=-127;
    assert(recheck_region(&track,3000,&recovered)==PAYLOAD_FATAL && !move_count);
    reset(); complete_at_ble_call=1;
    assert(recheck_region(&track,3000,&recovered)==PAYLOAD_COMPLETE && !wifi_calls && !move_count);
    reset(); fail_at_ble_call=1;
    assert(recheck_region(&track,3000,&recovered)==PAYLOAD_FATAL && !wifi_calls && !move_count);
    // Forward STOP and faults still prevent subsequent scans and BLE.
    follow_result_t followed;
    reset(); stop_at_step=2;
    assert(follow_direction(0,0,&followed)==PAYLOAD_FATAL && forwards_count==2 && ble_calls==1);
    reset(); fail_at_step=2;
    assert(follow_direction(0,0,&followed)==PAYLOAD_FATAL && fault_test(FAULT_IMU) && ble_calls==1);
    reset(); translation_ms=LINE_TRANSLATION_MS-3000;
    assert(follow_direction(0,0,&followed)==PAYLOAD_FATAL && !move_count && fault_test(FAULT_TRANSLATION_LIMIT));
    reset(); scan_delay_ms=1200; ble_delay_ms=5000;
    assert(!approach_beacon() && fault_test(FAULT_RUN_LIMIT) && !motors_on);
    reset(); assert(!approach_beacon() && fault_test(FAULT_TRANSLATION_LIMIT) && !motors_on);
    assert(translation_ms<=LINE_TRANSLATION_MS);
}
