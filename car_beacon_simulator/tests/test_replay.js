'use strict';
// Exercise the real file-input, playback and export handlers without browser packages.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const root=path.resolve(__dirname,'..');

function harness(fetchImplementation){
  const elements={},downloads=[],drawCalls=[],rect={left:0,top:0,width:700,height:400};
  const drawing=new Proxy({measureText:text=>({width:text.length*6}),createRadialGradient:()=>({addColorStop(){}})},
    {get:(target,key)=>key in target?target[key]:(...args)=>drawCalls.push({method:key,args})});
  const element=()=>({value:'',style:{},dataset:{},children:[],options:[],listeners:{},classList:{toggle(){}},
    clientWidth:700,clientHeight:400,scrollHeight:0,scrollTop:0,parentElement:{},files:[],
    getBoundingClientRect:()=>rect,getContext:()=>drawing,reportValidity:()=>true,
    addEventListener(name,fn){this.listeners[name]=fn;},replaceChildren(){this.children=[];this.options=[];},
    append(child){this.children.push(child);this.options.push(child);},setPointerCapture(){},click(){}});
  const html=fs.readFileSync(path.join(root,'dist/index.html'),'utf8');
  for(const match of html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)){
    const value=match[0].match(/\bvalue="([^"]*)"/);
    elements[match[1]]={...element(),value:value?value[1]:'',hidden:match[0].includes(' hidden'),checked:/\bchecked\b/.test(match[0]),disabled:/\bdisabled\b/.test(match[0])};
  }
  for(const match of html.matchAll(/<select\b[^>]*id="([^"]+)"[^>]*>(.*?)<\/select>/gs)){
    const options=[...match[2].matchAll(/<option value="([^"]+)"([^>]*)>(.*?)<\/option>/gs)].map(option=>({...element(),value:option[1],selected:/\bselected\b/.test(option[2]),textContent:option[3]}));
    elements[match[1]].options=options;elements[match[1]].children=options;
    elements[match[1]].value=(options.find(option=>option.selected)||options[0])?.value||'';
  }
  elements.evidenceTabs.children=['events','history'].map(view=>({...element(),dataset:{view}}));
  const context=vm.createContext({console,Blob,document:{body:{},getElementById:id=>{
    assert.ok(elements[id],`Unknown element ${id}`);return elements[id];
  },createElement:element},devicePixelRatio:1,getComputedStyle:()=>({fontFamily:'sans-serif'}),
  ResizeObserver:class{observe(){}},performance:{now:()=>0},requestAnimationFrame(){},setTimeout(){},
  URL:{createObjectURL(blob){downloads.push(blob);return 'blob:test';},revokeObjectURL(){}},
  fetch:fetchImplementation||(()=>{throw Error('Import and replay must not call the simulation server.');})});
  const evaluate=code=>vm.runInContext(code,context);
  evaluate(fs.readFileSync(path.join(root,'dist/app.js'),'utf8').replace(/init\(\);\s*$/,''));
  evaluate('resetPreview(true)');
  const fileInput=async (text,name='previous-run.json',size=Buffer.byteLength(text))=>{
    elements.traceFile.files=[{name,size,text:async()=>text}];
    return elements.traceFile.onchange();
  };
  return {elements,downloads,drawCalls,evaluate,fileInput};
}

function fixture(success=false){
  const scenario={x:0,y:0,heading:12.34,bx:1,by:0,speed:.76,wheelbase:.14,wifi_ref:-50,
    exponent:2,ble_radius:.25,stop_at:-1,horizon:1000,corrupt_reply:0};
  const first={type:'frame',t:0,x:0,y:0,heading:12.34,gyro_heading:0,distance:1,closest:1,path:0,
    wifi:-50,ble:-62,wifi_observed:-127,ble_observed:-127,left:0,right:0,left_pwm:0,right_pwm:0,
    wifi_scans:0,ble_reports:0,ble_attempts:0,faults:0,buzzer:false,wifi_active:false,rx_bytes:0,tx_bytes:0,
    peer_verified:false,peer_complete:false,peer_state:'Advertising',ble_state:'Idle',ble_samples:0,
    ble_close_count:0,ble_filtered:-127,ble_threshold:-50,payload_verified:false};
  return {id:'example-recording',metadata:{ref:'origin/no-longer-available',commit:'abcdef1234567890',
    moving_samples:false,beacon_mode:'branch-firmware',model_version:2},scenario,
    result:{type:'result',success,t:2,reason:success?'Firmware confirmed payload':'Firmware stopped without payload',
      distance:.9,closest:.9,path:.1,faults:success?0:1024,wall_seconds:.01,
      failure_messages:success?[]:['FAULT: NO_DIRECTION']},
    frames:[first,{...first,t:1,x:.1,path:.1,distance:.9,closest:.9,left:.2,right:.2},
      {...first,t:2,x:.1,path:.1,distance:.9,closest:.9,faults:success?0:1024,
        payload_verified:success,peer_verified:success,peer_complete:success}],
    logs:[{type:'log',t:0,source:'firmware',message:'GP20 START'},
      {type:'log',t:2,source:'firmware',message:success?'RESULT: PAYLOAD CONFIRMED':'RESULT: STOPPED WITHOUT PAYLOAD'}]};
}

test('imports a failed recording, pauses at start and seeks through its original frames and logs',async()=>{
  const h=harness(),trace=fixture(false),text=JSON.stringify(trace);
  await h.fileInput(text);
  assert.equal(h.evaluate('JSON.stringify(run)'),text);
  assert.equal(h.evaluate('playing'),false);
  assert.equal(h.evaluate('playTime'),0);
  assert.equal(h.elements.heading.value,'12.34');
  assert.equal(h.elements.customRef.value,trace.metadata.ref);
  assert.equal(h.elements.traceNotice.hidden,false);
  assert.match(h.elements.traceNotice.textContent,/previous-run.json/);
  assert.equal(h.elements.timeline.disabled,false);
  h.elements.playBtn.onclick();assert.equal(h.evaluate('playing'),true);
  h.elements.timeline.value='1';h.elements.timeline.oninput();
  assert.equal(h.evaluate('frame.x'),.1);assert.equal(h.evaluate('playing'),false);
  h.elements.endBtn.onclick();assert.equal(h.evaluate('playTime'),2);
  assert.equal(h.evaluate('run.result.success'),false);
  assert.equal(h.elements.outcome.hidden,false);
  assert.match(h.elements.outcomeTitle.textContent,/without a confirmed payload/);
  assert.match(h.elements.events.innerHTML,/STOPPED WITHOUT PAYLOAD/);
});

test('re-export preserves every recorded field and the original success condition',async()=>{
  const h=harness(),trace=fixture(true);
  await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  assert.equal(h.evaluate('run.result.success'),true);
  assert.match(h.elements.outcomeTitle.textContent,/Payload confirmed/);
  h.elements.jsonBtn.onclick();assert.equal(h.downloads.length,1);
  assert.deepEqual(JSON.parse(await h.downloads[0].text()),trace);
});

test('v16 dip decisions and GATT diagnostics remain visible without changing the outcome',async()=>{
  const h=harness(),trace=fixture(false);
  trace.logs.splice(1,0,
    {t:1,source:'firmware',message:'DIP: three weak positions; stationary BLE check'},
    {t:1,source:'firmware',message:'DIP_CHECK,stage=probe,probe=1,recovered=0'},
    {t:1,source:'firmware',message:'AP1 GATT: service discovery completed, status 0x00'});
  await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  assert.match(h.elements.events.innerHTML,/DIP: three weak/);
  assert.match(h.elements.events.innerHTML,/DIP_CHECK,stage=probe/);
  h.elements.logFilter.value='ble';h.elements.logFilter.onchange();
  assert.match(h.elements.events.innerHTML,/AP1 GATT: service discovery/);
  assert.equal(h.evaluate('run.result.success'),false);
});

test('threshold displays use recorded firmware values and never invent a preview threshold',async()=>{
  const h=harness();
  assert.equal(h.evaluate('firmwareThreshold(previewFrame())'),null);
  const trace=fixture();for(const f of trace.frames)f.ble_threshold=-58;
  await h.fileInput(JSON.stringify(trace));
  assert.equal(h.evaluate('firmwareThreshold(frame)'),-58);
  h.evaluate("chartMode='ble';drawChart();drawArena()");
  h.elements.customRef.value='another-branch';h.elements.customRef.oninput();
  assert.equal(h.evaluate('firmwareThreshold(previewFrame())'),null);
  // Old recordings predate threshold telemetry and used the original -50 gate.
  for(const f of trace.frames)delete f.ble_threshold;
  await h.fileInput(JSON.stringify(trace));
  assert.equal(h.evaluate('firmwareThreshold(frame)'),-50);
});

test('rejects setup files, malformed JSON and incomplete traces without replacing the active run',async()=>{
  const h=harness(),original=JSON.stringify(fixture());await h.fileInput(original);
  h.elements.playBtn.onclick();
  const bad=[JSON.stringify({scenario:fixture().scenario}),'{bad json',
    JSON.stringify({...fixture(),frames:[]}),
    JSON.stringify({...fixture(),result:{...fixture().result,success:'false'}}),
    JSON.stringify({...fixture(),frames:fixture().frames.slice(0,2)}),
    JSON.stringify({...fixture(),frames:[fixture().frames[2],fixture().frames[0]]}),
    JSON.stringify({...fixture(),logs:[{t:0,source:'firmware'}]}),
    JSON.stringify({...fixture(),frames:[{...fixture().frames[0],wifi:null},...fixture().frames.slice(1)]})];
  for(const text of bad){
    await h.fileInput(text);assert.equal(h.elements.error.hidden,false);
    assert.equal(h.evaluate('JSON.stringify(run)'),original);assert.equal(h.evaluate('playing'),true);
    assert.equal(h.evaluate('busy'),false);assert.equal(h.elements.traceFile.value,'');
  }
  await h.fileInput(original,'oversized.json',51*1024*1024);
  assert.match(h.elements.error.textContent,/50 MB/);
  await h.fileInput(original);assert.equal(h.elements.error.hidden,true);
});

test('loading a trace prevents a concurrent build and resets its import notice when editing starts',async()=>{
  const h=harness();let resolveText;
  h.elements.traceFile.files=[{name:'slow-read.json',size:100,text:()=>new Promise(resolve=>{resolveText=resolve;})}];
  const pending=h.elements.traceFile.onchange();
  assert.equal(h.evaluate('busy'),true);assert.equal(h.elements.runBtn.disabled,true);
  await h.evaluate('runExperiment()'); // A network call would throw in this harness.
  resolveText(JSON.stringify(fixture()));await pending;
  assert.equal(h.evaluate('busy'),false);assert.equal(h.elements.loadTraceBtn.disabled,false);
  h.elements.editStartBtn.onclick();assert.equal(h.evaluate('run'),null);
  assert.equal(h.elements.traceNotice.hidden,true);
});

test('accepts original traces from before branch beacon firmware was added',async()=>{
  const h=harness(),trace=fixture();delete trace.metadata.beacon_mode;
  for(const frame of trace.frames)delete frame.ble_threshold;
  await h.fileInput(JSON.stringify(trace));assert.equal(h.elements.error.hidden,true);
  assert.match(h.elements.provenance.textContent,/legacy run/);
});

test('selecting custom arrangement clears the previous 1:17.6 recording and starts at zero',async()=>{
  const h=harness(),trace=fixture();trace.result.t=77.67;trace.frames.at(-1).t=77.67;trace.logs.at(-1).t=77.67;
  await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  assert.equal(h.elements.duration.textContent,'Total 1:17.6');
  h.elements.preset.value='custom';h.elements.preset.onchange();
  assert.equal(h.evaluate('run'),null);assert.equal(h.evaluate('playTime'),0);
  assert.equal(h.elements.clock.textContent,'0:00.0');assert.equal(h.elements.duration.textContent,'Not run');
  assert.equal(h.elements.timeline.value,0);assert.equal(h.elements.timeline.disabled,true);
  assert.equal(h.elements.playBtn.disabled,true);assert.equal(h.elements.outcome.hidden,true);
  assert.equal(h.elements.jsonBtn.disabled,true);assert.match(h.elements.runStatus.textContent,/not run yet/);
});

test('typing a new starting condition or changing source invalidates the old result immediately',async()=>{
  const h=harness();await h.fileInput(JSON.stringify(fixture()));h.elements.endBtn.onclick();
  h.elements.x.value='-2.64';h.elements.x.listeners.input();
  assert.equal(h.evaluate('run'),null);assert.equal(h.evaluate('frame.x'),-2.64);
  assert.equal(h.elements.clock.textContent,'0:00.0');assert.equal(h.elements.duration.textContent,'Not run');
  await h.fileInput(JSON.stringify(fixture()));h.elements.branch.value='origin/main';h.elements.branch.onchange();
  assert.equal(h.evaluate('run'),null);assert.equal(h.elements.duration.textContent,'Not run');
  await h.fileInput(JSON.stringify(fixture()));h.elements.customRef.value='origin/Alvin';h.elements.customRef.oninput();
  assert.equal(h.evaluate('run'),null);assert.equal(h.elements.duration.textContent,'Not run');
  h.elements.exponent.value='';h.elements.exponent.listeners.input();
  assert.equal(h.elements.clock.textContent,'0:00.0');assert.match(h.elements.runStatus.textContent,/Finish entering/);
});

test('the original build flow submits the selected setup and plays the new execution automatically',async()=>{
  let resolveRun;const requests=[];
  const h=harness((url,options)=>{
    requests.push({url,body:options?.body&&JSON.parse(options.body)});
    if(url==='/api/run')return new Promise(resolve=>{resolveRun=resolve;});
    if(url==='/api/runs')return Promise.resolve({ok:true,json:async()=>[]});
    throw Error('Unexpected request '+url);
  });
  await h.fileInput(JSON.stringify(fixture()));h.elements.endBtn.onclick();
  h.elements.x.value='-2.64';h.elements.x.listeners.input();
  h.elements.heading.value='314.79';h.elements.heading.listeners.input();
  h.elements.customRef.value='origin/main';h.elements.customRef.oninput();
  const pending=h.evaluate('runExperiment()');
  assert.equal(requests.length,1);assert.equal(requests[0].url,'/api/run');
  assert.equal(requests[0].body.scenario.x,-2.64);assert.equal(requests[0].body.scenario.heading,314.79);
  assert.equal(h.evaluate('run'),null);assert.equal(h.elements.clock.textContent,'0:00.0');
  assert.equal(h.elements.duration.textContent,'Not run');
  assert.equal(h.evaluate('busy'),true);assert.equal(h.elements.runBtn.disabled,true);
  const response=fixture();response.id='fresh-execution';response.metadata.ref='origin/main';
  response.scenario=requests[0].body.scenario;response.result.t=10.25;response.frames.at(-1).t=10.25;response.logs.at(-1).t=10.25;
  resolveRun({ok:true,json:async()=>response});await pending;
  assert.equal(h.evaluate('run.id'),'fresh-execution');assert.equal(h.evaluate('playTime'),0);
  assert.equal(h.evaluate('playing'),true);
  assert.equal(h.elements.duration.textContent,'Total 0:10.2');
  assert.equal(h.elements.heading.value,'314.79');assert.equal(h.elements.x.disabled,false);
  assert.equal(h.elements.runBtn.disabled,false);assert.equal(requests.filter(r=>r.url==='/api/run').length,1);
});

test('playback restores the original default of 20 times speed',()=>{
  const html=fs.readFileSync(path.join(root,'dist/index.html'),'utf8');
  const options=html.match(/<select id="playSpeed"[^>]*>(.*?)<\/select>/s)[1];
  assert.match(options,/<option value="20" selected>/);assert.doesNotMatch(options,/<option value="1" selected>/);
});

test('every build click requests a separate fresh experiment even when its duration is unchanged',async()=>{
  let builds=0;
  const h=harness(async(url,options)=>{
    if(url==='/api/runs')return {ok:true,json:async()=>[]};
    assert.equal(url,'/api/run');assert.equal(options.method,'POST');assert.equal(options.cache,'no-store');
    const request=JSON.parse(options.body),response=fixture();
    response.id='fresh-run-'+(++builds);response.scenario=request.scenario;response.metadata.robot_build=request.build_target;
    return {ok:true,json:async()=>response};
  });
  h.elements.customRef.value='origin/main';
  await h.evaluate('runExperiment()');assert.equal(h.evaluate('run.id'),'fresh-run-1');
  h.elements.endBtn.onclick();assert.equal(h.evaluate('playTime'),2);
  await h.evaluate('runExperiment()');assert.equal(builds,2);assert.equal(h.evaluate('run.id'),'fresh-run-2');
  assert.equal(h.evaluate('playTime'),0);assert.equal(h.evaluate('frame.t'),0);
  assert.equal(h.evaluate('playing'),true);assert.match(h.elements.runStatus.textContent,/Playing from the start/);
  assert.equal(h.evaluate('frame.path'),0);assert.equal(h.evaluate('frame.faults'),0);
  assert.match(h.elements.runStatus.textContent,/New experiment.*fresh-run-2/);
  assert.equal(h.elements.duration.textContent,'Total 0:02.0');
});

function routeFixture(success=false){
  const trace=fixture(success);
  trace.metadata.robot_build='route';trace.metadata.model_version=3;
  trace.scenario={...trace.scenario,heading:0,mat_radius:.35,sensor_offset:.08,box_radius:.07,radio_range:5,colour_enabled:1,
    beacons:[{id:1,x:1,y:0,mat:'pink'},{id:2,x:1,y:1.5,mat:'blue'},{id:3,x:-.5,y:1.5,mat:'purple'}]};
  trace.scenario.mats=structuredClone(trace.scenario.beacons);
  trace.result.completed_mask=success?7:1;
  trace.frames=trace.frames.map((f,i)=>({...f,active_target:i===0?1:2,completed_mask:i===0?0:success&&i===2?7:1,ble_threshold:i===0?-58:-62,
    beacons:trace.scenario.beacons.map(b=>({...b,wifi:-50-b.id,ble:-60-b.id,verified:b.id===2,complete:true,rx_bytes:b.id*10,tx_bytes:b.id*20,peer_state:'AP'+b.id+' state',ssid:'SSID AP'+b.id})),
    sensor:{x:f.x+.08,y:f.y,enabled:true,mat_id:2,band:3,c:6650,r:1493,g:2235,b:2739},collision:i===2}));
  trace.logs.splice(1,0,{t:1,source:'beacon2',message:'AP2 reply bytes sent'},
    {t:1,source:'firmware',message:'ROUTE: AP2 centre uncertain; stop'});
  return trace;
}

async function setupFile(h,data){
  const text=JSON.stringify(data);
  h.elements.importFile.files=[{name:'setup.json',size:Buffer.byteLength(text),text:async()=>text}];
  await h.elements.importFile.onchange();
}

function dragPoint(h,kind,x,y){
  const start=h.evaluate(kind==='heading'?'headingHandle(frame,700,400)':`worldToScreen(${x},${y},700,400)`);
  const arena=h.elements.arena;
  arena.onpointerdown({button:0,pointerId:7,clientX:start.x,clientY:start.y});
  return (toX,toY)=>{
    const end=h.evaluate(`worldToScreen(${toX},${toY},700,400)`);
    arena.onpointermove({pointerId:7,clientX:end.x,clientY:end.y});
    arena.onpointerup({pointerId:7});
  };
}

test('startup selects the three-beacon route and triangle with illustrative defaults',()=>{
  const h=harness();
  assert.equal(h.elements.robotBuild.value,'route');assert.equal(h.elements.preset.value,'triangle');
  const s=JSON.parse(h.evaluate('JSON.stringify(scenario())'));
  assert.deepEqual(s.beacons,[{id:1,x:1,y:0,mat:'pink'},{id:2,x:1,y:1.5,mat:'blue'},{id:3,x:-.5,y:1.5,mat:'purple'}]);
  assert.deepEqual([s.x,s.y,s.heading],[0,0,0]);
  assert.deepEqual([s.mat_radius,s.sensor_offset,s.box_radius,s.radio_range,s.colour_enabled],[.35,.08,.07,5,true]);
  const html=fs.readFileSync(path.join(root,'dist/index.html'),'utf8');
  assert.match(html,/Colour sensor ahead of driven axle · m/);assert.match(html,/not chair or sensor calibration/);
});

test('initialization only reads config/history and prefers the available route source, otherwise main',async()=>{
  for(const routeExists of [true,false]){
    const calls=[],refs=[{name:'origin/main',supported:true,commit:'main-sha'}];
    if(routeExists)refs.push({name:'codex/three-beacon-route',supported:true,commit:'route-sha'});
    const h=harness(async(url,options)=>{calls.push({url,options});return {ok:true,json:async()=>url==='/api/config'?{refs}:[]};});
    await h.evaluate('init()');
    assert.deepEqual(calls.map(c=>c.url),['/api/config','/api/runs']);assert.ok(calls.every(c=>!c.options.method));
    assert.equal(h.elements.branch.value,routeExists?'codex/three-beacon-route':'origin/main');
    assert.equal(h.elements.robotBuild.value,'route');
    assert.match(h.elements.branch.options.at(-1).textContent,/Current firmware working tree/);
  }
});

test('rearward and sideways presets only edit the setup and retain all three targets',()=>{
  const h=harness();
  for(const [preset,heading] of [['rearward',180],['sideways',90],['triangle',0]]){
    h.elements.preset.value=preset;h.elements.preset.onchange();
    assert.equal(Number(h.elements.heading.value),heading);assert.equal(h.evaluate('scenario().beacons.length'),3);
    assert.equal(h.evaluate('run'),null);assert.equal(h.elements.robotBuild.value,'route');
  }
});

test('each beacon and the heading handle can be dragged independently without executing firmware',()=>{
  const h=harness();
  dragPoint(h,'beacon',1,1.5)(2,2.5);
  assert.equal(h.evaluate('scenario().beacons[1].x'),2);assert.equal(h.evaluate('scenario().beacons[1].y'),2.5);
  assert.equal(h.evaluate('scenario().beacons[0].x'),1);assert.equal(h.evaluate('scenario().beacons[2].x'),-.5);
  dragPoint(h,'beacon',-.5,1.5)(-2,1.5);assert.equal(h.evaluate('scenario().beacons[2].x'),-2);
  dragPoint(h,'beacon',1,0)(1.5,0);assert.equal(h.evaluate('scenario().bx'),1.5);
  dragPoint(h,'heading')(1,0);assert.equal(Number(h.elements.heading.value),270);
  assert.equal(h.evaluate('run'),null);
});

test('single AP2 radio with three independent mats and wrong colours survives setup save/load',async()=>{
  const h=harness();h.elements.beaconCount.value='1';h.elements.singleBeacon.value='2';h.elements.beaconCount.onchange();
  h.elements.matLayout.value='three';h.elements.matLayout.onchange();
  h.elements.mat2.value='purple';h.elements.mat2.onchange();
  h.elements.m2x.value='4.25';h.elements.m2x.listeners.input();
  h.elements.customRef.value='codex/three-beacon-route';h.elements.saveScenario.onclick();
  const saved=JSON.parse(await h.downloads.at(-1).text());
  assert.equal(saved.build_target,'route');assert.deepEqual(saved.scenario.beacons,[{id:2,x:1,y:1.5,mat:'purple'}]);
  assert.equal(saved.scenario.mats.length,3);assert.equal(saved.scenario.mats[1].x,4.25);
  const next=harness();await setupFile(next,saved);
  assert.equal(next.elements.error.hidden,true);assert.equal(next.elements.robotBuild.value,'route');
  assert.deepEqual(JSON.parse(next.evaluate('JSON.stringify(scenario())')),saved.scenario);
  next.elements.saveScenario.onclick();assert.deepEqual(JSON.parse(await next.downloads[0].text()),saved);
  dragPoint(next,'mat',4.25,1.5)(3,1.5);
  assert.equal(next.evaluate('scenario().mats[1].x'),3);assert.equal(next.evaluate('scenario().beacons[0].x'),1);
});

test('route playback displays active AP2 telemetry while completion uses only the firmware mask',async()=>{
  const h=harness(),trace=routeFixture(false),before=JSON.stringify(trace);
  await h.fileInput(before);assert.equal(h.evaluate('playing'),false);h.elements.endBtn.onclick();
  assert.equal(h.elements.activeBeaconTitle.textContent,'Active target · AP2');
  assert.equal(h.elements.peerState.textContent,'AP2 state');assert.equal(h.elements.wifiModel.textContent,'-52 dBm');
  assert.equal(h.elements.bleModel.textContent,'-62 dBm');assert.equal(h.elements.bytes.textContent,'20 / 40 bytes');
  assert.equal(h.elements.peerCRC.textContent,'Verified');assert.equal(h.elements.robotCRC.textContent,'Pending');
  assert.match(h.elements.beaconSourceLabel.textContent,/SSID AP2/);
  assert.match(h.elements.routeProgress.innerHTML,/AP1 · Firmware verified/);
  assert.match(h.elements.routeProgress.innerHTML,/AP2 · Active target/);
  assert.doesNotMatch(h.elements.routeProgress.innerHTML,/AP2 · Firmware verified/);
  assert.match(h.elements.colourRead.textContent,/RGB 1493 \/ 2235 \/ 2739/);assert.equal(h.elements.boxCollision.textContent,'Collision flagged');
  assert.match(h.elements.outcomeTitle.textContent,/without a confirmed payload/);assert.equal(h.evaluate('run.result.success'),false);
  h.elements.logFilter.value='ble';h.elements.logFilter.onchange();assert.match(h.elements.events.innerHTML,/beacon2/);
  h.elements.jsonBtn.onclick();assert.equal(JSON.stringify(JSON.parse(await h.downloads[0].text())),before);
});

test('route success and each target verification replay exactly as recorded',async()=>{
  const h=harness(),trace=routeFixture(true);await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  assert.equal((h.elements.routeProgress.innerHTML.match(/Firmware verified/g)||[]).length,3);
  assert.match(h.elements.outcomeTitle.textContent,/Route confirmed by the firmware/);assert.match(h.elements.outcomeDetail.textContent,/mask 0x7/);
  h.elements.replayBtn.onclick();assert.equal(h.evaluate('playTime'),0);assert.equal(h.evaluate('playing'),true);
  assert.doesNotMatch(h.elements.routeProgress.innerHTML,/Firmware verified/);
});

test('route progress never uses a legacy payload boolean when the firmware mask was not recorded',async()=>{
  const h=harness(),trace=routeFixture();for(const f of trace.frames){delete f.completed_mask;f.payload_verified=true;}
  await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  assert.doesNotMatch(h.elements.routeProgress.innerHTML,/Firmware verified/);assert.equal(h.elements.robotCRC.textContent,'Pending');
});

test('BLE charts use changing per-frame thresholds and annotate target switches',async()=>{
  const h=harness(),trace=routeFixture();await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  h.drawCalls.length=0;h.evaluate("chartMode='ble';drawChart()");
  const labels=h.drawCalls.filter(c=>c.method==='fillText').map(c=>c.args[0]);
  assert.ok(labels.includes('→ AP2'));assert.ok(labels.includes('recorded firmware threshold'));
  // The threshold has a vertical step at the AP2 timestamp, rather than a single horizontal line.
  const lines=h.drawCalls.filter(c=>c.method==='lineTo');
  assert.ok(lines.some((line,i)=>i>0&&line.args[0]===lines[i-1].args[0]&&line.args[1]!==lines[i-1].args[1]));
  h.evaluate("chartMode='wifi'");assert.equal(h.evaluate('chartValue(frame)'),-52);
});

test('optional nested fields are validated before replacing a recording; IDs are unique within each array',async()=>{
  const h=harness(),original=routeFixture();await h.fileInput(JSON.stringify(original));
  const corruptions=[
    t=>t.scenario.beacons.push({...t.scenario.beacons[0]}),
    t=>t.scenario.beacons[1].id=1,
    t=>t.scenario.beacons[0].x=null,
    t=>t.scenario.mats[1].id=1,
    t=>t.frames[0].beacons[1].id=1,
    t=>t.frames[0].beacons[0].wifi='-50',
    t=>t.frames[0].beacons[0].verified=1,
    t=>t.frames[0].sensor.r=null,
    t=>t.frames[0].collision='yes',
    t=>t.frames[0].completed_mask=8,
    t=>t.result.completed_mask=-1,
    t=>t.frames[0].active_target=4,
    t=>t.scenario.colour_enabled='yes',
    t=>t.metadata.robot_build='substitute',
  ];
  for(const corrupt of corruptions){
    const bad=structuredClone(original);corrupt(bad);await h.fileInput(JSON.stringify(bad));
    assert.equal(h.elements.error.hidden,false);assert.equal(h.evaluate('JSON.stringify(run)'),JSON.stringify(original));
  }
  // IDs can occur once in each separate radio and floor-mat namespace.
  await h.fileInput(JSON.stringify(original));assert.equal(h.elements.error.hidden,true);
  const bad=structuredClone(original);bad.frames[0].beacons[0].x=Infinity;
  assert.throws(()=>h.evaluate(`validateTrace(${JSON.stringify(bad).replace('"x":null','"x":Infinity')})`),/finite/);
});

test('old full traces stay unchanged even after returning to edit their setup',async()=>{
  const h=harness(),trace=fixture(),before=JSON.stringify(trace);await h.fileInput(before);
  assert.equal(h.elements.robotBuild.value,'ap1');assert.equal(h.elements.beaconCount.value,'1');
  assert.equal(h.elements.colour_enabled.checked,false);h.elements.endBtn.onclick();h.elements.jsonBtn.onclick();
  assert.equal(JSON.stringify(JSON.parse(await h.downloads[0].text())),before);
  h.elements.editStartBtn.onclick();assert.equal(h.evaluate('run'),null);
  assert.equal(h.elements.beaconCount.value,'1');assert.equal(h.elements.robotBuild.value,'ap1');
});

test('only an explicit build submits build_target, locks setup, and autoplays the fresh trace',async()=>{
  let resolveRun;const calls=[];
  const h=harness((url,options)=>{calls.push({url,options});
    if(url==='/api/run')return new Promise(resolve=>{resolveRun=resolve;});
    if(url==='/api/runs')return Promise.resolve({ok:true,json:async()=>[]});
    throw Error('Unexpected request '+url);
  });
  h.elements.customRef.value='codex/three-beacon-route';h.elements.customRef.oninput();
  h.elements.preset.value='rearward';h.elements.preset.onchange();assert.equal(calls.length,0);
  const pending=h.elements.runBtn.onclick();const request=JSON.parse(calls[0].options.body);
  assert.equal(request.build_target,'route');assert.equal(request.ref,'codex/three-beacon-route');
  assert.equal(request.scenario.beacons.length,3);assert.equal(request.scenario.heading,180);
  for(const id of ['x','b2x','b3y','robotBuild','matLayout','colour_enabled','branch','customRef','importFile','traceFile','runBtn'])assert.equal(h.elements[id].disabled,true,id+' must be locked');
  await h.elements.runBtn.onclick();assert.equal(calls.length,1);
  const response=routeFixture();response.scenario=request.scenario;
  resolveRun({ok:true,json:async()=>response});await pending;
  assert.equal(h.evaluate('playing'),true);assert.equal(h.elements.playSpeed.value,'20');
  assert.equal(h.elements.robotBuild.disabled,false);assert.equal(h.elements.x.disabled,false);
  assert.equal(calls.filter(call=>call.url==='/api/run').length,1);
});

test('incompatible route build errors are shown without retrying or substituting main/AP1',async()=>{
  const calls=[],h=harness(async(url,options)=>{calls.push({url,options});return {ok:false,json:async()=>({error:'Selected source has no route build'})};});
  h.elements.customRef.value='origin/main';await h.elements.runBtn.onclick();
  assert.equal(calls.length,1);assert.equal(JSON.parse(calls[0].options.body).build_target,'route');
  assert.match(h.elements.error.textContent,/no route build/);assert.equal(h.evaluate('run'),null);
  assert.equal(h.elements.robotBuild.value,'route');assert.equal(h.elements.runBtn.disabled,false);
});

test('known source capabilities flag an unsupported route and block its build without changing selection',async()=>{
  const h=harness();h.evaluate("config={refs:[{name:'origin/main',supported:true,commit:'main',supported_builds:['ap1']}]};populateRefs(config.refs)");
  assert.equal(h.elements.robotBuild.value,'route');assert.match(h.elements.buildNote.textContent,/does not support/);
  assert.equal(h.elements.robotBuild.options.find(o=>o.value==='route').disabled,true);
  await h.elements.runBtn.onclick();assert.match(h.elements.error.textContent,/does not support/);
});

test('an engine response identifying another robot build is never displayed as the requested route',async()=>{
  const h=harness(async()=>({ok:true,json:async()=>({...routeFixture(),metadata:{...routeFixture().metadata,robot_build:'ap1'}})}));
  await h.elements.runBtn.onclick();assert.equal(h.evaluate('run'),null);assert.match(h.elements.error.textContent,/different robot build/);
});

test('history inspection reads a saved run and opens paused at the start',async()=>{
  const calls=[],trace=routeFixture(),h=harness(async(url,options)=>{calls.push({url,options});return {ok:true,json:async()=>trace};});
  await h.elements.history.onclick({target:{closest:()=>({dataset:{run:trace.id}})}});
  assert.deepEqual(calls.map(c=>c.url),['/api/run/'+trace.id]);assert.ok(!calls[0].options.method);
  assert.equal(h.evaluate('playing'),false);assert.equal(h.evaluate('playTime'),0);
  assert.equal(h.evaluate('JSON.stringify(run)'),JSON.stringify(trace));
});

test('missing active radio has unavailable distance and chart gaps rather than a zero-distance arrival',async()=>{
  const h=harness();h.elements.beaconCount.value='1';h.elements.singleBeacon.value='2';h.elements.beaconCount.onchange();
  assert.equal(h.evaluate('previewFrame().distance'),null);assert.equal(h.elements.distance.innerHTML,'Unavailable');
  assert.equal(h.elements.peerState.textContent,'Target radio absent');
  h.evaluate("chartMode='distance'");assert.equal(h.evaluate('chartValue(frame)'),null);
  const trace=routeFixture(false);trace.scenario.beacons=trace.scenario.beacons.filter(b=>b.id!==2);
  for(const f of trace.frames){f.beacons=f.beacons.filter(b=>b.id!==2);f.distance=0;}
  await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  assert.equal(h.elements.distance.innerHTML,'Unavailable');assert.equal(h.evaluate('chartValue(frame)'),null);
  assert.match(h.elements.beaconPosition.innerHTML,/Target radio absent/);assert.equal(h.evaluate('run.result.success'),false);
  assert.match(h.elements.outcomeTitle.textContent,/without a confirmed payload/);
  await h.fileInput(JSON.stringify(fixture()));assert.equal(h.evaluate('chartValue(frame)'),1);
});

test('active target distance does not fall back to the legacy AP1 scalar in backend v3 frames',async()=>{
  const h=harness(),trace=routeFixture(false);
  for(const f of trace.frames){f.active_target=2;f.distance=.01;for(const peer of f.beacons)delete peer.distance;}
  await h.fileInput(JSON.stringify(trace));h.elements.endBtn.onclick();
  const f=trace.frames.at(-1),target=trace.scenario.beacons.find(b=>b.id===2);
  const expected=Math.hypot(f.x-target.x,f.y-target.y);
  assert.equal(h.evaluate('targetDistance(frame,run.scenario)'),expected);
  assert.equal(h.elements.distance.innerHTML,expected.toFixed(2)+' <small>m</small>');
  assert.equal(h.evaluate('JSON.stringify(run)'),JSON.stringify(trace));
});


test('closest approach follows the active beacon over recorded positions up to playback time without changing the trace',async()=>{
  const h=harness(),trace=routeFixture(false);
  // AP2 was passed while AP1 was active; its historical minimum must still count.
  // The future AP2 approach at t=2 must not appear when inspecting t=1.
  for(const [i,position] of [[1,1.25],[1,1],[1,1.4]].entries()){
    Object.assign(trace.frames[i],{x:position[0],y:position[1],closest:.01});
  }
  const original=JSON.stringify(trace);
  await h.fileInput(original);
  assert.equal(h.elements.closest.innerHTML,'1.25 <small>m</small>');
  h.elements.timeline.value='1';h.elements.timeline.oninput();
  assert.equal(h.elements.activeBeaconTitle.textContent,'Active target · AP2');
  assert.equal(h.elements.closest.innerHTML,'0.25 <small>m</small>');
  assert.equal(h.evaluate('frame.closest'),.01);
  h.elements.endBtn.onclick();assert.equal(h.elements.closest.innerHTML,'0.10 <small>m</small>');
  h.elements.timeline.value='1';h.elements.timeline.oninput();
  assert.equal(h.elements.closest.innerHTML,'0.25 <small>m</small>');
  assert.equal(h.evaluate('JSON.stringify(run)'),original);assert.equal(h.evaluate('run.result.success'),false);
  h.elements.jsonBtn.onclick();assert.equal(JSON.stringify(JSON.parse(await h.downloads[0].text())),original);
});

test('closest approach keeps legacy scalars, uses preview target distance, and is unavailable for an absent target',async()=>{
  const h=harness(),legacy=fixture();
  legacy.frames[0].closest=.123;await h.fileInput(JSON.stringify(legacy));
  assert.equal(h.elements.closest.innerHTML,'0.12 <small>m</small>');
  assert.equal(h.evaluate('closestApproach(frame,run.scenario)'),.123);
  h.elements.editStartBtn.onclick();
  assert.equal(h.evaluate('closestApproach(frame,scenario())'),h.evaluate('targetDistance(frame,scenario())'));
  h.elements.beaconCount.value='1';h.elements.singleBeacon.value='2';h.elements.beaconCount.onchange();
  assert.equal(h.elements.closest.innerHTML,'Unavailable');
  const route=routeFixture();route.scenario.beacons=route.scenario.beacons.filter(b=>b.id!==2);
  for(const f of route.frames){f.beacons=f.beacons.filter(b=>b.id!==2);f.closest=0;}
  await h.fileInput(JSON.stringify(route));h.elements.endBtn.onclick();
  assert.equal(h.elements.closest.innerHTML,'Unavailable');assert.equal(h.evaluate('run.result.success'),false);
});
