'use strict';
// Exercise the real file-input, playback and export handlers without browser packages.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const root=path.resolve(__dirname,'..');

function harness(fetchImplementation){
  const elements={},downloads=[],rect={left:0,top:0,width:700,height:400};
  const drawing=new Proxy({measureText:text=>({width:text.length*6}),createRadialGradient:()=>({addColorStop(){}})},
    {get:(target,key)=>key in target?target[key]:()=>{}});
  const element=()=>({value:'',style:{},dataset:{},children:[],options:[],listeners:{},classList:{toggle(){}},
    clientWidth:700,clientHeight:400,scrollHeight:0,scrollTop:0,parentElement:{},files:[],
    getBoundingClientRect:()=>rect,getContext:()=>drawing,reportValidity:()=>true,
    addEventListener(name,fn){this.listeners[name]=fn;},replaceChildren(){this.children=[];},
    append(child){this.children.push(child);},setPointerCapture(){},click(){}});
  const html=fs.readFileSync(path.join(root,'dist/index.html'),'utf8');
  for(const match of html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)){
    const value=match[0].match(/\bvalue="([^"]*)"/);
    elements[match[1]]={...element(),value:value?value[1]:'',hidden:match[0].includes(' hidden')};
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
  return {elements,downloads,evaluate,fileInput};
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

test('a new build clears old duration, locks its starting conditions and loads only the new execution',async()=>{
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
  assert.equal(h.elements.duration.textContent,'Computing…');
  for(const key of ['x','heading','preset','branch','loadTraceBtn','runBtn'])assert.equal(h.elements[key].disabled,true,key);
  const response=fixture();response.id='fresh-execution';response.metadata.ref='origin/main';
  response.scenario=requests[0].body.scenario;response.result.t=10.25;response.frames.at(-1).t=10.25;response.logs.at(-1).t=10.25;
  resolveRun({ok:true,json:async()=>response});await pending;
  assert.equal(h.evaluate('run.id'),'fresh-execution');assert.equal(h.evaluate('playTime'),0);
  assert.equal(h.elements.duration.textContent,'Total 0:10.2');
  assert.equal(h.elements.heading.value,'314.79');assert.equal(h.elements.x.disabled,false);
  assert.equal(h.elements.runBtn.disabled,false);assert.equal(requests.filter(r=>r.url==='/api/run').length,1);
});

test('playback defaults to real time instead of silently skipping ahead at 20 times speed',()=>{
  const html=fs.readFileSync(path.join(root,'dist/index.html'),'utf8');
  const options=html.match(/<select id="playSpeed"[^>]*>(.*?)<\/select>/s)[1];
  assert.match(options,/<option value="1" selected>/);assert.doesNotMatch(options,/<option value="20" selected>/);
});

test('every build click requests a separate fresh experiment even when its duration is unchanged',async()=>{
  let builds=0;
  const h=harness(async(url,options)=>{
    if(url==='/api/runs')return {ok:true,json:async()=>[]};
    assert.equal(url,'/api/run');assert.equal(options.method,'POST');assert.equal(options.cache,'no-store');
    const request=JSON.parse(options.body),response=fixture();
    response.id='fresh-run-'+(++builds);response.scenario=request.scenario;
    return {ok:true,json:async()=>response};
  });
  h.elements.customRef.value='origin/main';
  await h.evaluate('runExperiment()');assert.equal(h.evaluate('run.id'),'fresh-run-1');
  h.elements.endBtn.onclick();assert.equal(h.evaluate('playTime'),2);
  await h.evaluate('runExperiment()');assert.equal(builds,2);assert.equal(h.evaluate('run.id'),'fresh-run-2');
  assert.equal(h.evaluate('playTime'),0);assert.equal(h.evaluate('frame.t'),0);
  assert.equal(h.evaluate('frame.path'),0);assert.equal(h.evaluate('frame.faults'),0);
  assert.match(h.elements.runStatus.textContent,/New experiment.*fresh-run-2/);
  assert.equal(h.elements.duration.textContent,'Total 0:02.0');
});
