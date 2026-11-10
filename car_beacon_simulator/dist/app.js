'use strict';
const $=id=>document.getElementById(id);
const legacyKeys=['x','y','heading','bx','by','speed','wheelbase','wifi_ref','exponent','ble_radius','stop_at'];
const modelDefaults={mat_radius:.35,sensor_offset:.08,box_radius:.07,radio_range:5};
const keys=[...legacyKeys,'b2x','b2y','b3x','b3y','m1x','m1y','m2x','m2y','m3x','m3y',...Object.keys(modelDefaults)];
const setupControls=[...keys,'preset','branch','customRef','moving','robotBuild','beaconCount','singleBeacon','matLayout','mat1','mat2','mat3','colour_enabled','runBtn','fetchBtn','importFile','saveScenario','loadTraceBtn','traceFile','editStartBtn'];
const beaconColours={1:'#ff8fbd',2:'#7bafff',3:'#bf9bff'};
const matColours={pink:'#ff8fbd',blue:'#7bafff',purple:'#bf9bff',none:'#607786'};
const routeTargets={route:[1,2,3],route2:[1,2]};
const triangle={x:0,y:0,heading:0,bx:1,by:0,b2x:1,b2y:1.5,b3x:-.5,b3y:1.5};
let scenarioExtras={},twoBeaconIds=[1,2],customMatIds=[1,2,3];
const presets={triangle,rearward:{...triangle,heading:180},sideways:{...triangle,heading:90},straight:{x:0,y:0,heading:0,bx:1,by:0},diagonal:{x:0,y:0,heading:0,bx:1,by:1},far:{x:-2,y:-1,heading:0,bx:1,by:1},behind:{x:0,y:0,heading:0,bx:-1,by:0},close:{x:0,y:0,heading:0,bx:.15,by:0}};
let run=null,playing=false,playTime=0,frame=null,busy=false,chartMode='wifi',logMode='decisions',lastLogCount=-1;
let view={x:.5,y:0,span:5},drag=null,history=[],config=null;
const fmt=(v,n=2)=>Number(v).toFixed(n);
const norm=a=>((a%360)+360)%360;
const clock=t=>`${Math.floor(t/60)}:${String(Math.floor(t%60)).padStart(2,'0')}.${Math.floor((t%1)*10)}`;
const escape=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function beaconPrefix(id){return id===1?'b':`b${id}`;}
function setupBeacons(){
  return [1,2,3].map(id=>({id,x:Number($(beaconPrefix(id)+'x').value),y:Number($(beaconPrefix(id)+'y').value),mat:$('mat'+id).value}));
}
function setupMats(){return [1,2,3].map(id=>({id,x:Number($(`m${id}x`).value),y:Number($(`m${id}y`).value),mat:$('mat'+id).value}));}
function enabledIds(){return $('beaconCount').value==='1'?[Number($('singleBeacon').value)]:$('beaconCount').value==='2'?twoBeaconIds:[1,2,3];}
function scenario(){
  const s={horizon:1000,corrupt_reply:0,...scenarioExtras};
  for(const k of keys)s[k]=Number($(k).value);
  const ids=enabledIds(),positions=setupBeacons();
  s.beacons=positions.filter(b=>ids.includes(b.id)).map(b=>({...scenarioExtras.beacons?.find(old=>old.id===b.id),...b}));
  s.mats=$('matLayout').value==='enabled'?positions.filter(b=>ids.includes(b.id)):setupMats().filter(m=>$('matLayout').value==='three'||customMatIds.includes(m.id));
  s.colour_enabled=$('colour_enabled').checked;
  return s;
}
// Legacy recordings are read through this view; the original object is never changed.
function beaconsFor(s){return s.beacons||[{id:1,x:s.bx,y:s.by,mat:'none'}];}
function targetId(f){return f.active_target??1;}
function activeBeacon(f,s){return beaconsFor(s).find(b=>b.id===targetId(f));}
function activeTelemetry(f){return f.beacons?.find(b=>b.id===targetId(f))||{};}
function activeValue(f,key){return activeTelemetry(f)[key]??f[key];}
function targetAvailable(f,s){return !!activeBeacon(f,s)&&(!f.beacons||f.beacons.some(b=>b.id===targetId(f)));}
function targetDistance(f,s){
  if(!targetAvailable(f,s))return s.beacons||f.beacons?null:Number.isFinite(f.distance)?f.distance:null;
  const peer=activeTelemetry(f);
  if(Number.isFinite(peer.distance))return peer.distance;
  // Model v3 retains AP1-referenced legacy scalars. Modern traces provide
  // physical placements, so calculate the active target's display separately.
  if(s.beacons){const target=activeBeacon(f,s);return Math.hypot(f.x-target.x,f.y-target.y);}
  return Number.isFinite(f.distance)?f.distance:null;
}
// Playback-only diagnostic: legacy scalar closest remains untouched in the recording.
function closestApproach(f,s){
  if(!run)return targetDistance(f,s);
  if(!s.beacons)return f.closest;
  if(!targetAvailable(f,s))return null;
  const target=activeBeacon(f,s);
  let closest=Infinity;
  for(const point of run.frames){
    if(point.t>playTime)break;
    closest=Math.min(closest,Math.hypot(point.x-target.x,point.y-target.y));
  }
  return Number.isFinite(closest)?closest:null;
}
function previewReady(){
  const s=scenario();
  return keys.every(k=>$(k).value!==''&&Number.isFinite(s[k]))&&s.speed>0&&s.wheelbase>0&&s.exponent>0&&s.ble_radius>0&&s.mat_radius>0&&s.sensor_offset>=0&&s.box_radius>=0&&s.radio_range>0;
}
function updateSetupLabels(){
  const ids=enabledIds();
  $('singleBeacon').hidden=$('singleBeaconLabel').hidden=ids.length!==1;
  for(const id of [1,2,3]){
    $('beacon'+id+'Setup').classList.toggle('inactive',!ids.includes(id)&&$('matLayout').value==='enabled');
    $('mat'+id+'Position').hidden=$('matLayout').value==='enabled';
  }
  setText('entityCount',`01 car / 0${ids.length} beacon${ids.length===1?'':'s'}`);
}
function setBusy(value){busy=value;for(const id of setupControls)$(id).disabled=value;}
function showError(message){$('error').hidden=!message;$('error').textContent=message||'';}
async function api(path,body){const r=await fetch(path,{cache:'no-store',...(body?{method:'POST',headers:{'Content-Type':'application/json','X-Simulator':'1'},body:JSON.stringify(body)}:{})});const d=await r.json();if(!r.ok)throw Error(d.error||`Request failed (${r.status})`);return d;}
function populateRefs(refs){
  const old=$('branch').value;
  $('branch').replaceChildren();
  for(const ref of refs){
    const option=document.createElement('option');option.value=ref.name;
    option.textContent=`${ref.name} ${ref.supported?'':'· needs adapter'}`;option.disabled=!ref.supported;
    $('branch').append(option);
  }
  const option=document.createElement('option');option.value='working-tree';option.textContent='Current firmware working tree · saved edits';$('branch').append(option);
  const preferred=['codex/three-beacon-route','origin/codex/three-beacon-route','origin/main','main'].find(name=>refs.some(ref=>ref.name===name));
  $('branch').value=old&&[...$('branch').options].some(o=>o.value===old)?old:preferred||'working-tree';
  branchNote();
}
function branchNote(){
  const custom=$('customRef').value.trim(),current=config?.refs.find(r=>r.name===$('branch').value),build=$('robotBuild').value;
  $('branchNote').textContent=custom?`Next run: ${custom} · robot ${build} + enabled beacons`:current?`${current.commit} · robot ${build} + enabled beacons from this exact branch`:'Snapshots saved source in the engine’s configured repository; other worktrees are not included.';
  const details=$('sourceDetails');details.replaceChildren();
  if(!custom&&current?.supported){
    for(const [label,path] of [['Robot',current.car_source],['Beacon',current.beacon_source]])if(path){const item=document.createElement('div');item.textContent=label+': '+path;details.append(item);}
  }else details.textContent='Source paths are resolved and recorded when you build.';
  const supported=!custom&&current?.supported_builds;
  for(const option of $('robotBuild').options)option.disabled=Array.isArray(supported)&&!supported.includes(option.value);
  $('buildNote').textContent=Array.isArray(supported)&&!supported.includes(build)?`This source does not support robot ${build}. Select a compatible source or build.`:routeTargets[build]?'Switching routes sets the matching radios. You can change enabled radios separately to test missing beacons.':'Requires a compatible source build.';
}

// A threshold is known only from compiled firmware telemetry; legacy traces used -50.
function firmwareThreshold(f){return run?(activeValue(f,'ble_threshold')??(routeTargets[run.metadata.robot_build]?null:-50)):null;}
function previewFrame(){
  const s=scenario(),build=$('robotBuild').value,id=routeTargets[build]?.[0]??Number(build.slice(2));
  const beacons=s.beacons.map(b=>{
    const distance=Math.hypot(s.x-b.x,s.y-b.y);
    return {...b,distance,wifi:Math.max(-126,Math.min(-1,s.wifi_ref-10*s.exponent*Math.log10(Math.max(.01,distance)))),ble:Math.max(-126,Math.min(-1,-50-10*s.exponent*Math.log10(Math.max(.01,distance)/s.ble_radius))),peer_state:'Advertising',rx_bytes:0,tx_bytes:0};
  });
  const target=beacons.find(b=>b.id===id),distance=target?.distance??null;
  return {t:0,x:s.x,y:s.y,heading:s.heading,gyro_heading:0,distance,closest:distance,path:0,wifi:target?.wifi,ble:target?.ble,beacons,active_target:id,completed_mask:0,left:0,right:0,left_pwm:0,right_pwm:0,wifi_observed:-127,ble_observed:-127,wifi_scans:0,ble_reports:0,ble_attempts:0,faults:0,ble_state:'Idle',peer_state:target?.peer_state||'Target radio disabled',ble_samples:0,ble_close_count:0,ble_filtered:-127,ble_threshold:null,rx_bytes:0,tx_bytes:0};
}
function resetPreview(fit=false){
  playing=false;run=null;playTime=0;frame=previewFrame();lastLogCount=-1;updateSetupLabels();
  for(const id of ['playBtn','stepBtn','endBtn','timeline','csvBtn','jsonBtn'])$(id).disabled=true;
  $('traceNotice').hidden=true;$('editStartBtn').hidden=true;$('timeline').value=0;$('timeline').max=1;
  $('duration').textContent='Not run';$('duration').title='The total duration is known only after running or loading a recording.';
  $('outcome').hidden=true;$('events').innerHTML='<div class="empty">Run an experiment to inspect the firmware’s decisions and sensor readings.</div>';
  $('arenaState').textContent='READY TO RUN';$('mapHint').textContent='Drag car to move · drag circle to rotate · scroll to zoom';$('playBtn').textContent='▶';
  $('runStatus').textContent=previewReady()?'New setup · not run yet. Press Build & run experiment to execute it.':'Finish entering the starting conditions before running.';
  $('provenance').textContent='Native C execution · deterministic hardware model · local only';if(fit&&previewReady())fitView();render();
}
function writeScenario(s){
  scenarioExtras={...s};
  for(const k of legacyKeys)if(k in s)$(k).value=fmt(s[k]);
  for(const [k,v] of Object.entries(modelDefaults))$(k).value=String(s[k]??v);
  // Defaults belong only to editable controls, never to the imported recording.
  for(const id of [1,2,3]){
    const b=s.beacons?.find(b=>b.id===id),mat=s.mats?.find(m=>m.id===id),prefix=beaconPrefix(id);
    $(prefix+'x').value=String(b?.x??s[prefix+'x']??triangle[prefix+'x']);
    $(prefix+'y').value=String(b?.y??s[prefix+'y']??triangle[prefix+'y']);
    $('mat'+id).value=mat?.mat??b?.mat??['pink','blue','purple'][id-1];
    $(`m${id}x`).value=String(mat?.x??b?.x??s[prefix+'x']??triangle[prefix+'x']);
    $(`m${id}y`).value=String(mat?.y??b?.y??s[prefix+'y']??triangle[prefix+'y']);
  }
  const ids=beaconsFor(s).map(b=>b.id);
  $('beaconCount').value=String(ids.length);$('singleBeacon').value=String(ids[0]);if(ids.length===2)twoBeaconIds=ids;
  customMatIds=s.mats?.map(m=>m.id)||[1,2,3];
  const independent=s.mats&&(s.mats.length!==beaconsFor(s).length||s.mats.some(m=>{const b=s.beacons?.find(b=>b.id===m.id);return !b||b.x!==m.x||b.y!==m.y;}));
  $('matLayout').value=independent?(s.mats.length===3?'three':'custom'):'enabled';
  $('colour_enabled').checked=s.colour_enabled??false;
  updateSetupLabels();
}
function applyScenario(s){writeScenario(s);resetPreview(true);}
function applyPreset(name){
  const preset=presets[name];
  if(!preset){resetPreview();return;}
  const isTriangle=['triangle','rearward','sideways'].includes(name);
  const next={...scenario(),...preset};
  delete next.beacons;delete next.mats;
  // Presets alter placement, while build/source and illustrative model choices stay explicit.
  for(const k of ['bx','by','b2x','b2y','b3x','b3y'])next[k]=preset[k]??Number($(k).value);
  const ids=isTriangle?(routeTargets[$('robotBuild').value]??[1,2,3]):enabledIds();
  next.beacons=setupBeacons().filter(b=>ids.includes(b.id)).map(b=>({...b,x:next[beaconPrefix(b.id)+'x'],y:next[beaconPrefix(b.id)+'y']}));
  applyScenario(next);
}
function fitView(){
  const s=run?.scenario||scenario(),pts=[...(run?run.frames:[{x:s.x,y:s.y}]),...beaconsFor(s),...(s.mats||[])];
  let minx=Infinity,maxx=-Infinity,miny=Infinity,maxy=-Infinity;
  for(const p of pts){minx=Math.min(minx,p.x);maxx=Math.max(maxx,p.x);miny=Math.min(miny,p.y);maxy=Math.max(maxy,p.y);}
  const ratio=$('arena').clientWidth/Math.max(1,$('arena').clientHeight),margin=2*Math.max(.75,s.mat_radius||0);
  view={x:(minx+maxx)/2,y:(miny+maxy)/2,span:Math.max(3,maxy-miny+margin,(maxx-minx+margin)/ratio)};
}
function canvasContext(id){const c=$(id),rect=c.getBoundingClientRect(),dpr=devicePixelRatio||1;if(c.width!==Math.round(rect.width*dpr)||c.height!==Math.round(rect.height*dpr)){c.width=Math.round(rect.width*dpr);c.height=Math.round(rect.height*dpr);}const ctx=c.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);return{ctx,w:rect.width,h:rect.height};}
function worldToScreen(x,y,w,h){const scale=h/view.span;return{x:w/2+(x-view.x)*scale,y:h/2-(y-view.y)*scale};}
function screenToWorld(x,y,w,h){const scale=h/view.span;return{x:view.x+(x-w/2)/scale,y:view.y-(y-h/2)/scale};}
function headingHandle(f,w,h){const p=worldToScreen(f.x,f.y,w,h),angle=(f.heading+90)*Math.PI/180;return{x:p.x+50*Math.cos(angle),y:p.y-50*Math.sin(angle)};}
function drawHeadingHandle(c,f,w,h){
  const p=worldToScreen(f.x,f.y,w,h),handle=headingHandle(f,w,h),active=drag?.kind==='heading';
  c.save();c.strokeStyle=active?'#6ee7bb90':'#6ee7bb35';c.lineWidth=1;c.setLineDash([3,5]);c.beginPath();c.arc(p.x,p.y,50,0,Math.PI*2);c.stroke();c.setLineDash([]);
  c.strokeStyle='#6ee7bb80';c.beginPath();c.moveTo(p.x+(handle.x-p.x)*.55,p.y+(handle.y-p.y)*.55);c.lineTo(handle.x,handle.y);c.stroke();
  c.fillStyle=active?'#6ee7bb':'#163a30';c.strokeStyle='#6ee7bb';c.lineWidth=2;c.beginPath();c.arc(handle.x,handle.y,9,0,Math.PI*2);c.fill();c.stroke();
  c.fillStyle=active?'#163a30':'#a6f4d5';c.beginPath();c.arc(handle.x,handle.y,2.5,0,Math.PI*2);c.fill();
  const label=`Rotate · ${fmt(norm(f.heading),1)}°`;c.font='10px ui-monospace,monospace';const width=c.measureText(label).width;
  const lx=Math.max(8,Math.min(w-width-8,handle.x-width/2)),ly=handle.y<30?handle.y+26:handle.y-18;
  c.fillStyle='#10171fee';c.fillRect(lx-4,ly-11,width+8,16);c.fillStyle='#a6d9c3';c.fillText(label,lx,ly);c.restore();
}
function drawBeaconLayout(c,s,f,w,h,scale){
  const enabled=beaconsFor(s),positions=s.mats||enabled;
  for(const mat of positions){
    if(!s.mat_radius||mat.mat==='none')continue;
    const p=worldToScreen(mat.x,mat.y,w,h),colour=matColours[mat.mat]||'#607786';
    // These bands are a visual key for the illustrative radial colour model.
    const bands=mat.mat==='pink'?6:7;
    for(let band=0;band<bands;band++){
      const fraction=(bands-band)/bands,alpha=band%2?'18':'22';
      c.fillStyle=colour+alpha;c.strokeStyle=colour+'55';c.lineWidth=1;
      c.beginPath();c.arc(p.x,p.y,s.mat_radius*fraction*scale,0,Math.PI*2);c.fill();c.stroke();
    }
    if(!enabled.some(b=>b.id===mat.id&&b.x===mat.x&&b.y===mat.y)){
      c.fillStyle=colour;c.font='10px ui-monospace,monospace';c.fillText(`AP${mat.id} mat`,p.x+12,p.y-10);c.strokeStyle=colour;c.beginPath();c.arc(p.x,p.y,5,0,Math.PI*2);c.stroke();
    }
  }
  for(const beacon of enabled){
    const b=worldToScreen(beacon.x,beacon.y,w,h),colour=beaconColours[beacon.id];
    if(s.box_radius){c.fillStyle='#ff8c9430';c.strokeStyle='#ff8c9470';c.beginPath();c.arc(b.x,b.y,s.box_radius*scale,0,Math.PI*2);c.fill();c.stroke();}
    if(beacon.id===targetId(f)){
      const threshold=firmwareThreshold(f);
      if(threshold!==null){c.fillStyle=colour+'10';c.strokeStyle=colour+'90';c.setLineDash([4,5]);c.beginPath();c.arc(b.x,b.y,s.ble_radius*Math.pow(10,(-50-threshold)/(10*s.exponent))*scale,0,Math.PI*2);c.fill();c.stroke();c.setLineDash([]);}
    }
    c.fillStyle=colour;c.beginPath();c.arc(b.x,b.y,8,0,Math.PI*2);c.fill();
    c.strokeStyle=colour+'a0';c.beginPath();c.arc(b.x,b.y,14,0,Math.PI*2);c.stroke();
    c.fillStyle='#192127';c.beginPath();c.arc(b.x,b.y,3,0,Math.PI*2);c.fill();
    c.fillStyle=colour;c.font='bold 10px sans-serif';c.fillText(`AP${beacon.id}`,b.x+19,b.y-3);
    c.font='9px ui-monospace,monospace';c.fillText(beacon.mat&&beacon.mat!=='none'?beacon.mat+' mat':'No mat',b.x+19,b.y+11);
  }
}
function drawSensor(c,s,f,w,h){
  if(s.sensor_offset===undefined)return;
  const angle=f.heading*Math.PI/180;
  const sensor=f.sensor||f.colour?.sensor||f.sensor_position||{x:f.x+s.sensor_offset*Math.cos(angle),y:f.y+s.sensor_offset*Math.sin(angle)};
  const p=worldToScreen(sensor.x,sensor.y,w,h),car=worldToScreen(f.x,f.y,w,h);
  c.strokeStyle='#dce8f090';c.beginPath();c.moveTo(car.x,car.y);c.lineTo(p.x,p.y);c.stroke();
  c.fillStyle=!s.colour_enabled?'#70808d':'#ffffff';c.beginPath();c.arc(p.x,p.y,3,0,Math.PI*2);c.fill();
}
function drawArena(){const {ctx:c,w,h}=canvasContext('arena');if(!w||!h)return;c.clearRect(0,0,w,h);const s=run?.scenario||scenario(),f=frame||previewFrame(),scale=h/view.span;
  if(!run&&!previewReady())return;
  const lo=screenToWorld(0,h,w,h),hi=screenToWorld(w,0,w,h);const grid=view.span>20?5:1;
  c.lineWidth=1;for(let x=Math.ceil(lo.x/grid)*grid;x<=hi.x;x+=grid){const p=worldToScreen(x,0,w,h);c.strokeStyle=x===0?'#2a3b47':'#1b2934';c.beginPath();c.moveTo(p.x,0);c.lineTo(p.x,h);c.stroke();}for(let y=Math.ceil(lo.y/grid)*grid;y<=hi.y;y+=grid){const p=worldToScreen(0,y,w,h);c.strokeStyle=y===0?'#2a3b47':'#1b2934';c.beginPath();c.moveTo(0,p.y);c.lineTo(w,p.y);c.stroke();}
  const target=activeBeacon(f,s),b=target?worldToScreen(target.x,target.y,w,h):null,p=worldToScreen(f.x,f.y,w,h),start=worldToScreen(s.x,s.y,w,h);
  drawBeaconLayout(c,s,f,w,h,scale);
  if(run){c.strokeStyle='#6ee7bb90';c.lineWidth=2;c.beginPath();let first=true;for(const pt of run.frames){if(pt.t>playTime)break;const q=worldToScreen(pt.x,pt.y,w,h);if(first){c.moveTo(q.x,q.y);first=false;}else c.lineTo(q.x,q.y);}c.stroke();c.strokeStyle='#638477';c.setLineDash([3,3]);c.beginPath();c.arc(start.x,start.y,9,0,Math.PI*2);c.stroke();c.setLineDash([]);c.fillStyle='#77978a';c.font='9px ui-monospace,monospace';c.fillText('START',start.x+13,start.y+4);}
  if(b){
    c.strokeStyle='#8ba2af65';c.lineWidth=1;c.setLineDash([4,5]);c.beginPath();c.moveTo(p.x,p.y);c.lineTo(b.x,b.y);c.stroke();c.setLineDash([]);
    const mx=(p.x+b.x)/2,my=(p.y+b.y)/2,text=fmt(targetDistance(f,s))+' m';c.font='10px ui-monospace,monospace';const tw=c.measureText(text).width;c.fillStyle='#10171fee';c.fillRect(mx-tw/2-6,my-10,tw+12,19);c.fillStyle='#aabcc8';c.fillText(text,mx-tw/2,my+3);
  }
  c.save();c.translate(p.x,p.y);c.rotate(-f.heading*Math.PI/180);const size=Math.max(18,Math.min(30,.22*scale));c.fillStyle='#0c1116';c.strokeStyle='#7ca997';c.lineWidth=1;c.fillRect(-size*.6,-size*.58,size*.72,5);c.strokeRect(-size*.6,-size*.58,size*.72,5);c.fillRect(-size*.6,size*.58-5,size*.72,5);c.strokeRect(-size*.6,size*.58-5,size*.72,5);c.fillStyle='#193e32';c.strokeStyle='#6ee7bb';c.lineWidth=1.5;c.beginPath();c.roundRect(-size*.7,-size*.43,size*1.4,size*.86,5);c.fill();c.stroke();c.fillStyle='#6ee7bb';c.beginPath();c.moveTo(size*.85,0);c.lineTo(size*.3,-size*.21);c.lineTo(size*.3,size*.21);c.closePath();c.fill();c.fillStyle='#578e79';c.fillRect(-size*.3,-size*.2,size*.3,size*.4);c.restore();
  c.fillStyle='#8fbcaa';c.font='9px ui-monospace,monospace';c.fillText('CAR',p.x-11,p.y+33);
  drawSensor(c,s,f,w,h);
  if(!run&&!busy)drawHeadingHandle(c,f,w,h);
  const bar=scale*grid;c.strokeStyle='#607786';c.beginPath();c.moveTo(19,h-39);c.lineTo(19+bar,h-39);c.stroke();c.fillStyle='#718797';c.font='9px ui-monospace,monospace';c.fillText(`${grid} m`,20,h-44);
}
function chartValue(f){const s=run?.scenario||scenario();return chartMode==='distance'?targetDistance(f,s):targetAvailable(f,s)?activeValue(f,chartMode):null;}
function drawChart(){
  const {ctx:c,w,h}=canvasContext('chart');c.clearRect(0,0,w,h);if(!run&&!previewReady())return;
  const pad={l:43,r:17,t:17,b:21},cw=w-pad.l-pad.r,ch=h-pad.t-pad.b,frames=run?.frames||[frame||previewFrame()];
  const vals=frames.map(chartValue).filter(Number.isFinite);
  if(chartMode==='ble')for(const f of frames){const threshold=firmwareThreshold(f);if(threshold!==null)vals.push(threshold);}
  let min=vals.length?Math.min(...vals):0,max=vals.length?Math.max(...vals):1;
  if(chartMode==='distance'){min=0;max=Math.max(1,max*1.1);}else{min=Math.floor((min-3)/5)*5;max=Math.ceil((max+3)/5)*5;}
  const duration=run?.result.t||10,px=t=>pad.l+t/duration*cw,py=v=>pad.t+(max-v)/(max-min||1)*ch;
  c.font='8px ui-monospace,monospace';
  for(let i=0;i<=3;i++){const val=min+(max-min)*i/3,y=py(val);c.strokeStyle='#25323d';c.lineWidth=1;c.beginPath();c.moveTo(pad.l,y);c.lineTo(w-pad.r,y);c.stroke();c.fillStyle='#7f95a7';c.fillText(fmt(val,chartMode==='distance'?1:0),8,y+3);}
  for(let i=0;i<=4;i++){const t=duration*i/4;c.fillStyle='#7f95a7';c.fillText(`${Math.round(t)}s`,px(t)-7,h-4);}
  if(chartMode==='ble'&&run){
    c.strokeStyle='#ffca7580';c.setLineDash([4,4]);c.beginPath();let previous=null;
    for(const f of frames){
      const threshold=firmwareThreshold(f);if(threshold===null){previous=null;continue;}
      if(previous){c.lineTo(px(f.t),py(previous.threshold));c.lineTo(px(f.t),py(threshold));}else c.moveTo(px(f.t),py(threshold));
      previous={threshold};
    }
    c.stroke();c.setLineDash([]);c.fillStyle='#ffca75';c.fillText('recorded firmware threshold',pad.l,10);
  }
  c.lineWidth=1.7;c.beginPath();let previous=null;
  for(const f of frames){
    if(f.t>playTime)break;
    const value=chartValue(f);if(!Number.isFinite(value)){previous=null;continue;}
    if(!previous||targetId(previous)!==targetId(f)){
      c.stroke();c.strokeStyle=beaconColours[targetId(f)]||'#6ee7bb';c.beginPath();c.moveTo(px(f.t),py(value));
    }else c.lineTo(px(f.t),py(value));
    previous=f;
  }
  c.stroke();
  if(chartMode!=='distance'&&run){
    let previous='';c.fillStyle='#8ab5ff';
    for(const f of frames){if(f.t>playTime)break;const count=activeValue(f,chartMode==='wifi'?'wifi_reports':'ble_reports')??f.wifi_scans,obs=activeValue(f,chartMode+'_observed'),key=targetId(f)+':'+count;
      if(targetAvailable(f,run.scenario)&&key!==previous&&Number.isFinite(obs)&&obs!==-127){c.beginPath();c.arc(px(f.t),py(obs),1.8,0,Math.PI*2);c.fill();previous=key;}}
  }
  if(run){
    for(let i=1;i<frames.length;i++)if(targetId(frames[i])!==targetId(frames[i-1])){
      const f=frames[i];c.strokeStyle='#9c8db565';c.setLineDash([2,4]);c.beginPath();c.moveTo(px(f.t),pad.t);c.lineTo(px(f.t),h-pad.b);c.stroke();c.setLineDash([]);c.fillStyle=beaconColours[targetId(f)]||'#aabcc8';c.fillText(`→ AP${targetId(f)}`,Math.min(w-45,px(f.t)+3),pad.t+10);
    }
    c.strokeStyle='#d4e6ef60';c.beginPath();c.moveTo(px(playTime),pad.t);c.lineTo(px(playTime),h-pad.b);c.stroke();
  }else{c.fillStyle='#8d9ca9';c.font='10px sans-serif';c.fillText('Build an experiment to record the active target',pad.l+10,pad.t+ch/2);}
}
function setText(id,text){$(id).textContent=text;}
function routeStatus(f,id){
  if(Number.isInteger(f.completed_mask))return f.completed_mask&(1<<(id-1))?'Firmware verified':targetId(f)===id?'Active target':'Pending';
  if(!routeTargets[run?.metadata.robot_build]&&id===1&&f.payload_verified)return 'Firmware verified';
  return targetId(f)===id?'Active target':'Not recorded';
}
function renderRoute(f,s){
  const build=run?(run.metadata.robot_build||'ap1'):$('robotBuild').value;
  const ids=routeTargets[build]??beaconsFor(s).map(b=>b.id);
  $('routeProgress').innerHTML=ids.map(id=>{
    const peer=f.beacons?.find(b=>b.id===id),enabled=beaconsFor(s).some(b=>b.id===id),status=routeStatus(f,id);
    const transfer=(peer?.peer_complete??peer?.complete)?'Beacon transfer complete':peer?.peer_state||'No beacon telemetry';
    return `<div class="route-card ${targetId(f)===id?'active':''}" style="--target-colour:${beaconColours[id]}"><span class="dot"></span><strong>AP${id} · ${status}</strong><small>${escape(enabled?transfer:'Radio disabled')}${peer?` · ${peer.rx_bytes??0} / ${peer.tx_bytes??0} bytes`:''}</small></div>`;
  }).join('');
}
function colourText(f){
  const colour=f.sensor??f.colour??f.colour_reading;
  if(colour===undefined)return 'Not recorded';
  if(typeof colour==='string')return colour;
  if(colour===null)return 'Not sampled';
  if(colour.enabled===false)return 'Disabled';
  const rgb=colour.rgb??(['r','g','b'].every(k=>Number.isFinite(colour[k]))?[colour.r,colour.g,colour.b]:null),ratios=colour.ratios;
  const sensorLabel=colour.mat_id?`Mat AP${colour.mat_id}`:colour.mat_id===0?'Off mat':null;
  const parts=[sensorLabel,colour.mat,colour.band!==undefined?`band ${colour.band}`:undefined,colour.c!==undefined?`clear ${colour.c}`:undefined,colour.name,colour.classification].filter(v=>v!==undefined&&v!==null);
  if(rgb)parts.push('RGB '+(Array.isArray(rgb)?rgb.join(' / '):['r','g','b'].map(k=>rgb[k]).join(' / ')));
  if(ratios)parts.push('ratios '+(Array.isArray(ratios)?ratios.map(v=>fmt(v,3)).join(' / '):['r','g','b'].map(k=>fmt(ratios[k],3)).join(' / ')));
  return parts.length?parts.join(' · '):'Recorded';
}
function renderStats(){
  const f=frame||previewFrame(),s=run?.scenario||scenario(),target=activeBeacon(f,s),peer=activeTelemetry(f),telemetry={...f,...peer};
  const distance=targetDistance(f,s),closest=closestApproach(f,s),available=targetAvailable(f,s);
  $('distance').innerHTML=Number.isFinite(distance)?`${fmt(distance)} <small>m</small>`:'Unavailable';
  $('path').innerHTML=`${fmt(f.path)} <small>m</small>`;$('closest').innerHTML=Number.isFinite(closest)?`${fmt(closest)} <small>m</small>`:'Unavailable';$('elapsed').innerHTML=`${fmt(playTime,1)} <small>s</small>`;setText('clock',clock(playTime));$('timeline').value=playTime;
  $('carPosition').innerHTML=`${fmt(f.x)}, ${fmt(f.y)} <small>m</small>`;
  $('beaconPosition').innerHTML=available?`${fmt(target.x)}, ${fmt(target.y)} <small>m</small>`:'Target radio absent / distance unavailable';
  setText('activeBeaconTitle',targetId(f)?`Active target · AP${targetId(f)}`:'No active target');
  const source=run?.metadata.beacons?.find(b=>b.id===targetId(f));
  setText('beaconSourceLabel',`${peer.ssid||source?.ssid||run?.metadata.beacon_ssid||'Selected beacon firmware'} · ${run?run.metadata.beacon_mode==='branch-firmware'?'branch firmware':'recorded peer':'preview'}`);
  setText('worldHeading',`${fmt(norm(f.heading),1)}°`);setText('gyroHeading',`${fmt(f.gyro_heading,1)}°`);setText('wheelSpeeds',`${fmt(f.left)} / ${fmt(f.right)} m/s`);setText('wheelDuty',`${fmt(f.left_pwm,0)} / ${fmt(f.right_pwm,0)} %`);
  const sampled=(value,suffix=' dBm')=>value===undefined||value===-127?'Not sampled':`${fmt(value,Number.isInteger(value)?0:2)}${suffix}`;
  setText('wifiRead',sampled(telemetry.wifi_observed));setText('wifiScans',telemetry.wifi_scans);setText('bleState',f.ble_state);setText('buzzer',f.buzzer?'On · 2 kHz':'Off');setText('faults','0x'+f.faults.toString(16).padStart(4,'0'));$('faults').className=f.faults?'bad':'good';
  setText('motionBadge',Math.abs(f.left)+Math.abs(f.right)<.001?'Stationary':f.left*f.right<0?'Turning':f.left+f.right<0?'Reversing':'Forward');
  setText('peerState',available?telemetry.peer_state:'Target radio absent');setText('wifiModel',available?sampled(telemetry.wifi):'Unavailable');setText('bleModel',available?sampled(telemetry.ble):'Unavailable');setText('bleRead',sampled(telemetry.ble_observed));
  setText('bleAverage',telemetry.ble_filtered===-127?`${telemetry.ble_samples} / 12 samples`:sampled(telemetry.ble_filtered));setText('closeCount',`${telemetry.ble_close_count??0} / 3`);setText('bleCounts',`${telemetry.ble_reports??0} / ${telemetry.ble_attempts??0}`);setText('bytes',`${telemetry.rx_bytes??0} / ${telemetry.tx_bytes??0} bytes`);
  setText('peerCRC',(peer.peer_verified??peer.verified??f.peer_verified)?'Verified':'Pending');
  const verified=Number.isInteger(f.completed_mask)?!!(f.completed_mask&(1<<(targetId(f)-1))):!routeTargets[run?.metadata.robot_build]&&f.payload_verified;
  setText('robotCRC',verified?'Verified + final ACK':'Pending');$('peerCRC').className=(peer.peer_verified??peer.verified??f.peer_verified)?'good':'';$('robotCRC').className=verified?'good':'';
  setText('colourRead',colourText(f));
  const collision=f.box_collision??f.collision;setText('boxCollision',collision===undefined?'Not recorded':collision?'Collision flagged':'Clear');$('boxCollision').className=collision?'bad':'';
  renderRoute(f,s);
  if(run){const atEnd=playTime>=run.result.t-.001;$('arenaState').textContent=atEnd?(run.result.success?'FIRMWARE CONFIRMED':'FIRMWARE STOPPED'):playing?'PLAYING RECORDED EXECUTION':'PAUSED · INSPECT EXECUTION';$('outcome').hidden=!atEnd;}
}
function filteredLogs(){if(!run)return[];return run.logs.filter(l=>l.t<=playTime+.00001).filter(l=>logMode==='all'||(logMode==='faults'?/FAIL|FAULT:|ERROR|STOP LATCH/i.test(l.message):logMode==='ble'?/^beacon(?:[1-3])?$/.test(l.source)||/BLE|GATT|payload|Bluetooth|CRC|HANDSHAKE/i.test(l.message):/TARGET|ROUTE|COLOUR|COLOR|MAT_|CENTRE|CENTER|DIP:|DIP_CHECK|FOLLOW:|FOLLOW_POINT|DIRECTION|RETURN:|SIDE |APPROACH:|RESULT:|FAULT:|FAILED:|COMPLETION|PAYLOAD COMPLETE|START:|calibrat/i.test(l.message)));}
function renderLogs(){if(!run)return;const logs=filteredLogs();if(logs.length===lastLogCount)return;lastLogCount=logs.length;const el=$('events'),nearEnd=el.scrollTop+el.clientHeight>=el.scrollHeight-35;el.innerHTML=logs.slice(-250).map(l=>`<div class="log-row ${/^beacon(?:[1-3])?$/.test(l.source)?'beacon':''} ${/FAIL|FAULT:|ERROR/.test(l.message)?'fault':''}" data-time="${l.t}"><span class="time">${clock(l.t)}</span><span class="source">${escape(l.source)}</span><span class="message">${escape(l.message.trim())}</span></div>`).join('')||'<div class="empty">No matching events at this time. Play or scrub forward.</div>';if(nearEnd)el.scrollTop=el.scrollHeight;}
function render(){drawArena();drawChart();renderStats();renderLogs();}
function seek(t){if(!run)return;playTime=Math.min(run.result.t,Math.max(0,t));let lo=0,hi=run.frames.length-1;while(lo<hi){const m=Math.ceil((lo+hi)/2);if(run.frames[m].t<=playTime)lo=m;else hi=m-1;}frame=run.frames[lo];if(playTime>=run.result.t){playing=false;frame=run.frames.at(-1);$('playBtn').textContent='▶';}render();}
function loadRun(data,autoplay=false){
  run=data;playing=autoplay;playTime=0;frame=run.frames[0];lastLogCount=-1;$('traceNotice').hidden=true;
  writeScenario(run.scenario);
  $('robotBuild').value=run.metadata.robot_build||'ap1';
  $('moving').checked=run.metadata.moving_samples;
  if([...$('branch').options].some(o=>o.value===run.metadata.ref)){$('branch').value=run.metadata.ref;$('customRef').value='';}
  else $('customRef').value=run.metadata.ref;
  branchNote();$('preset').value='custom';$('editStartBtn').hidden=false;
  $('mapHint').textContent='Edit start to arrange the next run · scroll to zoom';
  for(const id of ['playBtn','stepBtn','endBtn','timeline','csvBtn','jsonBtn'])$(id).disabled=false;
  $('timeline').max=run.result.t;$('duration').textContent='Total '+clock(run.result.t);
  $('duration').title='Recorded duration of this execution, not a preset time limit.';
  $('runStatus').textContent=`${autoplay?'New experiment':'Recorded execution'} · Run ${run.id} · ${clock(run.result.t)} total. ${autoplay?'Playing from the start at the selected speed.':'Paused; press Play to view it.'}`;
  $('playBtn').textContent=playing?'Ⅱ':'▶';$('outcome').classList.toggle('failed',!run.result.success);
  setText('outcomeIcon',run.result.success?'✓':'!');
  const isRoute=!!routeTargets[run.metadata.robot_build];
  setText('outcomeTitle',run.result.success?(isRoute?'Route confirmed by the firmware':'Payload confirmed by the firmware'):'The firmware stopped without a confirmed payload');
  setText('outcomeDetail',run.result.success?`${fmt(run.result.t,1)} simulated seconds · ${fmt(run.result.path)} m travelled · recorded firmware completion${isRoute?` · completed mask 0x${(run.result.completed_mask??run.frames.at(-1).completed_mask??0).toString(16)}`:' · payload checks and final acknowledgement completed'}.`:run.result.failure_messages.join(' · ')||run.result.reason);
  setText('provenance',`${run.metadata.ref} · ${run.metadata.commit.slice(0,12)} · ${run.metadata.robot_build?'robot '+run.metadata.robot_build+' + '+beaconsFor(run.scenario).length+' beacon(s) · exact selected firmware':run.metadata.beacon_mode==='branch-firmware'?'car + AP1 branch firmware':'legacy run: car firmware + simulated peer'} · computed in ${run.result.wall_seconds}s`);
  setText('beaconSourceLabel',`${run.metadata.beacon_ssid||'I AM PICO W'} · ${run.metadata.beacon_mode==='branch-firmware'?'branch firmware':'legacy simulated peer'}`);
  fitView();seek(0);
}
async function runExperiment(){
  if(busy)return;
  showError('');for(const k of keys)if(!$(k).reportValidity())return;
  if(!previewReady()){showError('Complete valid starting conditions before building.');return;}
  const source=!$('customRef').value.trim()&&config?.refs.find(ref=>ref.name===$('branch').value);
  if(Array.isArray(source?.supported_builds)&&!source.supported_builds.includes($('robotBuild').value)){showError('Selected source does not support this robot build. Choose a compatible source or build.');return;}
  const request={ref:$('customRef').value.trim()||$('branch').value,build_target:$('robotBuild').value,scenario:scenario(),moving:$('moving').checked};
  resetPreview();setBusy(true);$('runBtn').textContent='Compiling & simulating…';setText('connection','Native engine running…');
  try{
    const data=await api('/api/run',request);
    // A mismatched engine response must never silently replay a substituted controller.
    if(data.metadata.robot_build&&data.metadata.robot_build!==request.build_target)throw Error('Engine returned a different robot build; the requested source/build was not substituted.');
    if(request.build_target!=='ap1'&&!data.metadata.robot_build)throw Error('Engine did not identify the requested robot build. Update the engine; no substitute run will be displayed.');
    loadRun(data,true);setText('connection',`Native engine ready · ${data.metadata.commit.slice(0,7)}`);
    await loadHistory();
  }catch(e){showError(e.message);setText('connection','Build / run failed · see setup panel');}
  finally{setBusy(false);$('runBtn').innerHTML='<span>▶</span> Build & run experiment';}
}
async function loadHistory(){history=await api('/api/runs');const cur=JSON.stringify(run?.scenario||scenario());$('history').innerHTML=history.length?`<table><thead><tr><th>BRANCH / COMMIT</th><th>OUTCOME</th><th>TIME</th><th>PATH</th><th>CLOSEST</th><th>SETUP</th><th></th></tr></thead><tbody>${history.map(r=>`<tr><td>${escape(r.metadata.ref)} · ${escape(r.metadata.robot_build||'ap1')}<br><small>${escape(r.metadata.commit.slice(0,10))}</small></td><td class="${r.result.success?'pass':'fail'}">${r.result.success?'Confirmed':'Stopped'}</td><td>${fmt(r.result.t,1)} s</td><td>${fmt(r.result.path)} m</td><td>${fmt(r.result.closest)} m</td><td>${JSON.stringify(r.scenario)===cur&&(r.metadata.robot_build||'ap1')===$('robotBuild').value&&r.metadata.moving_samples===!!$('moving').checked&&r.metadata.model_version===(run?.metadata.model_version||2)?'Same setup':'Different setup'}</td><td><button data-run="${escape(r.id)}">Inspect →</button></td></tr>`).join('')}</tbody></table>`:'<div class="empty">Saved experiments will appear here.</div>';}
function download(name,text,type='application/json'){const url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
const isObject=value=>value!==null&&typeof value==='object'&&!Array.isArray(value);
const builds=['ap1','ap2','ap3',...Object.keys(routeTargets)];
function validateBeaconArray(value,label){
  if(!Array.isArray(value)||value.length<1||value.length>3)throw Error(`Invalid ${label}: expected one to three entries.`);
  const ids=new Set();
  for(const b of value){
    if(!isObject(b)||!Number.isInteger(b.id)||b.id<1||b.id>3||ids.has(b.id))throw Error(`Invalid ${label}: beacon IDs must be unique integers from 1 to 3.`);
    ids.add(b.id);
    for(const key of ['x','y'])if(typeof b[key]!=='number'||!Number.isFinite(b[key]))throw Error(`Invalid ${label}: ${key} must be finite.`);
    if('mat' in b&&!['pink','blue','purple'].includes(b.mat))throw Error(`Invalid ${label}: unknown mat colour.`);
  }
}
function validateSetup(s){
  if(!isObject(s))throw Error('Invalid scenario file.');
  for(const k of keys)if(k in s&&(typeof s[k]!=='number'||!Number.isFinite(s[k])))throw Error('Invalid starting setup '+k+'.');
  for(const key of ['beacons','mats'])if(key in s)validateBeaconArray(s[key],key);
  for(const key of ['mat_radius','radio_range'])if(key in s&&s[key]<=0)throw Error(`Invalid ${key}: must be positive.`);
  for(const key of ['sensor_offset','box_radius'])if(key in s&&s[key]<0)throw Error(`Invalid ${key}: cannot be negative.`);
  if('colour_enabled' in s&&typeof s.colour_enabled!=='boolean'&&s.colour_enabled!==0&&s.colour_enabled!==1)throw Error('Invalid colour_enabled flag.');
}
function validateOptionalTelemetry(f,label){
  const finite=(value,key)=>{if(typeof value!=='number'||!Number.isFinite(value))throw Error(`Invalid ${label} ${key}: must be finite.`);};
  const flag=(value,key)=>{if(typeof value!=='boolean')throw Error(`Invalid ${label} ${key}: must be boolean.`);};
  if('active_target' in f&&(!Number.isInteger(f.active_target)||f.active_target<0||f.active_target>3))throw Error(`Invalid ${label} active_target.`);
  if('completed_mask' in f&&(!Number.isInteger(f.completed_mask)||f.completed_mask<0||f.completed_mask>7))throw Error(`Invalid ${label} completed_mask.`);
  if('beacons' in f){
    validateBeaconArray(f.beacons,label+' beacons');
    for(const b of f.beacons){
      for(const key of ['wifi','ble','distance','ble_threshold','wifi_observed','ble_observed','ble_filtered','ble_samples','ble_close_count','wifi_scans','wifi_reports','ble_reports','ble_attempts','rx','tx','rx_bytes','tx_bytes','connections','verified_events','complete_events','payload_events'])if(key in b)finite(b[key],'beacon '+key);
      for(const key of ['peer_verified','peer_complete','payload_verified','verified','complete','visible','buzzer'])if(key in b)flag(b[key],'beacon '+key);
      for(const key of ['peer_state','ssid','bssid'])if(key in b&&typeof b[key]!=='string')throw Error(`Invalid ${label} beacon ${key}.`);
    }
  }
  for(const key of ['collision','box_collision'])if(key in f)flag(f[key],key);
  if('sensor' in f){
    if(!isObject(f.sensor))throw Error(`Invalid ${label} sensor.`);
    for(const key of ['x','y','c','r','g','b'])if(key in f.sensor)finite(f.sensor[key],'sensor '+key);
    if('enabled' in f.sensor)flag(f.sensor.enabled,'sensor enabled');
    if('mat_id' in f.sensor&&(!Number.isInteger(f.sensor.mat_id)||f.sensor.mat_id<0||f.sensor.mat_id>3))throw Error(`Invalid ${label} sensor mat_id.`);
    if('band' in f.sensor&&typeof f.sensor.band!=='string')finite(f.sensor.band,'sensor band');
  }
  for(const key of ['colour','colour_reading'])if(key in f&&f[key]!==null){
    const colour=f[key];
    if(typeof colour==='string')continue;
    if(!isObject(colour))throw Error(`Invalid ${label} ${key}.`);
    for(const field of ['rgb','ratios'])if(field in colour){
      const channels=Array.isArray(colour[field])?colour[field]:isObject(colour[field])?['r','g','b'].map(k=>colour[field][k]):[];
      if(channels.length!==3)throw Error(`Invalid ${label} ${field}.`);
      channels.forEach(value=>finite(value,field));
    }
    if('sensor' in colour){if(!isObject(colour.sensor))throw Error(`Invalid ${label} colour sensor.`);for(const field of ['x','y'])finite(colour.sensor[field],'colour sensor '+field);}
  }
}
function validateTrace(data){
  const object=v=>v!==null&&typeof v==='object'&&!Array.isArray(v);
  const fail=message=>{throw Error(message);};
  const finite=(v,label)=>{if(typeof v!=='number'||!Number.isFinite(v))fail(`Invalid full trace: ${label} must be a number.`);};
  if(!object(data)||!object(data.metadata)||!object(data.scenario)||!object(data.result)||!Array.isArray(data.frames)||!data.frames.length||!Array.isArray(data.logs))
    fail('Choose a Full trace JSON downloaded from an experiment. A setup file or CSV does not contain the recording.');
  if(data.frames.length>30000)fail('This trace contains too many frames to replay (maximum 30,000).');
  for(const key of ['id'])if(typeof data[key]!=='string'||!data[key])fail(`Invalid full trace: missing ${key}.`);
  for(const key of ['ref','commit'])if(typeof data.metadata[key]!=='string'||!data.metadata[key])fail(`Invalid full trace: missing source ${key}.`);
  for(const key of legacyKeys)finite(data.scenario[key],`starting setup ${key}`);
  validateSetup(data.scenario);
  if('robot_build' in data.metadata&&!builds.includes(data.metadata.robot_build))fail('Invalid full trace: unknown robot build.');
  validateOptionalTelemetry(data.result,'result');
  if(data.scenario.speed<=0||data.scenario.wheelbase<=0||data.scenario.exponent<=0||data.scenario.ble_radius<=0)fail('Invalid full trace: environment values must be positive.');
  for(const key of ['t','distance','closest','path','wall_seconds']){finite(data.result[key],`result ${key}`);if(data.result[key]<0)fail(`Invalid full trace: result ${key} cannot be negative.`);}
  if(typeof data.result.success!=='boolean'||typeof data.result.reason!=='string'||!Array.isArray(data.result.failure_messages)||data.result.failure_messages.some(m=>typeof m!=='string'))fail('Invalid full trace: missing recorded outcome or failure messages.');
  const fields=['t','x','y','heading','gyro_heading','distance','closest','path','wifi','ble','wifi_observed','ble_observed','left','right','left_pwm','right_pwm','wifi_scans','ble_reports','ble_attempts','faults','ble_samples','ble_close_count','ble_filtered','rx_bytes','tx_bytes'];
  let previous=-1;
  for(const [i,f] of data.frames.entries()){
    if(!object(f))fail(`Invalid full trace: frame ${i+1} is missing.`);
    for(const key of fields)finite(f[key],`frame ${i+1} ${key}`);
    if(f.t<0||f.t<previous||f.t>data.result.t+.001)fail('Invalid full trace: frame timestamps must be in order and within the recording.');
    previous=f.t;
    if(f.distance<0||f.closest<0||f.path<0||!Number.isInteger(f.faults)||f.faults<0||f.faults>0xffffffff)fail(`Invalid full trace: invalid measurements in frame ${i+1}.`);
    for(const key of ['ble_state','peer_state'])if(typeof f[key]!=='string')fail(`Invalid full trace: frame ${i+1} is missing ${key}.`);
    for(const key of ['buzzer','peer_verified','peer_complete','payload_verified'])if(key in f&&typeof f[key]!=='boolean')fail(`Invalid full trace: frame ${i+1} has invalid ${key}.`);
    validateOptionalTelemetry(f,`frame ${i+1}`);
    if('ble_threshold' in f)finite(f.ble_threshold,`frame ${i+1} BLE threshold`);
  }
  if(Math.abs(previous-data.result.t)>.001)fail('Invalid full trace: the recording is missing its final frame.');
  previous=-1;
  for(const [i,l] of data.logs.entries()){
    if(!object(l)||typeof l.source!=='string'||typeof l.message!=='string')fail(`Invalid full trace: log entry ${i+1} is incomplete.`);
    finite(l.t,`log entry ${i+1} timestamp`);
    if(l.t<0||l.t<previous||l.t>data.result.t+.001)fail('Invalid full trace: log timestamps must be in order and within the recording.');
    previous=l.t;
  }
  return data;
}
function openTrace(data,name){
  validateTrace(data); // Validate before replacing the current recording or setup.
  loadRun(data);
  $('traceNotice').textContent=`Recorded run loaded: ${name}. Press Play to replay its movement, readings and logs. The saved outcome is preserved; the firmware is not run again.`;
  $('traceNotice').hidden=false;$('events').scrollTop=0;
  for(const button of $('evidenceTabs').children)button.classList.toggle('active',button.dataset.view==='events');
  $('eventsView').hidden=false;$('historyView').hidden=true;$('logFilter').hidden=false;showError('');
}
async function importTraceFile(file){
  if(!file||busy)return;
  setBusy(true);
  try{
    if(file.size>50*1024*1024)throw Error('Choose a Full trace JSON smaller than 50 MB.');
    const data=JSON.parse(await file.text());openTrace(data,file.name);
  }catch(e){showError(e instanceof SyntaxError?'This file is not valid JSON. Choose the downloaded Full trace JSON.':e.message);}
  finally{setBusy(false);$('traceFile').value='';}
}
$('loadTraceBtn').onclick=()=>{if(!busy)$('traceFile').click();};
$('traceFile').onchange=()=>importTraceFile($('traceFile').files[0]);
$('runBtn').onclick=runExperiment;$('branch').onchange=()=>{if(busy)return;$('customRef').value='';branchNote();resetPreview();};$('customRef').oninput=()=>{if(busy)return;branchNote();resetPreview();};$('fetchBtn').onclick=async()=>{if(busy)return;try{$('fetchBtn').disabled=true;const d=await api('/api/fetch',{});if(!config)config={refs:[]};config.refs=d.refs;populateRefs(d.refs);}catch(e){showError(e.message);}finally{$('fetchBtn').disabled=false;}};
$('preset').onchange=()=>{if(!busy)applyPreset($('preset').value);};
for(const k of keys){const changed=()=>{if(!busy){$('preset').value='custom';resetPreview();}};$(k).addEventListener('input',changed);$(k).addEventListener('change',changed);}
for(const id of ['moving','colour_enabled','beaconCount','singleBeacon','matLayout','mat1','mat2','mat3'])$(id).onchange=()=>{if(!busy){$('preset').value='custom';resetPreview(true);}};
$('robotBuild').onchange=()=>{
  if(busy)return;
  const targets=routeTargets[$('robotBuild').value];
  if(targets){
    $('beaconCount').value=String(targets.length);
    if(targets.length===2)twoBeaconIds=targets;
    $('preset').value='custom';
  }
  branchNote();resetPreview(true);
};
$('playBtn').onclick=()=>{if(!run)return;if(playTime>=run.result.t)seek(0);playing=!playing;$('playBtn').textContent=playing?'Ⅱ':'▶';renderStats();};$('stepBtn').onclick=()=>{playing=false;$('playBtn').textContent='▶';seek(playTime+.1);};$('endBtn').onclick=()=>seek(run.result.t);$('replayBtn').onclick=()=>{seek(0);playing=true;$('playBtn').textContent='Ⅱ';};$('timeline').oninput=()=>{playing=false;$('playBtn').textContent='▶';seek(Number($('timeline').value));};$('fitBtn').onclick=()=>{fitView();drawArena();};
$('chartTabs').onclick=e=>{const b=e.target.closest('[data-chart]');if(!b)return;chartMode=b.dataset.chart;for(const el of $('chartTabs').children)el.classList.toggle('active',el===b);setText('chartLegend',chartMode==='distance'?'Ground-truth distance':'Ideal signal');setText('chartOther',chartMode==='distance'?'Not available to the controller':'● Actual readings delivered to firmware');drawChart();};
$('evidenceTabs').onclick=e=>{const b=e.target.closest('[data-view]');if(!b)return;for(const el of $('evidenceTabs').children)el.classList.toggle('active',el===b);$('eventsView').hidden=b.dataset.view!=='events';$('historyView').hidden=b.dataset.view!=='history';$('logFilter').hidden=b.dataset.view!=='events';if(b.dataset.view==='history')loadHistory().catch(e=>showError(e.message));};$('logFilter').onchange=()=>{logMode=$('logFilter').value;lastLogCount=-1;renderLogs();};$('events').onclick=e=>{const row=e.target.closest('[data-time]');if(row){playing=false;$('playBtn').textContent='▶';seek(Number(row.dataset.time));}};$('history').onclick=async e=>{const b=e.target.closest('[data-run]');if(!b)return;try{if(busy)return;const data=await api('/api/run/'+b.dataset.run);if(busy)return;validateTrace(data);loadRun(data);}catch(e){showError(e.message);}};
$('jsonBtn').onclick=()=>run&&download(run.id+'.json',JSON.stringify(run,null,2));
$('csvBtn').onclick=()=>{
  if(!run)return;const fields=Object.keys(run.frames[0]).filter(k=>k!=='type');
  const quote=value=>`"${String(typeof value==='object'?JSON.stringify(value):value).replace(/"/g,'""')}"`;
  const text=[fields.map(quote).join(','),...run.frames.map(f=>fields.map(k=>quote(f[k])).join(','))].join('\n');
  download(run.id+'.csv',text,'text/csv');
};
$('saveScenario').onclick=()=>{if(!busy)download('beacon-lab-scenario.json',JSON.stringify({scenario:scenario(),ref:$('customRef').value.trim()||$('branch').value,build_target:$('robotBuild').value,moving:$('moving').checked},null,2));};
$('importFile').onchange=async()=>{
  if(busy)return;
  const file=$('importFile').files[0];if(!file)return;
  setBusy(true);
  try{
    if(file.size>50*1024*1024)throw Error('Choose a setup smaller than 50 MB.');
    const data=JSON.parse(await file.text()),s=data.scenario||data;validateSetup(s);
    if('frames' in data)throw Error('Use Load trace to replay a full recording.');
    if(data.build_target!==undefined&&!builds.includes(data.build_target))throw Error('Invalid robot build.');
    if(data.ref!==undefined&&typeof data.ref!=='string')throw Error('Invalid source reference.');
    applyScenario(s);$('robotBuild').value=data.build_target||'ap1';
    if(data.ref)$('customRef').value=data.ref;
    $('moving').checked=!!data.moving;$('preset').value='custom';branchNote();resetPreview(true);showError('');
  }catch(e){showError(e.message);}
  finally{setBusy(false);$('importFile').value='';}
};
for(const id of ['aboutBtn','aboutFooter'])$(id).onclick=()=>$('about').showModal();$('closeAbout').onclick=()=>$('about').close();$('about').onclick=e=>{if(e.target===$('about')){const r=$('about').getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)$('about').close();}};
$('editStartBtn').onclick=()=>{if(!busy&&run){applyScenario(run.scenario);$('preset').value='custom';}};
const arena=$('arena');
function arenaTarget(x,y,r){
  const s=run?.scenario||scenario(),f=frame||previewFrame(),p=worldToScreen(f.x,f.y,r.width,r.height),handle=headingHandle(f,r.width,r.height);
  if(!run&&Math.hypot(x-handle.x,y-handle.y)<16)return 'heading';
  if(Math.hypot(x-p.x,y-p.y)<28)return 'car';
  for(const [kind,items] of [['beacon',beaconsFor(s)],['mat',s.mats||[]]])for(const b of items){const point=worldToScreen(b.x,b.y,r.width,r.height);if(Math.hypot(x-point.x,y-point.y)<25)return kind+b.id;}
  return 'pan';
}
arena.onpointerdown=e=>{
  if(busy||e.button!==0||drag)return;
  const r=arena.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top,f=frame||previewFrame(),p=worldToScreen(f.x,f.y,r.width,r.height),kind=arenaTarget(x,y,r);
  if(kind!=='pan'&&run)applyScenario({...run.scenario,x:f.x,y:f.y,heading:norm(f.heading)});
  $('preset').value='custom';drag={kind,pointerId:e.pointerId,x:e.clientX,y:e.clientY,vx:view.x,vy:view.y,headingOffset:f.heading-Math.atan2(p.y-y,x-p.x)*180/Math.PI};
  arena.style.cursor='grabbing';arena.setPointerCapture(e.pointerId);if(kind==='heading')drawArena();
};
arena.onpointermove=e=>{
  if(busy)return;
  const r=arena.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;
  if(!drag){arena.style.cursor=arenaTarget(x,y,r)==='pan'?'grab':'pointer';if(arenaTarget(x,y,r)==='heading')arena.style.cursor='grab';return;}
  if(e.pointerId!==drag.pointerId)return;
  if(drag.kind==='pan'){view.x=drag.vx-(e.clientX-drag.x)/(r.height/view.span);view.y=drag.vy+(e.clientY-drag.y)/(r.height/view.span);drawArena();return;}
  if(drag.kind==='heading'){
    const s=scenario(),center=worldToScreen(s.x,s.y,r.width,r.height),dx=x-center.x,dy=center.y-y;
    if(Math.hypot(dx,dy)<6)return;
    $('heading').value=fmt(norm(Number(fmt(Math.atan2(dy,dx)*180/Math.PI+drag.headingOffset))));resetPreview();return;
  }
  const p=screenToWorld(x,y,r.width,r.height),prefix=drag.kind.startsWith('beacon')?beaconPrefix(Number(drag.kind.slice(6))):drag.kind.startsWith('mat')?'m'+drag.kind.slice(3):'';$(prefix+'x').value=fmt(Math.max(-50,Math.min(50,p.x)));$(prefix+'y').value=fmt(Math.max(-50,Math.min(50,p.y)));resetPreview();
};
function endArenaDrag(e){if(drag&&e.pointerId===drag.pointerId){drag=null;arena.style.cursor='grab';drawArena();}}
arena.onpointerup=endArenaDrag;arena.onpointercancel=endArenaDrag;arena.onlostpointercapture=endArenaDrag;
arena.addEventListener('wheel',e=>{e.preventDefault();view.span=Math.max(1,Math.min(120,view.span*Math.exp(e.deltaY*.001)));drawArena();},{passive:false});
new ResizeObserver(()=>{drawArena();drawChart();}).observe($('arena').parentElement);new ResizeObserver(drawChart).observe($('chart'));
let last=performance.now(),lastPaint=0,accumulator=0;function loop(now){const dt=Math.min(.15,(now-last)/1000);last=now;if(playing&&run){accumulator+=dt;if(now-lastPaint>32){seek(playTime+accumulator*Number($('playSpeed').value));accumulator=0;lastPaint=now;}}else accumulator=0;requestAnimationFrame(loop);}requestAnimationFrame(loop);
async function init(){resetPreview(true);try{config=await api('/api/config');populateRefs(config.refs);setText('connection','Native engine ready · local only');await loadHistory();}catch(e){showError(e.message);setText('connection','Local engine unavailable');}}init();
