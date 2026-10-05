/** Browser application entry point. All data is read via the loopback Node API. */
import {sampleIndex,degrees,dialLead,jointDialSpeed,clock} from './math.mjs';
import {createRobot} from './robot.mjs';

const $ = selector => document.querySelector(selector); // Short DOM lookup helper used throughout this module.
const colors = {a:'#53ddd0',b:'#a697ff',robot:'#f6bf6b',grid:'#28394d',text:'#90a4bd'}; // Consistent data-series palette.
const bones = [[0,1],[0,2],[1,3],[2,4],[5,6],[5,7],[7,9],[6,8],[8,10],[5,11],[6,12],[11,12],[11,13],[13,15],[12,14],[14,16]]; // COCO-17 topology.
const state = {rows:[],sort:'id',direction:-1,page:0,game:null,time:0,playing:false,last:0,selected:{a:0,b:0},robots:{},model:null,loading:0,anchorDrag:null,charts:{},dirty:true}; // Mutable UI/playback state, never sent to controller hardware.
const PAGE_SIZE = 40; // Browser pagination size; tune for table density.
let anchors = restoreAnchors(); // Persistent camera controller guide positions, normalized to full image.

/** HTML-escape values originating in CSV or recordings before templating the library. */
function escape(value) { return String(value??'').replace(/[&<>"']/g,character=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[character])); }
/** Format optional measurements without converting missing values to zero. */
function number(value,digits=1) { return Number.isFinite(value) ? (Math.abs(value)<.5*10**-digits?0:value).toFixed(digits) : '—'; }
/** Fetch an API response and report useful conversion/server errors to the caller. */
async function fetchJSON(url) { const response=await fetch(url); if(!response.ok) throw new Error(await response.text()); return response.json(); }
/** Default controller guides are uncalibrated, left-to-right placeholders for user adjustment. */
function defaultAnchors() { return Object.fromEntries(['a','b'].map(team=>[team,Array.from({length:6},(_,index)=>[(index+.5)/6,.86])])); }
/** Restore camera guides defensively; browser storage is optional and may be disabled. */
function restoreAnchors() { try { const saved=JSON.parse(localStorage.getItem('record-player-guides-v1')); if(['a','b'].every(team=>saved?.[team]?.length===6 && saved[team].every(point=>point.length===2&&point.every(v=>Number.isFinite(v)&&v>=0&&v<=1)))) return saved; } catch {} return defaultAnchors(); }
/** Persist adjusted camera guides locally; called after dragging or suggesting positions. */
function saveAnchors() { try { localStorage.setItem('record-player-guides-v1',JSON.stringify(anchors)); } catch {} state.dirty=true; }

/** Load/refetch the CSV index and update library summary cards. */
async function loadIndex() {
  $('#rowCount').textContent='Reading index…';
  try {
    state.rows=await fetchJSON('/api/index');
    const local=state.rows.filter(row=>row.local); // Locally playable recordings.
    const dates=new Set(local.map(row=>row.date)); // Available day count.
    $('#summary').innerHTML=[['INDEXED GAMES',state.rows.length.toLocaleString(),'The complete recording ledger'],['READY TO PLAY',local.length,`${dates.size} day${dates.size===1?'':'s'} stored on this computer`],['LOCAL PLAY TIME',`${(local.reduce((total,row)=>total+(row.play_duration||0),0)/3600).toFixed(1)} h`,'Tutorial time excluded'],['SHORT LOCAL GAMES',local.filter(row=>row.play_duration!==null&&row.play_duration<60).length,'Under 60 seconds of Play']].map(([label,value,caption])=>`<article><p class="eyebrow">${label}</p><strong>${value}</strong><span>${caption}</span></article>`).join('');
    renderLibrary();
  } catch(error) { $('#rowCount').textContent=`Cannot read recordings: ${error.message}`; }
}

/** Filter, sort, and paginate the library while preserving offline records for inspection. */
function renderLibrary() {
  const query=$('#search').value.toLowerCase(); // Free-text library filter.
  const filtered=state.rows.filter(row=>(!$('#localOnly').checked||row.local)&&(!$('#shortOnly').checked||(row.play_duration!==null&&row.play_duration<60))&&`${row.id} ${row.profile} ${row.flags.join(' ')}`.toLowerCase().includes(query)); // Visible records.
  filtered.sort((left,right)=>{
    const a=left[state.sort],b=right[state.sort]; // Selected sort values; missing metrics always last.
    if(a==null) return b==null?0:1; if(b==null) return -1;
    return (typeof a==='number'?a-b:String(a).localeCompare(String(b)))*state.direction;
  });
  const pages=Math.max(1,Math.ceil(filtered.length/PAGE_SIZE)); // Number of filtered pages.
  state.page=Math.min(state.page,pages-1);
  $('#games tbody').innerHTML=filtered.slice(state.page*PAGE_SIZE,(state.page+1)*PAGE_SIZE).map(row=>`<tr><td><b>${escape(row.date)}</b> <span>${escape(row.time.replaceAll('-',':'))}</span><span class="secondary">${escape(row.profile)}</span></td><td>${number(row.play_duration,0)} / ${number(row.duration,0)} s</td><td class="team-a">${number(row.score_a,0)}</td><td class="team-b">${number(row.score_b,0)}</td><td>${number(row.travel_a)}</td><td>${number(row.travel_b)}</td><td><span class="team-a">${number(row.lead_a)}</span> / <span class="team-b">${number(row.lead_b)}</span></td><td><span class="team-a">${number(row.limited_a)}</span> / <span class="team-b">${number(row.limited_b)}</span></td><td><span class="tag ${row.local?'available':''}">${row.local?`${row.skeletons}/2 cams · ${row.microphones}/12 mics`:'Index only'}</span>${row.flags.map(flag=>`<span class="tag flag">${escape(flag)}</span>`).join('')}</td><td><button data-game="${escape(row.id)}" ${row.local?'':'disabled'}>${row.local?'Explore ↗':'Offline'}</button></td></tr>`).join('') || '<tr><td colspan="10" class="empty">No recordings match these filters.</td></tr>';
  $('#rowCount').textContent=`${filtered.length.toLocaleString()} recordings`;
  $('#pageInfo').textContent=`${state.page+1} / ${pages}`;
  $('#previous').disabled=state.page===0; $('#next').disabled=state.page===pages-1;
  for(const heading of document.querySelectorAll('th[data-sort]')) { heading.textContent=heading.textContent.replace(/ [↑↓]$/,'')+(heading.dataset.sort===state.sort?(state.direction>0?' ↑':' ↓'):''); heading.setAttribute('aria-sort',heading.dataset.sort===state.sort?(state.direction>0?'ascending':'descending'):'none'); }
}

/** Build stable per-team panels once so canvas/WebGL contexts survive switching recordings. */
function buildPanels() {
  for(const team of ['a','b']) {
    $('#skeletonRow').insertAdjacentHTML('beforeend',`<article class="panel ${team}-panel"><div class="panel-title"><strong>${team.toUpperCase()} · SKELETON & SOUND</strong><span>CAMERA VIEW · P1 ON LEFT</span></div><canvas class="skeleton" id="skeleton-${team}" aria-label="Team ${team} skeletons"></canvas><div class="audio-strip">${Array.from({length:6},(_,j)=>`<div class="audio-meter">P${j+1}<b id="loud-${team}-${j}">—</b><div class="meter-track"><div class="meter-fill" id="meter-${team}-${j}"></div></div></div>`).join('')}</div></article>`);
    $('#dialRow').insertAdjacentHTML('beforeend',`<article class="panel ${team}-panel"><div class="panel-title"><strong>${team.toUpperCase()} · PLAYER INPUT</strong><span>CLICK A PLAYER TO INSPECT</span></div><div class="dial-grid">${Array.from({length:6},(_,j)=>`<button class="dial-card ${j===0?'active':''}" data-team="${team}" data-joint="${j}"><span class="player-label">P${j+1} / J${j+1}</span><b class="lead" id="lead-${team}-${j}">—</b><small>LEAD · JOINT °</small><span class="lead-bar"><i id="leadbar-${team}-${j}"></i></span><span class="speed team-${team}" id="dial-${team}-${j}">D — °/s</span><span class="speed robot-speed" id="velocity-${team}-${j}">R — °/s</span></button>`).join('')}</div><div class="dial-legend"><span class="team-${team}">D dial speed → joint units</span> · <span style="color:var(--amber)">R actual robot speed</span><br>Lead scale ±30° (numbers remain unclipped). Signed speeds include force feedback.</div></article>`);
    $('#robotRow').insertAdjacentHTML('beforeend',`<article class="panel"><div class="panel-title"><strong>${team.toUpperCase()} · ROBOT & CELL</strong><span>DRAG ORBIT · WHEEL ZOOM · RIGHT-DRAG PAN</span></div><div class="robot-view" id="robot-${team}"><div class="robot-message" id="robot-message-${team}">Loading URDF-derived scene…</div></div><div class="control-status" id="control-${team}"></div></article>`);
    $('#chartRow').insertAdjacentHTML('beforeend',`<article class="panel"><div class="panel-title"><strong id="chart-title-${team}">${team.toUpperCase()} · PLAYER 1 TRACE</strong><span>CLICK PLOT TO SEEK</span></div><canvas class="trace" id="trace-${team}" aria-label="Joint lead and speed time series"></canvas><canvas class="weights" id="weights-${team}" aria-label="Bucket weights time series"></canvas><div class="control-status" id="extra-${team}"></div></article>`);
    $(`#skeleton-${team}`).addEventListener('pointerdown',event=>beginGuideDrag(event,team));
    $(`#skeleton-${team}`).addEventListener('pointermove',event=>moveGuide(event,team));
    $(`#skeleton-${team}`).addEventListener('pointerup',()=>{state.anchorDrag=null;saveAnchors();});
    $(`#skeleton-${team}`).addEventListener('pointercancel',()=>{state.anchorDrag=null;});
    for(const prefix of ['trace','weights']) $(`#${prefix}-${team}`).addEventListener('click',event=>{ if(!state.game)return; const bounds=event.currentTarget.getBoundingClientRect(),domain=traceDomain();seek(domain[0]+(event.clientX-bounds.left-48)/(bounds.width-60)*(domain[1]-domain[0])); });
  }
}

/** Load one recording, preserving independent source time bases and guarding against request races. */
async function openGame(id) {
  const request=++state.loading; // Generation token prevents a slow previous selection overwriting the current game.
  state.playing=false; state.game=null; state.time=0; state.dirty=true;
  $('#player').dataset.loading='true';
  $('#library').hidden=true; $('#player').hidden=false; $('#play').disabled=true;
  $('#gameTitle').textContent=id.replace('/',' · ').replace(/(\d{2})-(\d{2})-(\d{2})$/,'$1:$2:$3');
  $('#gameSubtitle').textContent='Reading local Parquet streams…'; $('#loadStatus').textContent='Preparing synchronized preview. The first load may take a few seconds.';
  $('#gameScores').textContent='';
  history.replaceState(null,'',`#game=${encodeURIComponent(id)}`);
  try {
    const [game,model]=await Promise.all([fetchJSON(`/api/game?id=${encodeURIComponent(id)}`),state.model?Promise.resolve(state.model):fetchJSON('/api/scene')]); // Independent data and robot asset loads.
    if(request!==state.loading)return;
    state.game=game; state.model=model; state.time=Math.min(game.duration,game.play_start??0); state.charts={};
    delete $('#player').dataset.loading;
    $('#gameSubtitle').textContent=`${game.metadata.profile_name} · ${clock(game.duration)} recorded · 20 Hz telemetry preview · source camera frames`;
    $('#gameScores').innerHTML=`<div class="team-a"><span>FINAL SCORE A</span><strong>${escape(game.metadata.score_a)}</strong></div><div class="team-b"><span>FINAL SCORE B</span><strong>${escape(game.metadata.score_b)}</strong></div>`;
    $('#loadStatus').textContent=`Audio timing is provisional. Controller guides and hand associations are uncalibrated.${game.warnings.length?` ${game.warnings.length} stream warning(s); see Data quality.`:''}`;
    $('#scrubber').max=game.duration; $('#play').disabled=false;
    $('#playRegion').style.left=`${100*(game.play_start??game.duration)/game.duration}%`;
    $('#playMarker').textContent=game.play_start===null?'NO PLAY STAGE':`PLAY START ${clock(game.play_start)}`;
    $('#jumpPlay').disabled=game.play_start===null;
    for(const team of ['a','b']) {
      if(!state.robots[team]) {
        try { state.robots[team]=createRobot($(`#robot-${team}`),model,team); }
        catch(error) { $(`#robot-message-${team}`).textContent=`3D unavailable: ${error.message}`; }
      }
    }
    if(!$('#geometryOptions').children.length) {
      $('#geometryOptions').innerHTML=model.bodies.map(body=>`<label><input type="checkbox" data-body="${escape(body.name)}" ${body.hidden?'':'checked'}>${escape(body.name.replaceAll('_',' '))}</label>`).join('');
      $('#geometryOptions').addEventListener('change',event=>{ for(const robot of Object.values(state.robots))robot.visibility(event.target.dataset.body,event.target.checked); });
    }
    $('#quality').innerHTML=`<p>${escape(game.audio_note)}</p><p>Skeletons use camera epoch timestamps; their clock offset versus the controller has not been independently verified. Empty camera frames mean no detections. Samples older than 250 ms (camera 350 ms, weights 1 s) are shown as missing, not held indefinitely.</p><p>Lead = recorded dial_robot_deg − actual joint angle, without angle wrapping. Dial velocity uses the signed gearing recovered from that recording. No current profile assumptions are applied. A high lead can reflect obstruction, tracking/telemetry problems, initialization, or behavior; it is not a skill label.</p><p>Lead P95 pools absolute lead across six joints on a 20 Hz Play grid. Limited % measures time with clamp_final &lt; 0.99 on valid Play samples. Preview decimation can miss very brief events. Source files remain unmodified.</p>${['a','b'].map(team=>`<p><b>Team ${team.toUpperCase()}</b> · paired dial/robot coverage ${number(game.metrics[`coverage_${team}`])}% · lead P95 ${number(game.metrics[`lead_${team}`])}° · limited ${number(game.metrics[`limited_${team}`])}% · gear ratios ${game.teams[team].gear_ratio.map(value=>number(value,3)).join(', ')}<br>${['haptic','robot_actual','game_controller','weight'].map(name=>`${name}: ${game.teams[team][name].info.rows} source rows, max gap ${number(game.teams[team][name].info.max_gap_s,2)}s`).join(' · ')}</p>`).join('')}${game.warnings.map(warning=>`<p class="limited">${escape(warning)}</p>`).join('')}`;
    const row=state.rows.find(item=>item.id===id); // Update browser metrics immediately after loading.
    if(row)Object.assign(row,game.metrics);
    state.dirty=true; window.scrollTo({top:0});
  } catch(error) { if(request===state.loading){$('#loadStatus').textContent=`Could not load recording: ${error.message}`;$('#gameSubtitle').textContent='Return to the library and select another game, or check setup.';} }
}

/** Read a field at the current timestamp with explicit freshness limits. */
function valueAt(stream,field,time,maxAge=.25) { const index=sampleIndex(stream.t,time,maxAge); return index<0?null:stream[field]?.[index]??null; }
/** Clamp and update playhead position; used by slider, keyboard, charts, and frame controls. */
function seek(time) { if(!state.game)return; state.time=Math.max(0,Math.min(state.game.duration,time));state.dirty=true; }
/** Resize a 2D canvas to its CSS display size and return a pixel-scaled drawing context. */
function canvasContext(canvas) {
  const width=canvas.clientWidth,height=canvas.clientHeight,ratio=Math.min(devicePixelRatio,2); // Logical size and capped backing-store resolution.
  if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);}
  const context=canvas.getContext('2d'); // 2D canvas context; coordinates below use CSS pixels.
  context.setTransform(ratio,0,0,ratio,0,0);context.clearRect(0,0,width,height);
  return {context,width,height};
}

/** Compute a letterboxed camera transform shared by skeleton drawing and controller dragging. */
function cameraTransform(team) {
  const canvas=$(`#skeleton-${team}`),source=state.game.teams[team].skeleton; // Current camera panel and source dimensions.
  const top=$('#crop').checked?source.height*.45:0; // Optional crop of upper empty camera area.
  const scale=Math.min(canvas.clientWidth/source.width,canvas.clientHeight/(source.height-top)); // Preserve camera aspect ratio.
  return {scale,left:(canvas.clientWidth-source.width*scale)/2,top:(canvas.clientHeight-(source.height-top)*scale)/2-top*scale,width:source.width,height:source.height};
}

/** Associate confident wrist pairs with nearest provisional controller, one person per slot. */
function handMatches(people,team,threshold) {
  const candidates=[]; // Candidate distances between paired hands and guide positions.
  const source=state.game.teams[team].skeleton; // Camera normalization dimensions.
  people.forEach((person,index)=>{
    if(person[3][9]<threshold||person[3][10]<threshold)return;
    const x=(person[1][9]+person[1][10])/2/source.width,y=(person[2][9]+person[2][10])/2/source.height; // Hand-pair midpoint.
    anchors[team].forEach((point,slot)=>candidates.push({index,slot,distance:Math.hypot(x-point[0],y-point[1])}));
  });
  candidates.sort((a,b)=>a.distance-b.distance);
  const matches=new Map(),used=new Set(); // Greedy one-to-one matching, explicitly not identity tracking.
  for(const candidate of candidates)if(candidate.distance<.18&&!matches.has(candidate.index)&&!used.has(candidate.slot)){matches.set(candidate.index,candidate.slot);used.add(candidate.slot);}
  return matches;
}

/** Draw confidence-filtered poses, inferred hand sound halos, and editable controller guides. */
function drawSkeleton(team,time) {
  const {context:ctx,width,height}=canvasContext($(`#skeleton-${team}`)); // Fresh drawing surface.
  const source=state.game.teams[team].skeleton,transform=cameraTransform(team); // Camera trace and image-to-canvas transform.
  const index=sampleIndex(source.t,time,.35),people=index<0?[]:source.people[index]; // Preserve true empty frames and stale-camera states.
  const confidence=Number($('#confidence').value); // Adjustable keypoint confidence threshold.
  const matches=handMatches(people,team,confidence); // Provisional microphone/pose mapping.
  const point=(x,y)=>[transform.left+x*transform.scale,transform.top+y*transform.scale]; // Camera pixel to display point.
  ctx.strokeStyle='#1b2b3f';ctx.lineWidth=1;
  for(let x=0;x<width;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,height);ctx.stroke();}
  for(let y=0;y<height;y+=40){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(width,y);ctx.stroke();}
  people.forEach((person,personIndex)=>{
    const slot=matches.get(personIndex); // Optional inferred player index.
    if(slot!==undefined&&$('#halos').checked){
      const loud=valueAt(state.game.teams[team].audio[slot],'loudness',time-Number($('#audioOffset').value)); // Relative audio shifted by user offset.
      if(Number.isFinite(loud)) for(const hand of [9,10]){
        const [x,y]=point(person[1][hand],person[2][hand]); // Sound visual follows tracked hand.
        const radius=8+Math.min(55,Math.sqrt(Math.max(0,loud))*38*Number($('#gain').value)); // Shared intensity scale across microphones.
        const gradient=ctx.createRadialGradient(x,y,0,x,y,radius); // Soft emitting circle.
        gradient.addColorStop(0,team==='a'?'#53ddd080':'#a697ff80');gradient.addColorStop(1,team==='a'?'#53ddd000':'#a697ff00');ctx.fillStyle=gradient;ctx.beginPath();ctx.arc(x,y,radius,0,Math.PI*2);ctx.fill();
      }
    }
    ctx.strokeStyle=slot===undefined?'#7e93ac':colors[team];ctx.lineWidth=2;
    for(const [a,b] of bones){if(person[3][a]<confidence||person[3][b]<confidence)continue;ctx.beginPath();ctx.moveTo(...point(person[1][a],person[2][a]));ctx.lineTo(...point(person[1][b],person[2][b]));ctx.stroke();}
    for(let joint=0;joint<17;joint++){if(person[3][joint]<confidence)continue;ctx.fillStyle=joint===9||joint===10?'#f0e3c9':ctx.strokeStyle;ctx.beginPath();ctx.arc(...point(person[1][joint],person[2][joint]),joint===9||joint===10?3.5:2.5,0,Math.PI*2);ctx.fill();}
    const labelJoint=person[3][0]>=confidence?0:person[3][5]>=confidence?5:-1; // Label only when a reliable landmark exists.
    if(labelJoint>=0){const [x,y]=point(person[1][labelJoint],person[2][labelJoint]);ctx.font='10px Segoe UI';ctx.fillStyle=colors[team];ctx.fillText(slot===undefined?`track ${person[0]}`:`P${slot+1}?`,x+6,y-8);}
  });
  if($('#anchors').checked)anchors[team].forEach((anchor,slot)=>{
    const [x,y]=point(anchor[0]*source.width,anchor[1]*source.height); // Provisional guide center.
    ctx.strokeStyle='#edc680';ctx.setLineDash([4,3]);ctx.strokeRect(x-14,y-8,28,16);ctx.setLineDash([]);ctx.font='10px Segoe UI';ctx.fillStyle='#edc680';ctx.fillText(`P${slot+1}`,x-8,y-14);
  });
  ctx.font='10px Segoe UI';ctx.fillStyle='#8d9eb4';ctx.fillText('P1 ← camera left    •    P6 camera right →',12,18);
  ctx.fillText(index<0?'No fresh camera frame':`${people.length} detections · ${matches.size} provisional hand matches`,12,height-12);
  if(!people.length){ctx.font='13px Segoe UI';ctx.fillStyle='#8d9eb4';ctx.textAlign='center';ctx.fillText(index<0?'Camera unavailable / data gap':'No people detected in this frame',width/2,height/2);ctx.textAlign='left';}
}

/** Begin dragging the nearest controller guide when the guide overlay is enabled. */
function beginGuideDrag(event,team) {
  if(!state.game||!$('#anchors').checked)return;
  const rect=event.currentTarget.getBoundingClientRect(),transform=cameraTransform(team); // Screen/camera coordinate transform.
  const distances=anchors[team].map(point=>Math.hypot(event.clientX-rect.left-transform.left-point[0]*transform.width*transform.scale,event.clientY-rect.top-transform.top-point[1]*transform.height*transform.scale)); // Hit distance in screen pixels.
  const slot=distances.indexOf(Math.min(...distances)); // Nearest guide.
  if(distances[slot]<32){state.anchorDrag={team,slot};event.currentTarget.setPointerCapture(event.pointerId);}
}
/** Move a captured guide in normalized camera coordinates, clamped to the image bounds. */
function moveGuide(event,team) {
  if(state.anchorDrag?.team!==team)return;
  const rect=event.currentTarget.getBoundingClientRect(),transform=cameraTransform(team); // Current canvas transform.
  anchors[team][state.anchorDrag.slot]=[Math.max(0,Math.min(1,(event.clientX-rect.left-transform.left)/transform.scale/transform.width)),Math.max(0,Math.min(1,(event.clientY-rect.top-transform.top)/transform.scale/transform.height))];state.dirty=true;
}

/** Estimate six hand-position clusters from the selected game; leave guides unchanged if evidence is weak. */
function suggestGuides() {
  if(!state.game)return;
  const messages=[]; // Per-camera support counts reported to the user.
  for(const team of ['a','b']) {
    const source=state.game.teams[team].skeleton; // All poses in this recording, not only the current frame.
    const points=source.people.flatMap(people=>people.filter(person=>person[3][9]>.65&&person[3][10]>.65).map(person=>[(person[1][9]+person[1][10])/2/source.width,(person[2][9]+person[2][10])/2/source.height])).filter(point=>point[1]>.6&&point[1]<.98); // High-confidence lower-image paired hands.
    if(points.length<120){messages.push(`${team.toUpperCase()}: insufficient paired-hand observations; guides unchanged`);continue;}
    let centers=Array.from({length:6},(_,index)=>(index+.5)/6); // X-only initial clusters preserve left-to-right slots.
    let groups=[]; // Point assignments recomputed by Lloyd iterations.
    for(let iteration=0;iteration<25;iteration++){
      groups=centers.map(()=>[]);
      for(const point of points){const distances=centers.map(center=>Math.abs(point[0]-center));groups[distances.indexOf(Math.min(...distances))].push(point);}
      centers=centers.map((center,index)=>groups[index].length?groups[index].reduce((sum,point)=>sum+point[0],0)/groups[index].length:center);
    }
    if(groups.some(group=>group.length<20)||centers.some((center,index)=>index>0&&center-centers[index-1]<.06)){messages.push(`${team.toUpperCase()}: six well-supported positions not found; guides unchanged`);continue;}
    anchors[team]=groups.map((group,index)=>[centers[index],group.map(point=>point[1]).sort((a,b)=>a-b)[Math.floor(group.length/2)]]);
    messages.push(`${team.toUpperCase()}: provisional guides from ${points.length} hand-pair observations`);
  }
  $('#anchors').checked=true;saveAnchors();$('#loadStatus').textContent=messages.join(' · ')+'. Inspect and drag to correct; these are not a verified calibration. Audio timing remains provisional.';
}

/** Cache selected-joint series at 20 Hz; missing values break traces instead of spanning gaps. */
function traceSeries(team) {
  const joint=state.selected[team],key=`${team}-${joint}`; // Cached series key within current game.
  if(state.charts[key])return state.charts[key];
  const data=state.game.teams[team],series={t:[],lead:[],dial:[],robot:[],limited:[],weights:[[],[],[]]}; // Shared plotting sample grid.
  for(let time=0;time<=state.game.duration;time+=.05){
    const h=valueAt(data.haptic,'dial_robot_deg',time),q=valueAt(data.robot_actual,'q_rad',time),dv=valueAt(data.haptic,'dial_vel_rad_s',time),rv=valueAt(data.robot_actual,'qd_rad_s',time); // Synchronized optional vectors.
    series.t.push(time);series.lead.push(dialLead(h?.[joint],q?.[joint]));series.dial.push(jointDialSpeed(dv?.[joint],data.gear_ratio[joint]));series.robot.push(degrees(rv?.[joint]));series.limited.push(valueAt(data.game_controller,'clamp_final',time));
    for(let bucket=0;bucket<3;bucket++)series.weights[bucket].push(valueAt(data.weight,`bucket_${bucket+1}_g`,time,1));
  }
  state.charts[key]=series;return series;
}

/** Draw a single polyline with explicit breaks for missing samples and no hidden clipping. */
function plotLine(ctx,times,values,color,x,y,start=0,end=Infinity) {
  ctx.strokeStyle=color;ctx.lineWidth=1.2;ctx.beginPath();
  let pen=false; // Whether the previous sample was valid.
  for(let index=0;index<times.length;index++) { if(times[index]<start||times[index]>end||!Number.isFinite(values[index])){pen=false;continue;} if(pen)ctx.lineTo(x(times[index]),y(values[index]));else ctx.moveTo(x(times[index]),y(values[index]));pen=true; }
  ctx.stroke();
}

/** Return the visible time domain for whole-game, Play-only, or 30-second inspection plots. */
function traceDomain() {
  const duration=state.game.duration,mode=$('#traceRange').value; // Selected time-window mode.
  if(mode==='window'){const start=Math.max(0,Math.min(duration-30,state.time-15));return [start,Math.min(duration,start+30)];}
  return [mode==='play'?(state.game.play_start??0):0,duration];
}

/** Overlay a shared white playhead on a cached chart without rebuilding its thousands of samples. */
function plotCursor(ctx,x,start,end,top,bottom) {
  if(state.time<start||state.time>end)return;
  ctx.strokeStyle='#eff5ff';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x(state.time),top);ctx.lineTo(x(state.time),bottom);ctx.stroke();
}

/** Draw linked lead, velocity, limitation, and bucket-weight charts for the selected player. */
function drawCharts(team) {
  const series=traceSeries(team),{context:ctx,width,height}=canvasContext($(`#trace-${team}`)); // Cached synchronized data and current plot surface.
  const [start,end]=traceDomain(),left=48,right=width-12,x=time=>left+(time-start)/Math.max(.001,end-start)*(right-left); // Common visible time-to-pixel mapping.
  const weightCanvas=canvasContext($(`#weights-${team}`)),wc=weightCanvas.context; // Compact bucket-weight plot.
  const cacheKey=`${team}-${state.selected[team]}-${width}-${height}-${start}-${end}`; // Reuse static pixels until size, joint, or visible domain changes.
  const cached=state.charts[`image-${team}`]; // At most two offscreen chart pairs retained.
  if(cached?.key===cacheKey){ctx.drawImage(cached.trace,0,0,width,height);wc.drawImage(cached.weights,0,0,width,weightCanvas.height);plotCursor(ctx,x,start,end,23,205);plotCursor(wc,x,start,end,19,weightCanvas.height-5);return;}
  const visible=values=>values.filter((value,index)=>series.t[index]>=start&&series.t[index]<=end&&Number.isFinite(value)); // Values in the selected window only.
  const absMax=values=>visible(values).reduce((maximum,value)=>Math.max(maximum,Math.abs(value)),5); // Safe reduction for recordings of arbitrary length.
  const leadMax=absMax(series.lead); // Full visible range preserves outliers.
  const speedMax=Math.max(absMax(series.dial),absMax(series.robot)); // Shared scale makes speed comparisons meaningful.
  const leadY=value=>67-value/leadMax*35,speedY=value=>164-value/speedMax*35; // Independent lead/speed axis positions.
  ctx.font='9px Segoe UI';
  let limitedStart=null; // Merge adjacent limited samples to avoid opacity buildup on dense plots.
  for(let i=0;i<=series.t.length;i++){
    const limited=i<series.t.length&&series.t[i]>=start&&series.t[i]<=end&&series.limited[i]!==null&&series.limited[i]<.99; // Valid visible limitation state.
    if(limited&&limitedStart===null)limitedStart=series.t[i];
    if(!limited&&limitedStart!==null){ctx.fillStyle='#f6bf6b28';ctx.fillRect(x(limitedStart),25,Math.max(.5,x(Math.min(end,series.t[i]??end))-x(limitedStart)),176);limitedStart=null;}
  }
  for(const y of [32,67,102,129,164,199]){ctx.strokeStyle=colors.grid;ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();}
  ctx.fillStyle=colors.text;ctx.fillText(`+${number(leadMax,0)}°`,3,35);ctx.fillText('0',25,70);ctx.fillText(`−${number(leadMax,0)}°`,3,105);ctx.fillText(`+${number(speedMax,0)}`,3,132);ctx.fillText('0',25,167);ctx.fillText(`−${number(speedMax,0)}`,3,202);
  ctx.fillStyle=colors[team];ctx.fillText('LEAD · JOINT °',left,16);ctx.fillText('DIAL · JOINT °/s',left,121);ctx.fillStyle=colors.robot;ctx.fillText('ROBOT · °/s',left+110,121);ctx.fillStyle=colors.text;ctx.fillText('amber background = motion limited',Math.max(left+140,right-187),16);
  plotLine(ctx,series.t,series.lead,colors[team],x,leadY,start,end);plotLine(ctx,series.t,series.dial,colors[team],x,speedY,start,end);plotLine(ctx,series.t,series.robot,colors.robot,x,speedY,start,end);
  if(state.game.play_start!==null&&state.game.play_start>=start&&state.game.play_start<=end){ctx.strokeStyle='#718798';ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(x(state.game.play_start),24);ctx.lineTo(x(state.game.play_start),205);ctx.stroke();ctx.setLineDash([]);}
  ctx.fillStyle=colors.text;for(let i=0;i<=4;i++){const time=start+(end-start)*i/4;ctx.textAlign=i===4?'right':'left';ctx.fillText(clock(time),x(time),height-9);}ctx.textAlign='left';
  const all=series.weights.flatMap(visible),maximum=all.reduce((a,b)=>Math.max(a,b),100),minimum=all.reduce((a,b)=>Math.min(a,b),0); // Weight units are grams; retain negatives from sensor noise.
  const wy=value=>weightCanvas.height-9-(value-minimum)/(maximum-minimum)*(weightCanvas.height-31); // Weight vertical scale.
  wc.font='9px Segoe UI';wc.fillStyle=colors.text;wc.fillText(`BUCKET WEIGHTS · ${number(minimum,0)}–${number(maximum,0)} g`,left,13);
  const palette=['#6dd5c4','#a99bf2','#f2ba78']; // Stable bucket 1/2/3 colors.
  series.weights.forEach((values,index)=>{plotLine(wc,series.t,values,palette[index],x,wy,start,end);wc.fillStyle=palette[index];wc.fillText(`B${index+1}`,right-72+index*24,13);});
  const images={key:cacheKey}; // Offscreen snapshots omit playheads so scrubbing only draws cursors.
  for(const prefix of ['trace','weights']){const canvas=$(`#${prefix}-${team}`),copy=document.createElement('canvas');copy.width=canvas.width;copy.height=canvas.height;copy.getContext('2d').drawImage(canvas,0,0);images[prefix]=copy;}
  state.charts[`image-${team}`]=images;plotCursor(ctx,x,start,end,23,205);plotCursor(wc,x,start,end,19,weightCanvas.height-5);
}

/** Update all telemetry readouts and canvases at the shared playhead. */
function renderPlayer() {
  if(!state.game)return;
  const time=state.time,game=state.game; // Shared synchronized playhead.
  $('#scrubber').value=time;$('#timeLabel').textContent=`${clock(time)} / ${clock(game.duration)}`;
  const stage=valueAt(game.global,'stage',time),paused=valueAt(game.global,'paused',time); // Recorded gameplay state, independent of viewer pause.
  $('#stage').textContent=stage?`${stage.toUpperCase()}${paused?' · GAME PAUSED':''}`:'STATE GAP';$('#play').textContent=state.playing?'❚❚ Pause':'▶ Play';
  for(const team of ['a','b']) {
    const data=game.teams[team],haptic=valueAt(data.haptic,'dial_robot_deg',time),robot=valueAt(data.robot_actual,'q_rad',time),dialV=valueAt(data.haptic,'dial_vel_rad_s',time),robotV=valueAt(data.robot_actual,'qd_rad_s',time); // Causally aligned vectors with freshness checks.
    const limit=valueAt(data.game_controller,'clamp_final',time),collision=valueAt(data.game_controller,'in_collision',time),detail=valueAt(data.game_controller,'first_hit_detail',time); // Recorded controller motion state.
    const joint=state.selected[team],torque=valueAt(data.haptic,'torque_ma',time); // Selected-joint diagnostic extras.
    const fault=valueAt(data.robot_actual,'fault_active',time),practice=valueAt(data.game_controller,'practice_player',time); // Robot fault and active tutorial player.
    $(`#status-${team}`).textContent=`${robot?'ROBOT OK':'ROBOT GAP'} · ${haptic?'DIALS OK':'DIAL GAP'}`;
    for(let j=0;j<6;j++){
      const lead=dialLead(haptic?.[j],robot?.[j]); // Unwrapped joint lead.
      $(`#lead-${team}-${j}`).textContent=number(lead);$(`#leadbar-${team}-${j}`).style.left=`${50+Math.max(-1,Math.min(1,(lead??0)/30))*47}%`;$(`#leadbar-${team}-${j}`).style.opacity=lead===null?'0':'1';
      $(`#dial-${team}-${j}`).textContent=`D ${number(jointDialSpeed(dialV?.[j],data.gear_ratio[j]))}°/s`;$(`#velocity-${team}-${j}`).textContent=`R ${number(degrees(robotV?.[j]))}°/s`;
      const loud=valueAt(data.audio[j],'loudness',time-Number($('#audioOffset').value)); // Player-mapped but provisionally timed sound intensity.
      $(`#loud-${team}-${j}`).textContent=number(loud,3);$(`#meter-${team}-${j}`).style.width=`${Math.min(100,Math.sqrt(Math.max(0,loud??0))*60*Number($('#gain').value))}%`;
    }
    $(`#control-${team}`).textContent=`${collision?'COLLISION':collision===null?'CONTROLLER GAP':'No collision flag'} · motion scale ${number(limit,2)} · path ${number(valueAt(data.game_controller,'clamp_path',time),2)} / proximity ${number(valueAt(data.game_controller,'clamp_prox',time),2)}${fault?' · ROBOT FAULT':''}${detail?' · '+detail:''}`;
    $(`#control-${team}`).classList.toggle('limited',collision===true||(limit!==null&&limit<.99)||!!fault);
    if(state.robots[team])$(`#robot-message-${team}`).textContent=robot?'Recorded actual joints · curated cell geometry':'No fresh robot sample — pose hidden';
    $(`#chart-title-${team}`).textContent=`${team.toUpperCase()} · PLAYER ${joint+1} / JOINT ${joint+1}`;
    $(`#extra-${team}`).textContent=`Raw dial speed ${number(degrees(dialV?.[joint]))} °/s · torque ${number(torque?.[joint],0)} mA · actual angle ${number(degrees(robot?.[joint]))}° · tutorial player ${practice===null?'—':practice} · buckets ${[1,2,3].map(bucket=>number(valueAt(data.weight,`bucket_${bucket}_g`,time,1),0)).join(' / ')} g`;
    drawSkeleton(team,time);drawCharts(team);
  }
}

/** Animate playback using elapsed real time; redraw telemetry at 20 Hz and orbit controls every frame. */
function animate(timestamp) {
  const delta=state.last?Math.min(.25,(timestamp-state.last)/1000):0; // Cap foreground-resume jumps.
  state.last=timestamp;
  if(state.game&&!$('#player').hidden){
    if(state.playing){seek(state.time+delta*Number($('#speed').value));if(state.time>=state.game.duration)state.playing=false;}
    if(state.dirty&&(timestamp-(state.drawnAt||0)>45)){renderPlayer();state.dirty=false;state.drawnAt=timestamp;}
    for(const team of ['a','b'])state.robots[team]?.update(valueAt(state.game.teams[team].robot_actual,'q_rad',state.time));
  }
  requestAnimationFrame(animate);
}

/** Toggle playback; restarting after the end returns to the recording start. */
function togglePlay(){if(!state.game)return;if(state.time>=state.game.duration)seek(0);state.playing=!state.playing;state.dirty=true;}

buildPanels();
$('#games').addEventListener('click',event=>{const button=event.target.closest('[data-game]');if(button&&!button.disabled)openGame(button.dataset.game);const header=event.target.closest('[data-sort]');if(header){state.direction=state.sort===header.dataset.sort?-state.direction:-1;state.sort=header.dataset.sort;renderLibrary();}});
for(const selector of ['#search','#localOnly','#shortOnly'])$(selector).addEventListener('input',()=>{state.page=0;renderLibrary();});
$('#refresh').onclick=loadIndex;$('#previous').onclick=()=>{state.page--;renderLibrary();};$('#next').onclick=()=>{state.page++;renderLibrary();};
$('#sortMetric').onchange=event=>{state.sort=event.target.value;state.direction=-1;state.page=0;renderLibrary();};
$('#back').onclick=()=>{state.playing=false;state.loading++;$('#player').hidden=true;$('#library').hidden=false;history.replaceState(null,'',location.pathname);renderLibrary();};
$('#dialRow').addEventListener('click',event=>{const card=event.target.closest('[data-joint]');if(!card)return;state.selected[card.dataset.team]=Number(card.dataset.joint);for(const item of document.querySelectorAll(`[data-team="${card.dataset.team}"]`))item.classList.toggle('active',item===card);state.dirty=true;});
$('#play').onclick=togglePlay;$('#scrubber').oninput=event=>seek(Number(event.target.value));$('#stepBack').onclick=()=>{state.playing=false;seek(state.time-.05);};$('#stepForward').onclick=()=>{state.playing=false;seek(state.time+.05);};$('#jumpPlay').onclick=()=>seek(state.game?.play_start??0);
for(const selector of ['#confidence','#crop','#halos','#gain','#audioOffset','#anchors','#traceRange'])$(selector).addEventListener('input',()=>{state.dirty=true;$('#confidenceValue').textContent=$('#confidence').value;});
$('#suggestAnchors').onclick=suggestGuides;$('#resetAnchors').onclick=()=>{anchors=defaultAnchors();saveAnchors();};
$('#barriers').onchange=event=>{for(const body of state.model?.bodies||[])if(body.hidden){for(const robot of Object.values(state.robots))robot.visibility(body.name,event.target.checked);const checkbox=$(`[data-body="${body.name}"]`);if(checkbox)checkbox.checked=event.target.checked;}};
$('#resetCamera').onclick=()=>Object.values(state.robots).forEach(robot=>robot.reset());
$('#guide').onclick=()=>$('#guideDialog').showModal();$('#closeGuide').onclick=()=>$('#guideDialog').close();
window.addEventListener('resize',()=>{state.dirty=true;});
document.addEventListener('keydown',event=>{if(!state.game||$('#player').hidden||$('#guideDialog').open||['INPUT','SELECT','TEXTAREA','BUTTON'].includes(document.activeElement.tagName))return;if(event.code==='Space'){event.preventDefault();togglePlay();}if(event.key==='ArrowLeft'||event.key==='ArrowRight'){event.preventDefault();seek(state.time+(event.key==='ArrowLeft'?-1:1)*(event.shiftKey?10:1));}});
await loadIndex();
const initial=new URLSearchParams(location.hash.slice(1)).get('game'); // Optional deep link copied from the address bar.
if(initial)openGame(initial);
requestAnimationFrame(animate);
