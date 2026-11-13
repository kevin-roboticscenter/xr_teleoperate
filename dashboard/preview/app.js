'use strict';

// This standalone design preview has no robot client, SDK imports or API requests.
// Its Content Security Policy also blocks all fetch/WebSocket connections.
const ICONS = {
  grid:'<rect x="3" y="3" width="7" height="7" rx="1.5"/><rect x="14" y="3" width="7" height="7" rx="1.5"/><rect x="3" y="14" width="7" height="7" rx="1.5"/><rect x="14" y="14" width="7" height="7" rx="1.5"/>',
  arrow:'<path d="M5 12h14m-6-6 6 6-6 6"/>', plus:'<path d="M12 5v14M5 12h14"/>',
  sliders:'<path d="M4 6h3m5 0h8M4 12h9m5 0h2M4 18h3m5 0h8"/><circle cx="9" cy="6" r="2"/><circle cx="15" cy="12" r="2"/><circle cx="9" cy="18" r="2"/>',
  camera:'<rect x="3" y="6" width="18" height="14" rx="3"/><path d="m8 6 1.5-3h5L16 6"/><circle cx="12" cy="13" r="3.5"/>',
  record:'<rect x="3" y="4" width="18" height="16" rx="3"/><path d="m10 9 5 3-5 3V9Z"/>',
  link:'<path d="m10 14 4-4m-7 6-1 1a4 4 0 0 1-6-6l5-5a4 4 0 0 1 6 0m2 2 1-1a4 4 0 0 1 6 6l-5 5a4 4 0 0 1-6 0" transform="translate(1 0)"/>',
  search:'<circle cx="10.5" cy="10.5" r="6.5"/><path d="m16 16 5 5"/>',
  info:'<circle cx="12" cy="12" r="9"/><path d="M12 10v6m0-9v.1"/>',
  play:'<path d="m8 4 11 8-11 8V4Z"/>', stop:'<rect x="5" y="5" width="14" height="14" rx="2"/>',
  prepare:'<rect x="5" y="6" width="14" height="15" rx="2"/><path d="M9 6V3h6v3m-7 6h8m-8 4h5"/>',
  check:'<path d="m5 12 4 4L19 6"/>', wifi:'<path d="M2 8a16 16 0 0 1 20 0M5 12a11 11 0 0 1 14 0m-11 4a6 6 0 0 1 8 0m-4 4v.1"/>',
  ethernet:'<rect x="4" y="3" width="16" height="12" rx="2"/><path d="M8 15v4h8v-4M8 7v3m4-3v3m4-3v3m-4 9v3"/>',
  headset:'<path d="M5 8h14l2 8a3 3 0 0 1-5 3l-4-3-4 3a3 3 0 0 1-5-3l2-8ZM7 8V5h10v3"/><path d="M7 12h3m-1-1v2m7-1v.1"/>',
  hand:'<path d="M7 11V6a1.5 1.5 0 0 1 3 0v4-6a1.5 1.5 0 0 1 3 0v6-5a1.5 1.5 0 0 1 3 0v6-3a1.5 1.5 0 0 1 3 0v8c0 4-3 6-6 6-3 0-4-1-6-4l-3-5a1.5 1.5 0 0 1 2-2l3 3"/>',
  file:'<path d="M14 3H6a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V9l-6-6Z"/><path d="M14 3v6h6m-12 5h8m-8 3h5"/>',
  clock:'<circle cx="12" cy="12" r="9"/><path d="M12 6v6l4 2"/>', robot:'<rect x="5" y="5" width="14" height="13" rx="4"/><path d="M12 2v3M2 10v5m20-5v5M8 21v-3m8 3v-3M8 10h.1m7.9 0h.1M9 14h6"/>'
};
const icon = (name,cls='') => `<svg class="${cls}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.65" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${ICONS[name]||ICONS.robot}</svg>`;
const esc = text => String(text??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const $ = selector => document.querySelector(selector);
const MODELS = {
  h2:{name:'Unitree H2',short:'H2',hands:'Unitree Dex5',features:['PICO / Quest','Wuji gloves','Head following'],inputs:['PICO controllers','PICO wrist trackers','Quest / PICO browser hands'],sides:['Both arms','Left arm'],cameras:['Head camera','Left wrist camera','Right wrist camera'],branch:'H2 integration',source:'https://github.com/kevin-roboticscenter/xr_teleoperate/tree/053ba99af55c1d8ddc6b87918cb0fa234625886e/integrations/h2'},
  g1:{name:'Unitree G1',short:'G1',hands:'BrainCo Revo2',features:['PICO controllers / trackers','Wuji gloves','Arm calibration'],inputs:['PICO controllers','PICO wrist trackers'],sides:['Both arms','Left arm','Right arm'],cameras:['Robot camera'],branch:'New G1 dashboard branch',source:'https://github.com/RoboticsCenter/xr_teleoperate/tree/50e8e07e93f6360197efca81fbd4e21c4339f477/integrations/g1'},
  custom:{name:'Custom robot',short:'R',hands:'Adapter required',features:['Custom model','Profile based'],inputs:[],sides:[],cameras:[],branch:'Adapter required'}
};
const makeProfile = (id,model,name,transport,address,connected=true) => ({id,model,name,transport,address,connected,phase:'idle',armInput:MODELS[model].inputs[0]||'',side:MODELS[model].sides[0]||'',reference:'Fixed at Start',bodyMode:'Standing',autoRecord:true,headFollow:false,recording:false,started:0,samples:[]});
const seed = () => [makeProfile('h2-main','h2','H2 · Main lab','Ethernet','h2-main.local'),makeProfile('g1-main','g1','G1 · Manipulation lab','Wi-Fi','g1-main.local')];
const STORAGE='robotics-center-dashboard-preview-v1';
let robots=seed(),filter='all',search='',toastTimer;
try{const saved=JSON.parse(localStorage.getItem(STORAGE));if(Array.isArray(saved)&&saved.length&&saved.every(r=>MODELS[r.model]&&/^[a-z0-9-]+$/.test(r.id)))robots=saved.map(r=>({...r,phase:'idle',recording:false,started:0}));}catch{}
const save=()=>{try{localStorage.setItem(STORAGE,JSON.stringify(robots));}catch{}};
const profile=id=>robots.find(r=>r.id===id);
const selected=()=>profile(decodeURIComponent(location.hash.split('/')[2]||''));
const tabNames=['operate','cameras','recordings','setup'];
function route(){const p=location.hash.replace(/^#\/?/,'').split('/');return p[0]==='robots'&&profile(p[1])?{robot:profile(p[1]),tab:tabNames.includes(p[2])?p[2]:'operate'}:{robot:null,tab:'home'};}
function go(id,tab='operate'){location.hash=id?`/robots/${id}/${tab}`:'/';}
function toast(text){$('#toast').textContent=text;$('#toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('#toast').classList.remove('visible'),2600);}

let artSequence=0;
function robotArt(model='h2'){
  const uid='metal'+(++artSequence),g1=model==='g1',accent=g1?'#849ea7':'#a6a49c';
  return `<svg class="robot-illustration" viewBox="0 0 150 230" aria-hidden="true"><defs><linearGradient id="${uid}" x1="0" x2="1"><stop stop-color="#aeb9bc"/><stop offset=".28" stop-color="#e8edeb"/><stop offset=".6" stop-color="#c8d1d0"/><stop offset="1" stop-color="#98a8ac"/></linearGradient><linearGradient id="${uid}dark"><stop stop-color="#253940"/><stop offset="1" stop-color="#52636a"/></linearGradient></defs><ellipse cx="76" cy="221" rx="39" ry="5" fill="#8b9ba0" opacity=".18"/><g stroke="#87989d" stroke-width=".7"><path d="M61 39h29v17H61z" fill="#61747b"/><rect x="${g1?54:48}" y="${g1?12:8}" width="${g1?43:55}" height="${g1?31:35}" rx="${g1?12:9}" fill="url(#${uid})"/><path d="${g1?'M56 21q19-10 39 0v12H56Z':'M51 20h49v13H51Z'}" fill="url(#${uid}dark)"/><path d="M64 26h7m10 0h7" stroke="#bdd3d3" stroke-width="1.8"/><path d="M49 53q26-10 53 0l-3 52q-23 9-47 0Z" fill="url(#${uid})"/><path d="M58 62h34l-4 20H63Z" fill="${accent}" opacity=".5"/><path d="M58 89h34" stroke="#879ba1"/><rect x="67" y="83" width="17" height="4" rx="1" fill="#758c94"/><circle cx="44" cy="60" r="11" fill="#667d86"/><circle cx="106" cy="60" r="11" fill="#667d86"/><path d="m35 68 15 2-6 43-12-1Z" fill="url(#${uid})"/><path d="m101 70 15-2 3 44-12 1Z" fill="url(#${uid})"/><circle cx="38" cy="114" r="7" fill="#607983"/><circle cx="113" cy="114" r="7" fill="#607983"/><path d="m31 119 13 1-4 35-11-1Z" fill="url(#${uid})"/><path d="m107 120 13-1 2 35-11 1Z" fill="url(#${uid})"/><path d="m29 154 12 2 2 14-4 9-4-3-2-10-3 8-3-3Z" fill="#7e939a"/><path d="m111 156 12-2 3 17-3 3-3-8-2 10-4 3-4-9Z" fill="#7e939a"/><path d="M53 108h45l-2 22-40 1Z" fill="url(#${uid}dark)"/><circle cx="59" cy="129" r="10" fill="#6c838c"/><circle cx="91" cy="129" r="10" fill="#6c838c"/><path d="m50 133 19 1-3 40H48Z" fill="url(#${uid})"/><path d="m82 134 19-1 2 41H85Z" fill="url(#${uid})"/><rect x="48" y="172" width="18" height="13" rx="5" fill="#5a747e"/><rect x="85" y="172" width="18" height="13" rx="5" fill="#5a747e"/><path d="M49 184h16l-1 29H47Z" fill="url(#${uid})"/><path d="M86 184h16l2 29H87Z" fill="url(#${uid})"/><path d="m46 211 20 1-2 10H38v-5Z" fill="#526a75"/><path d="m85 212 20-1 8 6v5H87Z" fill="#526a75"/></g></svg>`;
}
function scene(label){return `<div class="camera-scene"><svg viewBox="0 0 480 250" preserveAspectRatio="xMidYMid slice" aria-hidden="true"><rect width="480" height="250" fill="#e8edee"/><path d="M0 0h480v160H0z" fill="#e0e6e6"/><path d="M0 160h480v90H0z" fill="#d5dfe0"/><path d="M0 0 70 45v115L0 220m480-220-70 45v115l70 60M70 45h340M70 160h340" stroke="#b7c7ca" fill="none"/><path d="m70 160-70 90m135-90-35 90m105-90-5 90m75-90 25 90m45-90 55 90m10-90 70 90M30 192h420M0 230h480" stroke="#bdccce" fill="none"/><rect x="98" y="85" width="100" height="75" rx="3" fill="#bccacc"/><rect x="103" y="90" width="90" height="8" fill="#cbd5d6"/><rect x="105" y="132" width="86" height="5" fill="#a7b9bd"/><path d="m269 133 52-17 51 17-50 19Z" fill="#c6b9a7"/><path d="m269 133 53 19v42l-53-18Z" fill="#b1a28e"/><path d="m322 152 50-19v43l-50 18Z" fill="#bfaf99"/><path d="M240 110v30m-15-15h30" stroke="#90a7ae" opacity=".8"/><rect x="337" y="68" width="53" height="43" rx="2" fill="#c4d2d5"/><path d="M340 72h47v33h-47Z" fill="#afc2c8"/></svg><span class="camera-overlay">${esc(label.toUpperCase())} · SIMULATED</span><span class="camera-corner">UI PREVIEW</span></div>`;}


function renderNav(current){
 $('#fleet-nav').innerHTML=icon('grid')+'<span>Robots</span><span class="count">'+robots.length+'</span>';
 $('#fleet-nav').classList.toggle('active',!current);
 $('#robot-nav').innerHTML=robots.map(r=>`<a class="robot-nav-item ${current?.id===r.id?'active':''}" href="#/robots/${r.id}/operate" title="${esc(r.name)}"><span class="mini-model">${MODELS[r.model].short}</span><span class="robot-nav-name">${esc(r.name.split(' · ')[0])}<small>${esc(r.name.split(' · ')[1]||MODELS[r.model].name)}</small></span>${r.phase==='following'?'<i class="dot pulse"></i>':''}</a>`).join('');
 $('#breadcrumb').innerHTML=current?`<a href="#/">Robots</a><span class="crumb-chevron">›</span><strong>${esc(current.name)}</strong>`:'<strong>Robots</strong>';
}

function card(r){
 const m=MODELS[r.model];
 return `<article class="robot-card" data-robot-card="${r.id}"><a class="robot-card-art ${r.model}" href="#/robots/${r.id}/operate" aria-label="Open ${esc(r.name)}"><span class="card-badge"><i class="dot ${r.connected?'':'muted'}"></i>${r.connected?'Ready · Preview':'Setup needed'}</span>${robotArt(r.model)}</a><div class="robot-card-content"><div class="robot-card-title"><h2>${esc(r.name.split(' · ')[0])}</h2></div><p class="robot-card-description">${esc(r.name.split(' · ').slice(1).join(' · ')||m.name)}</p><p class="card-hardware">${esc(m.hands)} <span>·</span> ${esc(r.transport)}</p><div class="card-open"><button class="button primary" data-action="open" data-id="${r.id}">Open dashboard ${icon('arrow')}</button></div></div></article>`;
}

function renderCards(){
 const visible=robots.filter(r=>(filter!=='setup'||!r.connected)&&`${r.name} ${MODELS[r.model].name} ${MODELS[r.model].hands}`.toLowerCase().includes(search.toLowerCase()));
 $('#robot-grid').innerHTML=(visible.length?visible.map(card).join(''):'<div class="empty-search">No robots found. Try another name.</div>')+`<button class="add-card" data-action="add"><span class="add-circle">${icon('plus')}</span><span><h3>Add a robot</h3><p>Another H2, a G1,<br>or something new.</p></span></button>`;
 document.querySelectorAll('[data-filter]').forEach(b=>b.classList.toggle('active',b.dataset.filter===filter));
}

function renderHome(){
 $('#main').innerHTML=`<section class="page-heading"><div><h1>Your robots.</h1><p class="muted">Choose a robot to get started.</p></div><button class="button primary" data-action="add">${icon('plus')} Add robot</button></section><section class="fleet-tools" aria-label="Filter robots"><div class="filters"><button class="active" data-filter="all">All robots</button><button data-filter="setup">Needs setup</button></div><label class="search" aria-label="Search robots">${icon('search')}<input id="robot-search" placeholder="Search robots" value="${esc(search)}"></label></section><section class="robot-grid" id="robot-grid" aria-label="Robot profiles"></section><p class="fleet-footnote">Each robot keeps its own controls, cameras, and settings.</p>`;
 renderCards();
}

function heading(r,tab){
 const m=MODELS[r.model];
 return `<section class="page-heading robot-heading"><div class="robot-title"><span class="model-avatar ${r.model}">${m.short}</span><div><h1>${esc(r.name)}</h1><p>${esc(m.name)} <span>·</span> ${esc(m.hands)}</p></div></div><div class="heading-actions">${r.phase==='following'?'<button class="button danger" data-action="stop">'+icon('stop')+'Stop</button>':''}<span class="connection-status"><i class="dot ${r.connected?'':'muted'}"></i>${r.connected?'Ready':'Not connected'}<span>· ${esc(r.transport)}</span></span></div></section><nav class="tabs" aria-label="Robot workspace pages">${[['operate','sliders','Control'],['cameras','camera','Cameras'],['recordings','record','Recordings'],['setup','link','Settings']].map(([id,ic,label])=>`<a class="tab ${tab===id?'active':''}" href="#/robots/${r.id}/${id}" ${tab===id?'aria-current="page"':''}>${icon(ic)}${label}</a>`).join('')}</nav>`;
}
const optionList=(values,current)=>values.map(v=>`<option value="${esc(v)}" ${v===current?'selected':''}>${esc(v==='Quest / PICO browser hands'?'Headset hand tracking':v)}</option>`).join('');
const inputRow=(ic,title,detail)=>`<div class="device-row"><span class="device-symbol">${icon(ic)}</span><span class="device-info">${title}<small>${detail}</small></span><span class="device-ready"><i class="dot"></i>Ready</span></div>`;
const elapsed=r=>r.started?Math.floor((Date.now()-r.started)/1000):0;
const timeText=t=>String(Math.floor(t/60)).padStart(2,'0')+':'+String(t%60).padStart(2,'0');

function operate(r){
 const m=MODELS[r.model],following=r.phase==='following',ready=r.phase==='ready',usable=r.connected&&r.model!=='custom',vr=r.armInput.includes('browser');
 const title=following?'Following your movements.':ready?'Ready to start.':'Let’s get ready.';
 const detail=following?'Move naturally. Your selected arms and hands follow.':ready?'Hold a comfortable starting pose, then select Start.':'Check your controls below, then select Prepare.';
 const fingers=vr?'Headset hand tracking':'Wuji gloves';
 return `${!usable?'<div class="setup-notice">'+(r.model==='custom'?'This model needs a robot integration before it can be controlled.':'Open Settings to connect this robot first.')+'</div>':''}<div class="operate-grid"><div><section class="panel control-panel"><div class="session-status"><div><span class="state-pill ${r.phase}"><i class="dot"></i>${following?'Following':ready?'Prepared':'Not started'}</span><h2>${title}</h2><p>${detail}</p></div><div class="session-time"><span class="session-clock" data-clock="${r.id}">${timeText(elapsed(r))}</span><small>Session time</small></div></div><div class="session-buttons"><button class="button" data-action="prepare" ${!usable||r.phase!=='idle'?'disabled':''}>${icon('prepare')}Prepare</button><button class="button primary" data-action="start" ${!usable||!ready?'disabled':''}>${icon('play')}Start</button><button class="button danger" data-action="stop" ${r.phase==='idle'?'disabled':''}>${icon('stop')}Stop</button></div><div class="step-guide"><span class="${r.connected?'done':'current'}"><b>1</b>Connect</span><i></i><span class="${ready||following?'done':'current'}"><b>2</b>Prepare</span><i></i><span class="${following?'done':ready?'current':''}"><b>3</b>Control</span></div></section><section class="panel session-config"><div class="panel-heading"><h3>Your controls</h3><span>Saved for this robot</span></div><div class="form-grid"><label>Move arms with<select data-setting="armInput" ${following||!usable?'disabled':''}>${optionList(m.inputs,r.armInput)}</select></label><label>Arms to control<select data-setting="side" ${following||!usable?'disabled':''}>${optionList(vr?['Both arms']:m.sides,r.side)}</select></label></div><div class="settings-summary"><span>${icon('hand')}Fingers</span><strong>${esc(fingers)}</strong></div>${r.model==='h2'?`<label class="switch-row"><span>Head tracking<small>Let the robot’s head follow your headset.</small></span><input class="switch" type="checkbox" role="switch" data-setting="headFollow" ${r.headFollow?'checked':''} ${following?'disabled':''}></label>`:''}${r.model==='g1'?`<details class="more-options"><summary>More options</summary><label>Arm reference<select data-setting="reference" ${following?'disabled':''}>${optionList(['Fixed at Start','Relative to headset'],r.reference)}</select></label><p class="settings-note">Choose whether arm movement is relative to your starting position or your headset.</p></details>`:''}</section><section class="panel camera-panel"><div class="panel-heading"><h3>${r.model==='g1'?'Robot view':'Head camera'}</h3><button class="inline-link" data-action="tab" data-tab="cameras">View cameras ${icon('arrow')}</button></div>${scene(r.model==='g1'?'Robot view':'Head camera')}</section></div><div class="side-panels"><section class="panel device-panel"><div class="panel-heading"><h3>Connections</h3><span class="status-caption">Preview</span></div>${inputRow('robot',esc(m.short),esc(r.transport))}${inputRow('headset',vr?'Headset tracking':'PICO',vr?'Arms and fingers':esc(r.armInput))}${inputRow('hand',fingers,vr?esc(m.hands):(r.side==='Both arms'?'Left and right glove':r.side==='Left arm'?'Left glove':'Right glove'))}<button class="inline-link settings-link" data-action="tab" data-tab="setup">Connection settings ${icon('arrow')}</button></section><section class="panel recording-panel"><div class="panel-heading"><h3>Recording</h3>${r.recording?'<span class="record-badge"><i class="dot"></i>Recording</span>':'<span>Not recording</span>'}</div><label class="switch-row"><span>Record automatically<small>Save the camera views when following starts.</small></span><input class="switch" type="checkbox" role="switch" data-setting="autoRecord" ${r.autoRecord?'checked':''}></label><button class="button full" data-action="record" ${!following?'disabled':''}>${icon('record')}${r.recording?'Stop recording':'Record now'}</button><button class="inline-link recordings-link" data-action="tab" data-tab="recordings">View recordings ${icon('arrow')}</button></section><div class="context-note">${r.side==='Both arms'?'Both arms are selected.':r.side==='Left arm'?'Only the left side follows. The right side holds its position.':'Only the right side follows. The left side holds its position.'}${r.model==='g1'?'<br>Walking stays on the Unitree remote.':''}</div></div></div>`;
}
function cameras(r){const m=MODELS[r.model];return `<p class="page-subtitle">Keep the robot’s views together. These example scenes show the layout; they are not live camera feeds.</p><div class="camera-grid">${m.cameras.map((name,i)=>`<section class="panel"><div class="panel-heading"><h3>${esc(name)}</h3><span>Simulated camera ${i+1}</span></div>${scene(name)}${i===0?`<p class="settings-note">${r.model==='h2'?'Head following is controlled separately on the Operate page.':'G1 camera controls remain separate from arm following.'}</p>`:''}</section>`).join('')||'<div class="setup-notice">No camera capability is registered for this model yet.</div>'}</div>`;}
function recordings(r){
 const samples=[...r.samples,{name:'Sample manipulation session',when:'Example recording',duration:'04:32',views:r.model==='h2'?'Head + both wrists':'Robot camera',size:'Preview sample'},{name:'Sample calibration check',when:'Example recording',duration:'01:18',views:r.model==='h2'?'Head + both wrists':'Robot camera',size:'Preview sample'}];
 return `<section class="panel"><div class="recordings-heading"><div><h3>Saved sessions</h3><p class="muted" style="margin-top:7px">Recordings are kept with ${esc(r.name)}. All rows here are preview samples.</p></div><span class="sample-pill">SIMULATED DATA</span></div><table class="record-table"><thead><tr><th>SESSION</th><th>DURATION</th><th>VIEWS</th><th></th></tr></thead><tbody>${samples.map(s=>`<tr><td><div class="file-icon">${icon('file')}<span>${esc(s.name)}<small>${esc(s.when)}</small></span></div></td><td class="table-muted">${esc(s.duration)}</td><td class="table-muted">${esc(s.views)}</td><td><button class="button small" data-action="sample">View</button></td></tr>`).join('')}</tbody></table></section>`;
}

function setup(r){
 const m=MODELS[r.model];
 return `${r.model==='custom'?'<div class="setup-notice">This profile is saved. A robot integration is needed before controls can be enabled for this model.</div>':''}<div class="setup-grid"><section class="panel"><div class="panel-heading"><h3>Connection</h3><span>${esc(m.short)}</span></div><form id="connection-form"><label>Robot address<input name="address" value="${esc(r.address)}" placeholder="IP address or hostname" maxlength="100" required></label><div class="form-grid"><label>Connect using<select name="transport">${optionList(['Ethernet','Wi-Fi'],r.transport)}</select></label><label>Robot model<input value="${esc(m.name)}" readonly></label></div><div class="form-actions"><button class="button primary" type="submit">Save</button><button type="button" class="button" data-action="connect" ${r.model==='custom'?'disabled':''}>${r.connected?'Reconnect':'Connect'}</button></div></form><p class="settings-note">Preview settings are saved on this computer. No network connection is made.</p></section><section class="panel"><div class="panel-heading"><h3>Calibration</h3><span>Personal settings</span></div><div class="calibration-row"><span>Glove fit<small>Your personal left and right hand models</small></span><button class="button small" data-action="calibration" data-kind="glove">Set up</button></div><div class="calibration-row"><span>Arm alignment<small>Match your movement to the robot</small></span><button class="button small" data-action="calibration" data-kind="arm">Set up</button></div>${r.model==='g1'?'<div class="calibration-row"><span>Open and close<small>Calibrate finger movement</small></span><button class="button small" data-action="calibration" data-kind="mapping">Set up</button></div>':''}<p class="settings-note">Each robot and input type keeps its own calibration.</p></section>${r.model!=='custom'?`<section class="panel model-settings"><div class="panel-heading"><h3>${m.short} options</h3></div>${r.model==='h2'?`<label>Robot mode<select data-setting="bodyMode" ${r.phase!=='idle'?'disabled':''}>${optionList(['Standing','Walking with arm control'],r.bodyMode||'Standing')}</select></label><p class="settings-note">The Unitree remote controls walking. Choose the body mode before preparing the arms.</p><label class="switch-row"><span>Head tracking<small>Follow your headset with the robot’s head.</small></span><input type="checkbox" role="switch" class="switch" data-setting="headFollow" ${r.headFollow?'checked':''} ${r.phase==='following'?'disabled':''}></label>`:`<div class="settings-summary"><span>Robot hands</span><strong>BrainCo Revo2</strong></div><div class="settings-summary"><span>Finger input</span><strong>Wuji gloves</strong></div><p class="settings-note">Controller and wrist-tracker calibration are saved separately. The arm reference can be changed on the Control page.</p>`}</section>`:''}</div>`;
}
function render(){
 const {robot:r,tab}=route();renderNav(r);
 if(!r)renderHome();else{$('#main').innerHTML=heading(r,tab)+({operate,cameras,recordings,setup}[tab])(r);}
 document.title=(r?r.name+' · ':'')+'Robotics Center · Preview';
}
function showDetail(title,content){$('#detail-title').textContent=title;$('#detail-content').innerHTML=content;$('#detail-dialog').showModal();}
function modelHelp(){const value=$('#new-model').value;$('#model-help').textContent=value==='custom'?'Save this robot now. A supported integration is needed before it can be controlled.':`${MODELS[value].name} controls will be available in its own dashboard. Its connection and calibration stay separate from your other robots.`;}
function openAdd(){$('#add-form').reset();modelHelp();$('#add-dialog').showModal();}
function stopPreview(r){
 if(r.recording)r.samples.unshift({name:'Preview session',when:'Created in this UI preview',duration:timeText(elapsed(r)),views:r.model==='h2'?'Head + both wrists':'Robot camera',size:'Preview sample'});
 r.phase='idle';r.recording=false;r.started=0;
}
document.addEventListener('click',e=>{
 const filterButton=e.target.closest('[data-filter]');if(filterButton){filter=filterButton.dataset.filter;renderCards();return;}
 const button=e.target.closest('[data-action]');if(!button||button.disabled)return;
 const a=button.dataset.action,r=selected();
 if(a==='add')return openAdd();
 if(a==='close-dialog')return $('#add-dialog').close();
 if(a==='close-detail')return $('#detail-dialog').close();
 if(a==='open')return go(button.dataset.id);
 if(a==='reset'){robots=seed();filter='all';search='';save();if(location.hash&&location.hash!=='#/')go(null);else render();return toast('Preview reset.');}
 if(!r)return;
 if(a==='tab')return go(r.id,button.dataset.tab);
 if(a==='sample')return showDetail('Sample recording',`<p>This represents a session saved under <strong>${esc(r.name)}</strong>. In the connected dashboard, this view would contain that robot’s videos and session details.</p><p>No recording file is created or downloaded by this preview.</p>`);
 if(a==='calibration'){
  const kind=button.dataset.kind;return showDetail(kind==='glove'?'Wuji glove calibration':kind==='mapping'?'Open / fist mapping':'Arm calibration',kind==='glove'?'<p>The connected workspace would open the existing official Wuji calibration workflow.</p><ol><li>Select the operator and left/right glove.</li><li>Review the saved hand model or begin a new calibration.</li><li>Follow the guided poses, then save and verify.</li></ol><p>This preview does not start calibration.</p>':kind==='mapping'?'<p>G1 retains the original Wuji → BrainCo finger mapping. The connected page would guide open/fist capture and mapping verification.</p><p>No finger targets are sent by this preview.</p>':'<p>The connected page would manage this robot’s existing arm calibration.</p><ol><li>Select controller or wrist tracker input.</li><li>Choose the arm and reference frame.</li><li>Capture alignment and measured-distance samples, then verify.</li></ol><p>G1 keeps input/reference-specific calibrations. H2 uses its own existing mapping.</p>');
 }
 if(a==='connect'){r.connected=true;toast('Preview connected.');}
 if(a==='prepare'&&r.connected&&r.model!=='custom'&&r.phase==='idle'){r.phase='ready';toast('Prepared. You can start now.');}
 if(a==='start'&&r.phase==='ready'){r.phase='following';r.started=Date.now();r.recording=r.autoRecord;toast('Following started in this preview.');}
 if(a==='stop'){stopPreview(r);toast('Preview following stopped for '+r.name+'.');}
 if(a==='record'&&r.phase==='following'){r.recording=!r.recording;toast(r.recording?'Preview recording started.':'Preview recording stopped.');}
 save();render();
});
document.addEventListener('change',e=>{
 if(e.target.id==='new-model')return modelHelp();
 const setting=e.target.dataset.setting,r=selected();if(!r||!setting)return;
 if(!['armInput','side','reference','autoRecord','headFollow','bodyMode'].includes(setting))return;
 r[setting]=e.target.type==='checkbox'?e.target.checked:e.target.value;
 if(setting==='armInput'&&r.armInput.includes('browser'))r.side='Both arms';
 save();render();
});
document.addEventListener('input',e=>{if(e.target.id==='robot-search'){search=e.target.value;renderCards();}});
$('#add-form').addEventListener('submit',e=>{
 e.preventDefault();const form=e.currentTarget;if(!form.reportValidity())return;
 const data=new FormData(form),id='robot-'+Date.now().toString(36),name=String(data.get('name')).trim(),address=String(data.get('address')).trim();
 if(!name||!address)return;
 robots.push(makeProfile(id,String(data.get('model')),name,String(data.get('transport')),address,false));save();$('#add-dialog').close();go(id,'setup');toast('Robot added. Finish its setup below.');
});
document.addEventListener('submit',e=>{
 if(e.target.id!=='connection-form')return;e.preventDefault();if(!e.target.reportValidity())return;
 const r=selected(),data=new FormData(e.target);r.address=String(data.get('address')).trim();r.transport=String(data.get('transport'));save();render();toast('Preview connection profile saved.');
});
window.addEventListener('hashchange',render);
setInterval(()=>document.querySelectorAll('[data-clock]').forEach(n=>{const r=profile(n.dataset.clock);if(r)n.textContent=timeText(elapsed(r));}),1000);
$('#preview-icon').innerHTML=icon('info');render();
