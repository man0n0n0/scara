// Tracker dashboard and control endpoints (port 80).
//   /              dashboard
//   /status        JSON
//   /enable        start tracking (FluidNC not Idle: soft reset + $H first; refused unless calibrated, Idle, inside limits)
//   /disable       stop tracking + jog cancel; no automatic motion (auto-enable,
//                  soft-limit return) until reboot or /enable
//   /calibrate     measure the image Jacobian (target must be visible and still)
//   /clearcal      forget the calibration
//   /set?kp=1.2&vmax=40   change parameters, saved in NVS
//   /setcal?th=-90&k=0.0015   set J by hand: +X at image angle th (deg, atan2(dv,du)),
//                  k image units per mm, valid at the current pose; saved in NVS
//   Fixed camera (CAMERA_FIXED), teaching the floor map:
//   /mark          image point = measured detection (head visible and still)
//   /markat?u=0.1&v=-0.3   image point = given position (-1..1)
//   /record        pair the marked image point with the current arm XY
//   /delpoint?i=0  delete taught point i (0-based)
//   /solvemap      fit the floor map to the points (>= 4)
//   /clearmap      forget all points and the map
//   /snapshot.jpg  last camera frame
//   /light?c=spot+1   UDP command to the light controller (spot/light/all 0/1), returns its state;
//                  turns automatic lights off (until /autolights or reboot)
//   /autolights?on=1  automatic lights: target seen -> spot, none for light_s -> light
//   /cam           camera settings JSON; /camctl?var=aec_value&val=300 changes one now;
//   /camsave       saves them (NVS "xcam", shared with the capture firmware)
//   /fake?u=0.3&v=0&ttl=1000   (stub build) pretend the target is at (u, v)

#include <Arduino.h>
#include <esp_http_server.h>
#include <lwip/sockets.h>
#include <stdarg.h>
#include "config.h"
#include "controller.h"
#include "lights.h"
#include "vision.h"

static const char DASH_HTML[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Wormbot tracker</title>
<style>
body{font-family:system-ui,sans-serif;margin:16px;background:#111;color:#eee;max-width:900px}
button{padding:10px 18px;margin:4px;font-size:1em}
#stop{background:#c22;color:#fff;font-weight:bold}
canvas{max-width:100%;border:1px solid #444}
pre{background:#222;padding:8px;overflow:auto;font-size:12px}
input{width:6em}
fieldset{border:1px solid #444;margin:8px 0}
.msg{color:#ff8;margin:4px 0}
table{border-collapse:collapse;font-size:13px}td button{padding:2px 8px;margin:0;font-size:12px}td,th{border:1px solid #444;padding:2px 6px;text-align:right}
.mode{font-size:1.4em;font-weight:bold}
</style></head><body>
<h3>Wormbot visual tracker</h3>
<div><span class="mode" id="mode">-</span> <span id="reason"></span></div>
<div>
<button id="stop" onclick="go('/disable')" title="no automatic motion until reboot or Enable">STOP</button>
<button onclick="go('/enable')">Enable</button>
<span class="jac"><button onclick="go('/calibrate')">Calibrate</button>
<button onclick="if(confirm('Forget the calibration?'))go('/clearcal')">Clear calibration</button></span>
</div>
<div>Lights: <button onclick="light('spot 1')">Spot on</button><button onclick="light('spot 0')">Spot off</button>
<button onclick="light('light 1')">Light on</button><button onclick="light('light 0')">Light off</button>
<button onclick="light('all 0')">All off</button> <span id="lights">-</span>
 <button onclick="go('/autolights?on=1')">Auto lights</button> <span id="autol">-</span></div>
<canvas id="cv" width="320" height="240"></canvas>
<fieldset id="fm" class="fix"><legend>Floor map (fixed camera)</legend>
<ol>
<li>Put the head on the floor where the camera sees it, then
 <button onclick="go('/mark')">Mark from detection</button> or click its spot on the image.</li>
<li>Jog the tool exactly over that spot with FluidNC, then <button onclick="go('/record')">Record arm position</button>.</li>
<li>Repeat for 4 or more spots spread over the workspace (6&ndash;8 is better: the extra ones check the fit), then
 <button onclick="go('/solvemap')">Solve</button>.</li>
</ol>
<div id="fm_msg" class="msg"></div>
<table id="fm_pts"></table>
<button onclick="if(confirm('Forget all points and the map?'))go('/clearmap')">Clear all points</button>
<div><small>On the image: numbered dots = taught points, cyan ring = marked point waiting for Record,
grid = arm X/Y every 200 mm as the map sees the floor, amber = soft limits, orange = where the map puts the tool now.
If the map is right, the orange marker sits on the tool.</small></div>
</fieldset>
<fieldset id="man" class="jac"><legend>Manual calibration</legend>
<div>+X direction <input type="range" id="m_a" min="-180" max="180" step="1" style="width:14em" oninput="medit()">
 <span id="m_av"></span>&deg; <small>(or click on the image)</small></div>
<div>scale <input id="m_k" oninput="medit()"> image units / 100 mm</div>
<div><button onclick="mload()">From current</button> <button onclick="mapply()">Apply</button>
 <button onclick="mcancel()">Cancel</button> <span id="m_msg"></span></div>
<small>Dotted white/yellow = preview: +X white, +Y yellow. The calibration is taken at the arm's current pose.</small>
</fieldset>
<details id="camd"><summary>Camera</summary>
<div id="camctl"></div>
<button onclick="fetch('/camsave').then(r=>r.text()).then(t=>document.getElementById('cammsg').textContent=t)">Save camera settings</button> <span id="cammsg"></span>
<div><small>Changes apply at once (watch the image above) but are lost at reboot until saved. Saved settings
are shared with the capture firmware. The model was trained on images taken with certain settings: big changes
(framesize, exposure, white balance) can make detection worse.</small></div>
</details>
<pre id="st"></pre>
<div id="params"></div>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');let S={};
// What each setting does (PC) and each status field means (SC). Units: mm,
// mm/min, s, ms; image units are -1..1 across the image. "forearm only" =
// used only with CAMERA_FIXED 0 (camera on the arm), not with the wall camera.
const PC={
 kp:'forearm only: share of the remaining move commanded per second (1/s)',
 vmax:'forearm only: speed cap, mm/s',
 amax:'forearm only: acceleration cap, mm/s\u00b2',
 db_in:'forearm only: stop correcting when the target is this close to the image centre (image units)',
 db_out:'forearm only: start correcting again beyond this (image units, > db_in)',
 alpha:'smoothing of the target position: 1 = raw detection, smaller = smoother but slower',
 min_conf:'detection confidence needed to count as a target (0..1)',
 lost_ms:'no detection for this long = target lost, ms',
 horizon_s:'forearm only: length of each tracking jog, s',
 cal_step:'forearm only: first calibration move, mm',
 cal_feed:'forearm only: calibration speed, mm/min',
 off_x:'where the tool goes relative to the target, X mm (also the spiral centre)',
 off_y:'same, Y mm',
 db_mm_in:'over the target: stop moving when the tool is this close to it, mm',
 db_mm_out:'start moving again when the target is farther than this, mm (> db_mm_in)',
 recover_feed:'speed of the jog back inside the soft limits, to the centre after boot/homing, and of the ALARM:2 return, mm/min',
 wander_s:'no target for this long: random straight moves, s (0 = never)',
 wander_feed:'speed of the random moves, mm/min',
 center_x:'workspace centre X, mm: where the tool goes after boot/homing, ALARM:2 return point, wander fallback',
 center_y:'workspace centre Y, mm',
 light_s:'automatic lights: no target for this long switches spot off, light on, s',
 orbit_r:'target lost: outer radius of the spiral round the last spot, mm',
 orbit_feed:'target lost: speed along the spiral, mm/min',
 spiral_pitch:'how much the spiral radius changes per turn, mm',
 spiral_r_min:'inner radius: the spiral turns back out here, mm',
 light_hold_s:'automatic lights: once the light is on it stays on at least this long, s',
 fast_feed:'target seen: speed of the straight move over it, mm/min (FluidNC max_rate may cap it)',
 z_max:'Z while the general light is on (no target), mm',
 z_min:'Z otherwise (target seen / spot), mm',
 z_up_feed:'Z speed going up, mm/min (above FluidNC Z max_rate, 600, it slows the XY move too)',
 z_down_feed:'Z speed going down, mm/min',
 wander_margin:'random moves stay this far inside the soft-limit border, mm',
 z_chunk_s:'length of each Z-only jog piece, s (shorter = XY jogs wait less behind Z)'};
const SC={
 mode:'disabled / tracking (moving over the target) / holding (over it) / lost (spiralling round the last spot) / wandering / waiting (no FluidNC) / calibrating',
 operator_off:'true after STOP: no automatic motion until Enable or reboot',
 human_off:'true after UDP "human 0": motion and lights off until "human 1" (or Enable)',
 auto_lights:'automatic spot/light switching (off after a manual light button, on at boot)',
 lamp:'what automatic control last switched on',
 reason:'why it is doing what it does, or why it is not moving',
 fluidnc:'the arm controller',
 connected:'telnet link up',
 reconnects:'times the link was lost since boot',
 state:'FluidNC state: Idle, Jog, Alarm, Home...',
 mpos:'tool position X, Y, Z, mm (machine coordinates)',
 target_u:'filtered target position in the image, -1..1 (null = none yet)',
 target_v:'',
 v_cmd:'forearm only: commanded velocity, mm/s',
 detection:'last camera frame',
 found:'target in this frame',
 u:'detection position in the image, -1..1',
 v:'',
 conf:'detection confidence',
 cells:'FOMO cells that fired',
 infer_ms:'model run time per frame, ms',
 sharp:'image sharpness (low = blurred)',
 cal_log:'last calibration / floor map message',
 fps:'frames analysed per second',
 jogs_sent:'jogs sent to FluidNC since boot',
 cancels:'jog-cancels sent since boot',
 backlog_mm:'distance FluidNC still has queued, mm',
 calibrated:'forearm only: image Jacobian measured',
 J:'forearm only: image Jacobian at calibration',
 phi0_deg:'forearm only: forearm angle at calibration',
 z0:'forearm only: Z at calibration, mm',
 phi_deg:'forearm only: forearm angle now',
 mirrored:'CAM_MIRRORED build setting',
 Jnow:'forearm only: Jacobian at the current pose',
 model:'Edge Impulse model in use',
 heap:'free RAM, bytes',
 psram:'free PSRAM, bytes',
 fixed:'1 = wall camera with floor map (CAMERA_FIXED)',
 lim:'soft limits: X min, X max, Y min, Y max, R min, R max, mm',
 map:'floor map: image point -> arm X/Y',
 n:'taught points',
 solved:'map fitted',
 rms:'average error of the taught points after the fit, mm',
 H:'homography (image -> floor)',
 pts:'taught points: u, v, x, y, error mm',
 mark:'image point waiting for Record',
 target_mm:'target on the floor through the map, mm',
 z_goal:'Z the tracker is taking the tool to (z_max with the light, z_min otherwise; null = leaves Z alone)'};
// Camera controls: [name, min, max, comment]
const CF=[["framesize",0,8,"image size: 5 = QVGA 320x240, 8 = VGA 640x480 (the model sees a centre square)"],
 ["quality",4,63,"JPEG quality, lower = better and bigger"],
 ["brightness",-2,2,""],["contrast",-2,2,""],["saturation",-2,2,""],
 ["ae_level",-2,2,"auto-exposure target (with aec = 1)"],
 ["aec",0,1,"1 = auto exposure"],["aec_value",0,1200,"manual exposure (with aec = 0)"],
 ["agc",0,1,"1 = auto gain"],["agc_gain",0,30,"manual gain (with agc = 0)"],
 ["awb",0,1,"1 = auto white balance"],["vflip",0,1,"flip vertically"],["hmirror",0,1,"mirror horizontally"]];
function camui(){fetch('/cam').then(r=>r.json()).then(c=>{const d=document.getElementById('camctl');d.innerHTML='';
 for(const[n,lo,hi,h]of CF){const e=document.createElement('div');
  e.innerHTML=`${n} <input type=range min=${lo} max=${hi} value=${c[n]} style="width:12em"> <span>${c[n]}</span> <small>${h}</small>`;
  const i=e.querySelector('input'),v=e.querySelector('span');
  i.oninput=()=>v.textContent=i.value;
  i.onchange=()=>fetch(`/camctl?var=${n}&val=${i.value}`).then(r=>r.text()).then(t=>document.getElementById('cammsg').textContent=t);
  d.appendChild(e);}});}
document.getElementById('camd').addEventListener('toggle',e=>{if(e.target.open)camui();});
function go(u){fetch(u).then(r=>r.text()).then(t=>console.log(t));}
function light(c){fetch('/light?c='+c.replace(' ','+')).then(r=>r.text()).then(t=>{document.getElementById('lights').textContent=t;});}
light('state');
function poll(){fetch('/status').then(r=>r.json()).then(s=>{S=s;
 for(const e of document.querySelectorAll('.fix'))e.style.display=s.fixed?'':'none';
 for(const e of document.querySelectorAll('.jac'))e.style.display=s.fixed?'none':'';
 if(s.fixed)fmtable();
 document.getElementById('mode').textContent=s.mode;
 document.getElementById('reason').textContent=s.reason;
 document.getElementById('autol').textContent=s.auto_lights?'auto: '+s.lamp:'auto off (manual)';
 const p=Object.assign({},s);delete p.params;
 document.getElementById('st').textContent=JSON.stringify(p,null,1).split('\n').map(l=>{const m=l.match(/^\s*"(\w+)":/);
  return m&&SC[m[1]]?l+'   // '+SC[m[1]]:l;}).join('\n');
 if(!document.getElementById('params').childElementCount){
  const d=document.getElementById('params');
  for(const k in s.params){const r=document.createElement('div');
   r.innerHTML=`${k} <input id="p_${k}" value="${s.params[k]}"> <button onclick="setp('${k}')">set</button> <small>${PC[k]||''}</small>`;d.appendChild(r);}
 }}).catch(()=>{}).finally(()=>setTimeout(poll,300));}
function f0(x){return x===null||x===undefined?'-':x.toFixed(0);}
function fmtable(){$('fm_msg').textContent=S.cal_log;const m=S.map;
 let h='<tr><th>#</th><th>image u, v</th><th>arm x, y mm</th><th>error mm</th><th></th></tr>';
 m.pts.forEach((p,i)=>{h+=`<tr><td>${i+1}</td><td>${p[0].toFixed(3)}, ${p[1].toFixed(3)}</td><td>${f0(p[2])}, ${f0(p[3])}</td>`+
  `<td>${m.solved?f0(p[4]):'-'}</td><td><button onclick="go('/delpoint?i=${i}')">delete</button></td></tr>`;});
 h+=`<tr><td colspan="5" style="text-align:left">${m.n} points. `+(m.solved?`Solved, rms ${m.rms.toFixed(0)} mm.`:'Not solved.')+'</td></tr>';
 const t=$('fm_pts');if(t.innerHTML!==h)t.innerHTML=h;}
function setp(k){go(`/set?${k}=${document.getElementById('p_'+k).value}`);}
function snap(){const im=new Image();im.onload=()=>{cv.width=im.width;cv.height=im.height;ctx.drawImage(im,0,0);
 const s=Math.min(im.width,im.height),ox=(im.width-s)/2,oy=(im.height-s)/2;
 ctx.strokeStyle='#0f0';ctx.setLineDash([4,4]);ctx.strokeRect(ox,oy,s,s);ctx.setLineDash([]);
 const cx=ox+s/2,cy=oy+s/2,db=(S.params?S.params.db_in:0)*s/2;
 if(!S.fixed){ctx.strokeStyle='#ff0';ctx.beginPath();ctx.arc(cx,cy,Math.max(db,2),0,7);ctx.stroke();}
 if(S.target_u!==null&&S.target_u!==undefined){const x=ox+(S.target_u+1)*s/2,y=oy+(S.target_v+1)*s/2;
  ctx.strokeStyle='#f0f';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x-8,y);ctx.lineTo(x+8,y);ctx.moveTo(x,y-8);ctx.lineTo(x,y+8);ctx.stroke();ctx.lineWidth=1;}
 G={cx:ox+s/2,cy:oy+s/2,ox,oy,s};
 if(S.fixed)drawmap(ox,oy,s);else axes(ox,oy,s);
 setTimeout(snap,400);};
 im.onerror=()=>setTimeout(snap,1000);im.src='/snapshot.jpg?'+Date.now();}
// Arm axes as the camera sees them now: columns of J(phi) = J0 R(phi0-phi).
// Solid = current (after rotation compensation), dashed = at calibration.
// Cyan = where the commanded move pushes the target; it should point at the centre.
function arrow(x0,y0,dx,dy,col,lbl,dash){const x1=x0+dx,y1=y0+dy,a=Math.atan2(dy,dx);
 ctx.strokeStyle=ctx.fillStyle=col;ctx.lineWidth=2;ctx.setLineDash(dash?[5,4]:[]);
 ctx.beginPath();ctx.moveTo(x0,y0);ctx.lineTo(x1,y1);ctx.stroke();ctx.setLineDash([]);
 ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x1-9*Math.cos(a-0.4),y1-9*Math.sin(a-0.4));
 ctx.lineTo(x1-9*Math.cos(a+0.4),y1-9*Math.sin(a+0.4));ctx.fill();
 if(lbl){ctx.font='bold 13px system-ui';ctx.fillText(lbl,x1+6*Math.cos(a)-5,y1+6*Math.sin(a)+5);}ctx.lineWidth=1;}
// Manual calibration. Angles in the UI are on-screen, counter-clockwise from
// "right"; the firmware takes th = atan2(dv, du) with v pointing down.
let M=null,G=null; // M: {a: deg, k: per 100 mm} while editing; G: last image geometry
const $=id=>document.getElementById(id);
function mshow(){$('m_a').value=M.a;$('m_av').textContent=M.a;if(document.activeElement!==$('m_k'))$('m_k').value=M.k;}
function mload(){M={a:0,k:0.15};if(S.calibrated&&S.Jnow){const u=S.Jnow[0][0],v=S.Jnow[1][0];
 M.a=Math.round(Math.atan2(-v,u)*57.2958);M.k=+(Math.hypot(u,v)*100).toFixed(4);}mshow();$('m_msg').textContent='editing';}
function medit(){const a=+$('m_a').value,k=parseFloat($('m_k').value);if(!M)mload();
 M.a=a;$('m_a').value=a;$('m_av').textContent=a;if(k>0)M.k=k;$('m_msg').textContent='editing';}
function mcancel(){M=null;$('m_msg').textContent='';}
function mapply(){if(!M)return;if(!(M.k>0)){$('m_msg').textContent='scale must be > 0';return;}
 fetch(`/setcal?th=${-M.a}&k=${(M.k/100).toPrecision(6)}`).then(r=>r.text().then(t=>{$('m_msg').textContent=t;if(r.ok)M=null;}))
 .catch(()=>{$('m_msg').textContent='no answer from the tracker';});}
cv.addEventListener('click',e=>{if(!G)return;const r=cv.getBoundingClientRect();
 const x=(e.clientX-r.left)*cv.width/r.width-G.cx,y=(e.clientY-r.top)*cv.height/r.height-G.cy;
 if(S.fixed){go(`/markat?u=${(2*x/G.s).toFixed(4)}&v=${(2*y/G.s).toFixed(4)}`);return;}
 if(Math.hypot(x,y)<5)return;if(!M)mload();M.a=Math.round(Math.atan2(-y,x)*57.2958);mshow();$('m_msg').textContent='editing';});
function preview(cx,cy,L){if(!M)return;const a=M.a/57.2958,ux=Math.cos(a),vx=-Math.sin(a);
 const uy=S.mirrored?-vx:vx,vy=S.mirrored?ux:-ux; // +Y: 90 deg counter-clockwise on screen (clockwise if mirrored)
 arrow(cx,cy,ux*L*1.15,vx*L*1.15,'#fff','X',true);arrow(cx,cy,uy*L*1.15,vy*L*1.15,'#ff0','Y',true);}
// ---- fixed camera: what the floor map says, drawn on the image
// H maps image -> arm; its inverse maps arm mm back onto the image.
function inv3(m){const[a,b,c]=m[0],[d,e,f]=m[1],[g,h,i]=m[2];
 const A=e*i-f*h,B=-(d*i-f*g),C=d*h-e*g,det=a*A+b*B+c*C;
 return [[A/det,-(b*i-c*h)/det,(b*f-c*e)/det],[B/det,(a*i-c*g)/det,-(a*f-c*d)/det],[C/det,-(a*h-b*g)/det,(a*e-b*d)/det]];}
function drawmap(ox,oy,s){const m=S.map,px=(u,v)=>[ox+(u+1)*s/2,oy+(v+1)*s/2];
 ctx.font='bold 12px system-ui';
 if(m.solved){const Hi=inv3(m.H);
  const raw=(x,y)=>[Hi[0][0]*x+Hi[0][1]*y+Hi[0][2],Hi[1][0]*x+Hi[1][1]*y+Hi[1][2],Hi[2][0]*x+Hi[2][1]*y+Hi[2][2]];
  const sg=Math.sign(raw(m.pts[0][2],m.pts[0][3])[2]); // points on the camera's side of the horizon share this sign
  const P=(x,y)=>{const[a,b,w]=raw(x,y);return w*sg>1e-9?px(a/w,b/w):null;};
  const path=(f,n,col,w)=>{ctx.strokeStyle=col;ctx.lineWidth=w||1;ctx.beginPath();let pen=false;
   for(let k=0;k<=n;k++){const q=f(k/n);if(!q){pen=false;continue;}if(pen)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);pen=true;}
   ctx.stroke();ctx.lineWidth=1;};
  const L=S.lim,[x0,x1,y0,y1,r0,r1]=L;
  for(let x=Math.ceil(x0/200)*200;x<=x1;x+=200){path(t=>P(x,y0+t*(y1-y0)),24,x?'#fff5':'#f66b');
   const q=P(x,y1);if(q){ctx.fillStyle='#fffa';ctx.fillText('x'+x,q[0]+2,q[1]-2);}}
  for(let y=Math.ceil(y0/200)*200;y<=y1;y+=200){path(t=>P(x0+t*(x1-x0),y),24,'#fff5');
   const q=P(x1,y);if(q){ctx.fillStyle='#fffa';ctx.fillText('y'+y,q[0]+2,q[1]-2);}}
  path(t=>{const k=t*4,e=Math.floor(Math.min(k,3.999)),f=k-e,c=[[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]];
   return P(c[e][0]+f*(c[e+1][0]-c[e][0]),c[e][1]+f*(c[e+1][1]-c[e][1]));},96,'#fa0',2);
  for(const r of[r0,r1])path(t=>{const a=t*Math.PI,x=r*Math.cos(a),y=r*Math.sin(a);
   return x>=x0&&x<=x1&&y>=y0&&y<=y1?P(x,y):null;},96,'#fa0',2);
  const T=P(S.fluidnc.mpos[0],S.fluidnc.mpos[1]),info=[`tool ${f0(S.fluidnc.mpos[0])}, ${f0(S.fluidnc.mpos[1])} mm`];
  if(T){ctx.strokeStyle='#f80';ctx.lineWidth=3;ctx.beginPath();ctx.arc(T[0],T[1],9,0,7);ctx.stroke();ctx.lineWidth=1;
   ctx.fillStyle='#f80';ctx.fillText('tool',T[0]+11,T[1]+4);}
  if(S.target_mm){const gx=S.target_mm[0]+S.params.off_x,gy=S.target_mm[1]+S.params.off_y,Q=P(gx,gy);
   info.push(`target ${f0(S.target_mm[0])}, ${f0(S.target_mm[1])} mm`);
   info.push(`to go ${f0(Math.hypot(gx-S.fluidnc.mpos[0],gy-S.fluidnc.mpos[1]))} mm`);
   if(T&&Q&&Math.hypot(Q[0]-T[0],Q[1]-T[1])>12)arrow(T[0],T[1],Q[0]-T[0],Q[1]-T[1],'#0ff','');}
  ctx.font='12px monospace';ctx.fillStyle='#000a';ctx.fillRect(ox+2,oy+2,190,16*info.length+4);
  ctx.fillStyle='#fff';info.forEach((t,i)=>ctx.fillText(t,ox+6,oy+16+16*i));ctx.font='bold 12px system-ui';}
 m.pts.forEach((p,i)=>{const[x,y]=px(p[0],p[1]);ctx.fillStyle='#4af';ctx.beginPath();ctx.arc(x,y,5,0,7);ctx.fill();
  ctx.fillText(`${i+1}`+(m.solved&&p[4]!==null?` ${p[4].toFixed(0)}mm`:''),x+7,y-5);});
 if(S.mark){const[x,y]=px(S.mark[0],S.mark[1]);ctx.strokeStyle='#0ff';ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,10,0,7);ctx.stroke();
  ctx.lineWidth=1;ctx.fillStyle='#0ff';ctx.fillText('record?',x+12,y+4);}}
function axes(ox,oy,s){preview(G.cx,G.cy,s*0.3);if(!S.calibrated||!S.Jnow)return;
 const cx=ox+s/2,cy=oy+s/2,L=s*0.3,info=[];
 for(const[k,col,n]of[[0,'#f44','X'],[1,'#4f4','Y']]){
  const g=S.J[0][k],h=S.J[1][k],u=S.Jnow[0][k],v=S.Jnow[1][k];
  const m0=Math.hypot(g,h)||1,m=Math.hypot(u,v)||1;
  arrow(cx,cy,g/m0*L,h/m0*L,col+'8','',true);
  arrow(cx,cy,u/m*L,v/m*L,col,'+'+n);
  info.push(`+${n}: ${(m*100).toFixed(2)}/100mm`);}
 if(S.phi_deg!==null){let d=S.phi_deg-S.phi0_deg;d=((d+540)%360)-180;
  info.unshift(`phi ${S.phi_deg.toFixed(1)}\u00b0  rot ${d>=0?'+':''}${d.toFixed(1)}\u00b0`);}
 const vx=S.v_cmd[0],vy=S.v_cmd[1];
 if(S.target_u!==null&&S.target_u!==undefined&&Math.hypot(vx,vy)>0.01){
  const iu=S.Jnow[0][0]*vx+S.Jnow[0][1]*vy,iv=S.Jnow[1][0]*vx+S.Jnow[1][1]*vy,im=Math.hypot(iu,iv);
  const x=ox+(S.target_u+1)*s/2,y=oy+(S.target_v+1)*s/2;
  arrow(x,y,iu/im*L*0.6,iv/im*L*0.6,'#0ff','');
  let e=(Math.atan2(iv,iu)-Math.atan2(-S.target_v,-S.target_u))*57.2958;e=((e+540)%360)-180;
  info.push(`move dir err ${e.toFixed(0)}\u00b0`);}
 ctx.font='12px monospace';ctx.fillStyle='#000a';ctx.fillRect(ox+2,oy+2,170,16*info.length+4);
 ctx.fillStyle='#fff';info.forEach((t,i)=>ctx.fillText(t,ox+6,oy+16+16*i));}
poll();snap();
</script></body></html>)HTML";

static esp_err_t h_index(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");  // a reflash must not leave an old page running
    return httpd_resp_send(req, DASH_HTML, HTTPD_RESP_USE_STRLEN);
}

// printf onto the end of b; n tracks the length (>= size means it overflowed)
static void jappend(char* b, size_t size, size_t& n, const char* fmt, ...) {
    if (n >= size) return;
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(b + n, size - n, fmt, ap);
    va_end(ap);
}

static esp_err_t h_status(httpd_req_t* req) {
    CtlStatus s = controller::status();
    Params    p = controller::params();
    Detection d = vision::latest();
    char tu[16], tv[16];
    if (isnan(s.fu)) strcpy(tu, "null"), strcpy(tv, "null");
    else snprintf(tu, sizeof(tu), "%.3f", s.fu), snprintf(tv, sizeof(tv), "%.3f", s.fv);
    char phi[16];
    if (isnan(s.phi)) strcpy(phi, "null");
    else snprintf(phi, sizeof(phi), "%.1f", s.phi * 57.2958f);
    static char b[4096];
    snprintf(b, sizeof(b),
             "{\"mode\":\"%s\",\"operator_off\":%s,\"human_off\":%s,\"auto_lights\":%s,\"lamp\":\"%s\",\"reason\":\"%s\",\"fluidnc\":{\"connected\":%s,\"reconnects\":%u,\"state\":\"%s\","
             "\"mpos\":[%.2f,%.2f,%.2f]},"
             "\"target_u\":%s,\"target_v\":%s,\"v_cmd\":[%.1f,%.1f],"
             "\"detection\":{\"found\":%s,\"u\":%.3f,\"v\":%.3f,\"conf\":%.2f,\"cells\":%u,\"infer_ms\":%u,\"sharp\":%.1f},"
             "\"cal_log\":\"%s\",\"fps\":%.1f,\"jogs_sent\":%u,\"cancels\":%u,\"backlog_mm\":%.0f,"
             "\"calibrated\":%s,\"J\":[[%.5f,%.5f],[%.5f,%.5f]],\"phi0_deg\":%.1f,\"z0\":%.1f,"
             "\"phi_deg\":%s,\"mirrored\":%d,\"Jnow\":[[%.5f,%.5f],[%.5f,%.5f]],"
             "\"model\":\"%s\",\"heap\":%u,\"psram\":%u,"
             "\"params\":{\"kp\":%.3f,\"vmax\":%.1f,\"amax\":%.1f,\"db_in\":%.3f,\"db_out\":%.3f,\"alpha\":%.2f,"
             "\"min_conf\":%.2f,\"lost_ms\":%.0f,\"horizon_s\":%.2f,\"cal_step\":%.1f,\"cal_feed\":%.0f,"
             "\"off_x\":%.0f,\"off_y\":%.0f,\"db_mm_in\":%.0f,\"db_mm_out\":%.0f,"
             "\"recover_feed\":%.0f,\"wander_s\":%.1f,\"wander_feed\":%.0f,"
             "\"center_x\":%.0f,\"center_y\":%.0f,\"light_s\":%.1f,\"orbit_r\":%.0f,\"orbit_feed\":%.0f,"
             "\"spiral_pitch\":%.0f,\"spiral_r_min\":%.0f,\"light_hold_s\":%.1f,\"fast_feed\":%.0f,"
             "\"z_max\":%.0f,\"z_min\":%.0f,\"z_up_feed\":%.0f,\"z_down_feed\":%.0f,\"wander_margin\":%.0f,\"z_chunk_s\":%.2f}",
             controller::modeName(s.mode), s.operator_off ? "true" : "false", s.human_off ? "true" : "false", s.auto_lights ? "true" : "false", s.lamp, s.reason, s.fnc_connected ? "true" : "false", (unsigned)s.fnc_reconnects, s.fnc_state, s.mpos[0],
             s.mpos[1], s.mpos[2], tu, tv, s.vx, s.vy, d.found ? "true" : "false", d.u, d.v, d.conf, d.cells,
             d.infer_ms, d.sharp, s.cal_log, s.fps, (unsigned)s.jogs_sent, (unsigned)s.cancels, s.backlog_mm, s.calibrated ? "true" : "false",
             s.cal.J[0][0], s.cal.J[0][1], s.cal.J[1][0], s.cal.J[1][1], s.cal.phi0 * 57.2958f, s.cal.z0,
             phi, CAM_MIRRORED, s.Jnow[0][0], s.Jnow[0][1], s.Jnow[1][0], s.Jnow[1][1],
             vision::model_info(), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(), p.kp, p.vmax, p.amax,
             p.db_in, p.db_out, p.alpha, p.min_conf, p.lost_ms, p.horizon_s, p.cal_step, p.cal_feed,
             p.off_x, p.off_y, p.db_mm_in, p.db_mm_out, p.recover_feed, p.wander_s, p.wander_feed, p.center_x, p.center_y, p.light_s, p.orbit_r, p.orbit_feed, p.spiral_pitch, p.spiral_r_min, p.light_hold_s, p.fast_feed, p.z_max, p.z_min, p.z_up_feed, p.z_down_feed, p.wander_margin, p.z_chunk_s);
    // fixed camera: taught points, floor map, soft limits (for drawing the map on the image)
    size_t n = strlen(b);
#define add(...) jappend(b, sizeof(b), n, __VA_ARGS__)
    const FloorMap& m = s.map;
    add(",\"fixed\":%d,\"lim\":[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]", CAMERA_FIXED, LIM_X_MIN, LIM_X_MAX, LIM_Y_MIN,
        LIM_Y_MAX, LIM_R_MIN, LIM_R_MAX);
    add(",\"map\":{\"n\":%d,\"solved\":%s,\"rms\":%.1f,\"H\":[", (int)m.n, m.solved ? "true" : "false",
        m.rms);
    for (int r = 0; r < 3; r++)
        add("%s[%.7g,%.7g,%.7g]", r ? "," : "", m.H[r][0], m.H[r][1], m.H[r][2]);
    add("],\"pts\":[");
    for (int i = 0; i < m.n; i++) {
        add("%s[%.4f,%.4f,%.1f,%.1f,", i ? "," : "", m.u[i], m.v[i], m.x[i], m.y[i]);
        if (m.solved && !isnan(m.err[i]) && !isinf(m.err[i])) add("%.1f]", m.err[i]);
        else add("null]");
    }
    add("]}");
    if (s.mark_set) add(",\"mark\":[%.4f,%.4f]", s.mark_u, s.mark_v);
    else add(",\"mark\":null");
    if (isnan(s.z_goal)) add(",\"z_goal\":null");
    else add(",\"z_goal\":%.0f", s.z_goal);
    if (isnan(s.target_x)) add(",\"target_mm\":null}");
    else add(",\"target_mm\":[%.1f,%.1f]}", s.target_x, s.target_y);
#undef add
    if (n >= sizeof(b)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "status too long");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, b);
}

static esp_err_t h_enable(httpd_req_t* req) {
    controller::requestEnable();
    return httpd_resp_sendstr(req, "enable requested - check /status");
}

static esp_err_t h_disable(httpd_req_t* req) {
    controller::requestDisable("stopped from web");
    return httpd_resp_sendstr(req, "stopped");
}

static esp_err_t h_calibrate(httpd_req_t* req) {
    controller::requestCalibrate();
    return httpd_resp_sendstr(req, "calibration requested - watch /status");
}

static esp_err_t h_mark(httpd_req_t* req) {
    controller::requestMark();
    return httpd_resp_sendstr(req, "marking - keep the head still");
}

static esp_err_t h_markat(httpd_req_t* req) {
    char q[64], u[16], v[16];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK || httpd_query_key_value(q, "u", u, sizeof(u)) != ESP_OK ||
        httpd_query_key_value(q, "v", v, sizeof(v)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "rejected - usage: /markat?u=0.1&v=-0.3");
        return ESP_FAIL;
    }
    controller::requestMarkAt(atof(u), atof(v));
    return httpd_resp_sendstr(req, "marked");
}

static esp_err_t h_record(httpd_req_t* req) {
    controller::requestRecord();
    return httpd_resp_sendstr(req, "record requested");
}

static esp_err_t h_delpoint(httpd_req_t* req) {
    char q[32], i[8];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK || httpd_query_key_value(q, "i", i, sizeof(i)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "rejected - usage: /delpoint?i=0");
        return ESP_FAIL;
    }
    controller::requestDeletePoint(atoi(i));
    return httpd_resp_sendstr(req, "delete requested");
}

static esp_err_t h_solvemap(httpd_req_t* req) {
    controller::requestSolveMap();
    return httpd_resp_sendstr(req, "solve requested");
}

static esp_err_t h_clearmap(httpd_req_t* req) {
    controller::requestClearMap();
    return httpd_resp_sendstr(req, "clear requested");
}

static esp_err_t h_clearcal(httpd_req_t* req) {
    controller::requestClearCal();
    return httpd_resp_sendstr(req, "clear requested - check status");
}

static esp_err_t h_setcal(httpd_req_t* req) {
    char q[128], th[24], k[24];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
        httpd_query_key_value(q, "th", th, sizeof(th)) != ESP_OK ||
        httpd_query_key_value(q, "k", k, sizeof(k)) != ESP_OK || !(atof(k) > 0)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "rejected - usage: /setcal?th=-90&k=0.0015");
        return ESP_FAIL;
    }
    controller::requestManualCal(atof(th), atof(k));
    return httpd_resp_sendstr(req, "calibration requested - check status");
}

static esp_err_t h_set(httpd_req_t* req) {
    char q[256];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "usage: /set?kp=1.0&vmax=50");
        return ESP_FAIL;
    }
    String out;
    char* save = nullptr;
    for (char* kv = strtok_r(q, "&", &save); kv; kv = strtok_r(nullptr, "&", &save)) {
        char* eq = strchr(kv, '=');
        if (!eq) continue;
        *eq = 0;
        out += kv;
        out += controller::setParam(kv, atof(eq + 1)) ? " ok\n" : " unknown\n";
    }
    return httpd_resp_sendstr(req, out.c_str());
}

static esp_err_t h_snapshot(httpd_req_t* req) {
    size_t   len;
    uint8_t* buf = vision::snapshot(&len);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no frame yet");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, (const char*)buf, len);
    free(buf);
    return r;
}

// Sends one command to the light controller and returns its reply ("spot 1 light 0").
static esp_err_t h_light(httpd_req_t* req) {
    char q[48], c[24];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK || httpd_query_key_value(q, "c", c, sizeof(c)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "usage: /light?c=spot+1");
        return ESP_FAIL;
    }
    for (char* p = c; *p; p++)
        if (*p == '+') *p = ' ';  // the page sends "spot+1"
    if (strcmp(c, "state")) controller::setAutoLights(false);  // a hand choice is not overwritten
    char buf[64];
    return httpd_resp_sendstr(req, lights::query(c, buf, sizeof(buf)) ? buf : "no reply from lights");
}

static esp_err_t h_cam(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, vision::cam_json().c_str());
}

// Change one camera setting now (not saved): /camctl?var=aec_value&val=300
static esp_err_t h_camctl(httpd_req_t* req) {
    char q[64], var[24], val[12];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
        httpd_query_key_value(q, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(q, "val", val, sizeof(val)) != ESP_OK || !vision::cam_set(var, atoi(val))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "usage: /camctl?var=<name>&val=<int>");
        return ESP_FAIL;
    }
    return httpd_resp_sendstr(req, "applied (not saved)");
}

static esp_err_t h_camsave(httpd_req_t* req) {
    return httpd_resp_sendstr(req, vision::cam_save() ? "camera settings saved" : "save failed");
}

static esp_err_t h_autolights(httpd_req_t* req) {
    char q[16], v[4] = "1";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) httpd_query_key_value(q, "on", v, sizeof(v));
    controller::setAutoLights(atoi(v) != 0);
    return httpd_resp_sendstr(req, atoi(v) ? "auto lights on" : "auto lights off");
}

#ifdef TRACKER_STUB
static esp_err_t h_fake(httpd_req_t* req) {
    char q[64], u[12] = "0", v[12] = "0", ttl[12] = "1000";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "u", u, sizeof(u));
        httpd_query_key_value(q, "v", v, sizeof(v));
        httpd_query_key_value(q, "ttl", ttl, sizeof(ttl));
    }
    vision::fake(atof(u), atof(v), atoi(ttl));
    return httpd_resp_sendstr(req, "fake target set");
}
#endif

void web_begin() {
    httpd_handle_t  h;
    httpd_config_t  c = HTTPD_DEFAULT_CONFIG();
    c.core_id         = 0;
    c.max_uri_handlers = 32;
    c.stack_size       = 8192;  // /status: a CtlStatus copy + a long snprintf overflow the default 4 KB
    if (httpd_start(&h, &c) != ESP_OK) {
        Serial.println("http server failed to start");
        return;
    }
    const httpd_uri_t uris[] = {
        {"/", HTTP_GET, h_index, nullptr},
        {"/status", HTTP_GET, h_status, nullptr},
        {"/enable", HTTP_GET, h_enable, nullptr},
        {"/disable", HTTP_GET, h_disable, nullptr},
        {"/stop", HTTP_GET, h_disable, nullptr},
        {"/calibrate", HTTP_GET, h_calibrate, nullptr},
        {"/set", HTTP_GET, h_set, nullptr},
        {"/setcal", HTTP_GET, h_setcal, nullptr},
        {"/clearcal", HTTP_GET, h_clearcal, nullptr},
        {"/mark", HTTP_GET, h_mark, nullptr},
        {"/markat", HTTP_GET, h_markat, nullptr},
        {"/record", HTTP_GET, h_record, nullptr},
        {"/delpoint", HTTP_GET, h_delpoint, nullptr},
        {"/solvemap", HTTP_GET, h_solvemap, nullptr},
        {"/clearmap", HTTP_GET, h_clearmap, nullptr},
        {"/snapshot.jpg", HTTP_GET, h_snapshot, nullptr},
        {"/light", HTTP_GET, h_light, nullptr},
        {"/autolights", HTTP_GET, h_autolights, nullptr},
        {"/cam", HTTP_GET, h_cam, nullptr},
        {"/camctl", HTTP_GET, h_camctl, nullptr},
        {"/camsave", HTTP_GET, h_camsave, nullptr},
#ifdef TRACKER_STUB
        {"/fake", HTTP_GET, h_fake, nullptr},
#endif
    };
    for (auto& u : uris) httpd_register_uri_handler(h, &u);
}
