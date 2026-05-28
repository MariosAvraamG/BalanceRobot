#pragma once
#include <pgmspace.h>

static const char HTML[] PROGMEM = R"html(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>BalanceBot Dashboard</title>
<style>
:root{
  --bg:#060915;--line:rgba(148,163,184,.18);--text:#e5eefc;--muted:#94a3b8;
  --cyan:#22d3ee;--blue:#60a5fa;--green:#34d399;--amber:#fbbf24;--red:#fb7185;--violet:#a78bfa;
  --shadow:0 24px 80px rgba(0,0,0,.45);--radius:22px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0;color:var(--text);
  font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;
  background:
    radial-gradient(circle at 18% 12%,rgba(34,211,238,.22),transparent 28%),
    radial-gradient(circle at 85% 0%,rgba(167,139,250,.22),transparent 30%),
    radial-gradient(circle at 80% 78%,rgba(52,211,153,.12),transparent 34%),
    linear-gradient(135deg,#020617 0%,#07111f 42%,#050816 100%);
  overflow-x:hidden;
}
body:before{
  content:"";position:fixed;inset:0;pointer-events:none;
  background-image:linear-gradient(rgba(255,255,255,.035) 1px,transparent 1px),
                   linear-gradient(90deg,rgba(255,255,255,.035) 1px,transparent 1px);
  background-size:42px 42px;
  mask-image:linear-gradient(to bottom,rgba(0,0,0,.9),transparent 82%);
}
.shell{width:min(1480px,calc(100% - 32px));margin:0 auto;padding:22px 0 32px}
/* TOPBAR */
.topbar{display:flex;align-items:center;justify-content:space-between;gap:16px;margin-bottom:18px;flex-wrap:wrap}
.brand{display:flex;align-items:center;gap:14px}
.logo{
  width:50px;height:50px;border-radius:16px;display:grid;place-items:center;
  background:linear-gradient(135deg,rgba(34,211,238,.95),rgba(96,165,250,.55) 55%,rgba(167,139,250,.72));
  box-shadow:0 0 40px rgba(34,211,238,.35);color:#02111c;font-weight:900;font-size:24px;
}
h1{margin:0;font-size:clamp(20px,2.6vw,34px);letter-spacing:-.04em}
.subtitle{margin-top:3px;color:var(--muted);font-size:13px}
.pills{display:flex;flex-wrap:wrap;justify-content:flex-end;gap:9px}
.pill{
  display:inline-flex;align-items:center;gap:7px;padding:9px 13px;
  border:1px solid var(--line);border-radius:999px;
  background:rgba(15,23,42,.62);backdrop-filter:blur(16px);
  color:var(--muted);font-size:12px;box-shadow:0 10px 30px rgba(0,0,0,.18);
}
.dot{width:8px;height:8px;border-radius:50%;background:var(--amber)}
.dot.on{background:var(--green);box-shadow:0 0 10px var(--green)}
.dot.off{background:var(--red);box-shadow:0 0 10px var(--red)}
/* LAYOUT */
.grid{display:grid;grid-template-columns:minmax(270px,.68fr) minmax(380px,1.3fr) minmax(270px,.82fr);gap:14px;align-items:start}
.card{
  position:relative;overflow:hidden;border:1px solid var(--line);border-radius:var(--radius);
  background:linear-gradient(180deg,rgba(15,23,42,.78),rgba(15,23,42,.52));
  box-shadow:var(--shadow);backdrop-filter:blur(20px);
}
.card:before{
  content:"";position:absolute;inset:0 0 auto 0;height:1px;
  background:linear-gradient(90deg,transparent,rgba(34,211,238,.55),transparent);
}
.cp{padding:18px}
.sec{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:13px}
.sec strong{font-size:13px;letter-spacing:.08em;text-transform:uppercase;color:#cbd5e1}
.mini{font-size:11px;color:var(--muted)}
/* DPAD */
.dpad{display:grid;grid-template-columns:repeat(3,68px);grid-template-rows:repeat(3,68px);gap:7px;justify-content:center;margin:6px 0 10px}
.db{
  border:1px solid rgba(148,163,184,.2);border-radius:18px;font-size:20px;
  background:linear-gradient(180deg,rgba(15,23,42,.85),rgba(2,6,23,.7));
  color:var(--text);cursor:pointer;user-select:none;-webkit-user-select:none;
  transition:transform .1s,background .1s,border-color .1s;touch-action:none;
}
.db span{display:block;font-size:10px;color:var(--muted);font-weight:700;letter-spacing:.1em;margin-top:2px}
.db:active,.db.lit{background:rgba(34,211,238,.26);border-color:var(--cyan);transform:scale(.94)}
.db.stp{color:#fecdd3;border-color:rgba(251,113,133,.4);background:linear-gradient(180deg,rgba(251,113,133,.18),rgba(2,6,23,.65))}
/* STATS */
.stats{display:grid;grid-template-columns:1fr 1fr;gap:9px}
.stat{padding:13px;border-radius:18px;background:rgba(2,6,23,.46);border:1px solid var(--line)}
.stat .lbl{color:var(--muted);font-size:10px;margin-bottom:5px;text-transform:uppercase;letter-spacing:.06em}
.stat .val{font-size:19px;font-weight:900;letter-spacing:-.04em}
/* BAR */
.barwrap{height:8px;background:rgba(148,163,184,.15);border-radius:999px;overflow:hidden;margin-top:8px}
.barfill{display:block;height:100%;width:0;border-radius:999px;transition:width .8s,background .8s}
/* TELE TABLE */
.tele{display:grid;grid-template-columns:1fr 1fr;gap:3px 10px;font-size:11px}
.tele .k{color:var(--muted)}
.tele .v{color:var(--cyan);text-align:right;font-variant-numeric:tabular-nums}
/* BATTERY */
.bat-hero{display:flex;align-items:flex-end;gap:14px;margin-bottom:11px}
.bat-soc{font-size:38px;font-weight:900;letter-spacing:-.05em;line-height:1}
.bat-bar-wrap{flex:1}
.bat-bigbar{height:13px;background:rgba(148,163,184,.15);border-radius:8px;overflow:hidden;margin-bottom:5px}
.bat-tiles{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.tile{padding:9px 11px;border-radius:14px;background:rgba(2,6,23,.46);border:1px solid var(--line)}
.tile .tlbl{color:var(--muted);font-size:10px;text-transform:uppercase;letter-spacing:.06em;margin-bottom:3px}
.tile .tval{font-size:15px;font-weight:700;color:var(--cyan)}
/* BUTTONS */
.btn{
  border:1px solid rgba(34,211,238,.25);border-radius:14px;padding:10px 14px;
  color:var(--text);font-weight:700;font-size:12px;cursor:pointer;
  background:linear-gradient(135deg,rgba(34,211,238,.2),rgba(96,165,250,.12));
  box-shadow:0 10px 28px rgba(0,0,0,.18);
  transition:transform .13s,border-color .13s,background .13s;
}
.btn:hover{transform:translateY(-1px);border-color:rgba(34,211,238,.55)}
.btn:active{transform:scale(.97);background:rgba(34,211,238,.3)}
.btn:disabled{opacity:.45;cursor:default}
/* CANVAS */
#viz{display:block;width:100%;background:rgba(2,6,23,.55);border-radius:16px;border:1px solid rgba(148,163,184,.13)}
/* SLIDERS */
.controls{display:grid;gap:9px}
.srow{display:grid;grid-template-columns:80px 1fr 58px;gap:9px;align-items:center;padding:9px 11px;border-radius:14px;background:rgba(2,6,23,.35);border:1px solid rgba(148,163,184,.11)}
.srow label{font-size:11px;color:var(--muted);font-weight:700;text-transform:uppercase;letter-spacing:.07em}
input[type=range]{width:100%;accent-color:var(--cyan)}
.rdout{font-variant-numeric:tabular-nums;color:#a5f3fc;text-align:right;font-size:11px}
/* RESPONSIVE */
@media(max-width:1100px){.grid{grid-template-columns:1fr 1fr}.ctr{grid-column:1/-1;grid-row:1}}
@media(max-width:680px){.grid{grid-template-columns:1fr}.stats{grid-template-columns:1fr}.dpad{grid-template-columns:repeat(3,60px);grid-template-rows:repeat(3,60px)}}
</style>
</head><body>
<div class="shell">

  <header class="topbar">
    <div class="brand">
      <div class="logo">B</div>
      <div><h1>BalanceBot</h1><div class="subtitle">Balance control &middot; PID tuning &middot; yaw &middot; power telemetry</div></div>
    </div>
    <div class="pills">
      <div class="pill"><span id="cDot" class="dot"></span><span id="cTxt">Connecting</span></div>
      <div class="pill">IMU&nbsp;<strong id="pImu" style="color:var(--muted)">—</strong></div>
      <div class="pill">Cal&nbsp;<strong id="pCal">—</strong></div>
      <div class="pill">State&nbsp;<strong id="pState" style="color:var(--muted)">—</strong></div>
    </div>
  </header>

  <main class="grid">

    <!-- ── LEFT: DRIVE ─────────────────────────────── -->
    <aside class="card cp">
      <div class="sec"><strong>Drive</strong><span class="mini">WASD &middot; diagonals &middot; Space stop</span></div>
      <div class="dpad">
        <button class="db" id="bWA" onpointerdown="startDiag('wa')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8598;<span>WA</span></button>
        <button class="db" id="bW"  onpointerdown="startMove('w')"  onpointerup="stopMove('w')"  onpointerleave="stopMove('w')">&#9650;<span>W</span></button>
        <button class="db" id="bWD" onpointerdown="startDiag('wd')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8599;<span>WD</span></button>
        <button class="db" id="bA"  onpointerdown="startMove('a')"  onpointerup="stopMove('a')"  onpointerleave="stopMove('a')">&#9664;<span>A</span></button>
        <button class="db stp" onclick="sendStop()">&#9632;<span>STOP</span></button>
        <button class="db" id="bD"  onpointerdown="startMove('d')"  onpointerup="stopMove('d')"  onpointerleave="stopMove('d')">&#9654;<span>D</span></button>
        <button class="db" id="bSA" onpointerdown="startDiag('sa')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8601;<span>SA</span></button>
        <button class="db" id="bS"  onpointerdown="startMove('s')"  onpointerup="stopMove('s')"  onpointerleave="stopMove('s')">&#9660;<span>S</span></button>
        <button class="db" id="bSD" onpointerdown="startDiag('sd')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8600;<span>SD</span></button>
      </div>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:9px;margin-bottom:16px">
        <button class="btn" id="calBtn" onclick="doCalibrate()">Calibrate</button>
        <button class="btn" onclick="sendStop()">Full Stop</button>
      </div>
      <div class="sec"><strong>Velocity</strong><span class="mini" id="velDisp">0 / 0 cm/s</span></div>
      <div class="tele">
        <div class="k">velTarget</div><div class="v" id="t_vt">—</div>
        <div class="k">velEst</div>   <div class="v" id="t_ve">—</div>
        <div class="k">tiltSP</div>   <div class="v" id="t_ts">—</div>
        <div class="k">vIntegral</div><div class="v" id="t_vi">—</div>
        <div class="k">yaw &omega;</div>  <div class="v" id="t_yr">—</div>
        <div class="k">yawCorr</div>  <div class="v" id="t_yc">—</div>
        <div class="k">turnBias</div> <div class="v" id="t_yt">—</div>
      </div>
    </aside>

    <!-- ── CENTER: VISUALIZER + KEY STATS ──────────── -->
    <section class="card ctr cp">
      <div class="sec"><strong>Robot Visualizer</strong><span class="mini">live tilt &middot; setpoint</span></div>
      <canvas id="viz" width="580" height="290"></canvas>
      <div class="stats" style="margin-top:13px">
        <div class="stat"><div class="lbl">Theta</div>    <div class="val" id="s_th">—</div></div>
        <div class="stat"><div class="lbl">Setpoint</div> <div class="val" id="s_sp">—</div></div>
        <div class="stat"><div class="lbl">Gyro rate</div><div class="val" id="s_gy">—</div></div>
        <div class="stat"><div class="lbl">Motor spd</div><div class="val" id="s_ms">—</div></div>
      </div>
    </section>

    <!-- ── RIGHT: BATTERY + SYSTEM ─────────────────── -->
    <aside class="card cp">
      <div class="sec"><strong>Power System</strong><span class="mini">NiMH 14.4V &middot; 2Ah</span></div>
      <div class="bat-hero">
        <div>
          <div class="mini" style="margin-bottom:3px">State of Charge</div>
          <div class="bat-soc" id="b_soc">—%</div>
        </div>
        <div class="bat-bar-wrap">
          <div class="bat-bigbar"><span class="barfill" id="b_bar"></span></div>
          <div class="mini" id="b_trem">— remaining</div>
        </div>
      </div>
      <div class="bat-tiles">
        <div class="tile"><div class="tlbl">Voltage</div>    <div class="tval" id="b_v">—</div></div>
        <div class="tile"><div class="tlbl">Power</div>      <div class="tval" id="b_p">—</div></div>
        <div class="tile"><div class="tlbl">Motor I</div>    <div class="tval" id="b_im">—</div></div>
        <div class="tile"><div class="tlbl">Logic I</div>    <div class="tval" id="b_il">—</div></div>
        <div class="tile"><div class="tlbl">Energy left</div><div class="tval" id="b_e">—</div></div>
        <div class="tile"><div class="tlbl">IMU errors</div> <div class="tval" id="b_ie">—</div></div>
      </div>
      <div class="sec" style="margin-top:16px"><strong>System</strong></div>
      <div class="tele">
        <div class="k">IMU</div>     <div class="v" id="s_imu">—</div>
        <div class="k">Cal age</div> <div class="v" id="s_cal">—</div>
        <div class="k">Bias Y</div>  <div class="v" id="s_by">—</div>
        <div class="k">Bias Z</div>  <div class="v" id="s_bz">—</div>
        <div class="k">CF coeff</div><div class="v" id="s_cf">—</div>
        <div class="k">err count</div><div class="v" id="s_ec">—</div>
      </div>
    </aside>

    <!-- ── TUNING ROW ────────────────────────────────── -->
    <section class="card cp">
      <div class="sec"><strong>Inner PID</strong><span class="mini">balance controller</span></div>
      <div class="controls">
        <div class="srow"><label>Kp</label>      <input id="kp" type="range" min="0"    max="5000"  step="10"    oninput="send('kp',this.value,1)">      <span id="kp_v"  class="rdout">—</span></div>
        <div class="srow"><label>Kd</label>      <input id="kd" type="range" min="0"    max="1000"  step="1"     oninput="send('kd',this.value,1)">      <span id="kd_v"  class="rdout">—</span></div>
        <div class="srow"><label>Ki</label>      <input id="ki" type="range" min="0"    max="20"    step="0.1"   oninput="send('ki',this.value,3)">      <span id="ki_v"  class="rdout">—</span></div>
        <div class="srow"><label>Setpoint</label><input id="sp" type="range" min="-0.3" max="0.3"   step="0.001" oninput="send('sp',this.value,4)">      <span id="sp_v"  class="rdout">—</span></div>
        <div class="srow"><label>CF &alpha;</label><input id="cf" type="range" min="0.9" max="0.999" step="0.001" oninput="send('cf',this.value,3)">    <span id="cf_v"  class="rdout">—</span></div>
      </div>
    </section>

    <section class="card cp">
      <div class="sec"><strong>Drive &amp; Motor</strong><span class="mini">velocity outer loop</span></div>
      <div class="controls">
        <div class="srow"><label>Kp vel</label>   <input id="kpv" type="range" min="0"    max="0.05"  step="0.001"  oninput="send('kpv',this.value,4)">  <span id="kpv_v" class="rdout">—</span></div>
        <div class="srow"><label>Ki vel</label>   <input id="kvi" type="range" min="0"    max="0.01"  step="0.0001" oninput="send('kvi',this.value,5)">  <span id="kvi_v" class="rdout">—</span></div>
        <div class="srow"><label>Max tiltSP</label><input id="mts" type="range" min="0.005" max="0.3" step="0.005"  oninput="send('mts',this.value,3)">  <span id="mts_v" class="rdout">—</span></div>
        <div class="srow"><label>EMA &alpha;</label><input id="ema" type="range" min="0.01" max="1"   step="0.01"   oninput="send('ema',this.value,2)"> <span id="ema_v" class="rdout">—</span></div>
        <div class="srow"><label>Accel</label>    <input id="ac"  type="range" min="10"   max="3000"  step="10"     oninput="send('ac',this.value,0)">   <span id="ac_v"  class="rdout">—</span></div>
        <div class="srow"><label>Max spd</label>  <input id="mw"  type="range" min="3"    max="120"   step="1"      oninput="send('mw',this.value,1)">   <span id="mw_v"  class="rdout">—</span></div>
      </div>
    </section>

    <section class="card cp">
      <div class="sec"><strong>Yaw Control</strong><span class="mini">heading PID</span></div>
      <div class="controls">
        <div class="srow"><label>Kp yaw</label>  <input id="kyp"  type="range" min="0"   max="5"    step="0.01"  oninput="send('kyp',this.value,3)">   <span id="kyp_v"  class="rdout">—</span></div>
        <div class="srow"><label>Ki yaw</label>  <input id="kiy"  type="range" min="0"   max="0.5"  step="0.005" oninput="send('kiy',this.value,4)">   <span id="kiy_v"  class="rdout">—</span></div>
        <div class="srow"><label>Kd yaw</label>  <input id="kdy"  type="range" min="0"   max="2"    step="0.01"  oninput="send('kdy',this.value,3)">   <span id="kdy_v"  class="rdout">—</span></div>
        <div class="srow"><label>Yaw EMA</label> <input id="yea"  type="range" min="0.01" max="1"   step="0.01"  oninput="send('yea',this.value,2)">   <span id="yea_v"  class="rdout">—</span></div>
        <div class="srow"><label>Turn step</label><input id="trns" type="range" min="0.1" max="5"   step="0.1"   oninput="send('trns',this.value,1)">  <span id="trns_v" class="rdout">—</span></div>
        <div class="srow"><label>Max turn</label><input id="mtb"  type="range" min="0.5" max="10"   step="0.5"   oninput="send('mtb',this.value,1)">   <span id="mtb_v"  class="rdout">—</span></div>
      </div>
    </section>

  </main>
</div>

<script>
(function(){
"use strict";
var moveIv=null;

function sm(dir){fetch('/move?dir='+encodeURIComponent(dir)).catch(function(){});}

function startMove(dir){
  clearInterval(moveIv);
  sm(dir);
  moveIv=setInterval(function(){sm(dir);},120);
}
function stopMove(dir){
  clearInterval(moveIv);moveIv=null;
  sm(dir==='a'||dir==='d'?'stop_turn':'stop_fb');
}
function startDiag(dir){
  clearInterval(moveIv);
  sm(dir);
  moveIv=setInterval(function(){sm(dir);},120);
}
function stopDiag(){
  clearInterval(moveIv);moveIv=null;
  sm('stop_fb');sm('stop_turn');
}
function sendStop(){
  clearInterval(moveIv);moveIv=null;
  sm('stop');
}
window.startMove=startMove;window.stopMove=stopMove;
window.startDiag=startDiag;window.stopDiag=stopDiag;
window.sendStop=sendStop;window.sendMove=sm;

// keyboard
var kdown={};
var kmap={w:'w',a:'a',s:'s',d:'d',arrowup:'w',arrowleft:'a',arrowdown:'s',arrowright:'d'};
window.addEventListener('keydown',function(e){
  var k=e.key.toLowerCase();
  if(k===' '){e.preventDefault();sendStop();return;}
  var dir=kmap[k];
  if(!dir||kdown[k])return;
  kdown[k]=true;e.preventDefault();startMove(dir);
});
window.addEventListener('keyup',function(e){
  var k=e.key.toLowerCase();
  var dir=kmap[k];
  if(!dir)return;
  delete kdown[k];e.preventDefault();stopMove(dir);
});

// sliders
function send(p,v,d){
  var el=document.getElementById(p+'_v');
  if(el)el.textContent=parseFloat(v).toFixed(d);
  fetch('/set?'+encodeURIComponent(p)+'='+encodeURIComponent(v)).catch(function(){});
}
window.send=send;

// calibrate
function doCalibrate(){
  var b=document.getElementById('calBtn');
  b.textContent='Calibrating…';b.disabled=true;
  fetch('/calibrate').then(function(r){return r.json();}).then(function(d){
    if(d.sp!==undefined){
      document.getElementById('sp').value=d.sp;
      document.getElementById('sp_v').textContent=parseFloat(d.sp).toFixed(4);
    }
    b.textContent='Calibrate';b.disabled=false;
  }).catch(function(){b.textContent='Calibrate';b.disabled=false;});
}
window.doCalibrate=doCalibrate;

// canvas
var viz=document.getElementById('viz');
var ctx=viz.getContext('2d');

function drawRobot(theta,setpt){
  var W=viz.width,H=viz.height;
  ctx.clearRect(0,0,W,H);

  var grd=ctx.createLinearGradient(0,0,W,H);
  grd.addColorStop(0,'rgba(34,211,238,.07)');
  grd.addColorStop(1,'rgba(167,139,250,.07)');
  ctx.fillStyle=grd;ctx.fillRect(0,0,W,H);

  ctx.strokeStyle='rgba(148,163,184,.12)';ctx.lineWidth=1;
  for(var x=20;x<W;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();}
  for(var y=20;y<H;y+=40){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}

  var cx=W/2,gy=H*0.76,wr=30,bl=150;

  ctx.strokeStyle='rgba(148,163,184,.32)';ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(36,gy+wr);ctx.lineTo(W-36,gy+wr);ctx.stroke();

  ctx.fillStyle='rgba(0,0,0,.22)';
  ctx.beginPath();ctx.ellipse(cx,gy+wr+3,wr*1.35,5,0,0,Math.PI*2);ctx.fill();

  ctx.strokeStyle='rgba(96,165,250,.65)';ctx.lineWidth=5;
  ctx.beginPath();ctx.arc(cx-20,gy,wr,0,Math.PI*2);ctx.stroke();
  ctx.beginPath();ctx.arc(cx+20,gy,wr,0,Math.PI*2);ctx.stroke();
  ctx.fillStyle='rgba(96,165,250,.5)';
  ctx.beginPath();ctx.arc(cx-20,gy,5,0,Math.PI*2);ctx.fill();
  ctx.beginPath();ctx.arc(cx+20,gy,5,0,Math.PI*2);ctx.fill();

  if(setpt!==undefined&&setpt!==null){
    var sx=cx+Math.sin(setpt)*bl,sy=gy-Math.cos(setpt)*bl;
    ctx.strokeStyle='rgba(34,211,238,.3)';ctx.lineWidth=2;ctx.setLineDash([6,5]);
    ctx.beginPath();ctx.moveTo(cx,gy);ctx.lineTo(sx,sy);ctx.stroke();
    ctx.setLineDash([]);
  }

  var danger=Math.abs(theta);
  var col=danger<0.15?'#34d399':danger<0.35?'#fbbf24':'#fb7185';
  var bx=cx+Math.sin(theta)*bl,by=gy-Math.cos(theta)*bl;

  ctx.strokeStyle=col;ctx.lineWidth=12;ctx.lineCap='round';
  ctx.shadowColor=col;ctx.shadowBlur=18;
  ctx.beginPath();ctx.moveTo(cx,gy);ctx.lineTo(bx,by);ctx.stroke();
  ctx.shadowBlur=0;

  ctx.fillStyle=col;
  ctx.beginPath();ctx.arc(bx,by,13,0,Math.PI*2);ctx.fill();
  ctx.fillStyle='rgba(255,255,255,.3)';
  ctx.beginPath();ctx.arc(bx-3,by-3,4,0,Math.PI*2);ctx.fill();

  ctx.fillStyle='#e5eefc';ctx.font='700 20px ui-monospace,monospace';ctx.textAlign='right';
  ctx.fillText((theta*180/Math.PI).toFixed(2)+'°',W-16,H-16);
  ctx.fillStyle='rgba(148,163,184,.75)';ctx.font='10px ui-monospace,monospace';
  ctx.fillText('tilt',W-16,H-34);ctx.textAlign='left';

  if(Math.abs(theta)>0.39){
    ctx.fillStyle='rgba(251,113,133,.13)';ctx.fillRect(0,0,W,H);
    ctx.fillStyle='#fb7185';ctx.font='700 26px Inter,sans-serif';
    ctx.textAlign='center';ctx.fillText('FALLEN',W/2,H/2);ctx.textAlign='left';
  }
}
drawRobot(0,0.06);

function fx(id,v){var el=document.getElementById(id);if(el)el.textContent=v;}
function fmt(v,d,s){var n=parseFloat(v);return isNaN(n)?'—':n.toFixed(d)+(s||'');}

// slider init (once on first good poll)
var sinited=false;
var SP=[['kp',1],['kd',1],['ki',3],['sp',4],['cf',3],
        ['kpv',4],['kvi',5],['mts',3],['ema',2],['ac',0],['mw',1],
        ['kyp',3],['kiy',4],['kdy',3],['yea',2],['trns',1],['mtb',1]];

function initSliders(d){
  if(sinited)return;
  var ok=false;
  SP.forEach(function(p){
    if(d[p[0]]===undefined)return;
    var el=document.getElementById(p[0]);
    if(!el)return;
    el.value=d[p[0]];
    fx(p[0]+'_v',parseFloat(d[p[0]]).toFixed(p[1]));
    ok=true;
  });
  if(ok)sinited=true;
}

function poll(){
  fetch('/status',{cache:'no-store'}).then(function(r){
    if(!r.ok)throw new Error('');
    return r.json();
  }).then(function(d){
    var dot=document.getElementById('cDot'),txt=document.getElementById('cTxt');
    dot.className='dot on';txt.textContent='Online';

    var iok=!!d.imu_ok;
    var pImu=document.getElementById('pImu');
    pImu.textContent=iok?'OK':'ERR';pImu.style.color=iok?'var(--green)':'var(--red)';
    fx('pCal',d.cal_s!==undefined?d.cal_s+'s ago':'—');
    var fallen=d.theta!==undefined&&Math.abs(d.theta)>0.39;
    var ps=document.getElementById('pState');
    ps.textContent=fallen?'FALLEN':'Balancing';ps.style.color=fallen?'var(--red)':'var(--green)';

    drawRobot(d.theta||0,d.setpt);

    fx('s_th',fmt(d.theta,4,' rad'));
    fx('s_sp',fmt(d.setpt,4,' rad'));
    fx('s_gy',fmt(d.gyro,3,' r/s'));
    fx('s_ms',fmt(d.spd,2,' r/s'));

    fx('t_vt',fmt((d.velTarget||0)*3,2,' cm/s'));
    fx('t_ve',fmt((d.velEst||0)*3,2,' cm/s'));
    fx('t_ts',fmt(d.tiltSP,4,' rad'));
    fx('t_vi',fmt(d.vint,4,''));
    fx('t_yr',fmt(d.yaw_rate,4,' r/s'));
    fx('t_yc',fmt(d.yawCorr,4,' r/s'));
    fx('t_yt',fmt(d.turnBias,3,' r/s'));
    fx('velDisp',fmt((d.velTarget||0)*3,1,'')+' / '+fmt((d.velEst||0)*3,1,' cm/s'));

    var imuEl=document.getElementById('s_imu');
    imuEl.textContent=iok?'OK':'ERROR';imuEl.style.color=iok?'var(--green)':'var(--red)';
    fx('s_cal',d.cal_s!==undefined?d.cal_s+'s ago':'—');
    fx('s_by',fmt(d.bias,4,' r/s'));
    fx('s_bz',fmt(d.biasZ,4,' r/s'));
    fx('s_cf',fmt(d.cf,3,''));
    fx('s_ec',d.imu_err||0);

    if(d.soc!==undefined){
      var soc=d.soc;
      var fill=document.getElementById('b_bar');
      fx('b_soc',soc.toFixed(1)+'%');
      fill.style.width=Math.max(0,Math.min(100,soc))+'%';
      fill.style.background=soc>50?'linear-gradient(90deg,#34d399,#22d3ee)':soc>20?'linear-gradient(90deg,#fbbf24,#f59e0b)':'linear-gradient(90deg,#fb7185,#ef4444)';
      fx('b_trem',d.trem>900?'∞ remaining':d.trem.toFixed(0)+' min remaining');
      fx('b_v',fmt(d.vbat,2,' V'));
      fx('b_p',fmt(d.power,2,' W'));
      fx('b_im',fmt(d.imotor,3,' A'));
      fx('b_il',fmt(d.ilogic,3,' A'));
      fx('b_e',fmt(d.energy,2,' Wh'));
    }
    fx('b_ie',d.imu_err||0);

    initSliders(d);
  }).catch(function(){
    var dot=document.getElementById('cDot'),txt=document.getElementById('cTxt');
    dot.className='dot off';txt.textContent='Offline';
  });
}

setInterval(poll,500);poll();
})();
</script>
</body></html>
)html";
