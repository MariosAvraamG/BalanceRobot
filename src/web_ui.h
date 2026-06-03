#pragma once
#include <pgmspace.h>

static const char HTML[] PROGMEM = R"html(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceBot Tuner</title>
<style>
*{box-sizing:border-box}
html,body{height:100%;margin:0}
body{font-family:monospace;padding:6px 10px;background:#111;color:#ddd;overflow:hidden}
h2{color:#4af;margin:0 0 6px;font-size:15px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;height:calc(100vh - 34px)}
.col{display:flex;flex-direction:column;gap:6px;overflow-y:auto}
.card{background:#1e1e1e;border-radius:6px;padding:6px 10px}
.card b{color:#4af;font-size:11px;letter-spacing:.05em}
.row{display:flex;align-items:center;margin:4px 0;gap:6px}
label{width:66px;font-size:11px;color:#999;white-space:nowrap}
input[type=range]{flex:1;accent-color:#4af;height:14px}
.val{width:56px;text-align:right;font-size:11px;color:#4af}
.tele{display:grid;grid-template-columns:1fr 1fr;gap:2px 12px;margin-top:4px;font-size:11px}
.tele .k{color:#777}
.tele .v{color:#4af}
.dpad{display:grid;grid-template-columns:repeat(3,48px);grid-template-rows:repeat(3,48px);gap:4px;justify-content:center;margin:6px 0}
.dbtn{background:#1a1a2e;border:2px solid #333;border-radius:6px;color:#4af;font-size:18px;cursor:pointer;width:48px;height:48px;touch-action:none;user-select:none;-webkit-user-select:none}
.dbtn:active{background:#4af;color:#111;border-color:#4af}
.dbtn.stop{color:#f55;border-color:#555}
.dbtn.stop:active{background:#f55;color:#111;border-color:#f55}
.cbtn{width:100%;margin-top:6px;padding:6px;background:#1a1a2e;border:1px solid #4af;border-radius:4px;color:#4af;font-family:monospace;font-size:11px;cursor:pointer}
.cbtn:active{background:#4af;color:#111}
.cbtn:disabled{opacity:.5;cursor:default}
.bat-wrap{background:#333;border-radius:3px;height:8px;margin:4px 0 2px;overflow:hidden}
.bat-bar{height:100%;width:0%;background:#4f4;border-radius:3px;transition:width .5s,background .5s}
</style></head><body>
<h2>BalanceBot Tuner</h2>
<div class="grid">
  <div class="col">
    <div class="card">
      <b>TELEMETRY</b>
      <div class="tele">
        <div><span class="k">meas θ </span><span class="v" id="t_th">—</span></div>
        <div><span class="k">want θ </span><span class="v" id="t_st">—</span></div>
        <div><span class="k">gyro   </span><span class="v" id="t_gy">—</span></div>
        <div><span class="k">error  </span><span class="v" id="t_er">—</span></div>
        <div><span class="k">motor  </span><span class="v" id="t_sp">—</span></div>
        <div><span class="k">tiltSP </span><span class="v" id="t_ts">—</span></div>
        <div><span class="k">velEst </span><span class="v" id="t_ve">—</span></div>
        <div><span class="k">velTgt </span><span class="v" id="t_vt">—</span></div>
        <div><span class="k">vInteg </span><span class="v" id="t_vi">—</span></div>
        <div><span class="k">yaw ω  </span><span class="v" id="t_yr">—</span></div>
        <div><span class="k">yawCorr</span><span class="v" id="t_yc">—</span></div>
        <div><span class="k">yawTgt </span><span class="v" id="t_yt">—</span></div>
      </div>
    </div>
    <div class="card">
      <b>SYSTEM</b>
      <div class="tele">
        <div><span class="k">IMU </span><span class="v" id="s_imu">—</span></div>
      </div>
    </div>
    <div class="card">
      <b>BATTERY</b>
      <div class="bat-wrap"><div class="bat-bar" id="bat_bar"></div></div>
      <div class="tele">
        <div><span class="k">SoC    </span><span class="v" id="b_soc">—</span></div>
        <div><span class="k">Voltage</span><span class="v" id="b_vbat">—</span></div>
        <div><span class="k">I motor</span><span class="v" id="b_im">—</span></div>
        <div><span class="k">I logic</span><span class="v" id="b_il">—</span></div>
        <div><span class="k">Power  </span><span class="v" id="b_pw">—</span></div>
        <div><span class="k">Energy </span><span class="v" id="b_en">—</span></div>
        <div><span class="k">t rem  </span><span class="v" id="b_tr">—</span></div>
      </div>
    </div>
  </div>
  <div class="col">
    <div class="card"><b>PID GAINS</b>
      <div class="row"><label>Kp</label><input type="range" id="kp" min="0" max="5000" step="10"     oninput="send('kp',this.value)"><span class="val" id="kp_v">—</span></div>
      <div class="row"><label>Kd</label><input type="range" id="kd" min="0" max="1000" step="1"      oninput="send('kd',this.value)"><span class="val" id="kd_v">—</span></div>
      <div class="row"><label>Ki</label><input type="range" id="ki" min="0" max="20"   step="0.1"    oninput="send('ki',this.value)"><span class="val" id="ki_v">—</span></div>
    </div>
    <div class="card"><b>MOTOR</b>
      <div class="row"><label>Accel</label><input type="range" id="ac" min="10" max="3000" step="10" oninput="send('ac',this.value)"><span class="val" id="ac_v">—</span></div>
      <div class="row"><label>Max spd</label><input type="range" id="mw" min="3" max="120" step="1" oninput="sendCms('mw',this.value)"><span class="val" id="mw_v">—</span></div>
      <div class="row"><label>CF coeff</label><input type="range" id="cf" min="0.9" max="0.999" step="0.001" oninput="send('cf',this.value)"><span class="val" id="cf_v">—</span></div>
    </div>
    <div class="card"><b>BALANCE</b>
      <div class="row"><label>Setpoint</label><input type="range" id="sp" min="-0.3" max="0.3" step="0.001" oninput="send('sp',this.value)"><span class="val" id="sp_v">—</span></div>
      <button class="cbtn" id="cal_btn" onclick="doCalibrate()">Calibrate Gyro &amp; Balance Angle</button>
    </div>
    <div class="card"><b>DRIVE</b>
      <div class="row"><label>Kp vel</label><input type="range" id="kpv" min="0" max="0.05" step="0.001" oninput="send('kpv',this.value)"><span class="val" id="kpv_v">—</span></div>
      <div class="row"><label>Ki vel</label><input type="range" id="kvi" min="0" max="0.01" step="0.0001" oninput="send('kvi',this.value)"><span class="val" id="kvi_v">—</span></div>
      <div class="row"><label>Max tiltSP</label><input type="range" id="mts" min="0.005" max="0.3" step="0.005" oninput="send('mts',this.value)"><span class="val" id="mts_v">—</span></div>
      <div class="row"><label>Vel step</label><input type="range" id="vs" min="0.1" max="5" step="0.1" oninput="send('vs',this.value)"><span class="val" id="vs_v">—</span></div>
      <div class="row"><label>Max vel</label><input type="range" id="mvt" min="3" max="45" step="0.5" oninput="sendCms('mvt',this.value)"><span class="val" id="mvt_v">—</span></div>
      <div class="row"><label>EMA α</label><input type="range" id="ema" min="0.01" max="1" step="0.01" oninput="send('ema',this.value)"><span class="val" id="ema_v">—</span></div>
    </div>
  </div>
  <div class="col">
    <div class="card"><b>YAW CONTROL</b>
      <div class="row"><label>Kp yaw</label><input type="range" id="kyp" min="0" max="5" step="0.01" oninput="send('kyp',this.value)"><span class="val" id="kyp_v">—</span></div>
      <div class="row"><label>Kd yaw</label><input type="range" id="kdy" min="0" max="2" step="0.01" oninput="send('kdy',this.value)"><span class="val" id="kdy_v">—</span></div>
      <div class="row"><label>Ki yaw</label><input type="range" id="kiy" min="0" max="0.5" step="0.005" oninput="send('kiy',this.value)"><span class="val" id="kiy_v">—</span></div>
      <div class="row"><label>Yaw EMA</label><input type="range" id="yea" min="0.01" max="1" step="0.01" oninput="send('yea',this.value)"><span class="val" id="yea_v">—</span></div>
      <div class="row"><label>Yaw step</label><input type="range" id="trns" min="0.1" max="5" step="0.1" oninput="send('trns',this.value)"><span class="val" id="trns_v">—</span></div>
      <div class="row"><label>Max yaw</label><input type="range" id="mtb" min="0.5" max="10" step="0.5" oninput="send('mtb',this.value)"><span class="val" id="mtb_v">—</span></div>
      <div class="dpad">
        <button class="dbtn" onpointerdown="startDiag('wa')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8598;</button>
        <button class="dbtn" onpointerdown="startMove('w')" onpointerup="stopMove('w')" onpointerleave="stopMove('w')">&#9650;</button>
        <button class="dbtn" onpointerdown="startDiag('wd')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8599;</button>
        <button class="dbtn" onpointerdown="startMove('a')" onpointerup="stopMove('a')" onpointerleave="stopMove('a')">&#9664;</button>
        <button class="dbtn stop" onpointerdown="sendMove('stop')">&#9632;</button>
        <button class="dbtn" onpointerdown="startMove('d')" onpointerup="stopMove('d')" onpointerleave="stopMove('d')">&#9654;</button>
        <button class="dbtn" onpointerdown="startDiag('sa')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8601;</button>
        <button class="dbtn" onpointerdown="startMove('s')" onpointerup="stopMove('s')" onpointerleave="stopMove('s')">&#9660;</button>
        <button class="dbtn" onpointerdown="startDiag('sd')" onpointerup="stopDiag()" onpointerleave="stopDiag()">&#8600;</button>
      </div>
    </div>
    <div class="card"><b>LINE FOLLOW</b>
      <div class="row"><label>Speed</label><input type="range" id="lfs" min="0" max="21.6" step="0.3" oninput="sendCms('lfs',this.value)"><span class="val" id="lfs_v">—</span></div>
      <div class="row"><label>Kp IR</label><input type="range" id="kpir" min="0" max="0.02" step="0.0001" oninput="send('kpir',this.value)"><span class="val" id="kpir_v">—</span></div>
      <div class="row"><label>Ki IR</label><input type="range" id="kiir" min="0" max="0.005" step="0.00005" oninput="send('kiir',this.value)"><span class="val" id="kiir_v">—</span></div>
      <div class="row"><label>Kd IR</label><input type="range" id="kdir" min="0" max="0.0002" step="0.000002" oninput="send('kdir',this.value)"><span class="val" id="kdir_v">—</span></div>
      <div class="row"><label>Lost spd</label><input type="range" id="lflsf" min="0" max="1" step="0.05" oninput="send('lflsf',this.value)"><span class="val" id="lflsf_v">—</span></div>
      <div class="row"><label>Spd scale</label><input type="range" id="lfvs" min="200" max="2000" step="50" oninput="send('lfvs',this.value)"><span class="val" id="lfvs_v">—</span></div>
      <div class="row"><label>Min spd</label><input type="range" id="lfms" min="0.1" max="1" step="0.05" oninput="send('lfms',this.value)"><span class="val" id="lfms_v">—</span></div>
      <div class="tele" style="margin:4px 0">
        <div><span class="k">IR pos  </span><span class="v" id="t_ip">—</span></div>
        <div><span class="k">IR corr </span><span class="v" id="t_ic">—</span></div>
      </div>
      <button class="cbtn" id="ir_cal_btn" onclick="doCalibrateIR()">Calibrate IR Sensors</button>
      <button class="cbtn" id="lf_btn" onclick="toggleLF()">Enable Line Follow</button>
    </div>
  </div>
</div>
<script>
function send(p,v){
  var dp=p==='kdir'?5:(p==='sp'||p==='ki'||p==='kpv'||p==='kvi'||p==='kiy'||p==='kpir'||p==='kiir')?4:(p==='lflsf'||p==='lfms')?2:p==='lfvs'?0:(p==='cf'||p==='mts'||p==='kyp'||p==='kdy')?3:(p==='ema'||p==='yea')?2:1;
  document.getElementById(p+'_v').textContent=parseFloat(v).toFixed(dp);
  fetch('/set?'+p+'='+v);
}
function sendCms(p,v){
  document.getElementById(p+'_v').textContent=parseFloat(v).toFixed(1)+' cm/s';
  fetch('/set?'+p+'='+(parseFloat(v)/3));
}
function toggleLF(){
  var en=document.getElementById('lf_btn').dataset.on!=='1';
  fetch('/linefollow?en='+(en?1:0)).then(function(r){return r.json();}).then(function(d){setLFBtn(d.lf);});
}
function setLFBtn(on){
  var b=document.getElementById('lf_btn');
  b.dataset.on=on?'1':'0';
  if(on){b.textContent='Disable Line Follow';b.style.color='#f55';b.style.borderColor='#f55';}
  else{b.textContent='Enable Line Follow';b.style.color='#4af';b.style.borderColor='#4af';}
}
var inited=false;
function poll(){
  fetch('/status').then(function(r){return r.json();}).then(function(d){
    document.getElementById('t_th').textContent=d.theta.toFixed(4)+' rad';
    document.getElementById('t_st').textContent=d.setpt.toFixed(4)+' rad';
    document.getElementById('t_gy').textContent=d.gyro.toFixed(3);
    document.getElementById('t_er').textContent=d.err.toFixed(4);
    document.getElementById('t_sp').textContent=d.spd.toFixed(2);
    document.getElementById('t_ts').textContent=d.tiltSP.toFixed(4)+' rad';
    document.getElementById('t_ve').textContent=(d.velEst*3).toFixed(2)+' cm/s';
    document.getElementById('t_vt').textContent=(d.velTarget*3).toFixed(2)+' cm/s';
    document.getElementById('t_vi').textContent=(d.vint||0).toFixed(4);
    document.getElementById('t_yr').textContent=(d.yaw_rate||0).toFixed(4)+' r/s';
    document.getElementById('t_yc').textContent=(d.yawCorr||0).toFixed(4)+' r/s';
    document.getElementById('t_yt').textContent=(d.turnBias||0).toFixed(3)+' r/s';
    document.getElementById('t_ip').textContent=(d.irPos>=0)?d.irPos.toFixed(0):'none';
    document.getElementById('t_ic').textContent=(d.irCorr||0).toFixed(4);
    setLFBtn(d.lf||false);
    var soc=d.soc!=null?d.soc:0;
    document.getElementById('b_soc').textContent=soc.toFixed(1)+'%';
    document.getElementById('b_vbat').textContent=(d.vbat||0).toFixed(2)+' V';
    document.getElementById('b_im').textContent=(d.imotor||0).toFixed(3)+' A';
    document.getElementById('b_il').textContent=(d.ilogic||0).toFixed(3)+' A';
    document.getElementById('b_pw').textContent=(d.power||0).toFixed(2)+' W';
    document.getElementById('b_en').textContent=(d.energy||0).toFixed(2)+' Wh';
    document.getElementById('b_tr').textContent=d.trem>=999?'—':(d.trem||0).toFixed(0)+' min';
    var bar=document.getElementById('bat_bar');
    bar.style.width=Math.min(soc,100)+'%';
    bar.style.background=soc>50?'#4f4':soc>20?'#fa4':'#f44';
    var imuEl=document.getElementById('s_imu');
    imuEl.textContent=d.imu_ok?'OK':'ERROR';
    imuEl.style.color=d.imu_ok?'#4f4':'#f44';
    if(!inited){inited=true;
      ['kp','kd','ki','ac','cf','sp','kpv','kvi','mts','vs','ema','trns','mtb','kyp','kdy','kiy','yea','kpir','kiir','kdir','lflsf','lfvs','lfms'].forEach(function(p){
        document.getElementById(p).value=d[p]||0;
        var dp=p==='kdir'?5:(p==='sp'||p==='ki'||p==='kpv'||p==='kvi'||p==='kiy'||p==='kpir'||p==='kiir')?4:(p==='lflsf'||p==='lfms')?2:p==='lfvs'?0:(p==='cf'||p==='mts'||p==='kyp'||p==='kdy')?3:(p==='ema'||p==='yea')?2:1;
        document.getElementById(p+'_v').textContent=parseFloat(d[p]||0).toFixed(dp);
      });
      ['mw','mvt','lfs'].forEach(function(p){
        var cms=(d[p]||0)*3;
        document.getElementById(p).value=cms;
        document.getElementById(p+'_v').textContent=cms.toFixed(1)+' cm/s';
      });
    }
  }).catch(function(){});
}
var moveIv=null;
function startMove(dir){
  if(moveIv)clearInterval(moveIv);
  sendMove(dir);
  moveIv=setInterval(function(){sendMove(dir);},120);
}
function stopMove(dir){
  clearInterval(moveIv);moveIv=null;
  if(dir==='a'||dir==='d') sendMove('stop_turn');
  else sendMove('stop_fb');
}
function sendMove(dir){fetch('/move?dir='+dir).catch(function(){});}
function startDiag(dir){
  if(moveIv)clearInterval(moveIv);
  sendMove(dir);
  moveIv=setInterval(function(){sendMove(dir);},120);
}
function stopDiag(){
  clearInterval(moveIv);moveIv=null;
  sendMove('stop_fb');
  sendMove('stop_turn');
}
function doCalibrate(){
  var b=document.getElementById('cal_btn');
  b.textContent='Calibrating…';b.disabled=true;
  fetch('/calibrate').then(function(r){return r.json();}).then(function(d){
    document.getElementById('sp').value=d.sp;
    document.getElementById('sp_v').textContent=parseFloat(d.sp).toFixed(4);
    b.textContent='Calibrate Gyro & Balance Angle';b.disabled=false;
  }).catch(function(){
    b.textContent='Calibrate Gyro & Balance Angle';b.disabled=false;
  });
}
function doCalibrateIR(){
  var b=document.getElementById('ir_cal_btn');
  b.textContent='Calibrating IR… (5s)';b.disabled=true;
  fetch('/calibrateIR').then(function(){
    b.textContent='Calibrate IR Sensors';b.disabled=false;
  }).catch(function(){
    b.textContent='Calibrate IR Sensors';b.disabled=false;
  });
}
setInterval(poll,500);poll();
</script></body></html>
)html";