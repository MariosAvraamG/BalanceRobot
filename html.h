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
h2{color:#4af;margin:0 0 4px;font-size:15px}

/* ── Tabs ── */
.tabs{display:flex;gap:0;border-bottom:1px solid #2a2a2a;margin-bottom:0}
.tab-btn{background:none;border:1px solid transparent;border-bottom:none;border-radius:4px 4px 0 0;color:#555;font-family:monospace;font-size:12px;padding:5px 18px;cursor:pointer;margin-bottom:-1px;transition:color .15s}
.tab-btn.active{border-color:#2a2a2a;border-bottom-color:#111;color:#4af;background:#111}
.tab-btn:hover:not(.active){color:#aaa}
.tab-pane{display:none;height:calc(100vh - 66px);padding-top:6px}
.tab-pane.active{display:block}

/* ── Dashboard ── */
.dash-grid{display:grid;grid-template-columns:1fr 256px;gap:8px;height:100%}
.cam-card{background:#1e1e1e;border-radius:6px;padding:8px;display:flex;flex-direction:column;gap:6px;min-height:0}
.lbl{color:#4af;font-size:11px;font-weight:bold;letter-spacing:.05em;display:block}
.cam-wrap{flex:1;min-height:0;background:#0a0a0a;border-radius:4px;border:1px solid #252525;overflow:hidden;display:flex;align-items:center;justify-content:center;position:relative}
.cam-wrap img{width:100%;height:100%;object-fit:contain;display:block}
.no-cam{position:absolute;display:flex;flex-direction:column;align-items:center;gap:8px;color:#333;font-size:12px;pointer-events:none;text-align:center}
.no-cam svg{width:48px;height:48px;opacity:.4}
.dash-side{display:flex;flex-direction:column;gap:6px;overflow-y:auto;padding-right:2px}
.dash-card{background:#1e1e1e;border-radius:6px;padding:8px 10px;flex-shrink:0}
.mode-badge{font-size:15px;font-weight:bold;text-align:center;padding:9px 4px;border-radius:4px;background:#0d0d0d;letter-spacing:.1em;margin-top:5px}
.srow{display:flex;justify-content:space-between;align-items:baseline;padding:4px 0;border-bottom:1px solid #222}
.srow:last-child{border-bottom:none}
.sk{color:#666;font-size:10px}
.sv{color:#4af;font-size:11px;font-weight:bold}
.bat-mini{background:#252525;border-radius:3px;height:4px;margin:3px 0 6px;overflow:hidden}
.bat-mini-bar{height:100%;width:0%;background:#4f4;border-radius:3px;transition:width .5s,background .5s}

/* ── Calibration (original layout) ── */
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;height:100%}
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

/* ── Server tab ── */
#pane_srv.active{display:flex;flex-direction:column}
.srv-hdr{display:flex;align-items:center;justify-content:space-between;padding:4px 0 8px;border-bottom:1px solid #2a2a2a;margin-bottom:8px;flex-shrink:0}
.srv-hdr-title{font-size:13px;font-weight:600;color:#ddd}
.srv-status{display:flex;align-items:center;gap:7px;font-size:11px;color:#555}
.srv-dot{width:8px;height:8px;border-radius:50%;flex-shrink:0;transition:background .3s,box-shadow .3s}
.srv-dot.green{background:#22c55e;box-shadow:0 0 6px #22c55e88}
.srv-dot.red{background:#ef4444;box-shadow:0 0 6px #ef444488}
.srv-tiles{display:flex;gap:8px;margin-bottom:8px;flex-shrink:0}
.srv-tile{background:#1e1e1e;border:1px solid #2a2a2a;border-radius:6px;padding:10px 14px;min-width:180px}
.srv-tile-val{font-size:28px;font-weight:700;line-height:1;margin-bottom:3px;font-variant-numeric:tabular-nums}
.srv-tile-lbl{font-size:10px;color:#555;text-transform:uppercase;letter-spacing:.07em}
.srv-tile-bot .srv-tile-val{color:#4f4}
.srv-fbar{display:flex;align-items:center;margin-bottom:6px;flex-shrink:0}
.btn-srv-clear{margin-left:auto;background:none;border:1px solid #333;border-radius:4px;color:#555;padding:3px 10px;font-size:11px;cursor:pointer;font-family:monospace;transition:border-color .15s,color .15s}
.btn-srv-clear:hover{border-color:#ef4444;color:#ef4444}
.srv-tbl-wrap{flex:1;min-height:0;overflow-y:auto;background:#1e1e1e;border:1px solid #2a2a2a;border-radius:6px}
.srv-tbl-wrap table{width:100%;border-collapse:collapse;font-size:11px;font-family:monospace}
.srv-tbl-wrap thead th{background:#161616;padding:7px 12px;text-align:left;font-weight:600;font-size:10px;text-transform:uppercase;letter-spacing:.07em;color:#444;border-bottom:1px solid #2a2a2a;white-space:nowrap;position:sticky;top:0;z-index:1}
.srv-tbl-wrap tbody td{padding:6px 12px;border-bottom:1px solid #1a1a1a;vertical-align:middle;color:#bbb}
.srv-tbl-wrap tbody tr:last-child td{border-bottom:none}
.srv-tbl-wrap tbody tr:hover{background:#242424}
.srv-dim{color:#444}
.srv-empty{text-align:center;padding:28px;color:#444;font-style:italic}
.s-badge{display:inline-block;padding:2px 7px;border-radius:3px;font-size:10px;font-weight:600;letter-spacing:.03em;white-space:nowrap}
.s-badge-pi{background:#0d1f33;color:#4af}
.s-badge-bot{background:#0d2b1a;color:#4f4}
.s-badge-ctrl{background:#2b2006;color:#fa4}
.s-badge-active{background:#0d2b1a;color:#4f4}
.s-badge-sleeping{background:#1e1e1e;color:#555;border:1px solid #333}
.s-badge-manual{background:#2b2006;color:#fa4}
.s-badge-auto{background:#0d1f33;color:#4af}
@keyframes srv-flash{0%{background:#0d1f33}100%{background:transparent}}
.srv-flash{animation:srv-flash 1.4s ease-out}
</style></head><body>
<h2>BalanceBot Tuner</h2>
<div class="tabs">
  <button class="tab-btn active" id="btn_dash" onclick="switchTab('dash')">Dashboard</button>
  <button class="tab-btn" id="btn_cal" onclick="switchTab('cal')">Calibration</button>
  <button class="tab-btn" id="btn_srv" onclick="switchTab('srv')">Server</button>
</div>

<!-- ════════════════════════════ DASHBOARD ════════════════════════════ -->
<div id="pane_dash" class="tab-pane active">
  <div class="dash-grid">

    <!-- Camera -->
    <div class="cam-card">
      <span class="lbl">CAMERA FEED</span>
      <div class="cam-wrap">
        <img id="cam_img" src="http://192.168.1.66/stream" alt=""
             onload="document.getElementById('no_cam').style.display='none'"
             onerror="camError()">
        <div class="no-cam" id="no_cam">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.2">
            <path d="M23 7l-7 5 7 5V7z"/><rect x="1" y="5" width="15" height="14" rx="2" ry="2"/>
            <line x1="1" y1="1" x2="23" y2="23"/>
          </svg>
          No camera signal
        </div>
      </div>
    </div>

    <!-- Side panel -->
    <div class="dash-side">

      <!-- Mode -->
      <div class="dash-card">
        <span class="lbl">CURRENT MODE</span>
        <div class="mode-badge" id="d_mode">—</div>
      </div>

      <!-- Battery -->
      <div class="dash-card">
        <span class="lbl">BATTERY</span>
        <div class="bat-mini"><div class="bat-mini-bar" id="d_bat_bar"></div></div>
        <div class="srow"><span class="sk">State of charge</span><span class="sv" id="d_soc">—</span></div>
        <div class="srow"><span class="sk">Voltage</span><span class="sv" id="d_vbat">—</span></div>
        <div class="srow"><span class="sk">Motor current</span><span class="sv" id="d_mcur">—</span></div>
        <div class="srow"><span class="sk">Power draw</span><span class="sv" id="d_pw">—</span></div>
        <div class="srow"><span class="sk">Time remaining</span><span class="sv" id="d_tr">—</span></div>
      </div>

      <!-- Motion -->
      <div class="dash-card">
        <span class="lbl">MOTION</span>
        <div class="srow"><span class="sk">Velocity</span><span class="sv" id="d_vel">—</span></div>
        <div class="srow"><span class="sk">Angular velocity</span><span class="sv" id="d_angvel">—</span></div>
        <div class="srow"><span class="sk">Tilt angle</span><span class="sv" id="d_tilt">—</span></div>
        <div class="srow"><span class="sk">Motor output</span><span class="sv" id="d_mtr">—</span></div>
      </div>

      <!-- Tracking -->
      <div class="dash-card">
        <span class="lbl">TRACKING</span>
        <div class="srow"><span class="sk">Object tracked</span><span class="sv" id="d_obj">—</span></div>
      </div>

    </div>
  </div>
</div>

<!-- ═══════════════════════════ CALIBRATION ════════════════════════════ -->
<div id="pane_cal" class="tab-pane">
  <div class="grid">

    <!-- Col 1: Telemetry + System + Battery -->
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
          <div><span class="k">Q used </span><span class="v" id="b_qu">—</span></div>
        </div>
      </div>
    </div>

    <!-- Col 2: PID + Motor + Balance + Drive -->
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
        <div class="row"><label>Max vInt</label><input type="range" id="mvi" min="0" max="2000" step="10" oninput="send('mvi',this.value)"><span class="val" id="mvi_v">—</span></div>
        <div class="row"><label>Max tiltSP</label><input type="range" id="mts" min="0.005" max="0.3" step="0.005" oninput="send('mts',this.value)"><span class="val" id="mts_v">—</span></div>
        <div class="row"><label>Vel step</label><input type="range" id="vs" min="0.1" max="5" step="0.1" oninput="send('vs',this.value)"><span class="val" id="vs_v">—</span></div>
        <div class="row"><label>Max vel</label><input type="range" id="mvt" min="3" max="45" step="0.5" oninput="sendCms('mvt',this.value)"><span class="val" id="mvt_v">—</span></div>
        <div class="row"><label>EMA α</label><input type="range" id="ema" min="0.01" max="1" step="0.01" oninput="send('ema',this.value)"><span class="val" id="ema_v">—</span></div>
      </div>
    </div>

    <!-- Col 3: Yaw + Line Follow -->
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
        <div class="row"><label>Kd IR</label><input type="range" id="kdir" min="0" max="0.05" step="0.0005" oninput="send('kdir',this.value)"><span class="val" id="kdir_v">—</span></div>
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
</div>

<!-- ═══════════════════════════ SERVER ════════════════════════════ -->
<div id="pane_srv" class="tab-pane">
  <div class="srv-hdr">
    <span class="srv-hdr-title">Telemetry Log</span>
    <div class="srv-status">
      <span id="srv_dot" class="srv-dot red"></span>
      <span id="srv_txt">Not started</span>
    </div>
  </div>
  <div class="srv-tiles">
    <div class="srv-tile srv-tile-bot">
      <div class="srv-tile-val" id="srv_cnt_bot">0</div>
      <div class="srv-tile-lbl">Bot telemetry / 60s</div>
    </div>
  </div>
  <div class="srv-fbar">
    <button class="btn-srv-clear" id="srv_clear">Clear display</button>
  </div>
  <div class="srv-tbl-wrap">
    <table>
      <thead>
        <tr>
          <th style="width:130px">Mode</th>
          <th style="width:120px">Velocity (cm/s)</th>
          <th style="width:130px">Angular (rad/s)</th>
          <th style="width:120px">Battery Level</th>
          <th style="width:120px">State of Charge</th>
          <th style="width:110px">Voltage</th>
          <th>Object Detected</th>
        </tr>
      </thead>
      <tbody id="srv_tbody">
        <tr><td colspan="7" class="srv-empty">Waiting for data…</td></tr>
      </tbody>
    </table>
  </div>
</div>

<script>
/* ── Tab switching ── */
var srvInited=false;
function switchTab(id){
  ['dash','cal','srv'].forEach(function(t){
    document.getElementById('pane_'+t).classList.toggle('active',t===id);
    document.getElementById('btn_'+t).classList.toggle('active',t===id);
  });
  if(id==='srv'&&!srvInited){srvInited=true;SRV.init();}
}

var ESP_URL='http://192.168.1.66';
var SRV_URL='http://192.168.1.172:5001';

/* ── Camera ── */
var camRetryTimer=null;
function camError(){
  var img=document.getElementById('cam_img');
  var nc=document.getElementById('no_cam');
  img.style.display='none';
  nc.style.display='flex';
  clearTimeout(camRetryTimer);
  camRetryTimer=setTimeout(function(){
    img.style.display='block';
    img.src=ESP_URL+'/stream?t='+Date.now();
  },4000);
}

/* ── Parameter send helpers ── */
function send(p,v){
  var dp=p==='kdir'?5:(p==='sp'||p==='ki'||p==='kpv'||p==='kvi'||p==='kiy'||p==='kpir'||p==='kiir')?4:(p==='lflsf'||p==='lfms')?2:p==='lfvs'?0:(p==='cf'||p==='mts'||p==='kyp'||p==='kdy')?3:(p==='ema'||p==='yea')?2:1;
  document.getElementById(p+'_v').textContent=parseFloat(v).toFixed(dp);
  fetch(ESP_URL+'/set?'+p+'='+v);
}
function sendCms(p,v){
  document.getElementById(p+'_v').textContent=parseFloat(v).toFixed(1)+' cm/s';
  fetch(ESP_URL+'/set?'+p+'='+(parseFloat(v)/3));
}

/* ── Line follow toggle ── */
function toggleLF(){
  var en=document.getElementById('lf_btn').dataset.on!=='1';
  fetch(ESP_URL+'/linefollow?en='+(en?1:0)).then(function(r){return r.json();}).then(function(d){setLFBtn(d.lf);});
}
function setLFBtn(on){
  var b=document.getElementById('lf_btn');
  b.dataset.on=on?'1':'0';
  if(on){b.textContent='Disable Line Follow';b.style.color='#f55';b.style.borderColor='#f55';}
  else{b.textContent='Enable Line Follow';b.style.color='#4af';b.style.borderColor='#4af';}
}

/* ── Mode helper ── */
var MODE_COLOR={
  'BALANCING':'#4af','LINE FOLLOW':'#fa4','MANUAL':'#4f4',
  'FALLEN':'#f55','IMU ERROR':'#f44','IDLE':'#888'
};
function resolveMode(d){
  if(d.mode) return d.mode.toUpperCase();
  if(!d.imu_ok) return 'IMU ERROR';
  if(d.lf)      return 'LINE FOLLOW';
  return 'BALANCING';
}

/* ── Poll ── */
var inited=false;
function poll(){
  fetch(ESP_URL+'/status').then(function(r){return r.json();}).then(function(d){

    /* ── Calibration tab telemetry ── */
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

    /* Calibration battery */
    var soc=d.soc!=null?d.soc:0;
    document.getElementById('b_soc').textContent=soc.toFixed(1)+'%';
    document.getElementById('b_vbat').textContent=(d.vbat||0).toFixed(2)+' V';
    document.getElementById('b_im').textContent=(d.imotor||0).toFixed(3)+' A';
    document.getElementById('b_il').textContent=(d.ilogic||0).toFixed(3)+' A';
    document.getElementById('b_pw').textContent=(d.power||0).toFixed(2)+' W';
    document.getElementById('b_en').textContent=(d.energy||0).toFixed(2)+' Wh';
    document.getElementById('b_tr').textContent=d.trem>=999?'—':(d.trem||0).toFixed(0)+' min';
    document.getElementById('b_qu').textContent=(d.qused||0).toFixed(3)+' Ah';
    var bar=document.getElementById('bat_bar');
    bar.style.width=Math.min(soc,100)+'%';
    bar.style.background=soc>50?'#4f4':soc>20?'#fa4':'#f44';

    var imuEl=document.getElementById('s_imu');
    imuEl.textContent=d.imu_ok?'OK':'ERROR';
    imuEl.style.color=d.imu_ok?'#4f4':'#f44';

    /* ── Dashboard ── */
    var mode=resolveMode(d);
    var modeEl=document.getElementById('d_mode');
    modeEl.textContent=mode;
    modeEl.style.color=MODE_COLOR[mode]||'#4af';

    /* Dashboard battery */
    document.getElementById('d_soc').textContent=soc.toFixed(1)+'%';
    document.getElementById('d_vbat').textContent=(d.vbat||0).toFixed(2)+' V';
    document.getElementById('d_mcur').textContent=(d.imotor||0).toFixed(3)+' A';
    document.getElementById('d_pw').textContent=(d.power||0).toFixed(2)+' W';
    document.getElementById('d_tr').textContent=d.trem>=999?'—':(d.trem||0).toFixed(0)+' min';
    var dbar=document.getElementById('d_bat_bar');
    dbar.style.width=Math.min(soc,100)+'%';
    dbar.style.background=soc>50?'#4f4':soc>20?'#fa4':'#f44';

    /* Dashboard motion */
    document.getElementById('d_vel').textContent=(d.velEst*3).toFixed(2)+' cm/s';
    document.getElementById('d_angvel').textContent=(d.yaw_rate||0).toFixed(4)+' rad/s';
    document.getElementById('d_tilt').textContent=d.theta.toFixed(4)+' rad';
    document.getElementById('d_mtr').textContent=d.spd.toFixed(2);

    /* Dashboard tracking */
    document.getElementById('d_obj').textContent=d.trackedObj||'None';

    /* Init sliders once */
    if(!inited){inited=true;
      ['kp','kd','ki','ac','cf','sp','kpv','kvi','mvi','mts','vs','ema','trns','mtb','kyp','kdy','kiy','yea','kpir','kiir','kdir','lflsf','lfvs','lfms'].forEach(function(p){
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
    lastStatus=d;
  }).catch(function(){});
}

/* ── 5-second telemetry logger ── */
var lastStatus=null;
function logToServer(){
  if(!lastStatus) return;
  var d=lastStatus;
  fetch(SRV_URL+'/telemetry/esp-bot',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({
      mode:resolveMode(d),
      v_actual:parseFloat((d.velEst*3).toFixed(2)),
      omega_actual:parseFloat((d.yaw_rate||0).toFixed(4)),
      soc:parseFloat((d.soc||0).toFixed(1)),
      battery_level:parseFloat((d.soc||0).toFixed(1)),
      vbat:parseFloat((d.vbat||0).toFixed(2))
    })
  }).catch(function(){});
}

/* ── D-pad movement ── */
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
function sendMove(dir){fetch(ESP_URL+'/move?dir='+dir).catch(function(){});}
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

/* ── Calibration buttons ── */
function doCalibrate(){
  var b=document.getElementById('cal_btn');
  b.textContent='Calibrating…';b.disabled=true;
  fetch(ESP_URL+'/calibrate').then(function(r){return r.json();}).then(function(d){
    document.getElementById('sp').value=d.sp;
    document.getElementById('sp_v').textContent=parseFloat(d.sp).toFixed(4);
    b.textContent='Calibrate Gyro & Balance Angle';b.disabled=false;
  }).catch(function(){b.textContent='Calibrate Gyro & Balance Angle';b.disabled=false;});
}
function doCalibrateIR(){
  var b=document.getElementById('ir_cal_btn');
  b.textContent='Calibrating IR… (5s)';b.disabled=true;
  fetch(ESP_URL+'/calibrateIR').then(function(){
    b.textContent='Calibrate IR Sensors';b.disabled=false;
  }).catch(function(){b.textContent='Calibrate IR Sensors';b.disabled=false;});
}

/* ── Server tab module ── */
var SRV=(function(){
  var POLL_MS=1000,MAX_BUF=200,DISP=50;
  var latestId=0,allRows=[];
  var tbody;
  function setStatus(ok){
    document.getElementById('srv_dot').className='srv-dot '+(ok?'green':'red');
    document.getElementById('srv_txt').textContent=ok?'Connected':'Disconnected';
  }
  function updateCounts(c){
    document.getElementById('srv_cnt_bot').textContent=(c&&c.bot_telemetry)||0;
  }
  function extract(row){
    var p=row.payload;
    var rm=(p.mode||p.drive_mode||'').toLowerCase();
    var mode='—';
    if(rm==='manual') mode='manual';
    else if(rm==='auto'||rm==='autonomous') mode='autonomous';
    else if(rm) mode=rm;
    return {mode:mode,
      vel:p.v_actual!=null?p.v_actual:(p.lin_vel!=null?p.lin_vel:(p.vel_est!=null?p.vel_est:null)),
      ang:p.omega_actual!=null?p.omega_actual:(p.ang_vel!=null?p.ang_vel:(p.yaw_rate!=null?p.yaw_rate:(p.gyro_rate!=null?p.gyro_rate:null))),
      bat:p.battery_level!=null?p.battery_level:(p.bat_pct!=null?p.bat_pct:(p.soc!=null?p.soc:null)),
      soc:p.soc!=null?p.soc:(p.state_of_charge!=null?p.state_of_charge:null),
      vbat:p.vbat!=null?p.vbat:(p.voltage!=null?p.voltage:(p.bat_v!=null?p.bat_v:null)),
      obj:p.object||p.object_detected||null};
  }
  function esc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');}
  function fmtU(n,d,u){return(n===null||n===undefined)?'<span class="srv-dim">—</span>':Number(n).toFixed(d)+' '+u;}
  function modeBadge(m){
    if(m==='manual') return '<span class="s-badge s-badge-manual">manual</span>';
    if(m==='autonomous') return '<span class="s-badge s-badge-auto">autonomous</span>';
    if(m!=='—') return '<span class="s-badge">'+esc(m)+'</span>';
    return '<span class="srv-dim">—</span>';
  }
  function buildTr(row,flash){
    var tr=document.createElement('tr');
    if(flash){tr.classList.add('srv-flash');tr.addEventListener('animationend',function(){tr.classList.remove('srv-flash');},{once:true});}
    var f=extract(row);
    tr.innerHTML=
      '<td>'+modeBadge(f.mode)+'</td>'+
      '<td>'+fmtU(f.vel,1,'cm/s')+'</td>'+
      '<td>'+fmtU(f.ang,3,'rad/s')+'</td>'+
      '<td>'+fmtU(f.bat,1,'%')+'</td>'+
      '<td>'+fmtU(f.soc,1,'%')+'</td>'+
      '<td>'+fmtU(f.vbat,2,'V')+'</td>'+
      '<td>'+(f.obj?esc(f.obj):'<span class="srv-dim">—</span>')+'</td>';
    return tr;
  }
  function rebuildTable(){
    var visible=allRows.slice(0,DISP);
    tbody.innerHTML='';
    if(!visible.length){tbody.innerHTML='<tr><td colspan="7" class="srv-empty">No data yet</td></tr>';return;}
    for(var i=0;i<visible.length;i++) tbody.appendChild(buildTr(visible[i],false));
  }
  function prependRows(newRows){
    if(tbody.rows.length===1&&tbody.rows[0].querySelector('.srv-empty')) tbody.innerHTML='';
    for(var i=newRows.length-1;i>=0;i--) tbody.insertBefore(buildTr(newRows[i],true),tbody.firstChild);
    while(tbody.rows.length>DISP) tbody.removeChild(tbody.lastChild);
  }
  function srvPoll(){
    fetch(SRV_URL+'/dashboard/view?since_id='+latestId+'&limit=50').then(function(r){
      if(!r.ok) throw new Error();
      return r.json();
    }).then(function(data){
      if(data.latest_id>latestId) latestId=data.latest_id;
      if(data.rows.length){allRows=data.rows.concat(allRows).slice(0,MAX_BUF);prependRows(data.rows);}
      updateCounts(data.counts);setStatus(true);
    }).catch(function(){setStatus(false);}).then(function(){setTimeout(srvPoll,POLL_MS);});
  }
  function init(){
    tbody=document.getElementById('srv_tbody');
    document.getElementById('srv_clear').addEventListener('click',function(){
      allRows=[];
      tbody.innerHTML='<tr><td colspan="7" class="srv-empty">Display cleared — new entries will appear as they arrive</td></tr>';
    });
    fetch(SRV_URL+'/dashboard/view?limit=50').then(function(r){
      if(!r.ok) throw new Error();
      return r.json();
    }).then(function(data){
      latestId=data.latest_id;allRows=data.rows;
      rebuildTable();updateCounts(data.counts);setStatus(true);
    }).catch(function(){setStatus(false);rebuildTable();}).then(function(){setTimeout(srvPoll,POLL_MS);});
  }
  return {init:init};
})();

setInterval(logToServer,5000);
setInterval(poll,500);poll();
</script></body></html>
)html";
