
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

const float OMEGA_N = 25.0f;  // target bandwidth (rad/s) — tune upward
const float ZETA    = 0.7f;   // target damping ratio


float Kp   = 2100.0f;  // saturasting — sign determines motor ramp direction
float Kd   =  240.0f;  // gyro braking — primary tuning parameter
float Ki   =    1.0f;  // small steady-state trim


float BALANCE_ANGLE = 0.06f;  // rad — calibrate per Phase 0 above

float CF_COEFF = 0.996f;

float       maxWheelSpeed    = 19.0f;  // rad/s — hardware ceiling (tunable: mw)
float       motorAccel       = 30.0f; // rad/s² — stepper slew rate  (tunable: ac)
const float MAX_INTEGRAL     = 0.1f;   // anti-windup clamp
const float FALL_ANGLE       = 0.8f;   // rad (~46°) — give up balancing
const float WHEEL_RADIUS     = 0.03f;  // m


float MAX_TILT_OFFSET = 0.15f;         // max lean command magnitude (rad, ~8.5°)
const float MOVE_STEP       = 0.01745f; // rad per W/S keypress (1°)
const float TURN_STEP       = 2.0f;    // rad/s per A/D keypress
const float MAX_TURN_BIAS   = 10.0f;   // rad/s

float leanCommand      = 0.0f;  // actual tilt offset sent to inner loop (rad)
float turnBias         = 0.0f;  // differential speed for turning (rad/s); + = right
float wheelSpeedAlpha  = 0.6f; // EMA alpha for wheel speed filter (higher = faster, noisier)
float velocitySetpoint = 0.0f;  // target speed (cm/s); + = forward
float Kvp              = 0.005f;// velocity P gain (rad per cm/s error)
float Kvi              = 0.0005f;// velocity I gain
float Kvd              = 0.0f;  // velocity D gain
float velIntegral      = 0.0f;
float prevVelError     = 0.0f;
float velError         = 0.0f;
const float VEL_STEP         = 5.0f;   // cm/s per keypress
const float MAX_VEL_SETPOINT = 60.0f;  // cm/s
const float MAX_VEL_INTEGRAL = 300.0f; // = MAX_TILT_OFFSET / Kvi default
const int   VEL_LOOP_DIVIDER = 20;     // outer loop runs every N inner iterations (80 ms)


const int   LOOP_INTERVAL_MS  = 4;           // ms
const float LOOP_INTERVAL_S   = 0.004f;      // s  (nominal — actual dt measured per iteration)
const int   STEPPER_INTERVAL_US = 50;        // µs — 20 kHz ISR
const int   PRINT_INTERVAL_MS = 2000;         // ms

// ─────────────────────────────────────────────────────────────────
//  Pins
// ─────────────────────────────────────────────────────────────────
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;

// ─────────────────────────────────────────────────────────────────
//  Live telemetry globals (read by web /status endpoint)
// ─────────────────────────────────────────────────────────────────
float    theta       = 0.0f;
float    gyro_rate   = 0.0f;
float    gyro_raw          = 0.0f;  // raw (un-biased) gyro reading
float    filteredWheelSpeed = 0.0f;  // EMA-filtered wheel speed in cm/s
float    integral    = 0.0f;   // PID integral — global so calibration can reset it
bool     imuOk       = true;
uint32_t imuErrCount = 0;
uint32_t lastCalibMs = 0;
volatile bool calibrating = false;

// ─────────────────────────────────────────────────────────────────
//  Web tuner
// ─────────────────────────────────────────────────────────────────
WebServer server(80);

static const char HTML[] PROGMEM = R"html(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceBot Tuner</title>
<style>
body{font-family:monospace;margin:8px 10px;background:#111;color:#ddd}
h2{color:#4af;margin:0 0 8px;font-size:15px}
.layout{display:flex;gap:10px;align-items:flex-start;max-width:900px;margin:0 auto}
.viz-panel{position:sticky;top:8px;flex-shrink:0;width:140px}
.viz-panel canvas{border-radius:8px;display:block}
.controls-panel{flex:1;min-width:0;display:grid;grid-template-columns:1fr 1fr;gap:6px;align-items:start}
.card{background:#1e1e1e;border-radius:6px;padding:8px 10px}
.card b{color:#4af;font-size:12px;letter-spacing:.05em}
.full{grid-column:1/-1}
.row{display:flex;align-items:center;margin:5px 0;gap:8px}
label{width:68px;font-size:12px;color:#999}
input[type=range]{flex:1;accent-color:#4af}
.val{width:60px;text-align:right;font-size:12px;color:#4af}
.nval{width:72px;text-align:right;font-size:12px;color:#4af;background:transparent;border:none;border-bottom:1px solid #333;font-family:monospace;padding:0}
.tele{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px 8px;margin-top:6px;font-size:11px}
.tele .k{color:#777}
.tele .v{color:#4af}
.dpad{display:grid;grid-template-columns:repeat(3,52px);grid-template-rows:repeat(3,52px);gap:5px;justify-content:center;margin:6px 0}
.dbtn{background:#1a1a2e;border:2px solid #333;border-radius:8px;color:#4af;font-size:20px;cursor:pointer;width:52px;height:52px;touch-action:none;user-select:none;-webkit-user-select:none}
.dbtn:active{background:#4af;color:#111;border-color:#4af}
.dbtn.stop{color:#f55;border-color:#555}
.dbtn.stop:active{background:#f55;color:#111;border-color:#f55}
.cbtn{width:100%;margin-top:6px;padding:7px;background:#1a1a2e;border:1px solid #4af;border-radius:6px;color:#4af;font-family:monospace;font-size:12px;cursor:pointer}
.cbtn:active{background:#4af;color:#111}
.cbtn:disabled{opacity:.5;cursor:default}
.lbtn{background:transparent;border:1px solid #555;border-radius:4px;color:#555;font-family:monospace;font-size:11px;padding:2px 6px;cursor:pointer;white-space:nowrap}
.lbtn.open{border-color:#f84;color:#f84}
input[type=range]:disabled{opacity:0.25;cursor:default}
</style></head><body>
<h2>BalanceBot Tuner</h2>
<div class="layout">
<div class="viz-panel">
  <canvas id="viz" width="140" height="280" style="background:#1e1e1e"></canvas>
</div>
<div class="controls-panel">
<div class="card">
  <b>TELEMETRY</b>
  <div class="tele">
    <div><span class="k">meas θ</span><span class="v" id="t_th">—</span></div>
    <div><span class="k">want θ</span><span class="v" id="t_st">—</span></div>
    <div><span class="k">gyro</span><span class="v" id="t_gy">—</span></div>
    <div><span class="k">error</span><span class="v" id="t_er">—</span></div>
    <div><span class="k">motor</span><span class="v" id="t_sp">—</span></div>
    <div><span class="k">speed</span><span class="v" id="t_vel">—</span></div>
    <div><span class="k">v setp</span><span class="v" id="t_vsp">—</span></div>
    <div><span class="k">leanCmd</span><span class="v" id="t_lcmd">—</span></div>
    <div><span class="k">v integ</span><span class="v" id="t_vint">—</span></div>
    <div><span class="k">v err</span><span class="v" id="t_verr">—</span></div>
    <div><span class="k">IMU</span><span class="v" id="s_imu">—</span></div>
  </div>
</div>
<div class="card"><b>PID GAINS</b>
  <div class="row"><label>Kp</label><input type="range" id="kp" min="0" max="5000" step="10" oninput="send('kp',this.value)"><span class="val" id="kp_v">—</span></div>
  <div class="row"><label>Kd</label><input type="range" id="kd" min="0" max="1000" step="1"  oninput="send('kd',this.value)"><span class="val" id="kd_v">—</span></div>
  <div class="row"><label>Ki</label><input type="range" id="ki" min="0" max="20"   step="0.1" oninput="send('ki',this.value)"><span class="val" id="ki_v">—</span></div>
</div>
<div class="card"><b>VELOCITY PID</b>
  <div class="row"><label>Kvp</label><input type="range" id="vp" min="0" max="0.5"   step="0.0001"  oninput="send('vp',this.value)"><input type="number" class="nval" id="vp_v" min="0" max="0.5"  step="0.0001" oninput="sendNum('vp',this.value)"></div>
  <div class="row"><label>Kvi</label><input type="range" id="vi" min="0" max="0.05"  step="0.00001" oninput="send('vi',this.value)"><input type="number" class="nval" id="vi_v" min="0" max="0.05" step="0.00001" oninput="sendNum('vi',this.value)"></div>
  <div class="row"><label>Kvd</label><input type="range" id="vd" min="0" max="0.02"  step="0.00001" oninput="send('vd',this.value)"><input type="number" class="nval" id="vd_v" min="0" max="0.02" step="0.00001" oninput="sendNum('vd',this.value)"></div>
</div>
<div class="card"><b>MOTOR</b>
  <div class="row"><label>Accel</label><input type="range" id="ac" min="10" max="3000" step="10" oninput="send('ac',this.value)"><span class="val" id="ac_v">—</span></div>
  <div class="row"><label>Max spd</label><input type="range" id="mw" min="1" max="40" step="0.5" oninput="send('mw',this.value)"><span class="val" id="mw_v">—</span></div>
  <div class="row"><label>CF coeff</label><input type="range" id="cf" min="0.9" max="0.999" step="0.001" oninput="send('cf',this.value)"><span class="val" id="cf_v">—</span></div>
  <div class="row"><label>Spd alpha</label><input type="range" id="wa" min="0.01" max="1.0" step="0.01" oninput="send('wa',this.value)"><span class="val" id="wa_v">—</span></div>
</div>
<div class="card"><b>BALANCE</b>
  <div class="row"><label>Setpoint</label><input type="range" id="sp" min="-0.3" max="0.3" step="0.001" oninput="send('sp',this.value)"><span class="val" id="sp_v">—</span></div>
  <div class="row"><label>Max tilt</label><input type="range" id="mt" min="0.01" max="0.3" step="0.005" oninput="send('mt',this.value)" disabled><span class="val" id="mt_v">—</span><button class="lbtn" id="mt_lock" onclick="toggleMtLock()">LOCK</button></div>
  <button class="cbtn" id="cal_btn" onclick="doCalibrate()">Calibrate Gyro &amp; Balance Angle</button>
</div>
<div class="card full">
  <b>CHART</b>
  <canvas id="chart" width="520" height="130" style="width:100%;background:#141414;border-radius:4px;display:block;margin-top:4px"></canvas>
  <div style="display:flex;gap:14px;margin-top:4px;font-size:10px;flex-wrap:wrap">
    <span style="color:#4af">— θ</span>
    <span style="color:#f84">— leanCmd</span>
    <span style="color:#4f4">— speed</span>
    <span style="color:#888">-- v setpt</span>
  </div>
</div>
<div class="card"><b>DRIVE</b>
  <div class="dpad">
    <div></div>
    <button class="dbtn" onpointerdown="startMove('w')" onpointerup="stopMove('w')" onpointerleave="stopMove('w')">&#9650;</button>
    <div></div>
    <button class="dbtn" onpointerdown="startMove('a')" onpointerup="stopMove('a')" onpointerleave="stopMove('a')">&#9664;</button>
    <button class="dbtn stop" onpointerdown="sendMove('stop')">&#9632;</button>
    <button class="dbtn" onpointerdown="startMove('d')" onpointerup="stopMove('d')" onpointerleave="stopMove('d')">&#9654;</button>
    <div></div>
    <button class="dbtn" onpointerdown="startMove('s')" onpointerup="stopMove('s')" onpointerleave="stopMove('s')">&#9660;</button>
    <div></div>
  </div>
</div>
</div></div>
<script>
function send(p,v){
  var dp=(p==='sp'||p==='ki'||p==='vp'||p==='vi'||p==='vd')?5:(p==='cf'||p==='mt'||p==='wa'?3:1);
  var el=document.getElementById(p+'_v');
  if(el.tagName==='INPUT')el.value=parseFloat(v).toFixed(dp);
  else el.textContent=parseFloat(v).toFixed(dp);
  fetch('/set?'+p+'='+v);
}
function sendNum(p,v){document.getElementById(p).value=v;fetch('/set?'+p+'='+v);}
function drawRobot(theta){
  var c=document.getElementById('viz');
  var ctx=c.getContext('2d');
  var W=c.width,H=c.height;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#1e1e1e';
  ctx.fillRect(0,0,W,H);
  var cx=W/2, baseY=H-52, wheelR=26, bodyLen=170;
  // ground line
  ctx.beginPath();ctx.moveTo(10,baseY+wheelR);ctx.lineTo(W-10,baseY+wheelR);
  ctx.strokeStyle='#333';ctx.lineWidth=1;ctx.stroke();
  // vertical reference
  ctx.setLineDash([4,6]);
  ctx.beginPath();ctx.moveTo(cx,baseY);ctx.lineTo(cx,baseY-bodyLen-10);
  ctx.strokeStyle='#2a2a2a';ctx.lineWidth=1;ctx.stroke();
  ctx.setLineDash([]);
  // wheel
  ctx.beginPath();ctx.arc(cx,baseY,wheelR,0,2*Math.PI);
  ctx.strokeStyle='#4af';ctx.lineWidth=3;ctx.stroke();
  ctx.beginPath();ctx.arc(cx,baseY,4,0,2*Math.PI);
  ctx.fillStyle='#4af';ctx.fill();
  // body colour based on tilt severity
  var absT=Math.abs(theta);
  var col=absT<0.12?'#4af':absT<0.35?'#fa4':'#f44';
  // body
  var bx=cx+Math.sin(theta)*bodyLen;
  var by=baseY-Math.cos(theta)*bodyLen;
  ctx.beginPath();ctx.moveTo(cx,baseY);ctx.lineTo(bx,by);
  ctx.strokeStyle=col;ctx.lineWidth=7;ctx.lineCap='round';ctx.stroke();
  // head
  ctx.beginPath();ctx.arc(bx,by,10,0,2*Math.PI);
  ctx.fillStyle=col;ctx.fill();
  // angle label
  ctx.fillStyle='#777';ctx.font='12px monospace';ctx.textAlign='center';
  ctx.fillText((theta*180/Math.PI).toFixed(2)+'°',cx,H-10);
  // tilt arc indicator
  ctx.beginPath();ctx.arc(cx,baseY,44,-(Math.PI/2),-(Math.PI/2)+theta,theta<0);
  ctx.strokeStyle=col;ctx.lineWidth=2;ctx.stroke();
}
drawRobot(0);
var inited=false;
// ── Chart ──────────────────────────────────────────────────────────
var CHART_LEN=60; // 15 s at 250 ms
var cBuf={theta:[],lcmd:[],vel:[],vsp:[]};
function chartPush(d){
  var keys=['theta','lcmd','vel','vsp'];
  keys.forEach(function(k){
    cBuf[k].push(d[k]||0);
    if(cBuf[k].length>CHART_LEN)cBuf[k].shift();
  });
  drawChart();
}
function drawChart(){
  var c=document.getElementById('chart');
  var ctx=c.getContext('2d');
  var W=c.width,H=c.height,n=cBuf.theta.length;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#141414';ctx.fillRect(0,0,W,H);
  if(n<2)return;
  var pH=H/2-4; // panel height
  // grid
  ctx.strokeStyle='#222';ctx.lineWidth=1;
  [0,pH+4,H].forEach(function(y){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();});
  ctx.beginPath();ctx.moveTo(0,pH/2+2);ctx.lineTo(W,pH/2+2);ctx.strokeStyle='#1a1a1a';ctx.stroke();
  ctx.beginPath();ctx.moveTo(0,pH+4+pH/2);ctx.lineTo(W,pH+4+pH/2);ctx.stroke();
  function plotSeries(buf,panelTop,panelH,vMin,vMax,color,dashed){
    var xStep=W/(CHART_LEN-1);
    ctx.beginPath();
    if(dashed)ctx.setLineDash([4,4]);else ctx.setLineDash([]);
    ctx.strokeStyle=color;ctx.lineWidth=1.5;
    for(var i=0;i<buf.length;i++){
      var x=(i+(CHART_LEN-buf.length))*xStep;
      var y=panelTop+panelH-(buf[i]-vMin)/(vMax-vMin)*panelH;
      y=Math.max(panelTop,Math.min(panelTop+panelH,y));
      i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    }
    ctx.stroke();ctx.setLineDash([]);
  }
  // top panel: angles (rad), range ±0.3
  plotSeries(cBuf.theta,2,pH,-0.3,0.3,'#4af',false);
  plotSeries(cBuf.lcmd, 2,pH,-0.3,0.3,'#f84',false);
  // bottom panel: speeds (cm/s), range ±60
  plotSeries(cBuf.vel, pH+6,pH,-60,60,'#4f4',false);
  plotSeries(cBuf.vsp, pH+6,pH,-60,60,'#888',true);
  // axis labels
  ctx.fillStyle='#444';ctx.font='10px monospace';ctx.textAlign='left';
  ctx.fillText('±0.3 rad',2,12);
  ctx.fillText('±60 cm/s',2,pH+18);
}
drawChart();
// ── Step test ──────────────────────────────────────────────────────
var stepTmr=null;
function runStep(){
  if(stepTmr)return;
  var sz=parseFloat(document.getElementById('step_sz').value)||20;
  var dur=parseInt(document.getElementById('step_dur').value)||3;
  var btn=document.getElementById('step_btn');
  btn.disabled=true;
  fetch('/set?vs='+sz);
  var rem=dur;
  btn.textContent='Running… '+rem+'s';
  stepTmr=setInterval(function(){
    rem--;
    if(rem<=0){
      clearInterval(stepTmr);stepTmr=null;
      fetch('/set?vs=0');
      btn.disabled=false;btn.textContent='Run Step Test';
    } else {
      btn.textContent='Running… '+rem+'s';
    }
  },1000);
}
function poll(){
  fetch('/status').then(function(r){return r.json();}).then(function(d){
    drawRobot(d.theta);
    document.getElementById('t_th').textContent=d.theta.toFixed(4)+' rad';
    document.getElementById('t_st').textContent=d.setpt.toFixed(4)+' rad';
    document.getElementById('t_gy').textContent=d.gyro.toFixed(3);
    document.getElementById('t_er').textContent=d.err.toFixed(4);
    document.getElementById('t_sp').textContent=d.spd.toFixed(2);
    document.getElementById('t_vel').textContent=d.vel.toFixed(2)+' cm/s';
    document.getElementById('t_vsp').textContent=(d.vsp||0).toFixed(1)+' cm/s';
    document.getElementById('t_lcmd').textContent=(d.lcmd||0).toFixed(4)+' rad';
    document.getElementById('t_vint').textContent=(d.vint||0).toFixed(2);
    document.getElementById('t_verr').textContent=(d.verr||0).toFixed(2)+' cm/s';
    chartPush({theta:d.theta,lcmd:d.lcmd||0,vel:d.vel,vsp:d.vsp||0});
    var imuEl=document.getElementById('s_imu');
    imuEl.textContent=d.imu_ok?'OK':'ERROR';
    imuEl.style.color=d.imu_ok?'#4f4':'#f44';
    if(!inited){inited=true;
      ['kp','kd','ki','vp','vi','vd','ac','mw','cf','wa','sp','mt'].forEach(function(p){
        document.getElementById(p).value=d[p];
        var dp=(p==='sp'||p==='ki'||p==='vp'||p==='vi'||p==='vd')?5:(p==='cf'||p==='mt'||p==='wa'?3:1);
        var el=document.getElementById(p+'_v');
        if(el.tagName==='INPUT')el.value=parseFloat(d[p]).toFixed(dp);
        else el.textContent=parseFloat(d[p]).toFixed(dp);
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
  sendMove(dir==='w'||dir==='s'?'stop_fb':'stop_turn');
}
function sendMove(dir){fetch('/move?dir='+dir).catch(function(){});}
function toggleMtLock(){
  var sl=document.getElementById('mt');
  var btn=document.getElementById('mt_lock');
  var locked=!sl.disabled;
  sl.disabled=locked;
  btn.textContent=locked?'LOCK':'UNLK';
  locked?btn.classList.remove('open'):btn.classList.add('open');
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
setInterval(poll,250);poll();
</script></body></html>
)html";

// ─────────────────────────────────────────────────────────────────
//  Objects
// ─────────────────────────────────────────────────────────────────
float            gyroBias = 0.0f;  // measured at startup, subtracted each iteration
ESP32Timer       ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// ─────────────────────────────────────────────────────────────────
//  Stepper ISR — fires every STEPPER_INTERVAL_US µs
//  MUST be kept minimal: no floats, no Serial
// ─────────────────────────────────────────────────────────────────
bool IRAM_ATTR TimerHandler(void*)
{
    static bool tog = false;
    step1.runStepper();
    step2.runStepper();
    digitalWrite(TOGGLE_PIN, tog);
    tog = !tog;
    return true;
}

// ─────────────────────────────────────────────────────────────────
//  Print theoretical gain calculations at startup
// ─────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    pinMode(TOGGLE_PIN,    OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW);  // LOW = motors enabled

    Wire.begin(21, 22);
    Wire.setClock(100000);  // 100 kHz — robust under ISR interruptions

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found — check wiring");
        while (1) delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    

    // Set acceleration high so the slew-rate limiter in the step
    // library does NOT restrict the PID. The PID sets target speed;
    // the library ramps to it. At 1000 rad/s², it reaches 19.6 rad/s
    // in ~20 ms — fast enough not to impede control.
    step1.setAccelerationRad(motorAccel);
    step2.setAccelerationRad(motorAccel);

    if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
        Serial.println("Stepper ISR attach failed");
        while (1) delay(10);
    }

    // ── Gyro bias calibration ─────────────────────────────────────
    // Average 200 readings at 5 ms intervals (1 second total).
    // Robot must be stationary during this window.
    // ── WiFi Access Point ─────────────────────────────────────────
    WiFi.softAP("BalanceBot", "balance123");
    WiFi.setSleep(false);  // disable modem sleep — prevents client disconnects
    Serial.printf("Web tuner: connect to WiFi 'BalanceBot' then open http://%s\n",
                  WiFi.softAPIP().toString().c_str());

    server.on("/", [](){
        server.send_P(200, "text/html", HTML);
    });
    server.on("/set", [](){
        if (server.hasArg("kp")) Kp              = server.arg("kp").toFloat();
        if (server.hasArg("kd")) Kd              = server.arg("kd").toFloat();
        if (server.hasArg("ki")) Ki              = server.arg("ki").toFloat();
        if (server.hasArg("vp")) Kvp             = server.arg("vp").toFloat();
        if (server.hasArg("vi")) Kvi             = server.arg("vi").toFloat();
        if (server.hasArg("vd")) Kvd             = server.arg("vd").toFloat();
        if (server.hasArg("wa")) wheelSpeedAlpha  = constrain(server.arg("wa").toFloat(), 0.01f, 1.0f);
        if (server.hasArg("vs")) velocitySetpoint = constrain(server.arg("vs").toFloat(), -MAX_VEL_SETPOINT, MAX_VEL_SETPOINT);
        if (server.hasArg("sp")) BALANCE_ANGLE = server.arg("sp").toFloat();
        if (server.hasArg("mw")) maxWheelSpeed = server.arg("mw").toFloat();
        if (server.hasArg("cf")) CF_COEFF       = constrain(server.arg("cf").toFloat(), 0.0f, 0.9999f);
        if (server.hasArg("mt")) MAX_TILT_OFFSET = constrain(server.arg("mt").toFloat(), 0.01f, 0.3f);
        if (server.hasArg("ac")) {
            motorAccel = server.arg("ac").toFloat();
            step1.setAccelerationRad(motorAccel);
            step2.setAccelerationRad(motorAccel);
        }
        server.send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/move", [](){
        String dir = server.arg("dir");
        if      (dir == "w")         velocitySetpoint = constrain(velocitySetpoint + VEL_STEP, -MAX_VEL_SETPOINT, MAX_VEL_SETPOINT);
        else if (dir == "s")         velocitySetpoint = constrain(velocitySetpoint - VEL_STEP, -MAX_VEL_SETPOINT, MAX_VEL_SETPOINT);
        else if (dir == "a")         turnBias         = constrain(turnBias - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);
        else if (dir == "d")         turnBias         = constrain(turnBias + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);
        else if (dir == "stop")    { velocitySetpoint = 0.0f; velIntegral = 0.0f; leanCommand = 0.0f; turnBias = 0.0f; }
        else if (dir == "stop_fb") { velocitySetpoint = 0.0f; velIntegral = 0.0f; }
        else if (dir == "stop_turn") turnBias         = 0.0f;
        server.send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/calibrate", [](){
        calibrating      = true;  // pause PID loop on core 1
        step1.setTargetSpeedRad(0.0f);
        step2.setTargetSpeedRad(0.0f);
        velocitySetpoint = 0.0f;
        velIntegral      = 0.0f;
        prevVelError     = 0.0f;
        leanCommand      = 0.0f;
        turnBias         = 0.0f;
        integral         = 0.0f;
        delay(300);  // let motors coast to stop before sampling
        Serial.println("Web calibration — hold robot upright and still...");
        float gyroSum  = 0.0f;
        float accelSum = 0.0f;
        const int N = 200;
        for (int i = 0; i < N; i++) {
            sensors_event_t a, g, tmp;
            mpu.getEvent(&a, &g, &tmp);
            gyroSum  += g.gyro.y;
            accelSum += atan2f(a.acceleration.z, a.acceleration.x);
            delay(5);
        }
        gyroBias      = gyroSum  / N;
        BALANCE_ANGLE = accelSum / N;
        lastCalibMs   = millis();
        calibrating   = false;
        Serial.printf("Calibrated — bias=%.4f  balance=%.4f rad (%.2f deg)\n",
                      gyroBias, BALANCE_ANGLE, BALANCE_ANGLE * 180.0f / PI);
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"ok\":true,\"sp\":%.4f}", BALANCE_ANGLE);
        server.send(200, "application/json", buf);
    });
    server.on("/status", [](){
        uint32_t upSec    = millis() / 1000;
        uint32_t calSec   = lastCalibMs ? upSec - lastCalibMs / 1000 : 0;
        char buf[512];
        snprintf(buf, sizeof(buf),
            "{\"theta\":%.4f,\"setpt\":%.4f,\"gyro\":%.4f,\"err\":%.4f,\"spd\":%.2f,\"vel\":%.3f,"
            "\"kp\":%.1f,\"kd\":%.1f,\"ki\":%.4f,\"sp\":%.4f,\"ac\":%.1f,\"mw\":%.1f,"
            "\"bias\":%.4f,\"raw\":%.4f,\"imu_ok\":%d,\"imu_err\":%u,\"cal_s\":%u,\"cf\":%.3f,\"mt\":%.3f,"
            "\"wa\":%.3f,\"vp\":%.5f,\"vi\":%.5f,\"vd\":%.5f,"
            "\"vsp\":%.2f,\"lcmd\":%.4f,\"vint\":%.3f,\"verr\":%.3f}",
            theta, BALANCE_ANGLE + leanCommand, gyro_rate, BALANCE_ANGLE - theta, step1.getSpeedRad(),
            filteredWheelSpeed,
            Kp, Kd, Ki, BALANCE_ANGLE, motorAccel, maxWheelSpeed,
            gyroBias, gyro_raw, (int)imuOk, imuErrCount, calSec, CF_COEFF, MAX_TILT_OFFSET,
            wheelSpeedAlpha, Kvp, Kvi, Kvd,
            velocitySetpoint, leanCommand, velIntegral, velError);
        server.send(200, "application/json", buf);
    });
    server.begin();
    xTaskCreatePinnedToCore(
        [](void*){ for(;;){ server.handleClient(); vTaskDelay(1); } },
        "web", 4096, nullptr, 1, nullptr, 0);

    Serial.println("Calibrating — hold robot upright and still for 1 second...");
    {
        float gyroSum  = 0.0f;
        float accelSum = 0.0f;
        const int N = 200;
        for (int i = 0; i < N; i++) {
            sensors_event_t a, g, tmp;
            mpu.getEvent(&a, &g, &tmp);
            gyroSum  += g.gyro.y;
            accelSum += atan2f(a.acceleration.z, a.acceleration.x);
            delay(5);
        }
        gyroBias     = gyroSum  / N;
        BALANCE_ANGLE = accelSum / N;
        Serial.printf("Gyro bias:     %.4f rad/s\n", gyroBias);
        Serial.printf("Balance angle: %.4f rad (%.2f deg)\n",
                      BALANCE_ANGLE, BALANCE_ANGLE * 180.0f / PI);
    }

}

// ─────────────────────────────────────────────────────────────────
//  Main loop
// ─────────────────────────────────────────────────────────────────
void loop()
{
    static unsigned long loopTimer  = 0;
    static unsigned long printTimer = 0;
    static unsigned long lastLoopUs = 0;

    // ── Serial command parser ──────────────────────────────────────
    if (Serial.available()) {
        char peek = (char)Serial.peek();

        // Single-character WASD commands — no Enter needed
        if (peek=='w'||peek=='W'||peek=='s'||peek=='S'||
            peek=='a'||peek=='A'||peek=='d'||peek=='D'||peek==' ') {
            Serial.read();  // consume
            switch (tolower(peek)) {
                case 'w': velocitySetpoint = constrain(velocitySetpoint + VEL_STEP, -MAX_VEL_SETPOINT, MAX_VEL_SETPOINT); break;
                case 's': velocitySetpoint = constrain(velocitySetpoint - VEL_STEP, -MAX_VEL_SETPOINT, MAX_VEL_SETPOINT); break;
                case 'a': turnBias         = constrain(turnBias - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);               break;
                case 'd': turnBias         = constrain(turnBias + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);               break;
                case ' ': velocitySetpoint = 0.0f; velIntegral = 0.0f; leanCommand = 0.0f; turnBias = 0.0f; break;
            }
            Serial.printf("MOVE  lean=%.4f  turn=%.2f\n", leanCommand, turnBias);
        } else {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if      (cmd.startsWith("kp"))   Kp            = cmd.substring(3).toFloat();
            else if (cmd.startsWith("kd"))   Kd            = cmd.substring(3).toFloat();
            else if (cmd.startsWith("ki"))   Ki            = cmd.substring(3).toFloat();
            else if (cmd.startsWith("sp"))   BALANCE_ANGLE = cmd.substring(3).toFloat();
            else if (cmd.startsWith("ac")) { motorAccel    = cmd.substring(3).toFloat();
                                             step1.setAccelerationRad(motorAccel);
                                             step2.setAccelerationRad(motorAccel); }
            else if (cmd.startsWith("mw"))   maxWheelSpeed = cmd.substring(3).toFloat();
            else if (cmd.startsWith("vs"))   velocitySetpoint = constrain(cmd.substring(3).toFloat(), -MAX_VEL_SETPOINT, MAX_VEL_SETPOINT);
            else if (cmd.startsWith("vp"))   Kvp         = cmd.substring(3).toFloat();
            else if (cmd.startsWith("vi"))   Kvi         = cmd.substring(3).toFloat();
            else if (cmd.startsWith("vd"))   Kvd         = cmd.substring(3).toFloat();
            else if (cmd.startsWith("tr"))   turnBias      = constrain(cmd.substring(3).toFloat(), -MAX_TURN_BIAS,   MAX_TURN_BIAS);
            else if (cmd == "en 0")          { digitalWrite(STEPPER_EN_PIN, HIGH); Serial.println("Motors DISABLED"); return; }
            else if (cmd == "en 1")          { digitalWrite(STEPPER_EN_PIN, LOW);  Serial.println("Motors ENABLED");  return; }

            Serial.printf("Kp=%.1f  Kd=%.1f  Ki=%.3f  sp=%.4f  ac=%.1f  mw=%.1f\n",
                          Kp, Kd, Ki, BALANCE_ANGLE, motorAccel, maxWheelSpeed);
        }
    }

    // ── Control loop at 200 Hz ────────────────────────────────────
    if (millis() - loopTimer >= LOOP_INTERVAL_MS) {
        loopTimer += LOOP_INTERVAL_MS;
        if (calibrating) { lastLoopUs = 0; return; }  // reset dt so first post-cal iteration uses nominal
        unsigned long nowUs = micros();
        float dt = (lastLoopUs == 0) ? LOOP_INTERVAL_S
                                     : constrain((nowUs - lastLoopUs) * 1e-6f, 0.001f, 0.020f);
        lastLoopUs = nowUs;

        // 1. Read IMU (with retry for I2C errors under ISR load)
        sensors_event_t a, g, tmp;
        bool ok = false;
        for (int i = 0; i < 3 && !ok; i++) {
            ok = mpu.getEvent(&a, &g, &tmp);
            if (!ok) { Wire.begin(21, 22); Wire.setClock(100000); delayMicroseconds(100); }
        }
        if (!ok) { imuOk = false; imuErrCount++; return; }
        imuOk = true;

        // 2. Complementary filter
        //
        //    Accelerometer angle: atan(az/ax) gives absolute tilt but
        //    is corrupted by longitudinal acceleration (ẍ/g error).
        //    → Accurate at LOW frequency (slow motion).
        //
        //    Gyroscope rate: accurate θ̇ but integrates drift.
        //    → Accurate at HIGH frequency (fast motion).
        //
        //    Filter crossover at fc = (1-C)/(2π·C·Δt) ≈ 0.48 Hz:
        //    below fc → accelerometer dominates (drift correction)
        //    above fc → gyro dominates (dynamic accuracy)
        //
        float accel_angle = atan2f(a.acceleration.z, a.acceleration.x);
        gyro_raw          = g.gyro.y;
        gyro_rate         = g.gyro.y - gyroBias;  // bias-corrected pitch rate

        theta = (1.0f - CF_COEFF) * accel_angle
              + CF_COEFF * (theta + gyro_rate * dt);

        // 3. Fall detection — disable motors if tipped too far
        if (fabsf(theta) > FALL_ANGLE) {
            step1.setTargetSpeedRad(0.0f);
            step2.setTargetSpeedRad(0.0f);
            integral         = 0.0f;
            velIntegral      = 0.0f;
            prevVelError     = 0.0f;
            velocitySetpoint = 0.0f;
            return;
        }

        // 4. Wheel speed — convert to cm/s then apply EMA
        float wheelSpeedAvg = (step2.getSpeedRad() - step1.getSpeedRad()) / 2.0f;
        float wheelSpeedCms = wheelSpeedAvg * WHEEL_RADIUS * 100.0f;
        filteredWheelSpeed  = wheelSpeedAlpha * wheelSpeedCms + (1.0f - wheelSpeedAlpha) * filteredWheelSpeed;

        // 4a. Outer velocity PID — runs at 1/VEL_LOOP_DIVIDER of the inner rate (50 ms)
        //     Slower outer loop gives the inner loop time to settle between commands,
        //     improving phase margin and reducing oscillation.
        static int velLoopCount = 0;
        if (++velLoopCount >= VEL_LOOP_DIVIDER) {
            velLoopCount = 0;
            float velDt    = dt * VEL_LOOP_DIVIDER;
            velError = velocitySetpoint - filteredWheelSpeed;
            float velDeriv = (velError - prevVelError) / velDt;
            prevVelError   = velError;
            float rawLean  = Kvp * velError + Kvi * velIntegral + Kvd * velDeriv;
            if (fabsf(rawLean) < MAX_TILT_OFFSET)  // pause integration when output is saturated
                velIntegral += velError * velDt;
            velIntegral = constrain(velIntegral, -MAX_VEL_INTEGRAL, MAX_VEL_INTEGRAL);
            leanCommand = constrain(rawLean, -MAX_TILT_OFFSET, MAX_TILT_OFFSET);
        }

        // 4c. Tilt setpoint
        float tiltSetpoint = BALANCE_ANGLE + leanCommand;

        // 5. PID
        //
        //    error = setpoint − θ
        //
        //    P term: proportional restoring force.
        //
        //    I term: corrects residual steady-state lean.
        //    NOTE: prefer trimming BALANCE_ANGLE over using large Ki.
        //    The s-zero cancellation means Ki safely shifts ωn upward
        //    without changing ζ or system order.
        //
        //    D term: uses raw gyro rate directly.
        //    d(error)/dt = d(setpoint−θ)/dt ≈ −θ̇ = −gyro_rate
        //    This avoids numerical differentiation noise entirely.
        //
        float error = tiltSetpoint - theta;

        integral += error * dt;
        integral  = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);  // anti-windup

        float P_term = Kp * error;
        float I_term = Ki * integral;
        float D_term = Kd * (-gyro_rate);  // negative: d(error)/dt = -θ̇

        float output = P_term + I_term + D_term;

        // Clamp to hardware maximum
        // Beyond 19.6 rad/s the stepper driver simply won't go faster.
        // Clamping here keeps the integral from winding up against a
        // limit the motor cannot achieve.
        output = constrain(output, -maxWheelSpeed, maxWheelSpeed);

        // 6. Drive motors
        //    Motor 2 is mounted mirrored → opposite sign.
        //    turnBias added to both: because step2 is already inverted,
        //    this creates a differential that turns the robot.
        step1.setTargetSpeedRad( output + turnBias);
        step2.setTargetSpeedRad(-output + turnBias);
    }

    // ── Diagnostics at 2 Hz ───────────────────────────────────────
    if (millis() - printTimer >= PRINT_INTERVAL_MS) {
        printTimer += PRINT_INTERVAL_MS;
        Serial.printf("theta=%.4f  gyro=%.3f  err=%.4f  w1=%.2f  "
                      "Kp=%.1f  Kd=%.1f  Ki=%.3f  ac=%.1f  mw=%.1f  sp=%.4f\n",
                      theta, gyro_rate, BALANCE_ANGLE - theta,
                      step1.getSpeedRad(), Kp, Kd, Ki, motorAccel, maxWheelSpeed, BALANCE_ANGLE);
    }
}