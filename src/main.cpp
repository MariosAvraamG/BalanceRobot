#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

float Kp = 2100.0f;  // saturasting — sign determines motor ramp direction
float Kd =  240.0f;  // gyro braking — primary tuning parameter
float Ki =    1.0f;  // small steady-state trim


float BALANCE_ANGLE = 0.06f;  // rad — calibrate per Phase 0 above

float CF_COEFF = 0.996f;

float       maxWheelSpeed    = 19.0f;  // rad/s — hardware ceiling (tunable: mw)
float       motorAccel       = 30.0f; // rad/s² — stepper slew rate  (tunable: ac)
const float MAX_INTEGRAL     = 0.1f;   // anti-windup clamp
const float FALL_ANGLE       = 0.4f;   // rad (~23°) — give up balancing


float MAX_TILT_SP    = 0.1f;  // outer loop output clamp (rad)
float EMA_ALPHA      = 0.90f;    // velEst smoothing (0=frozen, 1=raw)
float Kp_vel         = 0.005f;  // velocity P gain: velErr (rad/s) → tiltSP (rad)
float Ki_vel         = 0.001f;  // velocity I gain
float VEL_STEP       = 1.0f;    // rad/s per button press
float MAX_VEL_TARGET = 9.5f;    // rad/s ceiling on velTarget
float TURN_STEP      = 1.0f;    // rad/s added to turnBias per A/D press
float MAX_TURN_BIAS  = 3.0f;    // rad/s — turnBias ceiling

float velTarget   = 0.0f;  // commanded velocity (rad/s)
float velIntegral = 0.0f;  // velocity I accumulator
float tiltSP      = 0.0f;  // outer loop output: lean offset fed to inner PID (rad)
float turnBias    = 0.0f;  // yaw rate setpoint (rad/s); + = right

float Kp_yaw        = 0.180f;   // yaw P gain
float Ki_yaw        = 0.0100f;   // yaw I gain — tune after Kp is stable
float Kd_yaw        = 0.0230f;   // yaw D gain — differentiates filtered yaw_rate
float YAW_EMA_ALPHA = 0.90f;   // gyro.z EMA smoothing (0=frozen, 1=raw)


const int   LOOP_INTERVAL_MS    = 5;      // ms
const float LOOP_INTERVAL_S     = 0.005f; // s
const int   STEPPER_INTERVAL_US = 50;     // µs — 20 kHz ISR
const int   PRINT_INTERVAL_MS   = 2000;   // ms

// ─────────────────────────────────────────────────────────────────
//  Pins
// ─────────────────────────────────────────────────────────────────
const int STEPPER1_DIR_PIN  = 27;
const int STEPPER1_STEP_PIN = 26;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;

// ─────────────────────────────────────────────────────────────────
//  Live telemetry globals (read by web /status endpoint)
// ─────────────────────────────────────────────────────────────────
float    theta       = 0.0f;
float    gyro_rate   = 0.0f;
float    gyro_raw    = 0.0f;   // raw (un-biased) gyro reading
float    integral    = 0.0f;   // PID integral — global so calibration can reset it
float    velEst      = 0.0f;   // EMA-filtered wheel speed (rad/s) from getSpeedRad()
bool          imuOk       = true;
uint32_t      imuErrCount = 0;
uint32_t      lastCalibMs   = 0;
uint32_t      lastTurnCmdMs = 0;
volatile bool calibrating = false;  // raised by web handler; main loop yields I2C
bool          fallen      = false;  // true while tipped past FALL_ANGLE
unsigned long lastLoopUs  = 0;      // tracks micros() of last control tick
float gyroBiasZ   = 0.0f;  // gyro.z offset measured at calibration
float yaw_rate    = 0.0f;  // EMA-filtered bias-corrected gyro.z (rad/s) — telemetry
float yawCorrection = 0.0f; // yaw controller output applied to motors — telemetry
float yawIntegral   = 0.0f; // yaw I accumulator
float prevYawRate   = 0.0f; // previous yaw_rate for D term
unsigned long lastEspNowMs  = 0; // timestamp of last ESP-NOW command (0 = never received)
unsigned long lastUartMs    = 0; // timestamp of last UART packet from Pi (0 = never)
float         uartLinear    = 0.0f;
float         uartAngular   = 0.0f;

// ─────────────────────────────────────────────────────────────────
//  Web tuner
// ─────────────────────────────────────────────────────────────────
#include "web_ui.h"

WebServer server(80);

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
//  Shared command struct — used by both ESP-NOW and UART (Pi) paths.
//  UART framing: 0xAA 0x55 + 8 bytes (two little-endian floats).
// ─────────────────────────────────────────────────────────────────
#define UART_RX_PIN 16   // Pi TX → ESP32 GPIO16

typedef struct {
    float linear_vel;   // rad/s → velTarget  (wheel angular speed)
    float angular_vel;  // rad/s → turnBias   (yaw rate setpoint)
} EspNowCmd;

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < (int)sizeof(EspNowCmd)) return;
    EspNowCmd cmd;
    memcpy(&cmd, data, sizeof(cmd));
    velTarget     = constrain(cmd.linear_vel,  -MAX_VEL_TARGET, MAX_VEL_TARGET);
    turnBias      = constrain(cmd.angular_vel, -MAX_TURN_BIAS,  MAX_TURN_BIAS);
    lastTurnCmdMs = millis();
    lastEspNowMs  = millis();
}

void calibrate()
{
    float gyroSum  = 0.0f;
    float gyroZSum = 0.0f;
    float accelSum = 0.0f;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        sensors_event_t a, g, tmp;
        mpu.getEvent(&a, &g, &tmp);
        gyroSum  += g.gyro.y;
        gyroZSum += g.gyro.z;
        accelSum += atan2f(a.acceleration.z, a.acceleration.x);
        delay(5);
    }

    gyroBias      = gyroSum  / N;
    gyroBiasZ     = gyroZSum / N;
    BALANCE_ANGLE = accelSum / N;
    yaw_rate      = 0.0f;
    yawCorrection = 0.0f;
    yawIntegral   = 0.0f;
    prevYawRate   = 0.0f;
    fallen        = false;
    lastCalibMs   = millis();
    Serial.printf("Calibrated — bias_y=%.4f  bias_z=%.4f  balance=%.4f rad (%.2f deg)\n",
                  gyroBias, gyroBiasZ, BALANCE_ANGLE, BALANCE_ANGLE * 180.0f / PI);
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, -1);  // Raspberry Pi UART RX
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
    WiFi.softAP("BalanceBot2", "balance123");
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("Web tuner: connect to WiFi 'BalanceBot2' then open http://%s\n",
                  WiFi.softAPIP().toString().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
    } else {
        esp_now_register_recv_cb(onEspNowRecv);
        Serial.printf("ESP-NOW ready — target this AP MAC on sender: %s  channel: %d\n",
                      WiFi.softAPmacAddress().c_str(), WiFi.channel());
    }

    server.on("/", [](){
        server.send_P(200, "text/html", HTML);
    });
    server.on("/set", [](){
        if (server.hasArg("kp"))  Kp            = server.arg("kp").toFloat();
        if (server.hasArg("kd"))  Kd            = server.arg("kd").toFloat();
        if (server.hasArg("ki"))  Ki            = server.arg("ki").toFloat();
        if (server.hasArg("sp"))  BALANCE_ANGLE  = server.arg("sp").toFloat();
        if (server.hasArg("kpv")) Kp_vel         = server.arg("kpv").toFloat();
        if (server.hasArg("kvi")) Ki_vel         = server.arg("kvi").toFloat();
        if (server.hasArg("mts")) MAX_TILT_SP    = server.arg("mts").toFloat();
        if (server.hasArg("vs"))  VEL_STEP       = server.arg("vs").toFloat();
        if (server.hasArg("mvt")) MAX_VEL_TARGET = server.arg("mvt").toFloat();
        if (server.hasArg("ema")) EMA_ALPHA      = constrain(server.arg("ema").toFloat(), 0.01f, 1.0f);
        if (server.hasArg("trns")) TURN_STEP     = server.arg("trns").toFloat();
        if (server.hasArg("mtb")) MAX_TURN_BIAS  = server.arg("mtb").toFloat();
        if (server.hasArg("mw"))  maxWheelSpeed  = server.arg("mw").toFloat();
        if (server.hasArg("cf"))  CF_COEFF       = constrain(server.arg("cf").toFloat(), 0.0f, 0.9999f);
        if (server.hasArg("kyp")) Kp_yaw         = server.arg("kyp").toFloat();
        if (server.hasArg("kiy")) Ki_yaw         = server.arg("kiy").toFloat();
        if (server.hasArg("kdy")) Kd_yaw         = server.arg("kdy").toFloat();
        if (server.hasArg("yea")) YAW_EMA_ALPHA  = constrain(server.arg("yea").toFloat(), 0.01f, 1.0f);
        if (server.hasArg("ac")) {
            motorAccel = server.arg("ac").toFloat();
            step1.setAccelerationRad(motorAccel);
            step2.setAccelerationRad(motorAccel);
        }
        server.send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/move", [](){
        String dir = server.arg("dir");
        if      (dir == "w")  velTarget = constrain(velTarget + VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET);
        else if (dir == "s")  velTarget = constrain(velTarget - VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET);
        else if (dir == "a")  { turnBias = constrain(turnBias - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); lastTurnCmdMs = millis(); }
        else if (dir == "d")  { turnBias = constrain(turnBias + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); lastTurnCmdMs = millis(); }
        else if (dir == "wa") { velTarget = constrain(velTarget + VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); turnBias = constrain(turnBias - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); lastTurnCmdMs = millis(); }
        else if (dir == "wd") { velTarget = constrain(velTarget + VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); turnBias = constrain(turnBias + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); lastTurnCmdMs = millis(); }
        else if (dir == "sa") { velTarget = constrain(velTarget - VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); turnBias = constrain(turnBias - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); lastTurnCmdMs = millis(); }
        else if (dir == "sd") { velTarget = constrain(velTarget - VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); turnBias = constrain(turnBias + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); lastTurnCmdMs = millis(); }
        else if (dir == "stop")      { velTarget = 0.0f; tiltSP = 0.0f; integral = 0.0f; velIntegral = 0.0f; turnBias = 0.0f; yawIntegral = 0.0f; }
        else if (dir == "stop_fb")   { velTarget = 0.0f; integral = 0.0f; velIntegral = 0.0f; }
        else if (dir == "stop_turn") { turnBias  = 0.0f; yawIntegral = 0.0f; }
        server.send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/calibrate", [](){
        step1.setTargetSpeedRad(0.0f);
        step2.setTargetSpeedRad(0.0f);
        tiltSP   = 0.0f;
        turnBias = 0.0f;
        integral = 0.0f;
        calibrating = true;   // pause control-loop I2C access (core 1)
        delay(300);           // let motors coast; main loop sees flag within one 5 ms tick
        Serial.println("Web calibration — hold robot upright and still...");
        calibrate();
        velTarget   = 0.0f;
        velIntegral = 0.0f;
        velEst      = 0.0f;
        lastLoopUs  = 0;      // force dt=LOOP_INTERVAL_S on first tick after resume
        calibrating = false;
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"ok\":true,\"sp\":%.4f}", BALANCE_ANGLE);
        server.send(200, "application/json", buf);
    });
    server.on("/status", [](){
        uint32_t upSec    = millis() / 1000;
        uint32_t calSec   = lastCalibMs ? upSec - lastCalibMs / 1000 : 0;
        char buf[740];
        snprintf(buf, sizeof(buf),
            "{\"theta\":%.4f,\"setpt\":%.4f,\"gyro\":%.4f,\"err\":%.4f,\"spd\":%.2f,"
            "\"kp\":%.1f,\"kd\":%.1f,\"ki\":%.4f,\"sp\":%.4f,\"ac\":%.1f,\"mw\":%.1f,"
            "\"bias\":%.4f,\"raw\":%.4f,\"imu_ok\":%d,\"imu_err\":%lu,\"cal_s\":%lu,\"cf\":%.3f,"
            "\"velEst\":%.3f,\"velTarget\":%.3f,\"tiltSP\":%.4f,\"vint\":%.4f,"
            "\"kpv\":%.4f,\"kvi\":%.5f,\"mts\":%.3f,\"vs\":%.1f,\"mvt\":%.1f,\"ema\":%.2f,\"trns\":%.1f,\"mtb\":%.1f,"
            "\"yaw_rate\":%.4f,\"yawCorr\":%.4f,\"yawInt\":%.4f,\"turnBias\":%.3f,\"kyp\":%.4f,\"kiy\":%.5f,\"kdy\":%.4f,\"yea\":%.2f,\"biasZ\":%.4f}",
            theta, BALANCE_ANGLE + tiltSP, gyro_rate, BALANCE_ANGLE - theta, step1.getSpeedRad(),
            Kp, Kd, Ki, BALANCE_ANGLE, motorAccel, maxWheelSpeed,
            gyroBias, gyro_raw, (int)imuOk, imuErrCount, calSec, CF_COEFF,
            velEst, velTarget, tiltSP, velIntegral,
            Kp_vel, Ki_vel, MAX_TILT_SP, VEL_STEP, MAX_VEL_TARGET, EMA_ALPHA, TURN_STEP, MAX_TURN_BIAS,
            yaw_rate, yawCorrection, yawIntegral, turnBias, Kp_yaw, Ki_yaw, Kd_yaw, YAW_EMA_ALPHA, gyroBiasZ);
        server.send(200, "application/json", buf);
    });
    server.begin();
    xTaskCreatePinnedToCore(
        [](void*){ for(;;){ server.handleClient(); vTaskDelay(1); } },
        "web", 8192, nullptr, 1, nullptr, 0);

    Serial.println("Calibrating — hold robot upright and still for 1 second...");
    calibrate();
    velTarget   = 0.0f;
    velIntegral = 0.0f;
    velEst      = 0.0f;
}

// ─────────────────────────────────────────────────────────────────
//  Main loop
// ─────────────────────────────────────────────────────────────────
void loop()
{
    static unsigned long loopTimer  = 0;
    static unsigned long printTimer = 0;
    static unsigned long uartDiagTimer = 0;

    // ── UART (Raspberry Pi) command parser ────────────────────────
    // Packet: 0xAA 0x55 + sizeof(EspNowCmd) bytes, same dead-man as ESP-NOW.
    {
        static enum : uint8_t { HUNT_A, HUNT_B, READ_PAYLOAD } uartState = HUNT_A;
        static uint8_t uartBuf[sizeof(EspNowCmd)];
        static uint8_t uartIdx = 0;
        while (Serial2.available()) {
            uint8_t b = Serial2.read();
            switch (uartState) {
                case HUNT_A:   if (b == 0xAA) uartState = HUNT_B; break;
                case HUNT_B:
                    if      (b == 0x55) { uartIdx = 0; uartState = READ_PAYLOAD; }
                    else if (b != 0xAA)  uartState = HUNT_A;
                    break;
                case READ_PAYLOAD:
                    uartBuf[uartIdx++] = b;
                    if (uartIdx == sizeof(EspNowCmd)) {
                        EspNowCmd cmd;
                        memcpy(&cmd, uartBuf, sizeof(cmd));
                        velTarget     = constrain(cmd.linear_vel,  -MAX_VEL_TARGET, MAX_VEL_TARGET);
                        turnBias      = constrain(cmd.angular_vel, -MAX_TURN_BIAS,  MAX_TURN_BIAS);
                        lastTurnCmdMs = millis();
                        lastEspNowMs  = millis();
                        lastUartMs    = millis();
                        uartLinear    = cmd.linear_vel;
                        uartAngular   = cmd.angular_vel;
                        uartState = HUNT_A;
                    }
                    break;
            }
        }
    }

    // ── Serial command parser ──────────────────────────────────────
    if (Serial.available()) {
        char peek = (char)Serial.peek();

        // Single-character WASD commands — no Enter needed
        if (peek=='w'||peek=='W'||peek=='s'||peek=='S'||
            peek=='a'||peek=='A'||peek=='d'||peek=='D'||peek==' ') {
            Serial.read();  // consume
            switch (tolower(peek)) {
                case 'w': velTarget = constrain(velTarget + VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); break;
                case 's': velTarget = constrain(velTarget - VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); break;
                case 'a': turnBias = constrain(turnBias - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); break;
                case 'd': turnBias = constrain(turnBias + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS); break;
                case ' ': { velTarget = 0.0f; tiltSP = 0.0f; turnBias = 0.0f; integral = 0.0f; } break;
            }
            Serial.printf("MOVE  velTarget=%.2f  velEst=%.2f  turn=%.2f\n", velTarget, velEst, turnBias);
        } else {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if      (cmd.startsWith("kpv"))  Kp_vel         = cmd.substring(4).toFloat();
            else if (cmd.startsWith("kvi"))  Ki_vel         = cmd.substring(4).toFloat();
            else if (cmd.startsWith("kyp"))  Kp_yaw         = cmd.substring(4).toFloat();
            else if (cmd.startsWith("kiy"))  Ki_yaw         = cmd.substring(4).toFloat();
            else if (cmd.startsWith("kdy"))  Kd_yaw         = cmd.substring(4).toFloat();
            else if (cmd.startsWith("yea"))  YAW_EMA_ALPHA  = constrain(cmd.substring(4).toFloat(), 0.01f, 1.0f);
            else if (cmd.startsWith("kp"))   Kp             = cmd.substring(3).toFloat();
            else if (cmd.startsWith("kd"))   Kd             = cmd.substring(3).toFloat();
            else if (cmd.startsWith("ki"))   Ki             = cmd.substring(3).toFloat();
            else if (cmd.startsWith("sp"))   BALANCE_ANGLE  = cmd.substring(3).toFloat();
            else if (cmd.startsWith("ac")) { motorAccel     = cmd.substring(3).toFloat();
                                             step1.setAccelerationRad(motorAccel);
                                             step2.setAccelerationRad(motorAccel); }
            else if (cmd.startsWith("mts"))  MAX_TILT_SP    = cmd.substring(4).toFloat();
            else if (cmd.startsWith("mvt"))  MAX_VEL_TARGET = cmd.substring(4).toFloat();
            else if (cmd.startsWith("mw"))   maxWheelSpeed  = cmd.substring(3).toFloat();
            else if (cmd.startsWith("vs"))   VEL_STEP       = cmd.substring(3).toFloat();
            else if (cmd.startsWith("vt"))   velTarget      = constrain(cmd.substring(3).toFloat(), -MAX_VEL_TARGET, MAX_VEL_TARGET);
            else if (cmd.startsWith("trns"))  TURN_STEP      = cmd.substring(5).toFloat();
            else if (cmd.startsWith("tr"))   turnBias       = constrain(cmd.substring(3).toFloat(), -MAX_TURN_BIAS, MAX_TURN_BIAS);
            else if (cmd == "en 0")          { digitalWrite(STEPPER_EN_PIN, HIGH); Serial.println("Motors DISABLED"); return; }
            else if (cmd == "en 1")          { digitalWrite(STEPPER_EN_PIN, LOW);  Serial.println("Motors ENABLED");  return; }

            Serial.printf("Kp=%.1f  Kd=%.1f  Ki=%.3f  Kpv=%.4f  Kvi=%.5f  sp=%.4f  ac=%.1f  mw=%.1f\n",
                          Kp, Kd, Ki, Kp_vel, Ki_vel, BALANCE_ANGLE, motorAccel, maxWheelSpeed);
        }
    }

    // ── Control loop at 200 Hz ────────────────────────────────────
    if (millis() - loopTimer >= LOOP_INTERVAL_MS) {
        loopTimer += LOOP_INTERVAL_MS;

        if (calibrating) return;  // yield I2C bus to calibration running on core 0

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

        // Yaw rate: bias-correct gyro.z, negate so clockwise = positive, then EMA
        float raw_yaw = -(g.gyro.z - gyroBiasZ);
        yaw_rate = YAW_EMA_ALPHA * raw_yaw + (1.0f - YAW_EMA_ALPHA) * yaw_rate;

        theta = (1.0f - CF_COEFF) * accel_angle
              + CF_COEFF * (theta + gyro_rate * dt);

        // 3. Fall detection
        if (fabsf(theta) > FALL_ANGLE) fallen = true;

        if (fallen) {
            theta = accel_angle;  // bypass CF — snap to accelerometer
            if (fabsf(theta) >= FALL_ANGLE) {
                step1.setTargetSpeedRad(0.0f);
                step2.setTargetSpeedRad(0.0f);
                velTarget   = 0.0f;
                velIntegral = 0.0f;
                velEst      = 0.0f;
                tiltSP      = 0.0f;
                integral    = 0.0f;
                yawIntegral = 0.0f;
                prevYawRate = 0.0f;
                return;
            }
            // accel says we're upright — clear fallen state and resume on this tick
            fallen      = false;
            velTarget   = 0.0f;
            velIntegral = 0.0f;
            velEst      = 0.0f;
            tiltSP      = 0.0f;
            integral    = 0.0f;
            yawIntegral = 0.0f;
            prevYawRate = 0.0f;
        }

        // 4. Tilt setpoint
        float tiltSetpoint = BALANCE_ANGLE + tiltSP;

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

        // 6. Yaw PID controller
        //    turnBias is the yaw rate setpoint (rad/s).
        //    D term differentiates the EMA-filtered yaw_rate directly,
        //    which is smoother than differentiating raw gyro.z.
        //    Sign: d(error)/dt = -d(yaw_rate)/dt → subtract yaw_accel.
        //    Conditional integration anti-windup: integrator only
        //    accumulates when the full PID output is within the clamp.
        {
            float headroom  = maxWheelSpeed - fabsf(output);
            float yawErr    = turnBias - yaw_rate;
            float yaw_accel = (yaw_rate - prevYawRate) / dt;
            prevYawRate     = yaw_rate;
            float rawCorr   = Kp_yaw * yawErr
                            + Ki_yaw * yawIntegral
                            - Kd_yaw * yaw_accel;
            if (fabsf(rawCorr) < headroom)
                yawIntegral += yawErr * dt;
            float maxYI    = (Ki_yaw > 1e-6f) ? headroom / Ki_yaw : 1000.0f;
            yawIntegral    = constrain(yawIntegral, -maxYI, maxYI);
            yawCorrection  = constrain(rawCorr, -headroom, headroom);
        }

        // 7. Drive motors
        //    Motor 2 is mounted mirrored → opposite sign.
        //    yawCorrection subtracted from both: because step2 is already
        //    negated, subtracting the same value drives the wheels in
        //    opposite physical directions, creating the correct yaw torque.
        step1.setTargetSpeedRad( output - yawCorrection);
        step2.setTargetSpeedRad(-output - yawCorrection);
    }


    // ── ESP-NOW dead-man: stop if link drops for >500 ms ─────────
    if (lastEspNowMs && millis() - lastEspNowMs > 500) {
        velTarget   = 0.0f;
        turnBias    = 0.0f;
        velIntegral = 0.0f;
        yawIntegral = 0.0f;
        lastEspNowMs = 0;
    }

    // ── Outer velocity PI loop at 20 Hz ──────────────────────────
    if (lastTurnCmdMs && turnBias != 0.0f && millis() - lastTurnCmdMs > 300) {
        turnBias    = 0.0f;
        yawIntegral = 0.0f;
    }

    static unsigned long outerTimer = 0;
    if (millis() - outerTimer >= 50) {
        outerTimer += 50;
        const float dt_outer = 0.05f;
        velEst = EMA_ALPHA * 0.5f * (step2.getSpeedRad() - step1.getSpeedRad())
               + (1.0f - EMA_ALPHA) * velEst;
        float velErr  = velTarget - velEst;
        float rawLean = Kp_vel * velErr + Ki_vel * velIntegral;
        if (fabsf(rawLean) < MAX_TILT_SP)
            velIntegral += velErr * dt_outer;
        float maxVI = (Ki_vel > 1e-6f) ? MAX_TILT_SP / Ki_vel : 1000.0f;
        velIntegral = constrain(velIntegral, -maxVI, maxVI);
        tiltSP = constrain(rawLean, -MAX_TILT_SP, MAX_TILT_SP);
    }

    // ── UART Pi diagnostics at 2 Hz ──────────────────────────────
    if (millis() - uartDiagTimer >= 500) {
        uartDiagTimer += 500;
        if (lastUartMs == 0) {
            Serial.println("UART[Pi]: no packet received yet");
        } else {
            Serial.printf("UART[Pi]: linear=%.3f  angular=%.3f  age=%lums\n",
                          uartLinear, uartAngular, millis() - lastUartMs);
        }
    }

    // ── Diagnostics at 2 Hz ───────────────────────────────────────
    if (millis() - printTimer >= PRINT_INTERVAL_MS) {
        printTimer += PRINT_INTERVAL_MS;
        Serial.printf("theta=%.4f  gyro=%.3f  velEst=%.3f  velTgt=%.3f  tiltSP=%.4f  vint=%.4f  "
                      "Kp=%.1f  Kd=%.1f  Ki=%.3f  Kpv=%.4f  Kvi=%.5f  ac=%.1f  mw=%.1f  sp=%.4f  "
                      "yaw=%.4f  yawCorr=%.4f  yawInt=%.4f  kyp=%.3f  kyi=%.5f  kyd=%.4f\n",
                      theta, gyro_rate, velEst, velTarget, tiltSP, velIntegral,
                      Kp, Kd, Ki, Kp_vel, Ki_vel, motorAccel, maxWheelSpeed, BALANCE_ANGLE,
                      yaw_rate, yawCorrection, yawIntegral, Kp_yaw, Ki_yaw, Kd_yaw);
    }
}