#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
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
const float FALL_ANGLE       = 0.8f;   // rad (~46°) — give up balancing


float MAX_TILT_SP    = 0.1f;  // outer loop output clamp (rad)
float EMA_ALPHA      = 0.90f;    // velEst smoothing (0=frozen, 1=raw)
float Kp_vel         = 0.005f;  // velocity P gain: velErr (rad/s) → tiltSP (rad)
float Ki_vel         = 0.001f;  // velocity I gain
float VEL_STEP       = 1.0f;    // rad/s per button press
float MAX_VEL_TARGET = 15.0f;    // rad/s ceiling on velTarget
float TURN_STEP      = 2.0f;    // rad/s added to turnBias per A/D press
float MAX_TURN_BIAS  = 4.0f;    // rad/s — turnBias ceiling

float velTarget   = 0.0f;  // commanded velocity (rad/s)
float velIntegral = 0.0f;  // velocity I accumulator
float tiltSP      = 0.0f;  // outer loop output: lean offset fed to inner PID (rad)
float turnBias    = 0.0f;  // differential speed for turning (rad/s); + = right


const int   LOOP_INTERVAL_MS    = 5;      // ms
const float LOOP_INTERVAL_S     = 0.005f; // s
const int   STEPPER_INTERVAL_US = 50;     // µs — 20 kHz ISR
const int   PRINT_INTERVAL_MS   = 2000;   // ms

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
float    gyro_raw    = 0.0f;   // raw (un-biased) gyro reading
float    integral    = 0.0f;   // PID integral — global so calibration can reset it
float    velEst      = 0.0f;   // EMA-filtered wheel speed (rad/s) from getSpeedRad()
bool     imuOk       = true;
uint32_t imuErrCount = 0;
uint32_t lastCalibMs = 0;

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
//  Gyro bias + balance angle calibration (200 samples, ~1 s)
//  Robot must be stationary and upright during the sampling window.
// ─────────────────────────────────────────────────────────────────
void calibrate()
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
    
    gyroBias      = gyroSum  / N;
    BALANCE_ANGLE = accelSum / N;
    lastCalibMs   = millis();
    Serial.printf("Calibrated — bias=%.4f  balance=%.4f rad (%.2f deg)\n",
                  gyroBias, BALANCE_ANGLE, BALANCE_ANGLE * 180.0f / PI);
}

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
    WiFi.softAP("BalanceBot2", "balance123");
    Serial.printf("Web tuner: connect to WiFi 'BalanceBot' then open http://%s\n",
                  WiFi.softAPIP().toString().c_str());

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
        if (server.hasArg("mw")) maxWheelSpeed = server.arg("mw").toFloat();
        if (server.hasArg("cf")) CF_COEFF      = constrain(server.arg("cf").toFloat(), 0.0f, 0.9999f);
        if (server.hasArg("ac")) {
            motorAccel = server.arg("ac").toFloat();
            step1.setAccelerationRad(motorAccel);
            step2.setAccelerationRad(motorAccel);
        }
        server.send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/move", [](){
        String dir = server.arg("dir");
        if      (dir == "w")         velTarget = constrain(velTarget + VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET);
        else if (dir == "s")         velTarget = constrain(velTarget - VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET);
        else if (dir == "a")         turnBias  = constrain(turnBias - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);
        else if (dir == "d")         turnBias  = constrain(turnBias + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);
        else if (dir == "stop")    { velTarget = 0.0f; tiltSP = 0.0f; integral = 0.0f; turnBias = 0.0f; }
        else if (dir == "stop_fb") { velTarget = 0.0f; integral = 0.0f; }
        else if (dir == "stop_turn") turnBias  = 0.0f;
        server.send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/calibrate", [](){
        step1.setTargetSpeedRad(0.0f);
        step2.setTargetSpeedRad(0.0f);
        tiltSP   = 0.0f;
        turnBias = 0.0f;
        integral = 0.0f;
        delay(300);  // let motors coast to stop before sampling
        Serial.println("Web calibration — hold robot upright and still...");
        calibrate();
        velTarget   = 0.0f;
        velIntegral = 0.0f;
        velEst      = 0.0f;
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"ok\":true,\"sp\":%.4f}", BALANCE_ANGLE);
        server.send(200, "application/json", buf);
    });
    server.on("/status", [](){
        uint32_t upSec    = millis() / 1000;
        uint32_t calSec   = lastCalibMs ? upSec - lastCalibMs / 1000 : 0;
        char buf[560];
        snprintf(buf, sizeof(buf),
            "{\"theta\":%.4f,\"setpt\":%.4f,\"gyro\":%.4f,\"err\":%.4f,\"spd\":%.2f,"
            "\"kp\":%.1f,\"kd\":%.1f,\"ki\":%.4f,\"sp\":%.4f,\"ac\":%.1f,\"mw\":%.1f,"
            "\"bias\":%.4f,\"raw\":%.4f,\"imu_ok\":%d,\"imu_err\":%lu,\"cal_s\":%lu,\"cf\":%.3f,"
            "\"velEst\":%.3f,\"velTarget\":%.3f,\"tiltSP\":%.4f,\"vint\":%.4f,"
            "\"kpv\":%.4f,\"kvi\":%.5f,\"mts\":%.3f,\"vs\":%.1f,\"mvt\":%.1f,\"ema\":%.2f,\"trns\":%.1f,\"mtb\":%.1f}",
            theta, BALANCE_ANGLE + tiltSP, gyro_rate, BALANCE_ANGLE - theta, step1.getSpeedRad(),
            Kp, Kd, Ki, BALANCE_ANGLE, motorAccel, maxWheelSpeed,
            gyroBias, gyro_raw, (int)imuOk, imuErrCount, calSec, CF_COEFF,
            velEst, velTarget, tiltSP, velIntegral,
            Kp_vel, Ki_vel, MAX_TILT_SP, VEL_STEP, MAX_VEL_TARGET, EMA_ALPHA, TURN_STEP, MAX_TURN_BIAS);
        server.send(200, "application/json", buf);
    });
    server.begin();
    xTaskCreatePinnedToCore(
        [](void*){ for(;;){ server.handleClient(); vTaskDelay(1); } },
        "web", 4096, nullptr, 1, nullptr, 0);

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
    static unsigned long lastLoopUs = 0;

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
            else if (cmd.startsWith("tr"))   turnBias       = constrain(cmd.substring(3).toFloat(), -MAX_TURN_BIAS, MAX_TURN_BIAS);
            else if (cmd == "en 0")          { digitalWrite(STEPPER_EN_PIN, HIGH); Serial.println("Motors DISABLED"); return; }
            else if (cmd == "en 1")          { digitalWrite(STEPPER_EN_PIN, LOW);  Serial.println("Motors ENABLED");  return; }

            Serial.printf("Kp=%.1f  Kd=%.1f  Ki=%.3f  Kpv=%.4f  Kvi=%.5f  sp=%.4f  ac=%.1f  mw=%.1f\n",
                          Kp, Kd, Ki, Kp_vel, Ki_vel, BALANCE_ANGLE, motorAccel, maxWheelSpeed);
        }
    }

    // ── Control loop at 200 Hz ────────────────────────────────────
    if (millis() - loopTimer >= LOOP_INTERVAL_MS) {
        unsigned long nowUs = micros();
        float dt = (lastLoopUs == 0) ? LOOP_INTERVAL_S
                                     : constrain((nowUs - lastLoopUs) * 1e-6f, 0.001f, 0.020f);
        lastLoopUs = nowUs;
        loopTimer += LOOP_INTERVAL_MS;

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
            velTarget   = 0.0f;
            velIntegral = 0.0f;
            velEst      = 0.0f;
            tiltSP      = 0.0f;
            integral    = 0.0f;
            return;
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

        // 6. Drive motors
        //    Motor 2 is mounted mirrored → opposite sign.
        //    turnBias added to both: because step2 is already inverted,
        //    this creates a differential that turns the robot.
        step1.setTargetSpeedRad( output + turnBias);
        step2.setTargetSpeedRad(-output + turnBias);
    }


    // ── Outer velocity PI loop at 20 Hz ──────────────────────────
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

    // ── Diagnostics at 2 Hz ───────────────────────────────────────
    if (millis() - printTimer >= PRINT_INTERVAL_MS) {
        printTimer += PRINT_INTERVAL_MS;
        Serial.printf("theta=%.4f  gyro=%.3f  velEst=%.3f  velTgt=%.3f  tiltSP=%.4f  vint=%.4f  "
                      "Kp=%.1f  Kd=%.1f  Ki=%.3f  Kpv=%.4f  Kvi=%.5f  ac=%.1f  mw=%.1f  sp=%.4f\n",
                      theta, gyro_rate, velEst, velTarget, tiltSP, velIntegral,
                      Kp, Kd, Ki, Kp_vel, Ki_vel, motorAccel, maxWheelSpeed, BALANCE_ANGLE);
    }
}