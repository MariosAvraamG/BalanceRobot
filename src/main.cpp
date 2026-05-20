
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

const float OMEGA_N = 25.0f;  // target bandwidth (rad/s) — tune upward
const float ZETA    = 0.7f;   // target damping ratio


float Kp = 2000.0f;  // saturating — sign determines motor ramp direction
float Kd =  450.0f;  // gyro braking — primary tuning parameter
float Ki =    1.0f;  // small steady-state trim


float BALANCE_ANGLE = 0.06f;  // rad — calibrate per Phase 0 above

const float CF_COEFF = 0.985f;

const float MAX_WHEEL_SPEED  = 19.0f;  // rad/s — hardware ceiling
const float MAX_INTEGRAL     = 5.0f;   // anti-windup clamp
const float FALL_ANGLE       = 0.8f;   // rad (~46°) — give up balancing


const float MAX_TILT_OFFSET = 0.15f;   // max lean command magnitude (rad, ~8.5°)
const float MOVE_STEP       = 0.01745f; // rad per W/S keypress (1°)
const float TURN_STEP       = 2.0f;    // rad/s per A/D keypress
const float MAX_TURN_BIAS   = 10.0f;   // rad/s

float leanCommand = 0.0f;   // tilt setpoint offset from BALANCE_ANGLE (rad); + = forward
float turnBias    = 0.0f;   // differential speed for turning (rad/s); + = right


const int   LOOP_INTERVAL_MS  = 5;           // ms
const float LOOP_INTERVAL_S   = 0.005f;      // s  (nominal — actual dt measured per iteration)
const int   STEPPER_INTERVAL_US = 50;        // µs — 20 kHz ISR
const int   PRINT_INTERVAL_MS = 2000;         // ms

// ─────────────────────────────────────────────────────────────────
//  Pins
// ─────────────────────────────────────────────────────────────────
const int STEPPER1_DIR_PIN  = 25;
const int STEPPER1_STEP_PIN = 26;
const int STEPPER2_DIR_PIN  = 27;
const int STEPPER2_STEP_PIN = 4;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;

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
    step1.setAccelerationRad(40.0f);
    step2.setAccelerationRad(40.0f);

    if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
        Serial.println("Stepper ISR attach failed");
        while (1) delay(10);
    }

    // ── Gyro bias calibration ─────────────────────────────────────
    // Average 200 readings at 5 ms intervals (1 second total).
    // Robot must be stationary during this window.
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
    static float theta     = 0.0f;
    static float integral  = 0.0f;
    static float gyro_rate = 0.0f;  // retained for diagnostics

    // ── Serial command parser ──────────────────────────────────────
    if (Serial.available()) {
        char peek = (char)Serial.peek();

        // Single-character WASD commands — no Enter needed
        if (peek=='w'||peek=='W'||peek=='s'||peek=='S'||
            peek=='a'||peek=='A'||peek=='d'||peek=='D'||peek==' ') {
            Serial.read();  // consume
            switch (tolower(peek)) {
                case 'w': leanCommand = constrain(leanCommand + MOVE_STEP, -MAX_TILT_OFFSET, MAX_TILT_OFFSET); break;
                case 's': leanCommand = constrain(leanCommand - MOVE_STEP, -MAX_TILT_OFFSET, MAX_TILT_OFFSET); break;
                case 'a': turnBias    = constrain(turnBias    - TURN_STEP,  -MAX_TURN_BIAS,  MAX_TURN_BIAS);   break;
                case 'd': turnBias    = constrain(turnBias    + TURN_STEP,  -MAX_TURN_BIAS,  MAX_TURN_BIAS);   break;
                case ' ': leanCommand = 0.0f; turnBias = 0.0f; break;
            }
            Serial.printf("MOVE  lean=%.4f  turn=%.2f\n", leanCommand, turnBias);
        } else {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if      (cmd.startsWith("kp"))   Kp = cmd.substring(3).toFloat();
            else if (cmd.startsWith("kd"))   Kd = cmd.substring(3).toFloat();
            else if (cmd.startsWith("ki"))   Ki = cmd.substring(3).toFloat();
            else if (cmd.startsWith("sp"))   BALANCE_ANGLE = cmd.substring(3).toFloat();
            else if (cmd.startsWith("mv"))   leanCommand = constrain(cmd.substring(3).toFloat(), -MAX_TILT_OFFSET, MAX_TILT_OFFSET);
            else if (cmd.startsWith("tr"))   turnBias    = constrain(cmd.substring(3).toFloat(), -MAX_TURN_BIAS,   MAX_TURN_BIAS);
            else if (cmd == "en 0")          { digitalWrite(STEPPER_EN_PIN, HIGH); Serial.println("Motors DISABLED"); return; }
            else if (cmd == "en 1")          { digitalWrite(STEPPER_EN_PIN, LOW);  Serial.println("Motors ENABLED");  return; }
           

    
             
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
        if (!ok) return;  // skip this iteration rather than use stale data

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
        gyro_rate         = g.gyro.y - gyroBias;  // bias-corrected pitch rate

        theta = (1.0f - CF_COEFF) * accel_angle
              + CF_COEFF * (theta + gyro_rate * dt);

        // 3. Fall detection — disable motors if tipped too far
        if (fabsf(theta) > FALL_ANGLE) {
            step1.setTargetSpeedRad(0.0f);
            step2.setTargetSpeedRad(0.0f);
            integral = 0.0f;
            return;
        }

        // 4. Tilt setpoint — direct lean command, no wheel speed feedback
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
        output = constrain(output, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

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
        Serial.printf("theta=%.4f  gyro=%.3f  err=%.4f  "
                      "ω1=%.2f  Kp=%.1f  Kd=%.1f  Ki=%.3f  ωn=%.2f  ζ=%.3f  sp=%.4f\n",
                      theta, gyro_rate, BALANCE_ANGLE - theta,
                      step1.getSpeedRad(), Kp, Kd, Ki,BALANCE_ANGLE);
    }
}