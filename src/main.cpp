/***********************************************************************
 * Balance-Bot Inner Loop Controller
 *
 * ═══════════════════════════════════════════════════════════════════
 * PLANT MODEL
 * ═══════════════════════════════════════════════════════════════════
 *
 * The robot is an inverted pendulum. Small-angle EOM:
 *
 *   θ̈ = (g/l)·θ  −  (r/l)·ω̇_wheel
 *
 *   θ        = tilt angle (rad), positive = lean forward
 *   g        = 9.81 m/s²
 *   l        = CoM height above wheel axle (m)
 *   r        = wheel radius (m)
 *   ω_wheel  = wheel angular speed (rad/s)  ← our control input U
 *
 * Taking the Laplace transform and rearranging, the plant from
 * wheel SPEED command Ω(s) to tilt angle Θ(s) is:
 *
 *         Θ(s)         −(r/l)·s
 *   G(s) = ──── = ─────────────────
 *         Ω(s)      s²  −  g/l
 *
 * Key observations:
 *   • Unstable open-loop pole at  s = +√(g/l)
 *   • Zero at s = 0  (plant contributes NO DC gain)
 *
 * WHY SPEED AND NOT ACCELERATION?
 *   Stepper drivers accept step pulses → you control frequency → speed.
 *   setTargetSpeedRad() is the only available actuator interface.
 *   Commanding acceleration would require a double integrator plant,
 *   which has positive DC gain and is unstable under negative feedback.
 *   Speed output with G(s) above is stable under negative feedback
 *   provided Kd > l/r  (see stability analysis below).
 *
 * ═══════════════════════════════════════════════════════════════════
 * STEP LIBRARY — UNIT ANALYSIS
 * ═══════════════════════════════════════════════════════════════════
 *
 *   MICROSTEPS  = 16
 *   STEPS       = 200   (full steps per revolution)
 *   STEP_ANGLE  = 2π / (200×16) = 2π/3200 ≈ 1.963×10⁻³ rad/microstep
 *
 *   setTargetSpeedRad(ω):
 *     tSpeed = ω × SPEED_SCALE / STEP_ANGLE
 *            = ω × 2000 / 1.963e-3
 *            = ω × 1,018,591   (internal integer units)
 *
 *   getSpeedRad():
 *     returns  speed × STEP_ANGLE / SPEED_SCALE  → genuine rad/s ✓
 *
 *   MAX_SPEED = 10,000 steps/s
 *     → ω_max = 10000 × STEP_ANGLE = 10000 × 1.963e-3 ≈ 19.6 rad/s
 *
 *   The PID output must be constrained to ±19.6 rad/s.
 *   setAccelerationRad() is a slew-rate limiter only — NOT a control
 *   output. Set it high (≥1000 rad/s²) so it does not restrict PID.
 *
 * ═══════════════════════════════════════════════════════════════════
 * CLOSED-LOOP ANALYSIS  (negative feedback, PID)
 * ═══════════════════════════════════════════════════════════════════
 *
 *           Kd·s² + Kp·s + Ki
 *   C(s) = ──────────────────
 *                  s
 *
 * The s in G(s)'s numerator CANCELS the 1/s from the integrator:
 *
 *                   (r/l)·(Kd·s² + Kp·s + Ki)
 *   G(s)·C(s) = − ────────────────────────────
 *                         s²  −  g/l
 *
 * Characteristic equation  1 + G(s)·C(s) = 0 :
 *
 *   (s² − g/l)  −  (r/l)·(Kd·s² + Kp·s + Ki)  =  0
 *
 *   (1 − r·Kd/l)·s²  −  (r·Kp/l)·s  −  (g/l + r·Ki/l)  =  0
 *
 * Multiply through by  −l :
 *
 *   (r·Kd − l)·s²  +  r·Kp·s  +  (g + r·Ki)  =  0
 *
 * ─── Stability (Routh, all coefficients same sign & positive) ───
 *
 *   r·Kd − l  > 0   →   Kd  >  l/r          ← PRIMARY CONDITION
 *   r·Kp      > 0   →   Kp  >  0             ← trivially satisfied
 *   g + r·Ki  > 0   →   Ki  > −g/r ≈ −300   ← trivially satisfied
 *
 * ─── Standard second-order form ─────────────────────────────────
 *
 *   Dividing by (r·Kd − l):
 *
 *             r·Kp              g + r·Ki
 *   s²  +  ────────── · s  +  ─────────  =  0
 *           r·Kd − l           r·Kd − l
 *
 *   Natural frequency:    ωn  =  √( (g + r·Ki) / (r·Kd − l) )
 *
 *                               r·Kp
 *   Damping ratio:         ζ  = ──────────────────────────────
 *                               2·√( (r·Kd−l)·(g + r·Ki) )
 *
 * ─── Bandwidth requirement ───────────────────────────────────────
 *
 *   The open-loop unstable pole is at  p = √(g/l).
 *   Closed-loop bandwidth MUST satisfy:
 *
 *   ωn  >>  √(g/l)    (at least 3× for reliable stabilisation)
 *
 * ═══════════════════════════════════════════════════════════════════
 * DESIGN EQUATIONS  (Ki = 0 starting point)
 * ═══════════════════════════════════════════════════════════════════
 *
 *   Step 1 — Measure robot:  r (wheel radius),  l (CoM height)
 *
 *   Step 2 — Choose target bandwidth:
 *     ωn_target  ≥  3 × √(g/l)
 *
 *   Step 3 — Solve for Kd:
 *     r·Kd − l  =  g / ωn²
 *     Kd  =  ( g/ωn²  +  l ) / r
 *
 *   Step 4 — Solve for Kp  (target ζ = 0.7):
 *     Kp  =  2·ζ·g / (r·ωn)
 *
 *   Example with  r = 0.032 m,  l = 0.20 m,  ωn = 21 rad/s,  ζ = 0.7:
 *     Unstable pole:  √(9.81/0.20) = 7.0 rad/s  →  ωn = 3× = 21 rad/s
 *     Kd = (9.81/441 + 0.20) / 0.032  =  0.222/0.032  ≈  7.0
 *     Kp = 2×0.7×9.81 / (0.032×21)   =  13.73/0.672   ≈  20.4
 *
 *   IMPORTANT — These are theoretical starting values assuming ideal
 *   dynamics. In practice, sensor lag, loop delay (~10ms half-sample),
 *   and friction require higher gains. Use the tuning procedure below
 *   to scale up from these starting points. The RATIOS matter more
 *   than absolute values: maintain Kp / √(Kd) to preserve ζ.
 *
 * ═══════════════════════════════════════════════════════════════════
 * TUNING PROCEDURE
 * ═══════════════════════════════════════════════════════════════════
 *
 *  PHASE 0 — Physical measurements (do this before powering on)
 *  ─────────────────────────────────────────────────────────────
 *  a) Measure wheel radius r with calipers.
 *  b) Hold robot upright by hand, disable motors (EN pin HIGH).
 *     Read theta from serial — this is your REFERENCE_ANGLE.
 *     Set setpoint = this value. Now Ki is not needed.
 *  c) Estimate l: tape measure from axle centre to rough CoM.
 *
 *  PHASE 1 — Find minimum Kd (stability boundary)
 *  ────────────────────────────────────────────────
 *  Condition: Kd_min = l/r
 *  Example: l=0.20, r=0.032 → Kd_min = 6.25
 *  Start at Kd = Kd_min and Kp = 0.
 *  The robot will be marginally stable (sustained oscillation).
 *  This confirms your l/r estimate. If it falls immediately,
 *  increase Kd until oscillation is observed.
 *
 *  PHASE 2 — Increase bandwidth (raise Kd)
 *  ─────────────────────────────────────────
 *  ωn = √(g / (r·Kd − l))
 *  Increase Kd in ×2 steps until oscillation frequency RISES above
 *  roughly 3× your observed unstable pole frequency.
 *  Physical sign: robot tries to balance but oscillates on the spot.
 *
 *  PHASE 3 — Add damping (raise Kp)
 *  ──────────────────────────────────
 *  With Kd fixed, increase Kp to damp oscillations.
 *  Target: ζ = 0.7 → Kp = 2×0.7×√((r·Kd − l)·g) / r
 *  Physical sign: oscillation amplitude decreases; robot holds
 *  position briefly. Stop before it becomes sluggish.
 *
 *  PHASE 4 — Fine tune the ratio
 *  ───────────────────────────────
 *  Robot runs forward and falls → Kd too low relative to Kp.
 *    Increase Kd, or equivalently decrease Kp.
 *  Robot oscillates then falls → Kp too low relative to Kd.
 *    Increase Kp, keeping Kd fixed.
 *  Robot balances but drifts slowly → trim REFERENCE_ANGLE.
 *    Prefer this over adding Ki.
 *
 *  PHASE 5 — Add Ki (only if needed)
 *  ────────────────────────────────────
 *  Ki shifts ωn upward with NO effect on ζ (due to the s-zero
 *  cancellation). It is safe in small amounts.
 *  New ωn = √((g + r·Ki) / (r·Kd − l))
 *  Increase Ki slowly. If oscillations appear, Ki is too high.
 *  Anti-windup clamp (MAX_INTEGRAL) is essential.
 *
 * ═══════════════════════════════════════════════════════════════════
 * DERIVATIVE TERM — USE GYRO DIRECTLY
 * ═══════════════════════════════════════════════════════════════════
 *
 *  The derivative of tilt error is:
 *    d(error)/dt = d(setpoint − θ)/dt ≈ −θ̇  (setpoint is constant)
 *
 *  θ̇ is measured DIRECTLY by the gyroscope (g.gyro.y).
 *  Using −gyro_rate as the D term avoids numerical differentiation
 *  and the noise amplification that comes with it.
 *
 *  D_term = Kd × (−gyro_rate)
 *
 * ═══════════════════════════════════════════════════════════════════
 ***********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// ─────────────────────────────────────────────────────────────────
//  Physical parameters — MEASURE THESE ON YOUR ROBOT
// ─────────────────────────────────────────────────────────────────
//
//  Used to compute theoretical starting gains via:
//    Kd_start = (g / (OMEGA_N * OMEGA_N) + L_COM) / WHEEL_RADIUS
//    Kp_start = 2 * ZETA * g / (WHEEL_RADIUS * OMEGA_N)
//
const float WHEEL_RADIUS = 0.030f;   // r  (m) — measure with calipers
const float L_COM        = 0.15f;    // l  (m) — axle to CoM height
const float G_ACCEL      = 9.81f;    // g  (m/s²)

// ─────────────────────────────────────────────────────────────────
//  Controller design targets
// ─────────────────────────────────────────────────────────────────
//
//  Unstable pole:  p = √(g/l)
//  For l=0.15m:    p = √(9.81/0.15) = 8.09 rad/s
//  Target ωn must be >> p. Start at 3×p = 24.3 rad/s → rounded to 25.
//
const float OMEGA_N = 25.0f;  // target bandwidth (rad/s) — tune upward
const float ZETA    = 0.7f;   // target damping ratio

// ─────────────────────────────────────────────────────────────────
//  Derived theoretical starting gains  (Ki = 0)
//
//    Kd = (g/ωn² + l) / r
//    Kp = 2ζg / (r·ωn)
//
//  These are lower bounds. Sensor lag and loop delay will require
//  scaling both up together (preserve Kp/√Kd ratio to maintain ζ).
// ─────────────────────────────────────────────────────────────────
float Kp = 2000.0f;  // saturating — sign determines motor ramp direction
float Kd =  450.0f;  // gyro braking — primary tuning parameter
float Ki =    1.0f;  // small steady-state trim

// ─────────────────────────────────────────────────────────────────
//  Balance setpoint
//
//  HOW TO FIND IT:
//    1. Disable motors (comment out setTargetSpeedRad calls)
//    2. Hold robot perfectly upright by hand
//    3. Read theta from serial output
//    4. Set BALANCE_ANGLE to that value
//    This eliminates steady-state error without needing Ki.
// ─────────────────────────────────────────────────────────────────
float BALANCE_ANGLE = 0.06f;  // rad — calibrate per Phase 0 above

// ─────────────────────────────────────────────────────────────────
//  Complementary filter coefficient
//
//  C close to 1 → trusts gyro integration (accurate for fast motion)
//  (1-C) term → low-pass on accelerometer (corrects long-term drift)
//
//  Crossover frequency: fc = (1-C) / (2π·C·Δt)
//  At C=0.985, Δt=0.005s: fc = 0.015/(2π×0.985×0.005) ≈ 0.48 Hz
//  Below 0.48 Hz: accelerometer dominates (drift correction)
//  Above 0.48 Hz: gyro dominates (dynamic accuracy)
// ─────────────────────────────────────────────────────────────────
const float CF_COEFF = 0.985f;

// ─────────────────────────────────────────────────────────────────
//  Safety limits
// ─────────────────────────────────────────────────────────────────
//
//  MAX_WHEEL_SPEED: from step.h analysis:
//    MAX_SPEED = 10,000 steps/s
//    STEP_ANGLE = 2π/3200 ≈ 1.963e-3 rad/microstep
//    ω_max = 10000 × 1.963e-3 = 19.6 rad/s
//  Set software limit just below hardware max.
//
const float MAX_WHEEL_SPEED  = 19.0f;  // rad/s — hardware ceiling
const float MAX_INTEGRAL     = 5.0f;   // anti-windup clamp
const float FALL_ANGLE       = 0.8f;   // rad (~46°) — give up balancing

// ─────────────────────────────────────────────────────────────────
//  Timing
// ─────────────────────────────────────────────────────────────────
//
//  LOOP_INTERVAL = 5ms → 200 Hz control loop
//  This gives a half-sample delay of 2.5ms, limiting achievable
//  closed-loop bandwidth to roughly 1/(5×0.005) = 40 rad/s.
//  Keep ωn below this limit.
//
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
void printTheory()
{
    float unstable_pole = sqrtf(G_ACCEL / L_COM);
    float kd_min        = L_COM / WHEEL_RADIUS;
    float kd_theory     = (G_ACCEL / (OMEGA_N * OMEGA_N) + L_COM) / WHEEL_RADIUS;
    float kp_theory     = (2.0f * ZETA * G_ACCEL) / (WHEEL_RADIUS * OMEGA_N);
    float omega_n_check = sqrtf(G_ACCEL / (WHEEL_RADIUS * Kd - L_COM));
    float zeta_check    = (WHEEL_RADIUS * Kp) /
                          (2.0f * sqrtf((WHEEL_RADIUS * Kd - L_COM) * G_ACCEL));

    Serial.println("═══════════════════════════════════");
    Serial.println("  INNER LOOP THEORETICAL ANALYSIS  ");
    Serial.println("═══════════════════════════════════");
    Serial.printf("  r = %.4f m,  l = %.4f m\n", WHEEL_RADIUS, L_COM);
    Serial.printf("  Unstable pole  p  = √(g/l) = %.2f rad/s\n", unstable_pole);
    Serial.printf("  Stability cond Kd > l/r    = %.2f\n", kd_min);
    Serial.println("───────────────────────────────────");
    Serial.printf("  Target ωn = %.1f rad/s  (%.1fx pole)\n",
                  OMEGA_N, OMEGA_N / unstable_pole);
    Serial.printf("  Target ζ  = %.2f\n", ZETA);
    Serial.println("───────────────────────────────────");
    Serial.printf("  Kp theory = %.2f   →  loaded: %.2f\n", kp_theory, Kp);
    Serial.printf("  Kd theory = %.2f   →  loaded: %.2f\n", kd_theory, Kd);
    Serial.println("───────────────────────────────────");
    Serial.printf("  Actual ωn  = %.2f rad/s\n", omega_n_check);
    Serial.printf("  Actual ζ   = %.3f\n", zeta_check);
    Serial.printf("  Max wheel speed = 19.6 rad/s\n");
    Serial.println("═══════════════════════════════════");
    Serial.println("Serial commands:");
    Serial.println("  kp <val>  — set Kp");
    Serial.println("  kd <val>  — set Kd");
    Serial.println("  ki <val>  — set Ki");
    Serial.println("  sp <val>  — set balance setpoint (rad)");
    Serial.println("  en 0      — disable motors");
    Serial.println("  en 1      — enable motors");
    Serial.println("  info      — reprint this table");
    Serial.println("═══════════════════════════════════\n");
}

// ─────────────────────────────────────────────────────────────────
//  Setup
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
    // 21 Hz hardware filter: removes stepper vibration noise above
    // ~21 Hz while preserving the balance control bandwidth (≤40 rad/s ≈ 6 Hz)

    // Set acceleration high so the slew-rate limiter in the step
    // library does NOT restrict the PID. The PID sets target speed;
    // the library ramps to it. At 1000 rad/s², it reaches 19.6 rad/s
    // in ~20 ms — fast enough not to impede control.
    step1.setAccelerationRad(30.0f);
    step2.setAccelerationRad(30.0f);

    if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
        Serial.println("Stepper ISR attach failed");
        while (1) delay(10);
    }

    // ── Gyro bias calibration ─────────────────────────────────────
    // Average 200 readings at 5 ms intervals (1 second total).
    // Robot must be stationary during this window.
    Serial.println("Calibrating gyro — keep robot still for 1 second...");
    {
        float sum = 0.0f;
        const int N = 200;
        for (int i = 0; i < N; i++) {
            sensors_event_t a, g, tmp;
            mpu.getEvent(&a, &g, &tmp);
            sum += g.gyro.y;
            delay(5);
        }
        gyroBias = sum / N;
        Serial.printf("Gyro bias measured: %.4f rad/s\n", gyroBias);
    }

    printTheory();
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
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if      (cmd.startsWith("kp"))   Kp = cmd.substring(3).toFloat();
        else if (cmd.startsWith("kd"))   Kd = cmd.substring(3).toFloat();
        else if (cmd.startsWith("ki"))   Ki = cmd.substring(3).toFloat();
        else if (cmd.startsWith("sp"))   BALANCE_ANGLE = cmd.substring(3).toFloat();
        else if (cmd == "en 0")          { digitalWrite(STEPPER_EN_PIN, HIGH); Serial.println("Motors DISABLED"); return; }
        else if (cmd == "en 1")          { digitalWrite(STEPPER_EN_PIN, LOW);  Serial.println("Motors ENABLED");  return; }
        else if (cmd == "info")          { printTheory(); return; }

        // Print updated actual ωn and ζ after any gain change
        float rKd_l = WHEEL_RADIUS * Kd - L_COM;
        if (rKd_l > 0) {
            float wn = sqrtf((G_ACCEL + WHEEL_RADIUS * Ki) / rKd_l);
            float z  = (WHEEL_RADIUS * Kp) / (2.0f * sqrtf(rKd_l * (G_ACCEL + WHEEL_RADIUS * Ki)));
            Serial.printf("Kp=%.2f  Kd=%.2f  Ki=%.3f  sp=%.4f  |  ωn=%.2f  ζ=%.3f\n",
                          Kp, Kd, Ki, BALANCE_ANGLE, wn, z);
        } else {
            Serial.println("WARNING: Kd < l/r — system UNSTABLE. Increase Kd.");
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

        // 4. PID
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
        float error = BALANCE_ANGLE - theta;

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

        // 5. Drive motors
        //    Motor 2 is mounted mirrored → opposite sign
        step1.setTargetSpeedRad( output);
        step2.setTargetSpeedRad(-output);
    }

    // ── Diagnostics at 2 Hz ───────────────────────────────────────
    if (millis() - printTimer >= PRINT_INTERVAL_MS) {
        printTimer += PRINT_INTERVAL_MS;

        float rKd_l = WHEEL_RADIUS * Kd - L_COM;
        float wn    = (rKd_l > 0) ? sqrtf((G_ACCEL + WHEEL_RADIUS * Ki) / rKd_l) : -1.0f;
        float zeta  = (rKd_l > 0)
                    ? (WHEEL_RADIUS * Kp) / (2.0f * sqrtf(rKd_l * (G_ACCEL + WHEEL_RADIUS * Ki)))
                    : -1.0f;

        Serial.printf("theta=%.4f  gyro=%.3f  err=%.4f  "
                      "ω1=%.2f  Kp=%.1f  Kd=%.1f  Ki=%.3f  ωn=%.2f  ζ=%.3f  sp=%.4f\n",
                      theta, gyro_rate, BALANCE_ANGLE - theta,
                      step1.getSpeedRad(), Kp, Kd, Ki, wn, zeta, BALANCE_ANGLE);
    }
}