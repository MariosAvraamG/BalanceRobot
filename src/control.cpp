#include "control.h"
#include <Arduino.h>
#include "globals.h"
#include "config.h"
#include "motors.h"

bool controlTick(float dt)
{
    // ── Fall detection ────────────────────────────────────────────
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
            return false;
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

    // ── Inner balance PID ─────────────────────────────────────────
    float tiltSetpoint = BALANCE_ANGLE + tiltSP;
    float error = tiltSetpoint - theta;

    integral += error * dt;
    integral  = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float P_term = Kp * error;
    float I_term = Ki * integral;
    float D_term = Kd * (-gyro_rate);  // d(error)/dt = -θ̇

    float output = constrain(P_term + I_term + D_term, -maxWheelSpeed, maxWheelSpeed);

    // ── Yaw PID ───────────────────────────────────────────────────
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

    // ── Drive motors (motor 2 mounted mirrored → opposite sign) ──
    step1.setTargetSpeedRad( output - yawCorrection);
    step2.setTargetSpeedRad(-output - yawCorrection);
    return true;
}

void velLoopUpdate()
{
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
}

void deadManCheck()
{
    if (lastEspNowMs && millis() - lastEspNowMs > 500) {
        velTarget    = 0.0f;
        turnBias     = 0.0f;
        velIntegral  = 0.0f;
        yawIntegral  = 0.0f;
        lastEspNowMs = 0;
    }

    if (lastTurnCmdMs && turnBias != 0.0f && millis() - lastTurnCmdMs > 300) {
        turnBias    = 0.0f;
        yawIntegral = 0.0f;
    }
}

void printDiagnostics()
{
    static unsigned long printTimer    = 0;
    static unsigned long uartDiagTimer = 0;

    if (millis() - uartDiagTimer >= 500) {
        uartDiagTimer += 500;
        if (lastUartMs == 0) {
            Serial.println("UART[Pi]: no packet received yet");
        } else {
            Serial.printf("UART[Pi]: linear=%.3f  angular=%.3f  age=%lums\n",
                          uartLinear, uartAngular, millis() - lastUartMs);
        }
    }

    if (millis() - printTimer >= PRINT_INTERVAL_MS) {
        printTimer += PRINT_INTERVAL_MS;
        Serial.printf(
            "theta=%.4f  gyro=%.3f  velEst=%.3f  velTgt=%.3f  tiltSP=%.4f  vint=%.4f  "
            "Kp=%.1f  Kd=%.1f  Ki=%.3f  Kpv=%.4f  Kvi=%.5f  ac=%.1f  mw=%.1f  sp=%.4f  "
            "yaw=%.4f  yawCorr=%.4f  yawInt=%.4f  kyp=%.3f  kyi=%.5f  kyd=%.4f\n",
            theta, gyro_rate, velEst, velTarget, tiltSP, velIntegral,
            Kp, Kd, Ki, Kp_vel, Ki_vel, motorAccel, maxWheelSpeed, BALANCE_ANGLE,
            yaw_rate, yawCorrection, yawIntegral, Kp_yaw, Ki_yaw, Kd_yaw);
    }
}
