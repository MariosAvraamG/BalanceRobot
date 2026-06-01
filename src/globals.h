#pragma once
#include <Arduino.h>

// ── Tuning parameters (runtime-mutable via web/serial) ────────────
extern float Kp;
extern float Kd;
extern float Ki;
extern float BALANCE_ANGLE;
extern float CF_COEFF;

extern float maxWheelSpeed;
extern float motorAccel;

extern float MAX_TILT_SP;
extern float EMA_ALPHA;
extern float Kp_vel;
extern float Ki_vel;
extern float VEL_STEP;
extern float MAX_VEL_TARGET;
extern float TURN_STEP;
extern float MAX_TURN_BIAS;

extern float Kp_yaw;
extern float Ki_yaw;
extern float Kd_yaw;
extern float YAW_EMA_ALPHA;

// ── Control state ─────────────────────────────────────────────────
extern float velTarget;
extern float velIntegral;
extern float tiltSP;
extern float turnBias;

// ── Telemetry / runtime state ─────────────────────────────────────
extern float theta;
extern float gyro_rate;
extern float gyro_raw;
extern float accel_angle;
extern float integral;
extern float velEst;
extern float gyroBias;
extern float gyroBiasZ;
extern float yaw_rate;
extern float yawCorrection;
extern float yawIntegral;
extern float prevYawRate;
extern float uartLinear;
extern float uartAngular;

extern bool          imuOk;
extern uint32_t      imuErrCount;
extern uint32_t      lastCalibMs;
extern uint32_t      lastTurnCmdMs;
extern volatile bool calibrating;
extern bool          fallen;
extern unsigned long lastLoopUs;
extern unsigned long lastEspNowMs;
extern unsigned long lastUartMs;

// ── Line follow ───────────────────────────────────────────────────
extern bool  lineFollowMode;
extern float lineFollowSpeed;
extern float Kp_ir;
extern float Ki_ir;
extern float Kd_ir;
extern float irPosition;
extern float irSteering;
