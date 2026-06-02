#include "globals.h"

float Kp            = 2100.0f;
float Kd            =  240.0f;
float Ki            =    1.0f;
float BALANCE_ANGLE =   0.06f;
float CF_COEFF      =  0.996f;

float maxWheelSpeed  = 19.0f;
float motorAccel     = 30.0f;

float MAX_TILT_SP    = 0.115f;
float EMA_ALPHA      = 0.90f;
float Kp_vel         = 0.007f;
float Ki_vel         = 0.001f;
float VEL_STEP       = 1.0f;
float MAX_VEL_TARGET = 7.2f;
float TURN_STEP      = 1.0f;
float MAX_TURN_BIAS  = 5.0f;

float Kp_yaw        = 0.180f;
float Ki_yaw        = 0.0100f;
float Kd_yaw        = 0.0230f;
float YAW_EMA_ALPHA = 0.90f;

float velTarget   = 0.0f;
float velIntegral = 0.0f;
float tiltSP      = 0.0f;
float turnBias    = 0.0f;

float theta        = 0.0f;
float gyro_rate    = 0.0f;
float gyro_raw     = 0.0f;
float accel_angle  = 0.0f;
float integral     = 0.0f;
float velEst       = 0.0f;
float gyroBias     = 0.0f;
float gyroBiasZ    = 0.0f;
float yaw_rate     = 0.0f;
float yawCorrection = 0.0f;
float yawIntegral   = 0.0f;
float prevYawRate   = 0.0f;
float uartLinear    = 0.0f;
float uartAngular   = 0.0f;

bool          imuOk       = true;
uint32_t      imuErrCount = 0;
uint32_t      lastCalibMs   = 0;
uint32_t      lastTurnCmdMs = 0;
volatile bool calibrating = false;
bool          fallen      = false;
unsigned long lastLoopUs  = 0;
unsigned long lastEspNowMs  = 0;
unsigned long lastUartMs    = 0;

float SoC        = 100.0f;
float bat_vbat   = 0.0f;
float bat_imotor = 0.0f;
float bat_ilogic = 0.0f;
float bat_power  = 0.0f;
float bat_energy = 0.0f;
float bat_trem   = 999.0f;

bool  lineFollowMode  = false;
float lineFollowSpeed = 2.0f;
float Kp_ir           = 0.004f;
float Ki_ir           = 0.0f;
float Kd_ir           = 0.001f;
float irPosition      = -1.0f;
float irSteering      = 0.0f;
