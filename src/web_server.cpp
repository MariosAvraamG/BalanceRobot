#include "web_server.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include "web_ui.h"
#include "globals.h"
#include "config.h"
#include "motors.h"
#include "imu.h"

static WebServer server(80);

static void handleRoot()
{
    server.send_P(200, "text/html", HTML);
}

static void handleSet()
{
    if (server.hasArg("kp"))   Kp            = server.arg("kp").toFloat();
    if (server.hasArg("kd"))   Kd            = server.arg("kd").toFloat();
    if (server.hasArg("ki"))   Ki            = server.arg("ki").toFloat();
    if (server.hasArg("sp"))   BALANCE_ANGLE = server.arg("sp").toFloat();
    if (server.hasArg("kpv"))  Kp_vel        = server.arg("kpv").toFloat();
    if (server.hasArg("kvi"))  Ki_vel        = server.arg("kvi").toFloat();
    if (server.hasArg("mts"))  MAX_TILT_SP   = server.arg("mts").toFloat();
    if (server.hasArg("vs"))   VEL_STEP      = server.arg("vs").toFloat();
    if (server.hasArg("mvt"))  MAX_VEL_TARGET= server.arg("mvt").toFloat();
    if (server.hasArg("ema"))  EMA_ALPHA     = constrain(server.arg("ema").toFloat(), 0.01f, 1.0f);
    if (server.hasArg("trns")) TURN_STEP     = server.arg("trns").toFloat();
    if (server.hasArg("mtb"))  MAX_TURN_BIAS = server.arg("mtb").toFloat();
    if (server.hasArg("mw"))   maxWheelSpeed = server.arg("mw").toFloat();
    if (server.hasArg("cf"))   CF_COEFF      = constrain(server.arg("cf").toFloat(), 0.0f, 0.9999f);
    if (server.hasArg("kyp"))  Kp_yaw        = server.arg("kyp").toFloat();
    if (server.hasArg("kiy"))  Ki_yaw        = server.arg("kiy").toFloat();
    if (server.hasArg("kdy"))  Kd_yaw        = server.arg("kdy").toFloat();
    if (server.hasArg("yea"))  YAW_EMA_ALPHA = constrain(server.arg("yea").toFloat(), 0.01f, 1.0f);
    if (server.hasArg("ac")) {
        motorAccel = server.arg("ac").toFloat();
        step1.setAccelerationRad(motorAccel);
        step2.setAccelerationRad(motorAccel);
    }
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleMove()
{
    String dir = server.arg("dir");
    if      (dir == "w")    velTarget = constrain(velTarget + VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET);
    else if (dir == "s")    velTarget = constrain(velTarget - VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET);
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
}

static void handleCalibrate()
{
    step1.setTargetSpeedRad(0.0f);
    step2.setTargetSpeedRad(0.0f);
    tiltSP      = 0.0f;
    turnBias    = 0.0f;
    integral    = 0.0f;
    calibrating = true;
    delay(300);
    Serial.println("Web calibration — hold robot upright and still...");
    calibrate();
    velTarget   = 0.0f;
    velIntegral = 0.0f;
    velEst      = 0.0f;
    lastLoopUs  = 0;
    calibrating = false;
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"sp\":%.4f}", BALANCE_ANGLE);
    server.send(200, "application/json", buf);
}

static void handleStatus()
{
    uint32_t upSec  = millis() / 1000;
    uint32_t calSec = lastCalibMs ? upSec - lastCalibMs / 1000 : 0;
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
}

void webServerInit()
{
    WiFi.softAP("BalanceBot2", "balance123");
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("Web tuner: connect to WiFi 'BalanceBot2' then open http://%s\n",
                  WiFi.softAPIP().toString().c_str());

    server.on("/",          handleRoot);
    server.on("/set",       handleSet);
    server.on("/move",      handleMove);
    server.on("/calibrate", handleCalibrate);
    server.on("/status",    handleStatus);
    server.begin();

    xTaskCreatePinnedToCore(
        [](void*){ for(;;){ server.handleClient(); vTaskDelay(1); } },
        "web", 8192, nullptr, 1, nullptr, 0);
}
