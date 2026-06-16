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
#include "sensors.h"

static WebServer server(80);

static void sendJSON(int code, const String& body) {
    server.sendHeader("Access-Control-Allow-Origin",  "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(code, "application/json", body);
}

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
    if (server.hasArg("mvi"))  velIntMax     = server.arg("mvi").toFloat();
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
    if (server.hasArg("lfs"))   lineFollowSpeed = server.arg("lfs").toFloat();
    if (server.hasArg("kpir")) Kp_ir           = server.arg("kpir").toFloat();
    if (server.hasArg("kiir")) Ki_ir           = server.arg("kiir").toFloat();
    if (server.hasArg("kdir")) Kd_ir           = server.arg("kdir").toFloat();
    if (server.hasArg("lflsf")) lfLostSpeedFrac = constrain(server.arg("lflsf").toFloat(), 0.0f, 1.0f);
    if (server.hasArg("lfvs"))  lfVelScale      = constrain(server.arg("lfvs").toFloat(),  50.0f, 4000.0f);
    if (server.hasArg("lfms"))  lfMinSpeedFrac  = constrain(server.arg("lfms").toFloat(),  0.0f, 1.0f);
    sendJSON(200, "{\"ok\":true}");
}

static void handleCalibrateIR()
{
    lineFollowMode = false;
    velTarget      = 0.0f;
    turnBias       = 0.0f;
    sensorsCalibrateIR();
    sendJSON(200, "{\"ok\":true}");
}

static void handleLineFollow()
{
    if (server.hasArg("en")) {
        lineFollowMode = server.arg("en").toInt() != 0;
        if (!lineFollowMode) {
            velTarget = 0.0f;
            turnBias  = 0.0f;
        }
        lastUartMs   = 0;
        lastEspNowMs = 0;
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"lf\":%d}", (int)lineFollowMode);
    sendJSON(200, buf);
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
    sendJSON(200, "{\"ok\":true}");
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
    sendJSON(200, buf);
}

static char laptopIp[40] = LAPTOP_SERVER_IP;

static void handleSetServer()
{
    String h = server.arg("host");
    h.trim();
    if (h.length() == 0 || h.length() >= sizeof(laptopIp)) {
        server.send(400, "text/plain", "Bad host");
        return;
    }
    h.toCharArray(laptopIp, sizeof(laptopIp));
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"host\":\"%s\"}", laptopIp);
    sendJSON(200, buf);
}

static void sendTelemetryNow()
{
    if (!laptopIp[0]) return;
    WiFiClient c;
    c.setTimeout(800);
    if (!c.connect(laptopIp, LAPTOP_SERVER_PORT)) return;

    char body[320];
    snprintf(body, sizeof(body),
        "{\"theta\":%.4f,\"gyro_rate\":%.4f,"
        "\"vel_est\":%.3f,\"vel_target\":%.3f,\"turn_bias\":%.3f,"
        "\"mode\":\"%s\",\"imu_ok\":%d,"
        "\"soc\":%.1f,\"bat_v\":%.2f,\"bat_i_motor\":%.3f,\"bat_power_w\":%.2f,"
        "\"ir_pos\":%.0f,\"ir_steering\":%.4f,"
        "\"uptime_s\":%lu}",
        theta, gyro_rate,
        velEst, velTarget, turnBias,
        lineFollowMode ? "LINE_FOLLOW" : espNowPrimary ? "MANUAL" : "VISION",
        (int)imuOk,
        SoC, bat_vbat, bat_imotor, bat_power,
        irPosition, irSteering,
        millis() / 1000UL);

    int bodyLen = strlen(body);
    c.printf("POST /telemetry/esp-bot HTTP/1.0\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n\r\n",
             laptopIp, bodyLen);
    c.print(body);
    unsigned long t0 = millis();
    while (c.connected() && millis() - t0 < 500) {
        if (c.available()) c.read();
    }
    c.stop();
}

static void handleStatus()
{
    uint32_t upSec  = millis() / 1000;
    uint32_t calSec = lastCalibMs ? upSec - lastCalibMs / 1000 : 0;
    const char* mode_str = lineFollowMode ? "LINE FOLLOW"
                         : espNowPrimary  ? "MANUAL"
                                          : "VISION";
    char buf[1280];
    snprintf(buf, sizeof(buf),
        "{\"mode\":\"%s\","
        "\"theta\":%.4f,\"setpt\":%.4f,\"gyro\":%.4f,\"err\":%.4f,\"spd\":%.2f,"
        "\"kp\":%.1f,\"kd\":%.1f,\"ki\":%.4f,\"sp\":%.4f,\"ac\":%.1f,\"mw\":%.1f,"
        "\"bias\":%.4f,\"raw\":%.4f,\"imu_ok\":%d,\"imu_err\":%lu,\"cal_s\":%lu,\"cf\":%.3f,"
        "\"velEst\":%.3f,\"velTarget\":%.3f,\"tiltSP\":%.4f,\"vint\":%.4f,"
        "\"kpv\":%.4f,\"kvi\":%.5f,\"mvi\":%.1f,\"mts\":%.3f,\"vs\":%.1f,\"mvt\":%.1f,\"ema\":%.2f,\"trns\":%.1f,\"mtb\":%.1f,"
        "\"yaw_rate\":%.4f,\"yawCorr\":%.4f,\"yawInt\":%.4f,\"turnBias\":%.3f,\"kyp\":%.4f,\"kiy\":%.5f,\"kdy\":%.4f,\"yea\":%.2f,\"biasZ\":%.4f,"
        "\"lf\":%d,\"lfs\":%.3f,\"irPos\":%.0f,\"irCorr\":%.4f,\"kpir\":%.5f,\"kiir\":%.5f,\"kdir\":%.5f,"
        "\"lflsf\":%.2f,\"lfvs\":%.0f,\"lfms\":%.2f,"
        "\"soc\":%.1f,\"vbat\":%.2f,\"imotor\":%.3f,\"ilogic\":%.3f,\"power\":%.2f,\"energy\":%.2f,\"trem\":%.0f,\"qused\":%.3f}",
        mode_str,
        theta, BALANCE_ANGLE + tiltSP, gyro_rate, BALANCE_ANGLE - theta, step1.getSpeedRad(),
        Kp, Kd, Ki, BALANCE_ANGLE, motorAccel, maxWheelSpeed,
        gyroBias, gyro_raw, (int)imuOk, imuErrCount, calSec, CF_COEFF,
        velEst, velTarget, tiltSP, velIntegral,
        Kp_vel, Ki_vel, velIntMax, MAX_TILT_SP, VEL_STEP, MAX_VEL_TARGET, EMA_ALPHA, TURN_STEP, MAX_TURN_BIAS,
        yaw_rate, yawCorrection, yawIntegral, turnBias, Kp_yaw, Ki_yaw, Kd_yaw, YAW_EMA_ALPHA, gyroBiasZ,
        (int)lineFollowMode, lineFollowSpeed, irPosition, irSteering, Kp_ir, Ki_ir, Kd_ir,
        lfLostSpeedFrac, lfVelScale, lfMinSpeedFrac,
        SoC, bat_vbat, bat_imotor, bat_ilogic, bat_power, bat_energy, bat_trem, bat_qused);
    sendJSON(200, buf);
}

void webServerInit()
{
    WiFi.softAP("BalanceBot2", "balance123", WIFI_CHANNEL);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("AP ready — command endpoint http://%s\n",
                  WiFi.softAPIP().toString().c_str());

    if (WIFI_STA_SSID[0]) {
        WiFi.mode(WIFI_AP_STA);
        WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
        WiFi.setAutoReconnect(true);
        Serial.printf("[WiFi] Connecting STA to '%s'...\n", WIFI_STA_SSID);
    }

    server.on("/",            handleRoot);
    server.on("/set",         handleSet);
    server.on("/move",        handleMove);
    server.on("/calibrate",   handleCalibrate);
    server.on("/status",      handleStatus);
    server.on("/linefollow",  handleLineFollow);
    server.on("/calibrateIR", handleCalibrateIR);
    server.on("/setserver",   handleSetServer);
    server.onNotFound([]() {
        server.sendHeader("Access-Control-Allow-Origin",  "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        if (server.method() == HTTP_OPTIONS) server.send(204);
        else server.send(404, "text/plain", "Not Found");
    });
    server.begin();

    xTaskCreatePinnedToCore([](void*) {
        for (;;) { server.handleClient(); vTaskDelay(1); }
    }, "web", 8192, nullptr, 1, nullptr, 0);

    xTaskCreatePinnedToCore([](void*) {
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS));
            sendTelemetryNow();
        }
    }, "telemetry", 6144, nullptr, 1, nullptr, 0);
}
