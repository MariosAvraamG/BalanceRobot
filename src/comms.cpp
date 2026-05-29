#include "comms.h"
#include <WiFi.h>
#include <esp_now.h>
#include "globals.h"
#include "config.h"
#include "motors.h"

static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (len < (int)sizeof(EspNowCmd)) return;
    EspNowCmd cmd;
    memcpy(&cmd, data, sizeof(cmd));
    velTarget     = constrain(cmd.linear_vel,  -MAX_VEL_TARGET, MAX_VEL_TARGET);
    turnBias      = constrain(cmd.angular_vel, -MAX_TURN_BIAS,  MAX_TURN_BIAS);
    lastTurnCmdMs = millis();
    lastEspNowMs  = millis();
}

void commsInit()
{
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, -1);
}

void espNowInit()
{
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
    } else {
        esp_now_register_recv_cb(onEspNowRecv);
        Serial.printf("ESP-NOW ready — target this AP MAC on sender: %s  channel: %d\n",
                      WiFi.softAPmacAddress().c_str(), WiFi.channel());
    }
}

void parseUart()
{
    static enum : uint8_t { HUNT_A, HUNT_B, READ_PAYLOAD } uartState = HUNT_A;
    static uint8_t uartBuf[sizeof(EspNowCmd)];
    static uint8_t uartIdx = 0;
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        switch (uartState) {
            case HUNT_A:
                if (b == 0xAA) uartState = HUNT_B;
                break;
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

void parseSerial()
{
    if (!Serial.available()) return;
    char peek = (char)Serial.peek();

    if (peek=='w'||peek=='W'||peek=='s'||peek=='S'||
        peek=='a'||peek=='A'||peek=='d'||peek=='D'||peek==' ') {
        Serial.read();
        switch (tolower(peek)) {
            case 'w': velTarget = constrain(velTarget + VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); break;
            case 's': velTarget = constrain(velTarget - VEL_STEP, -MAX_VEL_TARGET, MAX_VEL_TARGET); break;
            case 'a': turnBias  = constrain(turnBias  - TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);  break;
            case 'd': turnBias  = constrain(turnBias  + TURN_STEP, -MAX_TURN_BIAS, MAX_TURN_BIAS);  break;
            case ' ': velTarget = 0.0f; tiltSP = 0.0f; turnBias = 0.0f; integral = 0.0f;           break;
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
        else if (cmd.startsWith("trns")) TURN_STEP      = cmd.substring(5).toFloat();
        else if (cmd.startsWith("tr"))   turnBias       = constrain(cmd.substring(3).toFloat(), -MAX_TURN_BIAS, MAX_TURN_BIAS);
        else if (cmd == "en 0")          { digitalWrite(STEPPER_EN_PIN, HIGH); Serial.println("Motors DISABLED"); return; }
        else if (cmd == "en 1")          { digitalWrite(STEPPER_EN_PIN, LOW);  Serial.println("Motors ENABLED");  return; }

        Serial.printf("Kp=%.1f  Kd=%.1f  Ki=%.3f  Kpv=%.4f  Kvi=%.5f  sp=%.4f  ac=%.1f  mw=%.1f\n",
                      Kp, Kd, Ki, Kp_vel, Ki_vel, BALANCE_ANGLE, motorAccel, maxWheelSpeed);
    }
}
