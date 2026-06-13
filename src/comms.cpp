#include "comms.h"
#include <WiFi.h>
#include <esp_now.h>
#include "globals.h"
#include "config.h"
#include "motors.h"

// espNowPrimary defined in globals.cpp — accessible everywhere
static bool    peerRegistered   = false;
static uint8_t controllerMac[6] = {0};

static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (len < (int)sizeof(ControllerCmd)) return;
    ControllerCmd cmd;
    memcpy(&cmd, data, sizeof(cmd));

    // Register sender as a peer on first contact so we can send status back
    if (!peerRegistered) {
        memcpy(controllerMac, mac, 6);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, controllerMac, 6);
        peer.channel = 11;
        peer.encrypt = false;
        peer.ifidx = WIFI_IF_AP;
        esp_now_add_peer(&peer);
        peerRegistered = true;
    }

    if (espNowPrimary && !lineFollowMode) {
        velTarget     = constrain(cmd.linear_vel,  -MAX_VEL_TARGET, MAX_VEL_TARGET);
        turnBias      = constrain(cmd.angular_vel, -MAX_TURN_BIAS,  MAX_TURN_BIAS);
        lastTurnCmdMs = millis();
        lastEspNowMs  = millis();
    }

    if (cmd.btn_red) {
        velTarget   = 0.0f;
        tiltSP      = 0.0f;
        integral    = 0.0f;
        velIntegral = 0.0f;
        turnBias    = 0.0f;
        yawIntegral = 0.0f;
    }
    if (cmd.btn_blue) {
        espNowPrimary = !espNowPrimary;
        velTarget     = 0.0f;
        turnBias      = 0.0f;
        velIntegral   = 0.0f;
        yawIntegral   = 0.0f;
        // Disarm the deadman of the source we just left so it cannot fire
        // while the new source is in control.
        if (espNowPrimary) lastUartMs   = 0;
        else               lastEspNowMs = 0;
        Serial.printf("[CTRL] Source -> %s\n", espNowPrimary ? "ESP-NOW" : "UART");
    }
    if (cmd.btn_green) {
        lineFollowMode = !lineFollowMode;
        velTarget      = 0.0f;
        turnBias       = 0.0f;
        velIntegral    = 0.0f;
        yawIntegral    = 0.0f;
        // Zero whichever background-source timer is now inactive so its
        // deadman cannot fire while line-follow is running (or just ended).
        lastUartMs   = 0;
        lastEspNowMs = 0;
        Serial.printf("[CTRL] IR mode -> %s\n", lineFollowMode ? "ON" : "OFF");
    }

    // Diagnostics: first-packet banner, per-packet data, and 2-second rate report
    static bool          firstPacket = true;
    static uint32_t      pktCount    = 0;
    static uint32_t      rateCount   = 0;
    static unsigned long rateTimer   = 0;
    static unsigned long printTimer  = 0;

    pktCount++;
    rateCount++;

    if (firstPacket) {
        firstPacket = false;
        Serial.printf("[ESP-NOW] First packet from %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // Print every packet — at ~50 Hz this is readable in the serial monitor
    if (millis() - printTimer >= 100) {
        printTimer = millis();
        Serial.printf("[ESP-NOW] #%lu  lin=%+.2f  ang=%+.2f  r=%d b=%d g=%d\n",
                      (unsigned long)pktCount,
                      cmd.linear_vel, cmd.angular_vel,
                      cmd.btn_red, cmd.btn_blue, cmd.btn_green);
    }

    // Packet-rate summary every 2 s
    if (millis() - rateTimer >= 2000) {
        Serial.printf("[ESP-NOW] rate=%.1f pkt/s  total=%lu\n",
                      rateCount / 2.0f, (unsigned long)pktCount);
        rateCount  = 0;
        rateTimer  = millis();
    }
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
                    uartLinear  = cmd.linear_vel;   // always update for diagnostics
                    uartAngular = cmd.angular_vel;
                    if (!espNowPrimary && !lineFollowMode) {
                        velTarget     = constrain(cmd.linear_vel,  -MAX_VEL_TARGET, MAX_VEL_TARGET);
                        turnBias      = constrain(cmd.angular_vel, -MAX_TURN_BIAS,  MAX_TURN_BIAS);
                        lastTurnCmdMs = millis();
                        lastUartMs    = millis();
                    }
                    uartState = HUNT_A;
                }
                break;
        }
    }
}

// mode values: 0 = MANUAL, 1 = CV, 2 = LINE_FOLLOW
void commsSendStatus()
{
    static unsigned long timer = 0;
    if (!peerRegistered)            return;
    if (millis() - timer < 1000UL)  return;
    timer = millis();

    RobotStatus msg;
    if      (lineFollowMode)  msg.mode = 2;
    else if (!espNowPrimary)  msg.mode = 1;
    else                      msg.mode = 0;
    msg.soc     = SoC;
    msg.linear  = velTarget;
    msg.angular = turnBias;

    esp_now_send(controllerMac, (const uint8_t*)&msg, sizeof(msg));

    static const char* MODE_STR[] = { "MANUAL", "CV", "LINE_FOLLOW" };
    Serial.printf("[STATUS->CTRL] %02X:%02X:%02X:%02X:%02X:%02X  mode=%s  soc=%.1f%%  lin=%+.2f  ang=%+.2f\n",
                  controllerMac[0], controllerMac[1], controllerMac[2],
                  controllerMac[3], controllerMac[4], controllerMac[5],
                  MODE_STR[msg.mode], msg.soc, msg.linear, msg.angular);
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
