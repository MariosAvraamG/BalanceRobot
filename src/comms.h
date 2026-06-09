#pragma once
#include <Arduino.h>

// UART path — Pi sends exactly sizeof(EspNowCmd) = 8 bytes.
// Framing: 0xAA 0x55 + 8 bytes (two little-endian floats). Do not change.
typedef struct {
    float linear_vel;   // rad/s → velTarget
    float angular_vel;  // rad/s → turnBias
} EspNowCmd;

// ESP-NOW path — matches handheld sender struct layout exactly.
// Must stay in sync with the sender's EspNowCmd typedef.
typedef struct {
    float linear_vel;
    float angular_vel;
    bool  btn_red;
    bool  btn_blue;
    bool  btn_green;
} ControllerCmd;

// Status sent back to the handheld controller via ESP-NOW at 2 Hz.
typedef struct {
    int  mode;  // "MANUAL", "CV", "LINE_FOLLOW"
    float soc;       // battery SoC 0–100 %
    float linear;
    float angular;
} RobotStatus;

void commsInit();
void espNowInit();
void parseUart();
void parseSerial();
void commsSendStatus();  // call from main loop — self-throttled to 2 Hz
