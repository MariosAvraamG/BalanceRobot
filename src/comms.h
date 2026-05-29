#pragma once
#include <Arduino.h>

// Shared command struct — used by both ESP-NOW and UART (Pi) paths.
// UART framing: 0xAA 0x55 + 8 bytes (two little-endian floats).
typedef struct {
    float linear_vel;   // rad/s → velTarget
    float angular_vel;  // rad/s → turnBias
} EspNowCmd;

void commsInit();
void espNowInit();
void parseUart();
void parseSerial();
