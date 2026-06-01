#pragma once

void sensorsBegin();        // SPI hardware init only (no calibration)
void sensorsInit();         // SPI init + 5-second IR calibration (combined)
void sensorsCalibrateIR();  // 5-second calibration sweep (call on demand)
void printIR();             // print line position + PID output every 100 ms
void lineFollowUpdate();    // apply IR steering to turnBias when lineFollowMode is active
