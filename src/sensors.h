#pragma once

void sensorsInit();  // SPI init + 5-second IR calibration
void printIR();      // print line position + PID output every 100 ms
