#pragma once

void displayInit();  // Wire1 init, find OLED, spawn render task on core 0
void ledInit();      // WS2812B init — set middle LED red, peripherals blue
