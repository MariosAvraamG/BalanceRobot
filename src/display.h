#pragma once

void displayInit();  // Wire1 init, find OLED, spawn render task on core 0
void ledInit();      // WS2812B init — set eye LEDs red, body blue
void ledUpdate();    // update strip colour based on current drive mode
