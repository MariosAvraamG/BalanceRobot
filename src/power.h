#pragma once

void powerInit();    // configure ADC pins, seed SoC from OCV — call before calibrate()
void powerUpdate();  // Coulomb counting + serial report — call every loop()
