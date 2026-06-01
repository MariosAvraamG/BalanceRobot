#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "motors.h"
#include "imu.h"
#include "comms.h"
#include "control.h"
#include "web_server.h"
#include "sensors.h"

void setup()
{
    Serial.begin(115200);
    commsInit();       // Serial2 (Raspberry Pi UART)
    motorsInit();      // pins, acceleration, stepper ISR
    imuInit();         // Wire, MPU6050
    webServerInit();   // WiFi AP, routes, web task on core 0
    espNowInit();      // ESP-NOW (requires WiFi to be up first)
    sensorsBegin();    // SPI ADC hardware init (IR calibration done via web UI)

    Serial.println("Calibrating — hold robot upright and still for 1 second...");
    calibrate();
    velTarget   = 0.0f;
    velIntegral = 0.0f;
    velEst      = 0.0f;
}

void loop()
{
    parseUart();
    parseSerial();

    static unsigned long loopTimer = 0;
    if (millis() - loopTimer >= LOOP_INTERVAL_MS) {
        loopTimer += LOOP_INTERVAL_MS;

        if (calibrating) return;

        unsigned long nowUs = micros();
        float dt = (lastLoopUs == 0) ? LOOP_INTERVAL_S
                                     : constrain((nowUs - lastLoopUs) * 1e-6f, 0.001f, 0.020f);
        lastLoopUs = nowUs;

        if (!imuRead(dt))    return;
        if (!controlTick(dt)) return;
    }

    lineFollowUpdate();
    velLoopUpdate();
    deadManCheck();
    printDiagnostics();
    printIR();
}
