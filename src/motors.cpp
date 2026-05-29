#include "motors.h"
#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include "globals.h"

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
static ESP32Timer ITimer(3);

static bool IRAM_ATTR TimerHandler(void*)
{
    static bool tog = false;
    step1.runStepper();
    step2.runStepper();
    digitalWrite(TOGGLE_PIN, tog);
    tog = !tog;
    return true;
}

void motorsInit()
{
    pinMode(TOGGLE_PIN,     OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW);

    step1.setAccelerationRad(motorAccel);
    step2.setAccelerationRad(motorAccel);

    if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
        Serial.println("Stepper ISR attach failed");
        while (1) delay(10);
    }
}
