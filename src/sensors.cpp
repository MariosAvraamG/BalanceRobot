#include "sensors.h"
#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "globals.h"

// IR2=ch2 (left), IR3=ch1 (centre), IR4=ch0 (right)
static const uint8_t NUM_SENSORS            = 5;
static const uint8_t SENSOR_CH[NUM_SENSORS] = {3, 2, 1, 0, 4};  // right → left (0 = hard right, 4000 = hard left)
static const int     SETPOINT               = 2000;  // centre of 0–4000 range

static const float KP = 0.5f;
static const float KI = 0.0f;
static const float KD = 0.1f;

static uint16_t calMin[NUM_SENSORS];
static uint16_t calMax[NUM_SENSORS];
static float    irIntegral       = 0.0f;
static int      lastProportional = 0;

static uint16_t readADC(uint8_t channel)
{
    uint8_t tx0 = 0x06 | (channel >> 2);
    uint8_t tx1 = (channel & 0x03) << 6;
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ADC_CS_PIN, LOW);
    SPI.transfer(tx0);
    uint8_t rx0 = SPI.transfer(tx1);
    uint8_t rx1 = SPI.transfer(0x00);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.endTransaction();
    return ((rx0 & 0x0F) << 8) | rx1;
}

static void calibrateIR()
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        calMin[i] = 4095;
        calMax[i] = 0;
    }
    unsigned long start = millis();
    int iter = 0;
    while (millis() - start < 5000) {
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            uint16_t v = readADC(SENSOR_CH[i]);
            if (v < calMin[i]) calMin[i] = v;
            if (v > calMax[i]) calMax[i] = v;
        }
        if (++iter % 100 == 0) vTaskDelay(1);
    }
}

// Returns 0–1000: 1000 = directly over black line, 0 = on white
static uint16_t readNormalized(uint8_t i)
{
    uint16_t raw   = constrain(readADC(SENSOR_CH[i]), calMin[i], calMax[i]);
    uint16_t range = calMax[i] - calMin[i];
    if (range == 0) return 0;
    return 1000UL * (calMax[i] - raw) / range;
}

// Returns 0 (line far left) – 2000 (line far right), 1000 = centred.
// Returns -1 when no line is detected.
static int readLinePosition()
{
    uint16_t vals[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) vals[i] = readNormalized(i);

    uint32_t weighted = 0, total = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        weighted += (uint32_t)vals[i] * i * 1000;
        total    += vals[i];
    }
    if (total == 0) return -1;
    return (int)(weighted / total);
}

// Positive = line is to the right, negative = line is to the left
static float computePID(int position)
{
    int proportional  = position - SETPOINT;
    int derivative    = proportional - lastProportional;
    irIntegral       += proportional;
    lastProportional  = proportional;
    return proportional * KP + irIntegral * KI + derivative * KD;
}

void sensorsBegin()
{
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}

void sensorsCalibrateIR()
{
    Serial.println("IR calibrating — sweep sensors over line for 5 seconds...");
    calibrateIR();
    Serial.println("IR calibration done.");
}

void sensorsInit()
{
    sensorsBegin();
    sensorsCalibrateIR();
}

void printIR()
{
    static unsigned long irTimer = 0;
    if (millis() - irTimer < 100) return;
    irTimer += 100;

    int   position  = readLinePosition();
    float pidOutput = (position == -1) ? 0.0f : computePID(position);

    Serial.print("pos="); Serial.print(position);
    Serial.print("  pid="); Serial.println(pidOutput, 2);
}

void lineFollowUpdate()
{
    static bool  prevMode   = false;
    static float lfIntegral = 0.0f;
    static int   lfLastProp = 0;
    static unsigned long lfTimer = 0;

    if (lineFollowMode && !prevMode) {
        lfIntegral = 0.0f;
        lfLastProp = 0;
    }
    prevMode = lineFollowMode;

    if (!lineFollowMode) return;

    if (millis() - lfTimer < 20) return;
    lfTimer += 20;

    int pos = readLinePosition();
    irPosition = (float)pos;

    if (pos == -1) {
        irSteering = 0.0f;
    } else {
        int proportional = pos - SETPOINT;  // SETPOINT shared with printIR
        int derivative   = proportional - lfLastProp;
        lfLastProp       = proportional;
        lfIntegral      += proportional * 0.020f;
        irSteering = proportional * Kp_ir
                   + lfIntegral   * Ki_ir
                   + derivative   * Kd_ir;
    }

    velTarget    = lineFollowSpeed;
    turnBias     = constrain(-irSteering, -MAX_TURN_BIAS, MAX_TURN_BIAS);
    lastEspNowMs = millis();
    lastTurnCmdMs= millis();
}
