#include "sensors.h"
#include <Arduino.h>
#include <SPI.h>
#include "config.h"

// IR2=ch2 (left), IR3=ch1 (centre), IR4=ch0 (right)
static const uint8_t NUM_SENSORS            = 3;
static const uint8_t SENSOR_CH[NUM_SENSORS] = {2, 1, 0};
static const int     SETPOINT               = 1000;  // centre of 0–2000 range

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
    while (millis() - start < 5000) {
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            uint16_t v = readADC(SENSOR_CH[i]);
            if (v < calMin[i]) calMin[i] = v;
            if (v > calMax[i]) calMax[i] = v;
        }
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

void sensorsInit()
{
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
    Serial.println("IR calibrating — sweep sensors over line for 5 seconds...");
    calibrateIR();
    Serial.println("IR calibration done.");
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
