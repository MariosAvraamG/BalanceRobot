#include "power.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"

// ── Hardware scaling ──────────────────────────────────────────────
static const float VB_RATIO    = 10000.0f / (47000.0f + 10000.0f); // 47k/10k voltage divider
static const float SCALE_MOTOR = 20.0f  * 0.1f;   // INA180A1: gain=20, shunt=0.1 Ω
static const float SCALE_LOGIC = 100.0f * 0.01f;  // INA180A3: gain=100, shunt=0.01 Ω
static const float C_NOM_AH    = 2.0f;             // 2×7.2 V NiMH in series, 2.0 Ah

// ── NiMH OCV → SoC lookup ────────────────────────────────────────
static const int N_TAB = 26;
static const float V_TAB[N_TAB] = {
    14.880f, 14.340f, 13.800f, 13.500f, 13.200f, 13.104f,
    13.020f, 12.960f, 12.900f, 12.840f, 12.780f, 12.744f,
    12.720f, 12.684f, 12.660f, 12.660f, 12.660f, 12.504f,
    12.360f, 12.180f, 12.000f, 11.640f, 11.280f, 10.860f,
    10.440f,  9.600f
};
static const float SOC_TAB[N_TAB] = {
    100.0f, 96.0f, 92.0f, 88.0f, 84.0f, 80.0f,
     76.0f, 72.0f, 68.0f, 64.0f, 60.0f, 56.0f,
     52.0f, 48.0f, 44.0f, 40.0f, 36.0f, 32.0f,
     28.0f, 24.0f, 20.0f, 16.0f, 12.0f,  8.0f,
      4.0f,  0.0f
};

static float Qused_Ah = 0.0f;
static float I_prev   = 0.0f;

static float readVolts(int pin)
{
    return analogReadMilliVolts(pin) * 0.001f;
}

static float lookupSoC(float Vpack)
{
    if (Vpack >= V_TAB[0])       return 100.0f;
    if (Vpack <= V_TAB[N_TAB-1]) return   0.0f;
    int i = 0;
    while (Vpack < V_TAB[i + 1]) i++;
    float frac = (V_TAB[i] - Vpack) / (V_TAB[i] - V_TAB[i + 1]);
    return SOC_TAB[i] + frac * (SOC_TAB[i + 1] - SOC_TAB[i]);
}

void powerInit()
{
    analogSetPinAttenuation(BAT_PIN_VBAT,   ADC_11db);
    analogSetPinAttenuation(BAT_PIN_IMOTOR, ADC_11db);
    analogSetPinAttenuation(BAT_PIN_ILOGIC, ADC_11db);

    // Motors not yet running → battery at rest → OCV is reliable for SoC seed
    float Vboot;
    do {
        Vboot = readVolts(BAT_PIN_VBAT) / VB_RATIO;
        if (Vboot < V_TAB[N_TAB-1])
            Serial.printf("Battery not ready (%.2fV), waiting...\n", Vboot);
        delay(500);
    } while (Vboot < V_TAB[N_TAB-1]);

    SoC      = lookupSoC(Vboot);
    Qused_Ah = C_NOM_AH * (1.0f - SoC / 100.0f);
    I_prev   = 0.0f;
    Serial.printf("Battery: %.2fV  SoC=%.1f%%  Qused=%.3fAh\n", Vboot, SoC, Qused_Ah);
}

void powerUpdate()
{
    static unsigned long battTimer = 0;
    if (millis() - battTimer < (unsigned long)BATTERY_INTERVAL_MS) return;
    battTimer += BATTERY_INTERVAL_MS;

    float Vpack  = readVolts(BAT_PIN_VBAT)   / VB_RATIO;
    float Imotor = max(0.0f, readVolts(BAT_PIN_IMOTOR) / SCALE_MOTOR);
    float Ilogic = max(0.0f, readVolts(BAT_PIN_ILOGIC) / SCALE_LOGIC);
    float I_total = Imotor + Ilogic;

    float dt_h = BATTERY_INTERVAL_MS / 3600000.0f;
    Qused_Ah  += 0.5f * (I_prev + I_total) * dt_h;  // trapezoidal
    I_prev     = I_total;

    SoC = constrain(100.0f - (Qused_Ah / C_NOM_AH * 100.0f), 0.0f, 100.0f);

    // Re-seed if Coulomb counter drifted to 0% but voltage says otherwise
    if (SoC <= 0.0f && Vpack >= V_TAB[N_TAB-1]) {
        SoC      = lookupSoC(Vpack);
        Qused_Ah = C_NOM_AH * (1.0f - SoC / 100.0f);
        I_prev   = 0.0f;
        Serial.printf("[BAT] Reseeded: %.2fV → %.1f%%\n", Vpack, SoC);
    }

    bat_vbat   = Vpack;
    bat_imotor = Imotor;
    bat_ilogic = Ilogic;
    bat_power  = Vpack * I_total;
    bat_energy = (SoC / 100.0f) * C_NOM_AH * Vpack;
    bat_trem   = (I_total > 0.01f) ? (SoC / 100.0f * C_NOM_AH) / I_total * 60.0f : 999.0f;

    if (Vpack < V_TAB[N_TAB-1]) {
        Serial.println("[BAT] Waiting for battery...");
    } else {
        Serial.printf("[BAT] Vb=%.2fV Im=%.3fA Il=%.3fA SoC=%.1f%% P=%.2fW E=%.2fWh t=%.0fmin\n",
            Vpack, Imotor, Ilogic, SoC, bat_power, bat_energy, bat_trem);
        Serial.printf("DATA:%.1f:%.2f:%.0f:%.2f:%.2f\n",
            SoC, bat_power, bat_trem, Vpack, bat_energy);
    }
}
