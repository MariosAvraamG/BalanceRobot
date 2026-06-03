#include "power.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "motors.h"

// ── Hardware scaling ──────────────────────────────────────────────
static const float VB_RATIO    = 10000.0f / (47000.0f + 10000.0f); // 47k/10k divider
static const float SCALE_MOTOR = 20.0f  * 0.1f;   // INA180A1: gain=20, shunt=0.1 Ω
static const float SCALE_LOGIC = 100.0f * 0.01f;  // INA180A3: gain=100, shunt=0.01 Ω
static const float C_NOM_AH    = 1.6f;  // effective capacity at robot discharge rate (~80% of 2.0Ah nameplate)

// ── Battery thresholds ────────────────────────────────────────────
static const float V_FLOOR_RESEED   = 11.5f;  // below this, don't re-seed (pack is genuinely low)
static const float RESEED_MAX_QUSED = 0.05f;  // if >50 mAh counted this session, depletion is real
static const float MIN_I_LOAD       = 0.15f;  // below this, voltage is near OCV — freeze bat_vbat
static const float MIN_I_TREM       = 0.20f;  // below this, t_rem is meaningless
static const float V_CRITICAL       = 10.5f;  // graceful-shutdown threshold

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
static float I_ema    = 0.0f;  // slow EMA for t_rem (α=0.02 → ~50 s window at 1 Hz)

// Average 16 samples to reduce ESP32 internal-ADC noise
static float readVolts(int pin)
{
    int32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(pin);
    return sum * (0.001f / 16.0f);
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
    bat_vbat = Vboot;
    Serial.printf("Battery: %.2fV  SoC=%.1f%%  Qused=%.3fAh\n", Vboot, SoC, Qused_Ah);
}

void powerUpdate()
{
    static unsigned long updateTimer = 0;
    static unsigned long printTimer  = 0;

    if (millis() - updateTimer < 1000UL) return;
    updateTimer += 1000UL;

    float Vpack   = readVolts(BAT_PIN_VBAT)   / VB_RATIO;
    float Imotor  = max(0.0f, readVolts(BAT_PIN_IMOTOR) / SCALE_MOTOR);
    float Ilogic  = max(0.0f, readVolts(BAT_PIN_ILOGIC) / SCALE_LOGIC);
    float I_total = Imotor + Ilogic;

    // Coulomb counter at 1-second resolution (trapezoidal)
    const float dt_h = 1.0f / 3600.0f;
    Qused_Ah += 0.5f * (I_prev + I_total) * dt_h;
    I_prev    = I_total;

    SoC = constrain(100.0f - (Qused_Ah / C_NOM_AH * 100.0f), 0.0f, 100.0f);

    // Re-seed guard: corrects a bad boot seed only.
    // Requires voltage above V_FLOOR_RESEED (rules out genuinely low packs)
    // AND less than 50 mAh counted (rules out real depletion this session).
    if (SoC <= 0.0f && Vpack >= V_FLOOR_RESEED && Qused_Ah < RESEED_MAX_QUSED) {
        SoC      = lookupSoC(Vpack);
        Qused_Ah = C_NOM_AH * (1.0f - SoC / 100.0f);
        I_prev   = 0.0f;
        Serial.printf("[BAT] Reseeded: %.2fV -> %.1f%%\n", Vpack, SoC);
    }

    // Freeze displayed voltage when motors are idle.
    // Under no/low load the pack recovers toward OCV (+0.5–1.5 V on NiMH),
    // making a depleted pack look healthier than it is.
    // bat_vbat holds the last valid under-load reading until load returns.
    if (I_total >= MIN_I_LOAD) bat_vbat = Vpack;
    if (bat_vbat < 1.0f)       bat_vbat = Vpack;  // first-boot fallback

    // Critical voltage: disable motors before brownout, force SoC to 0
    if (bat_vbat < V_CRITICAL && bat_vbat > 1.0f) {
        step1.setTargetSpeedRad(0.0f);
        step2.setTargetSpeedRad(0.0f);
        digitalWrite(STEPPER_EN_PIN, HIGH);
        SoC      = 0.0f;
        Qused_Ah = C_NOM_AH;
        Serial.printf("[BAT] CRITICAL %.2fV — motors disabled\n", bat_vbat);
    }

    float P_watts     = bat_vbat * I_total;
    float E_remain_Wh = (SoC / 100.0f) * C_NOM_AH * bat_vbat;

    // Slow EMA on current — smooths t_rem over ~50 s so momentary load changes
    // don't cause wild swings; I_ema persists across idle periods
    if (I_ema < 1e-6f) I_ema = I_total;
    else               I_ema += 0.02f * (I_total - I_ema);

    // t_rem: use EMA current so estimate reflects recent average draw;
    // show -- when never under real load; cap at 5 min in warning zone
    float t_remain_min;
    if (I_ema >= MIN_I_TREM) {
        t_remain_min = (SoC / 100.0f * C_NOM_AH) / I_ema * 60.0f;
        if (bat_vbat < V_FLOOR_RESEED) t_remain_min = min(t_remain_min, 5.0f);
    } else {
        t_remain_min = 999.0f;
    }

    bat_imotor = Imotor;
    bat_ilogic = Ilogic;
    bat_power  = P_watts;
    bat_energy = E_remain_Wh;
    bat_trem   = t_remain_min;

    if (millis() - printTimer < 5000UL) return;
    printTimer += 5000UL;

    Serial.printf("[BAT] Vraw=%.2fV Vdisp=%.2fV Im=%.3fA Il=%.3fA SoC=%.1f%% Qused=%.3fAh/%.1fAh P=%.2fW t=%.0fmin\n",
        Vpack, bat_vbat, Imotor, Ilogic, SoC, Qused_Ah, C_NOM_AH, P_watts, t_remain_min);
    Serial.printf("DATA:%.1f:%.2f:%.0f:%.2f:%.2f\n",
        SoC, P_watts, t_remain_min, bat_vbat, E_remain_Wh);
}
