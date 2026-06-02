#pragma once

// ── Pins ──────────────────────────────────────────────────────────
const int STEPPER1_DIR_PIN  = 27;
const int STEPPER1_STEP_PIN = 26;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;
#define   UART_RX_PIN       16

// ── Timing ────────────────────────────────────────────────────────
const int   LOOP_INTERVAL_MS    = 5;
const float LOOP_INTERVAL_S     = 0.005f;
const int   STEPPER_INTERVAL_US = 50;
const int   PRINT_INTERVAL_MS   = 2000;

// ── SPI / ADC (MCP3204) ───────────────────────────────────────────
const int ADC_CS_PIN   = 5;
const int ADC_SCK_PIN  = 18;
const int ADC_MISO_PIN = 19;
const int ADC_MOSI_PIN = 23;

// ── Battery sensing (ESP32 internal ADC, GPIO 33/34/35) ──────────
const int BAT_PIN_VBAT   = 33;
const int BAT_PIN_IMOTOR = 34;
const int BAT_PIN_ILOGIC = 35;
const int BATTERY_INTERVAL_MS = 5000;

// ── Fixed control limits ──────────────────────────────────────────
const float MAX_INTEGRAL = 0.1f;
const float FALL_ANGLE   = 0.4f;
