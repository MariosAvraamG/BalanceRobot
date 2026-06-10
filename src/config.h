#pragma once

// ── Pins ──────────────────────────────────────────────────────────
const int STEPPER1_DIR_PIN  = 27;
const int STEPPER1_STEP_PIN = 26;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
#define   UART_RX_PIN       16

// ── OLED display (second I²C bus, Wire1) ─────────────────────────
const int OLED_SDA_PIN = 25;
const int OLED_SCL_PIN = 32;

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
const int BAT_PIN_IMOTOR = 35;
const int BAT_PIN_ILOGIC = 34;
const int BATTERY_INTERVAL_MS = 5000;

// ── WiFi / ESP-NOW ────────────────────────────────────────────────
const int WIFI_CHANNEL = 1;  // SoftAP channel — controller must match this

// ── WS2812B LED strip ─────────────────────────────────────────────
const int WS2812B_PIN = 17;
const int WS2812B_NUM = 14;   // change to match actual LED count

// ── Fixed control limits ──────────────────────────────────────────
const float MAX_INTEGRAL = 0.1f;
const float FALL_ANGLE   = 0.4f;
