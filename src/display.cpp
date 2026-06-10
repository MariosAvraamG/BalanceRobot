#include "display.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "globals.h"

static Adafruit_NeoPixel strip(WS2812B_NUM, WS2812B_PIN, NEO_GRB + NEO_KHZ800);

static Adafruit_SSD1306 oled(128, 64, &Wire1, -1);
static bool oledOk = false;

// Layout uses the hardware two-colour split:
//   Yellow band (y=0–15)  → SoC % in large text, centred
//   Blue band  (y=16–63)  → iPhone-style battery outline + fill

static void drawDisplay()
{
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    // ── SoC percentage (yellow band) ─────────────────────────────
    char pct[6];
    snprintf(pct, sizeof(pct), "%d%%", (int)SoC);
    oled.setTextSize(2);                        // 12×16 px per char
    int16_t x1, y1; uint16_t tw, th;
    oled.getTextBounds(pct, 0, 0, &x1, &y1, &tw, &th);
    oled.setCursor((128 - (int16_t)tw) / 2, 0);
    oled.print(pct);

    // ── Battery icon (blue band, y=16–63) ────────────────────────
    const int16_t bx = 16, by = 24;
    const int16_t bw = 90, bh = 30, brad = 4;
    const int16_t pad = 3;

    // Nub — small rounded cap on the right, 1 px gap from body
    const int16_t nw = 6, nh = 14;
    const int16_t nx = bx + bw + 1;
    const int16_t ny = by + (bh - nh) / 2;
    oled.fillRoundRect(nx, ny, nw, nh, 2, SSD1306_WHITE);

    // Body outline
    oled.drawRoundRect(bx, by, bw, bh, brad, SSD1306_WHITE);

    // Fill proportional to SoC, with inner padding
    int16_t fillW = (int16_t)(SoC / 100.0f * (bw - 2 * pad));
    if (fillW > 0)
        oled.fillRoundRect(bx + pad, by + pad,
                           fillW, bh - 2 * pad,
                           (brad > pad) ? brad - pad : 1,
                           SSD1306_WHITE);

    oled.display();
}

void ledInit()
{
    strip.begin();
    strip.setBrightness(5);
    int moduleSize = WS2812B_NUM / 2;  // LEDs per module
    for (int i = 0; i < WS2812B_NUM; i++) {
        bool isLast = ((i + 1) % moduleSize == 1);
        strip.setPixelColor(i, isLast ? strip.Color(255, 255, 255)   // last of each module: red
                                      : strip.Color(255, 255, 255)); // rest: blue
    }
    strip.show();
}

void ledUpdate()
{
    static int lastMode = -1;

    int mode;
    if      (lineFollowMode)  mode = 2;   // green
    else if (!espNowPrimary)  mode = 1;   // blue
    else                      mode = 0;   // white

    if (mode == lastMode) return;         // no change — skip redundant show()
    lastMode = mode;

    uint32_t colour;
    switch (mode) {
        case 2:  colour = strip.Color(  0, 255,   0); break;  // line follow: green
        case 1:  colour = strip.Color(  0,   0, 255); break;  // UART:        blue
        default: colour = strip.Color(255, 0, 0); break;  // ESP-NOW:     white
    }

    for (int i = 0; i < WS2812B_NUM; i++)
        strip.setPixelColor(i, (i == 0 || i == 7) ? strip.Color(255, 255, 255) : colour);
    strip.show();
}

void displayInit()
{
    Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire1.setClock(400000);
    oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (!oledOk) {
        Serial.println("SSD1306 not found — check wiring on GPIO 25/32");
        return;
    }
    oled.clearDisplay();
    oled.display();

    xTaskCreatePinnedToCore(
        [](void*) {
            for (;;) {
                drawDisplay();
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        },
        "oled", 4096, nullptr, 1, nullptr, 0);
}
