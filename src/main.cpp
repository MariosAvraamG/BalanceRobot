#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_wifi.h"

static Adafruit_SSD1306 oled(128, 64, &Wire1, -1);

uint8_t receiverMac[] = {0xE8, 0x68, 0xE7, 0x30, 0x2A, 0xE9};

const int VRX_PIN       = 39;
const int VRY_PIN       = 34;
const int SW_PIN        = 27;
const int BTN_RED_PIN   = 26;
const int BTN_BLUE_PIN  = 25;
const int BTN_GREEN_PIN = 33;

const int BATT_PIN = 32;

typedef struct {
  float linear_vel;
  float angular_vel;
  bool  btn_red;
  bool  btn_blue;
  bool  btn_green;
} EspNowCmd;

typedef struct {
  int   mode;
  float soc;
  float linear;
  float angular;
} EspNowStatus;

EspNowCmd    outgoingMessage;
EspNowStatus incomingStatus;

static portMUX_TYPE statusMux    = portMUX_INITIALIZER_UNLOCKED;
volatile bool       statusUpdated = false;

int battPct = 0;

const float MAX_LINEAR_VEL  = 6.0f;
const float MAX_ANGULAR_VEL = 4.0f;

const int   X_CENTER      = 2800;
const int   Y_CENTER      = 2800;
const float DEADZONE       = 0.5f;
const float RESPONSE_CURVE = 1.5f;

float velTarget = 0.0f;
float turnBias  = 0.0f;

unsigned long lastRedSend   = 0;
unsigned long lastBlueSend  = 0;
unsigned long lastGreenSend = 0;

static portMUX_TYPE btnMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool redPressed   = false;
volatile bool bluePressed  = false;
volatile bool greenPressed = false;
volatile bool swPressed    = false;

static const uint64_t DEBOUNCE_US = 50000;

void IRAM_ATTR isrRed() {
  static uint64_t last = 0;
  uint64_t now = esp_timer_get_time();
  if (now - last < DEBOUNCE_US) return;
  last = now;
  portENTER_CRITICAL_ISR(&btnMux); redPressed = true; portEXIT_CRITICAL_ISR(&btnMux);
}
void IRAM_ATTR isrBlue() {
  static uint64_t last = 0;
  uint64_t now = esp_timer_get_time();
  if (now - last < DEBOUNCE_US) return;
  last = now;
  portENTER_CRITICAL_ISR(&btnMux); bluePressed = true; portEXIT_CRITICAL_ISR(&btnMux);
}
void IRAM_ATTR isrGreen() {
  static uint64_t last = 0;
  uint64_t now = esp_timer_get_time();
  if (now - last < DEBOUNCE_US) return;
  last = now;
  portENTER_CRITICAL_ISR(&btnMux); greenPressed = true; portEXIT_CRITICAL_ISR(&btnMux);
}
void IRAM_ATTR isrSw() {
  static uint64_t last = 0;
  uint64_t now = esp_timer_get_time();
  if (now - last < DEBOUNCE_US) return;
  last = now;
  portENTER_CRITICAL_ISR(&btnMux); swPressed = true; portEXIT_CRITICAL_ISR(&btnMux);
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void onRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  Serial.println("got a package");
  if (len != sizeof(EspNowStatus)) return;
  portENTER_CRITICAL_ISR(&statusMux);
  memcpy(&incomingStatus, data, sizeof(EspNowStatus));
  statusUpdated = true;
  portEXIT_CRITICAL_ISR(&statusMux);
}

float normalizeJoystick(int rawValue, int centerValue) {
  float deviation = (float)(rawValue - centerValue);
  float range = (deviation > 0) ? (4095.0f - centerValue) : (float)centerValue;
  float normalized = constrain(deviation / range, -1.0f, 1.0f);
  if (abs(normalized) < DEADZONE) return 0.0f;
  float sign = (normalized > 0) ? -1.0f : 1.0f;
  float remapped = (abs(normalized) - DEADZONE) / (1.0f - DEADZONE);
  return sign * pow(remapped, RESPONSE_CURVE);
}

void sendCmd() {
  portENTER_CRITICAL(&btnMux);
  bool red   = redPressed;   redPressed   = false;
  bool blue  = bluePressed;  bluePressed  = false;
  bool green = greenPressed; greenPressed = false;
  portEXIT_CRITICAL(&btnMux);

  unsigned long now = millis();
  if (red   && now - lastRedSend   >= 1000) { lastRedSend   = now; } else { red   = false; }
  if (blue  && now - lastBlueSend  >= 1000) { lastBlueSend  = now; } else { blue  = false; }
  if (green && now - lastGreenSend >= 1000) { lastGreenSend = now; } else { green = false; }

  outgoingMessage.linear_vel  = velTarget;
  outgoingMessage.angular_vel = turnBias;
  outgoingMessage.btn_red     = red;
  outgoingMessage.btn_blue    = blue;
  outgoingMessage.btn_green   = green;
  Serial.println(String(outgoingMessage.linear_vel) + " " + String(outgoingMessage.angular_vel) + " " +
    String(outgoingMessage.btn_red) + " " + String(outgoingMessage.btn_blue) + " " +
    String(outgoingMessage.btn_green));
  esp_now_send(receiverMac, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
}

void setup() {
  Serial.begin(115200);

  analogSetPinAttenuation(BATT_PIN, ADC_11db); 
  battPct = (int)constrain(analogRead(BATT_PIN) / 4095.0f * 110.0f, 0.0f, 100.0f);

  pinMode(SW_PIN,        INPUT_PULLUP);
  pinMode(BTN_RED_PIN,   INPUT_PULLUP);
  pinMode(BTN_BLUE_PIN,  INPUT_PULLUP);
  pinMode(BTN_GREEN_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SW_PIN),        isrSw,    FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_RED_PIN),   isrRed,   FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_BLUE_PIN),  isrBlue,  FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_GREEN_PIN), isrGreen, FALLING);

  Wire1.begin(21, 22);
  Wire1.setClock(400000);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
  }
  oled.clearDisplay();
  oled.display();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  Serial.printf("Controller MAC: %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNOW init failed");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 6;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Sender ready");
}

void loop() {
  int xRaw = analogRead(VRX_PIN);
  int yRaw = analogRead(VRY_PIN);

  float xNorm =  -normalizeJoystick(xRaw, X_CENTER);
  float yNorm = -normalizeJoystick(yRaw, Y_CENTER);

  if(xNorm == 0.0f && yNorm == 0.0f){
    velTarget = 0.0f;
    turnBias = 0.0f;
  }
  else if(xNorm == 0.0f && yNorm != 0.0f){
    velTarget = 0.0f;
    turnBias = yNorm * MAX_ANGULAR_VEL;
  }
  else if(xNorm != 0.0f && yNorm == 0.0f){
    velTarget = xNorm * MAX_LINEAR_VEL;
    turnBias = 0.0f;
  }
  else {
      velTarget = xNorm * MAX_LINEAR_VEL;
      turnBias = yNorm * MAX_ANGULAR_VEL;
  }

  static int screen = 0; 
  static unsigned long lastToggleMillis = 0;
  portENTER_CRITICAL(&btnMux);
  bool swSnap = swPressed; swPressed = false;
  portEXIT_CRITICAL(&btnMux);
  unsigned long nowMs = millis();

  if (swSnap && nowMs - lastToggleMillis >= 750) {
    screen = (screen + 1) % 3;
    lastToggleMillis = nowMs;
  }

  sendCmd();

  static unsigned long lastOledUpdate = 0;

  if (millis() - lastOledUpdate >= 500) {
    lastOledUpdate = millis();

    char buf[22];
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    EspNowStatus status;
    portENTER_CRITICAL(&statusMux);
    memcpy(&status, &incomingStatus, sizeof(EspNowStatus));
    bool gotStatus = statusUpdated; statusUpdated = false;
    portEXIT_CRITICAL(&statusMux);


    static const char* modeNames[] = { "Controller", "Vision", "InfraRed" };

    if (screen == 1) {

      oled.setTextSize(2);
      oled.setCursor(0, 0);
      snprintf(buf, sizeof(buf), "Ctrl:%3d%%", battPct);
      oled.print(buf);
      oled.setCursor(0, 24);
      snprintf(buf, sizeof(buf), "Bot: %3d%%", (int)status.soc);
      oled.print(buf);
    } else if (screen == 2) {

      oled.setTextSize(2);
      oled.setCursor(0, 0);
      snprintf(buf, sizeof(buf), "L:%+6.2f", status.linear);
      oled.print(buf);
      oled.setCursor(0, 18);
      snprintf(buf, sizeof(buf), "A:%+6.2f", status.angular);
      oled.print(buf);
    } else {

      oled.setTextSize(2);
      oled.setCursor(0, 0);
      snprintf(buf, sizeof(buf), "L:%+6.2f", outgoingMessage.linear_vel);
      oled.print(buf);
      oled.setCursor(0, 18);
      snprintf(buf, sizeof(buf), "A:%+6.2f", outgoingMessage.angular_vel);
      oled.print(buf);

      unsigned long now = millis();
      oled.setTextSize(1);
      oled.setCursor(0, 40);
      snprintf(buf, sizeof(buf), "Red:%d Blue:%d Grn:%d",
        now - lastRedSend   < 2000,
        now - lastBlueSend  < 2000,
        now - lastGreenSend < 2000);
      oled.print(buf);

      oled.setCursor(0, 54);
      int modeIdx = (status.mode >= 0 && status.mode <= 2) ? status.mode : 0;
      snprintf(buf, sizeof(buf), "Mode:%-5s", modeNames[modeIdx]);
      oled.print(buf);
    }

    oled.display();
  }

  delay(20);
}
