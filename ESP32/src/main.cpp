// src/main.cpp  (ESP32 - VOX EFX V4)
// TFT: ST7796 via TFT_eSPI
// UART0: ESP32 <-> Teensy (TX0/RX0)
// Meters: 2 segmented bars (INPUT/OUTPUT): 5 green, 2 amber, 1 red
// Encoder: adjusts pending level; press sends VOL,<pct>\n to Teensy
// Teensy sends: LVL,<pct>\n  and  MTR,<inSeg>,<outSeg>\n

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <string.h>

// ---------- Pins (from your schematic) ----------
static const int PIN_ENC_A  = 35;  // ENC_A
static const int PIN_ENC_B  = 34;  // ENC_B
static const int PIN_ENC_SW = 25;  // ENC_SW (active low)

// ---------- TFT ----------
TFT_eSPI tft;

// ---------- Encoder ----------
volatile int encDelta = 0;
int pendingLevel = 50;         // what user is adjusting
int teensyLevel  = -1;         // what Teensy reports (authoritative)

// ---------- UART RX line buffer ----------
static char rxLine[64];
static size_t rxLen = 0;
uint32_t lastTeensyRxMs = 0;

// ---------- Meter values from Teensy ----------
int inSeg = 0;     // 0..8
int outSeg = 0;    // 0..8
uint32_t lastMtrRxMs = 0;

// ---------- UI update control ----------
int lastShownPending = -999;
int lastShownTeensy  = -999;
static int lastInSeg  = -1;
static int lastOutSeg = -1;
uint32_t lastUiMs = 0;

// ---------- Button debounce ----------
uint32_t lastBtnMs = 0;
const uint32_t BTN_DEBOUNCE_MS = 180;

// ---------------- ISR ----------------
IRAM_ATTR void isrEncA() {
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  // If direction feels backwards, swap the ++/--
  if (a == b) encDelta++;
  else        encDelta--;
}

// ---------------- UI helpers ----------------
void drawStaticUI() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("VOX EFX V4");

  // Labels for meters
  tft.setCursor(10, 45);
  tft.print("INPUT");
  tft.setCursor(10, 125);
  tft.print("OUTPUT");

  // Numeric status area
  tft.setCursor(10, 205);
  tft.print("Pending:");
  tft.setCursor(10, 235);
  tft.print("Teensy :");
  tft.setCursor(10, 265);
  tft.print("MTR age:");

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 295);
  tft.print("Press encoder to send VOL");
}

void drawValueAt(int x, int y, int value, uint16_t color, int widthChars = 4) {
  // Overwrite a small region where numbers live
  int pxW = 12 * widthChars; // approx for textSize(2)
  int pxH = 16 * 2;
  tft.fillRect(x, y, pxW, pxH, TFT_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  if (value < 0) tft.print("--");
  else tft.print(value);
}

void drawAgeAt(int x, int y, uint32_t ageMs, uint16_t color) {
  tft.fillRect(x, y, 160, 32, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  tft.print(ageMs);
}

void drawSegmentMeter(int x, int y, int segLit) {
  const int segCount = 8;
  const int segW = 28;
  const int segH = 22;
  const int gap  = 6;

  for (int i = 1; i <= segCount; i++) {
    int sx = x + (i - 1) * (segW + gap);
    int sy = y;

    uint16_t onColor;
    if (i <= 5) onColor = TFT_GREEN;
    else if (i <= 7) onColor = TFT_ORANGE;
    else onColor = TFT_RED;

    bool lit = (i <= segLit);
    if (lit) {
      tft.fillRoundRect(sx, sy, segW, segH, 4, onColor);
      tft.drawRoundRect(sx, sy, segW, segH, 4, onColor);
    } else {
      tft.fillRoundRect(sx, sy, segW, segH, 4, TFT_BLACK);
      tft.drawRoundRect(sx, sy, segW, segH, 4, TFT_DARKGREY);
    }
  }
}

void drawMetersIfChanged() {
  // meter segments
  if (inSeg != lastInSeg) {
    lastInSeg = inSeg;
    drawSegmentMeter(90, 70, inSeg);   // INPUT meter bar
  }
  if (outSeg != lastOutSeg) {
    lastOutSeg = outSeg;
    drawSegmentMeter(90, 150, outSeg); // OUTPUT meter bar
  }

  // comms age indicator
  uint32_t age = (lastMtrRxMs == 0) ? 999999UL : (millis() - lastMtrRxMs);
  uint16_t ageColor = (age < 300) ? TFT_GREEN : TFT_RED;
  drawAgeAt(120, 265, age, ageColor);
}

// ---------------- UART ----------------
void sendPendingToTeensy() {
  pendingLevel = constrain(pendingLevel, 0, 100);
  Serial.print("VOL,");
  Serial.print(pendingLevel);
  Serial.print("\n");
}

void processUartLine(const char* line) {
  // LVL,<0-100>
  if (strncmp(line, "LVL", 3) == 0) {
    const char* p = line + 3;
    while (*p && (*p == ',' || *p == ':' || *p == ' ')) p++;
    int v = atoi(p);
    if (v >= 0 && v <= 100) {
      teensyLevel = v;
      lastTeensyRxMs = millis();
    }
    return;
  }

  // MTR,<inSeg>,<outSeg>
  if (strncmp(line, "MTR", 3) == 0) {
    const char* p = line + 3;
    while (*p && (*p == ',' || *p == ':' || *p == ' ')) p++;
    int a = atoi(p);
    const char* c = strchr(p, ',');
    int b = c ? atoi(c + 1) : -1;

    if (a >= 0 && a <= 8 && b >= 0 && b <= 8) {
      inSeg = a;
      outSeg = b;
      lastMtrRxMs = millis();
    }
    return;
  }
}

void pollUart() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rxLine[rxLen] = '\0';
      if (rxLen > 0) processUartLine(rxLine);
      rxLen = 0;
    } else {
      if (rxLen < sizeof(rxLine) - 1) rxLine[rxLen++] = c;
      else rxLen = 0; // overflow reset
    }
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  // UART0 to Teensy
  Serial.begin(115200);

  // TFT
  tft.init();
  tft.setRotation(1);
  drawStaticUI();

  // Draw initial meters (all off)
  drawSegmentMeter(90, 70, 0);
  drawSegmentMeter(90, 150, 0);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
}

void loop() {
  pollUart();

  // apply encoder movement
  int d = 0;
  noInterrupts();
  d = encDelta;
  encDelta = 0;
  interrupts();

  if (d != 0) {
    pendingLevel = constrain(pendingLevel + d, 0, 100);
  }

  // button press commits pending to Teensy
  uint32_t now = millis();
  if (digitalRead(PIN_ENC_SW) == LOW && (now - lastBtnMs) > BTN_DEBOUNCE_MS) {
    lastBtnMs = now;
    sendPendingToTeensy();
  }

  // UI refresh at ~20 Hz max
  if (now - lastUiMs > 50) {
    lastUiMs = now;

    // Numeric displays
    if (pendingLevel != lastShownPending) {
      lastShownPending = pendingLevel;
      drawValueAt(120, 205, pendingLevel, TFT_YELLOW);
    }
    if (teensyLevel != lastShownTeensy) {
      lastShownTeensy = teensyLevel;
      drawValueAt(120, 235, teensyLevel, TFT_CYAN);
    }

    // Optional LVL RX age (small, white/red)
    uint32_t ageLvl = (lastTeensyRxMs == 0) ? 999999UL : (now - lastTeensyRxMs);
    uint16_t lvlColor = (ageLvl < 1200) ? TFT_WHITE : TFT_RED;
    // show it to the right of "Teensy :" as a subtle indicator
    tft.fillRect(200, 235, 110, 32, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(lvlColor, TFT_BLACK);
    tft.setCursor(200, 235);
    tft.print("(");
    tft.print(ageLvl);
    tft.print(")");

    // Meter bars + meter RX age
    drawMetersIfChanged();
  }

  delay(2);
}
