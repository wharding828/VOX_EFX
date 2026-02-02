#include <Arduino.h>
#include <TFT_eSPI.h>
#include <string.h>
#include <stdlib.h>

// ---------------- Pins ----------------
static const int PIN_ENC_A  = 35;
static const int PIN_ENC_B  = 34;
static const int PIN_ENC_SW = 25;   // active low (we WILL use this as fallback)

// ---------------- TFT ----------------
TFT_eSPI tft;

// ---------------- Layout constants ----------------
// LEDs
static const int UI_LED_COMMS_X = 15;
static const int UI_LED_COMMS_Y = 15;
static const int UI_LED_DLY_X   = 15;
static const int UI_LED_DLY_Y   = 35;

// Labels
static const int UI_LABEL_X     = 10;
static const int UI_INPUT_LBL_Y = 60;
static const int UI_OUT_LBL_Y   = 90;

// Meter bars
static const int UI_METER_X     = 88;
static const int UI_IN_METER_Y  = 58;
static const int UI_OUT_METER_Y = 88;

// Volume / Delay text
static const int UI_VOL_LBL_Y   = 145;
static const int UI_DLY_LBL_Y   = 175;
static const int UI_VOL_VAL_X   = 120;
static const int UI_VOL_VAL_Y   = 145;
static const int UI_VOL_RPT_X   = 220;
static const int UI_VOL_RPT_Y   = 145;
static const int UI_DLY_VAL_X   = 120;
static const int UI_DLY_VAL_Y   = 175;

// Debug panel (bottom area)
static const int UI_DBG_X       = 10;
static const int UI_DBG_Y0      = 210;
static const int UI_DBG_LINE_H  = 22;

// ---- "Button" area: DRY channel toggle ----
static const int UI_BTN_X = 240;
static const int UI_BTN_Y = 6;
static const int UI_BTN_W = 150;
static const int UI_BTN_H = 40;

// ---------------- Encoder smoothing ----------------
volatile int encDelta = 0;
int encAccum = 0;

int volume = 50;
int teensyVolume = -1;

static const int ENC_STEP = 2;
static const int ENC_DEADBAND = 2;
static const uint32_t VOL_TX_PERIOD_MS = 40;

// Encoder button debounce
static uint32_t lastBtnMs = 0;
static const uint32_t BTN_DEBOUNCE_MS = 220;

// ---------------- UART ----------------
static char rxLine[128];
static size_t rxLen = 0;
uint32_t lastRxMs = 0;

// ---------------- Meters ----------------
int inSeg = 0;
int outSeg = 0;

// ---------------- Delay ----------------
int delayOn = 0;

// ---------------- Dry Channel ----------------
int dryChan = 0;        // 0 or 2 (from Teensy)
int lastDryChanShown = -1;

// ---------------- Debug telemetry ----------------
float dbgDry = -1.0f;
float dbgWet = -1.0f;
int   dbgDtMs = -1;
float dbgPki = -1.0f;
float dbgPkd = -1.0f;
float dbgPks = -1.0f;
float dbgPko = -1.0f;
bool  dbgSeen = false;

// ---------------- UI state ----------------
int lastVolShown = -999;
int lastTeensyVolShown = -999;
int lastInSeg = -1;
int lastOutSeg = -1;
int lastDelayShown = -1;
bool lastCommsGood = false;

uint32_t lastUiMs = 0;
uint32_t lastVolTxMs = 0;
uint32_t lastTouchMs = 0;

// ---------------- ISR ----------------
IRAM_ATTR void isrEncA() {
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  if (a == b) encDelta++;
  else        encDelta--;
}

// ---------------- UI helpers ----------------
void drawLed(int x, int y, uint16_t color) {
  tft.fillCircle(x, y, 6, color);
}

void drawValue(int x, int y, int v, uint16_t color) {
  tft.fillRect(x, y, 80, 30, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  if (v < 0) tft.print("--");
  else tft.print(v);
}

void drawValueFloat(int x, int y, float v, uint16_t color, int widthPx = 160) {
  tft.fillRect(x, y, widthPx, 20, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  if (v < 0.0f) tft.print("--");
  else tft.print(v, 2);
}

void drawTextField(int x, int y, const char* s, uint16_t color, int widthPx = 200) {
  tft.fillRect(x, y, widthPx, 20, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  tft.print(s);
}

void drawSegmentMeter(int x, int y, int segLit) {
  const int segW = 26, segH = 22, gap = 4;

  for (int i = 1; i <= 8; i++) {
    int sx = x + (i - 1) * (segW + gap);
    uint16_t color =
      (i <= 5) ? TFT_GREEN :
      (i <= 7) ? TFT_ORANGE :
                 TFT_RED;

    bool on = (i <= segLit);
    tft.fillRoundRect(sx, y, segW, segH, 4, on ? color : TFT_BLACK);
    tft.drawRoundRect(sx, y, segW, segH, 4, on ? color : TFT_DARKGREY);
  }
}

void drawDryChanButton() {
  // Frame
  tft.drawRoundRect(UI_BTN_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H, 8, TFT_DARKGREY);

  // Fill
  uint16_t fill = (dryChan == 2) ? TFT_DARKGREY : TFT_BLACK;
  tft.fillRoundRect(UI_BTN_X + 1, UI_BTN_Y + 1, UI_BTN_W - 2, UI_BTN_H - 2, 8, fill);

  // Text
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, fill);
  tft.setCursor(UI_BTN_X + 10, UI_BTN_Y + 10);
  tft.print("DRY ");
  tft.print((dryChan == 2) ? "CH2" : "CH0");
}

static bool pointInRect(int x, int y, int rx, int ry, int rw, int rh) {
  return (x >= rx && x < (rx + rw) && y >= ry && y < (ry + rh));
}

void drawStaticUI() {
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Labels
  tft.setCursor(UI_LABEL_X, UI_INPUT_LBL_Y);  tft.print("INPUT");
  tft.setCursor(UI_LABEL_X, UI_OUT_LBL_Y);    tft.print("OUTPUT");

  tft.setCursor(UI_LABEL_X, UI_VOL_LBL_Y);    tft.print("Volume:");
  tft.setCursor(UI_LABEL_X, UI_DLY_LBL_Y);    tft.print("Delay :");

  // Meter frames
  tft.drawRoundRect(UI_METER_X - 3, UI_IN_METER_Y - 2, 8 * 26 + 7 * 4 + 6, 22 + 4, 6, TFT_DARKGREY);
  tft.drawRoundRect(UI_METER_X - 3, UI_OUT_METER_Y - 2, 8 * 26 + 7 * 4 + 6, 22 + 4, 6, TFT_DARKGREY);

  // Legend
  tft.setCursor(30, 8);  tft.print("COMMS");
  tft.setCursor(30, 28); tft.print("DELAY");

  // Dry button
  drawDryChanButton();

  // Debug header + labels
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(UI_DBG_X, UI_DBG_Y0);
  tft.print("DBG:");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setCursor(UI_DBG_X, UI_DBG_Y0 + UI_DBG_LINE_H * 1);
  tft.print("DRYCH:");
  tft.setCursor(UI_DBG_X + 160, UI_DBG_Y0 + UI_DBG_LINE_H * 1);
  tft.print("DT(ms):");

  tft.setCursor(UI_DBG_X, UI_DBG_Y0 + UI_DBG_LINE_H * 2);
  tft.print("DRY:");
  tft.setCursor(UI_DBG_X + 160, UI_DBG_Y0 + UI_DBG_LINE_H * 2);
  tft.print("WET:");

  tft.setCursor(UI_DBG_X, UI_DBG_Y0 + UI_DBG_LINE_H * 3);
  tft.print("PKI:");
  tft.setCursor(UI_DBG_X + 160, UI_DBG_Y0 + UI_DBG_LINE_H * 3);
  tft.print("PKD:");

  tft.setCursor(UI_DBG_X, UI_DBG_Y0 + UI_DBG_LINE_H * 4);
  tft.print("PKS:");
  tft.setCursor(UI_DBG_X + 160, UI_DBG_Y0 + UI_DBG_LINE_H * 4);
  tft.print("PKO:");
}

// ---------------- UART helpers ----------------
void sendVolume() {
  uint32_t now = millis();
  if (now - lastVolTxMs < VOL_TX_PERIOD_MS) return;
  lastVolTxMs = now;

  Serial.print("VOL,");
  Serial.print(volume);
  Serial.print("\n");
}

void requestDryChan(int target) {
  target = (target == 2) ? 2 : 0;
  Serial.print("CH,");
  Serial.print(target);
  Serial.print("\n");
}

void toggleDryChanRequest() {
  int target = (dryChan == 2) ? 0 : 2;
  requestDryChan(target);

  // optimistic UI update; Teensy will confirm via CH/DBG
  dryChan = target;
  drawDryChanButton();
}

static bool parseKeyFloat(const char* line, const char* key, float &outVal) {
  const char* p = strstr(line, key);
  if (!p) return false;
  p += strlen(key);
  if (*p != '=') return false;
  p++;
  outVal = (float)atof(p);
  return true;
}

static bool parseKeyInt(const char* line, const char* key, int &outVal) {
  const char* p = strstr(line, key);
  if (!p) return false;
  p += strlen(key);
  if (*p != '=') return false;
  p++;
  outVal = atoi(p);
  return true;
}

void processLine(const char* line) {
  lastRxMs = millis();

  if (!strncmp(line, "LVL", 3)) {
    teensyVolume = atoi(line + 4);
    return;
  }

  if (!strncmp(line, "MTR", 3)) {
    inSeg  = atoi(line + 4);
    const char* c = strchr(line + 4, ',');
    if (c) outSeg = atoi(c + 1);
    return;
  }

  if (!strncmp(line, "DLY", 3)) {
    delayOn = atoi(line + 4);
    return;
  }

  // CH,<0|2>
  if (!strncmp(line, "CH", 2)) {
    dryChan = atoi(line + 3);
    if (dryChan != 2) dryChan = 0;
    return;
  }

  // DBG,DRYCH=0,DRY=1.00,WET=0.00,DT=180,PKI=0.12,PKD=0.05,PKS=0.10,PKO=0.09
  if (!strncmp(line, "DBG", 3)) {
    parseKeyInt(line, "DRYCH", dryChan);
    if (dryChan != 2) dryChan = 0;

    parseKeyFloat(line, "DRY",  dbgDry);
    parseKeyFloat(line, "WET",  dbgWet);
    parseKeyInt  (line, "DT",   dbgDtMs);
    parseKeyFloat(line, "PKI",  dbgPki);
    parseKeyFloat(line, "PKD",  dbgPkd);
    parseKeyFloat(line, "PKS",  dbgPks);
    parseKeyFloat(line, "PKO",  dbgPko);

    dbgSeen = true;
    return;
  }
}

void pollUart() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rxLine[rxLen] = 0;
      if (rxLen) processLine(rxLine);
      rxLen = 0;
    } else if (rxLen < sizeof(rxLine) - 1) {
      rxLine[rxLen++] = c;
    }
  }
}

// ---------------- Optional Touch (compile-safe) ----------------
// TFT_eSPI only provides getTouch() if touch is enabled in User_Setup.h.
// We'll compile touch code ONLY when TOUCH_CS is defined.
void pollTouch() {
#if defined(TOUCH_CS)
  uint16_t tx, ty;
  if (!tft.getTouch(&tx, &ty)) return;

  uint32_t now = millis();
  if (now - lastTouchMs < 250) return;
  lastTouchMs = now;

  if (pointInRect((int)tx, (int)ty, UI_BTN_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H)) {
    toggleDryChanRequest();
  }
#else
  // No touch compiled in. Use encoder press fallback.
  (void)lastTouchMs;
#endif
}

// ---------------- Encoder button fallback ----------------
void pollEncButton() {
  uint32_t now = millis();
  if (digitalRead(PIN_ENC_SW) == LOW && (now - lastBtnMs) > BTN_DEBOUNCE_MS) {
    lastBtnMs = now;
    toggleDryChanRequest();
  }
}

// ---------------- Setup ----------------
void setup() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  Serial.begin(115200);

  tft.init();
  drawStaticUI();

  drawLed(UI_LED_COMMS_X, UI_LED_COMMS_Y, TFT_DARKGREY);
  drawLed(UI_LED_DLY_X, UI_LED_DLY_Y, TFT_DARKGREY);

  drawSegmentMeter(UI_METER_X, UI_IN_METER_Y, 0);
  drawSegmentMeter(UI_METER_X, UI_OUT_METER_Y, 0);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
}

// ---------------- Loop ----------------
void loop() {
  pollUart();
  pollTouch();
  pollEncButton();

  uint32_t now = millis();

  // -------- Encoder smoothing --------
  int d;
  noInterrupts();
  d = encDelta;
  encDelta = 0;
  interrupts();

  encAccum += d;
  if (abs(encAccum) >= ENC_DEADBAND) {
    int steps = encAccum / ENC_DEADBAND;
    encAccum -= steps * ENC_DEADBAND;

    volume = constrain(volume + steps * ENC_STEP, 0, 100);
    sendVolume();
  }

  // -------- UI refresh --------
  if (now - lastUiMs > 50) {
    lastUiMs = now;

    // Comms LED
    bool commsGood = (now - lastRxMs) < 300;
    if (commsGood != lastCommsGood) {
      lastCommsGood = commsGood;
      drawLed(UI_LED_COMMS_X, UI_LED_COMMS_Y, commsGood ? TFT_GREEN : TFT_DARKGREY);
    }

    // Delay LED + value
    if (delayOn != lastDelayShown) {
      lastDelayShown = delayOn;
      drawLed(UI_LED_DLY_X, UI_LED_DLY_Y, delayOn ? TFT_GREEN : TFT_DARKGREY);
      drawValue(UI_DLY_VAL_X, UI_DLY_VAL_Y, delayOn ? 1 : 0, delayOn ? TFT_GREEN : TFT_LIGHTGREY);
    }

    // Volume displays
    if (volume != lastVolShown) {
      lastVolShown = volume;
      drawValue(UI_VOL_VAL_X, UI_VOL_VAL_Y, volume, TFT_YELLOW);
    }
    if (teensyVolume != lastTeensyVolShown) {
      lastTeensyVolShown = teensyVolume;
      drawValue(UI_VOL_RPT_X, UI_VOL_RPT_Y, teensyVolume, TFT_CYAN);
    }

    // Meters
    if (inSeg != lastInSeg) {
      lastInSeg = inSeg;
      drawSegmentMeter(UI_METER_X, UI_IN_METER_Y, inSeg);
    }
    if (outSeg != lastOutSeg) {
      lastOutSeg = outSeg;
      drawSegmentMeter(UI_METER_X, UI_OUT_METER_Y, outSeg);
    }

    // Dry button state refresh (if Teensy changed it)
    if (dryChan != lastDryChanShown) {
      lastDryChanShown = dryChan;
      drawDryChanButton();
    }

    // ---- DEBUG PANEL ----
    if (!dbgSeen) {
      drawTextField(UI_DBG_X + 70,  UI_DBG_Y0 + UI_DBG_LINE_H * 1, "--", TFT_WHITE, 70);
      drawTextField(UI_DBG_X + 240, UI_DBG_Y0 + UI_DBG_LINE_H * 1, "--", TFT_WHITE, 80);

      drawTextField(UI_DBG_X + 60,  UI_DBG_Y0 + UI_DBG_LINE_H * 2, "--", TFT_WHITE, 90);
      drawTextField(UI_DBG_X + 220, UI_DBG_Y0 + UI_DBG_LINE_H * 2, "--", TFT_WHITE, 90);

      drawTextField(UI_DBG_X + 60,  UI_DBG_Y0 + UI_DBG_LINE_H * 3, "--", TFT_WHITE, 90);
      drawTextField(UI_DBG_X + 220, UI_DBG_Y0 + UI_DBG_LINE_H * 3, "--", TFT_WHITE, 90);

      drawTextField(UI_DBG_X + 60,  UI_DBG_Y0 + UI_DBG_LINE_H * 4, "--", TFT_WHITE, 120);
      drawTextField(UI_DBG_X + 220, UI_DBG_Y0 + UI_DBG_LINE_H * 4, "--", TFT_WHITE, 120);
    } else {
      // DRYCH + DT
      char chBuf[8]; snprintf(chBuf, sizeof(chBuf), "%d", dryChan);
      drawTextField(UI_DBG_X + 70,  UI_DBG_Y0 + UI_DBG_LINE_H * 1, chBuf, TFT_YELLOW, 70);

      char dtBuf[16]; snprintf(dtBuf, sizeof(dtBuf), "%d", dbgDtMs);
      drawTextField(UI_DBG_X + 240, UI_DBG_Y0 + UI_DBG_LINE_H * 1, dtBuf, TFT_YELLOW, 80);

      // DRY / WET
      drawValueFloat(UI_DBG_X + 60,  UI_DBG_Y0 + UI_DBG_LINE_H * 2, dbgDry, TFT_CYAN, 90);
      drawValueFloat(UI_DBG_X + 220, UI_DBG_Y0 + UI_DBG_LINE_H * 2, dbgWet, TFT_CYAN, 90);

      // PKI / PKD
      drawValueFloat(UI_DBG_X + 60,  UI_DBG_Y0 + UI_DBG_LINE_H * 3, dbgPki, TFT_GREEN, 90);
      drawValueFloat(UI_DBG_X + 220, UI_DBG_Y0 + UI_DBG_LINE_H * 3, dbgPkd, TFT_GREEN, 90);

      // PKS / PKO
      drawValueFloat(UI_DBG_X + 60,  UI_DBG_Y0 + UI_DBG_LINE_H * 4, dbgPks, TFT_ORANGE, 120);
      drawValueFloat(UI_DBG_X + 220, UI_DBG_Y0 + UI_DBG_LINE_H * 4, dbgPko, TFT_ORANGE, 120);
    }
  }

  delay(2);
}
