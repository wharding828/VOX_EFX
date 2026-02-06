// ============================================================
// VOX EFX - ESP32 UI Skeleton (CLEANED, BEST METHOD)
// BEST METHOD: Minimal built-in FT6336U I2C reader (NO external touch library)
//  - Works even if GitHub libraries disappear / rename headers
//  - Reads FT6336U over Wire (I2C) at 0x38
//
//  - 5 pages (encoder nav + edit mode toggle)
//  - Vertical meters always visible (IN left, OUT right)
//  - CONFIG page with live Teensy DBG line (single-line window)
//
// TEMPORARY TEST CODE IS CLEARLY MARKED WITH:
//   // >>> TEMP TOUCH TEST START
//   // >>> TEMP TOUCH TEST END
// Remove that block after touch mapping is confirmed.
// ============================================================

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>

// ---------------- TFT ----------------
TFT_eSPI tft;

// ---------------- Pins ----------------
static const int PIN_ENC_A  = 35;
static const int PIN_ENC_B  = 34;
static const int PIN_ENC_SW = 25;   // active low

// ---------------- Pages ----------------
enum UiPage {
  PAGE_FX1 = 0,
  PAGE_FX2,
  PAGE_FX3,
  PAGE_FX4,
  PAGE_CONFIG,
  PAGE_COUNT
};

UiPage currentPage   = PAGE_FX1;
UiPage lastPage      = PAGE_COUNT;   // force redraw
bool   editMode      = false;
bool   lastEditMode  = false;

const char* pageNames[PAGE_COUNT] = {
  "REVERB",
  "DELAY",
  "CHORUS",
  "SATURATION",
  "CONFIG"
};

// ---------------- Config values (ESP32-side for now) ----------------
int  cfgInputGain = 8;        // 0..15 (SGTL5000 lineInLevel)
int  cfgOutputLvl = 72;       // 0..100 (maps to amp gain on Teensy)
int  cfgHpfIdx    = 3;        // index into HPF list below
bool cfgGateOn    = true;
int  cfgBrightPct = 80;       // 0..100 (ESP32 backlight control later)

static const int HPF_LIST[] = { 0, 80, 100, 120, 150 }; // 0 = OFF
static const int HPF_COUNT  = sizeof(HPF_LIST) / sizeof(HPF_LIST[0]);

// ---------------- Vertical meters ----------------
static const int METER_W   = 18;
static const int METER_H   = 160;
static const int IN_METER_Y  = 50;
static const int OUT_METER_Y = 50;

static const int LEFT_METER_X = 6;
static inline int rightMeterX() { return tft.width() - METER_W - 6; }

// ---------------- Content region (between meters) ----------------
// NOTE: assumes rotation(1) = 480x320
static const int CONTENT_X = LEFT_METER_X + METER_W + 10;
static const int CONTENT_Y = 44;
static const int CONTENT_W = 480 - CONTENT_X - (METER_W + 10);
static const int CONTENT_H = 320 - CONTENT_Y - 10;

// ---------------- Live DBG line ----------------
static const int DBG_LINE_LEN = 110;
char dbgLine[DBG_LINE_LEN] = "(waiting for DBG...)";
bool dbgDirty = true;

// ---------------- Encoder ----------------
volatile int encDelta = 0;
int encAccum = 0;

static const int ENC_DEADBAND = 1;
static const uint32_t BTN_DEBOUNCE_MS = 220;
static uint32_t lastBtnMs = 0;

// ---------------- UART ----------------
static char   rxLine[128];
static size_t rxLen = 0;
uint32_t lastRxMs = 0;

// ---------------- Meters ----------------
int inSeg = 0, outSeg = 0;
int lastInSeg = -1, lastOutSeg = -1;

// ---------------- UI timing ----------------
uint32_t lastUiMs = 0;

// ============================================================
// Minimal FT6336U I2C driver (NO external library)
// ============================================================
static const uint8_t FT_ADDR     = 0x38;
static const uint8_t REG_TD_STAT = 0x02; // touch points (low nibble)
static const uint8_t REG_P1_XH   = 0x03;
static const uint8_t REG_P1_XL   = 0x04;
static const uint8_t REG_P1_YH   = 0x05;
static const uint8_t REG_P1_YL   = 0x06;

static bool ftReadRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(FT_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false; // repeated-start
  int n = Wire.requestFrom((int)FT_ADDR, (int)len);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Returns true if touch detected; sx/sy are SCREEN coordinates (rotation(1) guess)
static bool readTouch(int &sx, int &sy) {
  uint8_t td = 0;
  if (!ftReadRegs(REG_TD_STAT, &td, 1)) return false;
  if ((td & 0x0F) == 0) return false;

  uint8_t b[4];
  if (!ftReadRegs(REG_P1_XH, b, 4)) return false;

  uint16_t x = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
  uint16_t y = ((uint16_t)(b[2] & 0x0F) << 8) | b[3];

  // --- TEMP mapping guess ---
  // Many panels report portrait coords (0..319, 0..479) while we use landscape 480x320.
  // Start with a common transform for rotation(1).
  int rawX = (int)x;
  int rawY = (int)y;

  sx = constrain(rawY, 0, 479);
  sy = constrain(320 - rawX, 0, 319);
  return true;
}

// ---------------- Forward declarations ----------------
void drawConfigPage();
void drawLogWindow();

// ---------------- ISR ----------------
IRAM_ATTR void isrEncA() {
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  if (a == b) encDelta++;
  else        encDelta--;
}

// ---------------- UI helpers ----------------
static const int ROW_H = 26;

void drawRowLabel(int x, int y, const char* label) {
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(x, y);
  tft.print(label);
}

void drawRowValueBox(int x, int y, int w, const char* value, bool selected) {
  uint16_t frame = selected ? TFT_YELLOW : TFT_DARKGREY;

  tft.drawRoundRect(x, y - 2, w, ROW_H, 6, frame);
  tft.fillRoundRect(x + 1, y - 1, w - 2, ROW_H - 2, 6, TFT_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(selected ? TFT_YELLOW : TFT_CYAN, TFT_BLACK);
  tft.setCursor(x + 8, y);
  tft.print(value);
}

void drawVerticalMeter(int x, int y, int segLit) {
  const int segCount = 8;
  const int segGap = 3;
  int segH = (METER_H - (segCount - 1) * segGap) / segCount;

  for (int i = 0; i < segCount; i++) {
    int sy = y + METER_H - (i + 1) * (segH + segGap) + segGap;

    uint16_t color =
      (i < 4) ? TFT_GREEN :
      (i < 6) ? TFT_ORANGE :
                TFT_RED;

    bool on = (i < segLit);
    tft.fillRect(x, sy, METER_W, segH, on ? color : TFT_BLACK);
    tft.drawRect(x, sy, METER_W, segH, TFT_DARKGREY);
  }
}

void drawStaticUI() {
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawFastHLine(0, 34, tft.width(), TFT_DARKGREY);

  drawVerticalMeter(LEFT_METER_X, IN_METER_Y, 0);
  drawVerticalMeter(rightMeterX(), OUT_METER_Y, 0);
}

void drawPageHeader() {
  tft.fillRect(0, 0, tft.width(), 34, TFT_BLACK);

  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(pageNames[currentPage]);

  if (editMode) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(180, 8);
    tft.print("EDIT");
  }
}

void drawClippedText(int x, int y, int w, const char* s) {
  tft.setCursor(x, y);
  int maxChars = w / 6; // approx for textSize(1)
  for (int i = 0; i < maxChars && s[i]; i++) tft.print(s[i]);
}

// ---------------- CONFIG log window geometry ----------------
static inline void configLogRect(int &x, int &y, int &w, int &h) {
  const int padY = 6;
  const int textH = 10;        // textSize(1) one line
  const int boxH = textH + padY * 2;

  x = CONTENT_X;
  w = CONTENT_W;
  h = boxH;

  const int bottomMargin = 8;
  y = tft.height() - bottomMargin - h;
}

// ---------------- CONFIG page ----------------
void drawConfigPage() {
  tft.fillRect(CONTENT_X, CONTENT_Y, CONTENT_W, CONTENT_H, TFT_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(CONTENT_X, CONTENT_Y);
  tft.print("GLOBAL SETTINGS");

  const int labelX = CONTENT_X;
  const int valueW = 120;
  const int valueX = CONTENT_X + CONTENT_W - valueW;
  int y = CONTENT_Y + 30;

  char buf[32];

  drawRowLabel(labelX, y, "Input Gain");
  snprintf(buf, sizeof(buf), "%d", cfgInputGain);
  drawRowValueBox(valueX, y, valueW, buf, false);

  y += ROW_H + 6;
  drawRowLabel(labelX, y, "Output Level");
  snprintf(buf, sizeof(buf), "%d", cfgOutputLvl);
  drawRowValueBox(valueX, y, valueW, buf, false);

  y += ROW_H + 6;
  drawRowLabel(labelX, y, "HPF Cutoff");
  if (HPF_LIST[cfgHpfIdx] == 0) snprintf(buf, sizeof(buf), "OFF");
  else snprintf(buf, sizeof(buf), "%dHz", HPF_LIST[cfgHpfIdx]);
  drawRowValueBox(valueX, y, valueW, buf, false);

  y += ROW_H + 6;
  drawRowLabel(labelX, y, "Noise Gate");
  snprintf(buf, sizeof(buf), "%s", cfgGateOn ? "ON" : "OFF");
  drawRowValueBox(valueX, y, valueW, buf, false);

  y += ROW_H + 6;
  drawRowLabel(labelX, y, "Brightness");
  snprintf(buf, sizeof(buf), "%d%%", cfgBrightPct);
  drawRowValueBox(valueX, y, valueW, buf, false);

  int logX, logY, logW, logH;
  configLogRect(logX, logY, logW, logH);

  tft.drawFastHLine(CONTENT_X, logY - 8, CONTENT_W, TFT_DARKGREY);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(logX, logY - 24);
  tft.print("LIVE DEBUG");

  tft.drawRect(logX, logY, logW, logH, TFT_DARKGREY);

  dbgDirty = true;
}

void drawLogWindow() {
  if (currentPage != PAGE_CONFIG) return;
  if (!dbgDirty) return;
  dbgDirty = false;

  int logX, logY, logW, logH;
  configLogRect(logX, logY, logW, logH);

  tft.fillRect(logX + 2, logY + 2, logW - 4, logH - 4, TFT_BLACK);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextSize(1);

  drawClippedText(logX + 6, logY + 6, logW - 12, dbgLine);

  tft.setTextSize(2);
}

// ---------------- UART parsing ----------------
void processLine(const char* line) {
  lastRxMs = millis();

  // DBG filtering (only capture DBG lines; strip "DBG,")
  if (!strncmp(line, "DBG,", 4)) {
    const char* p = line + 4;
    strncpy(dbgLine, p, DBG_LINE_LEN - 1);
    dbgLine[DBG_LINE_LEN - 1] = '\0';
    dbgDirty = true;
  }

  // Meters
  if (!strncmp(line, "MTR", 3)) {
    inSeg = atoi(line + 4);
    const char* c = strchr(line + 4, ',');
    if (c) outSeg = atoi(c + 1);
  }
}

void pollUart() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rxLine[rxLen] = 0;
      if (rxLen) processLine(rxLine);
      rxLen = 0;
    } else if (rxLen < sizeof(rxLine) - 1) {
      rxLine[rxLen++] = c;
    } else {
      rxLen = 0;
    }
  }
}

// ---------------- Encoder button ----------------
void pollEncButton() {
  uint32_t now = millis();
  if (digitalRead(PIN_ENC_SW) == LOW && (now - lastBtnMs) > BTN_DEBOUNCE_MS) {
    lastBtnMs = now;
    editMode = !editMode;
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
  drawPageHeader();

  // I2C for touch
  Wire.begin(); // if needed: Wire.begin(21, 22);

  // Touch presence check
  uint8_t td = 0;
  bool ok = ftReadRegs(REG_TD_STAT, &td, 1);
  strncpy(dbgLine, ok ? "TOUCH OK (FT6336U @0x38)" : "TOUCH FAIL (FT6336U @0x38)", DBG_LINE_LEN - 1);
  dbgLine[DBG_LINE_LEN - 1] = '\0';
  dbgDirty = true;

  if (currentPage == PAGE_CONFIG) drawConfigPage();

  lastPage = currentPage;
  lastEditMode = editMode;

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
}

// ---------------- Loop ----------------
void loop() {
  pollUart();
  pollEncButton();

  // Encoder navigation
  int d;
  noInterrupts();
  d = encDelta;
  encDelta = 0;
  interrupts();

  encAccum += d;
  if (abs(encAccum) >= ENC_DEADBAND) {
    int steps = encAccum / ENC_DEADBAND;
    encAccum -= steps * ENC_DEADBAND;

    if (!editMode) {
      int p = (int)currentPage + steps;
      if (p < 0) p = 0;
      if (p >= PAGE_COUNT) p = PAGE_COUNT - 1;
      currentPage = (UiPage)p;
    }
  }

  // >>> TEMP TOUCH TEST START
  // Show a dot at the touch point and overwrite dbgLine with coordinates.
  // REMOVE this block once mapping is verified.
  int tx, ty;
  if (readTouch(tx, ty)) {
    tft.fillCircle(tx, ty, 3, TFT_YELLOW);
    char buf[64];
    snprintf(buf, sizeof(buf), "TOUCH x=%d y=%d", tx, ty);
    strncpy(dbgLine, buf, DBG_LINE_LEN - 1);
    dbgLine[DBG_LINE_LEN - 1] = '\0';
    dbgDirty = true;
    delay(60); // temp debounce
  }
  // >>> TEMP TOUCH TEST END

  // UI refresh
  uint32_t now = millis();
  if (now - lastUiMs > 50) {
    lastUiMs = now;

    if (currentPage != lastPage || editMode != lastEditMode) {
      tft.fillRect(CONTENT_X, CONTENT_Y, CONTENT_W, CONTENT_H, TFT_BLACK);
      drawPageHeader();

      if (currentPage == PAGE_CONFIG) {
        drawConfigPage();
      }

      lastPage = currentPage;
      lastEditMode = editMode;
    }

    drawLogWindow();

    // meters
    if (inSeg != lastInSeg) {
      lastInSeg = inSeg;
      drawVerticalMeter(LEFT_METER_X, IN_METER_Y, inSeg);
    }
    if (outSeg != lastOutSeg) {
      lastOutSeg = outSeg;
      drawVerticalMeter(rightMeterX(), OUT_METER_Y, outSeg);
    }
  }

  delay(2);
}
