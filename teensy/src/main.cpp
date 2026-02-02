#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// ===================== Pins =====================
static const int PIN_STOMP_LEFT = 14;   // Delay ON/OFF (active low)  <-- per your wiring

// ===================== UART to ESP32 =====================
#define ESP_SERIAL Serial4  // confirmed working for you

// ===================== UART to Header =====================
#define MON_SERIAL Serial1   // pins 0 (RX1), 1 (TX1)
static const uint32_t MON_BAUD = 115200;

// ===================== Delay settings (hardcoded) =====================
static const int   DELAY_MS   = 180;
static const float WET_LEVEL  = 0.35f;

// ===================== Audio objects =====================
AudioInputI2S      i2sIn;

AudioEffectDelay   delayL;
AudioEffectDelay   delayR;

AudioMixer4        mixL;
AudioMixer4        mixR;

AudioAmplifier     ampL;
AudioAmplifier     ampR;

// Peaks: input, delay-out, mixer-out, amp-out
AudioAnalyzePeak   peakInL, peakInR;
AudioAnalyzePeak   peakDlyL, peakDlyR;
AudioAnalyzePeak   peakMixL, peakMixR;
AudioAnalyzePeak   peakOutL, peakOutR;

AudioOutputI2S     i2sOut;
AudioControlSGTL5000 sgtl5000;

// ===================== Patch cords =====================
// Dry into mixers
AudioConnection c1(i2sIn, 0, mixL, 0);
AudioConnection c2(i2sIn, 1, mixR, 0);

// Feed delay from input
AudioConnection c3(i2sIn, 0, delayL, 0);
AudioConnection c4(i2sIn, 1, delayR, 0);

// Delay output into mixers (wet)
AudioConnection c5(delayL, 0, mixL, 1);
AudioConnection c6(delayR, 0, mixR, 1);

// Mixer -> output amps -> I2S out
AudioConnection c7(mixL, 0, ampL, 0);
AudioConnection c8(mixR, 0, ampR, 0);
AudioConnection c9(ampL, 0, i2sOut, 0);
AudioConnection c10(ampR, 0, i2sOut, 1);

// Peak taps (use 4-arg form: src, srcIndex, dest, destIndex)
AudioConnection c11(i2sIn, 0, peakInL, 0);
AudioConnection c12(i2sIn, 1, peakInR, 0);

AudioConnection c13(delayL, 0, peakDlyL, 0);
AudioConnection c14(delayR, 0, peakDlyR, 0);

AudioConnection c15(mixL, 0, peakMixL, 0);
AudioConnection c16(mixR, 0, peakMixR, 0);

AudioConnection c17(ampL, 0, peakOutL, 0);
AudioConnection c18(ampR, 0, peakOutR, 0);


// ===================== State =====================
static bool delayEnabled = false;
static int levelPct = 50; // 0..100

static inline float levelToGain(int pct) {
  return constrain(pct, 0, 100) / 100.0f;
}

// ===================== Routing control =====================
static float gDry = 1.0f;
static float gWet = 0.0f;

static void applyDelayState() {
  delayL.delay(0, DELAY_MS);
  delayR.delay(0, DELAY_MS);

  gDry = 1.0f;
  gWet = delayEnabled ? WET_LEVEL : 0.0f;

  // IMPORTANT: mixer gains must be explicitly set
  mixL.gain(0, gDry);
  mixR.gain(0, gDry);
  mixL.gain(1, gWet);
  mixR.gain(1, gWet);

  // Make sure unused channels are off
  mixL.gain(2, 0.0f); mixL.gain(3, 0.0f);
  mixR.gain(2, 0.0f); mixR.gain(3, 0.0f);
}

static void toggleDelay() {
  delayEnabled = !delayEnabled;
  applyDelayState();

  ESP_SERIAL.print("DLY,");
  ESP_SERIAL.print(delayEnabled ? 1 : 0);
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("DLY,");
  MON_SERIAL.print(delayEnabled ? 1 : 0);
  MON_SERIAL.print("\n");

}

static void sendLevelToEsp() {
  ESP_SERIAL.print("LVL,");
  ESP_SERIAL.print(levelPct);
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("LVL,");
  MON_SERIAL.print(levelPct);
  MON_SERIAL.print("\n");

}

static void applyLevel(int pct) {
  int newPct = constrain(pct, 0, 100);
  if (newPct == levelPct) return;
  levelPct = newPct;

  float g = levelToGain(levelPct);
  ampL.gain(g);
  ampR.gain(g);

  sendLevelToEsp();
}

// ===================== UART RX (VOL,<0-100>) =====================
static void pollUart() {
  static char line[64];
  static size_t n = 0;

  while (ESP_SERIAL.available()) {
    char c = (char)ESP_SERIAL.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line[n] = '\0';
      if (n > 0 && strncmp(line, "VOL", 3) == 0) {
        const char* p = line + 3;
        while (*p && (*p == ',' || *p == ':' || *p == ' ')) p++;
        applyLevel(atoi(p));
      }
      n = 0;
    } else {
      if (n < sizeof(line) - 1) line[n++] = c;
      else n = 0;
    }
  }
}

// ===================== Metering to ESP32 =====================
static uint32_t lastMeterMs = 0;
static const uint32_t METER_PERIOD_MS = 50; // 20 Hz
static uint32_t lastDbgMs = 0;
static const uint32_t DBG_PERIOD_MS = 200;  // 5 Hz

static int peakToSegments(float peak) {
  if (peak <= 0.0001f) return 0;
  float db = 20.0f * log10f(peak);
  const float th[8] = {-42, -36, -30, -24, -18, -12, -9, -6};
  int seg = 0;
  for (int i = 0; i < 8; i++) if (db >= th[i]) seg = i + 1;
  return constrain(seg, 0, 8);
}

static void monPrint(const char* s) {
  MON_SERIAL.print(s);
}

static void sendMeters() {
  float inL  = peakInL.available()  ? peakInL.read()  : 0.0f;
  float inR  = peakInR.available()  ? peakInR.read()  : 0.0f;
  float outL = peakOutL.available() ? peakOutL.read() : 0.0f;
  float outR = peakOutR.available() ? peakOutR.read() : 0.0f;

  float inPk  = (inL > inR) ? inL : inR;
  float outPk = (outL > outR) ? outL : outR;

  ESP_SERIAL.print("MTR,");
  ESP_SERIAL.print(peakToSegments(inPk));
  ESP_SERIAL.print(",");
  ESP_SERIAL.print(peakToSegments(outPk));
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("MTR,");
  MON_SERIAL.print(peakToSegments(inPk));
  MON_SERIAL.print(",");
  MON_SERIAL.print(peakToSegments(outPk));
  MON_SERIAL.print("\n");

}

static void sendDbg() {
  // Stage peaks
  float pkiL = peakInL.available()  ? peakInL.read()  : 0.0f;
  float pkiR = peakInR.available()  ? peakInR.read()  : 0.0f;
  float pkdL = peakDlyL.available() ? peakDlyL.read() : 0.0f;
  float pkdR = peakDlyR.available() ? peakDlyR.read() : 0.0f;
  float pkmL = peakMixL.available() ? peakMixL.read() : 0.0f;
  float pkmR = peakMixR.available() ? peakMixR.read() : 0.0f;
  float pkoL = peakOutL.available() ? peakOutL.read() : 0.0f;
  float pkoR = peakOutR.available() ? peakOutR.read() : 0.0f;

  float pki = max(pkiL, pkiR);
  float pkd = max(pkdL, pkdR);
  float pkm = max(pkmL, pkmR);
  float pko = max(pkoL, pkoR);

  ESP_SERIAL.print("DBG,");
  ESP_SERIAL.print("DRY="); ESP_SERIAL.print(gDry, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("WET="); ESP_SERIAL.print(gWet, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("DT=");  ESP_SERIAL.print(DELAY_MS); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKI="); ESP_SERIAL.print(pki, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKD="); ESP_SERIAL.print(pkd, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKM="); ESP_SERIAL.print(pkm, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKO="); ESP_SERIAL.print(pko, 2);
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("DBG,");
  MON_SERIAL.print("DRY="); MON_SERIAL.print(gDry, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("WET="); MON_SERIAL.print(gWet, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("DT=");  MON_SERIAL.print(DELAY_MS); MON_SERIAL.print(",");
  MON_SERIAL.print("PKI="); MON_SERIAL.print(pki, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("PKD="); MON_SERIAL.print(pkd, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("PKM="); MON_SERIAL.print(pkm, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("PKO="); MON_SERIAL.print(pko, 2);
  MON_SERIAL.print("\n");

}

// ===================== Debounce =====================
static bool fallingEdgeDebounced(int pin, uint32_t &lastMs, uint32_t debounceMs = 200) {
  static uint8_t lastState = HIGH;
  uint8_t nowState = digitalRead(pin);
  uint32_t now = millis();

  bool edge = (lastState == HIGH && nowState == LOW && (now - lastMs) > debounceMs);
  if (edge) lastMs = now;
  lastState = nowState;
  return edge;
}

// ===================== Setup / Loop =====================
void setup() {
  pinMode(PIN_STOMP_LEFT, INPUT_PULLUP);
  ESP_SERIAL.begin(115200);
  MON_SERIAL.begin(MON_BAUD);
  MON_SERIAL.print("MON,BOOT\n");

  AudioMemory(80);

  sgtl5000.enable();
  sgtl5000.volume(0.6f);

  // External mic pre -> LINE IN (your proven-working setup)
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(0);
  sgtl5000.lineOutLevel(13);

  ampL.gain(levelToGain(levelPct));
  ampR.gain(levelToGain(levelPct));

  delayEnabled = false;
  applyDelayState();

  sendLevelToEsp();
  ESP_SERIAL.print("DLY,0\n");
}

void loop() {
  pollUart();

  // TEMPORARY: force gains continuously to rule out “gain not sticking”
  // Once fixed, we can remove this.
  mixL.gain(0, gDry);
  mixR.gain(0, gDry);
  mixL.gain(1, gWet);
  mixR.gain(1, gWet);

  static uint32_t lastLeftMs = 0;
  if (fallingEdgeDebounced(PIN_STOMP_LEFT, lastLeftMs)) {
    toggleDelay();
  }

  uint32_t now = millis();

  if (now - lastMeterMs >= METER_PERIOD_MS) {
    lastMeterMs = now;
    sendMeters();
  }
  if (now - lastDbgMs >= DBG_PERIOD_MS) {
    lastDbgMs = now;
    sendDbg();
  }
}
