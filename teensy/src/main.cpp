// VOX EFX - MONO REVERB DROP-IN (Teensy 4.0 + Audio Shield Rev D)
// - Mono path: Line-In Left only
// - Reverb effect (AudioEffectReverb)
// - Footswitch toggles Reverb ON/OFF
// - UART telemetry to ESP32 (Serial4) and header monitor (Serial1)

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// ===================== Pins =====================
static const int PIN_STOMP_LEFT = 14;   // Effect ON/OFF (active low)

// ===================== UARTs =====================
#define ESP_SERIAL Serial4              // to ESP32 (confirmed working for you)
#define MON_SERIAL Serial1              // header monitor pins 0(RX1),1(TX1)
static const uint32_t MON_BAUD = 115200;

// ===================== Effect settings =====================
// Reverb "roomsize" is typically 0.0 .. 1.0 (smaller -> subtle, larger -> bigger tail)
static const float REVERB_ROOMSIZE = 0.55f;

// Mix levels
static const float WET_LEVEL = 0.35f;   // wet mix when enabled

// ===================== Audio objects (MONO) =====================
AudioInputI2S            i2sIn;          // SGTL5000 ADC
AudioEffectFreeverb      reverb;         // mono reverb
AudioMixer4              mix;            // ch0=wet, ch1=dry
AudioAmplifier           amp;            // output level
AudioOutputI2S           i2sOut;         // SGTL5000 DAC
AudioControlSGTL5000     sgtl5000;

// Peaks (tap points)
AudioAnalyzePeak         peakIn;
AudioAnalyzePeak         peakWet;
AudioAnalyzePeak         peakMix;
AudioAnalyzePeak         peakOut;

// ===================== Patch cords (MONO) =====================
// Feed reverb from input (mono left)
AudioConnection          patchCord1(i2sIn, 0, reverb, 0);

// Tap input peak
AudioConnection          patchCord2(i2sIn, 0, peakIn, 0);

// Dry path to mixer channel 1
AudioConnection          patchCord3(i2sIn, 0, mix, 1);

// Wet path to mixer channel 0
AudioConnection          patchCord4(reverb, 0, mix, 0);

// Tap wet peak from reverb output
AudioConnection          patchCord5(reverb, 0, peakWet, 0);

// Mixer -> amp -> out (left only)
AudioConnection          patchCord6(mix, 0, amp, 0);
AudioConnection          patchCord7(mix, 0, peakMix, 0);
AudioConnection          patchCord8(amp, 0, peakOut, 0);
AudioConnection          patchCord9(amp, 0, i2sOut, 0);    // left out only

// ===================== State =====================
static bool effectEnabled = false;
static int  levelPct = 50;     // 0..100

static float gDry = 1.0f;
static float gWet = 0.0f;

static inline float levelToGain(int pct) {
  pct = constrain(pct, 0, 100);
  return (float)pct / 100.0f;
}

// ===================== Helpers =====================
static void monPrint(const char* s) { MON_SERIAL.print(s); }

static int peakToSegments(float peak) {
  if (peak <= 0.0001f) return 0;
  float db = 20.0f * log10f(peak);
  const float th[8] = {-42, -36, -30, -24, -18, -12, -9, -6};
  int seg = 0;
  for (int i = 0; i < 8; i++) if (db >= th[i]) seg = i + 1;
  return constrain(seg, 0, 8);
}

// ===================== Routing control =====================
static void applyEffectState() {
  // Freeverb parameters
  reverb.roomsize(REVERB_ROOMSIZE);   // 0.0 .. 1.0
  reverb.damping(0.5f);               // 0.0 .. 1.0 (higher = darker/less bright)

  gDry = 1.0f;
  gWet = effectEnabled ? WET_LEVEL : 0.0f;

  // Mixer mapping: ch1 = dry, ch0 = wet
  mix.gain(1, gDry);
  mix.gain(0, gWet);
  mix.gain(2, 0.0f);
  mix.gain(3, 0.0f);

  amp.gain(levelToGain(levelPct));
}


static void toggleEffect() {
  effectEnabled = !effectEnabled;
  applyEffectState();

  ESP_SERIAL.print("REV,");
  ESP_SERIAL.print(effectEnabled ? 1 : 0);
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("REV,");
  MON_SERIAL.print(effectEnabled ? 1 : 0);
  MON_SERIAL.print("\n");
}

static void sendLevel() {
  ESP_SERIAL.print("LVL,");
  ESP_SERIAL.print(levelPct);
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("LVL,");
  MON_SERIAL.print(levelPct);
  MON_SERIAL.print("\n");
}

static void applyLevel(int pct) {
  pct = constrain(pct, 0, 100);
  if (pct == levelPct) return;
  levelPct = pct;

  amp.gain(levelToGain(levelPct));
  sendLevel();
}

// ===================== UART RX (VOL,<0-100>) from ESP32 =====================
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

// ===================== Metering =====================
static uint32_t lastMeterMs = 0;
static const uint32_t METER_PERIOD_MS = 50;  // 20 Hz

static uint32_t lastDbgMs = 0;
static const uint32_t DBG_PERIOD_MS = 250;   // 4 Hz

static void sendMeters() {
  float inPk  = peakIn.available()  ? peakIn.read()  : 0.0f;
  float outPk = peakOut.available() ? peakOut.read() : 0.0f;

  int inSeg  = peakToSegments(inPk);
  int outSeg = peakToSegments(outPk);

  ESP_SERIAL.print("MTR,");
  ESP_SERIAL.print(inSeg);
  ESP_SERIAL.print(",");
  ESP_SERIAL.print(outSeg);
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("MTR,");
  MON_SERIAL.print(inSeg);
  MON_SERIAL.print(",");
  MON_SERIAL.print(outSeg);
  MON_SERIAL.print("\n");
}

static void sendDbg() {
  float pki = peakIn.available()  ? peakIn.read()  : 0.0f;
  float pkw = peakWet.available() ? peakWet.read() : 0.0f;
  float pkm = peakMix.available() ? peakMix.read() : 0.0f;
  float pko = peakOut.available() ? peakOut.read() : 0.0f;

  ESP_SERIAL.print("DBG,");
  ESP_SERIAL.print("DRY="); ESP_SERIAL.print(gDry, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("WET="); ESP_SERIAL.print(gWet, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("RV=");  ESP_SERIAL.print(REVERB_ROOMSIZE, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKI="); ESP_SERIAL.print(pki, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKW="); ESP_SERIAL.print(pkw, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKM="); ESP_SERIAL.print(pkm, 2); ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKO="); ESP_SERIAL.print(pko, 2);
  ESP_SERIAL.print("\n");

  MON_SERIAL.print("DBG,");
  MON_SERIAL.print("DRY="); MON_SERIAL.print(gDry, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("WET="); MON_SERIAL.print(gWet, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("RV=");  MON_SERIAL.print(REVERB_ROOMSIZE, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("PKI="); MON_SERIAL.print(pki, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("PKW="); MON_SERIAL.print(pkw, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("PKM="); MON_SERIAL.print(pkm, 2); MON_SERIAL.print(",");
  MON_SERIAL.print("PKO="); MON_SERIAL.print(pko, 2);
  MON_SERIAL.print("\n");
}

// ===================== Setup / Loop =====================
void setup() {
  pinMode(PIN_STOMP_LEFT, INPUT_PULLUP);

  ESP_SERIAL.begin(115200);
  MON_SERIAL.begin(MON_BAUD);
  MON_SERIAL.print("MON,BOOT\n");

  AudioMemory(80);

  // Codec init
  sgtl5000.enable();
  sgtl5000.volume(0.6f);

  // External mic pre -> LINE IN
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(0);
  sgtl5000.lineOutLevel(13);

  // Start with effect OFF (dry only)
  effectEnabled = false;
  applyEffectState();

  sendLevel();
  ESP_SERIAL.print("REV,0\n");
}

void loop() {
  pollUart();

  static uint32_t lastLeftMs = 0;
  if (fallingEdgeDebounced(PIN_STOMP_LEFT, lastLeftMs)) {
    toggleEffect();
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
