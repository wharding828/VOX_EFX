#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// ===================== UART =====================
#define ESP_SERIAL Serial4   // confirmed working for you

// ===================== Audio objects =====================
AudioInputI2S      i2sIn;
AudioAmplifier     ampL;
AudioAmplifier     ampR;
AudioAnalyzePeak   peakInL;
AudioAnalyzePeak   peakInR;
AudioAnalyzePeak   peakOutL;
AudioAnalyzePeak   peakOutR;
AudioOutputI2S     i2sOut;

// ===================== Audio connections =====================
AudioConnection c1(i2sIn, 0, ampL, 0);
AudioConnection c2(i2sIn, 1, ampR, 0);

AudioConnection c3(ampL, 0, i2sOut, 0);
AudioConnection c4(ampR, 0, i2sOut, 1);

// meters
AudioConnection c5(i2sIn, 0, peakInL, 0);
AudioConnection c6(i2sIn, 1, peakInR, 0);
AudioConnection c7(ampL, 0, peakOutL, 0);
AudioConnection c8(ampR, 0, peakOutR, 0);

// ===================== Codec =====================
AudioControlSGTL5000 sgtl5000;

// ===================== Level control =====================
static int levelPct = 50;   // 0..100

float levelToGain(int pct) {
  return constrain(pct, 0, 100) / 100.0f;
}

void applyLevel(int pct) {
  levelPct = constrain(pct, 0, 100);
  float g = levelToGain(levelPct);
  ampL.gain(g);
  ampR.gain(g);
}

// ===================== UART RX =====================
void pollUart() {
  static char line[32];
  static size_t n = 0;

  while (ESP_SERIAL.available()) {
    char c = ESP_SERIAL.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line[n] = '\0';
      if (strncmp(line, "VOL", 3) == 0) {
        const char* p = line + 3;
        while (*p && (*p == ',' || *p == ':' || *p == ' ')) p++;
        int v = atoi(p);
        applyLevel(v);
      }
      n = 0;
    } else if (n < sizeof(line) - 1) {
      line[n++] = c;
    } else {
      n = 0;
    }
  }
}

// ===================== Metering =====================
static uint32_t lastMeterMs = 0;
static const uint32_t METER_PERIOD_MS = 50; // 20 Hz

int peakToSegments(float peak) {
  if (peak <= 0.0001f) return 0;
  float db = 20.0f * log10f(peak);

  const float th[8] = {
    -42, -36, -30, -24, -18,  // green
    -12, -9,                 // amber
    -6                       // red
  };

  int seg = 0;
  for (int i = 0; i < 8; i++) {
    if (db >= th[i]) seg = i + 1;
  }
  return constrain(seg, 0, 8);
}

void sendMeters() {
  float inL  = peakInL.available()  ? peakInL.read()  : 0.0f;
  float inR  = peakInR.available()  ? peakInR.read()  : 0.0f;
  float outL = peakOutL.available() ? peakOutL.read() : 0.0f;
  float outR = peakOutR.available() ? peakOutR.read() : 0.0f;

  float inPk  = max(inL, inR);
  float outPk = max(outL, outR);

  ESP_SERIAL.print("MTR,");
  ESP_SERIAL.print(peakToSegments(inPk));
  ESP_SERIAL.print(",");
  ESP_SERIAL.print(peakToSegments(outPk));
  ESP_SERIAL.print("\n");
}

// ===================== Setup =====================
void setup() {
  ESP_SERIAL.begin(115200);

  AudioMemory(60);

  sgtl5000.enable();
  sgtl5000.volume(0.6f);

  // External mic pre → LINE IN (correct for your hardware)
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(0);      // highest line-in sensitivity
  sgtl5000.lineOutLevel(13);    // sane default

  applyLevel(levelPct);

  // Report initial level to ESP
  ESP_SERIAL.print("LVL,");
  ESP_SERIAL.print(levelPct);
  ESP_SERIAL.print("\n");
}

// ===================== Loop =====================
void loop() {
  pollUart();

  uint32_t now = millis();
  if (now - lastMeterMs >= METER_PERIOD_MS) {
    lastMeterMs = now;
    sendMeters();
  }
}
