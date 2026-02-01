//R1 - Teensy 4.0 + Audio Shield (SGTL5000) - LINE IN Meter
//  - Reads LINE IN via I2S
//  - Prints Peak + RMS + CLIP indication
//  - Use this to set analog gain (Rg) and SGTL5000 lineInLevel safely

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// ---------- Audio graph ----------
AudioInputI2S        i2sIn;
AudioAnalyzePeak     peakL;
AudioAnalyzePeak     peakR;
AudioAnalyzeRMS      rmsL;
AudioAnalyzeRMS      rmsR;

AudioConnection      patchCord1(i2sIn, 0, peakL, 0);
AudioConnection      patchCord2(i2sIn, 1, peakR, 0);
AudioConnection      patchCord3(i2sIn, 0, rmsL, 0);
AudioConnection      patchCord4(i2sIn, 1, rmsR, 0);

AudioControlSGTL5000 sgtl5000;

// ---------- Settings ----------
static const uint32_t PRINT_MS = 200;
static const float CLIP_THRESH = 0.98f; // near full-scale

void setup() {
  Serial.begin(115200);
  delay(300);

  AudioMemory(24);

  sgtl5000.enable();

  // IMPORTANT: Use LINE IN (your THAT1246 output should feed line-in)
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);

  // LINE IN level: 0..15 (0 = hottest, 15 = most attenuation)
  // Start conservative (more attenuation), then work toward louder if needed.
  sgtl5000.lineInLevel(10);

  // Output disabled / low – we’re only metering input right now
  sgtl5000.volume(0.0f);

  Serial.println("SGTL5000 LINE IN meter running");
  Serial.println("Send audio into your front end. Watch peak/rms. Avoid clipping.");
  Serial.println("Tip: If peak hits ~1.00, reduce input level or increase lineInLevel().");
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint < PRINT_MS) return;
  lastPrint = now;

  float pL = peakL.available() ? peakL.read() : 0.0f;
  float pR = peakR.available() ? peakR.read() : 0.0f;
  float rL = rmsL.available()  ? rmsL.read()  : 0.0f;
  float rR = rmsR.available()  ? rmsR.read()  : 0.0f;

  bool clip = (pL > CLIP_THRESH) || (pR > CLIP_THRESH);

  Serial.print("Peak L/R: ");
  Serial.print(pL, 3);
  Serial.print(" / ");
  Serial.print(pR, 3);

  Serial.print("   RMS L/R: ");
  Serial.print(rL, 3);
  Serial.print(" / ");
  Serial.print(rR, 3);

  Serial.print("   ");
  Serial.println(clip ? "CLIP!" : "OK");
}
