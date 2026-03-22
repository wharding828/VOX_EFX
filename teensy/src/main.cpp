// VOX EFX - MONO BASELINE + TRUE-BYPASS EQ + PLACEHOLDER SLOTS (Teensy 4.0 + Audio Shield Rev D)
//
// - Mono path: Line-In Left only
// - Serial4: ESP32 control + telemetry (ESP32 can forward to MQTT)
// - Serial1: header monitor telemetry (your only “human” serial)
//
// FX IMPLEMENTED:
//   EQ: HPF + shelves + 3 peaking bands
//
// PLACEHOLDERS (TRUE-BYPASS SLOTS):
//   COMP, REV, DLY  (enable toggles switch A/B, but "processed" is passthrough for now)
//
// COMMANDS (ESP32 -> Teensy):
//   VOL,<0-100>
//   INGAIN,<0-100>                  (NEW: global input gain placeholder)
//   EQ,EN,<0|1>
//   EQ,HPF,<20-300>
//   EQ,WARM,<dB>   (low shelf @ 160 Hz)
//   EQ,MUD,<dB>    (peak @ 350 Hz)
//   EQ,MID,<dB>    (peak @ 1 kHz)
//   EQ,PRES,<dB>   (peak @ 3 kHz)
//   EQ,AIR,<dB>    (high shelf @ 10 kHz)
//   COMP,EN,<0|1>
//   COMP,THR,<dB>   COMP,RAT,<x10>  COMP,ATK,<ms>  COMP,REL,<ms>  COMP,MAKE,<dB>   (stored only, for now)
//   REV,EN,<0|1>
//   REV,PDLY,<ms>  REV,MIX,<pct>  REV,SIZE,<pct>  REV,DAMP,<pct>  (stored only, for now)
//   DLY,EN,<0|1>
//   DLY,TIME,<ms>  DLY,FB,<pct>                                      (stored only, for now)
//
// TELEMETRY (Teensy -> ESP32 + Serial1):
//   LVL,<pct>
//   INGAIN,<pct>
//   EQ,EN,<0|1>   COMP,EN,<0|1>   REV,EN,<0|1>   DLY,EN,<0|1>
//   MTR,<inSeg>,<outSeg>          (0..8)
//   DBG,... PK* ... plus current params

#include <Arduino.h>
#include <Audio.h>
#include <math.h>
#include <string.h>

// ===================== UARTs =====================
#define ESP_SERIAL Serial4              // to ESP32 (your PCB wiring)
#define MON_SERIAL Serial1              // header monitor pins 0(RX1),1(TX1)
static const uint32_t ESP_BAUD = 115200;
static const uint32_t MON_BAUD = 115200;

// ===================== EQ constants =====================
static constexpr float EQ_WARM_HZ = 160.0f;
static constexpr float EQ_MUD_HZ  = 350.0f;
static constexpr float EQ_MID_HZ  = 1000.0f;
static constexpr float EQ_PRES_HZ = 3000.0f;
static constexpr float EQ_AIR_HZ  = 10000.0f;

static constexpr float EQ_HPF_Q   = 0.707f;
static constexpr float EQ_PEAK_Q  = 1.00f;
static constexpr float EQ_SLOPE   = 1.00f;

// ===================== Audio objects (MONO) =====================
AudioInputI2S            i2sIn;          // SGTL5000 ADC

// EQ stack
AudioFilterBiquad        eqHPF;
AudioFilterBiquad        eqWarm;
AudioFilterBiquad        eqMud;
AudioFilterBiquad        eqMid;
AudioFilterBiquad        eqPres;
AudioFilterBiquad        eqAir;

// True bypass switch: ch0=bypass(dry), ch1=processed
AudioMixer4              eqBypassMix;

// Placeholder slots (true-bypass A/B selector mixers)
AudioMixer4              compBypassMix;  // ch0=bypass, ch1=processed
AudioMixer4              revBypassMix;
AudioMixer4              dlyBypassMix;

AudioAmplifier           amp;            // output level
AudioOutputI2S           i2sOut;          // SGTL5000 DAC
AudioControlSGTL5000     sgtl5000;

// Peaks (tap points)
AudioAnalyzePeak         peakIn;
AudioAnalyzePeak         peakEq;          // EQ-only output (eqAir)
AudioAnalyzePeak         peakEqOut;       // after EQ slot selection
AudioAnalyzePeak         peakCompOut;     // after comp slot selection
AudioAnalyzePeak         peakRevOut;      // after rev slot selection
AudioAnalyzePeak         peakDlyOut;      // after dly slot selection
AudioAnalyzePeak         peakOut;         // post-amp

// ===================== Patch cords =====================
// Input peak
AudioConnection          patchCord1(i2sIn, 0, peakIn, 0);

// --- EQ slot ---
// Dry to EQ bypass mixer ch0
AudioConnection          patchCord2(i2sIn, 0, eqBypassMix, 0);

// EQ chain to EQ bypass mixer ch1
AudioConnection          patchCord3(i2sIn, 0, eqHPF, 0);
AudioConnection          patchCord4(eqHPF, 0, eqWarm, 0);
AudioConnection          patchCord5(eqWarm,0, eqMud,  0);
AudioConnection          patchCord6(eqMud, 0, eqMid,  0);
AudioConnection          patchCord7(eqMid, 0, eqPres, 0);
AudioConnection          patchCord8(eqPres,0, eqAir,  0);
AudioConnection          patchCord9(eqAir, 0, eqBypassMix, 1);

// Tap EQ-only output
AudioConnection          patchCord10(eqAir, 0, peakEq, 0);

// Tap after EQ selection
AudioConnection          patchCord11(eqBypassMix, 0, peakEqOut, 0);

// --- COMP slot (placeholder) ---
AudioConnection          patchCord12(eqBypassMix, 0, compBypassMix, 0); // bypass
AudioConnection          patchCord13(eqBypassMix, 0, compBypassMix, 1); // processed placeholder
AudioConnection          patchCord14(compBypassMix, 0, peakCompOut, 0);

// --- REV slot (placeholder) ---
AudioConnection          patchCord15(compBypassMix, 0, revBypassMix, 0);
AudioConnection          patchCord16(compBypassMix, 0, revBypassMix, 1);
AudioConnection          patchCord17(revBypassMix, 0, peakRevOut, 0);

// --- DLY slot (placeholder) ---
AudioConnection          patchCord18(revBypassMix,  0, dlyBypassMix, 0);
AudioConnection          patchCord19(revBypassMix,  0, dlyBypassMix, 1);
AudioConnection          patchCord20(dlyBypassMix,  0, peakDlyOut, 0);

// --- Output chain ---
AudioConnection          patchCord21(dlyBypassMix, 0, amp, 0);
AudioConnection          patchCord22(amp,          0, peakOut, 0);
AudioConnection          patchCord23(amp,          0, i2sOut, 0);
AudioConnection          patchCord24(amp,          0, i2sOut, 1);

// ===================== State =====================
static int  levelPct   = 50;     // 0..100
static int  inGainPct  = 60;     // 0..100 (placeholder until you insert an input gain stage)

static bool g_eqEnable   = true;
static bool g_compEnable = false;
static bool g_revEnable  = false;
static bool g_dlyEnable  = false;

// EQ params
static int g_hpfHz  = 100;
static int g_warmDb = 0;
static int g_mudDb  = 0;
static int g_midDb  = 0;
static int g_presDb = 0;
static int g_airDb  = 0;

// Stored placeholder params (so you can confirm UI traffic)
static int g_compThrDb    = -18;
static int g_compRatioX10 = 40;   // 4.0:1 default (x10)
static int g_compAtkMs    = 10;
static int g_compRelMs    = 120;
static int g_compMakeDb   = 0;

static int g_dlyTimeMs    = 280;
static int g_dlyFbPct     = 35;

static int g_revPreMs     = 20;
static int g_revMixPct    = 25;
static int g_revSizePct   = 50;
static int g_revDampPct   = 35;

// ===================== Helpers =====================
static inline float pctToLin(int pct) {
  pct = constrain(pct, 0, 100);
  return (float)pct / 100.0f;
}

// RBJ peaking -> Teensy biquad coefficients
static void setPeaking(AudioFilterBiquad& f, uint32_t stage, float freqHz, float q, float gainDb)
{
  int32_t coef[5];

  const double A  = pow(10.0, gainDb / 40.0);
  const double w0 = freqHz * (2.0 * 3.141592653589793 / AUDIO_SAMPLE_RATE_EXACT);
  const double sinW0 = sin(w0);
  const double cosW0 = cos(w0);
  const double alpha = sinW0 / (2.0 * (double)q);

  const double b0 = 1.0 + alpha * A;
  const double b1 = -2.0 * cosW0;
  const double b2 = 1.0 - alpha * A;
  const double a0 = 1.0 + alpha / A;
  const double a1 = -2.0 * cosW0;
  const double a2 = 1.0 - alpha / A;

  const double scale = 1073741824.0 / a0; // 2^30 / a0

  coef[0] = (int32_t)(b0 * scale);
  coef[1] = (int32_t)(b1 * scale);
  coef[2] = (int32_t)(b2 * scale);
  coef[3] = (int32_t)(a1 * scale);
  coef[4] = (int32_t)(a2 * scale);

  f.setCoefficients(stage, coef);
}

static void applySlotBypass(AudioMixer4& m, bool enabled)
{
  m.gain(0, enabled ? 0.0f : 1.0f); // bypass
  m.gain(1, enabled ? 1.0f : 0.0f); // processed
  m.gain(2, 0.0f);
  m.gain(3, 0.0f);
}

static void applyAllBypasses()
{
  applySlotBypass(eqBypassMix,   g_eqEnable);
  applySlotBypass(compBypassMix, g_compEnable);
  applySlotBypass(revBypassMix,  g_revEnable);
  applySlotBypass(dlyBypassMix,  g_dlyEnable);
}

static void applyEqCoeffs()
{
  g_hpfHz  = constrain(g_hpfHz,  20, 300);
  g_warmDb = constrain(g_warmDb, -24, 24);
  g_mudDb  = constrain(g_mudDb,  -24, 24);
  g_midDb  = constrain(g_midDb,  -24, 24);
  g_presDb = constrain(g_presDb, -24, 24);
  g_airDb  = constrain(g_airDb,  -24, 24);

  eqHPF.setHighpass(0, (float)g_hpfHz, EQ_HPF_Q);

  eqWarm.setLowShelf (0, EQ_WARM_HZ, (float)g_warmDb, EQ_SLOPE);
  eqAir .setHighShelf(0, EQ_AIR_HZ,  (float)g_airDb,  EQ_SLOPE);

  setPeaking(eqMud,  0, EQ_MUD_HZ,  EQ_PEAK_Q, (float)g_mudDb);
  setPeaking(eqMid,  0, EQ_MID_HZ,  EQ_PEAK_Q, (float)g_midDb);
  setPeaking(eqPres, 0, EQ_PRES_HZ, EQ_PEAK_Q, (float)g_presDb);

  applyAllBypasses();
}

static void applyLevel(int pct)
{
  pct = constrain(pct, 0, 100);
  if (pct == levelPct) return;
  levelPct = pct;
  amp.gain(pctToLin(levelPct));
}

// Placeholder input gain mapping:
// for now just stores the value and reports it.
// Later: insert an AudioAmplifier preGain between i2sIn and eqBypassMix/eqHPF.
static void applyInGain(int pct)
{
  pct = constrain(pct, 0, 100);
  if (pct == inGainPct) return;
  inGainPct = pct;
}

// ===================== Telemetry (latched peaks) =====================
static float g_pki=0, g_pkq=0, g_pke=0, g_pkc=0, g_pkr=0, g_pkd=0, g_pko=0;

static void pollPeaksOnce()
{
  if (peakIn.available())      g_pki = peakIn.read();
  if (peakEq.available())      g_pkq = peakEq.read();
  if (peakEqOut.available())   g_pke = peakEqOut.read();
  if (peakCompOut.available()) g_pkc = peakCompOut.read();
  if (peakRevOut.available())  g_pkr = peakRevOut.read();
  if (peakDlyOut.available())  g_pkd = peakDlyOut.read();
  if (peakOut.available())     g_pko = peakOut.read();
}

static int peakToSeg16(float peak)
{
  if (peak <= 0.00001f) return 0;

  const float db = 20.0f * log10f(peak);

  // 15 thresholds → 16 segments (0–15)
  static const float th[15] = {
    -48, -45, -42, -39, -36,
    -33, -30, -27, -24, -21,
    -18, -15, -12,  -9,  -6
  };

  int seg = 0;
  for (int i = 0; i < 15; i++)
    if (db >= th[i]) seg = i + 1;

  return constrain(seg, 0, 15);
}


static void sendMeters()
{
  const int inSeg  = peakToSeg16(g_pki);
  const int outSeg = peakToSeg16(g_pko);

  ESP_SERIAL.print("MTR,");
  ESP_SERIAL.print(inSeg);
  ESP_SERIAL.print(",");
  ESP_SERIAL.print(outSeg);
  ESP_SERIAL.print("\n");

  // Optional: also mirror to monitor port
  MON_SERIAL.print("MTR,");
  MON_SERIAL.print(inSeg);
  MON_SERIAL.print(",");
  MON_SERIAL.print(outSeg);
  MON_SERIAL.print("\n");
}

static void sendDbg()
{
  // include placeholders so you can validate UI messages arrive
  ESP_SERIAL.print("DBG,");
  ESP_SERIAL.print("EQ=");   ESP_SERIAL.print(g_eqEnable ? 1 : 0);   ESP_SERIAL.print(",");
  ESP_SERIAL.print("C=");    ESP_SERIAL.print(g_compEnable ? 1 : 0); ESP_SERIAL.print(",");
  ESP_SERIAL.print("R=");    ESP_SERIAL.print(g_revEnable ? 1 : 0);  ESP_SERIAL.print(",");
  ESP_SERIAL.print("D=");    ESP_SERIAL.print(g_dlyEnable ? 1 : 0);  ESP_SERIAL.print(",");
  ESP_SERIAL.print("HPF=");  ESP_SERIAL.print(g_hpfHz);              ESP_SERIAL.print(",");
  ESP_SERIAL.print("ING=");  ESP_SERIAL.print(inGainPct);            ESP_SERIAL.print(",");
  ESP_SERIAL.print("VOL=");  ESP_SERIAL.print(levelPct);             ESP_SERIAL.print(",");

  ESP_SERIAL.print("PKI=");  ESP_SERIAL.print(g_pki, 2);             ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKQ=");  ESP_SERIAL.print(g_pkq, 2);             ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKE=");  ESP_SERIAL.print(g_pke, 2);             ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKC=");  ESP_SERIAL.print(g_pkc, 2);             ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKR=");  ESP_SERIAL.print(g_pkr, 2);             ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKD=");  ESP_SERIAL.print(g_pkd, 2);             ESP_SERIAL.print(",");
  ESP_SERIAL.print("PKO=");  ESP_SERIAL.print(g_pko, 2);

  ESP_SERIAL.print("\n");

  MON_SERIAL.print("DBG,");
  MON_SERIAL.print("EQ=");   MON_SERIAL.print(g_eqEnable ? 1 : 0);   MON_SERIAL.print(",");
  MON_SERIAL.print("C=");    MON_SERIAL.print(g_compEnable ? 1 : 0); MON_SERIAL.print(",");
  MON_SERIAL.print("R=");    MON_SERIAL.print(g_revEnable ? 1 : 0);  MON_SERIAL.print(",");
  MON_SERIAL.print("D=");    MON_SERIAL.print(g_dlyEnable ? 1 : 0);  MON_SERIAL.print(",");
  MON_SERIAL.print("HPF=");  MON_SERIAL.print(g_hpfHz);              MON_SERIAL.print(",");
  MON_SERIAL.print("ING=");  MON_SERIAL.print(inGainPct);            MON_SERIAL.print(",");
  MON_SERIAL.print("VOL=");  MON_SERIAL.print(levelPct);             MON_SERIAL.print(",");

  MON_SERIAL.print("PKI=");  MON_SERIAL.print(g_pki, 2);             MON_SERIAL.print(",");
  MON_SERIAL.print("PKQ=");  MON_SERIAL.print(g_pkq, 2);             MON_SERIAL.print(",");
  MON_SERIAL.print("PKE=");  MON_SERIAL.print(g_pke, 2);             MON_SERIAL.print(",");
  MON_SERIAL.print("PKC=");  MON_SERIAL.print(g_pkc, 2);             MON_SERIAL.print(",");
  MON_SERIAL.print("PKR=");  MON_SERIAL.print(g_pkr, 2);             MON_SERIAL.print(",");
  MON_SERIAL.print("PKD=");  MON_SERIAL.print(g_pkd, 2);             MON_SERIAL.print(",");
  MON_SERIAL.print("PKO=");  MON_SERIAL.print(g_pko, 2);

  MON_SERIAL.print("\n");
}

static void sendStateOnce()
{
  ESP_SERIAL.print("LVL,");    ESP_SERIAL.print(levelPct);  ESP_SERIAL.print("\n");
  ESP_SERIAL.print("INGAIN,"); ESP_SERIAL.print(inGainPct); ESP_SERIAL.print("\n");

  ESP_SERIAL.print("EQ,EN,");   ESP_SERIAL.print(g_eqEnable ? 1 : 0);   ESP_SERIAL.print("\n");
  ESP_SERIAL.print("COMP,EN,"); ESP_SERIAL.print(g_compEnable ? 1 : 0); ESP_SERIAL.print("\n");
  ESP_SERIAL.print("REV,EN,");  ESP_SERIAL.print(g_revEnable ? 1 : 0);  ESP_SERIAL.print("\n");
  ESP_SERIAL.print("DLY,EN,");  ESP_SERIAL.print(g_dlyEnable ? 1 : 0);  ESP_SERIAL.print("\n");

  MON_SERIAL.print("LVL,");    MON_SERIAL.print(levelPct);  MON_SERIAL.print("\n");
  MON_SERIAL.print("INGAIN,"); MON_SERIAL.print(inGainPct); MON_SERIAL.print("\n");

  MON_SERIAL.print("EQ,EN,");   MON_SERIAL.print(g_eqEnable ? 1 : 0);   MON_SERIAL.print("\n");
  MON_SERIAL.print("COMP,EN,"); MON_SERIAL.print(g_compEnable ? 1 : 0); MON_SERIAL.print("\n");
  MON_SERIAL.print("REV,EN,");  MON_SERIAL.print(g_revEnable ? 1 : 0);  MON_SERIAL.print("\n");
  MON_SERIAL.print("DLY,EN,");  MON_SERIAL.print(g_dlyEnable ? 1 : 0);  MON_SERIAL.print("\n");
}

// ===================== Parsing helpers =====================
static void trimInPlace(char* s)
{
  if (!s) return;
  // leading
  while (*s == ' ' || *s == '\t') s++;

  // trailing
  char* end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t')) end--;
  *end = 0;
}

static void handleEqCmd(const char* key, int val)
{
  if      (!strcmp(key, "EN"))   g_eqEnable = (val != 0);
  else if (!strcmp(key, "HPF"))  g_hpfHz    = val;
  else if (!strcmp(key, "WARM")) g_warmDb   = val;
  else if (!strcmp(key, "MUD"))  g_mudDb    = val;
  else if (!strcmp(key, "MID"))  g_midDb    = val;
  else if (!strcmp(key, "PRES")) g_presDb   = val;
  else if (!strcmp(key, "AIR"))  g_airDb    = val;
  else return;

  applyEqCoeffs();
}

static void handleCompParam(const char* key, int val)
{
  if      (!strcmp(key, "EN"))   g_compEnable  = (val != 0);
  else if (!strcmp(key, "THR"))  g_compThrDb   = val;
  else if (!strcmp(key, "RAT"))  g_compRatioX10= val;
  else if (!strcmp(key, "ATK"))  g_compAtkMs   = val;
  else if (!strcmp(key, "REL"))  g_compRelMs   = val;
  else if (!strcmp(key, "MAKE")) g_compMakeDb  = val;
  else return;

  applyAllBypasses();
}

static void handleRevParam(const char* key, int val)
{
  if      (!strcmp(key, "EN"))   g_revEnable = (val != 0);
  else if (!strcmp(key, "PDLY")) g_revPreMs  = val;
  else if (!strcmp(key, "MIX"))  g_revMixPct = val;
  else if (!strcmp(key, "SIZE")) g_revSizePct= val;
  else if (!strcmp(key, "DAMP")) g_revDampPct= val;
  else return;

  applyAllBypasses();
}

static void handleDlyParam(const char* key, int val)
{
  if      (!strcmp(key, "EN"))   g_dlyEnable = (val != 0);
  else if (!strcmp(key, "TIME")) g_dlyTimeMs = val;
  else if (!strcmp(key, "FB"))   g_dlyFbPct  = val;
  else return;

  applyAllBypasses();
}

static void parseLine(char* s)
{
  if (!s || !*s) return;
  trimInPlace(s);
  if (!*s) return;

  // Fast-path: VOL,<pct>
  if (!strncmp(s, "VOL", 3)) {
    const char* p = s + 3;
    while (*p && (*p == ',' || *p == ':' || *p == ' ')) p++;
    applyLevel(atoi(p));

    ESP_SERIAL.print("LVL,"); ESP_SERIAL.print(levelPct); ESP_SERIAL.print("\n");
    MON_SERIAL.print("LVL,"); MON_SERIAL.print(levelPct); MON_SERIAL.print("\n");
    return;
  }

  // Fast-path: INGAIN,<pct>
  if (!strncmp(s, "INGAIN", 6)) {
    const char* p = s + 6;
    while (*p && (*p == ',' || *p == ':' || *p == ' ')) p++;
    applyInGain(atoi(p));

    ESP_SERIAL.print("INGAIN,"); ESP_SERIAL.print(inGainPct); ESP_SERIAL.print("\n");
    MON_SERIAL.print("INGAIN,"); MON_SERIAL.print(inGainPct); MON_SERIAL.print("\n");
    return;
  }

  // Token form: WHICH,KEY,VAL  (EQ,HPF,120)  (COMP,EN,1) (DLY,TIME,300) ...
  char* which = strtok(s, ",");
  char* key   = strtok(nullptr, ",");
  char* val   = strtok(nullptr, ",");

  if (!which || !key || !val) return;

  int ival = atoi(val);

  if (!strcmp(which, "EQ"))   { handleEqCmd(key, ival); return; }
  if (!strcmp(which, "COMP")) { handleCompParam(key, ival); return; }
  if (!strcmp(which, "REV"))  { handleRevParam(key, ival); return; }
  if (!strcmp(which, "DLY"))  { handleDlyParam(key, ival); return; }

  if (!strcmp(s, "PING")) {
  ESP_SERIAL.print("PONG\n");
  MON_SERIAL.print("PONG\n");
  return;
}

}

static void pollUart()
{
  static char line[96];
  static size_t n = 0;

  while (ESP_SERIAL.available()) {
    const char c = (char)ESP_SERIAL.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line[n] = '\0';
      if (n > 0) parseLine(line);
      n = 0;
    } else {
      if (n < sizeof(line) - 1) line[n++] = c;
      else n = 0; // overflow -> reset line
    }
  }
}

// ===================== Setup / Loop =====================
static uint32_t lastMeterMs = 0;
static constexpr uint32_t METER_PERIOD_MS = 50;

static uint32_t lastDbgMs = 0;
static constexpr uint32_t DBG_PERIOD_MS = 250;

void setup()
{
  ESP_SERIAL.begin(ESP_BAUD);
  MON_SERIAL.begin(MON_BAUD);
  MON_SERIAL.print("MON,BOOT\n");

  AudioMemory(80);

  sgtl5000.enable();
  sgtl5000.volume(0.6f);

  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(0);
  sgtl5000.lineOutLevel(13);

  amp.gain(pctToLin(levelPct));

  // IMPORTANT: mixers default all gains to 0 -> no audio unless we set them
  applyAllBypasses();
  applyEqCoeffs();

  sendStateOnce();
}

void loop()
{
  pollUart();
  pollPeaksOnce();

  const uint32_t now = millis();

  if (now - lastMeterMs >= METER_PERIOD_MS) {
    lastMeterMs = now;
    sendMeters();
  }

  if (now - lastDbgMs >= DBG_PERIOD_MS) {
    lastDbgMs = now;
    sendDbg();
  }
}