// ============================================================
// VOX_EFX - ESP32 UI (SquareLine + LVGL 9) + WiFi Settings + Teensy UART0 Control
//
// - SquareLine objects are declared in ui.h / ui_main.h (pasted by you)
// - LVGL draw buffer allocated dynamically (no PSRAM required)
// - UART0 (Serial) is HARD-WIRED to Teensy on your PCB -> DO NOT print debug to Serial
// - WiFi UI uses dropdown scan + connect + NVS saved credentials
// - Effects panels: EQ, COMP, DLY, REV (On/Bypass + sliders -> Teensy commands)
// - Global Settings: Input Gain + Output Vol (to Teensy)
//
// NOTES:
// 1) Your Teensy currently implements: VOL, EQ params, and EN toggles for COMP/REV/DLY.
//    This ESP32 code ALSO sends COMP/DLY/REV parameter commands (THR/RATIO/...), which
//    Teensy will ignore until you add handlers. That’s intentional to “wire up” the UI now.
// 2) LVGL logging: you said you'll do MQTT logging later. This file keeps silent on Serial.
// ============================================================

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include "esp_heap_caps.h"
#include <PubSubClient.h>

extern "C" {
  #include <lvgl.h>
  #include "ui.h"
}

// ====================== Forward declarations ======================
// MQTT helpers used by UI callbacks
static String t_ui();
static void mqttPublish(const String& topic, const String& payload);
static void mqttPublishRetained(const String& topic, const String& payload);

// WiFi state used by MQTT
bool g_wifiConnected = false;

// WiFi dropdown callback used by dropdownSelectSsidFast()
static void wifiSsidChangedCb(lv_event_t* e);

static String t_uart_tx();
static String t_uart_rx();
static String t_uart_info();

// ====================== Display settings ======================
static constexpr int DISP_HOR = 480;
static constexpr int DISP_VER = 320;
static constexpr int TFT_ROT  = 1;

static constexpr uint32_t BUF_LINES_DEFAULT = 12; // safe-ish on non-PSRAM; allocator will halve if needed
static lv_color_t*  g_buf1      = nullptr;
static uint32_t     g_bufPixels = 0;

static lv_display_t* g_disp = nullptr;
static TFT_eSPI      tft;

// ====================== Pins ======================
#define PIN_LED 2

// ====================== Optional Touch (FT6336U) ======================
#define ENABLE_FT6336U_TOUCH  1
#if ENABLE_FT6336U_TOUCH
static const uint8_t FT_ADDR     = 0x38;
static const uint8_t REG_TD_STAT = 0x02;
static const uint8_t REG_P1_XH   = 0x03;

static bool ftReadRegs(uint8_t startReg, uint8_t* buf, uint8_t len)
{
  Wire.beginTransmission(FT_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  int n = Wire.requestFrom((int)FT_ADDR, (int)len);
  if (n != len) return false;

  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool ftReadTouch(int& sx, int& sy, bool& pressed)
{
  pressed = false;

  uint8_t td = 0;
  if (!ftReadRegs(REG_TD_STAT, &td, 1)) return false;
  if ((td & 0x0F) == 0) return true; // released

  uint8_t b[4];
  if (!ftReadRegs(REG_P1_XH, b, 4)) return false;

  uint16_t x = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
  uint16_t y = ((uint16_t)(b[2] & 0x0F) << 8) | b[3];

  if (x >= 4095 || y >= 4095) return true;

  int rawX = (int)x;
  int rawY = (int)y;

  // Mapping for rotation(1) on your build.
  sx = constrain(rawY, 0, DISP_HOR - 1);
  sy = constrain((DISP_VER - 1) - rawX, 0, DISP_VER - 1);

  pressed = true;
  return true;
}

static void my_touch_read_cb(lv_indev_t* indev, lv_indev_data_t* data)
{
  (void)indev;

  static int lastX = 0;
  static int lastY = 0;

  int x = 0, y = 0;
  bool down = false;

  bool ok = ftReadTouch(x, y, down);
  if (!ok) {
    data->state = LV_INDEV_STATE_RELEASED;
    data->point.x = lastX;
    data->point.y = lastY;
    return;
  }

  if (down) {
    lastX = x;
    lastY = y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }

  data->point.x = lastX;
  data->point.y = lastY;
}
#endif

// ====================== Optional Encoder Nav ======================
#define ENABLE_ENCODER_NAV  1
#if ENABLE_ENCODER_NAV
static const int PIN_ENC_A   = 35;
static const int PIN_ENC_B   = 34;
static const int PIN_ENC_BTN = 32;

static int g_lastAB = 0;

static int read_encoder_step()
{
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  int ab = (a << 1) | b;

  static const int8_t tbl[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
  };

  int idx = ((g_lastAB & 0x3) << 2) | (ab & 0x3);
  g_lastAB = ab;
  return tbl[idx];
}

static bool read_button_pressed_edge()
{
  static bool last = true; // pullup idle HIGH
  bool now = digitalRead(PIN_ENC_BTN);
  bool pressed = (last == true && now == false);
  last = now;
  return pressed;
}
#endif

// ====================== LVGL flush callback ======================
static void my_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
  (void)disp;
  const int32_t w = (area->x2 - area->x1 + 1);
  const int32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow((int)area->x1, (int)area->y1, (int)w, (int)h);
  tft.pushPixels((uint16_t*)px_map, (uint32_t)(w * h));
  tft.endWrite();

  lv_display_flush_ready(disp);
}

// ====================== LVGL buffer allocation ======================
static bool allocLvglBuffer(uint32_t linesWanted)
{
  if (g_buf1) return true;

  g_bufPixels = (uint32_t)DISP_HOR * linesWanted;
  size_t bytes = g_bufPixels * sizeof(lv_color_t);

  // Prefer internal DRAM
  g_buf1 = (lv_color_t*)heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!g_buf1) g_buf1 = (lv_color_t*)heap_caps_malloc(bytes, MALLOC_CAP_8BIT);

  return (g_buf1 != nullptr);
}

// ====================== Small UI helpers ======================
static void uiSetValueLabel(lv_obj_t* lbl, const char* fmt, int v)
{
  if (!lbl) return;
  char b[32];
  snprintf(b, sizeof(b), fmt, v);
  lv_label_set_text(lbl, b);
}

static void makeClickable(lv_obj_t* o, lv_event_cb_t cb, void* user = nullptr)
{
  if (!o) return;
  lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_flag(o, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_add_event_cb(o, cb, LV_EVENT_CLICKED, user);
}

// ====================== Pages ======================
enum class Page : uint8_t { Home, Delay, Reverb, Comp, EQ, Settings };
static Page g_page = Page::Home;

static constexpr Page PAGE_ORDER[] = {
  Page::Home, Page::Delay, Page::Reverb, Page::Comp, Page::EQ, Page::Settings
};
static constexpr int PAGE_ORDER_COUNT = (int)(sizeof(PAGE_ORDER) / sizeof(PAGE_ORDER[0]));

static int pageIndex(Page p)
{
  for (int i = 0; i < PAGE_ORDER_COUNT; i++) if (PAGE_ORDER[i] == p) return i;
  return 0;
}

static void hideAllPages()
{
  if (ui_contNavigation) lv_obj_add_flag(ui_contNavigation, LV_OBJ_FLAG_HIDDEN);

  if (ui_pnlComp)     lv_obj_add_flag(ui_pnlComp, LV_OBJ_FLAG_HIDDEN);
  if (ui_pnlDelay)    lv_obj_add_flag(ui_pnlDelay, LV_OBJ_FLAG_HIDDEN);
  if (ui_pnlReverb)   lv_obj_add_flag(ui_pnlReverb, LV_OBJ_FLAG_HIDDEN);
  if (ui_pnlEQ)       lv_obj_add_flag(ui_pnlEQ, LV_OBJ_FLAG_HIDDEN);
  if (ui_pnlSettings) lv_obj_add_flag(ui_pnlSettings, LV_OBJ_FLAG_HIDDEN);
}

static void showPage(Page p)
{
  g_page = p;
  hideAllPages();

  // Leaving Settings -> hide keyboard
  if (p != Page::Settings && ui_pwKB) {
    lv_obj_add_flag(ui_pwKB, LV_OBJ_FLAG_HIDDEN);
    if (ui_taWifiPass) lv_obj_clear_state(ui_taWifiPass, LV_STATE_FOCUSED);
  }

  switch (p) {
    case Page::Home:     if (ui_contNavigation) lv_obj_remove_flag(ui_contNavigation, LV_OBJ_FLAG_HIDDEN); break;
    case Page::Delay:    if (ui_pnlDelay)       lv_obj_remove_flag(ui_pnlDelay,       LV_OBJ_FLAG_HIDDEN); break;
    case Page::Reverb:   if (ui_pnlReverb)      lv_obj_remove_flag(ui_pnlReverb,      LV_OBJ_FLAG_HIDDEN); break;
    case Page::Comp:     if (ui_pnlComp)        lv_obj_remove_flag(ui_pnlComp,        LV_OBJ_FLAG_HIDDEN); break;
    case Page::EQ:       if (ui_pnlEQ)          lv_obj_remove_flag(ui_pnlEQ,          LV_OBJ_FLAG_HIDDEN); break;
    case Page::Settings: if (ui_pnlSettings)    lv_obj_remove_flag(ui_pnlSettings,    LV_OBJ_FLAG_HIDDEN); break;
  }
}

static void showNextPrev(int dir)
{
  int idx = pageIndex(g_page) + dir;
  if (idx < 0) idx = PAGE_ORDER_COUNT - 1;
  if (idx >= PAGE_ORDER_COUNT) idx = 0;
  showPage(PAGE_ORDER[idx]);
}

// ====================== Navigation callbacks ======================
static void navToPageCb(lv_event_t* e)
{
  Page p = (Page)(uintptr_t)lv_event_get_user_data(e);
  showPage(p);
}

static void backToHomeCb(lv_event_t* e)
{
  (void)e;
  showPage(Page::Home);
}

// ====================== Teensy link on UART0 ======================
// UART0 is the Teensy link. Keep it clean.
static inline void teensySendLine(const char* line)
{
  Serial.print(line);
  Serial.print('\n');

  // MQTT “wire tap”
  if (line && *line) mqttPublish(t_uart_tx(), String(line));
}

static uint32_t g_lastSendMs = 0;
static constexpr uint32_t SEND_MIN_MS = 60;

static inline bool sendRateOk()
{
  uint32_t now = millis();
  if (now - g_lastSendMs < SEND_MIN_MS) return false;
  g_lastSendMs = now;
  return true;
}

static inline void teensySend2(const char* a, int v)
{
  char b[48];
  snprintf(b, sizeof(b), "%s,%d", a, v);
  teensySendLine(b);
}

static inline void teensySend3(const char* a, const char* k, int v)
{
  char b[64];
  snprintf(b, sizeof(b), "%s,%s,%d", a, k, v);
  teensySendLine(b);
}

// ====================== Local mirrors + defaults ======================
// Global
static int  g_inGainPct   = 60; // 0..100 (you’ll map this on Teensy)
static int  g_outVolPct   = 50; // 0..100 -> VOL,<pct>

// EQ
static bool g_eqEnabled   = true;
static int  g_eqHpfHz     = 100; // 20..300
static int  g_eqWarmDb    = 0;   // -24..24
static int  g_eqMudDb     = 0;
static int  g_eqMidDb     = 0;
static int  g_eqPresDb    = 0;
static int  g_eqAirDb     = 0;

// COMP
static bool g_compEnabled = false;
static int  g_compThrDb   = -18; // -60..0
static int  g_compRatioX10= 40;  // 10..200 (1.0..20.0) stored as x10
static int  g_compAtkMs   = 10;  // 1..100
static int  g_compRelMs   = 120; // 20..600
static int  g_compMakeDb  = 0;   // 0..24

// DLY
static bool g_dlyEnabled  = false;
static int  g_dlyTimeMs   = 280; // 0..1200
static int  g_dlyFbPct    = 35;  // 0..95

// REV
static bool g_revEnabled  = false;
static int  g_revPreMs    = 20;  // 0..200
static int  g_revMixPct   = 25;  // 0..100
static int  g_revSizePct  = 50;  // 0..100
static int  g_revDampPct  = 35;  // 0..100

// ====================== Effect callbacks ======================
// NOTE:
// - Braces added so MQTT publish follows the same sendRateOk() throttling.
// - Standardized MQTT payloads to: "<BLOCK>,<KEY>=<VAL>" (easy to parse in Node-RED)
//   Examples: "VOL=50", "EQ,HPF=100", "COMP,EN=1", "DLY,TIME=280", "REV,MIX=25"

// Global
static void globalOutVolCb(lv_event_t* e)
{
  lv_obj_t* s = (lv_obj_t*)lv_event_get_target(e);
  int v = (int)lv_slider_get_value(s);
  v = constrain(v, 0, 100);
  if (v == g_outVolPct) return;
  g_outVolPct = v;

  uiSetValueLabel(ui_lblGlobalOutputVolValue, "%d %%", g_outVolPct);

  if (sendRateOk()) {
    teensySend2("VOL", g_outVolPct);
    mqttPublish(t_ui(), String("VOL=") + String(g_outVolPct));
  }
}

static void globalInGainCb(lv_event_t* e)
{
  lv_obj_t* s = (lv_obj_t*)lv_event_get_target(e);
  int v = (int)lv_slider_get_value(s);
  v = constrain(v, 0, 100);
  if (v == g_inGainPct) return;
  g_inGainPct = v;

  uiSetValueLabel(ui_lblGlobalInputGainValue, "%d %%", g_inGainPct);

  if (sendRateOk()) {
    teensySend2("INGAIN", g_inGainPct);
    mqttPublish(t_ui(), String("INGAIN=") + String(g_inGainPct));
  }
}

// EQ
static void eqBypassChangedCb(lv_event_t* e)
{
  (void)e;
  bool swOn = ui_EqOnBypass && lv_obj_has_state(ui_EqOnBypass, LV_STATE_CHECKED);
  if (swOn == g_eqEnabled) return;
  g_eqEnabled = swOn;

  if (sendRateOk()) {
    const int en = g_eqEnabled ? 1 : 0;
    teensySend3("EQ", "EN", en);
    mqttPublish(t_ui(), String("EQ,EN=") + String(en));
  }
}

static void eqSliderCommonCb(lv_event_t* e, const char* key, int& target, lv_obj_t* valLbl, const char* fmt)
{
  lv_obj_t* s = (lv_obj_t*)lv_event_get_target(e);
  int v = (int)lv_slider_get_value(s);
  if (v == target) return;
  target = v;

  uiSetValueLabel(valLbl, fmt, v);

  if (sendRateOk()) {
    teensySend3("EQ", key, target);
    mqttPublish(t_ui(), String("EQ,") + key + "=" + String(target));
  }
}

static void eqHpfCb (lv_event_t* e) { eqSliderCommonCb(e, "HPF",  g_eqHpfHz,  ui_lblEqHPFValue,  "%d Hz"); }
static void eqWarmCb(lv_event_t* e) { eqSliderCommonCb(e, "WARM", g_eqWarmDb, ui_lblEqWarmValue, "%+d dB"); }
static void eqMudCb (lv_event_t* e) { eqSliderCommonCb(e, "MUD",  g_eqMudDb,  ui_lblEqMudValue,  "%+d dB"); }
static void eqMidCb (lv_event_t* e) { eqSliderCommonCb(e, "MID",  g_eqMidDb,  ui_lblEqMidValue,  "%+d dB"); }
static void eqPresCb(lv_event_t* e) { eqSliderCommonCb(e, "PRES", g_eqPresDb, ui_lblEqPresValue, "%+d dB"); }
static void eqAirCb (lv_event_t* e) { eqSliderCommonCb(e, "AIR",  g_eqAirDb,  ui_lblEqAirValue,  "%+d dB"); }

// COMP
static void compBypassChangedCb(lv_event_t* e)
{
  (void)e;
  bool swOn = ui_CompOnBypass && lv_obj_has_state(ui_CompOnBypass, LV_STATE_CHECKED);
  if (swOn == g_compEnabled) return;
  g_compEnabled = swOn;

  if (sendRateOk()) {
    const int en = g_compEnabled ? 1 : 0;
    teensySend3("COMP", "EN", en);
    mqttPublish(t_ui(), String("COMP,EN=") + String(en));
  }
}

static void compSliderCommonCb(lv_event_t* e, const char* key, int& target, lv_obj_t* valLbl, const char* fmt)
{
  lv_obj_t* s = (lv_obj_t*)lv_event_get_target(e);
  int v = (int)lv_slider_get_value(s);
  if (v == target) return;
  target = v;

  uiSetValueLabel(valLbl, fmt, v);

  if (sendRateOk()) {
    teensySend3("COMP", key, target);
    mqttPublish(t_ui(), String("COMP,") + key + "=" + String(target));
  }
}

static void compThrCb   (lv_event_t* e) { compSliderCommonCb(e, "THR",   g_compThrDb,     ui_lblCompThresholdValue, "%d dB"); }

static void compRatioCb (lv_event_t* e)
{
  lv_obj_t* s = (lv_obj_t*)lv_event_get_target(e);
  int v = (int)lv_slider_get_value(s); // x10
  if (v == g_compRatioX10) return;
  g_compRatioX10 = v;

  // UI display "x.y:1"
  if (ui_lblCompSlider2Value) {
    char b[24];
    snprintf(b, sizeof(b), "%d.%d:1", g_compRatioX10 / 10, g_compRatioX10 % 10);
    lv_label_set_text(ui_lblCompSlider2Value, b);
  }

  if (sendRateOk()) {
    teensySend3("COMP", "RAT", g_compRatioX10);
    mqttPublish(t_ui(), String("COMP,RAT=") + String(g_compRatioX10));
  }
}

static void compAtkCb   (lv_event_t* e) { compSliderCommonCb(e, "ATK",   g_compAtkMs,     ui_lblCompAttackValue,   "%d ms"); }
static void compRelCb   (lv_event_t* e) { compSliderCommonCb(e, "REL",   g_compRelMs,     ui_lblCompReleaseValue,  "%d ms"); }
static void compMakeCb  (lv_event_t* e) { compSliderCommonCb(e, "MAKE",  g_compMakeDb,    ui_lblCompGainValue,     "%d dB"); }

// DLY
static void dlyBypassChangedCb(lv_event_t* e)
{
  (void)e;
  bool swOn = ui_DelayOnBypass && lv_obj_has_state(ui_DelayOnBypass, LV_STATE_CHECKED);
  if (swOn == g_dlyEnabled) return;
  g_dlyEnabled = swOn;

  if (sendRateOk()) {
    const int en = g_dlyEnabled ? 1 : 0;
    teensySend3("DLY", "EN", en);
    mqttPublish(t_ui(), String("DLY,EN=") + String(en));
  }
}

static void dlySliderCommonCb(lv_event_t* e, const char* key, int& target, lv_obj_t* valLbl, const char* fmt)
{
  lv_obj_t* s = (lv_obj_t*)lv_event_get_target(e);
  int v = (int)lv_slider_get_value(s);
  if (v == target) return;
  target = v;

  uiSetValueLabel(valLbl, fmt, v);

  if (sendRateOk()) {
    teensySend3("DLY", key, target);
    mqttPublish(t_ui(), String("DLY,") + key + "=" + String(target));
  }
}

static void dlyTimeCb(lv_event_t* e) { dlySliderCommonCb(e, "TIME", g_dlyTimeMs, ui_lblDelayTimeValue,     "%d ms"); }
static void dlyFbCb  (lv_event_t* e) { dlySliderCommonCb(e, "FB",   g_dlyFbPct,  ui_lblDelayFeedbackValue, "%d %%"); }

// REV
static void revBypassChangedCb(lv_event_t* e)
{
  (void)e;
  bool swOn = ui_ReverbOnBypass && lv_obj_has_state(ui_ReverbOnBypass, LV_STATE_CHECKED);
  if (swOn == g_revEnabled) return;
  g_revEnabled = swOn;

  if (sendRateOk()) {
    const int en = g_revEnabled ? 1 : 0;
    teensySend3("REV", "EN", en);
    mqttPublish(t_ui(), String("REV,EN=") + String(en));
  }
}

static void revSliderCommonCb(lv_event_t* e, const char* key, int& target, lv_obj_t* valLbl, const char* fmt)
{
  lv_obj_t* s = (lv_obj_t*)lv_event_get_target(e);
  int v = (int)lv_slider_get_value(s);
  if (v == target) return;
  target = v;

  uiSetValueLabel(valLbl, fmt, v);

  if (sendRateOk()) {
    teensySend3("REV", key, target);
    mqttPublish(t_ui(), String("REV,") + key + "=" + String(target));
  }
}

static void revPreCb  (lv_event_t* e) { revSliderCommonCb(e, "PDLY", g_revPreMs,   ui_lblReverbPDelayValue, "%d ms"); }
static void revMixCb  (lv_event_t* e) { revSliderCommonCb(e, "MIX",  g_revMixPct,  ui_lblReverbMixValue,    "%d %%"); }
static void revSizeCb (lv_event_t* e) { revSliderCommonCb(e, "SIZE", g_revSizePct, ui_lblReverbSizeValue,   "%d %%"); }
static void revDampCb (lv_event_t* e) { revSliderCommonCb(e, "DAMP", g_revDampPct, ui_lblReverbDampValue,   "%d %%"); }

// ====================== Splash overlay ======================
static lv_obj_t*  g_splash = nullptr;
static uint32_t   g_splashStartMs = 0;
static bool       g_splashVisible = false;
static constexpr uint32_t SPLASH_MS = 700;

static void splashShow()
{
  if (g_splashVisible) return;
  g_splash = lv_obj_create(lv_screen_active());
  lv_obj_set_size(g_splash, DISP_HOR, DISP_VER);
  lv_obj_set_style_bg_color(g_splash, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(g_splash, LV_OPA_COVER, 0);
  lv_obj_clear_flag(g_splash, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* lbl = lv_label_create(g_splash);
  lv_label_set_text(lbl, "VOX_EFX\nBooting...");
  lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
  lv_obj_center(lbl);

  g_splashVisible = true;
  g_splashStartMs = millis();
}

static void splashHideIfTime(uint32_t now)
{
  if (!g_splashVisible) return;
  if (now - g_splashStartMs < SPLASH_MS) return;

  if (g_splash) {
    lv_obj_del(g_splash);
    g_splash = nullptr;
  }
  g_splashVisible = false;
}

// ====================== MQTT ======================
// NOTE: DO NOT use Serial for debug (UART0 is Teensy). Publish via MQTT instead.

static const char* MQTT_HOST = "10.0.0.19";   // <-- set your broker (example from your network)
static const uint16_t MQTT_PORT = 1883;

// Optional auth (leave empty if not used)
static const char* MQTT_USER = "";
static const char* MQTT_PASS = "";

// Device identity / topics
static const char* MQTT_CLIENT_ID = "VOX_EFX_ESP32";
static const char* TOPIC_BASE     = "vox_efx/esp32";   // everything publishes under this

// Common topics
static String t_status()   { return String(TOPIC_BASE) + "/status"; }     // retained: online/offline
static String t_boot()     { return String(TOPIC_BASE) + "/boot"; }       // boot info
static String t_wifi()     { return String(TOPIC_BASE) + "/wifi"; }       // wifi state changes
static String t_hb()       { return String(TOPIC_BASE) + "/hb"; }         // heartbeat
static String t_ui()       { return String(TOPIC_BASE) + "/ui"; }         // UI interactions
static String t_cmd_in()   { return String(TOPIC_BASE) + "/cmd/in"; }     // (optional) inbound commands

static String t_uart_tx()  { return String(TOPIC_BASE) + "/uart/tx"; }
static String t_uart_rx()  { return String(TOPIC_BASE) + "/uart/rx"; }
static String t_uart_info(){ return String(TOPIC_BASE) + "/uart/info"; }

static WiFiClient   g_wifiClient;
static PubSubClient g_mqtt(g_wifiClient);

static bool     g_mqttEverConnected = false;
static uint32_t g_lastMqttAttemptMs = 0;
static uint32_t g_lastHbMs          = 0;

static constexpr uint32_t MQTT_RETRY_MS = 3000;
static constexpr uint32_t HB_MS         = 5000;

static void mqttPublishRetained(const String& topic, const String& payload)
{
  if (!g_mqtt.connected()) return;
  g_mqtt.publish(topic.c_str(), payload.c_str(), true /*retained*/);
}

static void mqttPublish(const String& topic, const String& payload)
{
  if (!g_mqtt.connected()) return;
  g_mqtt.publish(topic.c_str(), payload.c_str(), false /*retained*/);
}

static void mqttOnMessage(char* topic, byte* payload, unsigned int len)
{
  // Optional: handle inbound MQTT commands without Serial printing.
  // Example payloads you might support later:
  //  - "PAGE,EQ" or "VOL,55" etc.
  //
  // For now: do nothing (or implement later).
  (void)topic; (void)payload; (void)len;
}

static void mqttConfigure()
{
  g_mqtt.setServer(MQTT_HOST, MQTT_PORT);
  g_mqtt.setCallback(mqttOnMessage);
  g_mqtt.setKeepAlive(15);
  g_mqtt.setSocketTimeout(3);
}

static bool mqttConnectNow()
{
  if (!g_wifiConnected) return false;

  // Last Will: offline (retained)
  const String willTopic = t_status();
  const char* willMsg = "offline";

  bool ok;
  if (MQTT_USER && MQTT_USER[0]) {
    ok = g_mqtt.connect(
      MQTT_CLIENT_ID,
      MQTT_USER,
      MQTT_PASS,
      willTopic.c_str(),
      1,     // qos
      true,  // retained
      willMsg
    );
  } else {
    ok = g_mqtt.connect(
      MQTT_CLIENT_ID,
      willTopic.c_str(),
      1,
      true,
      willMsg
    );
  }

  if (ok) {
    g_mqttEverConnected = true;

    // Online retained
    mqttPublishRetained(t_status(), "online");

    // Subscribe to optional inbound commands
    g_mqtt.subscribe(t_cmd_in().c_str(), 0);

    // Boot payload (short + useful)
    String boot = "boot: ok, ip=" + WiFi.localIP().toString() + ", rssi=" + String(WiFi.RSSI());
    mqttPublish(t_boot(), boot);

    // Initial wifi state snapshot
    mqttPublishRetained(t_wifi(), "connected:" + WiFi.SSID());
  }

  return ok;
}

static void mqttEnsureConnected(uint32_t nowMs)
{
  if (!g_wifiConnected) {
    // If WiFi dropped, ensure status reflects it when we can next connect.
    return;
  }

  if (g_mqtt.connected()) return;

  if (nowMs - g_lastMqttAttemptMs < MQTT_RETRY_MS) return;
  g_lastMqttAttemptMs = nowMs;

  mqttConnectNow();
}

static void mqttHeartbeat(uint32_t nowMs)
{
  if (!g_mqtt.connected()) return;
  if (nowMs - g_lastHbMs < HB_MS) return;
  g_lastHbMs = nowMs;

  // Keep it tiny to avoid heap churn
  mqttPublish(t_hb(), String("ms=") + String(nowMs));
}

static void mqttLoop(uint32_t nowMs)
{
  if (!g_wifiConnected) return;

  mqttEnsureConnected(nowMs);
  if (g_mqtt.connected()) g_mqtt.loop();
  mqttHeartbeat(nowMs);
}

// ====================== WiFi + NVS ======================
enum class WifiState : uint8_t { Disconnected, Connecting, Connected };
static WifiState g_wifiState = WifiState::Disconnected;

static Preferences g_prefs;
static String g_cfgSsid;
static String g_cfgPass;

static bool     g_scanInProgress = false;
static uint32_t g_scanStartMs    = 0;
static bool     g_connectInProgress = false;
static uint32_t g_connectStartMs    = 0;


static constexpr uint32_t CONNECT_TIMEOUT_MS = 15000;

static const char* WIFI_DD_PLACEHOLDER = "Select a network";

static void uiSetScanButtonText(const char* txt)    { if (ui_Label3)        lv_label_set_text(ui_Label3, txt); }
static void uiSetConnectButtonText(const char* txt) { if (ui_Label2)        lv_label_set_text(ui_Label2, txt); }
static void uiSetWifiStatus(const char* txt)        { if (ui_lblWifiStatus) lv_label_set_text(ui_lblWifiStatus, txt); }

static void setWifiUiBusy(bool busy)
{
  if (ui_btnWifiConnect) busy ? lv_obj_add_state(ui_btnWifiConnect, LV_STATE_DISABLED)
                              : lv_obj_clear_state(ui_btnWifiConnect, LV_STATE_DISABLED);
  if (ui_btnWifiScan)    busy ? lv_obj_add_state(ui_btnWifiScan,    LV_STATE_DISABLED)
                              : lv_obj_clear_state(ui_btnWifiScan,    LV_STATE_DISABLED);
  if (ui_listWifi)       busy ? lv_obj_add_state(ui_listWifi,       LV_STATE_DISABLED)
                              : lv_obj_clear_state(ui_listWifi,       LV_STATE_DISABLED);
}

static void setWifiState(WifiState state)
{
  if (ui_imgWifiDisconnected && ui_imgWifiConnecting && ui_imgWifiConnected) {
    lv_obj_add_flag(ui_imgWifiDisconnected, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_imgWifiConnecting,   LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_imgWifiConnected,    LV_OBJ_FLAG_HIDDEN);

    switch (state) {
      case WifiState::Disconnected: lv_obj_remove_flag(ui_imgWifiDisconnected, LV_OBJ_FLAG_HIDDEN); break;
      case WifiState::Connecting:   lv_obj_remove_flag(ui_imgWifiConnecting,   LV_OBJ_FLAG_HIDDEN); break;
      case WifiState::Connected:    lv_obj_remove_flag(ui_imgWifiConnected,    LV_OBJ_FLAG_HIDDEN); break;
    }
  }
  g_wifiState = state;
}
static void wifiConnectStartFromUi()
{
  if (!ui_listWifi || !ui_taWifiPass) return;

  // 1. Get selected SSID
  uint16_t idx = lv_dropdown_get_selected(ui_listWifi);
  if (idx == 0) {
    uiSetWifiStatus("Select a WiFi network.");
    return;
  }

  char ssidBuf[64] = {0};
  lv_dropdown_get_selected_str(ui_listWifi, ssidBuf, sizeof(ssidBuf));
  String ssid = ssidBuf;
  ssid.trim();

  if (!ssid.length()) {
    uiSetWifiStatus("Invalid SSID.");
    return;
  }

  // 2. Get password
  String pass = lv_textarea_get_text(ui_taWifiPass);
  pass.trim();

  // 3. Update UI
  uiSetWifiStatus("Connecting...");
  setWifiUiBusy(true);

  // 4. Start connection
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  delay(100);

  WiFi.begin(ssid.c_str(), pass.c_str());

  g_connectInProgress = true;
  g_connectStartMs    = millis();
  setWifiState(WifiState::Connecting);
  uiSetConnectButtonText("Con...");
}

static void netConnectBtnCb(lv_event_t* e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  wifiConnectStartFromUi();
}

static void saveCredsToNvs(const String& ssid, const String& pass)
{
  g_prefs.begin("vox_efx", false);
  g_prefs.putString("ssid", ssid);
  g_prefs.putString("pass", pass);
  g_prefs.end();

  g_cfgSsid = ssid;
  g_cfgPass = pass;
}

static void loadCredsFromNvs()
{
  g_prefs.begin("vox_efx", true);
  g_cfgSsid = g_prefs.getString("ssid", "");
  g_cfgPass = g_prefs.getString("pass", "");
  g_prefs.end();
}

static void wifiAutoConnectIfSaved()
{
  if (g_cfgSsid.length() == 0) return; // nothing saved

  // Start WiFi connection immediately
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(false);
  delay(20);

  uiSetWifiStatus(("Auto-connecting to " + g_cfgSsid + "...").c_str());
  setWifiState(WifiState::Connecting);
  uiSetConnectButtonText("Con...");
  setWifiUiBusy(true);

  WiFi.begin(g_cfgSsid.c_str(), g_cfgPass.c_str());
  g_connectInProgress = true;
  g_connectStartMs    = millis();
}

static String dropdownGetSelectedSsid()
{
  if (!ui_listWifi) return "";
  char buf[96] = {0};
  lv_dropdown_get_selected_str(ui_listWifi, buf, sizeof(buf));
  String s(buf); s.trim();
  return s;
}

static void dropdownShowPlaceholder()
{
  if (!ui_listWifi) return;
  lv_dropdown_set_selected(ui_listWifi, 0);
}

// Non-jitter SSID select: finds the option index without iterating selection.
// Returns true if it found + set the selection.
static bool dropdownSelectSsidFast(const String& ssid)
{
  if (!ui_listWifi) return false;

  String target = ssid;
  target.replace("\n", " ");
  target.trim();
  if (!target.length()) return false;

  const char* opts = lv_dropdown_get_options(ui_listWifi);
  if (!opts) return false;

  // Scan each option line in the options string
  int idx = 0;
  const char* p = opts;

  while (*p) {
    const char* e = p;
    while (*e && *e != '\n') e++;

    char buf[96];
    size_t len = (size_t)(e - p);
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, p, len);
    buf[len] = 0;

    String opt(buf);
    opt.replace("\n", " ");
    opt.trim();

    if (opt == target) {
      // Only do the actual selection once, with callback temporarily detached
      lv_obj_remove_event_cb(ui_listWifi, wifiSsidChangedCb);

      int cur = (int)lv_dropdown_get_selected(ui_listWifi);
      if (cur != idx) lv_dropdown_set_selected(ui_listWifi, idx);

      lv_obj_add_event_cb(ui_listWifi, wifiSsidChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);
      return true;
    }

    idx++;
    p = (*e == '\n') ? (e + 1) : e;
  }

  return false;
}


static void wifiSsidChangedCb(lv_event_t* e)
{
  (void)e;
  String ssid = dropdownGetSelectedSsid();

  if (!ssid.length() || ssid == WIFI_DD_PLACEHOLDER || ssid == "Scan failed") {
    uiSetWifiStatus("Select a network");
    return;
  }

  if (ssid == g_cfgSsid && ui_taWifiPass) {
    lv_textarea_set_text(ui_taWifiPass, g_cfgPass.c_str());
  }

  String msg = "Selected: " + ssid;
  uiSetWifiStatus(msg.c_str());
}

static void wifiScanStart()
{
  if (g_scanInProgress) return;
  if (g_connectInProgress) return;

  // Always ensure STA + no sleep (more reliable scanning)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Don't hard-reset WiFi unless absolutely necessary.
  // Scanning while connected is OK.
  // If you want a reset when *not connected*, keep it gentle:
  if (WiFi.status() != WL_CONNECTED) {
    // Stop any previous scan results and stop lingering attempts
    WiFi.disconnect(false);   // don't turn WiFi off; just disconnect
    delay(20);
  }

  WiFi.scanDelete();
  delay(20);

  // Use the full signature + PASSIVE scan for reliability.
  // async=true, show_hidden=true, passive=true, max_ms_per_chan=300
  int rc = WiFi.scanNetworks(
      true  /* async */,
      true  /* show_hidden */,
      true  /* passive */,
      300   /* max_ms_per_chan */,
      0     /* channel (0=all) */,
      nullptr /* ssid filter */
  );

  if (rc == WIFI_SCAN_RUNNING || rc >= 0) {
    g_scanInProgress = true;
    g_scanStartMs = millis();
    uiSetScanButtonText("Scan...");
    uiSetWifiStatus("Scanning...");
    setWifiUiBusy(true);
  } else {
    // Better on-screen diagnostics (no Serial)
    String msg = String("Scan start failed rc=") + rc +
                 " st=" + (int)WiFi.status() +
                 " heap=" + ESP.getFreeHeap() +
                 " min=" + ESP.getMinFreeHeap();

    if (ui_listWifi) {
      String o = String(WIFI_DD_PLACEHOLDER) + "\nScan failed";
      lv_dropdown_set_options(ui_listWifi, o.c_str());
      lv_dropdown_set_selected(ui_listWifi, 0);
    }
    uiSetScanButtonText("Scan");
    uiSetWifiStatus(msg.c_str());
    setWifiUiBusy(false);
  }
}

static void wifiScanPoll()
{
  if (!g_scanInProgress) return;

  // Safety: scan timeout (prevents "Scan..." forever)
  static constexpr uint32_t SCAN_TIMEOUT_MS = 12000;
  if (millis() - g_scanStartMs > SCAN_TIMEOUT_MS) {
    g_scanInProgress = false;
    WiFi.scanDelete();
    uiSetScanButtonText("Scan");
    setWifiUiBusy(false);

    // Add heap info to help diagnose memory pressure
    String msg = String("Scan timeout. heap=") + ESP.getFreeHeap()
               + " min=" + ESP.getMinFreeHeap();
    uiSetWifiStatus(msg.c_str());
    return;
  }

  int n = WiFi.scanComplete();

  // Still running
  if (n == WIFI_SCAN_RUNNING || n == -1) return;

  g_scanInProgress = false;

  // Always restore UI state
  uiSetScanButtonText("Scan");
  setWifiUiBusy(false);

  if (!ui_listWifi) {
    WiFi.scanDelete();
    return;
  }

  // Build options list
  String opts;
  opts.reserve((size_t)(n > 0 ? n : 0) * 24 + 32);
  opts = WIFI_DD_PLACEHOLDER;

  // If failed, show the actual failure code + heap info
  // If failed, show the actual failure code + heap info
if (n < 0) {
  String msg = String("Scan failed n=") + n +
               " st=" + (int)WiFi.status() +
               " heap=" + ESP.getFreeHeap() +
               " min=" + ESP.getMinFreeHeap();

  // Update status label (best place for long debug text)
  uiSetWifiStatus(msg.c_str());

  // Keep dropdown clean/simple
  opts += "\nScan failed";

  lv_obj_remove_event_cb(ui_listWifi, wifiSsidChangedCb);
  lv_dropdown_set_options(ui_listWifi, opts.c_str());
  lv_dropdown_set_selected(ui_listWifi, 0);
  lv_obj_add_event_cb(ui_listWifi, wifiSsidChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);

  WiFi.scanDelete();
  return;
}

  // Decide what SSID we want selected:
  //  - if connected -> current SSID
  //  - else -> saved SSID (if present)
  //  - else -> placeholder
  String want;
  if (WiFi.status() == WL_CONNECTED) want = WiFi.SSID();
  else if (g_cfgSsid.length())       want = g_cfgSsid;
  want.replace("\n", " ");
  want.trim();

  int wantIdx = 0; // placeholder is always index 0
  int addedCount = 0;

  // Collect SSIDs (dedupe) and compute wantIdx reliably
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    ssid.replace("\n", " ");
    ssid.trim();
    if (!ssid.length()) continue;

    // crude dedupe: check if "\nSSID" already exists
    String needle = "\n" + ssid;
    if (opts.indexOf(needle) >= 0) continue;

    // Append to dropdown options
    opts += "\n";
    opts += ssid;

    // Track real dropdown index (placeholder is 0, first SSID is 1, etc.)
    addedCount++;
    if (wantIdx == 0 && want.length() && ssid == want) {
      wantIdx = addedCount; // because placeholder=0, first added=1
    }
  }

  // Update dropdown options WITHOUT triggering value-changed churn
  lv_obj_remove_event_cb(ui_listWifi, wifiSsidChangedCb);
  lv_dropdown_set_options(ui_listWifi, opts.c_str());
  lv_dropdown_set_selected(ui_listWifi, wantIdx);
  lv_obj_add_event_cb(ui_listWifi, wifiSsidChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);

  WiFi.scanDelete();

  // Status text
  if (addedCount == 0) uiSetWifiStatus("No networks found.");
  else                 uiSetWifiStatus("Scan complete.");
}

static void wifiConnectWithUiSelection()
{
  if (g_scanInProgress) { uiSetWifiStatus("Wait for scan to finish."); return; }

  String ssid = dropdownGetSelectedSsid();
  if (!ssid.length() || ssid == WIFI_DD_PLACEHOLDER || ssid == "Scan failed") {
    uiSetWifiStatus("Select a network first.");
    return;
  }

  const char* pass = "";
  if (ui_taWifiPass) pass = lv_textarea_get_text(ui_taWifiPass);

  if (ui_pwKB)       lv_obj_add_flag(ui_pwKB, LV_OBJ_FLAG_HIDDEN);
  if (ui_taWifiPass) lv_obj_clear_state(ui_taWifiPass, LV_STATE_FOCUSED);

  saveCredsToNvs(ssid, String(pass));

  uiSetWifiStatus(("Connecting to " + ssid + "...").c_str());

  setWifiState(WifiState::Connecting);
  uiSetConnectButtonText("Con...");
  setWifiUiBusy(true);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  delay(20);
  WiFi.begin(ssid.c_str(), pass);

  g_connectInProgress = true;
  g_connectStartMs = millis();
}

static const char* wifiFailReason(wl_status_t s)
{
  switch (s) {
    case WL_NO_SSID_AVAIL:   return "Network not found";
    case WL_CONNECT_FAILED:  return "Authentication failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED:    return "Disconnected";
    default:                 return "Connection failed";
  }
}

static void wifiPollAndUpdateUi()
{
  if (g_scanInProgress) return;

  wl_status_t s = WiFi.status();
  bool connected = (s == WL_CONNECTED);

  if (g_connectInProgress) {
    if (connected) {
      g_connectInProgress = false;
      g_wifiConnected = true;

      setWifiState(WifiState::Connected);

      // MQTT: announce WiFi connected (retained)
      mqttPublishRetained(t_wifi(), "connected:" + WiFi.SSID());
      
      setWifiUiBusy(false);
      uiSetConnectButtonText("Con");

      dropdownSelectSsidFast(WiFi.SSID());

      String msg = "Connected: " + WiFi.SSID() + "\nIP: " + WiFi.localIP().toString();
      uiSetWifiStatus(msg.c_str());
      return;
    }

    if (millis() - g_connectStartMs > CONNECT_TIMEOUT_MS) {
      g_connectInProgress = false;
      g_wifiConnected = false;

      setWifiState(WifiState::Disconnected);
      setWifiUiBusy(false);
      uiSetConnectButtonText("Con");
      dropdownShowPlaceholder();

      String msg = String("Failed: ") + wifiFailReason(s);
      uiSetWifiStatus(msg.c_str());

      // MQTT: announce WiFi disconnected (retained when possible later)
      // If MQTT isn't connected, this won't publish (that's OK).
      mqttPublishRetained(t_wifi(),"disconnected");

      WiFi.disconnect(false);
      return;
    }
    return;
  }

  if (connected != g_wifiConnected) {
    g_wifiConnected = connected;

    if (connected) {
      setWifiState(WifiState::Connected);
      dropdownSelectSsidFast(WiFi.SSID());
      uiSetWifiStatus(("Connected: " + WiFi.SSID()).c_str());
      mqttPublishRetained(t_wifi(), "connected:" + WiFi.SSID());
    } else {
      setWifiState(WifiState::Disconnected);
      dropdownShowPlaceholder();
      uiSetWifiStatus("Not connected");
     mqttPublishRetained(t_wifi(), "disconnected");
      // If we were connected to MQTT, mark offline next time we can connect.
      // (LWT will cover sudden power loss; this covers clean disconnect cases.)
      if (g_mqtt.connected()) mqttPublishRetained(t_status(), "offline");
    }
  }
}

// ====================== LED state machine ======================
static uint32_t g_ledLastToggleMs = 0;
static bool     g_ledLevel = false;

static void wifiLedUpdate(uint32_t nowMs)
{
  if (g_wifiState == WifiState::Connected) {
    digitalWrite(PIN_LED, HIGH);
    return;
  }

  uint32_t periodMs = (g_wifiState == WifiState::Connecting) ? 250 : 1000;
  if (nowMs - g_ledLastToggleMs >= (periodMs / 2)) {
    g_ledLastToggleMs = nowMs;
    g_ledLevel = !g_ledLevel;
    digitalWrite(PIN_LED, g_ledLevel ? HIGH : LOW);
  }
}

// ====================== Keyboard events (Settings) ======================
static void kbEventCb(lv_event_t* e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (!ui_pwKB) return;

  if (code == LV_EVENT_READY) {
    // Hide KB overlay first
    lv_obj_add_flag(ui_pwKB, LV_OBJ_FLAG_HIDDEN);
    if (ui_taWifiPass) lv_obj_clear_state(ui_taWifiPass, LV_STATE_FOCUSED);

    // IMPORTANT: trigger connect immediately
    // (password text is already in the textarea at this point)
    wifiConnectWithUiSelection();
    return;
  }

  if (code == LV_EVENT_CANCEL) {
    lv_obj_add_flag(ui_pwKB, LV_OBJ_FLAG_HIDDEN);
    if (ui_taWifiPass) lv_obj_clear_state(ui_taWifiPass, LV_STATE_FOCUSED);
    uiSetWifiStatus("Password entry cancelled.");
    return;
  }
}

static void passTaEventCb(lv_event_t* e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (!ui_pwKB || !ui_taWifiPass) return;

  if (code == LV_EVENT_FOCUSED || code == LV_EVENT_CLICKED) {
    lv_keyboard_set_textarea(ui_pwKB, ui_taWifiPass);
    lv_obj_remove_flag(ui_pwKB, LV_OBJ_FLAG_HIDDEN);
    uiSetWifiStatus("Enter password, then OK.");
  }
}

// ====================== Boot sync to Teensy ======================
static void teensySendAll()
{
  // Use the same pacing rule as live UI sends (SEND_MIN_MS).
  // This prevents bursting 20+ lines back-to-back on UART0.
  auto waitSend = []() {
    while (!sendRateOk()) {
      delay(1);
    }
  };

  // Global
  waitSend(); teensySend2("INGAIN", g_inGainPct);
  waitSend(); teensySend2("VOL",    g_outVolPct);

  // EQ
  waitSend(); teensySend3("EQ", "EN",   g_eqEnabled ? 1 : 0);
  waitSend(); teensySend3("EQ", "HPF",  g_eqHpfHz);
  waitSend(); teensySend3("EQ", "WARM", g_eqWarmDb);
  waitSend(); teensySend3("EQ", "MUD",  g_eqMudDb);
  waitSend(); teensySend3("EQ", "MID",  g_eqMidDb);
  waitSend(); teensySend3("EQ", "PRES", g_eqPresDb);
  waitSend(); teensySend3("EQ", "AIR",  g_eqAirDb);

  // COMP
  waitSend(); teensySend3("COMP", "EN",   g_compEnabled ? 1 : 0);
  waitSend(); teensySend3("COMP", "THR",  g_compThrDb);
  waitSend(); teensySend3("COMP", "RAT",  g_compRatioX10);
  waitSend(); teensySend3("COMP", "ATK",  g_compAtkMs);
  waitSend(); teensySend3("COMP", "REL",  g_compRelMs);
  waitSend(); teensySend3("COMP", "MAKE", g_compMakeDb);

  // DLY
  waitSend(); teensySend3("DLY", "EN",   g_dlyEnabled ? 1 : 0);
  waitSend(); teensySend3("DLY", "TIME", g_dlyTimeMs);
  waitSend(); teensySend3("DLY", "FB",   g_dlyFbPct);

  // REV
  waitSend(); teensySend3("REV", "EN",   g_revEnabled ? 1 : 0);
  waitSend(); teensySend3("REV", "PDLY", g_revPreMs);
  waitSend(); teensySend3("REV", "MIX",  g_revMixPct);
  waitSend(); teensySend3("REV", "SIZE", g_revSizePct);
  waitSend(); teensySend3("REV", "DAMP", g_revDampPct);
}

static void btnNetworkSettingsCb(lv_event_t* e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  // Hide Settings page
  lv_obj_add_flag(ui_pnlSettings, LV_OBJ_FLAG_HIDDEN);

  // Show Network Setup page
  lv_obj_clear_flag(ui_pnlNetworkSetup, LV_OBJ_FLAG_HIDDEN);
}

static void btnNetworkSetupCloseCb(lv_event_t* e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  // Hide Network Setup page
  lv_obj_add_flag(ui_pnlNetworkSetup, LV_OBJ_FLAG_HIDDEN);

  // Show Settings page
  lv_obj_clear_flag(ui_pnlSettings, LV_OBJ_FLAG_HIDDEN);

  // Also hide password overlay if it happens to be open
  lv_obj_add_flag(ui_kbPassword, LV_OBJ_FLAG_HIDDEN);
}

static void btnWifiConnectCb(lv_event_t* e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  // Show overlay panel
  lv_obj_clear_flag(ui_kbPassword, LV_OBJ_FLAG_HIDDEN);

  // Focus the textarea (optional but nice)
  lv_obj_clear_state(ui_taWifiPass, LV_STATE_DISABLED);
  lv_textarea_set_text(ui_taWifiPass, "");         // optional: clear previous
  lv_obj_add_state(ui_taWifiPass, LV_STATE_FOCUSED);

  // Attach keyboard to textarea
  lv_keyboard_set_textarea(ui_pwKB, ui_taWifiPass);
}

static void pwKbEventCb(lv_event_t* e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_READY) {
    // User pressed the OK / checkmark on the keyboard:
    // Attempt connection using current dropdown SSID + textarea password
    wifiConnectWithUiSelection();

    // Hide the keyboard overlay panel
    lv_obj_add_flag(ui_kbPassword, LV_OBJ_FLAG_HIDDEN);
    return;
  }

  if (code == LV_EVENT_CANCEL) {
    // User cancelled keyboard
    lv_obj_add_flag(ui_kbPassword, LV_OBJ_FLAG_HIDDEN);
    return;
  }
}

static void uiSetPeakMeters(int inSeg, int outSeg)
{
  inSeg  = constrain(inSeg,  0, 15);
  outSeg = constrain(outSeg, 0, 15);

  if (ui_peakMeterInput)
    lv_slider_set_value(ui_peakMeterInput, inSeg, LV_ANIM_OFF);

  if (ui_peakMeterOutput)
    lv_slider_set_value(ui_peakMeterOutput, outSeg, LV_ANIM_OFF);
}


static void parseTeensyLine(char* line)
{
  if (!line || !*line) return;

  // Optional: handle PONG reply from Teensy
  if (!strcmp(line, "PONG")) {
    uiSetWifiStatus("Teensy: PONG");   // or your own UI label
    // optionally mqttPublish(t_uart_info(), "PONG");
    return;
  }

  if (!strcmp(line, "PONG")) {
  mqttPublish(String(TOPIC_BASE) + "/uart/pong", "1");
  return;
}
  // Existing meter parsing
  if (strncmp(line, "MTR,", 4) == 0)
  {
    char* p = line + 4;
    char* a = strtok(p, ",");
    char* b = strtok(nullptr, ",");

    if (a && b)
    {
      int inSeg  = atoi(a);
      int outSeg = atoi(b);
      uiSetPeakMeters(inSeg, outSeg);
    }
  }
}

static void teensyTelemetryPoll()
{
  static char line[128];
  static size_t n = 0;
 int budget = 256; // max bytes per loop

  while (budget-- > 0 && Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '\r') continue;

   if (c == '\n')
{
  line[n] = 0;

  if (n > 0)
  {
    // MQTT “wire tap” (throttle the high-rate MTR stream)
    static uint32_t lastMtrPubMs = 0;
    const uint32_t now = millis();

    if (strncmp(line, "MTR,", 4) == 0) {
      // Teensy sends MTR at 20Hz; publish at ~2Hz to avoid flooding
      if (now - lastMtrPubMs >= 500) {
        lastMtrPubMs = now;
        mqttPublish(t_uart_rx(), String(line));
      }
    } else {
      // publish everything else
      mqttPublish(t_uart_rx(), String(line));
    }

    // Existing parse
    parseTeensyLine(line);
  }

  n = 0;
}
    else
    {
      if (n < sizeof(line) - 1)
        line[n++] = c;
      else
        n = 0; // overflow reset
    }
  }
}

// ====================== setup / loop ======================
void setup()
{
  // UART0 to Teensy
  Serial.begin(115200);

  // LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_LED, HIGH); delay(60);
    digitalWrite(PIN_LED, LOW);  delay(60);
  }

  loadCredsFromNvs();

  mqttConfigure();

#if ENABLE_ENCODER_NAV
  pinMode(PIN_ENC_A,   INPUT_PULLUP);
  pinMode(PIN_ENC_B,   INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  g_lastAB = ((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
#endif

  // TFT init
  delay(20);
  tft.init();
  tft.setRotation(TFT_ROT);
  tft.setSwapBytes(true);
  delay(80);

  // LVGL init + buffer
  lv_init();

  uint32_t lines = BUF_LINES_DEFAULT;
  while (lines >= 2 && !allocLvglBuffer(lines)) lines /= 2;
  if (!g_buf1) while (true) { delay(1000); }

  g_disp = lv_display_create(DISP_HOR, DISP_VER);
  lv_display_set_flush_cb(g_disp, my_flush_cb);
  lv_display_set_buffers(
    g_disp,
    g_buf1,
    nullptr,
    g_bufPixels * sizeof(lv_color_t),
    LV_DISPLAY_RENDER_MODE_PARTIAL
  );

#if ENABLE_FT6336U_TOUCH
  Wire.begin();
  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touch_read_cb);
#endif

  // SquareLine UI
  ui_init();

  // Splash overlay (no long delays)
  splashShow();

  // Initial visibility states
lv_obj_clear_flag(ui_pnlSettings, LV_OBJ_FLAG_HIDDEN);
lv_obj_add_flag(ui_pnlNetworkSetup, LV_OBJ_FLAG_HIDDEN);
lv_obj_add_flag(ui_kbPassword, LV_OBJ_FLAG_HIDDEN);

// Button wiring
lv_obj_add_event_cb(ui_btnNetworkSettings,    btnNetworkSettingsCb,    LV_EVENT_CLICKED, nullptr);
lv_obj_add_event_cb(ui_btnNetworkSetupClose,  btnNetworkSetupCloseCb,  LV_EVENT_CLICKED, nullptr);
lv_obj_add_event_cb(ui_btnWifiConnect,        btnWifiConnectCb,        LV_EVENT_CLICKED, nullptr);

// Keyboard wiring (OK/Cancel)
lv_obj_add_event_cb(ui_pwKB, pwKbEventCb, LV_EVENT_ALL, nullptr);


lv_obj_add_event_cb(ui_netConnect, netConnectBtnCb, LV_EVENT_CLICKED, nullptr);


  // ---------------------- Bind NAV buttons ----------------------
  makeClickable(ui_btnCompressor, navToPageCb, (void*)(uintptr_t)Page::Comp);
  makeClickable(ui_ImageComp,     navToPageCb, (void*)(uintptr_t)Page::Comp);

  makeClickable(ui_btnDelay,      navToPageCb, (void*)(uintptr_t)Page::Delay);
  makeClickable(ui_ImageDelay,    navToPageCb, (void*)(uintptr_t)Page::Delay);

  makeClickable(ui_btnReverb,     navToPageCb, (void*)(uintptr_t)Page::Reverb);
  makeClickable(ui_ImageReverb,   navToPageCb, (void*)(uintptr_t)Page::Reverb);

  makeClickable(ui_btnEQ,         navToPageCb, (void*)(uintptr_t)Page::EQ);
  makeClickable(ui_ImageEQ,       navToPageCb, (void*)(uintptr_t)Page::EQ);

  makeClickable(ui_btnSetup,      navToPageCb, (void*)(uintptr_t)Page::Settings);
  makeClickable(ui_LabelSetup,    navToPageCb, (void*)(uintptr_t)Page::Settings);
  makeClickable(ui_Image4,        navToPageCb, (void*)(uintptr_t)Page::Settings);
  makeClickable(ui_Image5,        navToPageCb, (void*)(uintptr_t)Page::Settings);

  // Back bindings
  makeClickable(ui_btnBackComp,        backToHomeCb);
  makeClickable(ui_ImageBtnBackComp,   backToHomeCb);
  makeClickable(ui_btnBackDelay,       backToHomeCb);
  makeClickable(ui_ImageBtnBackDelay,  backToHomeCb);
  makeClickable(ui_btnBackReverb,      backToHomeCb);
  makeClickable(ui_ImageBtnBackReverb, backToHomeCb);
  makeClickable(ui_btnBackEq,          backToHomeCb);
  makeClickable(ui_ImageBtnBackEq,     backToHomeCb);
  makeClickable(ui_btnBackSetup,       backToHomeCb);
  makeClickable(ui_ImageBtnBackSetup,  backToHomeCb);

  // ---------------------- Bind SETTINGS (WiFi + Globals) ----------------------
  if (ui_pwKB) lv_obj_add_flag(ui_pwKB, LV_OBJ_FLAG_HIDDEN);
  if (ui_taWifiPass) lv_obj_add_event_cb(ui_taWifiPass, passTaEventCb, LV_EVENT_ALL, nullptr);
  if (ui_pwKB)       lv_obj_add_event_cb(ui_pwKB,       kbEventCb,     LV_EVENT_ALL, nullptr);

  makeClickable(ui_btnWifiScan,    (lv_event_cb_t)[](lv_event_t* e){ (void)e; wifiScanStart(); });
  makeClickable(ui_btnWifiConnect, (lv_event_cb_t)[](lv_event_t* e){ (void)e; wifiConnectWithUiSelection(); });

  if (ui_listWifi) {
    lv_dropdown_set_options(ui_listWifi, WIFI_DD_PLACEHOLDER);
    lv_dropdown_set_selected(ui_listWifi, 0);
    lv_obj_add_event_cb(ui_listWifi, wifiSsidChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  // Global sliders
  if (ui_sliderGlobalOutputVol) {
    lv_slider_set_range(ui_sliderGlobalOutputVol, 0, 100);
    lv_slider_set_value(ui_sliderGlobalOutputVol, g_outVolPct, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderGlobalOutputVol, globalOutVolCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderGlobalInputGain) {
    lv_slider_set_range(ui_sliderGlobalInputGain, 0, 100);
    lv_slider_set_value(ui_sliderGlobalInputGain, g_inGainPct, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderGlobalInputGain, globalInGainCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  uiSetValueLabel(ui_lblGlobalOutputVolValue, "%d %%", g_outVolPct);
  uiSetValueLabel(ui_lblGlobalInputGainValue, "%d %%", g_inGainPct);

  uiSetWifiStatus("Select a network");
  setWifiState(WifiState::Disconnected);
  setWifiUiBusy(false);

  if (ui_taWifiPass && g_cfgPass.length()) lv_textarea_set_text(ui_taWifiPass, g_cfgPass.c_str());
  
  wifiAutoConnectIfSaved();

  // ---------------------- Bind EQ ----------------------
  if (ui_EqOnBypass) lv_obj_add_event_cb(ui_EqOnBypass, eqBypassChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);

  if (ui_sliderEqHPF) {
    lv_slider_set_range(ui_sliderEqHPF, 20, 300);
    lv_slider_set_value(ui_sliderEqHPF, g_eqHpfHz, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderEqHPF, eqHpfCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderEqWarmth) {
    lv_slider_set_range(ui_sliderEqWarmth, -24, 24);
    lv_slider_set_value(ui_sliderEqWarmth, g_eqWarmDb, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderEqWarmth, eqWarmCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderEqMud) {
    lv_slider_set_range(ui_sliderEqMud, -24, 24);
    lv_slider_set_value(ui_sliderEqMud, g_eqMudDb, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderEqMud, eqMudCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderEqMid) {
    lv_slider_set_range(ui_sliderEqMid, -24, 24);
    lv_slider_set_value(ui_sliderEqMid, g_eqMidDb, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderEqMid, eqMidCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderEqPres) {
    lv_slider_set_range(ui_sliderEqPres, -24, 24);
    lv_slider_set_value(ui_sliderEqPres, g_eqPresDb, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderEqPres, eqPresCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderEqAir) {
    lv_slider_set_range(ui_sliderEqAir, -24, 24);
    lv_slider_set_value(ui_sliderEqAir, g_eqAirDb, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderEqAir, eqAirCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  uiSetValueLabel(ui_lblEqHPFValue,  "%d Hz",   g_eqHpfHz);
  uiSetValueLabel(ui_lblEqWarmValue, "%+d dB",  g_eqWarmDb);
  uiSetValueLabel(ui_lblEqMudValue,  "%+d dB",  g_eqMudDb);
  uiSetValueLabel(ui_lblEqMidValue,  "%+d dB",  g_eqMidDb);
  uiSetValueLabel(ui_lblEqPresValue, "%+d dB",  g_eqPresDb);
  uiSetValueLabel(ui_lblEqAirValue,  "%+d dB",  g_eqAirDb);

  // ---------------------- Bind COMP ----------------------
  if (ui_CompOnBypass) lv_obj_add_event_cb(ui_CompOnBypass, compBypassChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);

  if (ui_sliderCompThreshold) {
    lv_slider_set_range(ui_sliderCompThreshold, -60, 0);
    lv_slider_set_value(ui_sliderCompThreshold, g_compThrDb, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderCompThreshold, compThrCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  if (ui_sliderCompRatio) {
    // store ratio as x10: 10=1.0:1 ... 200=20.0:1
    lv_slider_set_range(ui_sliderCompRatio, 10, 200);
    lv_slider_set_value(ui_sliderCompRatio, g_compRatioX10, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderCompRatio, compRatioCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  if (ui_sliderCompAttack) {
    lv_slider_set_range(ui_sliderCompAttack, 1, 100);
    lv_slider_set_value(ui_sliderCompAttack, g_compAtkMs, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderCompAttack, compAtkCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  if (ui_sliderCompRelease) {
    lv_slider_set_range(ui_sliderCompRelease, 20, 600);
    lv_slider_set_value(ui_sliderCompRelease, g_compRelMs, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderCompRelease, compRelCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  if (ui_sliderCompGain) {
    lv_slider_set_range(ui_sliderCompGain, 0, 24);
    lv_slider_set_value(ui_sliderCompGain, g_compMakeDb, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderCompGain, compMakeCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  uiSetValueLabel(ui_lblCompThresholdValue, "%d dB", g_compThrDb);
  uiSetValueLabel(ui_lblCompAttackValue,    "%d ms", g_compAtkMs);
  uiSetValueLabel(ui_lblCompReleaseValue,   "%d ms", g_compRelMs);
  uiSetValueLabel(ui_lblCompGainValue,      "%d dB", g_compMakeDb);
  // Ratio label is ui_lblCompSlider2Value
  if (ui_lblCompSlider2Value) {
    char b[24];
    snprintf(b, sizeof(b), "%d.%d:1", g_compRatioX10 / 10, g_compRatioX10 % 10);
    lv_label_set_text(ui_lblCompSlider2Value, b);
  }

  // ---------------------- Bind DLY ----------------------
  if (ui_DelayOnBypass) lv_obj_add_event_cb(ui_DelayOnBypass, dlyBypassChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);

  if (ui_sliderDelayTime) {
    lv_slider_set_range(ui_sliderDelayTime, 0, 1200);
    lv_slider_set_value(ui_sliderDelayTime, g_dlyTimeMs, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderDelayTime, dlyTimeCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderDelayFeedback) {
    lv_slider_set_range(ui_sliderDelayFeedback, 0, 95);
    lv_slider_set_value(ui_sliderDelayFeedback, g_dlyFbPct, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderDelayFeedback, dlyFbCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  uiSetValueLabel(ui_lblDelayTimeValue,     "%d ms", g_dlyTimeMs);
  uiSetValueLabel(ui_lblDelayFeedbackValue, "%d %%", g_dlyFbPct);

  // ---------------------- Bind REV ----------------------
  if (ui_ReverbOnBypass) lv_obj_add_event_cb(ui_ReverbOnBypass, revBypassChangedCb, LV_EVENT_VALUE_CHANGED, nullptr);

  if (ui_sliderReverbPDelay) {
    lv_slider_set_range(ui_sliderReverbPDelay, 0, 200);
    lv_slider_set_value(ui_sliderReverbPDelay, g_revPreMs, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderReverbPDelay, revPreCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderReverbMix) {
    lv_slider_set_range(ui_sliderReverbMix, 0, 100);
    lv_slider_set_value(ui_sliderReverbMix, g_revMixPct, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderReverbMix, revMixCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderReverbSize) {
    lv_slider_set_range(ui_sliderReverbSize, 0, 100);
    lv_slider_set_value(ui_sliderReverbSize, g_revSizePct, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderReverbSize, revSizeCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }
  if (ui_sliderReverbDamp) {
    lv_slider_set_range(ui_sliderReverbDamp, 0, 100);
    lv_slider_set_value(ui_sliderReverbDamp, g_revDampPct, LV_ANIM_OFF);
    lv_obj_add_event_cb(ui_sliderReverbDamp, revDampCb, LV_EVENT_VALUE_CHANGED, nullptr);
  }

  uiSetValueLabel(ui_lblReverbPDelayValue, "%d ms", g_revPreMs);
  uiSetValueLabel(ui_lblReverbMixValue,    "%d %%", g_revMixPct);
  uiSetValueLabel(ui_lblReverbSizeValue,   "%d %%", g_revSizePct);
  uiSetValueLabel(ui_lblReverbDampValue,   "%d %%", g_revDampPct);

  //lv_slider_set_range(ui_peakMeterInput,  0, 15);
  //lv_slider_set_range(ui_peakMeterOutput, 0, 15);

  if (ui_peakMeterInput) {
    lv_slider_set_range(ui_peakMeterInput, 0, 15);
    lv_slider_set_value(ui_peakMeterInput, 0, LV_ANIM_OFF);
  }
  
  if (ui_peakMeterOutput) {
    lv_slider_set_range(ui_peakMeterOutput, 0, 15);
    lv_slider_set_value(ui_peakMeterOutput, 0, LV_ANIM_OFF);
  }

   // ---------------------- Start Home + WiFi scan prompt ----------------------
  showPage(Page::Home);

  // If you want to auto-scan on boot:
  wifiScanStart();

  // Boot sync to Teensy after UI exists
  teensySendAll();
}

void loop()
{
  static uint32_t last_ms = millis();
  uint32_t now  = millis();
  uint32_t diff = now - last_ms;
  last_ms = now;

  lv_tick_inc(diff);
  lv_timer_handler();
  teensyTelemetryPoll();
  
  splashHideIfTime(now);

#if ENABLE_ENCODER_NAV
  int step = read_encoder_step();
  if (step != 0) showNextPrev(step);
  if (read_button_pressed_edge()) showPage(Page::Home);
#endif

  static uint32_t lastWifiPoll = 0;
  if (now - lastWifiPoll >= 250) {
    lastWifiPoll = now;
    wifiPollAndUpdateUi();
  }
  mqttLoop(now);
  wifiScanPoll();
  wifiLedUpdate(now);

  static uint32_t lastPingMs = 0;
if (g_mqtt.connected() && millis() - lastPingMs > 1000) {
  lastPingMs = millis();
  teensySendLine("PING");
}

  delay(5);
}