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

// ====================== Screens ======================

enum class ScreenId : uint8_t {
  Main,
  Global,
  AudioDSP,
  Wifi,
  System
};

static ScreenId g_screen = ScreenId::Main;

static void showScreen(ScreenId s)
{
  g_screen = s;

  switch (s) {
    case ScreenId::Main:
      lv_screen_load(ui_ScreenMain);
      break;

    case ScreenId::Global:
      lv_screen_load(ui_ScreenGlobal);
      break;

    case ScreenId::AudioDSP:
      lv_screen_load(ui_ScreenAudioDSP);
      break;

    case ScreenId::Wifi:
      lv_screen_load(ui_ScreenWifi);
      break;

    case ScreenId::System:
      lv_screen_load(ui_ScreenSystem);
      break;
  }
}

static void navToScreenCb(lv_event_t* e)
{
  ScreenId s = (ScreenId)(uintptr_t)lv_event_get_user_data(e);
  showScreen(s);
}

// ====================== Navigation Callbacks ======================

static void navMainToGlobal(lv_event_t* e)
{
  (void)e;
  showScreen(ScreenId::Global);
}

static void navGlobalToAudio(lv_event_t* e)
{
  (void)e;
  showScreen(ScreenId::AudioDSP);
}

static void navGlobalToWifi(lv_event_t* e)
{
  (void)e;
  showScreen(ScreenId::Wifi);
}

static void navGlobalToSystem(lv_event_t* e)
{
  (void)e;
  showScreen(ScreenId::System);
}

static void navBackToMain(lv_event_t* e)
{
  (void)e;
  showScreen(ScreenId::Main);
}

static void navBackToGlobal(lv_event_t* e)
{
  (void)e;
  showScreen(ScreenId::Global);
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

//====================== MQTT ======================
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

  WiFi.begin(g_cfgSsid.c_str(), g_cfgPass.c_str());
  g_connectInProgress = true;
  g_connectStartMs    = millis();
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
    //parseTeensyLine(line);
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


// ====================== UI Navigation Wiring ======================

// Main screen -> Global Settings
makeClickable(ui_contFootorClick, navMainToGlobal);

// Global screen menu buttons
makeClickable(ui_contBtnAudio, navGlobalToAudio);
makeClickable(ui_contBtnWIFI, navGlobalToWifi);
makeClickable(ui_contBtnSystem, navGlobalToSystem);

// Back buttons
makeClickable(ui_btnBackAudioDSP, navBackToGlobal);
makeClickable(ui_imgBackArrow5,   navBackToGlobal);

makeClickable(ui_btnBackWIFI,     navBackToGlobal);
makeClickable(ui_imgBackArrow2,   navBackToGlobal);

makeClickable(ui_btnBackSystem,   navBackToGlobal);
makeClickable(ui_imgBackArrow1,   navBackToGlobal);

makeClickable(ui_btnBackGBLSettings,   navBackToMain);
makeClickable(ui_imgBackArrow,   navBackToMain);
  
  // ---------------------- Bind SETTINGS (WiFi + Globals) ----------------------

  uiSetWifiStatus("Select a network");
  
  
  wifiAutoConnectIfSaved();

 
   // ---------------------- Start Home + WiFi scan prompt ----------------------
  showScreen(ScreenId::Main);



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
  


#if ENABLE_ENCODER_NAV
  if (read_button_pressed_edge()) {
    showScreen(ScreenId::Main);
  }
#endif

  static uint32_t lastWifiPoll = 0;
  if (now - lastWifiPoll >= 250) {
    lastWifiPoll = now;
    //wifiPollAndUpdateUi();
  }
  mqttLoop(now);
  //wifiScanPoll();
  wifiLedUpdate(now);

  static uint32_t lastPingMs = 0;
if (g_mqtt.connected() && millis() - lastPingMs > 1000) {
  lastPingMs = millis();
  teensySendLine("PING");
}

  delay(5);
}