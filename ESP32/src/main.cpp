// ============================================================
// VOX_EFX - ESP32 UI (SquareLine + LVGL 9) NAV TEST (CURRENT UI)
//  - Home shows ui_contNavigation
//  - Clicking nav shows the corresponding panel (ui_pnl*)
//  - Back buttons return to Home
//  - WiFi status uses 3 SquareLine image widgets (Disconnected/Connecting/Connected)
//  - Onboard LED behavior:
//      * Disconnected  = slow blink
//      * Connecting    = fast blink
//      * Connected     = solid ON
//  - Optional FT6336U touch over I2C (no external library)
//  - Encoder + Serial fallback navigation (for testing)
// ============================================================

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <WiFi.h>

extern "C" {
  #include <lvgl.h>
  #include "ui.h"
}

// ====================== Display settings ======================
static constexpr int DISP_HOR = 480;
static constexpr int DISP_VER = 320;
static constexpr int TFT_ROT  = 1;

// LVGL draw buffer (single partial buffer to save DRAM)
static constexpr uint32_t BUF_LINES  = 8;
static constexpr uint32_t BUF_PIXELS = DISP_HOR * BUF_LINES;

TFT_eSPI tft;
static lv_display_t* g_disp = nullptr;
static lv_color_t g_buf1[BUF_PIXELS];

// ====================== Encoder (optional) ======================
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

// ====================== WiFi ======================
enum class WifiState : uint8_t { Disconnected, Connecting, Connected };
static WifiState g_wifiState = WifiState::Disconnected;

#define PIN_LED 2
static const char* WIFI_SSID = "IoTWifi";
static const char* WIFI_PSK  = "0828196700";

static uint32_t g_wifiLastAttemptMs = 0;
static bool     g_wifiConnected     = false;

// ====================== WiFi glow style (applied to the "connected" icon) ======================
static lv_style_t g_styleWifiGlow;
static bool g_wifiStyleInited = false;

static void initWifiGlowStyle()
{
  if (g_wifiStyleInited) return;
  g_wifiStyleInited = true;

  lv_style_init(&g_styleWifiGlow);
  lv_style_set_shadow_width(&g_styleWifiGlow, 18);
  lv_style_set_shadow_spread(&g_styleWifiGlow, 2);
  lv_style_set_shadow_color(&g_styleWifiGlow, lv_color_hex(0x00FF66));
  lv_style_set_shadow_opa(&g_styleWifiGlow, LV_OPA_80);
}

static void setWifiGlow(bool connected)
{
  // Apply glow only to the CONNECTED icon widget (looks cleaner)
  if (!ui_imgWifiConnected) return;

  //if (connected) lv_obj_add_style(ui_imgWifiConnected, &g_styleWifiGlow, 0);
  //else           lv_obj_remove_style(ui_imgWifiConnected, &g_styleWifiGlow, 0);
}

// ====================== WiFi icon state (3 widgets: show one, hide the others) ======================
static void setWifiState(WifiState state)
{
  // These are your 3 SquareLine image objects under contStatusRight
  if (!ui_imgWifiDisconnected || !ui_imgWifiConnecting || !ui_imgWifiConnected) return;

  // Hide all
  lv_obj_add_flag(ui_imgWifiDisconnected, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(ui_imgWifiConnecting,   LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(ui_imgWifiConnected,    LV_OBJ_FLAG_HIDDEN);

  // Show one
  switch (state) {
    case WifiState::Disconnected: lv_obj_remove_flag(ui_imgWifiDisconnected, LV_OBJ_FLAG_HIDDEN); break;
    case WifiState::Connecting:   lv_obj_remove_flag(ui_imgWifiConnecting,   LV_OBJ_FLAG_HIDDEN); break;
    case WifiState::Connected:    lv_obj_remove_flag(ui_imgWifiConnected,    LV_OBJ_FLAG_HIDDEN); break;
  }

  g_wifiState = state;
}

// ====================== LED State Machine ======================
static uint32_t g_ledLastToggleMs = 0;
static bool     g_ledLevel = false;

static void wifiLedUpdate(uint32_t nowMs)
{
  // Connected = solid ON
  if (g_wifiState == WifiState::Connected) {
    g_ledLevel = true;
    digitalWrite(PIN_LED, HIGH);
    return;
  }

  // Blink rate by state
  uint32_t periodMs = 1000; // Disconnected = slow blink
  if (g_wifiState == WifiState::Connecting) periodMs = 250; // Connecting = fast blink

  if (nowMs - g_ledLastToggleMs >= (periodMs / 2)) {
    g_ledLastToggleMs = nowMs;
    g_ledLevel = !g_ledLevel;
    digitalWrite(PIN_LED, g_ledLevel ? HIGH : LOW);
  }
}

// ====================== Pages ======================
enum class Page : uint8_t { Home, Delay, Reverb, Comp, EQ, Settings };
static Page g_page = Page::Home;

static constexpr Page PAGE_ORDER[] = { Page::Home, Page::Delay, Page::Reverb, Page::Comp, Page::EQ, Page::Settings };
static constexpr int PAGE_ORDER_COUNT = (int)(sizeof(PAGE_ORDER) / sizeof(PAGE_ORDER[0]));

static int pageIndex(Page p)
{
  for (int i = 0; i < PAGE_ORDER_COUNT; i++) {
    if (PAGE_ORDER[i] == p) return i;
  }
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

  switch (p) {
    case Page::Home:     if (ui_contNavigation) lv_obj_remove_flag(ui_contNavigation, LV_OBJ_FLAG_HIDDEN); break;
    case Page::Delay:    if (ui_pnlDelay)       lv_obj_remove_flag(ui_pnlDelay, LV_OBJ_FLAG_HIDDEN); break;
    case Page::Reverb:   if (ui_pnlReverb)      lv_obj_remove_flag(ui_pnlReverb, LV_OBJ_FLAG_HIDDEN); break;
    case Page::Comp:     if (ui_pnlComp)        lv_obj_remove_flag(ui_pnlComp, LV_OBJ_FLAG_HIDDEN); break;
    case Page::EQ:       if (ui_pnlEQ)          lv_obj_remove_flag(ui_pnlEQ, LV_OBJ_FLAG_HIDDEN); break;
    case Page::Settings: if (ui_pnlSettings)    lv_obj_remove_flag(ui_pnlSettings, LV_OBJ_FLAG_HIDDEN); break;
  }
}

static void showNextPrev(int dir)
{
  int idx = pageIndex(g_page);
  idx += dir;
  if (idx < 0) idx = PAGE_ORDER_COUNT - 1;
  if (idx >= PAGE_ORDER_COUNT) idx = 0;
  showPage(PAGE_ORDER[idx]);
}

// ====================== LVGL event helpers ======================
static void makeClickable(lv_obj_t* o, lv_event_cb_t cb, void* user = nullptr)
{
  if (!o) return;
  lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_flag(o, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_add_event_cb(o, cb, LV_EVENT_CLICKED, user);
}

// ====================== LVGL callbacks ======================
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
  if ((td & 0x0F) == 0) return true;

  uint8_t b[4];
  if (!ftReadRegs(REG_P1_XH, b, 4)) return false;

  uint16_t x = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
  uint16_t y = ((uint16_t)(b[2] & 0x0F) << 8) | b[3];

  int rawX = (int)x;
  int rawY = (int)y;

  sx = constrain(rawY, 0, DISP_HOR - 1);
  sy = constrain(DISP_VER - rawX, 0, DISP_VER - 1);
  pressed = true;
  return true;
}

static void my_touch_read_cb(lv_indev_t* indev, lv_indev_data_t* data)
{
  (void)indev;

  int x = 0, y = 0;
  bool down = false;

  bool ok = ftReadTouch(x, y, down);
  if (!ok) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  data->state   = down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  data->point.x = x;
  data->point.y = y;
}
#endif

// ====================== WiFi connect/update ======================
static void wifiStartIfNeeded()
{
  if (WiFi.status() == WL_CONNECTED) return;

  const uint32_t now = millis();
  if (now - g_wifiLastAttemptMs < 5000) return;
  g_wifiLastAttemptMs = now;

  Serial.println("WiFi: begin()");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  setWifiState(WifiState::Connecting);
}

static void wifiPollAndUpdateUi()
{
  wl_status_t s = WiFi.status();
  bool connected = (s == WL_CONNECTED);

  if (connected != g_wifiConnected) {
    g_wifiConnected = connected;

    if (connected) {
      Serial.printf("WiFi: connected IP=%s\n", WiFi.localIP().toString().c_str());
      setWifiState(WifiState::Connected);
    } else {
      Serial.println("WiFi: disconnected");
      setWifiState(WifiState::Disconnected);
    }

    setWifiGlow(connected);
  }

  if (!connected) wifiStartIfNeeded();
}

// ====================== Serial nav fallback ======================
static void pollSerialNav()
{
  while (Serial.available()) {
    char k = (char)Serial.read();
    if      (k == 'h') showPage(Page::Home);
    else if (k == 'd') showPage(Page::Delay);
    else if (k == 'r') showPage(Page::Reverb);
    else if (k == 'c') showPage(Page::Comp);
    else if (k == 'e') showPage(Page::EQ);
    else if (k == 's') showPage(Page::Settings);
    else if (k == 'n') showNextPrev(+1);
    else if (k == 'p') showNextPrev(-1);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // quick boot blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH); delay(100);
    digitalWrite(PIN_LED, LOW);  delay(100);
  }

#if ENABLE_ENCODER_NAV
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  g_lastAB = ((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
#endif

  // --- TFT ---
  tft.init();
  tft.setRotation(TFT_ROT);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);

  // --- LVGL ---
  lv_init();

  g_disp = lv_display_create(DISP_HOR, DISP_VER);
  lv_display_set_flush_cb(g_disp, my_flush_cb);
  lv_display_set_buffers(
    g_disp,
    g_buf1,
    nullptr,
    BUF_PIXELS * sizeof(lv_color_t),
    LV_DISPLAY_RENDER_MODE_PARTIAL
  );

#if ENABLE_FT6336U_TOUCH
  Wire.begin();
  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touch_read_cb);
#endif

  // --- SquareLine UI ---
  ui_init();

  // WiFi visuals must be set AFTER ui_init()
  initWifiGlowStyle();
  setWifiGlow(false);
  setWifiState(WifiState::Disconnected);

  // -------- Navigation bindings --------
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

  // -------- Back bindings --------
  makeClickable(ui_btnBackComp,           backToHomeCb);
  makeClickable(ui_ImageBtnBackComp,      backToHomeCb);

  makeClickable(ui_btnBackDelay,          backToHomeCb);
  makeClickable(ui_ImageBtnBackDelay,     backToHomeCb);

  makeClickable(ui_btnBackReverb,         backToHomeCb);
  makeClickable(ui_ImageBtnBackReverb,    backToHomeCb);

  makeClickable(ui_btnBackEQ,             backToHomeCb);
  makeClickable(ui_ImageBtnBackEQ,        backToHomeCb);

  makeClickable(ui_btnBackSettings,       backToHomeCb);
  makeClickable(ui_ImageBtnBackSettings,  backToHomeCb);

  // Start on Home
  showPage(Page::Home);

  // Start WiFi (non-blocking)
  wifiStartIfNeeded();

  Serial.println("NAV TEST READY");
  Serial.println("Serial keys: h d r c e s | n/p next/prev");
#if ENABLE_ENCODER_NAV
  Serial.println("Encoder: rotate next/prev page, press = home");
#endif
}

void loop()
{
  static uint32_t last_ms = millis();
  uint32_t now  = millis();
  uint32_t diff = now - last_ms;
  last_ms = now;

  lv_tick_inc(diff);
  lv_timer_handler();

#if ENABLE_ENCODER_NAV
  int step = read_encoder_step();
  if (step != 0) showNextPrev(step);
  if (read_button_pressed_edge()) showPage(Page::Home);
#endif

  pollSerialNav();

  static uint32_t lastWifiPoll = 0;
  if (now - lastWifiPoll >= 250) {
    lastWifiPoll = now;
    wifiPollAndUpdateUi();
  }

  // ✅ Keep LED updated continuously based on current WiFi state
  wifiLedUpdate(now);

  delay(5);
}