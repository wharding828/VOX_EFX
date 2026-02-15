#include <Arduino.h>
#include <TFT_eSPI.h>
#include <lvgl.h>

// ---------- Display ----------
static constexpr int DISP_HOR = 480;
static constexpr int DISP_VER = 320;
static constexpr int TFT_ROT  = 1;

// Partial buffer (safe on ESP32)
static constexpr uint32_t BUF_LINES  = 20;
static constexpr uint32_t BUF_PIXELS = DISP_HOR * BUF_LINES;

static TFT_eSPI tft;
static lv_display_t* disp = nullptr;
static lv_color_t buf1[BUF_PIXELS];

// Keep handles so timer callback can update safely
static lv_obj_t* g_bar = nullptr;

static void my_flush_cb(lv_display_t* d, const lv_area_t* a, uint8_t* px_map)
{
  (void)d;
  const int32_t w = (a->x2 - a->x1 + 1);
  const int32_t h = (a->y2 - a->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(a->x1, a->y1, w, h);
  tft.pushPixels((uint16_t*)px_map, (uint32_t)w * (uint32_t)h);
  tft.endWrite();

  lv_display_flush_ready(d);
}

static void bar_timer_cb(lv_timer_t* t)
{
  (void)t;
  if (!g_bar) return;

  static int vv = 0;
  vv = (vv + 2) % 101;
  lv_bar_set_value(g_bar, vv, LV_ANIM_OFF);
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  tft.init();
  tft.setRotation(TFT_ROT);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);

  lv_init();

  disp = lv_display_create(DISP_HOR, DISP_VER);
  lv_display_set_flush_cb(disp, my_flush_cb);
  lv_display_set_buffers(
    disp,
    buf1,
    nullptr,
    BUF_PIXELS * sizeof(lv_color_t),
    LV_DISPLAY_RENDER_MODE_PARTIAL
  );

  // --- Simple UI (no SquareLine) ---
  lv_obj_t* label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "LVGL BASELINE OK");
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);

  g_bar = lv_bar_create(lv_screen_active());
  lv_obj_set_size(g_bar, 420, 24);
  lv_obj_align(g_bar, LV_ALIGN_TOP_MID, 0, 50);
  lv_bar_set_range(g_bar, 0, 100);
  lv_bar_set_value(g_bar, 0, LV_ANIM_OFF);

  lv_timer_create(bar_timer_cb, 50, nullptr);
}

void loop()
{
  static uint32_t last = millis();
  uint32_t now = millis();
  lv_tick_inc(now - last);
  last = now;

  lv_timer_handler();
  delay(5);
}
