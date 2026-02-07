#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>

extern "C" {
  #include <lvgl.h>
  #include "ui.h"   // SquareLine export (lib/squareline_ui)
}

// ====================== Display settings ======================
static constexpr int DISP_HOR = 480;
static constexpr int DISP_VER = 320;
static constexpr int TFT_ROT  = 1;   // landscape for most ST7796 setups

// Reduce RAM usage: one partial buffer
static constexpr uint32_t BUF_LINES  = 20;                 // 10–20 is safe on ESP32
static constexpr uint32_t BUF_PIXELS = DISP_HOR * BUF_LINES;

TFT_eSPI tft;
static lv_display_t* g_disp = nullptr;
static lv_color_t g_buf1[BUF_PIXELS];                      // ONE buffer (fits DRAM)

// ====================== LVGL flush callback ======================
static void my_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
  const int32_t x1 = area->x1;
  const int32_t y1 = area->y1;
  const int32_t x2 = area->x2;
  const int32_t y2 = area->y2;

  const int32_t w = (x2 - x1 + 1);
  const int32_t h = (y2 - y1 + 1);

  tft.startWrite();
  tft.setAddrWindow((int)x1, (int)y1, (int)w, (int)h);

  // LVGL RGB565 buffer -> push as 16-bit pixels
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
  if ((td & 0x0F) == 0) return true; // no touch, but comm OK

  uint8_t b[4];
  if (!ftReadRegs(REG_P1_XH, b, 4)) return false;

  uint16_t x = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
  uint16_t y = ((uint16_t)(b[2] & 0x0F) << 8) | b[3];

  // Mapping guess for rotation(1): many panels report portrait coords
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

// ====================== LVGL tick ======================
static uint32_t last_ms = 0;

// Set to 1 if you want to force a simple LVGL test screen *instead* of SquareLine UI
#define LVGL_FORCE_TEST_SCREEN  0

void setup()
{
  Serial.begin(115200);
  delay(100);

  // --- TFT ---
  tft.init();
  tft.setRotation(TFT_ROT);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);

  // --- LVGL ---
  lv_init();

  // Create LVGL display and attach flush
  g_disp = lv_display_create(DISP_HOR, DISP_VER);
  lv_display_set_flush_cb(g_disp, my_flush_cb);

  // LVGL 9.x (PlatformIO build): 5-arg lv_display_set_buffers
  lv_display_set_buffers(
    g_disp,
    g_buf1,
    nullptr,
    BUF_PIXELS * sizeof(lv_color_t),
    LV_DISPLAY_RENDER_MODE_PARTIAL
  );

#if ENABLE_FT6336U_TOUCH
  // --- Touch I2C ---
  Wire.begin(); // if needed: Wire.begin(SDA, SCL);
  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touch_read_cb);
#endif

#if LVGL_FORCE_TEST_SCREEN
  // --- LVGL draw test (bypasses SquareLine) ---
  lv_obj_t* scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_obj_t* lbl = lv_label_create(scr);
  lv_label_set_text(lbl, "LVGL OK");
  lv_obj_center(lbl);
#else
  // --- SquareLine UI ---
  ui_init();
#endif

  last_ms = millis();
}

void loop()
{
  // LVGL tick increment
  uint32_t now  = millis();
  uint32_t diff = now - last_ms;
  last_ms = now;

  lv_tick_inc(diff);
  lv_timer_handler();

  delay(5);
}
