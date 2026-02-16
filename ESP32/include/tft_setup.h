#pragma once

// ============================================================
// TFT_eSPI User Setup for VOX_EFX
// Target: ST7796 480x320 SPI
// ============================================================

// --- Driver selection ---
#define ST7796_DRIVER

// --- Display resolution (some TFT_eSPI examples use these) ---
#define TFT_WIDTH  480
#define TFT_HEIGHT 320

// --- SPI pins (ESP32 VSPI default-ish, with your CS/DC/RST) ---
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC   16
#define TFT_RST  17

// --- SPI frequencies ---
#define SPI_FREQUENCY       10000000   // 10 MHz write
#define SPI_READ_FREQUENCY   5000000   // 5 MHz read (if used)

// --- Fonts / rendering options (matches your current build_flags) ---
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

// NOTE: If you ever use a touch controller via TFT_eSPI (XPT2046, etc),
// you would define touch pins/settings here as well.