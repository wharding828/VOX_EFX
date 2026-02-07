#ifndef LV_CONF_H
#define LV_CONF_H

/* Tell LVGL we are providing this config */
#define LV_CONF_INCLUDE_SIMPLE 1

/* Color depth: MUST match your flush (RGB565) */
#define LV_COLOR_DEPTH 16

/* Use standard malloc/free */
#define LV_MEM_CUSTOM 0

/* Enable needed features (safe defaults) */
#define LV_USE_LOG 0
#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0

/* Tick source is provided by your code (lv_tick_inc) */
#define LV_TICK_CUSTOM 0

/* Optional: reduce CPU */
#define LV_USE_GPU_STM32_DMA2D 0
#define LV_USE_GPU_NXP_PXP 0
#define LV_USE_GPU_NXP_VG_LITE 0

/* Fonts: keep default + one decent size */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_32 1
#define LV_FONT_DEFAULT &lv_font_montserrat_20

/* Enable what SquareLine commonly uses */
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_IMG 1

/* If you use PNG/JPG later, enable decoders then. For now keep off. */
#define LV_USE_BMP 1
#define LV_USE_SJPG 0
#define LV_USE_PNG 0

#endif /*LV_CONF_H*/
