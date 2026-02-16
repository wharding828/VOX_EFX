#ifndef LV_CONF_H
#define LV_CONF_H

/*====================
   COLOR SETTINGS
 *====================*/

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0   /* 0 = RGB565 (correct for TFT_eSPI + setSwapBytes(true)) */

/*====================
   MEMORY SETTINGS
 *====================*/

/* Use internal allocator */
#define LV_MEM_CUSTOM 0

/* Memory size (only used if LV_MEM_CUSTOM == 0) */
#define LV_MEM_SIZE (48U * 1024U)   /* 48 KB is safe for ESP32 partial buffer */

/*====================
   HAL SETTINGS
 *====================*/

#define LV_TICK_CUSTOM 0

/*====================
   FEATURE CONFIGURATION
 *====================*/

#define LV_USE_LOG 0

/*====================
   FONT CONFIGURATION
 *====================*/

#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_28 1

/* Required by SquareLine typically */
#define LV_USE_IMG 1
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_SLIDER 1
#define LV_USE_BAR 1
#define LV_USE_SWITCH 1

/*====================
   THEMES
 *====================*/

#define LV_USE_THEME_DEFAULT 1

#endif /*LV_CONF_H*/
