#if 1
#ifndef LV_CONF_H
#define LV_CONF_H

/* Only the options that differ from the LVGL defaults are listed here.
 * lv_conf_internal.h fills in everything else. */

#define LV_COLOR_DEPTH 16
/* 0 keeps lv_color_t byte compatible with lgfx::rgb565_t, which is what the
 * flush callback hands to M5GFX. */
#define LV_COLOR_16_SWAP 0

#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (48U * 1024U)

/* Without a tick source no LVGL timer ever fires, so nothing draws and the
 * touch input device is never polled. */
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

#define LV_DISP_DEF_REFR_PERIOD 20
#define LV_INDEV_DEF_READ_PERIOD 20

#define LV_USE_LOG 0
#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_STYLE 0

#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_SLIDER 1
#define LV_USE_ARC 0
/* lv_spinner defaults to on and hard-errors without lv_arc, so it has to
 * be turned off explicitly alongside it. */
#define LV_USE_SPINNER 0

#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1

#define LV_USE_FLEX 1
#define LV_USE_GRID 1
#define LV_USE_PERF_MONITOR 0
#define LV_USE_USER_DATA 1

#endif
#endif
