#include "main_loop/main_loop_simple.h"

extern lcd_t* lcd_impl_create(wh_t w, wh_t h);
extern ret_t platform_disaptch_key_events(main_loop_t* loop);
extern ret_t platform_disaptch_touch_events(main_loop_t* loop);

static ret_t platform_disaptch_input(main_loop_t* loop) {
  platform_disaptch_key_events(loop);
  platform_disaptch_touch_events(loop);

  return RET_OK;
}

lcd_t* platform_create_lcd(wh_t w, wh_t h) {
  return lcd_impl_create(w, h);
}

#include "main_loop/main_loop_raw.inc"
