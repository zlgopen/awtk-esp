# AWTK 在树莓派 pico 上的移植笔记

## 1. 配置文件 (awtk_config.h)

这块板子 [esps3](https://www.waveshare.net/wiki/ESP32-S3-Touch-LCD-4.3) 和 stm32f429 的配置差不多，内存可以提供双帧缓冲，可支持 png 和矢量字体。

我们在 [awtk-stm32f429igtx 的配置](https://github.com/zlgopen/awtk-stm32f429igtx-raw/blob/master/awtk-port/awtk_config.h) 基础稍作修改即可。

```c
/**
 * 使用标准的 malloc 函数。
 *
 */
#define HAS_STD_MALLOC 1

/**
 * 支持 printf 函数。
 *
 */
#define HAS_STDIO 1

/**
 * 包含 FreeRTOS 的头文件时带 freertos 目录。
 *
 */
#define WITH_FREE_RTOS_DIR 1
```

## 2. LCD 的实现

> 把 LVGL 例子中的代码拷贝过来，稍作修改即可。

```c
static ret_t lcd_esp_flush(lcd_t* lcd) {
  lcd_mem_t* mem = (lcd_mem_t*)lcd;
  void* offline_buf = lcd_mem_get_offline_fb(mem);
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lcd->impl_data;

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  xSemaphoreGive(sem_gui_ready);
  xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif

  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, lcd->w, lcd->h, offline_buf);

  return RET_OK;
}

lcd_t* lcd_impl_create(wh_t w, wh_t h) {
  lcd_t* lcd = NULL;
  ...
  /*把 LVGL 例子中创建 LCD 的代码拷贝过来*/
  ...
  lcd = lcd_mem_bgr565_create_single_fb(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, offline_buff);
  return_value_if_fail(lcd != NULL, NULL);

  lcd->impl_data = panel_handle;
  lcd->flush = lcd_esp_flush;
  s_touch_handle = tp;

  return lcd;
}  
```

> 如果使用双帧缓冲，会出现严重的撕裂感，所以我们只使用单帧缓冲，在 flush 函数中再刷新到屏幕上。

```c

## 3. 触摸屏的实现

把 LVGL 例子中的代码拷贝过来，稍作修改即可。不过这里有点小问题，即使一直按住，touchpad_pressed 仍然会变成为 false，会不断上报 down/up 事件。

```c

static int s_prev_x = 0;
static int s_prev_y = 0;
static bool s_prev_pressed = FALSE;

ret_t platform_disaptch_touch_events(main_loop_t* loop) {
  uint16_t touchpad_x[1] = {0};
  uint16_t touchpad_y[1] = {0};
  uint8_t touchpad_cnt = 0;
  uint64_t now = time_now_ms();
  /* Read touch controller data */
  if (esp_lcd_touch_read_data(s_touch_handle) != ESP_OK) {
    return RET_OK;
  }

  /* Get coordinates */
  bool touchpad_pressed =
      esp_lcd_touch_get_coordinates(s_touch_handle, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

  if (touchpad_pressed && touchpad_cnt > 0) {
    int x = touchpad_x[0];
    int y = touchpad_y[0];

    s_prev_x = x;
    s_prev_y = y;
    main_loop_post_pointer_event(loop, TRUE, x, y);
    printf("down x: %d, y: %d\n", x, y);
    s_prev_pressed = TRUE;
  } else if (s_prev_pressed) {
    /*FIXME: 很奇怪 即使一直按住，touchpad_pressed 仍然为 false */
    main_loop_post_pointer_event(loop, FALSE, s_prev_x, s_prev_y);
    printf("up time: %lld x: %d, y: %d touchpad_pressed: %d touchpad_cnt: %d\n", now, s_prev_x,
           s_prev_y, touchpad_pressed, touchpad_cnt);
    s_prev_pressed = FALSE;
  }

  return RET_OK;
}

```

## 4. 平台函数

平台函数只需要实现 `get_time_ms64` 和 `sleep_ms` 即可。

```c
uint64_t get_time_ms64(void) {
  return esp_timer_get_time() / 1000;
}

void sleep_ms(uint32_t ms) {
  rtos_delay(ms);
}

```

## 5. 编译脚本

* cmake
  
esp 使用 cmake 编译，我们需要在 CMakeLists.txt 中添加 AWTK 的源码和头文件。 awtk_mobile_common.mk 本来是给 Android/iOS/HarmonyOS/Web 用的，在这里也可以直接使用，省去不少功夫。

```sh
include(${CMAKE_CURRENT_LIST_DIR}/awtk/scripts/awtk_mobile_common.mk)

set(AWTK_FLAGS " -w ")
set(CMAKE_C_FLAGS "${AWTK_FLAGS} ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${AWTK_FLAGS} ${CMAKE_CXX_FLAGS}")

set(AWTK_ALL_SOURCES_FILES
  ${AWTK_BASE_SOURCE_FILES}
  ${AWTK_RAW_COMMON_SOURCE_FILES}
  ${AWTK_PORT_SOURCE_FILES}
  ${AWTK_PORT_FREERTOS_SOURCE_FILES}
  ${AWTK_VGCANVAS_SOFT_SOURCE_FILES}
  )

idf_component_register(
  SRCS ${AWTK_ALL_SOURCES_FILES}
  INCLUDE_DIRS ${AWTK_COMMON_INCLUDES} ${CMAKE_CURRENT_LIST_DIR}/awtk-port ${AWTK_NANOVG_AGGE_INCLUDES}
  REQUIRES "driver" "esp_lcd" "esp_timer" "esp_system" "espressif__esp_lcd_touch_gt911" "espressif__esp_lcd_touch" "freertos"
)
```

* Kconfig

```sh
menu "AWTK GUI"
    config HAS_AWTK_CONFIG
        bool "has awtk_config.h"
        default y

endmenu
```

## 默认配置修改

* 修改分区表

> 默认文件位置为 C://Espressif/frameworks/esp-idf-v5.3.2/components/partition_table/partitions_singleapp.csv

```csv
# Name,   Type, SubType, Offset,  Size, Flags
# Note: if you have increased the bootloader size, make sure to update the offsets to avoid overlap
nvs,      data, nvs,     ,        0x6000,
phy_init, data, phy,     ,        0x1000,
factory,  app,  factory, ,        6M,
```

* 修改主线程栈大小

```c
CONFIG_ESP_MAIN_TASK_STACK_SIZE=64000
```

* 修改 flash 分区大小

```c
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="8MB"
```

* 其它配置请参考 [开发板的配置](https://www.waveshare.net/wiki/ESP32-S3-Touch-LCD-4.3)
  




