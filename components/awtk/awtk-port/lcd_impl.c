#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"

#include "tkc/str.h"
#include "tkc/mem.h"
#include "tkc/time_now.h"
#include "base/main_loop.h"
#include "lcd/lcd_mem_bgr565.h"

#define I2C_MASTER_SCL_IO 9 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM \
  0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define GPIO_INPUT_IO_4 4
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_4

static const char* TAG = "example";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (18 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT -1
#define EXAMPLE_PIN_NUM_HSYNC 46
#define EXAMPLE_PIN_NUM_VSYNC 3
#define EXAMPLE_PIN_NUM_DE 5
#define EXAMPLE_PIN_NUM_PCLK 7
#define EXAMPLE_PIN_NUM_DATA0 14   // B3
#define EXAMPLE_PIN_NUM_DATA1 38   // B4
#define EXAMPLE_PIN_NUM_DATA2 18   // B5
#define EXAMPLE_PIN_NUM_DATA3 17   // B6
#define EXAMPLE_PIN_NUM_DATA4 10   // B7
#define EXAMPLE_PIN_NUM_DATA5 39   // G2
#define EXAMPLE_PIN_NUM_DATA6 0    // G3
#define EXAMPLE_PIN_NUM_DATA7 45   // G4
#define EXAMPLE_PIN_NUM_DATA8 48   // G5
#define EXAMPLE_PIN_NUM_DATA9 47   // G6
#define EXAMPLE_PIN_NUM_DATA10 21  // G7
#define EXAMPLE_PIN_NUM_DATA11 1   // R3
#define EXAMPLE_PIN_NUM_DATA12 2   // R4
#define EXAMPLE_PIN_NUM_DATA13 42  // R5
#define EXAMPLE_PIN_NUM_DATA14 41  // R6
#define EXAMPLE_PIN_NUM_DATA15 40  // R7
#define EXAMPLE_PIN_NUM_DISP_EN -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 800
#define EXAMPLE_LCD_V_RES 480

#define EXAMPLE_LCD_NUM_FB 1

esp_lcd_touch_handle_t s_touch_handle = NULL;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
  int i2c_master_port = I2C_MASTER_NUM;

  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };

  i2c_param_config(i2c_master_port, &conf);

  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

void gpio_init(void) {
  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // bit mask of the pins, use GPIO6 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  // set as input mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // enable pull-up mode
  //  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
}

// #define CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM 1

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel,
                                   const esp_lcd_rgb_panel_event_data_t* event_data,
                                   void* user_data) {
  BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
    xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
  }
#endif
  return high_task_awoken == pdTRUE;
}

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
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  ESP_LOGI(TAG, "Create semaphores");
  sem_vsync_end = xSemaphoreCreateBinary();
  assert(sem_vsync_end);
  sem_gui_ready = xSemaphoreCreateBinary();
  assert(sem_gui_ready);
#endif

  ESP_LOGI(TAG, "Install RGB LCD panel driver");
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_rgb_panel_config_t panel_config = {
      .data_width = 16,  // RGB565 in parallel mode, thus 16bit in width
      .psram_trans_align = 64,
      .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
      .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
      .clk_src = LCD_CLK_SRC_DEFAULT,
      .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
      .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
      .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
      .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
      .de_gpio_num = EXAMPLE_PIN_NUM_DE,
      .data_gpio_nums =
          {
              EXAMPLE_PIN_NUM_DATA0,
              EXAMPLE_PIN_NUM_DATA1,
              EXAMPLE_PIN_NUM_DATA2,
              EXAMPLE_PIN_NUM_DATA3,
              EXAMPLE_PIN_NUM_DATA4,
              EXAMPLE_PIN_NUM_DATA5,
              EXAMPLE_PIN_NUM_DATA6,
              EXAMPLE_PIN_NUM_DATA7,
              EXAMPLE_PIN_NUM_DATA8,
              EXAMPLE_PIN_NUM_DATA9,
              EXAMPLE_PIN_NUM_DATA10,
              EXAMPLE_PIN_NUM_DATA11,
              EXAMPLE_PIN_NUM_DATA12,
              EXAMPLE_PIN_NUM_DATA13,
              EXAMPLE_PIN_NUM_DATA14,
              EXAMPLE_PIN_NUM_DATA15,
          },
      .timings =
          {
              .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
              .h_res = EXAMPLE_LCD_H_RES,
              .v_res = EXAMPLE_LCD_V_RES,
              // The following parameters should refer to LCD spec
              .hsync_back_porch = 8,
              .hsync_front_porch = 8,
              .hsync_pulse_width = 4,
              .vsync_back_porch = 16,
              .vsync_front_porch = 16,
              .vsync_pulse_width = 4,
              .flags.pclk_active_neg = true,
          },
      .flags.fb_in_psram = true,  // allocate frame buffer in PSRAM
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

  ESP_LOGI(TAG, "Register event callbacks");
  esp_lcd_rgb_panel_event_callbacks_t cbs = {
      .on_vsync = example_on_vsync_event,
  };
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL));

  ESP_LOGI(TAG, "Initialize RGB LCD panel");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
  ESP_LOGI(TAG, "Turn on LCD backlight");
  gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized successfully");
  gpio_init();

  uint8_t write_buf = 0x01;
  i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1,
                             I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  // Reset the touch screen. It is recommended that you reset the touch screen before using it.
  write_buf = 0x2C;
  i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1,
                             I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  esp_rom_delay_us(100 * 1000);

  gpio_set_level(GPIO_INPUT_IO_4, 0);
  esp_rom_delay_us(100 * 1000);

  write_buf = 0x2E;
  i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1,
                             I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  esp_rom_delay_us(200 * 1000);

  esp_lcd_touch_handle_t tp = NULL;
  esp_lcd_panel_io_handle_t tp_io_handle = NULL;

  ESP_LOGI(TAG, "Initialize I2C");

  esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

  ESP_LOGI(TAG, "Initialize touch IO (I2C)");
  /* Touch IO handle */
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config,
                                           &tp_io_handle));
  esp_lcd_touch_config_t tp_cfg = {
      .x_max = EXAMPLE_LCD_V_RES,
      .y_max = EXAMPLE_LCD_H_RES,
      .rst_gpio_num = -1,
      .int_gpio_num = -1,
      .flags =
          {
              .swap_xy = 0,
              .mirror_x = 0,
              .mirror_y = 0,
          },
  };
  /* Initialize touch */
  ESP_LOGI(TAG, "Initialize touch controller GT911");
  ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

  void* online_buff = NULL;
  void* offline_buff = TKMEM_ALLOC(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 2);

  ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 1, &online_buff));

  /*AWTK create lcd */

  lcd = lcd_mem_bgr565_create_single_fb(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, offline_buff);
  return_value_if_fail(lcd != NULL, NULL);

  lcd->impl_data = panel_handle;
  lcd->flush = lcd_esp_flush;
  s_touch_handle = tp;

  return lcd;
}

ret_t platform_disaptch_key_events(main_loop_t* loop) {
  return RET_OK;
}

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
