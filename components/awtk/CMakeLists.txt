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
