#include "tkc/mem.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

ret_t platform_prepare(void) {
  return RET_OK;
}

uint64_t get_time_ms64(void) {
  return esp_timer_get_time() / 1000;
}

void sleep_ms(uint32_t ms) {
  rtos_delay(ms);
}
