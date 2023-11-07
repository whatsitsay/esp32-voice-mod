#ifndef __GPIO_BUTTON__
#define __GPIO_BUTTON__

#include "driver/gpio.h"
#include "esp_timer.h"

#define DEBOUNCE_TIMEOUT_US (15000) // 15 ms

typedef struct {
  gpio_num_t gpio_num;
  esp_timer_handle_t timer;
} button_cfg_t;

/**
 * @brief Create GPIO button
 * 
 * Instantiate interrupt-driven GPIO at the given pin, complete with debounce.
 * 
 * @param gpio_num - Pin for button. 
 * @param button_callback - Button callback after debounce
 * @return esp_err_t - ESP_OK if successful, error otherwise
 */
esp_err_t init_gpio_button(gpio_num_t gpio_num, esp_timer_cb_t button_callback);

#endif // __GPIO_BUTTON__