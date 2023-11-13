#ifndef __GPIO_BUTTON__
#define __GPIO_BUTTON__

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define DEBOUNCE_TIMEOUT_MS (15)

/**
 * @brief Create GPIO button
 * 
 * Instantiate interrupt-driven GPIO at the given pin.
 * Should be accompanied by a debounce check in the notified task
 * 
 * @param gpio_num - Pin for button. 
 * @return esp_err_t - ESP_OK if successful, error otherwise
 */
esp_err_t init_gpio_button(gpio_num_t gpio_num, TaskHandle_t task_to_notify);

/**
 * @brief Helper function for determining debounce conditions
 * 
 * ISR should be individually disabled before calling, to prevent spurious results
 * 
 * @param gpio_num - IO to check (should be from ISR)
 * @param gpio_level - Level of the gpio after debounce check (if valid)
 * @return true - debounce valid (same value before and after wait)
 * @return false - debounce invalid (different values before and after wait)
 */
bool gpio_debounce_check(gpio_num_t gpio_num, int* gpio_level);

#endif // __GPIO_BUTTON__