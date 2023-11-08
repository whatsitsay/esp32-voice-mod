#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "gpio_button.h"

// Debounce callback for GPIO button
static void IRAM_ATTR debounce_callback(void *args)
{
  // Get GPIO idx, timer group and idx
  button_cfg_t* cfg = (button_cfg_t *)args;
 
  // Disable interrupt
  gpio_intr_disable(cfg->gpio_num);
  
  // Start timer
  esp_timer_start_once(cfg->timer, DEBOUNCE_TIMEOUT_US);
}

esp_err_t init_gpio_button(gpio_num_t gpio_num, esp_timer_cb_t button_callback)
{
  char timer_name[20];
  esp_timer_handle_t timer_handle;
  esp_err_t err = ESP_OK;

  // First create timer
  snprintf(timer_name, sizeof(timer_name), "GPIO %0d Debounce", gpio_num);
  esp_timer_create_args_t timer_args = {
    .callback = button_callback,
    .name = timer_name
  };
  err |= esp_timer_create(&timer_args, &timer_handle);

  // Instantiate button
  button_cfg_t cfg = {.gpio_num = gpio_num, .timer = timer_handle};
  err |= gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
  // No pullup or pulldown internally (handled by circuit)
  err |= gpio_pullup_dis(gpio_num);
  err |= gpio_pulldown_dis(gpio_num);
  err |= gpio_install_isr_service(0);
  err |= gpio_set_intr_type(gpio_num, GPIO_INTR_NEGEDGE); // External pullup
  err |= gpio_isr_handler_add(gpio_num, debounce_callback, (void *)(&cfg));
  err |= gpio_intr_enable(gpio_num);

  return err;
}