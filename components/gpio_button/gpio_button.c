#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "gpio_button.h"

// Notify task
static void IRAM_ATTR _gpio_interrupt(void *args)
{
  // Get task to notify
  TaskHandle_t task_to_notify = (TaskHandle_t)args;

  // Notify task given by handle
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(task_to_notify, &xHigherPriorityTaskWoken);

  // Yield if higher prio task woken
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

esp_err_t init_gpio_button(gpio_num_t gpio_num, TaskHandle_t task_to_notify)
{
  esp_err_t err = ESP_OK;

  err |= gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
  // No pullup or pulldown internally (handled by circuit)
  err |= gpio_pullup_dis(gpio_num);
  err |= gpio_pulldown_dis(gpio_num);
  err |= gpio_install_isr_service(0);
  // ISR on change (rest will be handled by debounce)
  err |= gpio_set_intr_type(gpio_num, GPIO_INTR_ANYEDGE);
  err |= gpio_isr_handler_add(gpio_num, _gpio_interrupt, (void *)task_to_notify);
  err |= gpio_intr_enable(gpio_num);

  return err;
}

bool gpio_debounce_check(gpio_num_t gpio_num, int* gpio_level)
{
  // Get initial GPIO level
  int init_level = gpio_get_level(gpio_num);

  // Delay by debounce time
  vTaskDelay( DEBOUNCE_TIMEOUT_MS / portTICK_PERIOD_MS );

  // Get final GPIO level, store in pointer
  *gpio_level = gpio_get_level(gpio_num);

  // Return whether initial level matches final level
  return (*gpio_level == init_level);
}
