idf_component_register(SRCS  "gpio_button.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_timer)