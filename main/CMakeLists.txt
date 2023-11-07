# Included as a component
idf_component_register(SRCS "esp_vocoder_main.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES es8388 algo_common phase_vocoder_algo gpio_button driver esp-dsp esp_timer)