# Included as a component
idf_component_register(SRCS "delay_chorus_main.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES es8388 algo_common filters driver esp-dsp)