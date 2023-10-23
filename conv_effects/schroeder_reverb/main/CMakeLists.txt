# Included as a component
idf_component_register(SRCS "schroeder_reverb_main.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES es8388 algo_common filters driver esp-dsp)