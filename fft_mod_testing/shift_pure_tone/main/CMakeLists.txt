# Included as a component
idf_component_register(SRCS "shift_pure_tone_main.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES es8388 algo_common phase_vocoder_algo driver esp-dsp)