# Included as a component
idf_component_register(SRCS "shift_chorus_main.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES es8388 phase_vocoder_algo driver esp-dsp)