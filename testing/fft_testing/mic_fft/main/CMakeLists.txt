idf_component_register(SRCS "mic_fft_main.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES es8388 algo_common phase_vocoder_algo filters driver esp-dsp)
