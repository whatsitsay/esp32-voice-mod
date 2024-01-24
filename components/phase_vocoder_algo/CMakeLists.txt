idf_component_register(SRCS  "peak_shift/peak_shift.c"
                       INCLUDE_DIRS "peak_shift"
                       REQUIRES esp-dsp algo_common fast_trig)