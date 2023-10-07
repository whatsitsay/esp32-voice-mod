idf_component_register(SRCS  "algo_common/algo_common.c" "peak_shift/peak_shift.c")
                       INCLUDE_DIRS "algo_common" "peak_shift"
                       REQUIRES esp-dsp)