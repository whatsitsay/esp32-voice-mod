idf_component_register(SRCS "FastTrig/FastTrig.c"
                    INCLUDE_DIRS . "FastTrig")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
