idf_component_register(
    SRCS
        "ads1115_fsr402.c"
        "bh1750.c"
        "fsr402.c"
        "ky005.c"
        "ky022.c"
        "mq_135.c"
        "sht31.c"
        "usart.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc
)

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)
