set(EI_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(EI_SDK ${EI_ROOT}/edge-impulse-sdk)

file(GLOB EI_MODEL_SOURCES ${EI_ROOT}/tflite-model/*.cpp)

set(EI_SOURCES
    ${EI_MODEL_SOURCES}
    ${EI_SDK}/classifier/ei_run_classifier_c.cpp
    ${EI_SDK}/dsp/memory.cpp
    ${EI_SDK}/dsp/kissfft/kiss_fft.cpp
    ${EI_SDK}/dsp/kissfft/kiss_fftr.cpp
    ${EI_SDK}/dsp/dct/fast-dct-fft.cpp
    ${EI_SDK}/dsp/image/processing.cpp
    ${EI_SDK}/porting/espressif/ei_classifier_porting.cpp
    ${EI_SDK}/porting/espressif/debug_log.cpp
    ${EI_SDK}/tensorflow/lite/c/common.c
    ${EI_SDK}/tensorflow/lite/core/api/common.cc
    ${EI_SDK}/tensorflow/lite/core/api/error_reporter.cc
    ${EI_SDK}/tensorflow/lite/kernels/kernel_util_lite.cc
    ${EI_SDK}/tensorflow/lite/kernels/internal/portable_tensor_utils.cc
    ${EI_SDK}/tensorflow/lite/kernels/internal/quantization_util.cc
    ${EI_SDK}/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.cc
    ${EI_SDK}/tensorflow/lite/kernels/internal/tensor_utils.cc
    ${EI_SDK}/tensorflow/lite/micro/micro_allocator.cc
    ${EI_SDK}/tensorflow/lite/micro/micro_allocation_info.cc
    ${EI_SDK}/tensorflow/lite/micro/micro_context.cc
    ${EI_SDK}/tensorflow/lite/micro/micro_log.cc
    ${EI_SDK}/tensorflow/lite/micro/memory_helpers.cc
    ${EI_SDK}/tensorflow/lite/micro/micro_string.cc
    ${EI_SDK}/tensorflow/lite/micro/micro_utils.cc
    ${EI_SDK}/tensorflow/lite/micro/memory_planner/greedy_memory_planner.cc
    ${EI_SDK}/tensorflow/lite/micro/memory_planner/linear_memory_planner.cc
    ${EI_SDK}/tensorflow/lite/micro/memory_planner/non_persistent_buffer_planner_shim.cc
    ${EI_SDK}/tensorflow/lite/micro/non_persistent_arena_buffer_allocator.cc
    ${EI_SDK}/tensorflow/lite/micro/persistent_arena_buffer_allocator.cc
    ${EI_SDK}/tensorflow/lite/micro/single_arena_buffer_allocator.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/conv.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/conv_common.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/fully_connected.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/fully_connected_common.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/kernel_util_micro.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/micro_tensor_utils.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/pooling.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/pooling_common.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/reshape.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/softmax.cc
    ${EI_SDK}/tensorflow/lite/micro/kernels/softmax_common.cc
)

file(GLOB_RECURSE EI_ESP_NN_SOURCES
    ${EI_SDK}/porting/espressif/ESP-NN/src/*.c
    ${EI_SDK}/porting/espressif/ESP-NN/src/*.S
    ${EI_SDK}/porting/espressif/ESP-NN/src/*.s
)

file(GLOB_RECURSE EI_ESP_DSP_SOURCES
    ${EI_SDK}/porting/espressif/esp-dsp/modules/common/misc/*.cpp
    ${EI_SDK}/porting/espressif/esp-dsp/modules/fft/*.c
    ${EI_SDK}/porting/espressif/esp-dsp/modules/fft/*.S
    ${EI_SDK}/porting/espressif/esp-dsp/modules/fft/*.s
)

idf_component_register(
    SRCS
        ${EI_SOURCES}
        ${EI_ESP_NN_SOURCES}
        ${EI_ESP_DSP_SOURCES}
    INCLUDE_DIRS
        "."
        "edge-impulse-sdk"
        "edge-impulse-sdk/porting"
        "edge-impulse-sdk/porting/espressif/ESP-NN"
        "edge-impulse-sdk/porting/espressif/ESP-NN/include"
        "edge-impulse-sdk/porting/espressif/ESP-NN/src/common"
        "edge-impulse-sdk/porting/espressif/esp-dsp/modules/common/include"
        "edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/include"
    REQUIRES
        esp_timer
        heap
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EI_PORTING_ESPRESSIF=1
    EIDSP_USE_ESP_DSP=1
    EI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=1
    EI_CLASSIFIER_TFLITE_ENABLE_ESP_NN_S3=1
    EI_CLASSIFIER_USE_FULL_TFLITE=0
    EI_CLASSIFIER_ALLOCATION_STATIC=1
    CONFIG_DSP_MAX_FFT_SIZE=4096
    portTICK_RATE_MS=portTICK_PERIOD_MS
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=unused-variable
    -Wno-error=unused-function
    -Wno-error=maybe-uninitialized
    -Wno-error=missing-field-initializers
    -Wno-error=class-memaccess
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-missing-field-initializers
)
