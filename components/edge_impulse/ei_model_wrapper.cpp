#include "ei_model_wrapper.h"

#include <cstring>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

extern "C" size_t ei_model_frame_size(void)
{
    return EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
}

extern "C" int ei_model_run(const int16_t *samples,
                            size_t sample_count,
                            float *snoring_probability,
                            float *not_snoring_probability)
{
    if (samples == nullptr || snoring_probability == nullptr ||
        not_snoring_probability == nullptr ||
        sample_count != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        return -1;
    }

    signal_t signal;
    signal.total_length = sample_count;
    signal.get_data = [samples](size_t offset, size_t length, float *out) {
        for (size_t i = 0; i < length; ++i) {
            out[i] = static_cast<float>(samples[offset + i]);
        }
        return EIDSP_OK;
    };

    ei_impulse_result_t result = {};
    const EI_IMPULSE_ERROR error = run_classifier(&signal, &result, false);
    if (error != EI_IMPULSE_OK) {
        return static_cast<int>(error);
    }

    bool found_snoring = false;
    bool found_not_snoring = false;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
        const char *label = result.classification[i].label;
        if (std::strcmp(label, "snoring") == 0) {
            *snoring_probability = result.classification[i].value;
            found_snoring = true;
        }
        else if (std::strcmp(label, "not_snoring") == 0) {
            *not_snoring_probability = result.classification[i].value;
            found_not_snoring = true;
        }
    }

    return (found_snoring && found_not_snoring) ? 0 : -2;
}
