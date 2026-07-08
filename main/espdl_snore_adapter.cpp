#include "snore_infer_backend.h"

#include <stddef.h>
#include <string.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

static const int16_t *s_signal_pcm = nullptr;
static int s_signal_samples = 0;

static int snore_get_signal_data(size_t offset, size_t length, float *out_ptr)
{
    if (!s_signal_pcm || !out_ptr || offset + length > (size_t)s_signal_samples) {
        return EIDSP_OUT_OF_MEM;
    }

    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = static_cast<float>(s_signal_pcm[offset + i]);
    }
    return EIDSP_OK;
}

extern "C" const char *espdl_snore_adapter_name(void)
{
    return "espdl_compatible_edge_impulse_snore_backend";
}

extern "C" bool espdl_snore_adapter_run(const int16_t *pcm,
                                         int samples,
                                         snore_infer_result_t *out)
{
    if (!pcm || !out || samples != EI_CLASSIFIER_RAW_SAMPLE_COUNT) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    s_signal_pcm = pcm;
    s_signal_samples = samples;

    signal_t signal;
    signal.total_length = (size_t)samples;
    signal.get_data = snore_get_signal_data;

    ei_impulse_result_t result = {};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

    s_signal_pcm = nullptr;
    s_signal_samples = 0;

    if (err != EI_IMPULSE_OK) {
        return false;
    }

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        const char *label = result.classification[i].label;
        float value = result.classification[i].value;
        if (label && strcmp(label, "snore") == 0) {
            out->snore = value;
        } else if (label && strcmp(label, "non_snore") == 0) {
            out->non_snore = value;
        }
    }
    out->dsp_ms = result.timing.dsp;
    out->classification_ms = result.timing.classification;
    return true;
}
