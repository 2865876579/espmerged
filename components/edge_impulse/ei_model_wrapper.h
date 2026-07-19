#ifndef EI_MODEL_WRAPPER_H
#define EI_MODEL_WRAPPER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t ei_model_frame_size(void);

int ei_model_run(const int16_t *samples,
                 size_t sample_count,
                 float *snoring_probability,
                 float *not_snoring_probability);

#ifdef __cplusplus
}
#endif

#endif
