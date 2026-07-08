#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float snore;
    float non_snore;
    int dsp_ms;
    int classification_ms;
} snore_infer_result_t;

/*
 * ESP-DL compatible adapter layer.
 *
 * 初赛版本先复用已经验证稳定的 Edge Impulse / TFLite Micro 后端，
 * 对外统一封装为本地 AI 推理后端，后续可以替换为标准 .espdl 模型。
 */
const char *espdl_snore_adapter_name(void);
bool espdl_snore_adapter_run(const int16_t *pcm, int samples, snore_infer_result_t *out);

#ifdef __cplusplus
}
#endif
