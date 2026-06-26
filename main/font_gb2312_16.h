#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FONT_GB2312_16_W 16
#define FONT_GB2312_16_H 16
#define FONT_GB2312_16_BYTES 32

const uint8_t *font_gb2312_16_get(uint32_t codepoint);

#ifdef __cplusplus
}
#endif
