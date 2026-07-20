#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t screen_anim_start(void);
void screen_anim_set_subtitle(const char *speaker, const char *text);
void screen_anim_set_subtitle_timed(const char *speaker, const char *text, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif
