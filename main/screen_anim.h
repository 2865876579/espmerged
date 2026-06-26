#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t screen_anim_start(void);
void screen_anim_set_subtitle(const char *speaker, const char *text);

#ifdef __cplusplus
}
#endif
