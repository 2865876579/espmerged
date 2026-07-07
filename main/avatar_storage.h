#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define AVATAR_STORAGE_W 320
#define AVATAR_STORAGE_H 480
#define AVATAR_STORAGE_RGB666_SIZE (AVATAR_STORAGE_W * AVATAR_STORAGE_H * 3)

esp_err_t avatar_storage_init(void);
const uint8_t *avatar_storage_get_base(const uint8_t *fallback);
bool avatar_storage_custom_active(void);
uint32_t avatar_storage_get_version(void);

esp_err_t avatar_storage_download_rgb666(const char *url,
                                         size_t expected_size,
                                         uint32_t expected_crc32);
uint32_t avatar_storage_parse_crc32_text(const char *text);
