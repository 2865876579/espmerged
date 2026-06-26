#pragma once

#include <stdint.h>

#define AVATAR_BASE_W 320
#define AVATAR_BASE_H 480
#define AVATAR_HEAD_X 40
#define AVATAR_HEAD_Y 0
#define AVATAR_HEAD_W 240
#define AVATAR_HEAD_H 260
#define AVATAR_HEAD_FRAME_COUNT 12

extern const uint8_t *const avatar_base_rgb666;
extern const uint8_t *const avatar_head_rgb666_frames[AVATAR_HEAD_FRAME_COUNT];
