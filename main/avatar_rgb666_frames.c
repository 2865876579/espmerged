#include "avatar_rgb666_frames.h"

extern const uint8_t avatar_base_rgb666_bin_start[] asm("_binary_avatar_base_rgb666_bin_start");
extern const uint8_t avatar_head_rgb666_frame_0_bin_start[] asm("_binary_avatar_head_rgb666_frame_0_bin_start");
extern const uint8_t avatar_head_rgb666_frame_1_bin_start[] asm("_binary_avatar_head_rgb666_frame_1_bin_start");
extern const uint8_t avatar_head_rgb666_frame_2_bin_start[] asm("_binary_avatar_head_rgb666_frame_2_bin_start");
extern const uint8_t avatar_head_rgb666_frame_3_bin_start[] asm("_binary_avatar_head_rgb666_frame_3_bin_start");
extern const uint8_t avatar_head_rgb666_frame_4_bin_start[] asm("_binary_avatar_head_rgb666_frame_4_bin_start");
extern const uint8_t avatar_head_rgb666_frame_5_bin_start[] asm("_binary_avatar_head_rgb666_frame_5_bin_start");
extern const uint8_t avatar_head_rgb666_frame_6_bin_start[] asm("_binary_avatar_head_rgb666_frame_6_bin_start");
extern const uint8_t avatar_head_rgb666_frame_7_bin_start[] asm("_binary_avatar_head_rgb666_frame_7_bin_start");
extern const uint8_t avatar_head_rgb666_frame_8_bin_start[] asm("_binary_avatar_head_rgb666_frame_8_bin_start");
extern const uint8_t avatar_head_rgb666_frame_9_bin_start[] asm("_binary_avatar_head_rgb666_frame_9_bin_start");
extern const uint8_t avatar_head_rgb666_frame_10_bin_start[] asm("_binary_avatar_head_rgb666_frame_10_bin_start");
extern const uint8_t avatar_head_rgb666_frame_11_bin_start[] asm("_binary_avatar_head_rgb666_frame_11_bin_start");

const uint8_t *const avatar_base_rgb666 = avatar_base_rgb666_bin_start;

const uint8_t *const avatar_head_rgb666_frames[AVATAR_HEAD_FRAME_COUNT] = {
    avatar_head_rgb666_frame_0_bin_start,
    avatar_head_rgb666_frame_1_bin_start,
    avatar_head_rgb666_frame_2_bin_start,
    avatar_head_rgb666_frame_3_bin_start,
    avatar_head_rgb666_frame_4_bin_start,
    avatar_head_rgb666_frame_5_bin_start,
    avatar_head_rgb666_frame_6_bin_start,
    avatar_head_rgb666_frame_7_bin_start,
    avatar_head_rgb666_frame_8_bin_start,
    avatar_head_rgb666_frame_9_bin_start,
    avatar_head_rgb666_frame_10_bin_start,
    avatar_head_rgb666_frame_11_bin_start,
};
