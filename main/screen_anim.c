#include <stdint.h>
#include <string.h>

#include "avatar_rgb666_frames.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_ili9488.h"
#include "screen_anim.h"

#define DIRECT_DRAW_LINES 8
#define ANIMATION_TASK_STACK 4096
#define ANIMATION_TASK_PRIORITY 2
#define ENABLE_LOWER_PARTICLES 0
#define LOWER_PARTICLE_X 30
#define LOWER_PARTICLE_Y 270
#define LOWER_PARTICLE_W 260
#define LOWER_PARTICLE_H 190
#define LOWER_PARTICLE_REFRESH_DIV 3
#define LOWER_PARTICLE_WAVE_PERIOD 128
#define ENABLE_AMBIENT_FX 0
#define ENABLE_AI_STATUS_PANEL 1
#define STATUS_PANEL_X 0
#define STATUS_PANEL_Y 426
#define STATUS_PANEL_W LCD_ILI9488_H_RES
#define STATUS_PANEL_H 34

#if ENABLE_AMBIENT_FX
#define AMBIENT_TILE_MAX 48
#define AMBIENT_WAVE_PERIOD 128
#define AMBIENT_FX_PER_STEP 2
#endif

static const char *TAG = "screen_anim";
static TaskHandle_t s_animation_task_handle;

static const uint16_t s_frame_durations_ms[AVATAR_HEAD_FRAME_COUNT] = {
    420, 170, 170, 180, 180, 180, 120, 130, 170, 180, 180, 420,
};

static uint8_t *s_tx_buf;

#if ENABLE_LOWER_PARTICLES
typedef struct {
    int16_t x;
    int16_t y;
    int8_t drift_x;
    int8_t drift_y;
    uint8_t radius;
    uint8_t phase;
    uint8_t strength;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} lower_particle_t;

static const lower_particle_t s_lower_particles[] = {
    {.x = 52, .y = 292, .drift_x = 12, .drift_y = 7, .radius = 6, .phase = 0, .strength = 166, .r = 255, .g = 136, .b = 64},
    {.x = 270, .y = 306, .drift_x = -11, .drift_y = 8, .radius = 5, .phase = 17, .strength = 154, .r = 210, .g = 76, .b = 255},
    {.x = 68, .y = 344, .drift_x = 10, .drift_y = -9, .radius = 5, .phase = 31, .strength = 152, .r = 70, .g = 176, .b = 255},
    {.x = 260, .y = 356, .drift_x = -12, .drift_y = 8, .radius = 6, .phase = 45, .strength = 160, .r = 255, .g = 92, .b = 170},
    {.x = 88, .y = 404, .drift_x = 10, .drift_y = -10, .radius = 6, .phase = 62, .strength = 148, .r = 245, .g = 88, .b = 230},
    {.x = 236, .y = 420, .drift_x = -11, .drift_y = -9, .radius = 5, .phase = 78, .strength = 154, .r = 78, .g = 142, .b = 255},
    {.x = 42, .y = 376, .drift_x = 8, .drift_y = 6, .radius = 4, .phase = 96, .strength = 142, .r = 255, .g = 114, .b = 206},
    {.x = 278, .y = 452, .drift_x = -8, .drift_y = 8, .radius = 4, .phase = 112, .strength = 146, .r = 255, .g = 154, .b = 70},
    {.x = 118, .y = 454, .drift_x = 8, .drift_y = -7, .radius = 4, .phase = 52, .strength = 140, .r = 82, .g = 214, .b = 232},
    {.x = 212, .y = 322, .drift_x = -8, .drift_y = 9, .radius = 4, .phase = 86, .strength = 142, .r = 178, .g = 72, .b = 255},
};

static uint8_t clamp_rgb666_channel(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 252) {
        return 252;
    }
    return (uint8_t)(value & 0xFC);
}

static int particle_wave_offset(uint32_t tick, uint8_t phase, int amplitude)
{
    const int p = (int)((tick + phase) & (LOWER_PARTICLE_WAVE_PERIOD - 1));
    const int half = LOWER_PARTICLE_WAVE_PERIOD / 2;
    const int triangle = p < half ? p : LOWER_PARTICLE_WAVE_PERIOD - p;
    return ((triangle * 2 - half) * amplitude) / half;
}

static void add_particle_pixel(uint8_t *pixel, const lower_particle_t *particle, int alpha)
{
    if (alpha <= 0) {
        return;
    }
    if (alpha > 230) {
        alpha = 230;
    }

    pixel[0] = clamp_rgb666_channel(pixel[0] + ((int)particle->r - pixel[0]) * alpha / 255);
    pixel[1] = clamp_rgb666_channel(pixel[1] + ((int)particle->g - pixel[1]) * alpha / 255);
    pixel[2] = clamp_rgb666_channel(pixel[2] + ((int)particle->b - pixel[2]) * alpha / 255);
}

static void overlay_lower_particles(int chunk_x, int chunk_y, int w, int lines, uint32_t tick)
{
    for (size_t i = 0; i < sizeof(s_lower_particles) / sizeof(s_lower_particles[0]); i++) {
        const lower_particle_t *particle = &s_lower_particles[i];
        const int cx = particle->x + particle_wave_offset(tick, particle->phase, particle->drift_x);
        const int cy = particle->y + particle_wave_offset(tick, (uint8_t)(particle->phase + 37), particle->drift_y);
        const int radius = particle->radius;
        const int r2 = radius * radius;

        int x0 = cx - radius;
        int x1 = cx + radius;
        int y0 = cy - radius;
        int y1 = cy + radius;

        if (x0 < chunk_x) {
            x0 = chunk_x;
        }
        if (x1 >= chunk_x + w) {
            x1 = chunk_x + w - 1;
        }
        if (y0 < chunk_y) {
            y0 = chunk_y;
        }
        if (y1 >= chunk_y + lines) {
            y1 = chunk_y + lines - 1;
        }
        if (x0 > x1 || y0 > y1) {
            continue;
        }

        const int pulse = particle_wave_offset(tick, (uint8_t)(particle->phase + 19), particle->strength / 2);
        const int intensity = particle->strength + pulse;
        for (int py = y0; py <= y1; py++) {
            for (int px = x0; px <= x1; px++) {
                const int dx = px - cx;
                const int dy = py - cy;
                const int dist = dx * dx + dy * dy;
                if (dist > r2) {
                    continue;
                }

                int alpha = (r2 - dist) * intensity / (r2 + 1);
                if (dist <= 1) {
                    alpha += intensity / 2;
                }
                const size_t offset = (((size_t)(py - chunk_y) * w) + (px - chunk_x)) * 3;
                add_particle_pixel(&s_tx_buf[offset], particle, alpha);
            }
        }
    }
}
#endif

#if ENABLE_AMBIENT_FX
typedef enum {
    AMBIENT_FX_BREATH,
    AMBIENT_FX_PARTICLE,
} ambient_fx_kind_t;

typedef struct {
    int x;
    int y;
    int size;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t phase;
    int8_t drift_x;
    int8_t drift_y;
    uint8_t strength;
    ambient_fx_kind_t kind;
} ambient_fx_t;

static const ambient_fx_t s_ambient_fx[] = {
    {.x = 2, .y = 118, .size = 34, .r = 255, .g = 238, .b = 214, .phase = 0, .drift_x = 0, .drift_y = 1, .strength = 20, .kind = AMBIENT_FX_BREATH},
    {.x = 284, .y = 286, .size = 34, .r = 255, .g = 236, .b = 210, .phase = 44, .drift_x = 0, .drift_y = -1, .strength = 18, .kind = AMBIENT_FX_BREATH},
    {.x = 16, .y = 78, .size = 7, .r = 255, .g = 246, .b = 220, .phase = 8, .drift_x = 3, .drift_y = 2, .strength = 96, .kind = AMBIENT_FX_PARTICLE},
    {.x = 296, .y = 102, .size = 6, .r = 255, .g = 232, .b = 204, .phase = 22, .drift_x = -2, .drift_y = 3, .strength = 86, .kind = AMBIENT_FX_PARTICLE},
    {.x = 28, .y = 226, .size = 8, .r = 255, .g = 242, .b = 218, .phase = 36, .drift_x = 2, .drift_y = -3, .strength = 82, .kind = AMBIENT_FX_PARTICLE},
    {.x = 288, .y = 220, .size = 7, .r = 255, .g = 238, .b = 210, .phase = 50, .drift_x = -2, .drift_y = 2, .strength = 90, .kind = AMBIENT_FX_PARTICLE},
    {.x = 56, .y = 332, .size = 9, .r = 255, .g = 248, .b = 226, .phase = 64, .drift_x = 3, .drift_y = -2, .strength = 76, .kind = AMBIENT_FX_PARTICLE},
    {.x = 250, .y = 356, .size = 8, .r = 255, .g = 236, .b = 212, .phase = 78, .drift_x = -3, .drift_y = -2, .strength = 84, .kind = AMBIENT_FX_PARTICLE},
    {.x = 98, .y = 420, .size = 6, .r = 255, .g = 248, .b = 228, .phase = 92, .drift_x = 2, .drift_y = -3, .strength = 78, .kind = AMBIENT_FX_PARTICLE},
    {.x = 198, .y = 410, .size = 7, .r = 255, .g = 238, .b = 214, .phase = 110, .drift_x = -2, .drift_y = -2, .strength = 80, .kind = AMBIENT_FX_PARTICLE},
};

enum {
    AMBIENT_FX_COUNT = sizeof(s_ambient_fx) / sizeof(s_ambient_fx[0]),
};

static uint32_t s_ambient_ticks[AMBIENT_FX_COUNT];
#endif

static esp_err_t draw_rgb666_rect(int x, int y, int w, int h,
                                  const uint8_t *pixels, int src_stride,
                                  uint32_t particle_tick)
{
    for (int row = 0; row < h; row += DIRECT_DRAW_LINES) {
        int lines = h - row;
        if (lines > DIRECT_DRAW_LINES) {
            lines = DIRECT_DRAW_LINES;
        }

        const size_t len = (size_t)w * lines * 3;
        const uint8_t *src = pixels + ((size_t)row * src_stride * 3);
        if (src_stride == w) {
            memcpy(s_tx_buf, src, len);
        } else {
            uint8_t *dst = s_tx_buf;
            for (int line = 0; line < lines; line++) {
                memcpy(dst, src + ((size_t)line * src_stride * 3), (size_t)w * 3);
                dst += (size_t)w * 3;
            }
        }
#if ENABLE_LOWER_PARTICLES
        if (particle_tick != UINT32_MAX) {
            overlay_lower_particles(x, y + row, w, lines, particle_tick);
        }
#else
        (void)particle_tick;
#endif
        ESP_RETURN_ON_ERROR(lcd_ili9488_draw_rgb666_image(x, y + row, w, lines, s_tx_buf),
                            TAG, "draw image chunk");
    }

    return ESP_OK;
}

#if ENABLE_AI_STATUS_PANEL
static uint8_t status_clamp_rgb666(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 252) {
        return 252;
    }
    return (uint8_t)(value & 0xFC);
}

static void status_blend_pixel(uint8_t *pixel, int r, int g, int b, int alpha)
{
    if (alpha <= 0) {
        return;
    }
    if (alpha > 255) {
        alpha = 255;
    }

    pixel[0] = status_clamp_rgb666(pixel[0] + (r - pixel[0]) * alpha / 255);
    pixel[1] = status_clamp_rgb666(pixel[1] + (g - pixel[1]) * alpha / 255);
    pixel[2] = status_clamp_rgb666(pixel[2] + (b - pixel[2]) * alpha / 255);
}

static const uint8_t s_status_cn_line1[10][11] = {
    {0x12, 0x05, 0xF8, 0xBF, 0x24, 0x40, 0x00, 0x35, 0x01, 0x00, 0x06},
    {0x36, 0x05, 0x08, 0xA1, 0x12, 0x80, 0x01, 0xC4, 0xBF, 0xFB, 0xB8},
    {0x27, 0xE7, 0xF8, 0xFF, 0x07, 0xE0, 0x00, 0x44, 0x02, 0x02, 0xA0},
    {0x64, 0x35, 0x0A, 0xA1, 0x74, 0x20, 0x01, 0xFF, 0xC4, 0x42, 0xA0},
    {0xE9, 0x55, 0xFE, 0xBF, 0x97, 0xE0, 0x00, 0x44, 0x08, 0x42, 0xBF},
    {0xA1, 0x05, 0x54, 0xAA, 0x96, 0xA0, 0x00, 0x7C, 0x8B, 0xF2, 0xA4},
    {0x25, 0x85, 0xFC, 0xBF, 0x92, 0x80, 0x01, 0xC3, 0x18, 0x42, 0xA4},
    {0x2D, 0x44, 0x98, 0x93, 0x16, 0x91, 0x00, 0x46, 0x68, 0x43, 0xA4},
    {0x29, 0x24, 0x60, 0x8C, 0x1C, 0x92, 0x00, 0x5B, 0x48, 0x42, 0x44},
    {0x27, 0x05, 0x9C, 0xB3, 0xB8, 0xF2, 0x01, 0xC1, 0x8F, 0xF8, 0x84},
};

static const uint8_t s_status_cn_line2[11][13] = {
    {0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00},
    {0x0C, 0x3F, 0xF8, 0xA0, 0x29, 0x02, 0xF8, 0x18, 0x7D, 0xC0, 0x00, 0x08, 0x00},
    {0xFF, 0xC2, 0x41, 0xA0, 0x49, 0x04, 0x89, 0xFF, 0x92, 0x40, 0x00, 0x0F, 0xC0},
    {0x80, 0x4F, 0xF1, 0xFE, 0x7F, 0xE4, 0xF9, 0x00, 0xFE, 0x80, 0x00, 0x08, 0x00},
    {0x3F, 0x08, 0x12, 0x20, 0x4A, 0x4A, 0x88, 0x30, 0x13, 0xE0, 0x00, 0xFF, 0x80},
    {0x02, 0x0F, 0xF0, 0x20, 0xCA, 0x45, 0xFF, 0xFF, 0xFE, 0xAF, 0xFE, 0x80, 0x80},
    {0x0C, 0x08, 0x13, 0xFF, 0xDD, 0x4C, 0x10, 0x82, 0x27, 0xF0, 0x00, 0xFF, 0x80},
    {0xFF, 0xFF, 0xFC, 0x50, 0x55, 0x95, 0xFC, 0x84, 0x3C, 0xA0, 0x00, 0x00, 0x00},
    {0x08, 0x05, 0x20, 0x51, 0x54, 0x84, 0x90, 0x78, 0x25, 0xE0, 0x00, 0xA4, 0x80},
    {0x08, 0x1D, 0x3D, 0x91, 0x5D, 0x44, 0x10, 0x26, 0x3C, 0x80, 0x00, 0xB6, 0x40},
    {0x38, 0x25, 0x66, 0x1F, 0x52, 0x24, 0x73, 0xC1, 0xAD, 0x80, 0x01, 0x12, 0x40},
};

static void status_draw_bitmap_line_chunk(uint8_t *buf, int width, int chunk_y, int lines,
                                          int draw_x, int draw_y, int bitmap_w, int bitmap_h,
                                          int row_bytes, const uint8_t *bitmap,
                                          int r, int g, int b, int alpha)
{
    for (int gy = 0; gy < bitmap_h; gy++) {
        const int py = draw_y + gy;
        if (py < chunk_y || py >= chunk_y + lines) {
            continue;
        }
        const uint8_t *src = bitmap + (size_t)gy * row_bytes;
        for (int gx = 0; gx < bitmap_w; gx++) {
            if ((src[gx >> 3] & (uint8_t)(0x80 >> (gx & 7))) == 0) {
                continue;
            }
            const int px = draw_x + gx;
            if (px < 0 || px >= width) {
                continue;
            }
            uint8_t *pixel = &buf[(((size_t)(py - chunk_y) * width) + px) * 3];
            status_blend_pixel(pixel, r, g, b, alpha);
        }
    }
}

static void status_draw_subtitle_line(uint8_t *buf, int width, int chunk_y, int lines,
                                      int draw_x, int draw_y, int bitmap_w, int bitmap_h,
                                      int row_bytes, const uint8_t *bitmap)
{
    static const int shadow_offsets[][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {1, 1},
    };

    for (size_t i = 0; i < sizeof(shadow_offsets) / sizeof(shadow_offsets[0]); i++) {
        status_draw_bitmap_line_chunk(buf, width, chunk_y, lines,
                                      draw_x + shadow_offsets[i][0],
                                      draw_y + shadow_offsets[i][1],
                                      bitmap_w, bitmap_h, row_bytes, bitmap,
                                      0, 0, 0, 210);
    }

    status_draw_bitmap_line_chunk(buf, width, chunk_y, lines,
                                  draw_x, draw_y,
                                  bitmap_w, bitmap_h, row_bytes, bitmap,
                                  252, 252, 252, 255);
}

static void status_overlay_chunk(int chunk_y, int lines, uint32_t tick)
{
    (void)tick;
    const int block_x = (STATUS_PANEL_W - 98) / 2;

    status_draw_subtitle_line(s_tx_buf, STATUS_PANEL_W, chunk_y, lines,
                              block_x + 5, 4,
                              88, 10, (int)sizeof(s_status_cn_line1[0]),
                              &s_status_cn_line1[0][0]);
    status_draw_subtitle_line(s_tx_buf, STATUS_PANEL_W, chunk_y, lines,
                              block_x, 18,
                              98, 11, (int)sizeof(s_status_cn_line2[0]),
                              &s_status_cn_line2[0][0]);
}

static esp_err_t draw_status_panel(uint32_t tick)
{
    for (int row = 0; row < STATUS_PANEL_H; row += DIRECT_DRAW_LINES) {
        int lines = STATUS_PANEL_H - row;
        if (lines > DIRECT_DRAW_LINES) {
            lines = DIRECT_DRAW_LINES;
        }

        uint8_t *dst = s_tx_buf;
        for (int line = 0; line < lines; line++) {
            const uint8_t *src = avatar_base_rgb666 +
                                 (((size_t)(STATUS_PANEL_Y + row + line) *
                                   LCD_ILI9488_H_RES + STATUS_PANEL_X) *
                                  3);
            memcpy(dst, src, STATUS_PANEL_W * 3);
            dst += STATUS_PANEL_W * 3;
        }

        status_overlay_chunk(row, lines, tick);
        ESP_RETURN_ON_ERROR(lcd_ili9488_draw_rgb666_image(STATUS_PANEL_X,
                                                          STATUS_PANEL_Y + row,
                                                          STATUS_PANEL_W,
                                                          lines,
                                                          s_tx_buf),
                            TAG, "draw status panel");
    }

    return ESP_OK;
}
#endif

#if ENABLE_AMBIENT_FX
static uint8_t clamp_rgb666(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 252) {
        return 252;
    }
    return (uint8_t)(value & 0xFC);
}

static void copy_base_tile(int x, int y, int w, int h)
{
    uint8_t *dst = s_tx_buf;
    for (int row = 0; row < h; row++) {
        const uint8_t *src = avatar_base_rgb666 + (((size_t)(y + row) * LCD_ILI9488_H_RES + x) * 3);
        memcpy(dst, src, (size_t)w * 3);
        dst += (size_t)w * 3;
    }
}

static int wave_offset(uint32_t tick, uint8_t phase, int amplitude)
{
    const int p = (int)((tick + phase) & (AMBIENT_WAVE_PERIOD - 1));
    const int half = AMBIENT_WAVE_PERIOD / 2;
    const int triangle = p < half ? p : AMBIENT_WAVE_PERIOD - p;
    return ((triangle * 2 - half) * amplitude) / half;
}

static void ambient_position(const ambient_fx_t *fx, uint32_t tick, int *x, int *y)
{
    const int size = fx->size > AMBIENT_TILE_MAX ? AMBIENT_TILE_MAX : fx->size;
    int px = fx->x + wave_offset(tick, fx->phase, fx->drift_x);
    int py = fx->y + wave_offset(tick, (uint8_t)(fx->phase + 17), fx->drift_y);

    if (px < 0) {
        px = 0;
    } else if (px + size > LCD_ILI9488_H_RES) {
        px = LCD_ILI9488_H_RES - size;
    }
    if (py < 0) {
        py = 0;
    } else if (py + size > LCD_ILI9488_V_RES) {
        py = LCD_ILI9488_V_RES - size;
    }

    *x = px;
    *y = py;
}

static void add_glow_pixel(uint8_t *pixel, uint8_t r, uint8_t g, uint8_t b, int alpha)
{
    if (alpha <= 0) {
        return;
    }
    if (alpha > 255) {
        alpha = 255;
    }

    pixel[0] = clamp_rgb666(pixel[0] + ((int)r - pixel[0]) * alpha / 255);
    pixel[1] = clamp_rgb666(pixel[1] + ((int)g - pixel[1]) * alpha / 255);
    pixel[2] = clamp_rgb666(pixel[2] + ((int)b - pixel[2]) * alpha / 255);
}

static esp_err_t draw_ambient_fx(const ambient_fx_t *fx, uint32_t tick)
{
    const int size = fx->size > AMBIENT_TILE_MAX ? AMBIENT_TILE_MAX : fx->size;
    int screen_x;
    int screen_y;
    ambient_position(fx, tick, &screen_x, &screen_y);

    const int cx = size / 2;
    const int cy = size / 2;
    const uint32_t phase = (tick + fx->phase) & (AMBIENT_WAVE_PERIOD - 1);
    const uint32_t half = AMBIENT_WAVE_PERIOD / 2;
    const int pulse = phase < half ? (int)phase : (int)(AMBIENT_WAVE_PERIOD - phase);
    int intensity = fx->strength;
    const int radius = size / 2;
    const int outer = radius * radius;

    if (fx->kind == AMBIENT_FX_BREATH) {
        intensity += pulse * fx->strength / (int)(half * 2);
    } else {
        intensity += pulse * fx->strength / (int)(half * 3);
    }

    copy_base_tile(screen_x, screen_y, size, size);

    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            const int dx = x - cx;
            const int dy = y - cy;
            const int dist = dx * dx + dy * dy;
            int alpha = 0;

            if (dist <= outer) {
                alpha = (outer - dist) * intensity / (outer + 1);
            }
            if (fx->kind == AMBIENT_FX_PARTICLE && dist <= 1) {
                alpha += intensity / 2;
            }

            add_glow_pixel(&s_tx_buf[((size_t)y * size + x) * 3], fx->r, fx->g, fx->b, alpha);
        }
    }

    return lcd_ili9488_draw_rgb666_image(screen_x, screen_y, size, size, s_tx_buf);
}

static esp_err_t restore_ambient_fx(const ambient_fx_t *fx, uint32_t tick)
{
    const int size = fx->size > AMBIENT_TILE_MAX ? AMBIENT_TILE_MAX : fx->size;
    int x;
    int y;
    ambient_position(fx, tick, &x, &y);
    copy_base_tile(x, y, size, size);
    return lcd_ili9488_draw_rgb666_image(x, y, size, size, s_tx_buf);
}

static void draw_ambient_frame(uint32_t tick)
{
    for (size_t i = 0; i < sizeof(s_ambient_fx) / sizeof(s_ambient_fx[0]); i++) {
        esp_err_t err = draw_ambient_fx(&s_ambient_fx[i], tick);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ambient fx %u failed: %s", (unsigned)i, esp_err_to_name(err));
        }
    }
}
#endif

static void animation_task(void *arg)
{
    (void)arg;

    s_tx_buf = heap_caps_malloc(LCD_ILI9488_H_RES * DIRECT_DRAW_LINES * 3,
                                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_tx_buf) {
        ESP_LOGE(TAG, "frame tx buffer allocation failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(draw_rgb666_rect(0, 0,
                                                  LCD_ILI9488_H_RES,
                                                  LCD_ILI9488_V_RES,
                                                  avatar_base_rgb666,
                                                  LCD_ILI9488_H_RES,
                                                  UINT32_MAX));
#if ENABLE_AMBIENT_FX
    draw_ambient_frame(0);
#endif

    size_t frame = 0;
    uint32_t animation_tick = 0;
#if ENABLE_AMBIENT_FX
    size_t ambient_cursor = 0;
#endif
#if ENABLE_AI_STATUS_PANEL
    ESP_ERROR_CHECK_WITHOUT_ABORT(draw_status_panel(0));
#endif
    while (1) {
        TickType_t start = xTaskGetTickCount();

#if ENABLE_AMBIENT_FX
        size_t ambient_indices[AMBIENT_FX_PER_STEP];
        for (size_t i = 0; i < AMBIENT_FX_PER_STEP; i++) {
            size_t idx = (ambient_cursor + i) % AMBIENT_FX_COUNT;
            ambient_indices[i] = idx;
            esp_err_t restore_err = restore_ambient_fx(&s_ambient_fx[idx], s_ambient_ticks[idx]);
            if (restore_err != ESP_OK) {
                ESP_LOGW(TAG, "ambient restore %u failed: %s", (unsigned)idx, esp_err_to_name(restore_err));
            }
        }
#endif

        esp_err_t err = draw_rgb666_rect(AVATAR_HEAD_X, AVATAR_HEAD_Y,
                                         AVATAR_HEAD_W, AVATAR_HEAD_H,
                                         avatar_head_rgb666_frames[frame],
                                         AVATAR_HEAD_W,
                                         UINT32_MAX);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "draw frame %u failed: %s", (unsigned)frame, esp_err_to_name(err));
        }

#if ENABLE_LOWER_PARTICLES
        if ((animation_tick % LOWER_PARTICLE_REFRESH_DIV) == 0) {
            const uint8_t *lower_pixels = avatar_base_rgb666 +
                                          (((size_t)LOWER_PARTICLE_Y * LCD_ILI9488_H_RES +
                                            LOWER_PARTICLE_X) *
                                           3);
            err = draw_rgb666_rect(LOWER_PARTICLE_X, LOWER_PARTICLE_Y,
                                   LOWER_PARTICLE_W, LOWER_PARTICLE_H,
                                   lower_pixels, LCD_ILI9488_H_RES,
                                   animation_tick);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "draw lower particles failed: %s", esp_err_to_name(err));
            }
        }
#endif

#if ENABLE_AMBIENT_FX
        for (size_t i = 0; i < AMBIENT_FX_PER_STEP; i++) {
            size_t idx = ambient_indices[i];
            uint32_t next_tick = s_ambient_ticks[idx] + 1;
            esp_err_t ambient_err = draw_ambient_fx(&s_ambient_fx[idx], next_tick);
            if (ambient_err != ESP_OK) {
                ESP_LOGW(TAG, "ambient fx %u failed: %s", (unsigned)idx, esp_err_to_name(ambient_err));
            }
            s_ambient_ticks[idx] = next_tick;
        }
        ambient_cursor = (ambient_cursor + AMBIENT_FX_PER_STEP) % AMBIENT_FX_COUNT;
#endif

        TickType_t elapsed = xTaskGetTickCount() - start;
        TickType_t duration = pdMS_TO_TICKS(s_frame_durations_ms[frame]);
        if (elapsed < duration) {
            vTaskDelay(duration - elapsed);
        } else {
            vTaskDelay(1);
        }

        frame = (frame + 1) % AVATAR_HEAD_FRAME_COUNT;
        animation_tick++;
    }
}

esp_err_t screen_anim_start(void)
{
    if (s_animation_task_handle) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "ESP32-S3 ILI9488 direct RGB666 animation");
    ESP_LOGI(TAG, "LCD VCC=5V/3V3 GND=GND SCK=39 MOSI=38 MISO=unused CS=41 DC=42 RST=16 LED=2");
    ESP_LOGI(TAG, "lower particles=%s ambient fx=%s status panel=%s",
             ENABLE_LOWER_PARTICLES ? "on" : "off",
             ENABLE_AMBIENT_FX ? "on" : "off",
             ENABLE_AI_STATUS_PANEL ? "on" : "off");

    ESP_ERROR_CHECK(lcd_ili9488_init());

    BaseType_t ok = xTaskCreatePinnedToCore(animation_task,
                                           "screen_anim",
                                           ANIMATION_TASK_STACK,
                                           NULL,
                                           ANIMATION_TASK_PRIORITY,
                                           &s_animation_task_handle,
                                           0);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
