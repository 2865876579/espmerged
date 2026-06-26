#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_ILI9488_H_RES 320
#define LCD_ILI9488_V_RES 480

esp_err_t lcd_ili9488_init(void);
esp_err_t lcd_ili9488_fill_rect_rgb565(int x, int y, int w, int h, uint16_t color);
esp_err_t lcd_ili9488_fill_screen_rgb565(uint16_t color);
esp_err_t lcd_ili9488_draw_test_pattern(void);
esp_err_t lcd_ili9488_draw_ai_face(void);
esp_err_t lcd_ili9488_draw_rgb565_bitmap(int x, int y, int w, int h, const uint16_t *data);
esp_err_t lcd_ili9488_draw_rgb666_image(int x, int y, int w, int h, const uint8_t *data);

#ifdef __cplusplus
}
#endif
