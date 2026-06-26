#include "lcd_ili9488.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define LCD_SPI_HOST SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define LCD_MAX_TRANSFER_LINES 8

#define LCD_PIN_MOSI GPIO_NUM_38
#define LCD_PIN_MISO (-1)
#define LCD_PIN_SCLK GPIO_NUM_39
#define LCD_PIN_CS GPIO_NUM_41
#define LCD_PIN_DC GPIO_NUM_42
#define LCD_PIN_RST GPIO_NUM_16
#define LCD_PIN_BK_LIGHT GPIO_NUM_2

#define RGB565_RED 0xF800
#define RGB565_GREEN 0x07E0
#define RGB565_BLUE 0x001F
#define RGB565_WHITE 0xFFFF
#define RGB565_BLACK 0x0000
#define RGB565_YELLOW 0xFFE0
#define RGB565_CYAN 0x07FF
#define RGB565_MAGENTA 0xF81F

static const char *TAG = "ili9488";
static esp_lcd_panel_io_handle_t s_lcd_io;
static SemaphoreHandle_t s_color_done_sem;
static bool s_lcd_ready;

static esp_err_t lcd_cmd(uint8_t cmd)
{
    return esp_lcd_panel_io_tx_param(s_lcd_io, cmd, NULL, 0);
}

static esp_err_t lcd_cmd_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    return esp_lcd_panel_io_tx_param(s_lcd_io, cmd, data, len);
}

static void rgb565_to_rgb666(uint16_t color, uint8_t out[3])
{
    uint8_t r5 = (uint8_t)((color >> 11) & 0x1F);
    uint8_t g6 = (uint8_t)((color >> 5) & 0x3F);
    uint8_t b5 = (uint8_t)(color & 0x1F);
    out[0] = (uint8_t)(((r5 << 1) | (r5 >> 4)) << 2);
    out[1] = (uint8_t)(g6 << 2);
    out[2] = (uint8_t)(((b5 << 1) | (b5 >> 4)) << 2);
}

static bool lcd_color_done_cb(esp_lcd_panel_io_handle_t panel_io,
                              esp_lcd_panel_io_event_data_t *edata,
                              void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    (void)user_ctx;

    BaseType_t task_woken = pdFALSE;
    if (s_color_done_sem) {
        xSemaphoreGiveFromISR(s_color_done_sem, &task_woken);
    }
    return task_woken == pdTRUE;
}

static esp_err_t lcd_tx_color_sync(int lcd_cmd, const void *color, size_t color_size)
{
    if (s_color_done_sem) {
        while (xSemaphoreTake(s_color_done_sem, 0) == pdTRUE) {
        }
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(s_lcd_io, lcd_cmd, color, color_size),
                        TAG, "tx color");

    if (s_color_done_sem &&
        xSemaphoreTake(s_color_done_sem, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t lcd_set_window(int x0, int y0, int x1, int y1)
{
    uint8_t caset[] = {
        (uint8_t)(x0 >> 8), (uint8_t)x0,
        (uint8_t)((x1 - 1) >> 8), (uint8_t)(x1 - 1),
    };
    uint8_t raset[] = {
        (uint8_t)(y0 >> 8), (uint8_t)y0,
        (uint8_t)((y1 - 1) >> 8), (uint8_t)(y1 - 1),
    };

    ESP_RETURN_ON_ERROR(lcd_cmd_data(LCD_CMD_CASET, caset, sizeof(caset)), TAG, "set column");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(LCD_CMD_RASET, raset, sizeof(raset)), TAG, "set row");
    return ESP_OK;
}

static void lcd_reset(void)
{
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(250));
}

static esp_err_t lcd_gpio_init(void)
{
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rst_conf), TAG, "reset gpio config");

    gpio_config_t backlight_conf = {
        .pin_bit_mask = (1ULL << LCD_PIN_BK_LIGHT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&backlight_conf), TAG, "backlight gpio config");

    gpio_set_level(LCD_PIN_RST, 1);
    gpio_set_level(LCD_PIN_BK_LIGHT, 0);
    return ESP_OK;
}

static esp_err_t lcd_backlight_init(void)
{
    gpio_set_level(LCD_PIN_BK_LIGHT, 1);
    return ESP_OK;
}

static esp_err_t lcd_spi_init(void)
{
    if (!s_color_done_sem) {
        s_color_done_sem = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_color_done_sem != NULL, ESP_ERR_NO_MEM, TAG, "color done semaphore");
    }

    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_SCLK,
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = LCD_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_ILI9488_H_RES * LCD_MAX_TRANSFER_LINES * 3,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus init");

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .spi_mode = 0,
        .trans_queue_depth = 1,
        .on_color_trans_done = lcd_color_done_cb,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST,
                                                 &io_config, &s_lcd_io),
                        TAG, "panel io init");
    return ESP_OK;
}

static esp_err_t lcd_register_init(void)
{
    const uint8_t f7[] = {0xA9, 0x51, 0x2C, 0x82};
    const uint8_t c0[] = {0x11, 0x09};
    const uint8_t c1[] = {0x41};
    const uint8_t c5[] = {0x00, 0x0A, 0x80};
    const uint8_t b1[] = {0xB0, 0x11};
    const uint8_t b4[] = {0x02};
    const uint8_t b6[] = {0x02, 0x42};
    const uint8_t b7[] = {0xC6};
    const uint8_t be[] = {0x00, 0x04};
    const uint8_t e9[] = {0x00};
    const uint8_t madctl[] = {0x08}; /* portrait, BGR */
    const uint8_t colmod[] = {0x66}; /* RGB666, 3 bytes per pixel */
    const uint8_t e0[] = {
        0x00, 0x07, 0x10, 0x09, 0x17, 0x0B, 0x41, 0x89,
        0x4B, 0x0A, 0x0C, 0x0E, 0x18, 0x1B, 0x0F,
    };
    const uint8_t e1[] = {
        0x00, 0x17, 0x1A, 0x04, 0x0E, 0x06, 0x2F, 0x45,
        0x43, 0x02, 0x0A, 0x09, 0x32, 0x36, 0x0F,
    };

    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xF7, f7, sizeof(f7)), TAG, "cmd F7");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xC0, c0, sizeof(c0)), TAG, "cmd C0");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xC1, c1, sizeof(c1)), TAG, "cmd C1");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xC5, c5, sizeof(c5)), TAG, "cmd C5");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xB1, b1, sizeof(b1)), TAG, "cmd B1");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xB4, b4, sizeof(b4)), TAG, "cmd B4");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xB6, b6, sizeof(b6)), TAG, "cmd B6");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xB7, b7, sizeof(b7)), TAG, "cmd B7");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xBE, be, sizeof(be)), TAG, "cmd BE");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xE9, e9, sizeof(e9)), TAG, "cmd E9");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(LCD_CMD_MADCTL, madctl, sizeof(madctl)), TAG, "madctl");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(LCD_CMD_COLMOD, colmod, sizeof(colmod)), TAG, "colmod");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xE0, e0, sizeof(e0)), TAG, "cmd E0");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xE1, e1, sizeof(e1)), TAG, "cmd E1");

    ESP_RETURN_ON_ERROR(lcd_cmd(LCD_CMD_SLPOUT), TAG, "sleep out");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_cmd(LCD_CMD_DISPON), TAG, "display on");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

esp_err_t lcd_ili9488_init(void)
{
    if (s_lcd_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(lcd_gpio_init(), TAG, "gpio init");
    ESP_RETURN_ON_ERROR(lcd_spi_init(), TAG, "spi init");
    lcd_reset();
    ESP_RETURN_ON_ERROR(lcd_register_init(), TAG, "register init");
    ESP_RETURN_ON_ERROR(lcd_backlight_init(), TAG, "backlight init");

    s_lcd_ready = true;
    ESP_LOGI(TAG, "ready: %dx%d SCK=%d MOSI=%d MISO=%d CS=%d DC=%d RST=%d LED=%d BK=on",
             LCD_ILI9488_H_RES, LCD_ILI9488_V_RES,
             LCD_PIN_SCLK, LCD_PIN_MOSI, LCD_PIN_MISO,
             LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_RST, LCD_PIN_BK_LIGHT);
    return ESP_OK;
}

esp_err_t lcd_ili9488_fill_rect_rgb565(int x, int y, int w, int h, uint16_t color)
{
    if (!s_lcd_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (w <= 0 || h <= 0) {
        return ESP_OK;
    }

    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = x + w;
    int y1 = y + h;
    if (x1 > LCD_ILI9488_H_RES) {
        x1 = LCD_ILI9488_H_RES;
    }
    if (y1 > LCD_ILI9488_V_RES) {
        y1 = LCD_ILI9488_V_RES;
    }
    if (x0 >= x1 || y0 >= y1) {
        return ESP_OK;
    }

    const int width = x1 - x0;
    uint8_t *line = heap_caps_malloc(width * 3, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_RETURN_ON_FALSE(line != NULL, ESP_ERR_NO_MEM, TAG, "line buffer alloc");

    uint8_t rgb[3];
    rgb565_to_rgb666(color, rgb);
    for (int i = 0; i < width; i++) {
        line[i * 3 + 0] = rgb[0];
        line[i * 3 + 1] = rgb[1];
        line[i * 3 + 2] = rgb[2];
    }

    esp_err_t ret = ESP_OK;
    for (int row = y0; row < y1 && ret == ESP_OK; row++) {
        ret = lcd_set_window(x0, row, x1, row + 1);
        if (ret == ESP_OK) {
            ret = lcd_tx_color_sync(LCD_CMD_RAMWR, line, width * 3);
        }
    }

    free(line);
    return ret;
}

esp_err_t lcd_ili9488_fill_screen_rgb565(uint16_t color)
{
    return lcd_ili9488_fill_rect_rgb565(0, 0, LCD_ILI9488_H_RES, LCD_ILI9488_V_RES, color);
}

esp_err_t lcd_ili9488_draw_test_pattern(void)
{
    if (!s_lcd_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t *line = heap_caps_malloc(LCD_ILI9488_H_RES * 3, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_RETURN_ON_FALSE(line != NULL, ESP_ERR_NO_MEM, TAG, "test line buffer alloc");

    const uint16_t bars[] = {
        RGB565_RED, RGB565_GREEN, RGB565_BLUE, RGB565_WHITE,
        RGB565_YELLOW, RGB565_CYAN, RGB565_MAGENTA, RGB565_BLACK,
    };
    const int bar_count = (int)(sizeof(bars) / sizeof(bars[0]));
    const int bar_width = LCD_ILI9488_H_RES / bar_count;
    esp_err_t ret = ESP_OK;

    for (int y = 0; y < LCD_ILI9488_V_RES && ret == ESP_OK; y++) {
        for (int x = 0; x < LCD_ILI9488_H_RES; x++) {
            uint8_t rgb[3];
            if (y < 80) {
                int idx = x / bar_width;
                if (idx >= bar_count) {
                    idx = bar_count - 1;
                }
                rgb565_to_rgb666(bars[idx], rgb);
            } else {
                rgb[0] = (uint8_t)(((x * 255) / (LCD_ILI9488_H_RES - 1)) & 0xFC);
                rgb[1] = (uint8_t)(((y * 255) / (LCD_ILI9488_V_RES - 1)) & 0xFC);
                rgb[2] = (uint8_t)((((x + y) * 255) /
                                    (LCD_ILI9488_H_RES + LCD_ILI9488_V_RES - 2)) & 0xFC);
            }
            line[x * 3 + 0] = rgb[0];
            line[x * 3 + 1] = rgb[1];
            line[x * 3 + 2] = rgb[2];
        }

        ret = lcd_set_window(0, y, LCD_ILI9488_H_RES, y + 1);
        if (ret == ESP_OK) {
            ret = lcd_tx_color_sync(LCD_CMD_RAMWR, line, LCD_ILI9488_H_RES * 3);
        }
    }

    free(line);
    return ret;
}

typedef struct {
    int r;
    int g;
    int b;
} rgb_t;

static int clamp8(int v)
{
    if (v < 0) {
        return 0;
    }
    if (v > 255) {
        return 255;
    }
    return v;
}

static rgb_t blend_rgb(rgb_t base, rgb_t top, int alpha)
{
    alpha = clamp8(alpha);
    rgb_t out = {
        .r = (base.r * (255 - alpha) + top.r * alpha) / 255,
        .g = (base.g * (255 - alpha) + top.g * alpha) / 255,
        .b = (base.b * (255 - alpha) + top.b * alpha) / 255,
    };
    return out;
}

static void rgb888_to_rgb666(rgb_t color, uint8_t out[3])
{
    out[0] = (uint8_t)(clamp8(color.r) & 0xFC);
    out[1] = (uint8_t)(clamp8(color.g) & 0xFC);
    out[2] = (uint8_t)(clamp8(color.b) & 0xFC);
}

static int rounded_rect_alpha(int x, int y, int x0, int y0, int x1, int y1, int radius)
{
    if (x < x0 || x >= x1 || y < y0 || y >= y1) {
        return 0;
    }

    int cx = x;
    int cy = y;
    if (x < x0 + radius) {
        cx = x0 + radius;
    } else if (x >= x1 - radius) {
        cx = x1 - radius - 1;
    }
    if (y < y0 + radius) {
        cy = y0 + radius;
    } else if (y >= y1 - radius) {
        cy = y1 - radius - 1;
    }

    int dx = x - cx;
    int dy = y - cy;
    int dist2 = dx * dx + dy * dy;
    int inner = (radius - 2) * (radius - 2);
    int outer = (radius + 2) * (radius + 2);

    if (dist2 <= inner || (x >= x0 + radius && x < x1 - radius) ||
        (y >= y0 + radius && y < y1 - radius)) {
        return 255;
    }
    if (dist2 >= outer) {
        return 0;
    }
    return (outer - dist2) * 255 / (outer - inner);
}

static int ellipse_alpha(int x, int y, int cx, int cy, int rx, int ry, int feather)
{
    int dx = x - cx;
    int dy = y - cy;
    int64_t norm = (int64_t)dx * dx * ry * ry + (int64_t)dy * dy * rx * rx;
    int64_t edge = (int64_t)rx * rx * ry * ry;
    int64_t soft = edge / ((rx + ry) / 2);
    soft *= feather > 0 ? feather : 1;

    if (norm <= edge - soft) {
        return 255;
    }
    if (norm >= edge + soft) {
        return 0;
    }
    return (int)((edge + soft - norm) * 255 / (soft * 2));
}

static int ring_alpha(int alpha_outer, int alpha_inner)
{
    int a = alpha_outer - alpha_inner;
    if (a < 0) {
        return 0;
    }
    if (a > 255) {
        return 255;
    }
    return a;
}

static bool is_sparkle(int x, int y)
{
    const int points[][2] = {
        {42, 76}, {276, 88}, {52, 392}, {266, 378},
        {72, 136}, {246, 150}, {92, 430}, {230, 52},
    };
    for (int i = 0; i < (int)(sizeof(points) / sizeof(points[0])); i++) {
        int dx = abs(x - points[i][0]);
        int dy = abs(y - points[i][1]);
        if ((dx == 0 && dy <= 5) || (dy == 0 && dx <= 5) || (dx + dy <= 4)) {
            return true;
        }
    }
    return false;
}

static rgb_t render_ai_face_pixel(int x, int y)
{
    rgb_t c = {
        .r = 10 + y * 35 / LCD_ILI9488_V_RES,
        .g = 20 + y * 25 / LCD_ILI9488_V_RES,
        .b = 52 + y * 84 / LCD_ILI9488_V_RES,
    };

    int dx = x - 92;
    int dy = y - 130;
    int glow = 190 - (dx * dx + dy * dy) / 95;
    if (glow > 0) {
        c = blend_rgb(c, (rgb_t){34, 245, 222}, glow);
    }

    dx = x - 246;
    dy = y - 330;
    glow = 160 - (dx * dx + dy * dy) / 130;
    if (glow > 0) {
        c = blend_rgb(c, (rgb_t){210, 76, 255}, glow);
    }

    int panel = rounded_rect_alpha(x, y, 22, 32, 298, 448, 30);
    if (panel) {
        c = blend_rgb(c, (rgb_t){12, 35, 78}, panel * 150 / 255);
    }
    int panel_inner = rounded_rect_alpha(x, y, 27, 37, 293, 443, 26);
    int panel_border = ring_alpha(panel, panel_inner);
    if (panel_border) {
        c = blend_rgb(c, (rgb_t){92, 244, 255}, panel_border);
    }

    int face = rounded_rect_alpha(x, y, 50, 116, 270, 350, 60);
    if (face) {
        rgb_t face_color = {
            .r = 192 + (x * 26 / LCD_ILI9488_H_RES),
            .g = 250 - (y * 18 / LCD_ILI9488_V_RES),
            .b = 255,
        };
        c = blend_rgb(c, face_color, face);
    }
    int face_inner = rounded_rect_alpha(x, y, 56, 122, 264, 344, 54);
    int face_border = ring_alpha(face, face_inner);
    if (face_border) {
        c = blend_rgb(c, (rgb_t){42, 226, 255}, face_border);
    }

    int visor = rounded_rect_alpha(x, y, 70, 166, 250, 260, 34);
    if (visor) {
        rgb_t visor_color = {
            .r = 20,
            .g = 48 + (x * 50 / LCD_ILI9488_H_RES),
            .b = 92 + (y * 50 / LCD_ILI9488_V_RES),
        };
        c = blend_rgb(c, visor_color, visor * 210 / 255);
    }
    int visor_inner = rounded_rect_alpha(x, y, 75, 171, 245, 255, 29);
    int visor_border = ring_alpha(visor, visor_inner);
    if (visor_border) {
        c = blend_rgb(c, (rgb_t){105, 254, 255}, visor_border);
    }

    int left_eye = ellipse_alpha(x, y, 116, 208, 19, 27, 3);
    if (left_eye) {
        c = blend_rgb(c, (rgb_t){20, 252, 236}, left_eye);
    }
    int right_eye = ellipse_alpha(x, y, 204, 208, 19, 27, 3);
    if (right_eye) {
        c = blend_rgb(c, (rgb_t){20, 252, 236}, right_eye);
    }

    int left_eye_core = ellipse_alpha(x, y, 116, 216, 13, 14, 2);
    if (left_eye_core) {
        c = blend_rgb(c, (rgb_t){6, 68, 116}, left_eye_core * 220 / 255);
    }
    int right_eye_core = ellipse_alpha(x, y, 204, 216, 13, 14, 2);
    if (right_eye_core) {
        c = blend_rgb(c, (rgb_t){6, 68, 116}, right_eye_core * 220 / 255);
    }

    int eye_glint = ellipse_alpha(x, y, 109, 196, 5, 7, 1) |
                    ellipse_alpha(x, y, 197, 196, 5, 7, 1);
    if (eye_glint) {
        c = blend_rgb(c, (rgb_t){245, 255, 255}, eye_glint);
    }

    int cheek_l = ellipse_alpha(x, y, 84, 258, 26, 13, 4);
    if (cheek_l) {
        c = blend_rgb(c, (rgb_t){255, 122, 190}, cheek_l * 120 / 255);
    }
    int cheek_r = ellipse_alpha(x, y, 236, 258, 26, 13, 4);
    if (cheek_r) {
        c = blend_rgb(c, (rgb_t){255, 122, 190}, cheek_r * 120 / 255);
    }

    int mouth = 0;
    int mx = x - 160;
    if (mx >= -48 && mx <= 48) {
        int curve_y = 281 - (mx * mx) / 190;
        int thick = abs(y - curve_y);
        if (thick <= 5 && y > 244) {
            mouth = 255 - thick * 38;
        }
    }
    if (mouth > 0) {
        c = blend_rgb(c, (rgb_t){12, 116, 152}, mouth);
    }

    int antenna = ellipse_alpha(x, y, 160, 92, 12, 12, 2);
    if (antenna) {
        c = blend_rgb(c, (rgb_t){92, 255, 238}, antenna);
    }
    if (x >= 157 && x <= 163 && y >= 100 && y <= 124) {
        c = blend_rgb(c, (rgb_t){92, 255, 238}, 180);
    }

    int left_ear = ellipse_alpha(x, y, 46, 230, 13, 35, 3);
    if (left_ear) {
        c = blend_rgb(c, (rgb_t){42, 210, 255}, left_ear * 190 / 255);
    }
    int right_ear = ellipse_alpha(x, y, 274, 230, 13, 35, 3);
    if (right_ear) {
        c = blend_rgb(c, (rgb_t){180, 96, 255}, right_ear * 190 / 255);
    }

    int badge = rounded_rect_alpha(x, y, 102, 366, 218, 400, 17);
    if (badge) {
        c = blend_rgb(c, (rgb_t){20, 58, 105}, badge * 180 / 255);
    }
    int pulse_l = ellipse_alpha(x, y, 126, 383, 7, 7, 1);
    int pulse_m = ellipse_alpha(x, y, 160, 383, 7, 7, 1);
    int pulse_r = ellipse_alpha(x, y, 194, 383, 7, 7, 1);
    if (pulse_l || pulse_m || pulse_r) {
        c = blend_rgb(c, (rgb_t){92, 255, 238}, pulse_l | pulse_m | pulse_r);
    }

    if (is_sparkle(x, y)) {
        c = blend_rgb(c, (rgb_t){242, 255, 255}, 210);
    }

    return c;
}

esp_err_t lcd_ili9488_draw_ai_face(void)
{
    if (!s_lcd_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t *line = heap_caps_malloc(LCD_ILI9488_H_RES * 3, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_RETURN_ON_FALSE(line != NULL, ESP_ERR_NO_MEM, TAG, "ai face line buffer alloc");

    esp_err_t ret = ESP_OK;
    for (int y = 0; y < LCD_ILI9488_V_RES && ret == ESP_OK; y++) {
        for (int x = 0; x < LCD_ILI9488_H_RES; x++) {
            rgb888_to_rgb666(render_ai_face_pixel(x, y), &line[x * 3]);
        }

        ret = lcd_set_window(0, y, LCD_ILI9488_H_RES, y + 1);
        if (ret == ESP_OK) {
            ret = lcd_tx_color_sync(LCD_CMD_RAMWR, line, LCD_ILI9488_H_RES * 3);
        }
    }

    free(line);
    return ret;
}

esp_err_t lcd_ili9488_draw_rgb565_bitmap(int x, int y, int w, int h, const uint16_t *data)
{
    if (!s_lcd_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || w <= 0 || h <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (x < 0 || y < 0 || x + w > LCD_ILI9488_H_RES || y + h > LCD_ILI9488_V_RES) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *line = heap_caps_malloc((size_t)w * 3, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_RETURN_ON_FALSE(line != NULL, ESP_ERR_NO_MEM, TAG, "rgb565 line buffer alloc");

    esp_err_t ret = ESP_OK;
    for (int row = 0; row < h && ret == ESP_OK; row++) {
        const uint16_t *src = data + ((size_t)row * w);
        for (int col = 0; col < w; col++) {
            rgb565_to_rgb666(src[col], &line[col * 3]);
        }

        ret = lcd_set_window(x, y + row, x + w, y + row + 1);
        if (ret == ESP_OK) {
            ret = lcd_tx_color_sync(LCD_CMD_RAMWR, line, (size_t)w * 3);
        }
    }

    free(line);
    return ret;
}

esp_err_t lcd_ili9488_draw_rgb666_image(int x, int y, int w, int h, const uint8_t *data)
{
    if (!s_lcd_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || w <= 0 || h <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (x < 0 || y < 0 || x + w > LCD_ILI9488_H_RES || y + h > LCD_ILI9488_V_RES) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(lcd_set_window(x, y, x + w, y + h), TAG, "set rgb666 window");
    return lcd_tx_color_sync(LCD_CMD_RAMWR, data, (size_t)w * h * 3);
}
