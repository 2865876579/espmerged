#include "led_strip_driver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/*
 * GPIO33~37 can be used by Octal PSRAM/Flash on ESP32-S3 N16R8 modules.
 * GPIO47/48 are reserved for the mmWave radar UART. GPIO40 is safe for the LED
 * strip only when the LCD SDO/MISO wire is left physically disconnected.
 */
#define LED_STRIP_GPIO                 GPIO_NUM_40
#define LED_STRIP_DEFAULT_BRIGHTNESS   56
#define LED_STRIP_DEFAULT_SPEED_PCT    30
#define LED_EFFECT_FRAME_MS            120
#define LED_STRIP_RESET_US             300
#define LED_STRIP_CLEAR_FRAMES         3
#define LED_STRIP_CLEAR_EXTRA_PIXELS   8

/*
 * The user's strip is wired as 5V / DIN / GND and currently behaves like a
 * 24-bit WS2812/SK6812 RGB strip. Sending RGBW 32-bit frames makes later LEDs
 * decode the byte stream out of phase, which shows up as first LED green and
 * the rest white/random.
 */
#define LED_STRIP_USE_RGBW             0
#if LED_STRIP_USE_RGBW
#define LED_STRIP_BYTES_PER_PIXEL      4
#define LED_STRIP_FORMAT_NAME          "GRBW"
#else
#define LED_STRIP_BYTES_PER_PIXEL      3
#define LED_STRIP_FORMAT_NAME          "GRB"
#endif

#define LED_STRIP_FRAME_PIXELS         (LED_STRIP_COUNT + LED_STRIP_CLEAR_EXTRA_PIXELS)
#define LED_STRIP_FRAME_BYTES          (LED_STRIP_FRAME_PIXELS * LED_STRIP_BYTES_PER_PIXEL)
#define LED_STRIP_RMT_RESOLUTION_HZ    10000000

static const char *TAG = "led_strip";

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

static const rgb_t s_default_pixel = { .r = 255, .g = 208, .b = 150 };

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_effect_task;
static rmt_channel_handle_t s_rmt_channel;
static rmt_encoder_handle_t s_rmt_encoder;
static bool s_ready;
static bool s_enabled = true;
static uint8_t s_brightness = LED_STRIP_DEFAULT_BRIGHTNESS;
static led_strip_effect_t s_effect = LED_STRIP_EFFECT_SOLID;
static uint8_t s_speed_pct = LED_STRIP_DEFAULT_SPEED_PCT;
static uint32_t s_duration_ms;
static uint32_t s_effect_start_ms;
static rgb_t s_pixel = { .r = 255, .g = 208, .b = 150 };
static uint8_t s_frame[LED_STRIP_FRAME_BYTES];

static inline uint32_t tick_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint8_t scale_brightness(uint8_t value, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)value * brightness) / 255);
}

static void write_pixel_bytes(uint8_t *dst, rgb_t pixel, uint8_t brightness)
{
    const uint8_t r = scale_brightness(pixel.r, brightness);
    const uint8_t g = scale_brightness(pixel.g, brightness);
    const uint8_t b = scale_brightness(pixel.b, brightness);

#if LED_STRIP_USE_RGBW
    dst[0] = g;
    dst[1] = r;
    dst[2] = b;
    dst[3] = 0;
#else
    dst[0] = g;
    dst[1] = r;
    dst[2] = b;
#endif
}

static void fill_frame(rgb_t pixel, uint8_t brightness, int pixel_count)
{
    if (pixel_count > LED_STRIP_FRAME_PIXELS) {
        pixel_count = LED_STRIP_FRAME_PIXELS;
    }

    for (int i = 0; i < pixel_count; ++i) {
        write_pixel_bytes(&s_frame[i * LED_STRIP_BYTES_PER_PIXEL], pixel, brightness);
    }

    const size_t used = (size_t)pixel_count * LED_STRIP_BYTES_PER_PIXEL;
    if (used < sizeof(s_frame)) {
        memset(&s_frame[used], 0, sizeof(s_frame) - used);
    }
}

static esp_err_t transmit_frame_locked(size_t byte_count)
{
    if (!s_rmt_channel || !s_rmt_encoder) {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };

    ESP_RETURN_ON_ERROR(rmt_encoder_reset(s_rmt_encoder), TAG, "rmt encoder reset failed");
    ESP_RETURN_ON_ERROR(rmt_transmit(s_rmt_channel, s_rmt_encoder, s_frame, byte_count, &tx_config),
                        TAG, "rmt transmit failed");
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(s_rmt_channel, 100), TAG, "rmt wait failed");

    esp_rom_delay_us(LED_STRIP_RESET_US);
    return ESP_OK;
}

static esp_err_t led_strip_clear_locked(void)
{
    memset(s_frame, 0, sizeof(s_frame));
    const size_t byte_count = LED_STRIP_FRAME_BYTES;

    esp_err_t ret = ESP_OK;
    for (int repeat = 0; repeat < LED_STRIP_CLEAR_FRAMES; ++repeat) {
        ret = transmit_frame_locked(byte_count);
        if (ret != ESP_OK) {
            break;
        }
    }
    esp_rom_delay_us(LED_STRIP_RESET_US);
    return ret;
}

static esp_err_t led_strip_show_pixel_locked(rgb_t pixel, uint8_t brightness)
{
    if (brightness == 0) {
        return led_strip_clear_locked();
    }

    fill_frame(pixel, brightness, LED_STRIP_COUNT);
    return transmit_frame_locked(LED_STRIP_COUNT * LED_STRIP_BYTES_PER_PIXEL);
}

static rgb_t color_wheel(uint16_t pos)
{
    pos %= 1536;
    const uint8_t seg = pos / 256;
    const uint8_t x = pos & 0xff;

    switch (seg) {
    case 0: return (rgb_t){ .r = 255, .g = x, .b = 0 };
    case 1: return (rgb_t){ .r = 255 - x, .g = 255, .b = 0 };
    case 2: return (rgb_t){ .r = 0, .g = 255, .b = x };
    case 3: return (rgb_t){ .r = 0, .g = 255 - x, .b = 255 };
    case 4: return (rgb_t){ .r = x, .g = 0, .b = 255 };
    default: return (rgb_t){ .r = 255, .g = 0, .b = 255 - x };
    }
}

static uint32_t speed_to_period_ms(uint8_t speed_pct, uint32_t slow_ms, uint32_t fast_ms)
{
    if (speed_pct > 100) {
        speed_pct = 100;
    }
    return slow_ms - (((slow_ms - fast_ms) * speed_pct) / 100);
}

static void render_effect_frame_locked(uint32_t now)
{
    if (!s_enabled || s_brightness == 0) {
        return;
    }

    if (s_duration_ms > 0 && (uint32_t)(now - s_effect_start_ms) >= s_duration_ms) {
        s_effect = LED_STRIP_EFFECT_SOLID;
        s_duration_ms = 0;
        (void)led_strip_show_pixel_locked(s_pixel, s_brightness);
        return;
    }

    const uint32_t elapsed = now - s_effect_start_ms;
    rgb_t pixel = s_pixel;
    uint8_t brightness = s_brightness;

    switch (s_effect) {
    case LED_STRIP_EFFECT_BLINK: {
        const uint32_t half_period = speed_to_period_ms(s_speed_pct, 850, 140);
        brightness = ((elapsed / half_period) & 0x01) ? 0 : s_brightness;
        break;
    }
    case LED_STRIP_EFFECT_BREATH: {
        const uint32_t period = speed_to_period_ms(s_speed_pct, 4200, 1200);
        const uint32_t phase = elapsed % period;
        const uint32_t half = period / 2;
        uint32_t wave = phase < half ? phase : (period - phase);
        if (half > 0) {
            wave = (wave * 255) / half;
        }
        const uint8_t min_brightness = s_brightness > 8 ? 2 : 0;
        brightness = (uint8_t)(min_brightness +
                               (((uint16_t)(s_brightness - min_brightness) * wave) / 255));
        break;
    }
    case LED_STRIP_EFFECT_GRADIENT: {
        const uint32_t period = speed_to_period_ms(s_speed_pct, 18000, 4500);
        const uint32_t phase = elapsed % period;
        pixel = color_wheel((uint16_t)((phase * 1536UL) / period));
        break;
    }
    case LED_STRIP_EFFECT_SOLID:
    default:
        return;
    }

    (void)led_strip_show_pixel_locked(pixel, brightness);
}

static void led_effect_task(void *arg)
{
    (void)arg;
    while (true) {
        if (s_ready && s_lock) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            const bool active_effect = s_enabled &&
                                       s_effect != LED_STRIP_EFFECT_SOLID &&
                                       s_brightness > 0;
            if (active_effect) {
                render_effect_frame_locked(tick_ms());
            }
            xSemaphoreGive(s_lock);
            vTaskDelay(pdMS_TO_TICKS(active_effect ? LED_EFFECT_FRAME_MS : 350));
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

esp_err_t led_strip_driver_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "create mutex failed");

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_STRIP_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = LED_STRIP_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 2,
        .flags.invert_out = false,
        .flags.with_dma = false,
        .flags.io_loop_back = false,
        .flags.io_od_mode = false,
        .intr_priority = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &s_rmt_channel), TAG,
                        "create rmt tx channel failed");
    gpio_set_drive_capability(LED_STRIP_GPIO, GPIO_DRIVE_CAP_3);

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 4,
            .level1 = 0,
            .duration1 = 9,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 8,
            .level1 = 0,
            .duration1 = 5,
        },
        .flags.msb_first = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &s_rmt_encoder), TAG,
                        "create rmt bytes encoder failed");
    ESP_RETURN_ON_ERROR(rmt_enable(s_rmt_channel), TAG, "enable rmt failed");

    s_enabled = true;
    s_brightness = LED_STRIP_DEFAULT_BRIGHTNESS;
    s_effect = LED_STRIP_EFFECT_SOLID;
    s_speed_pct = LED_STRIP_DEFAULT_SPEED_PCT;
    s_duration_ms = 0;
    s_effect_start_ms = tick_ms();
    s_pixel = s_default_pixel;
    ESP_RETURN_ON_ERROR(led_strip_show_pixel_locked(s_pixel, s_brightness), TAG,
                        "show default led frame failed");

    s_ready = true;
    BaseType_t task_ok = xTaskCreate(led_effect_task, "led_effect", 3072, NULL, 3, &s_effect_task);
    ESP_RETURN_ON_FALSE(task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create effect task failed");

    ESP_LOGI(TAG, "ready: GPIO=%d count=%d brightness=%u/255 format=%s default=on rmt",
             LED_STRIP_GPIO, LED_STRIP_COUNT, s_brightness, LED_STRIP_FORMAT_NAME);
    return ESP_OK;
}

esp_err_t led_strip_set_enabled(bool enabled)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (enabled && s_brightness == 0) {
        s_brightness = LED_STRIP_DEFAULT_BRIGHTNESS;
        s_pixel = s_default_pixel;
    } else if (!enabled) {
        s_brightness = 0;
        s_pixel = s_default_pixel;
    }
    s_enabled = enabled && s_brightness > 0;
    s_effect = LED_STRIP_EFFECT_SOLID;
    s_duration_ms = 0;
    esp_err_t ret = led_strip_show_pixel_locked(s_enabled ? s_pixel : (rgb_t){ 0 },
                                                s_enabled ? s_brightness : 0);
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "power=%s brightness=%u/255 ret=%d",
             s_enabled ? "on" : "off", s_brightness, ret);
    return ret;
}

esp_err_t led_strip_set_brightness(uint8_t brightness)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_brightness = brightness;
    s_enabled = brightness > 0;
    s_effect = LED_STRIP_EFFECT_SOLID;
    s_duration_ms = 0;
    esp_err_t ret = led_strip_show_pixel_locked(s_enabled ? s_pixel : (rgb_t){ 0 },
                                                s_enabled ? s_brightness : 0);
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "brightness=%u/255 power=%s ret=%d",
             s_brightness, s_enabled ? "on" : "off", ret);
    return ret;
}

esp_err_t led_strip_apply(bool enabled, uint8_t brightness)
{
    led_strip_config_t config = {
        .enabled = enabled && brightness > 0,
        .brightness = brightness,
        .effect = LED_STRIP_EFFECT_SOLID,
        .speed_pct = s_speed_pct,
        .duration_ms = 0,
        .r = s_pixel.r,
        .g = s_pixel.g,
        .b = s_pixel.b,
    };
    return led_strip_apply_effect(&config);
}

esp_err_t led_strip_apply_effect(const led_strip_config_t *config)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_enabled = config->enabled && config->brightness > 0;
    s_brightness = s_enabled ? config->brightness : 0;
    s_effect = config->effect;
    s_speed_pct = config->speed_pct > 100 ? 100 : config->speed_pct;
    s_duration_ms = config->duration_ms;
    s_effect_start_ms = tick_ms();
    s_pixel = (rgb_t){ .r = config->r, .g = config->g, .b = config->b };
    if (s_pixel.r == 0 && s_pixel.g == 0 && s_pixel.b == 0 && s_enabled) {
        s_pixel = s_default_pixel;
    }

    esp_err_t ret;
    if (!s_enabled) {
        s_effect = LED_STRIP_EFFECT_SOLID;
        s_pixel = s_default_pixel;
        ret = led_strip_show_pixel_locked((rgb_t){ 0 }, 0);
    } else {
        ret = led_strip_show_pixel_locked(s_pixel, s_brightness);
    }
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "apply power=%s brightness=%u/255 effect=%d speed=%u duration=%lu color=%u,%u,%u ret=%d",
             s_enabled ? "on" : "off",
             s_brightness,
             (int)s_effect,
             s_speed_pct,
             (unsigned long)s_duration_ms,
             s_pixel.r, s_pixel.g, s_pixel.b,
             ret);
    return ret;
}

void led_strip_get_state(bool *enabled, uint8_t *brightness)
{
    led_strip_get_effect_state(enabled, brightness, NULL, NULL, NULL, NULL, NULL);
}

void led_strip_get_effect_state(bool *enabled, uint8_t *brightness,
                                led_strip_effect_t *effect, uint8_t *speed_pct,
                                uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    if (enabled) {
        *enabled = s_enabled;
    }
    if (brightness) {
        *brightness = s_brightness;
    }
    if (effect) {
        *effect = s_effect;
    }
    if (speed_pct) {
        *speed_pct = s_speed_pct;
    }
    if (r) {
        *r = s_pixel.r;
    }
    if (g) {
        *g = s_pixel.g;
    }
    if (b) {
        *b = s_pixel.b;
    }
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}
