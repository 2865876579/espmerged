#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define LED_STRIP_GPIO_NUM 40
#define LED_STRIP_COUNT    60

typedef enum {
    LED_STRIP_EFFECT_SOLID = 0,
    LED_STRIP_EFFECT_BLINK,
    LED_STRIP_EFFECT_BREATH,
    LED_STRIP_EFFECT_GRADIENT,
} led_strip_effect_t;

typedef struct {
    bool enabled;
    uint8_t brightness;
    led_strip_effect_t effect;
    uint8_t speed_pct;
    uint32_t duration_ms;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_strip_config_t;

esp_err_t led_strip_driver_init(void);
esp_err_t led_strip_set_enabled(bool enabled);
esp_err_t led_strip_set_brightness(uint8_t brightness);
esp_err_t led_strip_apply(bool enabled, uint8_t brightness);
esp_err_t led_strip_apply_effect(const led_strip_config_t *config);
void led_strip_get_state(bool *enabled, uint8_t *brightness);
void led_strip_get_effect_state(bool *enabled, uint8_t *brightness,
                                led_strip_effect_t *effect, uint8_t *speed_pct,
                                uint8_t *r, uint8_t *g, uint8_t *b);
