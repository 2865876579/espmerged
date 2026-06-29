#ifndef KY005_H
#define KY005_H

#include <stdint.h>
#include <stddef.h>
#include "driver/gpio.h"
#include "esp_err.h"

/**
 * KY-005 红外发射器配置
 */
typedef struct {
    gpio_num_t gpio_num;               /* GPIO 引脚号 */
    uint32_t resolution_hz;            /* RMT 分辨率 (Hz)，默认 1 MHz */
    uint32_t carrier_hz;               /* 载波频率 (Hz)，默认 38 kHz */
    float carrier_duty_percent;        /* 载波占空比 (%)，默认 33% */
} ky005_config_t;

/* KY-005 默认配置宏 */
#define KY005_DEFAULT_CONFIG(gpio)                 \
    (ky005_config_t) {                             \
        .gpio_num = (gpio),                        \
        .resolution_hz = 1000000,                  \
        .carrier_hz = 38000,                       \
        .carrier_duty_percent = 33.0f,             \
    }

/**
 * 初始化 KY-005 红外发射器
 * @param config 配置参数，不可为 NULL
 * @return ESP_OK 成功，否则失败
 */
esp_err_t ky005_init(const ky005_config_t *config);

/**
 * 发送 NEC 协议红外帧
 * @param address 地址码
 * @param command 命令码
 * @return ESP_OK 成功，否则失败
 */
esp_err_t ky005_send_nec(uint8_t address, uint8_t command);

/**
 * 发送 NEC 重复码
 * @return ESP_OK 成功，否则失败
 */
esp_err_t ky005_send_nec_repeat(void);

/**
 * Send a raw IR frame as high/low duration pairs in microseconds.
 * durations format: high1, low1, high2, low2...
 */
esp_err_t ky005_send_raw(const uint32_t *durations, size_t num_pairs);

/**
 * 反初始化 KY-005，释放资源
 * @return ESP_OK
 */
esp_err_t ky005_deinit(void);

#endif /* KY005_H */
