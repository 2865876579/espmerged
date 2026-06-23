#ifndef KY022_H
#define KY022_H

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

/**
 * KY-022 红外接收器配置
 */
typedef struct {
    gpio_num_t gpio_num;                /* GPIO 引脚号 */
    uint32_t resolution_hz;             /* RMT 分辨率 (Hz)，默认 1 MHz */
    uint32_t signal_range_min_ns;       /* 最小信号脉冲宽度 (ns)，默认 1250 */
    uint32_t signal_range_max_ns;       /* 最大信号脉冲宽度 (ns)，默认 12 ms */
} ky022_config_t;

/**
 * NEC 协议解码帧结构
 */
typedef struct {
    uint8_t address;                    /* 地址码 */
    uint8_t command;                    /* 命令码 */
    uint32_t raw_data;                  /* 32 位原始数据 */
} ky022_nec_frame_t;

/* KY-022 默认配置宏 */
#define KY022_DEFAULT_CONFIG(gpio)                 \
    {                                              \
        .gpio_num = (gpio),                        \
        .resolution_hz = 1000000,                  \
        .signal_range_min_ns = 1250,               \
        .signal_range_max_ns = 20000000,           \
    }

/**
 * 初始化 KY-022 红外接收器
 * @param config 配置参数，不可为 NULL
 * @return ESP_OK 成功，否则失败
 */
esp_err_t ky022_init(const ky022_config_t *config);

/**
 * 接收 NEC 协议红外帧（阻塞等待）
 * @param frame      输出解码后的帧数据
 * @param timeout_ms 超时时间 (ms)
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时，ESP_ERR_INVALID_RESPONSE 解码失败
 */
esp_err_t ky022_receive_nec(ky022_nec_frame_t *frame, uint32_t timeout_ms);

/**
 * 反初始化 KY-022，释放资源
 * @return ESP_OK
 */
esp_err_t ky022_deinit(void);

#endif /* KY022_H */
