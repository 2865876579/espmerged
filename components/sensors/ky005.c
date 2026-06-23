/**
 * @file ky005.c
 * @brief KY-005 红外发射器驱动实现
 *
 * 本文件实现了 KY-005 红外发射模块基于 ESP32 RMT（远程控制）外设的驱动。
 * 支持 NEC 协议的红外信号发送，包括数据帧和重复码。
 *
 * NEC 协议时序：
 *   - 引导码: 9ms 高 + 4.5ms 低
 *   - 数据 "0": 560us 高 + 560us 低
 *   - 数据 "1": 560us 高 + 1690us 低
 *   - 停止位: 560us 高
 *   - 重复码: 9ms 高 + 2.25ms 低
 */

#include "ky005.h"
#include <stdbool.h>
#include <string.h>
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"

/* ======================== NEC 协议时序常量 ======================== */
//这些值都是这些值是 NEC 协议规范
#define NEC_LEADING_CODE_HIGH_US  9000   /*!< 引导码高电平持续时间 (微秒) */
#define NEC_LEADING_CODE_LOW_US   4500   /*!< 引导码低电平持续时间 (微秒) */
#define NEC_REPEAT_CODE_LOW_US    2250   /*!< 重复码低电平持续时间 (微秒) */
#define NEC_PAYLOAD_HIGH_US       560    /*!< 数据位高电平持续时间 (微秒) */
#define NEC_PAYLOAD_ZERO_LOW_US   560    /*!< 数据位 "0" 低电平持续时间 (微秒) */
#define NEC_PAYLOAD_ONE_LOW_US    1690   /*!< 数据位 "1" 低电平持续时间 (微秒) */
#define NEC_STOP_HIGH_US          560    /*!< 停止位高电平持续时间 (微秒) */

/* ======================== 静态变量 ======================== */

static const char *TAG = "KY005";                    /*!< 日志标签 */

static rmt_channel_handle_t s_tx_channel    = NULL;  /*!< RMT TX 通道句柄 */
static rmt_encoder_handle_t s_copy_encoder  = NULL;  /*!< RMT 拷贝编码器句柄 */
static uint32_t s_resolution_hz             = 0;     /*!< RMT 时钟分辨率 (Hz) */

/* ======================== 工具函数 ======================== */

/**
 * @brief 将微秒值转换为 RMT 硬件时钟滴答数
 *
 * RMT 外设使用内部时钟计数，需要根据分辨率将时间（微秒）转换为时钟滴答数。
 * 计算公式: ticks = us * resolution_hz / 1000000
 *
 * @param us 时间值 (微秒)
 * @return 对应的 RMT 时钟滴答数
 */
static uint32_t us_to_ticks(uint32_t us)
{
    return (uint32_t)(((uint64_t)us * s_resolution_hz) / 1000000ULL);
}

/**
 * @brief 构造一个 RMT 符号（Symbol）数据
 *
 * RMT 的 symbol_word_t 结构由两段组成，每段包含电平极性（level）和持续时间（duration）：
 *   段0: level0 + duration0   —— 高电平持续一段时间
 *   段1: level1 + duration1   —— 之后切换为低电平持续一段时间
 * NEC 协议中的每一位（包括引导码、数据位、停止位）都由一次高-低电平切换构成，
 * 因此可以映射为一个 RMT symbol。
 *
 * @param high_us 高电平持续时间 (微秒)
 * @param low_us  低电平持续时间 (微秒)，为 0 表示最后不拉低
 * @return 构造好的 rmt_symbol_word_t 结构体
 */
static rmt_symbol_word_t make_symbol(uint32_t high_us, uint32_t low_us)
{
    return (rmt_symbol_word_t) {
        .level0 = 1,                                    /* 段0为高电平 */
        .duration0 = us_to_ticks(high_us),              /* 高电平持续时间 */
        .level1 = 0,                                    /* 段1为低电平 */
        .duration1 = us_to_ticks(low_us),               /* 低电平持续时间 */
    };
}

/* ======================== 初始化 / 反初始化 ======================== */

/**
 * @brief 初始化 KY-005 红外发射器
 *
 * 初始化流程：
 *   1. 校验传入参数是否合法（config 非空、GPIO 有效）
 *   2. 如果已经初始化过，直接返回 ESP_OK（防重复初始化）
 *   3. 创建 RMT TX 通道，绑定到指定 GPIO
 *   4. 配置 38kHz 载波调制（红外遥控标准载波频率）
 *   5. 创建拷贝编码器（将原始 symbol 数据编码为 RMT 可发送的格式）
 *   6. 使能 RMT TX 通道，开始工作
 *
 * @param config KY-005 配置参数指针（GPIO、分辨率、载波频率等）
 * @return ESP_OK       成功
 * @return ESP_ERR_INVALID_ARG  参数无效（config 为空或 GPIO 号非法）
 * @return 其他         RMT 硬件配置失败的错误码
 */
esp_err_t ky005_init(const ky005_config_t *config)
{
    /* ---------- 参数校验 ---------- */
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config 为空");
    esp_err_t ret = ESP_OK;  /* 用于 ESP_GOTO_ON_ERROR 宏 */
    ESP_RETURN_ON_FALSE(config->gpio_num >= 0, ESP_ERR_INVALID_ARG, TAG, "无效 GPIO");

    /* 已初始化则跳过（防止重复创建通道造成资源泄漏） */
    if (s_tx_channel) {
        return ESP_OK;
    }

    /* ---------- 参数默认值 ---------- */
    s_resolution_hz = config->resolution_hz;//结构体在 ky005.h 中定义了默认值，如果用户没有传入，就使用默认值
    uint32_t carrier_hz = config->carrier_hz;
    float duty = config->carrier_duty_percent;

    /* ---------- 步骤1: 创建 RMT TX 通道 ---------- */
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = config->gpio_num,           /* 发射 GPIO 引脚 */
        .clk_src = RMT_CLK_SRC_DEFAULT,         /* 默认时钟源，通常为 80 MHz APB 时钟 */
        .resolution_hz = s_resolution_hz,       /* 分辨率，影响 timing 精度 */
        .mem_block_symbols = 64,                /* 硬件内存块大小 (symbol 数量) */
        .trans_queue_depth = 4,                 /* 发送事务队列深度 */
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_config, &s_tx_channel),
                        TAG, "创建 TX 通道失败");

    /* ---------- 步骤2: 配置 38kHz 载波 ----------
     * 红外发射需要将信号调制在 38kHz 载波上（标准 NEC 协议），
     * 以增加抗干扰能力并降低功耗。 */
    rmt_carrier_config_t carrier_config = {
        .frequency_hz = carrier_hz,             /* 载波频率，标准 NEC 使用 38kHz */
        .duty_cycle = duty / 100.0f,            /* 占空比，典型值 33% ~ 50% */
        .flags.polarity_active_low = false,     /* 载波极性：高电平有效 */
    };
    ESP_GOTO_ON_ERROR(rmt_apply_carrier(s_tx_channel, &carrier_config),
                      err, TAG, "应用载波失败");

    /* ---------- 步骤3: 创建拷贝编码器 ----------
     * 拷贝编码器将我们构造好的 symbol 数组直接拷贝到 RMT 硬件发送缓冲区，
     * 不做额外编码变换，适用于原始数据就是最终发送波形的场景。 */
    rmt_copy_encoder_config_t encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&encoder_config, &s_copy_encoder),
                      err, TAG, "创建编码器失败");

    /* ---------- 步骤4: 使能 TX 通道 ---------- */
    ESP_GOTO_ON_ERROR(rmt_enable(s_tx_channel),
                      err, TAG, "使能 TX 通道失败");

    ESP_LOGI(TAG, "KY-005 已就绪，GPIO%d，载波 %lu Hz", config->gpio_num, carrier_hz);
    return ESP_OK;

    /* ---------- 错误处理 ----------
     * 任何一步失败都要回滚已创建的资源，避免泄漏。 */
err:
    ky005_deinit();
    return ESP_FAIL;
}

/* ======================== NEC 发送函数 ======================== */

/**
 * @brief 发送 NEC 协议红外数据帧
 *
 * NEC 协议数据帧格式（LSB 优先）：
 *   ┌──────────┬──────────────┬─────────────────┬─────────────────┬────────┐
 *   │ 引导码   │ 地址码 (8bit)│ 地址反码 (8bit)  │ 命令码 (8bit)   │ 停止位 │
 *   │ 9ms+4.5ms│ ~address     │                  │ ~command        │ 560us  │
 *   └──────────┴──────────────┴─────────────────┴─────────────────┴────────┘
 *   总计 34 个符号：1 引导码 + 32 数据位 + 1 停止位
 *
 * @param address 8 位地址码
 * @param command 8 位命令码
 * @return ESP_OK 发送成功
 * @return ESP_ERR_INVALID_STATE 未初始化
 * @return 其他 RMT 发送失败的错误码
 */
esp_err_t ky005_send_nec(uint8_t address, uint8_t command)
{
    ESP_RETURN_ON_FALSE(s_tx_channel && s_copy_encoder,
                        ESP_ERR_INVALID_STATE, TAG, "KY-005 未初始化");

    rmt_symbol_word_t symbols[34];  /* 34 = 1 引导码 + 32 数据位 + 1 停止位 */
    size_t index = 0;

    /* 组装 32 位 NEC 帧：
     *   bit[7:0]   = address       (地址码)
     *   bit[15:8]  = ~address      (地址反码，用于校验)
     *   bit[23:16] = command       (命令码)
     *   bit[31:24] = ~command      (命令反码，用于校验) */
    uint32_t frame = ((uint32_t)address) |
                     ((uint32_t)(~address & 0xFF) << 8) |
                     ((uint32_t)command << 16) |
                     ((uint32_t)(~command & 0xFF) << 24);

    /* --- 发送引导码：9ms 高 + 4.5ms 低，通知接收端开始接收 --- */
    symbols[index++] = make_symbol(NEC_LEADING_CODE_HIGH_US, NEC_LEADING_CODE_LOW_US);

    /* --- 发送 32 位数据（LSB 优先） --- */
    for (int bit = 0; bit < 32; bit++) {
        bool one = frame & (1UL << bit);        /* 逐位判断，低位先发 */
        symbols[index++] = make_symbol(NEC_PAYLOAD_HIGH_US,
                                       one ? NEC_PAYLOAD_ONE_LOW_US   /* "1": 560us + 1690us */
                                           : NEC_PAYLOAD_ZERO_LOW_US); /* "0": 560us + 560us  */
    }

    /* --- 发送停止位：560us 高电平（低电平为 0，表示结束） --- */
    symbols[index++] = make_symbol(NEC_STOP_HIGH_US, 0);

    /* --- 通过 RMT 发送 symbol 数组 --- */
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,    /* 不循环，只发送一次 */
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(s_tx_channel, s_copy_encoder, symbols,
                                     index * sizeof(symbols[0]), &transmit_config),
                        TAG, "发送 NEC 帧失败");

    /* 等待发送完成（最长 100ms），确保数据全部发出后再返回 */
    return rmt_tx_wait_all_done(s_tx_channel, 100);
}

/**
 * @brief 发送 NEC 协议重复码
 *
 * 当遥控器按键持续按住时，NEC 协议要求在第一次完整帧发送后，
 * 每隔约 110ms 发送一个重复码，告诉接收端按键仍在按下。
 * 重复码只有 2 个符号：9ms 高 + 2.25ms 低 + 560us 高（停止位）。
 *
 * @return ESP_OK 发送成功
 * @return ESP_ERR_INVALID_STATE 未初始化
 * @return 其他 RMT 发送失败的错误码
 */
esp_err_t ky005_send_nec_repeat(void)
{
    ESP_RETURN_ON_FALSE(s_tx_channel && s_copy_encoder,
                        ESP_ERR_INVALID_STATE, TAG, "KY-005 未初始化");

    /* 重复码格式：9ms 高 + 2.25ms 低 + 停止位 (560us 高) */
    rmt_symbol_word_t symbols[] = {
        make_symbol(NEC_LEADING_CODE_HIGH_US, NEC_REPEAT_CODE_LOW_US),
        make_symbol(NEC_STOP_HIGH_US, 0),
    };

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(s_tx_channel, s_copy_encoder, symbols,
                                     sizeof(symbols), &transmit_config),
                        TAG, "发送 NEC 重复码失败");

    return rmt_tx_wait_all_done(s_tx_channel, 100);
}

/**
 * @brief 反初始化 KY-005，释放所有占用的资源
 *
 * 按照创建顺序的逆序释放资源：
 *   1. 先禁用通道（停止发送）
 *   2. 删除编码器
 *   3. 删除 RMT TX 通道
 *   4. 重置全局状态变量
 *
 * @return ESP_OK 始终返回成功
 */
esp_err_t ky005_deinit(void)
{
    /* 逆序释放：先禁能通道，再删编码器，最后删通道 */
    if (s_tx_channel) {
        rmt_disable(s_tx_channel);              /* 停止 RMT 发送 */
    }
    if (s_copy_encoder) {
        rmt_del_encoder(s_copy_encoder);        /* 删除拷贝编码器 */
        s_copy_encoder = NULL;
    }
    if (s_tx_channel) {
        rmt_del_channel(s_tx_channel);          /* 删除 TX 通道 */
        s_tx_channel = NULL;
    }
    s_resolution_hz = 0;                        /* 重置分辨率 */

    ESP_LOGI(TAG, "KY-005 已反初始化");
    return ESP_OK;
}
