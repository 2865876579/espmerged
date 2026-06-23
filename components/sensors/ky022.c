/**
 * @file ky022.c
 * @brief KY-022 红外接收器驱动实现
 *
 * 本文件实现了 KY-022 红外接收模块基于 ESP32 RMT（远程控制）外设的驱动。
 * 支持 NEC 协议的红外信号接收与解码。
 *
 * 工作流程：
 *   1. 通过 RMT RX 通道捕获红外信号的脉冲宽度（symbol）
 *   2. 在 ISR 中通过队列通知任务层数据已到达
 *   3. 解码函数解析 symbol 序列，提取地址码和命令码
 *   4. 校验地址/命令的反码，确保数据完整性
 */

#include "ky022.h"

#include <stdbool.h>
#include <string.h>

#include "driver/rmt_rx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* ======================== 常量定义 ======================== */

#define RX_SYMBOL_BUFFER_SIZE  80   /*!< RMT 接收缓冲区符号数（足够容纳一帧 NEC + 前导噪声） */
#define NEC_BITS               32   /*!< NEC 协议有效数据位数（8 地址 + 8 地址反码 + 8 命令 + 8 命令反码） */

/* ======================== 静态变量 ======================== */

static const char *TAG = "KY022";                                    /*!< 日志标签 */

static rmt_channel_handle_t s_rx_channel          = NULL;            /*!< RMT RX 通道句柄 */
static QueueHandle_t s_rx_queue                   = NULL;            /*!< ISR 到任务的同步队列 */
static rmt_symbol_word_t s_rx_symbols[RX_SYMBOL_BUFFER_SIZE];       /*!< RMT 接收缓冲区 */
static uint32_t s_resolution_hz                    = 0;              /*!< RMT 时钟分辨率 (Hz) */
static rmt_receive_config_t s_receive_config       = {0};            /*!< RMT 接收配置 */

/**
 * @brief RMT 接收完成事件，通过该结构从 ISR 向任务传递收到的符号数量
 */
typedef struct {
    size_t num_symbols;    /*!< 本次接收到的 RMT 符号数 */
} ky022_rx_event_t;

/* ======================== 工具函数 ======================== */

/**
 * @brief 判断实际测量值是否在目标值的 ±30% 容差范围内
 *
 * 红外接收受发光管老化、环境光干扰、距离等因素影响，脉冲宽度会存在偏差。
 * 使用 ±30% 的容差窗口来判断测量值是否匹配理论值，提高解码的鲁棒性。
 *
 * @param value  实际测量值（微秒或滴答数）
 * @param target 理论标准值
 * @return true  在容差范围内
 * @return false 超出容差范围
 */
static bool within(uint32_t value, uint32_t target)
{
    uint32_t low  = target * 7 / 10;    /* target * 0.7 */
    uint32_t high = target * 13 / 10;   /* target * 1.3 */
    return value >= low && value <= high;
}

/**
 * @brief 将 RMT 硬件时钟滴答数转换为微秒
 *
 * 与 ky005.c 中的 us_to_ticks 互逆。
 * RMT 在接收模式下使用相同的时钟分辨率，将捕获的滴答数还原为时间值，
 * 方便与 NEC 协议的标准时序进行比较。
 *
 * @param ticks RMT 时钟滴答数
 * @return 对应的微秒值
 */
static uint32_t ticks_to_us(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000000ULL) / s_resolution_hz);
}

/* ======================== RMT 中断回调 ======================== */

/**
 * @brief RMT 接收完成中断回调函数
 *
 * 该函数在 ISR 上下文中执行（ISR），不能调用阻塞或耗时操作。
 * 工作流程：
 *   1. RMT 硬件接收完一帧信号后触发此回调
 *   2. 将从 edata 中获取的符号数量封装到 ky022_rx_event_t 中
 *   3. 通过 xQueueSendFromISR 发送到任务队列，唤醒阻塞的接收任务
 *   4. ISR 返回时决定是否需要进行上下文切换
 *
 * @param channel  触发回调的 RMT 通道句柄
 * @param edata    接收完成事件数据（包含符号数和缓冲区指针）
 * @param user_ctx 用户上下文指针（本例未使用）
 * @return pdTRUE  需要任务切换（有更高优先级任务被唤醒）
 * @return pdFALSE 无需任务切换
 */
static bool ky022_on_recv_done(rmt_channel_handle_t channel,
                               const rmt_rx_done_event_data_t *edata,
                               void *user_ctx)
{
    (void)channel;
    (void)user_ctx;

    BaseType_t high_task_wakeup = pdFALSE;

    ky022_rx_event_t event = {
        .num_symbols = edata->num_symbols,
    };

    /* 将接收事件发送到队列，在 ISR 中必须使用 FromISR 版本 */
    xQueueSendFromISR(s_rx_queue, &event, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}

/* ======================== NEC 解码引擎 ======================== */

/**
 * @brief 解码 NEC 协议 RMT symbol 序列
 *
 * NEC 协议解码步骤：
 *
 * 第一步 — 寻找引导码：
 *   遍历 symbol 数组，寻找 9ms 高 + 4.5ms 低的引导码模式。
 *   同时确定 mark 电平极性（因为 RMT 配置不同可能导致有效电平为高或低）。
 *
 * 第二步 — 逐位解码：
 *   NEC 数据位编码规则：
 *     "0" = 560us 高 + 560us 低   （高低比例 1:1）
 *     "1" = 560us 高 + 1690us 低  （高低比例 1:3）
 *   根据 mark 脉冲的宽度判断是否为有效数据位，再根据 space 宽度判断 0/1。
 *
 * 第三步 — 反码校验：
 *   地址码与地址反码按位取反应相等（address == ~address_inv），
 *   命令码同理。这能有效检测传输错误。
 *
 * @param symbols     从 RMT 接收到的 symbol 数组
 * @param num_symbols symbol 数组中的有效符号数量
 * @param frame       [out] 解码成功后填充的 NEC 帧数据
 * @return true  解码成功
 * @return false 解码失败（未找到引导码、数据位不足、校验失败等）
 */
static bool decode_nec_symbols(const rmt_symbol_word_t *symbols, size_t num_symbols,
                               ky022_nec_frame_t *frame)
{
    /* ---------- 前置校验 ----------
     * 最少需要：1 引导码 + 32 数据位 = 33 个符号 */
    if (num_symbols < NEC_BITS + 1 || !frame) {
        return false;
    }

    /* ---------- 第一步：寻找引导码 ----------
     * 引导码特征：9ms 高 + 4.5ms 低（或极性反转后的等价形式）
     * 由于每家发射器/接收器的极性可能不同，同时检测两种极性。 */
    size_t start = SIZE_MAX;
    int mark_level = -1;

    for (size_t i = 0; i < num_symbols; i++) {
        uint32_t d0 = ticks_to_us(symbols[i].duration0);
        uint32_t d1 = ticks_to_us(symbols[i].duration1);

        /* 情况 A: level0=高 → d0 应为 9ms，d1 应为 4.5ms */
        if (within(d0, 9000) && within(d1, 4500)) {
            start = i + 1;
            mark_level = symbols[i].level0;    /* mark 脉冲为高电平 */
            break;
        }
        /* 情况 B: level0=低 → d1 应为 9ms，d0 应为 4.5ms（极性反转） */
        if (within(d1, 9000) && within(d0, 4500)) {
            start = i + 1;
            mark_level = symbols[i].level1;    /* mark 脉冲为低电平 */
            break;
        }
    }

    /* 未找到引导码，或符号数不足以容纳 32 位数据 */
    if (start == SIZE_MAX || num_symbols < start + NEC_BITS) {
        return false;
    }

    /* ---------- 第二步：逐位解码 32 位数据 ----------
     * NEC 协议采用 LSB 优先传输，bit 0 最先收到。 */
    uint32_t data = 0;

    for (size_t bit = 0; bit < NEC_BITS; bit++) {
        const rmt_symbol_word_t *symbol = &symbols[start + bit];
        uint32_t mark_us  = 0;     /* Mark 脉冲宽度 (激活电平) */
        uint32_t space_us = 0;     /* Space 脉冲宽度 (非激活电平) */

        /* 根据引导码阶段确定的 mark_level，提取 mark 和 space 的微秒值 */
        if (symbol->level0 == mark_level) {
            mark_us  = ticks_to_us(symbol->duration0);
            space_us = ticks_to_us(symbol->duration1);
        } else if (symbol->level1 == mark_level) {
            mark_us  = ticks_to_us(symbol->duration1);
            space_us = ticks_to_us(symbol->duration0);
        } else {
            return false;   /* 无法识别电平极性，解码失败 */
        }

        /* 有效数据位的 mark 脉冲宽度应在 560us 附近 */
        if (!within(mark_us, 560)) {
            return false;
        }

        /* 根据 space 脉冲宽度判断 0 或 1：
         *   "0": space ≈ 560us   (1:1)
         *   "1": space ≈ 1690us  (1:3) */
        if (within(space_us, 560)) {
            data |= 0UL << bit;                     /* 写 0 可省略，但保留以表意 */
        } else if (within(space_us, 1690)) {
            data |= 1UL << bit;                     /* 写 1 */
        } else {
            return false;   /* 脉冲宽度不匹配 0 或 1 */
        }
    }

    /* ---------- 第三步：反码校验 ----------
     * NEC 协议规定地址和命令都携带反码用于错误检测：
     *   地址字节 + 地址反码字节 = 0xFF
     *   命令字节 + 命令反码字节 = 0xFF */
    uint8_t address     = data & 0xFF;
    uint8_t address_inv = (data >> 8) & 0xFF;
    uint8_t command     = (data >> 16) & 0xFF;
    uint8_t command_inv = (data >> 24) & 0xFF;

    if ((uint8_t)~address != address_inv || (uint8_t)~command != command_inv) {
        ESP_LOGW(TAG, "反码校验失败: addr=0x%02X addr_inv=0x%02X cmd=0x%02X cmd_inv=0x%02X",
                 address, address_inv, command, command_inv);
        return false;
    }

    /* ---------- 解码成功，填充结果 ---------- */
    frame->address  = address;
    frame->command  = command;
    frame->raw_data = data;
    return true;
}

/* ======================== 初始化 / 反初始化 ======================== */

/**
 * @brief 初始化 KY-022 红外接收器
 *
 * 初始化流程：
 *   1. 校验传入参数（config 非空、GPIO 有效）
 *   2. 如果已经初始化过，直接返回 ESP_OK（防重入）
 *   3. 设置分辨率与信号范围参数
 *   4. 创建队列用于 ISR 到任务的数据传递
 *   5. 创建 RMT RX 通道并绑定到指定 GPIO
 *   6. 注册接收完成回调函数
 *   7. 使能 RMT RX 通道
 *
 * @param config KY-022 配置参数指针
 * @return ESP_OK       成功
 * @return ESP_ERR_INVALID_ARG  参数无效
 * @return ESP_ERR_NO_MEM       队列创建失败
 * @return 其他         RMT 配置失败
 */
esp_err_t ky022_init(const ky022_config_t *config)
{
    /* ---------- 参数校验 ---------- */
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config 为空");
    ESP_RETURN_ON_FALSE(config->gpio_num >= 0, ESP_ERR_INVALID_ARG, TAG, "无效 GPIO");
    esp_err_t ret = ESP_OK;  /* 用于 ESP_GOTO_ON_ERROR 宏 */

    /* 已初始化则直接返回 */
    if (s_rx_channel) {
        return ESP_OK;
    }

    /* ---------- 参数默认值 ---------- */
    s_resolution_hz = config->resolution_hz ? config->resolution_hz : 1000000;   /* 默认 1 MHz */

    s_receive_config.signal_range_min_ns = config->signal_range_min_ns ?
                                           config->signal_range_min_ns : 1250;   /* 默认 1.25us */
    s_receive_config.signal_range_max_ns = config->signal_range_max_ns ?
                                           config->signal_range_max_ns : 12000000; /* 默认 12ms */

    /* ---------- 步骤1: 创建 ISR → 任务通信队列 ----------
     * 队列深度为 4，每个元素为 ky022_rx_event_t（仅包含符号数量）。
     * ISR 将接收完成事件入队，阻塞在 ky022_receive_nec 中的任务出队。 */
    s_rx_queue = xQueueCreate(4, sizeof(ky022_rx_event_t));
    ESP_RETURN_ON_FALSE(s_rx_queue, ESP_ERR_NO_MEM, TAG, "创建队列失败");

    /* ---------- 步骤2: 创建 RMT RX 通道 ---------- */
    rmt_rx_channel_config_t rx_config = {
        .gpio_num = config->gpio_num,           /* 接收 GPIO 引脚 */
        .clk_src = RMT_CLK_SRC_DEFAULT,         /* 默认时钟源 */
        .resolution_hz = s_resolution_hz,       /* 捕获精度 */
        .mem_block_symbols = 64,                /* 硬件缓冲区大小 */
        .flags.invert_in = false,               /* 不反转输入信号 */
        .flags.with_dma = false,                /* 不使用 DMA（64 符号以内无需 DMA） */
    };
    ESP_GOTO_ON_ERROR(rmt_new_rx_channel(&rx_config, &s_rx_channel),
                      err, TAG, "创建 RX 通道失败");

    /* ---------- 步骤3: 注册中断回调 ----------
     * RMT 接收完指定数量的 symbol 或检测到信号空闲时触发。 */
    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = ky022_on_recv_done,
    };
    ESP_GOTO_ON_ERROR(rmt_rx_register_event_callbacks(s_rx_channel, &callbacks, NULL),
                      err, TAG, "注册 RX 回调失败");

    /* ---------- 步骤4: 使能 RX 通道 ---------- */
    ESP_GOTO_ON_ERROR(rmt_enable(s_rx_channel),
                      err, TAG, "使能 RX 通道失败");

    ESP_LOGI(TAG, "KY-022 已就绪，GPIO%d", config->gpio_num);
    return ESP_OK;

    /* ---------- 错误回滚 ---------- */
err:
    ky022_deinit();
    return ESP_FAIL;
}

/* ======================== 接收函数 ======================== */

/**
 * @brief 接收 NEC 协议红外帧（阻塞调用）
 *
 * 工作流程：
 *   1. 清空队列和接收缓冲区，确保读到的是新数据
 *   2. 调用 rmt_receive 启动 RMT 硬件捕获
 *   3. 阻塞等待 ISR 通过队列通知接收完成
 *   4. 对收到的 symbol 序列进行 NEC 解码
 *
 * @param frame      [out] 解码后的 NEC 帧数据
 * @param timeout_ms 等待超时时间 (毫秒)
 * @return ESP_OK                 接收并解码成功
 * @return ESP_ERR_TIMEOUT        超时，未收到任何红外信号
 * @return ESP_ERR_INVALID_STATE  未初始化
 * @return ESP_ERR_INVALID_ARG    frame 参数为空
 * @return ESP_ERR_INVALID_RESPONSE  收到了信号但解码失败
 */
esp_err_t ky022_receive_nec(ky022_nec_frame_t *frame, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_rx_channel && s_rx_queue,
                        ESP_ERR_INVALID_STATE, TAG, "KY-022 未初始化");
    ESP_RETURN_ON_FALSE(frame,
                        ESP_ERR_INVALID_ARG, TAG, "frame 为空");

    /* 清空旧数据，防止上次残留影响本次读取 */
    xQueueReset(s_rx_queue);
    memset(s_rx_symbols, 0, sizeof(s_rx_symbols));

    /* 启动 RMT 接收（非阻塞：RMT 硬件在后台捕获，完成后触发 ISR） */
    ESP_RETURN_ON_ERROR(rmt_receive(s_rx_channel, s_rx_symbols, sizeof(s_rx_symbols), &s_receive_config),
                        TAG, "启动接收失败");

    /* 阻塞等待 ISR 发来的接收完成事件 */
    ky022_rx_event_t event = {};
    if (xQueueReceive(s_rx_queue, &event, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;     /* 超时未收到红外信号 */
    }

    /* 对收到的 symbol 序列进行 NEC 解码 */
    if (!decode_nec_symbols(s_rx_symbols, event.num_symbols, frame)) {
        ESP_LOGW(TAG, "收到 %u 个符号，但 NEC 解码失败", (unsigned)event.num_symbols);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

/**
 * @brief 反初始化 KY-022，释放所有资源
 *
 * 逆序释放：先禁能通道停止接收，再删除通道，最后删除队列。
 *
 * @return ESP_OK 始终返回成功
 */
esp_err_t ky022_deinit(void)
{
    if (s_rx_channel) {
        rmt_disable(s_rx_channel);          /* 停止 RMT 接收 */
        rmt_del_channel(s_rx_channel);      /* 删除 RX 通道 */
        s_rx_channel = NULL;
    }
    if (s_rx_queue) {
        vQueueDelete(s_rx_queue);           /* 删除同步队列 */
        s_rx_queue = NULL;
    }

    /* 重置全局状态 */
    s_resolution_hz = 0;
    memset(&s_receive_config, 0, sizeof(s_receive_config));

    ESP_LOGI(TAG, "KY-022 已反初始化");
    return ESP_OK;
}
