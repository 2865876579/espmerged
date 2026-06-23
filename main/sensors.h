/*
 * sensors.h — 传感器统一封装
 *
 * 把原 wanzheng-usart 的 8 个传感器驱动封装成一个模块：
 *   MQ-135 (空气质量) / MCP5010DP (气压) / FSR402×4 (压力)
 *   / BH1750 (光照) / SHT31 (温湿度) / KY-005+KY-022 (红外收发)
 *   / 淘晶驰串口屏
 *
 * 用法：
 *   1. init_sensors()        — 上电初始化全部传感器 + FSR 零点校准
 *   2. sensor_task()         — 作为 FreeRTOS 任务，1Hz 循环刷新
 *   3. sensor_get_latest()   — 获取最新传感器数据（线程安全）
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 传感器数据 ────────────────────────────────────────── */
typedef struct {
    /* MQ-135 空气质量 */
    float mq135_ppm;
    bool  mq135_valid;

    /* MCP5010DP 气压 (kPa) */
    float pressure_kpa;
    bool  pressure_valid;

    /* FSR402×4 压力 (牛顿) */
    float fsr_force_n[4];
    bool  fsr_valid[4];

    /* BH1750 光照 (lux) */
    float light_lux;
    bool  light_valid;

    /* SHT31 温湿度 */
    float temperature_c;
    float humidity_pct;
    bool  env_valid;

    /* 红外（最后一次收到的 NEC 帧） */
    uint8_t ir_address;
    uint8_t ir_command;
    bool    ir_has_data;
} sensor_data_t;

/* ── 初始化 ────────────────────────────────────────────── */
/**
 * 上电一次性初始化全部传感器和串口屏。
 * 失败的外设会被跳过，不会阻塞启动。
 * 调用时机：WiFi 连接前（不依赖网络）。
 */
void init_sensors(void);

/* ── 传感器轮询任务（FreeRTOS）────────────────────────── */
/**
 * 1 Hz 循环：读取全部传感器 → 刷串口屏 → 缓存最新数据。
 * 栈推荐 4096，优先级 1。
 */
void sensor_task(void *arg);

/* ── 线程安全读取 ─────────────────────────────────────── */
/**
 * 获取最新的传感器快照。
 * 可在任意任务/中断安全调用（mutex 保护）。
 */
void sensor_get_latest(sensor_data_t *out);

/**
 * 直接读取 MCP5010DP 气压值（不经过缓存，单次 I2C）。
 * 返回 kPa，失败返回 -1.0。用于泵闭环控制的即时反馈。
 */
float sensor_read_pressure_kpa(void);

/**
 * 请求传感器任务立即刷新一次（阻塞 ~150ms）。
 * 调用后 sensor_get_latest() 将返回最新数据。
 */
void sensor_request_refresh(void);

/**
 * 边沿触发：FSR 检测到有人躺下（>2N，去抖2秒）。
 * 返回 true 一次后自动清除。仅在非对话状态调用。
 */
bool sensor_person_just_laid_down(void);

/**
 * 轮询红外接收（20ms 非阻塞），收到帧时通过 KY-005 转发。
 * 应放在主循环中高频调用（每 50ms）。
 */
void sensor_poll_ir(void);

#ifdef __cplusplus
}
#endif
