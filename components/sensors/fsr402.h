#ifndef FSR402_H
#define FSR402_H

#include <stdbool.h>

typedef enum {
    // FSR402 接电源端、固定电阻接 GND，中点电压会随按压力增大而升高。
    FSR402_DIVIDER_PULL_DOWN = 0,
    // 固定电阻接电源端、FSR402 接 GND，中点电压会随按压力增大而降低。
    FSR402_DIVIDER_PULL_UP,
} fsr402_divider_t;

typedef struct {
    float supply_voltage_v;       // 分压电路供电电压。
    float fixed_resistor_ohm;     // 分压电路中的固定电阻阻值。
    fsr402_divider_t divider;     // 分压接法类型。
    float ema_alpha;              // 指数滑动平均滤波系数，范围 0.0-1.0。
    float zero_offset_v;          // 空载偏置电压，用于软件扣零。
} fsr402_config_t;

typedef struct {
    float voltage_v;              // 扣零后的原始电压。
    float filtered_voltage_v;     // 滤波后的电压。
    float resistance_ohm;         // 根据分压公式换算出的 FSR402 电阻。
    float conductance_us;         // 电导，单位微西门子。
    float force_g;                // 近似压力，单位克力。
    float force_n;                // 近似压力，单位牛顿。
} fsr402_sample_t;

typedef struct {
    fsr402_config_t config;       // 传感器配置。
    bool filter_ready;            // 滤波器是否已经有初始值。
    float filtered_voltage_v;     // 当前滤波电压状态。
} fsr402_t;

// 初始化 FSR402 计算模块。
void fsr402_init(fsr402_t *sensor, const fsr402_config_t *config);
// 使用当前测得电压作为空载零点。
void fsr402_set_zero_from_voltage(fsr402_t *sensor, float measured_voltage_v);
// 输入 ADS1115 读到的电压，输出电阻、电导和近似压力。
fsr402_sample_t fsr402_update(fsr402_t *sensor, float measured_voltage_v);
// 根据分压电路公式，把电压换算成 FSR402 电阻。
float fsr402_voltage_to_resistance(const fsr402_config_t *config, float voltage_v);
// 根据经验曲线，把 FSR402 电阻换算成近似克力。
float fsr402_resistance_to_force_g(float resistance_ohm);

#endif  // FSR402_H
