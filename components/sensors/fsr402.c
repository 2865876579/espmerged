#include "fsr402.h"
#include <math.h>
#include <stddef.h>

#define FSR402_NEWTONS_PER_KGF 9.80665f

static float clampf(float value, float min_value, float max_value)
{
    // 把数值限制在指定范围内，避免电压或滤波系数越界。
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void fsr402_init(fsr402_t *sensor, const fsr402_config_t *config)
{
    if (sensor == NULL || config == NULL) {
        return;
    }

    sensor->config = *config;
    // ema_alpha 必须在 0.0 到 1.0 之间。
    sensor->config.ema_alpha = clampf(sensor->config.ema_alpha, 0.0f, 1.0f);
    sensor->filter_ready = false;
    sensor->filtered_voltage_v = 0.0f;
}

void fsr402_set_zero_from_voltage(fsr402_t *sensor, float measured_voltage_v)
{
    if (sensor == NULL) {
        return;
    }

    // 记录空载电压，后续每次读数都会先减去这个偏置。
    sensor->config.zero_offset_v = measured_voltage_v;
    // 零点变化后重新初始化滤波器，避免旧状态影响新零点。
    sensor->filter_ready = false;
}

fsr402_sample_t fsr402_update(fsr402_t *sensor, float measured_voltage_v)
{
    fsr402_sample_t sample = {0};

    if (sensor == NULL) {
        return sample;
    }

    // 扣除空载偏置，并限制在 0 到供电电压之间。
    float compensated_voltage = measured_voltage_v - sensor->config.zero_offset_v;
    compensated_voltage = clampf(compensated_voltage, 0.0f, sensor->config.supply_voltage_v);

    if (!sensor->filter_ready) {
        // 第一次读数直接作为滤波初值，避免从 0V 慢慢爬升。
        sensor->filtered_voltage_v = compensated_voltage;
        sensor->filter_ready = true;
    } else {
        // 指数滑动平均滤波：新值占 alpha，旧值占 1-alpha。
        const float alpha = sensor->config.ema_alpha;
        sensor->filtered_voltage_v = alpha * compensated_voltage +
                                     (1.0f - alpha) * sensor->filtered_voltage_v;
    }

    // 根据滤波后的电压计算电阻，再由电阻估算压力。
    sample.voltage_v = compensated_voltage;
    sample.filtered_voltage_v = sensor->filtered_voltage_v;
    sample.resistance_ohm = fsr402_voltage_to_resistance(&sensor->config,
                                                         sample.filtered_voltage_v);
    sample.force_n = fsr402_resistance_to_force_n(sample.resistance_ohm);
    sample.force_kgf = sample.force_n / FSR402_NEWTONS_PER_KGF;

    if (sample.resistance_ohm > 0.0f && isfinite(sample.resistance_ohm)) {
        // 电导单位使用微西门子：uS = 1,000,000 / 欧姆。
        sample.conductance_us = 1000000.0f / sample.resistance_ohm;
    }

    return sample;
}

float fsr402_voltage_to_resistance(const fsr402_config_t *config, float voltage_v)
{
    if (config == NULL || config->fixed_resistor_ohm <= 0.0f ||
        config->supply_voltage_v <= 0.0f) {
        return INFINITY;
    }

    const float v = clampf(voltage_v, 0.0f, config->supply_voltage_v);
    // 避免电压接近 0 或接近供电电压时出现除以 0。
    const float epsilon = 0.0001f;

    if (config->divider == FSR402_DIVIDER_PULL_DOWN) {
        if (v <= epsilon) {
            return INFINITY;
        }
        // 接法：VCC -> FSR -> A0 -> 固定电阻 -> GND。
        // Vout = VCC * Rfixed / (Rfsr + Rfixed)，整理得 Rfsr = Rfixed * (VCC - Vout) / Vout。
        return config->fixed_resistor_ohm * (config->supply_voltage_v - v) / v;
    }

    if ((config->supply_voltage_v - v) <= epsilon) {
        return INFINITY;
    }

    // 接法：VCC -> 固定电阻 -> A0 -> FSR -> GND。
    // Vout = VCC * Rfsr / (Rfixed + Rfsr)，整理得 Rfsr = Rfixed * Vout / (VCC - Vout)。
    return config->fixed_resistor_ohm * v / (config->supply_voltage_v - v);
}

float fsr402_resistance_to_force_n(float resistance_ohm)
{
    if (resistance_ohm <= 0.0f || !isfinite(resistance_ohm)) {
        return 0.0f;
    }

    const float conductance_us = 1000000.0f / resistance_ohm;

    // 这里使用常见 FSR 示例曲线的近似公式。
    // 如果需要准确压力值，应使用已知砝码做标定并替换这段映射。
    if (conductance_us <= 1000.0f) {
        return conductance_us / 80.0f;
    }

    return (conductance_us - 1000.0f) / 30.0f;
}
