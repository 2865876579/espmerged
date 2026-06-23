#ifndef MQ_135_H
#define MQ_135_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "hal/adc_types.h"

/* 默认使用 GPIO1，对应 ESP32-S3 的 ADC1_CHANNEL_0。 */
#define MQ135_DEFAULT_ADC_UNIT        ADC_UNIT_1
#define MQ135_DEFAULT_ADC_CHANNEL     ADC_CHANNEL_0

/* 11dB 衰减适合读取接近 3.3V 的模拟输入。 */
#define MQ135_DEFAULT_ATTEN           ADC_ATTEN_DB_11
#define MQ135_DEFAULT_BITWIDTH        ADC_BITWIDTH_12

/* 每次读取时采样 10 次再平均，可以减小 ADC 抖动。 */
#define MQ135_DEFAULT_SAMPLE_COUNT    10

/* 常见 MQ 模块板载负载电阻为 10k，若你的模块不同需要改这里。 */
#define MQ135_DEFAULT_LOAD_RES_KOHM   10.0f

/* R0 是洁净空气或标定环境下测出来的传感器基准电阻，影响 ppm 准确度。 */
#define MQ135_DEFAULT_R0_KOHM         6.64f

/* MQ-135 模块 VCC 实际接多少伏，这里就填多少伏；本工程默认接 3.3V。 */
#define MQ135_DEFAULT_SUPPLY_VOLTAGE  3.3f

typedef struct {
    adc_unit_t unit;              /* ADC 单元，ESP32-S3 推荐使用 ADC_UNIT_1。 */
    adc_channel_t channel;        /* ADC 通道，默认 ADC_CHANNEL_0 即 GPIO1。 */
    adc_atten_t atten;            /* ADC 衰减，默认 11dB。 */
    adc_bitwidth_t bitwidth;      /* ADC 分辨率，默认 12bit。 */
    uint16_t sample_count;        /* 平均采样次数。 */
    float load_resistance_kohm;   /* 负载电阻 RL，单位 kOhm。 */
    float r0_kohm;                /* 传感器基准电阻 R0，单位 kOhm。 */
    float supply_voltage;         /* MQ-135 模块 VCC 电压，单位 V，用于计算 Rs。 */
} mq135_config_t;

typedef struct {
    int raw;              /* ADC 原始平均值。 */
    int voltage_mv;       /* ADC 校准后的电压，单位 mV；未校准时为 -1。 */
    float sensor_voltage; /* AO 引脚电压，单位 V。 */
    float rs_kohm;        /* 由分压公式计算出的传感器电阻 Rs，单位 kOhm。 */
    float ratio;          /* Rs/R0，用于判断气体浓度变化。 */
    float ppm;            /* 根据经验公式估算出的 ppm。 */
    bool ppm_valid;       /* ppm 是否有效，输入电压异常时为 false。 */
} mq135_data_t;

/* 初始化 MQ-135 ADC 通道，config 为 NULL 时使用上面的默认参数。 */
esp_err_t mq135_init(const mq135_config_t *config);

/* 读取一次 MQ-135 数据，会完成多次 ADC 采样、平均和浓度估算。 */
esp_err_t mq135_read(mq135_data_t *data);

/* 释放 ADC 和校准资源。简单工程中通常不会调用，保留给需要重新初始化的场景。 */
void mq135_deinit(void);

#endif
