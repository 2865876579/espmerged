#include "mq_135.h"
#include <math.h>
#include <string.h>
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MQ135";

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali_handle;
static bool s_cali_enabled;

/* 保存当前配置，未传入配置时使用默认值。 */
static mq135_config_t s_config = {
    .unit = MQ135_DEFAULT_ADC_UNIT,
    .channel = MQ135_DEFAULT_ADC_CHANNEL,
    .atten = MQ135_DEFAULT_ATTEN,
    .bitwidth = MQ135_DEFAULT_BITWIDTH,
    .sample_count = MQ135_DEFAULT_SAMPLE_COUNT,
    .load_resistance_kohm = MQ135_DEFAULT_LOAD_RES_KOHM,
    .r0_kohm = MQ135_DEFAULT_R0_KOHM,
    .supply_voltage = MQ135_DEFAULT_SUPPLY_VOLTAGE,
};

static int mq135_bitwidth_max_raw(adc_bitwidth_t bitwidth)
{
    /* 未启用 ADC 校准时，用最大原始值估算电压。 */
    switch (bitwidth) {
    case ADC_BITWIDTH_9:
        return 511;
    case ADC_BITWIDTH_10:
        return 1023;
    case ADC_BITWIDTH_11:
        return 2047;
    case ADC_BITWIDTH_12:
    default:
        return 4095;
    }
}

static void mq135_try_calibration(void)
{
    /* ESP-IDF 的 ADC 校准能把 raw 转成更接近真实值的 mV。 */
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = s_config.unit,
        .chan = s_config.channel,
        .atten = s_config.atten,
        .bitwidth = s_config.bitwidth,
    };

    if (adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle) == ESP_OK) {
        s_cali_enabled = true;
    }
}

esp_err_t mq135_init(const mq135_config_t *config)
{
    if (config != NULL) {
        s_config = *config;
    }

    /* 对容易写错的参数做兜底，避免后面除以 0。 */
    if (s_config.sample_count == 0) {
        s_config.sample_count = MQ135_DEFAULT_SAMPLE_COUNT;
    }
    if (s_config.load_resistance_kohm <= 0.0f) {
        s_config.load_resistance_kohm = MQ135_DEFAULT_LOAD_RES_KOHM;
    }
    if (s_config.r0_kohm <= 0.0f) {
        s_config.r0_kohm = MQ135_DEFAULT_R0_KOHM;
    }
    if (s_config.supply_voltage <= 0.0f) {
        s_config.supply_voltage = MQ135_DEFAULT_SUPPLY_VOLTAGE;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = s_config.unit,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &s_adc_handle), TAG, "ADC init failed");

    /* 配置 MQ-135 AO 所接的 ADC 通道。 */
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = s_config.atten,
        .bitwidth = s_config.bitwidth,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, s_config.channel, &channel_config),
                        TAG, "ADC channel config failed");

    mq135_try_calibration();
    return ESP_OK;
}

esp_err_t mq135_read(mq135_data_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is NULL");
    ESP_RETURN_ON_FALSE(s_adc_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "MQ135 is not initialized");

    int raw_sum = 0;
    int raw = 0;
    /* 多次采样后取平均，减少 MQ 模块和 ADC 本身带来的抖动。 */
    for (uint16_t i = 0; i < s_config.sample_count; i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_handle, s_config.channel, &raw), TAG, "ADC read failed");
        raw_sum += raw;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    memset(data, 0, sizeof(*data));
    data->raw = raw_sum / s_config.sample_count;

    /* 优先使用 ADC 校准结果；如果校准不可用，就按 3.3V 满量程粗略估算。 */
    if (s_cali_enabled) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali_handle, data->raw, &data->voltage_mv),
                            TAG, "ADC calibration failed");
        data->sensor_voltage = (float)data->voltage_mv / 1000.0f;
        /* 分压比补偿：AO 接 4.7k+10k 分压，ADC 读到的是 AO×(10/(4.7+10))≈AO×0.68，除回去得到真实 AO 电压。 */
        data->sensor_voltage /= (10.0f / (4.7f + 10.0f));
    } else {
        data->voltage_mv = -1;
        data->sensor_voltage = ((float)data->raw / (float)mq135_bitwidth_max_raw(s_config.bitwidth)) * 3.3f;
        /* 同上，AO 端分压补偿。 */
        data->sensor_voltage /= (10.0f / (4.7f + 10.0f));
    }

    /*
     * MQ 模块常见分压关系：
     * Vout = Vc * RL / (Rs + RL)
     * 整理可得 Rs = (Vc - Vout) * RL / Vout
     */
    if (data->sensor_voltage > 0.001f && data->sensor_voltage < s_config.supply_voltage) {
        data->rs_kohm = ((s_config.supply_voltage - data->sensor_voltage) * s_config.load_resistance_kohm)
                        / data->sensor_voltage;
        data->ratio = data->rs_kohm / s_config.r0_kohm;

        /* 参考资料中的经验公式，ppm 结果依赖 R0 标定，未标定时只能作趋势参考。 */
        if (data->rs_kohm > 0.001f) {
            data->ppm = powf(11.5428f * s_config.r0_kohm / data->rs_kohm, 0.6549f);
            data->ppm_valid = isfinite(data->ppm);
        }
    }

    return ESP_OK;
}

void mq135_deinit(void)
{
    /* 释放校准句柄。 */
    if (s_cali_enabled) {
        adc_cali_delete_scheme_curve_fitting(s_cali_handle);
        s_cali_handle = NULL;
        s_cali_enabled = false;
    }

    /* 释放 ADC oneshot 句柄。 */
    if (s_adc_handle != NULL) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
}
