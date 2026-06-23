/*
 * sensors.c — 传感器统一封装实现
 *
 * 全部传感器初始化和轮询逻辑，从 wanzheng-usart 迁移而来。
 * 唯一改动：I2C1 从 GPIO10/11 移到 GPIO14/15（避免与 INMP441 I2S 冲突）。
 */

#include "sensors.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "ads1115_fsr402.h"
#include "bh1750.h"
#include "fsr402.h"
#include "ky005.h"
#include "ky022.h"
#include "mq_135.h"
#include "sht31.h"
#include "usart.h"

static const char *TAG = "sensors";

/* ═══════════════════════════════════════════════════════════
 *  引脚定义（I2C1 已改为 GPIO14/15，避免与麦克风 I2S 冲突）
 * ═══════════════════════════════════════════════════════════ */
#define MCP_ADS_I2C_PORT        I2C_NUM_0
#define MCP_ADS_SDA_GPIO        GPIO_NUM_8
#define MCP_ADS_SCL_GPIO        GPIO_NUM_9
#define MCP_ADS_ADDR            ADS1115_DEFAULT_ADDR

#define FSR_ADS_I2C_PORT        I2C_NUM_1
#define FSR_I2C_SDA_GPIO        GPIO_NUM_14   /* ← 原 GPIO10 */
#define FSR_I2C_SCL_GPIO        GPIO_NUM_15   /* ← 原 GPIO11 */
#define FSR_ADS_ADDR            ADS1115_DEFAULT_ADDR

#define KY005_TX_GPIO           GPIO_NUM_12
#define KY022_RX_GPIO           GPIO_NUM_13

#define MQ135_ADC_UNIT          ADC_UNIT_1
#define MQ135_ADC_CHANNEL       ADC_CHANNEL_0

#define I2C_CLK_HZ              100000
#define FSR_SENSOR_COUNT        4

/* MCP5010DP 分压 & 量程 */
#define MCP_DIVIDER_TOP_OHM     4700.0f
#define MCP_DIVIDER_BOTTOM_OHM  10000.0f
#define MCP_SUPPLY_V            5.0f
#define MCP_PRESSURE_MIN_KPA    0.0f
#define MCP_PRESSURE_MAX_KPA    10.0f
#define MCP_OUTPUT_MIN_RATIO    0.04f
#define MCP_OUTPUT_MAX_RATIO    0.94f

/* ═══════════════════════════════════════════════════════════
 *  静态变量
 * ═══════════════════════════════════════════════════════════ */
static const ads1115_t s_mcp_ads = {
    .i2c_port    = MCP_ADS_I2C_PORT,
    .sda_io      = MCP_ADS_SDA_GPIO,
    .scl_io      = MCP_ADS_SCL_GPIO,
    .clk_speed_hz = I2C_CLK_HZ,
    .address     = MCP_ADS_ADDR,
    .gain        = ADS1115_GAIN_4_096V,
    .data_rate   = ADS1115_DR_128SPS,
};

static const ads1115_t s_fsr_ads = {
    .i2c_port    = FSR_ADS_I2C_PORT,
    .sda_io      = FSR_I2C_SDA_GPIO,
    .scl_io      = FSR_I2C_SCL_GPIO,
    .clk_speed_hz = I2C_CLK_HZ,
    .address     = FSR_ADS_ADDR,
    .gain        = ADS1115_GAIN_4_096V,
    .data_rate   = ADS1115_DR_128SPS,
};

static const ads1115_mux_t s_fsr_mux[FSR_SENSOR_COUNT] = {
    ADS1115_MUX_AIN0_GND,
    ADS1115_MUX_AIN1_GND,
    ADS1115_MUX_AIN2_GND,
    ADS1115_MUX_AIN3_GND,
};

static fsr402_t  s_fsr[FSR_SENSOR_COUNT];
static bh1750_t  s_bh1750;
static sht31_t   s_sht31;

/* 就绪标志 */
static bool s_mcp_ads_ready;
static bool s_fsr_ads_ready;
static bool s_bh1750_ready;
static bool s_sht31_ready;
static bool s_mq135_ready;
static bool s_ky005_ready;
static bool s_ky022_ready;
static bool s_usart_ready;

/* 最新数据缓存 + 互斥锁 */
static sensor_data_t s_latest;
static portMUX_TYPE   s_data_spinlock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t   s_sensor_task_handle = NULL;
static SemaphoreHandle_t s_i2c0_mutex = NULL;  /* MCP5010DP I2C0 互斥 */

/* ── 人员就寝检测（FSR 力敏传感器）─────────────── */
#define PERSON_FSR_THRESHOLD_N  1.0f
#define PERSON_DEBOUNCE_COUNT    2       // 连续2秒确认
static volatile bool s_person_on_bed  = false;
static volatile bool s_person_event   = false;
static          int  s_person_debounce = 0;

/* ═══════════════════════════════════════════════════════════
 *  辅助函数
 * ═══════════════════════════════════════════════════════════ */

static bool init_result(const char *name, esp_err_t err)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s init OK", name);
        return true;
    }
    ESP_LOGW(TAG, "%s init failed: %d", name, err);
    return false;
}

static esp_err_t read_ads_voltage(const ads1115_t *ads, ads1115_mux_t mux,
                                   int16_t *raw, float *voltage)
{
    if (!ads || !raw || !voltage) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(ads1115_read_raw(ads, mux, raw), TAG, "ads read raw");
    *voltage = ads1115_raw_to_voltage(ads, *raw);
    return ESP_OK;
}

static float mcp_adc_to_sensor_voltage(float adc_voltage)
{
    return adc_voltage * (MCP_DIVIDER_TOP_OHM + MCP_DIVIDER_BOTTOM_OHM)
           / MCP_DIVIDER_BOTTOM_OHM;
}

static float mcp_voltage_to_pressure_kpa(float sensor_voltage)
{
    float v_min = MCP_SUPPLY_V * MCP_OUTPUT_MIN_RATIO;
    float v_max = MCP_SUPPLY_V * MCP_OUTPUT_MAX_RATIO;
    if (sensor_voltage <= v_min) return MCP_PRESSURE_MIN_KPA;
    if (sensor_voltage >= v_max) return MCP_PRESSURE_MAX_KPA;
    return MCP_PRESSURE_MIN_KPA
           + (MCP_PRESSURE_MAX_KPA - MCP_PRESSURE_MIN_KPA)
             * (sensor_voltage - v_min) / (v_max - v_min);
}

static void init_fsr_models(void)
{
    const fsr402_config_t config = {
        .supply_voltage_v   = 3.3f,
        .fixed_resistor_ohm = 10000.0f,
        .divider            = FSR402_DIVIDER_PULL_DOWN,
        .ema_alpha          = 0.25f,
        .zero_offset_v      = 0.0f,
    };
    for (int i = 0; i < FSR_SENSOR_COUNT; i++) {
        fsr402_init(&s_fsr[i], &config);
    }
}

static void calibrate_fsr_zero(void)
{
    if (!s_fsr_ads_ready) return;
    ESP_LOGI(TAG, "FSR zero calibration — keep sensors unloaded for 300ms...");
    vTaskDelay(pdMS_TO_TICKS(300));
    for (int i = 0; i < FSR_SENSOR_COUNT; i++) {
        int16_t raw;
        float voltage;
        if (ads1115_read_raw(&s_fsr_ads, s_fsr_mux[i], &raw) != ESP_OK) continue;
        voltage = ads1115_raw_to_voltage(&s_fsr_ads, raw);
        fsr402_set_zero_from_voltage(&s_fsr[i], voltage);
        ESP_LOGI(TAG, "FSR%d zero=%.3fV", i + 1, voltage);
    }
}

/* ═══════════════════════════════════════════════════════════
 *  传感器读取
 * ═══════════════════════════════════════════════════════════ */

static void read_mcp5010dp(sensor_data_t *out)
{
    out->pressure_valid = false;
    if (!s_mcp_ads_ready || !s_i2c0_mutex) return;

    if (xSemaphoreTake(s_i2c0_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return;
    int16_t raw;
    float adc_v;
    esp_err_t err = read_ads_voltage(&s_mcp_ads, ADS1115_MUX_AIN0_GND, &raw, &adc_v);
    xSemaphoreGive(s_i2c0_mutex);

    if (err != ESP_OK) return;
    float sensor_v = mcp_adc_to_sensor_voltage(adc_v);
    float kpa = mcp_voltage_to_pressure_kpa(sensor_v);
    out->pressure_kpa = kpa;
    out->pressure_valid = true;

    if (s_usart_ready) usart_tjc_set_t7_pressure_kpa(kpa);
    ESP_LOGD(TAG, "MCP5010DP raw=%d adc=%.3fV sensor=%.3fV kpa=%.2f",
             raw, adc_v, sensor_v, kpa);
}

static void read_fsr402_all(sensor_data_t *out)
{
    for (int i = 0; i < FSR_SENSOR_COUNT; i++) {
        out->fsr_valid[i] = false;
        out->fsr_force_n[i] = 0.0f;
    }
    if (!s_fsr_ads_ready) return;

    for (int i = 0; i < FSR_SENSOR_COUNT; i++) {
        int16_t raw;
        float voltage;
        if (ads1115_read_raw(&s_fsr_ads, s_fsr_mux[i], &raw) != ESP_OK) continue;
        voltage = ads1115_raw_to_voltage(&s_fsr_ads, raw);

        fsr402_sample_t sample = fsr402_update(&s_fsr[i], voltage);
        /* fsr402_update always returns a valid struct; check force > 0 as valid */
        if (sample.force_n > 0.0f) {
            out->fsr_force_n[i] = sample.force_n;
            out->fsr_valid[i] = true;
            ESP_LOGD(TAG, "FSR%d N=%.3f", i + 1, sample.force_n);
        }
    }
}

static void read_environment(sensor_data_t *out)
{
    out->light_valid   = false;
    out->env_valid     = false;
    out->mq135_valid   = false;

    /* BH1750 光照 */
    if (s_bh1750_ready) {
        float lux;
        if (bh1750_read_lux(&s_bh1750, &lux) == ESP_OK) {
            out->light_lux = lux;
            out->light_valid = true;
            if (s_usart_ready) usart_tjc_set_t6_lux(lux);
            ESP_LOGD(TAG, "BH1750 lux=%.1f", lux);
        }
    }

    /* SHT31 温湿度 */
    if (s_sht31_ready) {
        float t, h;
        if (sht31_read_temp_humi(&s_sht31, &t, &h) == ESP_OK) {
            out->temperature_c = t;
            out->humidity_pct  = h;
            out->env_valid     = true;
            if (s_usart_ready) {
                usart_tjc_set_t4_temp_c(t);
                usart_tjc_set_t9_humidity(h);
            }
            ESP_LOGD(TAG, "SHT31 T=%.1fC H=%.1f%%", t, h);
        }
    }

    /* MQ-135 空气质量 */
    if (s_mq135_ready) {
        mq135_data_t mq;
        if (mq135_read(&mq) == ESP_OK && mq.ppm_valid) {
            out->mq135_ppm = mq.ppm;
            out->mq135_valid = true;
            if (s_usart_ready) usart_tjc_set_t5_mq135_ppm(mq.ppm);
            ESP_LOGD(TAG, "MQ135 raw=%d V=%.3f Rs=%.1fk R0=%.1fk ppm=%.1f",
                     mq.raw, mq.sensor_voltage, mq.rs_kohm,
                     (mq.ratio > 0.001f ? mq.rs_kohm / mq.ratio : 0.0f), mq.ppm);
        }
    }
}

/* ═══════════════════════════════════════════════════════════
 *  公开 API
 * ═══════════════════════════════════════════════════════════ */

void init_sensors(void)
{
    ESP_LOGI(TAG, "========== Sensor Init Start ==========");

    /* I2C0 互斥锁：防 pump_task / sensor_task / LLM read_sensors 抢 ADS1115 */
    s_i2c0_mutex = xSemaphoreCreateMutex();

    /* FSR 软件模型 */
    init_fsr_models();

    /* ADS1115 #1 — MCP5010DP 气压 (I2C0: GPIO8/9) */
    s_mcp_ads_ready = init_result("ADS1115-MCP", ads1115_init(&s_mcp_ads));

    /* ADS1115 #2 — FSR402×4 (I2C1: GPIO14/15) */
    s_fsr_ads_ready = init_result("ADS1115-FSR", ads1115_init(&s_fsr_ads));

    /* BH1750 光照 (I2C1 共用 GPIO14/15, addr 0x23) */
    esp_err_t bh1750_err = bh1750_init(&s_bh1750, FSR_ADS_I2C_PORT,
                                       FSR_I2C_SDA_GPIO, FSR_I2C_SCL_GPIO,
                                       I2C_CLK_HZ, BH1750_DEFAULT_ADDR);
    if (bh1750_err != ESP_OK) {
        ESP_LOGW(TAG, "BH1750 init retry 1/2 after 200ms (err=%d)", bh1750_err);
        vTaskDelay(pdMS_TO_TICKS(200));
        bh1750_err = bh1750_init(&s_bh1750, FSR_ADS_I2C_PORT,
                                 FSR_I2C_SDA_GPIO, FSR_I2C_SCL_GPIO,
                                 I2C_CLK_HZ, BH1750_DEFAULT_ADDR);
    }
    s_bh1750_ready = init_result("BH1750", bh1750_err);

    /* SHT31 温湿度 (I2C1 共用 GPIO14/15, addr 0x44) */
    s_sht31_ready = init_result("SHT31",
        sht31_init(&s_sht31, FSR_ADS_I2C_PORT, SHT31_I2C_ADDR_DEFAULT));

    /* MQ-135 空气质量 (ADC1_CH0 = GPIO1) */
    const mq135_config_t mq135_config = {
        .unit               = MQ135_ADC_UNIT,
        .channel            = MQ135_ADC_CHANNEL,
        .atten              = MQ135_DEFAULT_ATTEN,
        .bitwidth           = MQ135_DEFAULT_BITWIDTH,
        .sample_count       = MQ135_DEFAULT_SAMPLE_COUNT,
        .load_resistance_kohm = MQ135_DEFAULT_LOAD_RES_KOHM,
        .r0_kohm            = MQ135_DEFAULT_R0_KOHM,
        .supply_voltage     = 5.0f,
    };
    s_mq135_ready = init_result("MQ-135", mq135_init(&mq135_config));

    /* KY-005 红外发射 (RMT TX, GPIO12) */
    ky005_config_t ky005_cfg = KY005_DEFAULT_CONFIG(KY005_TX_GPIO);
    s_ky005_ready = init_result("KY-005", ky005_init(&ky005_cfg));

    /* KY-022 红外接收 (RMT RX, GPIO13) */
    ky022_config_t ky022_cfg = KY022_DEFAULT_CONFIG(KY022_RX_GPIO);
    s_ky022_ready = init_result("KY-022", ky022_init(&ky022_cfg));

    /* 淘晶驰串口屏 (UART1, GPIO17/18, 115200) */
    s_usart_ready = init_result("TJC-USART", usart_init());

    /* FSR 零点校准 */
    calibrate_fsr_zero();

    ESP_LOGI(TAG, "========== Sensor Init Done ==========");
}

void sensor_task(void *arg)
{
    (void)arg;
    sensor_data_t data;
    s_sensor_task_handle = xTaskGetCurrentTaskHandle();

    while (1) {
        memset(&data, 0, sizeof(data));

        read_mcp5010dp(&data);
        read_fsr402_all(&data);
        read_environment(&data);

        /* 更新缓存（临界区） */
        portENTER_CRITICAL(&s_data_spinlock);
        memcpy(&s_latest, &data, sizeof(s_latest));
        portEXIT_CRITICAL(&s_data_spinlock);

        /* ── 人员就寝检测（FSR 力敏传感器）─────── */
        bool person_now = false;
        for (int i = 0; i < FSR_SENSOR_COUNT; i++) {
            if (data.fsr_valid[i] && data.fsr_force_n[i] > PERSON_FSR_THRESHOLD_N) {
                person_now = true; break;
            }
        }
        if (person_now) {
            if (++s_person_debounce >= PERSON_DEBOUNCE_COUNT && !s_person_on_bed) {
                s_person_event = true;
                s_person_on_bed = true;
                ESP_LOGI(TAG, "person detected (FSR > %.0fN)", PERSON_FSR_THRESHOLD_N);
            }
        } else {
            s_person_debounce = 0;
            s_person_on_bed = false;
        }

        /* 休眠 1s，可被 sensor_request_refresh 唤醒 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
}

void sensor_request_refresh(void)
{
    if (!s_sensor_task_handle) return;
    xTaskNotifyGive(s_sensor_task_handle);
    vTaskDelay(pdMS_TO_TICKS(150));  /* 等待读取完成 */
}

float sensor_read_pressure_kpa(void)
{
    if (!s_mcp_ads_ready || !s_i2c0_mutex) return -1.0f;
    if (xSemaphoreTake(s_i2c0_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
        return -1.0f;
    int16_t raw;
    float adc_v;
    esp_err_t err = read_ads_voltage(&s_mcp_ads, ADS1115_MUX_AIN0_GND, &raw, &adc_v);
    xSemaphoreGive(s_i2c0_mutex);
    if (err != ESP_OK) return -1.0f;
    float kpa = mcp_voltage_to_pressure_kpa(mcp_adc_to_sensor_voltage(adc_v));
    if (s_usart_ready) usart_tjc_set_t7_pressure_kpa(kpa);
    return kpa;
}

void sensor_get_latest(sensor_data_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_data_spinlock);
    memcpy(out, &s_latest, sizeof(sensor_data_t));
    portEXIT_CRITICAL(&s_data_spinlock);
}

bool sensor_person_just_laid_down(void)
{
    bool val = s_person_event;
    s_person_event = false;
    return val;
}

void sensor_poll_ir(void)
{
    if (!s_ky022_ready || !s_ky005_ready) return;

    ky022_nec_frame_t frame;
    esp_err_t err = ky022_receive_nec(&frame, 20);
    if (err == ESP_ERR_TIMEOUT) return;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "IR receive error: %d", err);
        return;
    }

    ESP_LOGI(TAG, "IR NEC addr=0x%02X cmd=0x%02X", frame.address, frame.command);

    /* 保存到缓存 */
    portENTER_CRITICAL(&s_data_spinlock);
    s_latest.ir_address  = frame.address;
    s_latest.ir_command  = frame.command;
    s_latest.ir_has_data = true;
    portEXIT_CRITICAL(&s_data_spinlock);

    /* 红外转发 */
    ky005_send_nec(frame.address, frame.command);
}
