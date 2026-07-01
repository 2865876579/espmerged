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
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
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

#define KY005_TX_GPIO           GPIO_NUM_13
#define KY022_RX_GPIO           GPIO_NUM_12
#define ENABLE_TJC_USART        1

#define RADAR_UART_NUM          UART_NUM_2
#define RADAR_RX_GPIO           GPIO_NUM_47  /* ESP32 RX <- R60ABD1 TX */
#define RADAR_TX_GPIO           GPIO_NUM_48  /* ESP32 TX -> R60ABD1 RX */
#define RADAR_UART_BAUD         115200
#define RADAR_UART_BUF_SIZE     512
#define RADAR_STALE_MS          6000
#define RADAR_QUERY_INTERVAL_MS 3000
#define RADAR_DEBUG_FRAME_LIMIT 12


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

/* NTC 10K 3950 on ADS1115-MCP A1: 3V3 -> NTC -> A1 -> 10k -> GND */
#define NECK_NTC_SUPPLY_V       3.3f
#define NECK_NTC_FIXED_OHM      10000.0f
#define NECK_NTC_R0_OHM         10000.0f
#define NECK_NTC_BETA           3950.0f
#define NECK_NTC_T0_K           298.15f

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
static bool s_usart_ready;
static bool s_ir_fan_on;
static bool s_ir_humidifier_on;
static bool s_ir_fan_known;
static bool s_ir_humidifier_known;
static bool s_radar_ready;
static volatile bool s_radar_person_gate;
static uint8_t s_radar_heart_bpm;
static uint8_t s_radar_breath_bpm;
static TickType_t s_radar_last_update_tick;
static uint8_t s_radar_debug_frames;
static portMUX_TYPE s_radar_spinlock = portMUX_INITIALIZER_UNLOCKED;

/**
 * 风扇电机开关（Fan）— NEC 协议（34 对 / 67 脉冲）
 *   地址: 0x00, 命令: 0x43
 */
static const uint32_t s_signal_fan[] = {
     9073,  4486,       607,   528,       608,   525,       609,   524,
      610,   523,       609,   525,       609,   523,       609,   525,
      608,   525,       609,  1633,       608,  1634,       608,  1632,
      608,  1633,       609,  1633,       611,  1630,       610,  1633,
      610,  1631,       608,  1634,       608,  1634,       609,   525,
      608,   527,       607,   527,       604,   530,       603,  1634,
      606,   529,       604,   529,       604,   528,       604,  1636,
      604,  1636,       604,  1640,       601,  1638,       604,   531,
      602,  1639,       603,     0
};
#define FAN_PAIRS  (sizeof(s_signal_fan) / sizeof(s_signal_fan[0]) / 2)

/**
 * 从遥控器捕获的加湿器控制信号（完整 NEC 帧，34 对脉冲）
 * Address=0x00, Command=0x00 — toggle 型
 */
static const uint32_t HUMIDIFIER_SIGNAL[] = {
     9076,  4491,       549,   584,       573,   559,       576,   557,
      574,   559,       574,   557,       573,   560,       572,   560,
      577,   556,       572,  1660,       549,  1686,       548,  1684,
      572,  1662,       573,  1658,       548,  1686,       572,  1661,
      569,  1662,       572,   560,       570,   562,       571,   560,
      574,   560,       572,   558,       575,   556,       572,   559,
      570,   560,       547,  1686,       571,  1662,       569,  1661,
      572,  1660,       569,  1663,       570,  1662,       547,  1685,
      570,  1663,       569,     0
};
#define HUMIDIFIER_SIGNAL_PAIRS  (sizeof(HUMIDIFIER_SIGNAL) / sizeof(HUMIDIFIER_SIGNAL[0]) / 2)
#define TX_BURST_COUNT           1        /* 每次发射帧数 */
#define TX_BURST_GAP_MS          100      /* 帧间间隔 (ms) */

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

static bool ntc_voltage_to_temp_c(float voltage, float *temp_c)
{
    if (!temp_c || voltage <= 0.02f || voltage >= (NECK_NTC_SUPPLY_V - 0.02f)) {
        return false;
    }
    float r_ntc = NECK_NTC_FIXED_OHM * (NECK_NTC_SUPPLY_V / voltage - 1.0f);
    if (!isfinite(r_ntc) || r_ntc <= 0.0f) {
        return false;
    }
    float inv_t = (1.0f / NECK_NTC_T0_K) + (logf(r_ntc / NECK_NTC_R0_OHM) / NECK_NTC_BETA);
    if (!isfinite(inv_t) || inv_t <= 0.0f) {
        return false;
    }
    *temp_c = (1.0f / inv_t) - 273.15f;
    return isfinite(*temp_c) && *temp_c > -40.0f && *temp_c < 125.0f;
}

static uint8_t radar_checksum(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

static void radar_send_cmd(uint8_t control, uint8_t command, const uint8_t *payload, uint16_t payload_len)
{
    if (!s_radar_ready) return;
    if (payload_len > 32) return;

    uint8_t frame[2 + 1 + 1 + 2 + 32 + 1 + 2];
    size_t idx = 0;
    frame[idx++] = 0x53;
    frame[idx++] = 0x59;
    frame[idx++] = control;
    frame[idx++] = command;
    frame[idx++] = (uint8_t)(payload_len >> 8);
    frame[idx++] = (uint8_t)(payload_len & 0xFF);
    for (uint16_t i = 0; i < payload_len; i++) {
        frame[idx++] = payload[i];
    }
    frame[idx] = radar_checksum(frame, idx);
    idx++;
    frame[idx++] = 0x54;
    frame[idx++] = 0x43;
    uart_write_bytes(RADAR_UART_NUM, (const char *)frame, idx);
}

static void radar_enable_measurement(void)
{
    uint8_t enable = 0x01;
    radar_send_cmd(0x81, 0x00, &enable, 1);  /* breath monitor on */
    radar_send_cmd(0x85, 0x00, &enable, 1);  /* heart monitor on */
}

static void radar_query_values(void)
{
    uint8_t query = 0x0F;
    radar_send_cmd(0x81, 0x82, &query, 1);  /* breath value query */
    radar_send_cmd(0x85, 0x82, &query, 1);  /* heart value query */
}

static void radar_set_values(uint8_t heart_bpm, uint8_t breath_bpm, bool update_heart, bool update_breath)
{
    if (!s_radar_person_gate) {
        return;
    }

    portENTER_CRITICAL(&s_radar_spinlock);
    if (update_heart) {
        s_radar_heart_bpm = heart_bpm;
    }
    if (update_breath) {
        s_radar_breath_bpm = breath_bpm;
    }
    s_radar_last_update_tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&s_radar_spinlock);
}

static void radar_reset_values(void)
{
    portENTER_CRITICAL(&s_radar_spinlock);
    s_radar_heart_bpm = 0;
    s_radar_breath_bpm = 0;
    s_radar_last_update_tick = 0;
    portEXIT_CRITICAL(&s_radar_spinlock);
}

static void radar_set_person_gate(bool enabled)
{
    bool was_enabled = s_radar_person_gate;
    s_radar_person_gate = enabled;
    if (!enabled) {
        radar_reset_values();
    } else if (!was_enabled) {
        radar_enable_measurement();
        radar_query_values();
    }
}

static void radar_get_values(uint8_t *heart_bpm, uint8_t *breath_bpm, bool *valid)
{
    uint8_t heart;
    uint8_t breath;
    TickType_t last_tick;

    portENTER_CRITICAL(&s_radar_spinlock);
    heart = s_radar_heart_bpm;
    breath = s_radar_breath_bpm;
    last_tick = s_radar_last_update_tick;
    portEXIT_CRITICAL(&s_radar_spinlock);

    bool fresh = false;
    if (s_radar_person_gate && last_tick != 0) {
        fresh = (xTaskGetTickCount() - last_tick) <= pdMS_TO_TICKS(RADAR_STALE_MS);
    }

    if (!fresh) {
        heart = 0;
        breath = 0;
    }
    if (heart_bpm) *heart_bpm = heart;
    if (breath_bpm) *breath_bpm = breath;
    if (valid) *valid = fresh && (heart > 0 || breath > 0);
}

static void radar_handle_frame(const uint8_t *frame, size_t frame_len)
{
    if (!frame || frame_len < 9) return;
    if (frame[0] != 0x53 || frame[1] != 0x59) return;
    if (frame[frame_len - 2] != 0x54 || frame[frame_len - 1] != 0x43) return;

    uint16_t payload_len = ((uint16_t)frame[4] << 8) | frame[5];
    if ((size_t)(payload_len + 9) != frame_len) return;
    uint8_t sum = radar_checksum(frame, (size_t)payload_len + 6);
    if (sum != frame[6 + payload_len]) {
        ESP_LOGW(TAG, "R60ABD1 checksum mismatch");
        return;
    }

    uint8_t control = frame[2];
    uint8_t command = frame[3];
    const uint8_t *payload = &frame[6];
    if (payload_len < 1) return;

    if (s_radar_debug_frames > 0 && (control == 0x81 || control == 0x85)) {
        ESP_LOGI(TAG, "R60ABD1 frame ctrl=0x%02X cmd=0x%02X len=%u data0=%u",
                 control, command, payload_len, payload[0]);
        s_radar_debug_frames--;
    }

    if (control == 0x85 && (command == 0x02 || command == 0x82)) {
        radar_set_values(payload[0], 0, true, false);
    } else if (control == 0x81 && (command == 0x02 || command == 0x82)) {
        radar_set_values(0, payload[0], false, true);
    }
}

static void radar_uart_task(void *arg)
{
    (void)arg;
    uint8_t buf[128];
    size_t used = 0;
    TickType_t last_enable = 0;
    TickType_t last_query = 0;

    while (1) {
        if (s_radar_person_gate &&
            (xTaskGetTickCount() - last_enable) >= pdMS_TO_TICKS(5000)) {
            radar_enable_measurement();
            last_enable = xTaskGetTickCount();
        }
        if (s_radar_person_gate &&
            (xTaskGetTickCount() - last_query) >= pdMS_TO_TICKS(RADAR_QUERY_INTERVAL_MS)) {
            radar_query_values();
            last_query = xTaskGetTickCount();
        }
        if (used >= sizeof(buf)) {
            used = 0;
        }

        int len = uart_read_bytes(RADAR_UART_NUM, buf + used, sizeof(buf) - used,
                                  pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }
        used += (size_t)len;

        while (used >= 9) {
            size_t start = 0;
            while (start + 1 < used && !(buf[start] == 0x53 && buf[start + 1] == 0x59)) {
                start++;
            }
            if (start > 0) {
                memmove(buf, buf + start, used - start);
                used -= start;
            }
            if (used < 9) break;

            uint16_t payload_len = ((uint16_t)buf[4] << 8) | buf[5];
            size_t frame_len = (size_t)payload_len + 9;
            if (frame_len > sizeof(buf)) {
                memmove(buf, buf + 2, used - 2);
                used -= 2;
                continue;
            }
            if (used < frame_len) break;

            radar_handle_frame(buf, frame_len);
            memmove(buf, buf + frame_len, used - frame_len);
            used -= frame_len;
        }
    }
}

static esp_err_t init_r60abd1_radar(void)
{
    const uart_config_t uart_config = {
        .baud_rate = RADAR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(RADAR_UART_NUM, &uart_config), TAG,
                        "radar uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(RADAR_UART_NUM, RADAR_TX_GPIO, RADAR_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "radar uart pin");
    ESP_RETURN_ON_ERROR(uart_driver_install(RADAR_UART_NUM, RADAR_UART_BUF_SIZE,
                                            0, 0, NULL, ESP_INTR_FLAG_SHARED),
                        TAG, "radar uart driver");
    s_radar_ready = true;
    s_radar_debug_frames = RADAR_DEBUG_FRAME_LIMIT;
    radar_enable_measurement();
    BaseType_t ret = xTaskCreate(radar_uart_task, "r60abd1", 4096, NULL, 1, NULL);
    if (ret != pdPASS) {
        s_radar_ready = false;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "R60ABD1 UART2 init OK: RX=GPIO%d TX=GPIO%d",
             RADAR_RX_GPIO, RADAR_TX_GPIO);
    return ESP_OK;
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

static void read_neck_ntc(sensor_data_t *out)
{
    out->neck_temp_valid = false;
    if (!s_mcp_ads_ready || !s_i2c0_mutex) return;

    if (xSemaphoreTake(s_i2c0_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return;
    int16_t raw;
    float adc_v;
    esp_err_t err = read_ads_voltage(&s_mcp_ads, ADS1115_MUX_AIN1_GND, &raw, &adc_v);
    xSemaphoreGive(s_i2c0_mutex);

    if (err != ESP_OK) return;
    float temp_c;
    if (!ntc_voltage_to_temp_c(adc_v, &temp_c)) {
        ESP_LOGW(TAG, "neck NTC invalid raw=%d voltage=%.3fV", raw, adc_v);
        return;
    }
    out->neck_temp_c = temp_c;
    out->neck_temp_valid = true;
    ESP_LOGD(TAG, "neck NTC raw=%d voltage=%.3fV temp=%.2fC", raw, adc_v, temp_c);
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

    s_radar_ready = init_result("R60ABD1", init_r60abd1_radar());

    /* KY-005 红外发射 (RMT TX, GPIO13) */
    ky005_config_t ky005_cfg = KY005_DEFAULT_CONFIG(KY005_TX_GPIO);
    ky005_cfg.carrier_hz = 40000;
    ky005_cfg.carrier_duty_percent = 50.0f;
    s_ky005_ready = init_result("KY-005", ky005_init(&ky005_cfg));
    ESP_LOGI(TAG, "KY-022 RX reserved on GPIO%d", KY022_RX_GPIO);

    /* 淘晶驰串口屏 (UART1, GPIO17/18, 115200) */
#if ENABLE_TJC_USART
    s_usart_ready = init_result("TJC-USART", usart_init());
#else
    s_usart_ready = false;
    ESP_LOGI(TAG, "TJC-USART skipped");
#endif

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
        read_neck_ntc(&data);
        read_fsr402_all(&data);
        read_environment(&data);

        bool person_now = false;
        for (int i = 0; i < FSR_SENSOR_COUNT; i++) {
            if (data.fsr_valid[i] && data.fsr_force_n[i] > PERSON_FSR_THRESHOLD_N) {
                person_now = true;
                break;
            }
        }
        radar_set_person_gate(person_now);
        radar_get_values(&data.radar_heart_bpm, &data.radar_breath_bpm, &data.radar_valid);

        /* 更新缓存（临界区） */
        portENTER_CRITICAL(&s_data_spinlock);
        memcpy(&s_latest, &data, sizeof(s_latest));
        portEXIT_CRITICAL(&s_data_spinlock);

        if (data.neck_temp_valid) {
            ESP_LOGI(
                TAG,
                "[pressure] kPa=%.2f neck_temp=%.1fC fsr=[%.2f, %.2f, %.2f, %.2f]N radar_hr=%u radar_br=%u",
                data.pressure_kpa,
                data.neck_temp_c,
                data.fsr_force_n[0],
                data.fsr_force_n[1],
                data.fsr_force_n[2],
                data.fsr_force_n[3],
                data.radar_heart_bpm,
                data.radar_breath_bpm
            );
        } else {
            ESP_LOGI(
                TAG,
                "[pressure] kPa=%.2f neck_temp=NA fsr=[%.2f, %.2f, %.2f, %.2f]N radar_hr=%u radar_br=%u",
                data.pressure_kpa,
                data.fsr_force_n[0],
                data.fsr_force_n[1],
                data.fsr_force_n[2],
                data.fsr_force_n[3],
                data.radar_heart_bpm,
                data.radar_breath_bpm
            );
        }

        /* ── 人员就寝检测（FSR 力敏传感器）─────── */
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

static esp_err_t send_ir_frame(const char *name, const uint32_t *signal, size_t pairs)
{
    if (!s_ky005_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = ky005_send_raw(signal, pairs);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[%s] 发送 OK", name);
    } else {
        ESP_LOGE(TAG, "[%s] 发送失败 %s", name, esp_err_to_name(err));
    }
    return err;
}

static bool parse_ir_action(const char *action, bool current, bool *desired)
{
    if (!action || !desired) {
        return false;
    }
    if (strcmp(action, "toggle") == 0) {
        *desired = !current;
        return true;
    }
    if (strcmp(action, "on") == 0 || strcmp(action, "open") == 0) {
        *desired = true;
        return true;
    }
    if (strcmp(action, "off") == 0 || strcmp(action, "close") == 0) {
        *desired = false;
        return true;
    }
    return false;
}

static esp_err_t send_humidifier_signal(void)
{
    if (!s_ky005_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    int ok = 0;
    esp_err_t last_err = ESP_FAIL;
    for (int i = 0; i < TX_BURST_COUNT; i++) {
        esp_err_t err = ky005_send_raw(HUMIDIFIER_SIGNAL, HUMIDIFIER_SIGNAL_PAIRS);
        if (err == ESP_OK) {
            ok++;
        } else {
            last_err = err;
            ESP_LOGE(TAG, "TX fail[%d]: %s", i, esp_err_to_name(err));
        }
        if (i + 1 < TX_BURST_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(TX_BURST_GAP_MS));
        }
    }
    ESP_LOGI(TAG, "[加湿器] TX: sent %d/%d frame(s)", ok, TX_BURST_COUNT);
    return ok > 0 ? ESP_OK : last_err;
}

static esp_err_t control_toggle_device(const char *name,
                                       const uint32_t *signal,
                                       size_t pairs,
                                       bool *state, bool *known,
                                       const char *action)
{
    if (!action || !state || !known) {
        return ESP_ERR_INVALID_ARG;
    }

    bool desired;
    if (!parse_ir_action(action, *state, &desired)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = send_ir_frame(name, signal, pairs);
    if (err == ESP_OK) {
        *state = desired;
        *known = true;
        ESP_LOGI(TAG, "IR %s request -> %s", name, desired ? "on" : "off");
    }
    return err;
}

static esp_err_t control_humidifier_device(const char *action)
{
    bool desired;
    if (!parse_ir_action(action, s_ir_humidifier_on, &desired)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = send_humidifier_signal();
    if (err == ESP_OK) {
        s_ir_humidifier_on = desired;
        s_ir_humidifier_known = true;
        ESP_LOGI(TAG, "IR 加湿器 request -> %s", desired ? "on" : "off");
    }
    return err;
}

esp_err_t sensor_ir_control_device(const char *device, const char *action)
{
    if (!device || !action) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(device, "fan") == 0) {
        return control_toggle_device("风扇", s_signal_fan, FAN_PAIRS,
                                     &s_ir_fan_on, &s_ir_fan_known, action);
    }
    if (strcmp(device, "humidifier") == 0 || strcmp(device, "humid") == 0) {
        return control_humidifier_device(action);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

void sensor_ir_get_state(bool *fan_on, bool *humidifier_on)
{
    if (fan_on) {
        *fan_on = s_ir_fan_on;
    }
    if (humidifier_on) {
        *humidifier_on = s_ir_humidifier_on;
    }
}

void sensor_poll_ir(void)
{
    /* TX 调通前不轮询 RX，避免接收转发干扰判断发射波形。GPIO12 仍保留给红外接收。 */
}
