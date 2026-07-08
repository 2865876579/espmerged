#include "usart.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define USART_TJC_UART_NUM UART_NUM_1
#define USART_TJC_TX_GPIO GPIO_NUM_17
#define USART_TJC_RX_GPIO GPIO_NUM_18
#define USART_TJC_BAUD_RATE 115200
#define USART_TJC_BUF_SIZE 256
#define USART_TJC_CMD_BUF_SIZE 160
#define USART_TJC_RX_FRAME_MAX 128
#define USART_TJC_RX_TASK_STACK 3072
#define USART_TJC_RX_TASK_PRIO 6

#define TJC_COLOR_GREEN  42706u
#define TJC_COLOR_ORANGE 62857u
#define TJC_COLOR_RED    64329u
#define TJC_COLOR_CYAN   34587u
#define TJC_COLOR_GRAY   33808u
#define TJC_COLOR_PURPLE 48286u

#define TJC_PAGE_NAME "page0"
#define TJC_OBJ_AIR_QUALITY  TJC_PAGE_NAME ".t0"
#define TJC_OBJ_LIGHT        TJC_PAGE_NAME ".t1"
#define TJC_OBJ_HUMIDITY     TJC_PAGE_NAME ".t2"
#define TJC_OBJ_ROOM_TEMP    TJC_PAGE_NAME ".t3"
#define TJC_OBJ_PRESSURE     TJC_PAGE_NAME ".t4"
#define TJC_OBJ_HEART_RATE   TJC_PAGE_NAME ".t5"
#define TJC_OBJ_BREATH_RATE  TJC_PAGE_NAME ".t6"
#define TJC_OBJ_BODY_TEMP    TJC_PAGE_NAME ".t7"
#define TJC_OBJ_ALARM        TJC_PAGE_NAME ".t8"
#define TJC_PERSON_PRESENT_FORCE_N 0.05f
#define TJC_CURVE_PAGE_NAME "page2"
#define TJC_OBJ_CURVE_HEART "t_hr_now"
#define TJC_OBJ_CURVE_BREATH "t_br_now"
#define TJC_OBJ_CURVE_MOVE "t_move_now"

/* Fill these with the waveform component IDs shown in the HMI editor.
 * They are intentionally disabled by default because TJC/Nextion add
 * commands use numeric component IDs, not objname strings. */
#ifndef TJC_WF_HR_ID
#define TJC_WF_HR_ID 1
#endif
#ifndef TJC_WF_BR_ID
#define TJC_WF_BR_ID 5
#endif

static const char *TAG = "tjc_usart";
static bool s_usart_ready;
static TaskHandle_t s_tjc_rx_task;
static volatile int s_tjc_current_page = -1;
static usart_tjc_rx_callback_t s_tjc_rx_callback;
static void *s_tjc_rx_user_ctx;

static bool usart_tjc_warn_temp(float temp_c);
static bool usart_tjc_warn_humidity(float humidity_pct);
static bool usart_tjc_warn_air(float mq135_ppm);
static bool usart_tjc_warn_lux(float lux);
static bool usart_tjc_warn_pressure(float pressure_kpa);
static bool usart_tjc_warn_neck(float neck_temp_c);
static bool usart_tjc_warn_radar(uint8_t heart_bpm, uint8_t breath_bpm);
static void usart_tjc_rx_task(void *arg);
static void usart_tjc_process_rx_frame(const uint8_t *frame, size_t len);

static esp_err_t usart_tjc_send_command(const char *command, int len)
{
    if (!s_usart_ready || command == NULL || len <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_data[USART_TJC_CMD_BUF_SIZE + 3] = {0};
    for (int i = 0; i < len; i++) {
        tx_data[i] = (uint8_t)command[i];
    }

    tx_data[len] = 0xFF;
    tx_data[len + 1] = 0xFF;
    tx_data[len + 2] = 0xFF;

    const int tx_len = len + 3;
    const int written = uart_write_bytes(USART_TJC_UART_NUM,
                                         (const char *)tx_data,
                                         tx_len);
    if (written != tx_len) {
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "send: %s", command);
    return uart_wait_tx_done(USART_TJC_UART_NUM, pdMS_TO_TICKS(100));
}

esp_err_t usart_init(void)
{
    if (s_usart_ready) {
        return ESP_OK;
    }

    const uart_config_t uart_config = {
        .baud_rate = USART_TJC_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(USART_TJC_UART_NUM, &uart_config);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(USART_TJC_UART_NUM,
                       USART_TJC_TX_GPIO,
                       USART_TJC_RX_GPIO,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_driver_install(USART_TJC_UART_NUM, USART_TJC_BUF_SIZE, 0, 0, NULL,
                              ESP_INTR_FLAG_SHARED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    s_usart_ready = true;
    if (s_tjc_rx_task == NULL) {
        BaseType_t task_ret = xTaskCreate(usart_tjc_rx_task,
                                          "tjc_rx",
                                          USART_TJC_RX_TASK_STACK,
                                          NULL,
                                          USART_TJC_RX_TASK_PRIO,
                                          &s_tjc_rx_task);
        if (task_ret != pdPASS) {
            s_tjc_rx_task = NULL;
            s_usart_ready = false;
            ESP_LOGW(TAG, "rx task create failed");
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

void usart_tjc_set_rx_callback(usart_tjc_rx_callback_t callback,
                               void *user_ctx)
{
    s_tjc_rx_callback = callback;
    s_tjc_rx_user_ctx = user_ctx;
}

int usart_tjc_get_current_page(void)
{
    return s_tjc_current_page;
}

static void usart_tjc_dispatch_rx_message(const char *message, int page_id)
{
    usart_tjc_rx_callback_t callback = s_tjc_rx_callback;
    if (callback != NULL) {
        callback(message, page_id, s_tjc_rx_user_ctx);
    }
}

static void usart_tjc_process_rx_frame(const uint8_t *frame, size_t len)
{
    if (frame == NULL || len == 0) {
        return;
    }

    /* TJC/Nextion sendme response: 0x66 <page_id> 0xFF 0xFF 0xFF */
    if (len >= 2 && frame[0] == 0x66) {
        int page_id = frame[1];
        s_tjc_current_page = page_id;
        ESP_LOGI(TAG, "page=%d", page_id);
        usart_tjc_dispatch_rx_message(NULL, page_id);
        return;
    }

    bool printable = true;
    for (size_t i = 0; i < len; ++i) {
        uint8_t ch = frame[i];
        if (ch == '\0' || ch == '\r' || ch == '\n' || ch == '\t') {
            continue;
        }
        if (ch < 0x20 || ch > 0x7E) {
            printable = false;
            break;
        }
    }
    if (!printable) {
        ESP_LOGD(TAG, "ignore binary frame first=0x%02x len=%u",
                 frame[0], (unsigned)len);
        return;
    }

    char message[USART_TJC_RX_FRAME_MAX + 1] = {0};
    size_t copy_len = len < USART_TJC_RX_FRAME_MAX ? len : USART_TJC_RX_FRAME_MAX;
    memcpy(message, frame, copy_len);
    message[copy_len] = '\0';

    char *start = message;
    while (*start == ' ' || *start == '\r' || *start == '\n' || *start == '\t') {
        ++start;
    }
    char *end = start + strlen(start);
    while (end > start &&
           (end[-1] == ' ' || end[-1] == '\r' || end[-1] == '\n' || end[-1] == '\t')) {
        *--end = '\0';
    }
    if (*start == '\0') {
        return;
    }

    ESP_LOGI(TAG, "rx: %s", start);
    usart_tjc_dispatch_rx_message(start, -1);
}

static void usart_tjc_rx_task(void *arg)
{
    (void)arg;
    uint8_t rx_buf[64];
    uint8_t frame[USART_TJC_RX_FRAME_MAX];
    size_t frame_len = 0;
    int ff_count = 0;

    while (1) {
        int len = uart_read_bytes(USART_TJC_UART_NUM,
                                  rx_buf,
                                  sizeof(rx_buf),
                                  pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; ++i) {
            uint8_t ch = rx_buf[i];
            if (frame_len >= sizeof(frame)) {
                ESP_LOGW(TAG, "rx frame overflow, reset");
                frame_len = 0;
                ff_count = 0;
            }

            frame[frame_len++] = ch;
            if (ch == 0xFF) {
                ff_count++;
                if (ff_count >= 3) {
                    if (frame_len >= 3) {
                        usart_tjc_process_rx_frame(frame, frame_len - 3);
                    }
                    frame_len = 0;
                    ff_count = 0;
                }
            } else {
                ff_count = 0;
            }
        }
    }
}

esp_err_t usart_tjc_set_number(const char *object_name, int32_t value)
{
    if (object_name == NULL || object_name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    char command[USART_TJC_CMD_BUF_SIZE] = {0};
    int len = snprintf(command, sizeof(command), "%s.val=%ld", object_name, (long)value);
    if (len < 0 || len >= (int)sizeof(command)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return usart_tjc_send_command(command, len);
}

esp_err_t usart_tjc_set_text(const char *object_name, const char *text)
{
    if (object_name == NULL || object_name[0] == '\0' || text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char command[USART_TJC_CMD_BUF_SIZE] = {0};
    int len = snprintf(command, sizeof(command), "%s.txt=\"%s\"", object_name, text);
    if (len < 0 || len >= (int)sizeof(command)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return usart_tjc_send_command(command, len);
}

esp_err_t usart_tjc_set_text_color(const char *object_name, uint16_t rgb565)
{
    if (object_name == NULL || object_name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    char command[USART_TJC_CMD_BUF_SIZE] = {0};
    int len = snprintf(command, sizeof(command), "%s.pco=%u", object_name, (unsigned)rgb565);
    if (len < 0 || len >= (int)sizeof(command)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return usart_tjc_send_command(command, len);
}

static void usart_tjc_keep_first_error(esp_err_t err, esp_err_t *first_err)
{
    if (first_err != NULL && *first_err == ESP_OK && err != ESP_OK) {
        *first_err = err;
    }
}

static esp_err_t usart_tjc_set_text_with_color(const char *object_name,
                                               const char *text,
                                               uint16_t color)
{
    esp_err_t first_err = usart_tjc_set_text(object_name, text);
    esp_err_t color_err = usart_tjc_set_text_color(object_name, color);
    if (first_err == ESP_OK) {
        first_err = color_err;
    }
    return first_err;
}

static esp_err_t usart_tjc_set_float_field(const char *object_name,
                                           float value,
                                           const char *format,
                                           bool valid,
                                           bool warning,
                                           uint16_t normal_color)
{
    if (!valid || !isfinite(value)) {
        return usart_tjc_set_text_with_color(object_name, "--", TJC_COLOR_GRAY);
    }

    char text[16] = {0};
    int len = snprintf(text, sizeof(text), format, value);
    if (len < 0 || len >= (int)sizeof(text)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return usart_tjc_set_text_with_color(object_name,
                                         text,
                                         warning ? TJC_COLOR_RED : normal_color);
}

static esp_err_t usart_tjc_set_uint_field(const char *object_name,
                                          unsigned value,
                                          bool valid,
                                          bool warning,
                                          uint16_t normal_color)
{
    if (!valid) {
        return usart_tjc_set_text_with_color(object_name, "--", TJC_COLOR_GRAY);
    }

    char text[16] = {0};
    int len = snprintf(text, sizeof(text), "%u", value);
    if (len < 0 || len >= (int)sizeof(text)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return usart_tjc_set_text_with_color(object_name,
                                         text,
                                         warning ? TJC_COLOR_RED : normal_color);
}

static uint8_t __attribute__((unused)) usart_tjc_map_range_u8(unsigned value,
                                                              unsigned in_min,
                                                              unsigned in_max)
{
    if (in_max <= in_min) {
        return 0;
    }
    if (value <= in_min) {
        return 0;
    }
    if (value >= in_max) {
        return 255;
    }
    return (uint8_t)(((value - in_min) * 255u + ((in_max - in_min) / 2u)) /
                     (in_max - in_min));
}

esp_err_t usart_tjc_add_waveform(uint8_t component_id,
                                 uint8_t channel,
                                 uint8_t value)
{
    char command[USART_TJC_CMD_BUF_SIZE] = {0};
    int len = snprintf(command, sizeof(command), "add %u,%u,%u",
                       (unsigned)component_id,
                       (unsigned)channel,
                       (unsigned)value);
    if (len < 0 || len >= (int)sizeof(command)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return usart_tjc_send_command(command, len);
}

esp_err_t usart_tjc_update_curve_page(uint8_t heart_bpm,
                                      uint8_t breath_bpm,
                                      bool radar_valid,
                                      bool fsr_valid,
                                      float fsr_max_n)
{
    esp_err_t first_err = ESP_OK;
    bool radar_warn = radar_valid && usart_tjc_warn_radar(heart_bpm, breath_bpm);

    usart_tjc_keep_first_error(
        usart_tjc_set_uint_field(TJC_OBJ_CURVE_HEART, heart_bpm,
                                 radar_valid, radar_warn, TJC_COLOR_ORANGE),
        &first_err);
    usart_tjc_keep_first_error(
        usart_tjc_set_uint_field(TJC_OBJ_CURVE_BREATH, breath_bpm,
                                 radar_valid, radar_warn, TJC_COLOR_CYAN),
        &first_err);

    const char *move_text = "--";
    uint16_t move_color = TJC_COLOR_GRAY;
    if (fsr_valid) {
        if (fsr_max_n >= TJC_PERSON_PRESENT_FORCE_N) {
            move_text = "ACTIVE";
            move_color = TJC_COLOR_ORANGE;
        } else {
            move_text = "LOW";
            move_color = TJC_COLOR_GREEN;
        }
    }
    usart_tjc_keep_first_error(
        usart_tjc_set_text_with_color(TJC_OBJ_CURVE_MOVE, move_text, move_color),
        &first_err);

#if TJC_WF_HR_ID >= 0
    usart_tjc_keep_first_error(
        usart_tjc_add_waveform((uint8_t)TJC_WF_HR_ID,
                               0,
                               radar_valid ? usart_tjc_map_range_u8(heart_bpm, 40, 140) : 20),
        &first_err);
#endif

#if TJC_WF_BR_ID >= 0
    usart_tjc_keep_first_error(
        usart_tjc_add_waveform((uint8_t)TJC_WF_BR_ID,
                               0,
                               radar_valid ? usart_tjc_map_range_u8(breath_bpm, 8, 30) : 20),
        &first_err);
#endif

    return first_err;
}

esp_err_t usart_tjc_set_t4_temp_c(float temp_c)
{
    return usart_tjc_set_float_field(TJC_OBJ_ROOM_TEMP,
                                     temp_c,
                                     "%.1f",
                                     true,
                                     usart_tjc_warn_temp(temp_c),
                                     TJC_COLOR_CYAN);
}

esp_err_t usart_tjc_set_t5_mq135_ppm(float ppm)
{
    return usart_tjc_set_float_field(TJC_OBJ_AIR_QUALITY,
                                     ppm,
                                     "%.1f",
                                     true,
                                     usart_tjc_warn_air(ppm),
                                     TJC_COLOR_GREEN);
}

esp_err_t usart_tjc_set_t6_lux(float lux)
{
    return usart_tjc_set_float_field(TJC_OBJ_LIGHT,
                                     lux,
                                     "%.0f",
                                     true,
                                     usart_tjc_warn_lux(lux),
                                     TJC_COLOR_ORANGE);
}

esp_err_t usart_tjc_set_t7_pressure_kpa(float pressure_kpa)
{
    return usart_tjc_set_float_field(TJC_OBJ_PRESSURE,
                                     pressure_kpa,
                                     "%.1f",
                                     true,
                                     usart_tjc_warn_pressure(pressure_kpa),
                                     TJC_COLOR_PURPLE);
}

esp_err_t usart_tjc_set_t9_humidity(float humidity)
{
    return usart_tjc_set_float_field(TJC_OBJ_HUMIDITY,
                                     humidity,
                                     "%.0f",
                                     true,
                                     usart_tjc_warn_humidity(humidity),
                                     TJC_COLOR_CYAN);
}

static bool usart_tjc_warn_temp(float temp_c)
{
    return temp_c < 10.0f || temp_c > 35.0f;
}

static bool usart_tjc_warn_humidity(float humidity_pct)
{
    return humidity_pct < 30.0f || humidity_pct > 75.0f;
}

static bool usart_tjc_warn_air(float mq135_ppm)
{
    return mq135_ppm > 800.0f;
}

static bool usart_tjc_warn_lux(float lux)
{
    return lux > 1000.0f;
}

static bool usart_tjc_warn_pressure(float pressure_kpa)
{
    return pressure_kpa < 0.05f || pressure_kpa > 8.0f;
}

static bool usart_tjc_warn_neck(float neck_temp_c)
{
    return neck_temp_c < 32.0f || neck_temp_c > 37.8f;
}

static bool usart_tjc_warn_radar(uint8_t heart_bpm, uint8_t breath_bpm)
{
    return heart_bpm < 45 || heart_bpm > 120 || breath_bpm < 8 || breath_bpm > 24;
}

esp_err_t usart_tjc_update_sleep_home(float temp_c,
                                      bool temp_valid,
                                      float humidity_pct,
                                      bool humidity_valid,
                                      float mq135_ppm,
                                      bool mq135_valid,
                                      float lux,
                                      bool lux_valid,
                                      float pressure_kpa,
                                      bool pressure_valid,
                                      float neck_temp_c,
                                      bool neck_temp_valid,
                                      uint8_t heart_bpm,
                                      uint8_t breath_bpm,
                                      bool radar_valid,
                                      bool fsr_valid,
                                      float fsr_max_n)
{
    esp_err_t first_err = ESP_OK;
    bool person_present = fsr_valid && (fsr_max_n >= TJC_PERSON_PRESENT_FORCE_N);

    bool temp_warn = temp_valid && usart_tjc_warn_temp(temp_c);
    bool humi_warn = humidity_valid && usart_tjc_warn_humidity(humidity_pct);
    bool air_warn = mq135_valid && usart_tjc_warn_air(mq135_ppm);
    bool lux_warn = lux_valid && usart_tjc_warn_lux(lux);
    bool pressure_warn = pressure_valid && usart_tjc_warn_pressure(pressure_kpa);
    bool neck_warn = neck_temp_valid && usart_tjc_warn_neck(neck_temp_c);
    bool radar_warn = radar_valid && usart_tjc_warn_radar(heart_bpm, breath_bpm);

    usart_tjc_keep_first_error(
        usart_tjc_set_float_field(TJC_OBJ_AIR_QUALITY, mq135_ppm, "%.1f",
                                  mq135_valid, air_warn, TJC_COLOR_GREEN),
        &first_err);
    usart_tjc_keep_first_error(
        usart_tjc_set_float_field(TJC_OBJ_LIGHT, lux, "%.0f",
                                  lux_valid, lux_warn, TJC_COLOR_ORANGE),
        &first_err);
    usart_tjc_keep_first_error(
        usart_tjc_set_float_field(TJC_OBJ_HUMIDITY, humidity_pct, "%.0f",
                                  humidity_valid, humi_warn, TJC_COLOR_CYAN),
        &first_err);
    usart_tjc_keep_first_error(
        usart_tjc_set_float_field(TJC_OBJ_ROOM_TEMP, temp_c, "%.1f",
                                  temp_valid, temp_warn, TJC_COLOR_CYAN),
        &first_err);
    usart_tjc_keep_first_error(
        usart_tjc_set_float_field(TJC_OBJ_PRESSURE, pressure_kpa, "%.1f",
                                  pressure_valid, pressure_warn, TJC_COLOR_PURPLE),
        &first_err);
    usart_tjc_keep_first_error(
        usart_tjc_set_float_field(TJC_OBJ_BODY_TEMP, neck_temp_c, "%.1f",
                                  neck_temp_valid, neck_warn, TJC_COLOR_ORANGE),
        &first_err);

    if (person_present || radar_valid) {
        usart_tjc_keep_first_error(
            usart_tjc_set_uint_field(TJC_OBJ_HEART_RATE, heart_bpm,
                                     radar_valid, radar_warn, TJC_COLOR_ORANGE),
            &first_err);
        usart_tjc_keep_first_error(
            usart_tjc_set_uint_field(TJC_OBJ_BREATH_RATE, breath_bpm,
                                     radar_valid, radar_warn, TJC_COLOR_ORANGE),
            &first_err);
    } else {
        usart_tjc_keep_first_error(
            usart_tjc_set_text_with_color(TJC_OBJ_HEART_RATE, "--", TJC_COLOR_GRAY),
            &first_err);
        usart_tjc_keep_first_error(
            usart_tjc_set_text_with_color(TJC_OBJ_BREATH_RATE, "--", TJC_COLOR_GRAY),
            &first_err);
    }

    const char *alarm_text = "NORMAL";
    uint16_t alarm_color = TJC_COLOR_GREEN;

    if (!person_present && !radar_valid) {
        alarm_text = "STANDBY";
        alarm_color = TJC_COLOR_GRAY;
    }

    if (!mq135_valid) {
        alarm_text = "AIR OFFLINE";
        alarm_color = TJC_COLOR_GRAY;
    } else if (!lux_valid) {
        alarm_text = "LIGHT OFFLINE";
        alarm_color = TJC_COLOR_GRAY;
    } else if (!humidity_valid || !temp_valid) {
        alarm_text = "ENV OFFLINE";
        alarm_color = TJC_COLOR_GRAY;
    } else if (!pressure_valid) {
        alarm_text = "PRESS OFFLINE";
        alarm_color = TJC_COLOR_GRAY;
    } else if (!neck_temp_valid) {
        alarm_text = "BODY OFFLINE";
        alarm_color = TJC_COLOR_GRAY;
    } else if (person_present && !radar_valid) {
        alarm_text = "RADAR OFFLINE";
        alarm_color = TJC_COLOR_GRAY;
    } else if (neck_warn) {
        alarm_text = "BODY TEMP WARN";
        alarm_color = TJC_COLOR_RED;
    } else if (radar_warn) {
        alarm_text = "VITAL WARN";
        alarm_color = TJC_COLOR_RED;
    } else if (pressure_warn) {
        alarm_text = "PRESS WARN";
        alarm_color = TJC_COLOR_RED;
    } else if (air_warn) {
        alarm_text = "AIR WARN";
        alarm_color = TJC_COLOR_RED;
    } else if (temp_warn) {
        alarm_text = "TEMP WARN";
        alarm_color = TJC_COLOR_RED;
    } else if (humi_warn) {
        alarm_text = "HUMI WARN";
        alarm_color = TJC_COLOR_RED;
    } else if (lux_warn) {
        alarm_text = "LIGHT WARN";
        alarm_color = TJC_COLOR_RED;
    }

    usart_tjc_keep_first_error(
        usart_tjc_set_text_with_color(TJC_OBJ_ALARM, alarm_text, alarm_color),
        &first_err);

    return first_err;
}

