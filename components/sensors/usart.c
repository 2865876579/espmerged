#include "usart.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define USART_TJC_UART_NUM UART_NUM_1
#define USART_TJC_TX_GPIO GPIO_NUM_17
#define USART_TJC_RX_GPIO GPIO_NUM_18
#define USART_TJC_BAUD_RATE 115200
#define USART_TJC_BUF_SIZE 256
#define USART_TJC_CMD_BUF_SIZE 64

static const char *TAG = "tjc_usart";
static bool s_usart_ready;

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
    return ESP_OK;
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

static esp_err_t usart_tjc_set_float_text(const char *object_name,
                                          float value,
                                          const char *format)
{
    if (object_name == NULL || format == NULL || !isfinite(value)) {
        return ESP_ERR_INVALID_ARG;
    }

    char text[16] = {0};
    int len = snprintf(text, sizeof(text), format, value);
    if (len < 0 || len >= (int)sizeof(text)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return usart_tjc_set_text(object_name, text);
}

esp_err_t usart_tjc_set_t4_temp_c(float temp_c)
{
    return usart_tjc_set_float_text("page1.t4", temp_c, "%.2f");
}

esp_err_t usart_tjc_set_t5_mq135_ppm(float ppm)
{
    return usart_tjc_set_float_text("page1.t5", ppm, "%.2f");
}

esp_err_t usart_tjc_set_t6_lux(float lux)
{
    return usart_tjc_set_float_text("page1.t6", lux, "%.1f");
}

esp_err_t usart_tjc_set_t7_pressure_kpa(float pressure_kpa)
{
    return usart_tjc_set_float_text("page1.t7", pressure_kpa, "%.2f");
}

esp_err_t usart_tjc_set_t9_humidity(float humidity)
{
    return usart_tjc_set_float_text("page1.t9", humidity, "%.2f");
}
