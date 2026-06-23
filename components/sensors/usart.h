#ifndef USART_H
#define USART_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t usart_init(void);
esp_err_t usart_tjc_set_number(const char *object_name, int32_t value);
esp_err_t usart_tjc_set_text(const char *object_name, const char *text);
esp_err_t usart_tjc_set_t4_temp_c(float temp_c);
esp_err_t usart_tjc_set_t5_mq135_ppm(float ppm);
esp_err_t usart_tjc_set_t6_lux(float lux);
esp_err_t usart_tjc_set_t7_pressure_kpa(float pressure_kpa);
esp_err_t usart_tjc_set_t9_humidity(float humidity);

#ifdef __cplusplus
}
#endif

#endif  // USART_H
