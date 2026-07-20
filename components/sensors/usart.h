#ifndef USART_H
#define USART_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*usart_tjc_rx_callback_t)(const char *message,
                                        int page_id,
                                        void *user_ctx);

esp_err_t usart_init(void);
void usart_tjc_set_rx_callback(usart_tjc_rx_callback_t callback,
                               void *user_ctx);
int usart_tjc_get_current_page(void);
esp_err_t usart_tjc_set_number(const char *object_name, int32_t value);
esp_err_t usart_tjc_set_text(const char *object_name, const char *text);
esp_err_t usart_tjc_set_t4_temp_c(float temp_c);
esp_err_t usart_tjc_set_t5_mq135_ppm(float ppm);
esp_err_t usart_tjc_set_t6_lux(float lux);
esp_err_t usart_tjc_set_t7_pressure_kpa(float pressure_kpa);
esp_err_t usart_tjc_set_t9_humidity(float humidity);
esp_err_t usart_tjc_set_text_color(const char *object_name, uint16_t rgb565);
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
                                      float fsr_max_n);
esp_err_t usart_tjc_add_waveform(uint8_t component_id,
                                 uint8_t channel,
                                 uint8_t value);
esp_err_t usart_tjc_update_curve_page(uint8_t heart_bpm,
                                      uint8_t breath_bpm,
                                      bool radar_valid,
                                      bool motion_valid,
                                      float motion_level);

#ifdef __cplusplus
}
#endif

#endif  // USART_H
