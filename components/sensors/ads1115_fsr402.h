#ifndef ADS1115_H
#define ADS1115_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define ADS1115_DEFAULT_ADDR 0x48  // ADDR 引脚接 GND 时的默认 I2C 地址。

typedef enum {
    // 差分输入通道配置。
    ADS1115_MUX_AIN0_AIN1 = 0x0000,
    ADS1115_MUX_AIN0_AIN3 = 0x1000,
    ADS1115_MUX_AIN1_AIN3 = 0x2000,
    ADS1115_MUX_AIN2_AIN3 = 0x3000,
    // 单端输入通道配置，读取某个 AIN 引脚相对 GND 的电压。
    ADS1115_MUX_AIN0_GND  = 0x4000,
    ADS1115_MUX_AIN1_GND  = 0x5000,
    ADS1115_MUX_AIN2_GND  = 0x6000,
    ADS1115_MUX_AIN3_GND  = 0x7000,
} ads1115_mux_t;

typedef enum {
    // 可编程增益放大器量程。量程越小，分辨率越高，但输入不能超过量程。
    ADS1115_GAIN_6_144V = 0x0000,
    ADS1115_GAIN_4_096V = 0x0200,
    ADS1115_GAIN_2_048V = 0x0400,
    ADS1115_GAIN_1_024V = 0x0600,
    ADS1115_GAIN_0_512V = 0x0800,
    ADS1115_GAIN_0_256V = 0x0A00,
} ads1115_gain_t;

typedef enum {
    // ADS1115 每秒采样次数，SPS 越高转换越快，噪声通常也会略大。
    ADS1115_DR_8SPS   = 0x0000,
    ADS1115_DR_16SPS  = 0x0020,
    ADS1115_DR_32SPS  = 0x0040,
    ADS1115_DR_64SPS  = 0x0060,
    ADS1115_DR_128SPS = 0x0080,
    ADS1115_DR_250SPS = 0x00A0,
    ADS1115_DR_475SPS = 0x00C0,
    ADS1115_DR_860SPS = 0x00E0,
} ads1115_data_rate_t;

typedef struct {
    i2c_port_t i2c_port;              // 使用的 ESP32 I2C 控制器。
    gpio_num_t sda_io;                // I2C SDA 引脚。
    gpio_num_t scl_io;                // I2C SCL 引脚。
    uint32_t clk_speed_hz;            // I2C 时钟频率。
    uint8_t address;                  // ADS1115 I2C 地址。
    ads1115_gain_t gain;              // ADC 输入量程。
    ads1115_data_rate_t data_rate;    // ADC 转换速率。
} ads1115_t;

// 初始化 I2C 总线，用于和 ADS1115 通信。
esp_err_t ads1115_init(const ads1115_t *dev);
// 启动一次单次转换，并读取原始 16 位 ADC 数据。
esp_err_t ads1115_read_raw(const ads1115_t *dev, ads1115_mux_t mux, int16_t *raw);
// 根据当前增益量程，把 ADC 原始值换算成电压。
float ads1115_raw_to_voltage(const ads1115_t *dev, int16_t raw);
// 根据采样率估算一次转换大约需要等待的时间。
uint32_t ads1115_conversion_time_ms(ads1115_data_rate_t data_rate);

#endif  // ADS1115_H
