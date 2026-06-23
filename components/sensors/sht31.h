#ifndef SHT31_H
#define SHT31_H

#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

/* SHT31 默认 I2C 地址。ADDR 引脚接低电平时一般为 0x44，接高电平时一般为 0x45。 */
#define SHT31_I2C_ADDR_DEFAULT 0x44

/* SHT31 设备对象：记录使用的 I2C 端口和从机地址。 */
typedef struct {
    i2c_port_t i2c_port;
    uint8_t address;
} sht31_t;

/* 初始化 SHT31 设备对象，不会初始化 ESP32 的 I2C 总线。 */
esp_err_t sht31_init(sht31_t *dev, i2c_port_t i2c_port, uint8_t address);

/* 读取温湿度，temperature 单位为摄氏度，humidity 单位为 %RH。 */
esp_err_t sht31_read_temp_humi(const sht31_t *dev, float *temperature, float *humidity);

#endif
