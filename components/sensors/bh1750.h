#ifndef BH1750_H
#define BH1750_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define BH1750_ADDR_LOW       0x23
#define BH1750_ADDR_HIGH      0x5C
#define BH1750_DEFAULT_ADDR   BH1750_ADDR_LOW

/* BH1750 的测量模式命令。
 * CONT 表示连续测量；ONE_TIME 表示测量一次后自动进入低功耗状态。
 * HIGH_RES_2 的分辨率更高，最终 lux 值按数据手册通常还需要再除以 2。
 */
typedef enum {
    BH1750_MODE_CONT_HIGH_RES      = 0x10,
    BH1750_MODE_CONT_HIGH_RES_2    = 0x11,
    BH1750_MODE_CONT_LOW_RES       = 0x13,
    BH1750_MODE_ONE_TIME_HIGH_RES  = 0x20,
    BH1750_MODE_ONE_TIME_HIGH_RES_2 = 0x21,
    BH1750_MODE_ONE_TIME_LOW_RES   = 0x23,
} bh1750_mode_t;

/* 保存一个 BH1750 设备的基本信息，后续读写都通过这个结构体完成。 */
typedef struct {
    i2c_port_t i2c_port;    /* ESP32 使用的 I2C 控制器编号，例如 I2C_NUM_0 */
    uint8_t address;        /* BH1750 的 7 位 I2C 地址，常用 0x23 或 0x5C */
    bh1750_mode_t mode;     /* 当前测量模式 */
} bh1750_t;

/* 初始化 I2C 总线并配置 BH1750。
 * sda_io/scl_io 根据你的实际接线填写。
 */
esp_err_t bh1750_init(bh1750_t *dev,
                      i2c_port_t i2c_port,
                      gpio_num_t sda_io,
                      gpio_num_t scl_io,
                      uint32_t clk_speed_hz,
                      uint8_t address);

/* BH1750 基本控制命令。 */
esp_err_t bh1750_power_on(const bh1750_t *dev);
esp_err_t bh1750_power_down(const bh1750_t *dev);
esp_err_t bh1750_reset(const bh1750_t *dev);
esp_err_t bh1750_set_mode(bh1750_t *dev, bh1750_mode_t mode);

/* 读取光照强度，单位 lux。 */
esp_err_t bh1750_read_lux(const bh1750_t *dev, float *lux);

#endif
