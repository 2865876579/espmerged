#include "ads1115_fsr402.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADS1115_REG_CONVERSION 0x00  // 转换结果寄存器。
#define ADS1115_REG_CONFIG     0x01  // 配置寄存器。

#define ADS1115_CONFIG_OS_SINGLE    0x8000  // 写 1 开始一次单次转换。
#define ADS1115_CONFIG_MODE_SINGLE  0x0100  // 单次转换模式，省电且便于按需读取。
#define ADS1115_CONFIG_COMP_DISABLE 0x0003  // 关闭比较器功能。
#define ADS1115_CONFIG_OS_READY     0x8000  // 读配置寄存器时，该位为 1 表示转换完成。

static const char *TAG = "ads1115";

static esp_err_t ads1115_write_reg(const ads1115_t *dev, uint8_t reg, uint16_t value)
{
    // ADS1115 寄存器写入格式：寄存器地址 + 16 位数据高字节 + 低字节。
    const uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };

    return i2c_master_write_to_device(dev->i2c_port, dev->address, data, sizeof(data),
                                      pdMS_TO_TICKS(100));
}

static esp_err_t ads1115_read_reg(const ads1115_t *dev, uint8_t reg, uint16_t *value)
{
    // 先写入要读取的寄存器地址，再连续读回 2 个字节。
    uint8_t data[2] = {0};
    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(dev->i2c_port, dev->address,
                                                     &reg, 1, data, sizeof(data),
                                                     pdMS_TO_TICKS(100)),
                        TAG, "read register 0x%02x failed", reg);

    *value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t ads1115_init(const ads1115_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG, "device is NULL");
    ESP_RETURN_ON_FALSE(dev->address != 0, ESP_ERR_INVALID_ARG, TAG, "address is invalid");

    // 配置 ESP32 作为 I2C 主机，ADS1115 作为 I2C 从机。
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = dev->sda_io,
        .scl_io_num = dev->scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = dev->clk_speed_hz,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(dev->i2c_port, &conf), TAG, "i2c param config failed");

    esp_err_t ret = i2c_driver_install(dev->i2c_port, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        // 如果 I2C 驱动已经安装过，直接认为初始化成功。
        return ESP_OK;
    }

    return ret;
}

esp_err_t ads1115_read_raw(const ads1115_t *dev, ads1115_mux_t mux, int16_t *raw)
{
    ESP_RETURN_ON_FALSE(dev != NULL && raw != NULL, ESP_ERR_INVALID_ARG, TAG, "bad argument");

    // 组合 ADS1115 配置字：启动转换、选择通道、选择量程、单次模式、采样率、关闭比较器。
    const uint16_t config = ADS1115_CONFIG_OS_SINGLE |
                            (uint16_t)mux |
                            (uint16_t)dev->gain |
                            ADS1115_CONFIG_MODE_SINGLE |
                            (uint16_t)dev->data_rate |
                            ADS1115_CONFIG_COMP_DISABLE;

    ESP_RETURN_ON_ERROR(ads1115_write_reg(dev, ADS1115_REG_CONFIG, config),
                        TAG, "start conversion failed");

    // 按当前采样率估算截止时间，避免一直等待。
    const TickType_t deadline = xTaskGetTickCount() +
                                pdMS_TO_TICKS(ads1115_conversion_time_ms(dev->data_rate) + 20);
    uint16_t status = 0;

    // 轮询配置寄存器的 OS 位，直到转换完成或超时。
    do {
        vTaskDelay(pdMS_TO_TICKS(2));
        ESP_RETURN_ON_ERROR(ads1115_read_reg(dev, ADS1115_REG_CONFIG, &status),
                            TAG, "poll conversion failed");
    } while ((status & ADS1115_CONFIG_OS_READY) == 0 && xTaskGetTickCount() < deadline);

    ESP_RETURN_ON_ERROR(ads1115_read_reg(dev, ADS1115_REG_CONVERSION, &status),
                        TAG, "read conversion failed");

    // 转换结果寄存器本身是有符号 16 位数。
    *raw = (int16_t)status;
    return ESP_OK;
}

float ads1115_raw_to_voltage(const ads1115_t *dev, int16_t raw)
{
    // fs_range 是当前增益对应的满量程电压。
    float fs_range = 4.096f;

    switch (dev->gain) {
    case ADS1115_GAIN_6_144V:
        fs_range = 6.144f;
        break;
    case ADS1115_GAIN_4_096V:
        fs_range = 4.096f;
        break;
    case ADS1115_GAIN_2_048V:
        fs_range = 2.048f;
        break;
    case ADS1115_GAIN_1_024V:
        fs_range = 1.024f;
        break;
    case ADS1115_GAIN_0_512V:
        fs_range = 0.512f;
        break;
    case ADS1115_GAIN_0_256V:
        fs_range = 0.256f;
        break;
    default:
        break;
    }

    // ADS1115 有符号输出范围为 -32768 到 32767。
    return ((float)raw * fs_range) / 32768.0f;
}

uint32_t ads1115_conversion_time_ms(ads1115_data_rate_t data_rate)
{
    // 返回略取整后的单次转换时间，用于读数前等待。
    switch (data_rate) {
    case ADS1115_DR_8SPS:
        return 125;
    case ADS1115_DR_16SPS:
        return 63;
    case ADS1115_DR_32SPS:
        return 32;
    case ADS1115_DR_64SPS:
        return 16;
    case ADS1115_DR_128SPS:
        return 8;
    case ADS1115_DR_250SPS:
        return 4;
    case ADS1115_DR_475SPS:
        return 3;
    case ADS1115_DR_860SPS:
        return 2;
    default:
        return 8;
    }
}
