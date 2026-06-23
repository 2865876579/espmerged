#include "sht31.h"
#include <stdint.h>
#include "esp_rom_sys.h"

/* 单次测量命令：高重复性，关闭 clock stretching。 */
#define SHT31_CMD_MEAS_HIGHREP_STRETCH_DISABLED 0x2400

/* 高重复性测量典型等待时间约 15 ms，这里留 20 ms 余量。 */
#define SHT31_MEAS_DELAY_US 20000

/* I2C 读写超时时间，单位是 FreeRTOS tick。 */
#define SHT31_I2C_TIMEOUT_TICKS 1000

/* SHT31 数据手册使用 CRC-8，多项式 0x31，初始值 0xFF。 */
static uint8_t sht31_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/* SHT31 的命令为 16 位，高字节先发送。 */
static esp_err_t sht31_write_command(const sht31_t *dev, uint16_t command)
{
    uint8_t cmd[2] = {
        (uint8_t)(command >> 8),
        (uint8_t)(command & 0xFF),
    };

    return i2c_master_write_to_device(dev->i2c_port,
                                      dev->address,
                                      cmd,
                                      sizeof(cmd),
                                      SHT31_I2C_TIMEOUT_TICKS);
}

esp_err_t sht31_init(sht31_t *dev, i2c_port_t i2c_port, uint8_t address)
{
    /* 防止传入空指针导致程序崩溃。 */
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_port = i2c_port;
    dev->address = address;

    return ESP_OK;
}

esp_err_t sht31_read_temp_humi(const sht31_t *dev, float *temperature, float *humidity)
{
    /* 输出参数必须有效，否则无法返回温湿度结果。 */
    if (dev == NULL || temperature == NULL || humidity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 发送测量命令，SHT31 开始采集温度和湿度。 */
    esp_err_t ret = sht31_write_command(dev, SHT31_CMD_MEAS_HIGHREP_STRETCH_DISABLED);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 等待传感器完成测量。 */
    esp_rom_delay_us(SHT31_MEAS_DELAY_US);

    /*
     * 读取 6 字节：
     * data[0..1] 温度原始值，data[2] 温度 CRC；
     * data[3..4] 湿度原始值，data[5] 湿度 CRC。
     */
    uint8_t data[6] = {0};
    ret = i2c_master_read_from_device(dev->i2c_port,
                                      dev->address,
                                      data,
                                      sizeof(data),
                                      SHT31_I2C_TIMEOUT_TICKS);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 校验 CRC，避免总线干扰造成错误数据参与换算。 */
    if (sht31_crc8(&data[0], 2) != data[2] || sht31_crc8(&data[3], 2) != data[5]) {
        return ESP_ERR_INVALID_CRC;
    }

    /* SHT31 输出为大端格式，高字节在前。 */
    uint16_t raw_temp = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_humi = ((uint16_t)data[3] << 8) | data[4];

    /* 数据手册换算公式：T = -45 + 175 * raw / 65535，RH = 100 * raw / 65535。 */
    *temperature = -45.0f + (175.0f * (float)raw_temp / 65535.0f);
    *humidity = 100.0f * (float)raw_humi / 65535.0f;

    return ESP_OK;
}
