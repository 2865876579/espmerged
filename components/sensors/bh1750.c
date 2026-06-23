#include "bh1750.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* BH1750 基础命令，写 1 个字节即可完成控制。 */
#define BH1750_CMD_POWER_DOWN  0x00
#define BH1750_CMD_POWER_ON    0x01
#define BH1750_CMD_RESET       0x07

/* 向 BH1750 发送一个命令字节，例如上电、复位、切换测量模式。 */
static esp_err_t bh1750_write_cmd(const bh1750_t *dev, uint8_t command)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_write_to_device(dev->i2c_port,
                                      dev->address,
                                      &command,
                                      1,
                                      pdMS_TO_TICKS(1000));
}

/* 不同分辨率模式需要等待的测量时间不同。
 * 低分辨率最快，高分辨率通常需要约 120 ms，这里留到 180 ms 更稳。
 */
static TickType_t bh1750_measure_delay_ticks(bh1750_mode_t mode)
{
    switch (mode) {
    case BH1750_MODE_CONT_LOW_RES:
    case BH1750_MODE_ONE_TIME_LOW_RES:
        return pdMS_TO_TICKS(24);
    default:
        return pdMS_TO_TICKS(180);
    }
}

esp_err_t bh1750_init(bh1750_t *dev,
                      i2c_port_t i2c_port,
                      gpio_num_t sda_io,
                      gpio_num_t scl_io,
                      uint32_t clk_speed_hz,
                      uint8_t address)
{
    /* 参数检查：地址只能是 ADDR 脚低电平对应的 0x23，或高电平对应的 0x5C。 */
    if (dev == NULL || clk_speed_hz == 0 ||
        (address != BH1750_ADDR_LOW && address != BH1750_ADDR_HIGH)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 配置 ESP32-S3 的 I2C 主机模式。
     * 这里打开内部上拉；实际硬件上建议 SDA/SCL 仍然接 4.7k 左右外部上拉。
     */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed_hz,
        .clk_flags = 0,
    };

    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 安装 I2C 驱动。如果这个 I2C 端口已经安装过，则认为可以继续使用。 */
    dev->i2c_port = i2c_port;
    dev->address = address;
    dev->mode = BH1750_MODE_CONT_HIGH_RES;

    /* BH1750 上电后先等待一小段时间，再进入默认的连续高分辨率模式。 */
    ret = bh1750_power_on(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    return bh1750_set_mode(dev, dev->mode);
}

esp_err_t bh1750_power_on(const bh1750_t *dev)
{
    return bh1750_write_cmd(dev, BH1750_CMD_POWER_ON);
}

esp_err_t bh1750_power_down(const bh1750_t *dev)
{
    return bh1750_write_cmd(dev, BH1750_CMD_POWER_DOWN);
}

esp_err_t bh1750_reset(const bh1750_t *dev)
{
    /* 数据手册要求复位命令需要在 Power On 状态下发送。 */
    esp_err_t ret = bh1750_power_on(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    return bh1750_write_cmd(dev, BH1750_CMD_RESET);
}

esp_err_t bh1750_set_mode(bh1750_t *dev, bh1750_mode_t mode)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 模式本质上也是一个命令字节，写入成功后记录当前模式。 */
    esp_err_t ret = bh1750_write_cmd(dev, (uint8_t)mode);
    if (ret == ESP_OK) {
        dev->mode = mode;
    }

    return ret;
}

esp_err_t bh1750_read_lux(const bh1750_t *dev, float *lux)
{
    if (dev == NULL || lux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 单次测量模式每读一次都要重新发送测量命令；
     * 连续测量模式则只需要读传感器内部不断更新的结果。
     */
    if (dev->mode >= BH1750_MODE_ONE_TIME_HIGH_RES) {
        esp_err_t ret = bh1750_write_cmd(dev, (uint8_t)dev->mode);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    vTaskDelay(bh1750_measure_delay_ticks(dev->mode));

    /* BH1750 返回 2 个字节，高字节在前，低字节在后。 */
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_read_from_device(dev->i2c_port,
                                                dev->address,
                                                data,
                                                sizeof(data),
                                                pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    /* 默认测量时间 MTreg=69 时，数据手册给出的换算关系为 lux = raw / 1.2。 */
    *lux = (float)raw / 1.2f;

    return ESP_OK;
}
