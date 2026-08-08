#include "audio_out.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "audio_out";

// ═══════════════════════════════════════════════════
//  借鉴 xiaozhi 项目：两根独立 I2S 总线
//  I2S0 = TX(喇叭 MAX98357A)
//  I2S1 = RX(麦克风 INMP441)
//  ★ TX 常开，空闲写静音填充，避免开关跳变产生异响
// ═══════════════════════════════════════════════════

#define I2S0_BCLK_GPIO   GPIO_NUM_5
#define I2S0_LRC_GPIO    GPIO_NUM_4
#define I2S0_DOUT_GPIO   GPIO_NUM_6

#define I2S1_SCK_GPIO    GPIO_NUM_11
#define I2S1_WS_GPIO     GPIO_NUM_10
#define I2S1_DIN_GPIO    GPIO_NUM_3

#define SAMPLE_RATE      16000
#define DMA_DESC_NUM     4
#define DMA_FRAME_NUM    511

static i2s_chan_handle_t s_tx_chan = NULL;
static i2s_chan_handle_t s_rx_chan = NULL;
static volatile bool s_tx_enabled = false;

// ── AEC 参考信号 ring buffer ──────────────────────────
// 借鉴 xiaozhi：播放音频时同步抄一份给 AFE 做回声消除
#define REF_BUF_SAMPLES  9600   // 600ms @ 16kHz
static int16_t *s_ref_ring = NULL;
// 软件 AEC 参考必须是一条与麦克风等速前进的 PCM 流。
// 原先每次都按“当前写指针 - 固定延迟”取一块，当 60ms 播放包和
// 16ms AFE 帧不对齐时会重复/跳过参考样本，AEC 反而无法收敛。这里改为
// 单调读写指针的 FIFO，与小智 Box Lite 的软件回采方式一致。
static uint64_t s_ref_write_pos = 0;
static uint64_t s_ref_read_pos = 0;
static portMUX_TYPE s_ref_lock = portMUX_INITIALIZER_UNLOCKED;


void audio_out_init(void)
{
    s_ref_ring = heap_caps_calloc(REF_BUF_SAMPLES, sizeof(int16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_ref_ring) {
        ESP_LOGI(TAG, "AEC ref ring in PSRAM: %d bytes",
                 REF_BUF_SAMPLES * (int)sizeof(int16_t));
    } else {
        s_ref_ring = calloc(REF_BUF_SAMPLES, sizeof(int16_t));
        ESP_LOGW(TAG, "AEC ref ring fell back to internal RAM");
    }

    // I2S0 TX — MAX98357A 喇叭
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    tx_chan_cfg.dma_desc_num = DMA_DESC_NUM;
    tx_chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    tx_chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &s_tx_chan, NULL));

    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S0_BCLK_GPIO, .ws = I2S0_LRC_GPIO,
            .dout = I2S0_DOUT_GPIO, .din = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &tx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
    s_tx_enabled = true;
    // ★ I2S 信号增强驱动抗 WiFi 干扰（LRC 不加，紧挨 DIN 怕串扰麦克风）
    gpio_set_drive_capability(I2S0_BCLK_GPIO, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(I2S0_LRC_GPIO,  GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(I2S0_DOUT_GPIO, GPIO_DRIVE_CAP_3);

    // ★ 预写静音填满 DMA，避免首帧到达前 DMA 循环播残留数据
    int16_t *silence = calloc(1, DMA_FRAME_NUM * 4);  // stereo 16bit
    if (silence) {
        for (int i = 0; i < DMA_DESC_NUM; i++) {
            size_t w = 0;
            i2s_channel_write(s_tx_chan, silence, DMA_FRAME_NUM * 4, &w, portMAX_DELAY);
        }
        free(silence);
    }

    // I2S1 RX — INMP441 麦克风
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    rx_chan_cfg.dma_desc_num = DMA_DESC_NUM;
    rx_chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &s_rx_chan));

    i2s_std_config_t rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_BOTH,
            .ws_width       = I2S_SLOT_BIT_WIDTH_32BIT,
            .ws_pol         = false, .bit_shift = true,
            .left_align     = false, .big_endian = false, .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S1_SCK_GPIO, .ws = I2S1_WS_GPIO,
            .dout = I2S_GPIO_UNUSED, .din = I2S1_DIN_GPIO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx_chan, &rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));
}


void audio_out_start(void)
{
    if (s_tx_chan && !s_tx_enabled) {
        ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
        s_tx_enabled = true;
    }
}

void audio_out_stop(void)
{
    if (s_tx_chan && s_tx_enabled) {
        ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
        s_tx_enabled = false;
    }
}


void audio_out_write(const uint8_t *data, size_t len)
{
    if (s_tx_chan == NULL || data == NULL || len == 0) return;
    size_t written = 0;
    i2s_channel_write(s_tx_chan, data, len, &written, portMAX_DELAY);
    if (!s_ref_ring) {
        return;
    }

    // ★ AEC 参考：抄 mono PCM 到 FIFO ring（有锁，双核 cache 同步）
    int frames = (int)len / 4;
    const int16_t *stereo = (const int16_t *)data;
    portENTER_CRITICAL(&s_ref_lock);
    uint64_t queued = s_ref_write_pos - s_ref_read_pos;
    if (queued + (uint64_t)frames > REF_BUF_SAMPLES) {
        // AFE 一时跟不上时丢弃最旧参考，不能让旧音频与当前麦克风错位。
        s_ref_read_pos = s_ref_write_pos + (uint64_t)frames - REF_BUF_SAMPLES;
    }
    for (int i = 0; i < frames; i++) {
        s_ref_ring[(s_ref_write_pos + (uint64_t)i) % REF_BUF_SAMPLES] = stereo[i * 2];
    }
    s_ref_write_pos += (uint64_t)frames;
    portEXIT_CRITICAL(&s_ref_lock);
}


i2s_chan_handle_t audio_out_get_rx_chan(void) { return s_rx_chan; }


int audio_out_read_ref(int16_t *out, int want)
{
    if (!out || want <= 0) {
        return 0;
    }
    if (!s_ref_ring) {
        memset(out, 0, want * sizeof(int16_t));
        return want;
    }

    // ★ 有锁顺序读：每个播放样本只给 AFE 一次。
    portENTER_CRITICAL(&s_ref_lock);
    uint64_t available = s_ref_write_pos - s_ref_read_pos;
    int copy = available < (uint64_t)want ? (int)available : want;
    for (int i = 0; i < copy; i++) {
        out[i] = s_ref_ring[(s_ref_read_pos + (uint64_t)i) % REF_BUF_SAMPLES];
    }
    s_ref_read_pos += (uint64_t)copy;
    portEXIT_CRITICAL(&s_ref_lock);

    // 当前没有播放参考时补零，不重放旧参考帧。
    if (copy < want) {
        memset(out + copy, 0, (want - copy) * sizeof(int16_t));
    }
    return want;
}


void audio_out_flush_silence(void)
{
    // ★ 填满全部 DMA 描述符，消除 TTS 结束后残留的旧音频
    if (!s_tx_chan) return;
    static int16_t silence[511 * 2];  // 1 descriptor = 511 stereo samples
    static bool inited = false;
    if (!inited) { memset(silence, 0, sizeof(silence)); inited = true; }
    for (int i = 0; i < DMA_DESC_NUM; i++) {
        size_t w = 0;
        i2s_channel_write(s_tx_chan, silence, sizeof(silence), &w, portMAX_DELAY);
    }
}


void audio_out_play_test_tone(void)
{
    const int freq = 440;
    const int duration_ms = 1000;
    const int total_samples = SAMPLE_RATE * duration_ms / 1000;
    const int chunk_samples = 256;

    int16_t buf[chunk_samples * 2];
    audio_out_start();
    int sample_idx = 0;
    while (sample_idx < total_samples) {
        int n = (total_samples - sample_idx > chunk_samples)
                    ? chunk_samples : (total_samples - sample_idx);
        for (int i = 0; i < n; i++) {
            double t = (double)(sample_idx + i) / SAMPLE_RATE;
            int16_t val = (int16_t)(0.3 * 32767.0 * sin(2.0 * M_PI * freq * t));
            buf[i * 2]     = val;
            buf[i * 2 + 1] = val;
        }
        audio_out_write((const uint8_t *)buf, n * 2 * sizeof(int16_t));
        sample_idx += n;
    }
    audio_out_stop();
}
