#include "afe_wake_word.h"
#include "monitor_log.h"
#include "audio_out.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_wn_models.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "afe_wake";

// AFE v2.x: handle from esp_afe_handle_from_config() + data from create_from_config()
static const esp_afe_sr_iface_t *s_afe_handle = NULL;
static esp_afe_sr_data_t        *s_afe_data   = NULL;

// 播放期间的本地停止口令。仅在 TTS/音乐正在播放时运行，
// 避免常驻 MultiNet 占用 CPU，也避免普通对话中的“停止”被误拦截。
#define PLAYBACK_STOP_COMMAND_ID 1
static esp_mn_iface_t *s_mn_handle = NULL;
static model_iface_data_t *s_mn_data = NULL;
static volatile bool s_playback_stop_detection_enabled = false;
static volatile bool s_playback_stop_detected = false;

// 唤醒词回调
static wake_word_callback_t s_wake_cb = NULL;

// 录音采集缓存 (PSRAM)
// ★ 借鉴 xiaozhi：用 spinlock 保护 capture buffer 的并发访问
static int16_t *s_capture_buf   = NULL;
static volatile int  s_capture_max   = 0;
static volatile int  s_capture_idx   = 0;
static volatile bool s_capture_done  = false;
static volatile bool s_capture_seen_speech = false;
static volatile bool s_capture_vad_speech = false;
static volatile int  s_capture_vad_speech_samples = 0;
static volatile int  s_capture_last_voice_idx = 0;
static volatile int  s_capture_no_speech_ms = 3000;
static volatile int  s_capture_end_silence_ms = 500;
static portMUX_TYPE s_capture_lock = portMUX_INITIALIZER_UNLOCKED;

#define CAPTURE_SAMPLE_RATE          16000
#define CAPTURE_NO_SPEECH_NORMAL_MS  3000
#define CAPTURE_NO_SPEECH_BARGE_MS   1200
#define CAPTURE_END_SILENCE_NORMAL_MS 500
#define CAPTURE_END_SILENCE_BARGE_MS  320
#define CAPTURE_MIN_VAD_SPEECH_MS    240
#define CAPTURE_AC_AVG_THRESHOLD     350
#define CAPTURE_PEAK_THRESHOLD       2500
#define CAPTURE_ACTIVE_LEVEL         1000
// 防重复触发冷却 (fetch 周期数)
#define COOLDOWN_TICKS  150
#define AFE_INTERNAL_PRIORITY 1
#define AFE_FEED_TASK_PRIORITY 8
#define AFE_FETCH_TASK_PRIORITY 3
#define AFE_FEED_TASK_STACK_BYTES (8 * 1024)
// MultiNet7 的 Zipformer 推理会在调用任务栈上运行 ESP-DL 内核。官方
// ESP-Skainet 示例使用内部 RAM 的 8 KiB 栈。不能把该栈放在 PSRAM，
// 否则实机进入 detect() 后会在
// dl_nn_doubleswish_i16 一带异常，并表现为说出停止词后立即重启。
#define AFE_FETCH_TASK_STACK_BYTES (8 * 1024)
#define AFE_FEED_TASK_CORE 0
#define AFE_INTERNAL_CORE 1
#define AFE_ENABLE_AEC 1
static volatile int s_cooldown = 0;

// I2S1 RX 句柄
static i2s_chan_handle_t s_rx_chan = NULL;


// Keep twice the inference window so a lock-free snapshot cannot be
// overwritten while the snore task copies the latest two seconds.
#define AFE_RECENT_PCM_RING_SAMPLES 65536
#define SNORE_PREEMPHASIS_ALPHA 0.3f
static int16_t *s_recent_pcm_ring = NULL;
static volatile uint32_t s_recent_pcm_total = 0;
static portMUX_TYPE s_recent_pcm_lock = portMUX_INITIALIZER_UNLOCKED;

// feed/fetch 参数
static int s_feed_chunksize  = 0;
static int s_feed_channels   = 0;
static int s_fetch_chunksize = 0;
static bool s_feed_task_with_caps = false;
static bool s_fetch_task_with_caps = false;

static void delete_current_task(bool with_caps)
{
    if (with_caps) {
        vTaskDeleteWithCaps(NULL);
    } else {
        vTaskDelete(NULL);
    }
}

static void recent_pcm_write(const int16_t *pcm, int samples)
{
    if (!s_recent_pcm_ring || !pcm || samples <= 0) {
        return;
    }
    if (samples > AFE_RECENT_PCM_RING_SAMPLES) {
        pcm += samples - AFE_RECENT_PCM_RING_SAMPLES;
        samples = AFE_RECENT_PCM_RING_SAMPLES;
    }

    portENTER_CRITICAL(&s_recent_pcm_lock);
    uint32_t total = s_recent_pcm_total;
    int pos = (int)(total % AFE_RECENT_PCM_RING_SAMPLES);
    int first = AFE_RECENT_PCM_RING_SAMPLES - pos;
    if (first > samples) first = samples;
    memcpy(s_recent_pcm_ring + pos, pcm, first * sizeof(int16_t));
    if (samples > first) {
        memcpy(s_recent_pcm_ring, pcm + first, (samples - first) * sizeof(int16_t));
    }
    s_recent_pcm_total = total + (uint32_t)samples;
    portEXIT_CRITICAL(&s_recent_pcm_lock);
}

int afe_recent_audio_copy_latest(int16_t *out, int samples, uint32_t *out_total_written)
{
    if (!out || samples <= 0 || !s_recent_pcm_ring) {
        if (out_total_written) *out_total_written = 0;
        return 0;
    }
    if (samples > AFE_RECENT_PCM_RING_SAMPLES) {
        samples = AFE_RECENT_PCM_RING_SAMPLES;
    }

    // Only snapshot the writer position under the spinlock. Copying a full
    // inference window from PSRAM while holding this lock used to stall the
    // real-time AFE feed task and made WakeNet miss wake words.
    portENTER_CRITICAL(&s_recent_pcm_lock);
    uint32_t total = s_recent_pcm_total;
    int available = total < AFE_RECENT_PCM_RING_SAMPLES
                        ? (int)total
                        : AFE_RECENT_PCM_RING_SAMPLES;
    int to_copy = samples <= available ? samples : available;
    int start = (int)((total - (uint32_t)to_copy) % AFE_RECENT_PCM_RING_SAMPLES);
    portEXIT_CRITICAL(&s_recent_pcm_lock);

    int first = AFE_RECENT_PCM_RING_SAMPLES - start;
    if (first > to_copy) first = to_copy;
    if (to_copy > 0) {
        memcpy(out, s_recent_pcm_ring + start, first * sizeof(int16_t));
        if (to_copy > first) {
            memcpy(out + first, s_recent_pcm_ring, (to_copy - first) * sizeof(int16_t));
        }
    }

    if (out_total_written) {
        *out_total_written = total;
    }
    return to_copy;
}

static bool capture_chunk_has_voice(const int16_t *data, int samples)
{
    if (!data || samples <= 0) {
        return false;
    }

    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += data[i];
    }
    int dc = (int)(sum / samples);

    int64_t ac_sum = 0;
    int peak = 0;
    int active = 0;
    for (int i = 0; i < samples; i++) {
        int delta = (int)data[i] - dc;
        int a = delta >= 0 ? delta : -delta;
        ac_sum += a;
        if (a > peak) {
            peak = a;
        }
        if (a >= CAPTURE_ACTIVE_LEVEL) {
            active++;
        }
    }

    int ac_avg = (int)(ac_sum / samples);
    int min_active = samples / 20;
    if (min_active < 8) {
        min_active = 8;
    }

    return ac_avg >= CAPTURE_AC_AVG_THRESHOLD
        || (peak >= CAPTURE_PEAK_THRESHOLD && active >= min_active);
}


// ============================================================
//  Feed 任务: I2S1 DMA → 32→16 提取 → 累积 → feed AFE
//  参考小智项目：独立 feed/fetch 双任务，feed 按时序严格调用
// ============================================================
static void afe_feed_task(void *arg)
{
    int ch = s_feed_channels;
    int feed_bytes = s_feed_chunksize * ch * (int)sizeof(int16_t);

    // feed 间隔: chunksize/16000 秒
    int interval_ms = (s_feed_chunksize * 1000) / 16000;
    if (interval_ms < 10) interval_ms = 10;

    int16_t *feed_buf = heap_caps_calloc(s_feed_chunksize, ch * sizeof(int16_t),
                                          MALLOC_CAP_SPIRAM);
    if (!feed_buf) {
        ESP_LOGE(TAG, "feed malloc failed");
        delete_current_task(s_feed_task_with_caps);
        return;
    }

    int16_t *ref_buf = NULL;
    if (ch >= 2) {
        ref_buf = heap_caps_malloc(s_feed_chunksize * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (!ref_buf) {
            ref_buf = malloc(s_feed_chunksize * sizeof(int16_t));
        }
        if (!ref_buf) {
            ESP_LOGE(TAG, "ref_buf malloc failed");
            free(feed_buf);
            delete_current_task(s_feed_task_with_caps);
            return;
        }
    }

    int16_t *snore_mono_buf = heap_caps_malloc(s_feed_chunksize * sizeof(int16_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!snore_mono_buf) {
        snore_mono_buf = malloc(s_feed_chunksize * sizeof(int16_t));
    }
    if (!snore_mono_buf) {
        ESP_LOGW(TAG, "recent PCM buffer disabled");
    }

    ESP_LOGI(TAG, "feed task started, chunk=%d ch=%d aec=%d", s_feed_chunksize, ch, ch >= 2);

    // DMA 累积缓冲：每帧 511×2×4=4088 bytes (32-bit stereo I2S, 驱动限制511帧)
    const int dma_frame_num    = 511;  // 对齐 audio_out.c DMA_FRAME_NUM
    const int dma_frame_bytes  = dma_frame_num * 2 * (int)sizeof(int32_t);
    const int acc_capacity     = s_feed_chunksize + dma_frame_num;
    int32_t *acc = heap_caps_malloc(acc_capacity * 2 * sizeof(int32_t),
                                     MALLOC_CAP_SPIRAM);
    if (!acc) {
        ESP_LOGE(TAG, "acc malloc failed");
        free(snore_mono_buf);
        free(feed_buf);
        delete_current_task(s_feed_task_with_caps);
        return;
    }
    int acc_samples = 0;
    int16_t snore_previous_sample = 0;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(interval_ms);
    int cycle = 0;

    while (1) {
        // 读一个 DMA 帧
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(s_rx_chan,
            acc + acc_samples * 2,
            dma_frame_bytes, &bytes_read,
            pdMS_TO_TICKS(interval_ms * 2));
        int samples_read = (int)(bytes_read / (2 * sizeof(int32_t)));

        if (err == ESP_OK && samples_read > 0) {
            acc_samples += samples_read;
        }

        // 累积够一帧 → 提取 mono 16-bit → feed
        if (acc_samples >= s_feed_chunksize) {
            cycle++;
            if (ch >= 2 && ref_buf) {
                audio_out_read_ref(ref_buf, s_feed_chunksize);
            }

            memset(feed_buf, 0, feed_bytes);
            for (int i = 0; i < s_feed_chunksize; i++) {
                // INMP441: 24-bit 左对齐在 32-bit slot → 取高 16bit
                int16_t mic = (int16_t)(acc[i * 2] >> 16);
                feed_buf[i * ch] = mic;
                if (snore_mono_buf) {
                    float filtered = (float)mic -
                                     SNORE_PREEMPHASIS_ALPHA * (float)snore_previous_sample;
                    snore_previous_sample = mic;
                    if (filtered > INT16_MAX) filtered = INT16_MAX;
                    if (filtered < INT16_MIN) filtered = INT16_MIN;
                    snore_mono_buf[i] = (int16_t)lrintf(filtered);
                }
                if (ch >= 2) {
                    feed_buf[i * ch + 1] = ref_buf[i];  // 参考 = 喇叭输出
                }
            }

            // 第一帧快速验证 I2S 格式（仅首次）
            if (snore_mono_buf) {
                recent_pcm_write(snore_mono_buf, s_feed_chunksize);
            }

            if (cycle == 1) {
                int16_t mic0 = (int16_t)(acc[0] >> 16);
                ESP_LOGI(TAG, "mic working, first sample=%d", mic0);
            }

            if (ch >= 2) {
                audio_out_aec_observe_mic(feed_buf, ch, s_feed_chunksize);
            }

            // 剩余数据前移
            int remain = acc_samples - s_feed_chunksize;
            if (remain > 0) {
                memmove(acc, acc + s_feed_chunksize * 2,
                        remain * 2 * sizeof(int32_t));
            }
            acc_samples = remain;

            s_afe_handle->feed(s_afe_data, feed_buf);
        }
        vTaskDelayUntil(&last_wake, period);
    }
}


// ============================================================
//  Fetch 任务: fetch AFE → 唤醒检测 + 录音采集
//  参考小智项目：fetch 负责唤醒词 + 命令词检测
// ============================================================
static void afe_fetch_task(void *arg)
{
    bool multinet_running = false;
    while (1) {
        afe_fetch_result_t *res = s_afe_handle->fetch_with_delay
            ? s_afe_handle->fetch_with_delay(s_afe_data, portMAX_DELAY)
            : s_afe_handle->fetch(s_afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            vTaskDelay(1);
            continue;
        }

        // 冷却计数
        if (s_cooldown > 0) s_cooldown--;

        // 唤醒词检测
        if (res->wakeup_state == WAKENET_DETECTED
            && s_cooldown == 0) {
            MONITOR_DEBUG_PRINTF("*** WAKE WORD DETECTED! ***\n");
            s_cooldown = COOLDOWN_TICKS;
            // ★ 不 disable wakenet — 否则音频通路关闭，采集不到数据
            if (s_wake_cb) s_wake_cb();
        }

        // 播放打断走本地 MultiNet，而不是先上传一大段“用户语音 +
        // 扬声器回声”再做云端 STT。开关由 fetch 任务自己应用，以免
        // clean()/detect() 与其它任务并发访问模型内部状态。
        bool should_run_multinet = s_playback_stop_detection_enabled
                                   && s_mn_handle && s_mn_data;
        if (should_run_multinet != multinet_running) {
            s_mn_handle->clean(s_mn_data);
            multinet_running = should_run_multinet;
        }
        if (multinet_running && res->data != NULL) {
            esp_mn_state_t mn_state = s_mn_handle->detect(s_mn_data, res->data);
            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = s_mn_handle->get_results(s_mn_data);
                if (mn_result && mn_result->num > 0
                    && mn_result->command_id[0] == PLAYBACK_STOP_COMMAND_ID) {
                    ESP_LOGI(TAG,
                             "local stop detected: phrase=%d prob=%.3f text=%s",
                             mn_result->phrase_id[0],
                             (double)mn_result->prob[0],
                             mn_result->string);
                    s_playback_stop_detected = true;
                    s_playback_stop_detection_enabled = false;
                    multinet_running = false;
                }
                s_mn_handle->clean(s_mn_data);
            } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
                // 超时只表示当前检测窗结束；播放未结束时立即开新窗。
                s_mn_handle->clean(s_mn_data);
            }
        }

        // 录音采集: 从 AFE 降噪输出中拷贝，优先用 AFE VAD 做端点检测。
        // 保持 memcpy 和索引更新在同一临界区内，避免 finish/start 释放缓冲区。
        portENTER_CRITICAL(&s_capture_lock);
        bool capturing = (s_capture_buf != NULL && !s_capture_done
                          && s_capture_idx < s_capture_max && res->data != NULL);
        int16_t *capture_buf_ptr = s_capture_buf;
        int cur_idx = s_capture_idx;
        int cur_max = s_capture_max;
        int to_copy = 0;
        if (capturing) {
            int fetch_samples = res->data_size / (int)sizeof(int16_t);
            int remain = cur_max - cur_idx;
            to_copy = (fetch_samples < remain) ? fetch_samples : remain;
            memcpy(s_capture_buf + cur_idx, res->data, to_copy * sizeof(int16_t));
            s_capture_idx = cur_idx + to_copy;
        }
        portEXIT_CRITICAL(&s_capture_lock);

        if (capturing && to_copy > 0) {
            bool energy_speech = capture_chunk_has_voice(res->data, to_copy);

            bool vad_speech = (res->vad_state == VAD_SPEECH);
            portENTER_CRITICAL(&s_capture_lock);
            if (s_capture_buf != capture_buf_ptr) {
                portEXIT_CRITICAL(&s_capture_lock);
                continue;
            }
            if (vad_speech) {
                s_capture_seen_speech = true;
                s_capture_vad_speech_samples += to_copy;
                if (s_capture_vad_speech_samples >=
                    (CAPTURE_SAMPLE_RATE * CAPTURE_MIN_VAD_SPEECH_MS / 1000)) {
                    s_capture_vad_speech = true;
                }
                s_capture_last_voice_idx = s_capture_idx;
            } else if (!s_capture_seen_speech && energy_speech) {
                s_capture_seen_speech = true;
                s_capture_last_voice_idx = s_capture_idx;
            }

            int no_speech_limit = CAPTURE_SAMPLE_RATE * s_capture_no_speech_ms / 1000;
            int endpoint_ms = s_capture_end_silence_ms;
            // Long, clearly established speech can use a faster endpoint. Short
            // starts keep the full window so a natural hesitation is not clipped.
            if (s_capture_vad_speech_samples >= CAPTURE_SAMPLE_RATE * 12 / 10
                && endpoint_ms > 420) {
                endpoint_ms = 420;
            }
            int end_silence_limit = CAPTURE_SAMPLE_RATE * endpoint_ms / 1000;
            if (s_capture_idx >= cur_max) {
                s_capture_done = true;
            } else if (!s_capture_seen_speech && s_capture_idx >= no_speech_limit) {
                s_capture_done = true;
            } else if (s_capture_seen_speech &&
                       (s_capture_idx - s_capture_last_voice_idx) >= end_silence_limit) {
                s_capture_done = true;
            }
            portEXIT_CRITICAL(&s_capture_lock);
        }
    }
}


// ============================================================
//  公开 API
// ============================================================

int afe_wake_word_init(wake_word_callback_t cb)
{
    s_wake_cb = cb;

    s_rx_chan = audio_out_get_rx_chan();
    if (!s_rx_chan) {
        ESP_LOGE(TAG, "I2S1 RX handle is NULL — call audio_out_init() first");
        return -1;
    }

    // 1. 加载模型分区
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models || models->num <= 0) {
        ESP_LOGE(TAG, "esp_srmodel_init failed — no models in 'model' partition");
        return -1;
    }
    // 2. 动态获取唤醒词模型名
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    if (!wn_name) {
        ESP_LOGE(TAG, "no wake word model found in partition");
        return -1;
    }

    // 3. 创建 AFE 配置。MR 的 R 通道由 audio_out 的软件回采提供。
    ESP_LOGI(TAG, "heap before AFE create: internal free=%u largest=%u psram free=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // 当前链路的最终消费者是 WakeNet/MultiNet，因此使用语音识别（SR）
    // 前端，而不是照搬语音通话（VC/VOIP）前端。VC/VOIP 与本项目同时启用
    // WakeNet、MultiNet 和 PSRAM 任务栈后，实机启动时出现了 PSRAM heap
    // 元数据损坏，崩溃最终随机暴露在 cJSON_Delete() 或 lwIP malloc() 中。
    // 保留 M+R 软件回采和高性能 AEC，但让 AFE 类型、AEC 模式与识别场景匹配。
    afe_config_t *cfg = afe_config_init(AFE_ENABLE_AEC ? "MR" : "M", models,
                                        AFE_TYPE_SR,
                                        AFE_MODE_HIGH_PERF);
    if (!cfg) {
        ESP_LOGE(TAG, "afe_config_init failed");
        return -1;
    }

    // 4. 配置
    cfg->wakenet_model_name = wn_name;
    cfg->wakenet_init       = true;
    cfg->wakenet_mode       = DET_MODE_95;
    cfg->aec_init           = AFE_ENABLE_AEC;
    if (AFE_ENABLE_AEC) {
        cfg->aec_mode       = AEC_MODE_SR_HIGH_PERF;
        cfg->aec_nlp_level  = AEC_NLP_LEVEL_AGGR;
    }
    cfg->se_init            = false;
    cfg->ns_init            = false;
    cfg->vad_init           = true;
    cfg->vad_mode           = VAD_MODE_0;
    cfg->vad_model_name     = NULL;
    cfg->vad_min_speech_ms  = 96;
    cfg->vad_min_noise_ms   = 100;
    cfg->vad_delay_ms       = 128;
    cfg->agc_init           = false;
    cfg->afe_perferred_core     = AFE_INTERNAL_CORE;
    cfg->afe_perferred_priority = AFE_INTERNAL_PRIORITY;
    cfg->memory_alloc_mode      = AFE_MEMORY_ALLOC_MORE_PSRAM;

    // 5. 校验（调整不兼容参数）
    cfg = afe_config_check(cfg);

    // ★ 在 check 之后设 gain，防止被覆盖
    cfg->afe_perferred_core     = AFE_INTERNAL_CORE;
    cfg->afe_perferred_priority = AFE_INTERNAL_PRIORITY;
    cfg->afe_linear_gain = 1.0f;

    // 6. 获取 handle 并创建实例
    s_afe_handle = esp_afe_handle_from_config(cfg);
    if (!s_afe_handle) {
        ESP_LOGE(TAG, "esp_afe_handle_from_config failed");
        afe_config_free(cfg);
        return -1;
    }

    s_afe_data = s_afe_handle->create_from_config(cfg);
    if (!s_afe_data) {
        ESP_LOGE(TAG, "create_from_config failed");
        afe_config_free(cfg);
        return -1;
    }

    ESP_LOGI(TAG, "heap after AFE create: internal free=%u largest=%u psram free=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // 7. 提高唤醒阈值，降低扬声器回放和环境声误唤醒概率。
    s_afe_handle->set_wakenet_threshold(s_afe_data, 1, 0.65f);

    // 8. 查询 feed/fetch 参数
    s_feed_channels   = s_afe_handle->get_channel_num(s_afe_data);
    s_feed_chunksize  = s_afe_handle->get_feed_chunksize(s_afe_data);
    s_fetch_chunksize = s_afe_handle->get_fetch_chunksize(s_afe_data);
    int samp_rate     = s_afe_handle->get_samp_rate(s_afe_data);

    if (!s_recent_pcm_ring) {
        s_recent_pcm_ring = heap_caps_calloc(AFE_RECENT_PCM_RING_SAMPLES, sizeof(int16_t),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_recent_pcm_ring) {
            s_recent_pcm_ring = calloc(AFE_RECENT_PCM_RING_SAMPLES, sizeof(int16_t));
        }
        if (s_recent_pcm_ring) {
            ESP_LOGI(TAG, "recent PCM ring ready: %d samples", AFE_RECENT_PCM_RING_SAMPLES);
        } else {
            ESP_LOGW(TAG, "recent PCM ring alloc failed; snore detector will wait");
        }
    }

    ESP_LOGI(TAG, "AFE init OK: feed=%d(ch=%d) fetch=%d rate=%d",
             s_feed_chunksize, s_feed_channels, s_fetch_chunksize, samp_rate);

    // MultiNet7 检测结果只在本地用于停止播放；动态中文命令必须用拼音
    // 添加。初始化失败时保留云端 STT 打断作为后备。
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (mn_name) {
        s_mn_handle = esp_mn_handle_from_name(mn_name);
    }
    if (s_mn_handle) {
        s_mn_data = s_mn_handle->create(mn_name, 3000);
    }
    if (s_mn_handle && s_mn_data
        && esp_mn_commands_alloc(s_mn_handle, s_mn_data) == ESP_OK) {
        // 中文 MultiNet 的动态命令输入不是 UTF-8 汉字，而是以空格分隔的
        // 无声调拼音。直接传“停止/不要讲了”会被 check_speech_command()
        // 全部判为 invalid，设备启动日志中的“No commands available”正是
        // 因此产生。短于三个音节的词也容易误触发，所以保留明确且稳健的
        // 3～6 音节停止口令，并将同义短语映射到同一个 command id。
        static const char *const stop_phrases[] = {
            "ting xia lai",
            "bie shuo le",
            "bu yao shuo le",
            "bie jiang le",
            "bu yao jiang le",
            "ting zhi bo fang",
            "zan ting bo fang",
            "xiao an ting zhi",
            "xiao an bie jiang le",
            "xiao an bu yao jiang le",
        };
        size_t commands_added = 0;
        for (size_t i = 0; i < sizeof(stop_phrases) / sizeof(stop_phrases[0]); i++) {
            if (esp_mn_commands_add(PLAYBACK_STOP_COMMAND_ID, stop_phrases[i]) == ESP_OK) {
                commands_added++;
            }
        }
        esp_mn_error_t *command_errors = esp_mn_commands_update();
        if (commands_added == 0) {
            ESP_LOGE(TAG, "playback stop commands init failed");
            s_mn_handle->destroy(s_mn_data);
            s_mn_data = NULL;
            s_mn_handle = NULL;
            esp_mn_commands_free();
        } else {
            // 使用 MultiNet7 模型自带的阈值。之前手工设为 0.55
            // 会使扬声器播放期间的近端短口令难以达到阈值。
            if (command_errors != NULL) {
                ESP_LOGW(TAG, "MultiNet ignored %d invalid stop phrases",
                         command_errors->num);
            }
            int mn_chunksize = s_mn_handle->get_samp_chunksize(s_mn_data);
            if (mn_chunksize != s_fetch_chunksize) {
                ESP_LOGE(TAG, "MultiNet/AFE chunk mismatch: mn=%d afe=%d",
                         mn_chunksize, s_fetch_chunksize);
                s_mn_handle->destroy(s_mn_data);
                s_mn_data = NULL;
                s_mn_handle = NULL;
                esp_mn_commands_free();
            } else {
                ESP_LOGI(TAG, "MultiNet stop commands ready: %u",
                         (unsigned)commands_added);
                esp_mn_active_commands_print();
            }
        }
    }

    // 9. 释放配置（不再需要）
    afe_config_free(cfg);

    // 与 ESP-Skainet 官方 MultiNet 示例一致：识别任务使用内部 RAM 栈并
    // 固定在 core 1，采集任务固定在 core 0。此前 xTaskCreateWithCaps() 把
    // afe_fetch 栈放到 PSRAM，普通 AFE fetch 尚可运行，但 MultiNet7 的
    // ESP-DL 推理进入后会崩溃。
    s_fetch_task_with_caps = false;
    BaseType_t fetch_ret = xTaskCreatePinnedToCore(
        afe_fetch_task, "afe_fetch", AFE_FETCH_TASK_STACK_BYTES, NULL,
        AFE_FETCH_TASK_PRIORITY, NULL, AFE_INTERNAL_CORE);
    if (fetch_ret != pdPASS) {
        printf("[初始化] afe_fetch 内部栈创建失败: ret=%ld internal=%u largest=%u\n",
               (long)fetch_ret,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        return -1;
    }

    // feed 不执行 ESP-DL SIMD，只负责 I2S 搬运和 AFE feed，可以安全地把
    // 栈放入 PSRAM，从而为 Wi-Fi、AEC 和 MultiNet 保留内部 SRAM。
    s_feed_task_with_caps = true;
    BaseType_t feed_ret = xTaskCreatePinnedToCoreWithCaps(
        afe_feed_task, "afe_feed", AFE_FEED_TASK_STACK_BYTES, NULL,
        AFE_FEED_TASK_PRIORITY, NULL, AFE_FEED_TASK_CORE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (feed_ret != pdPASS) {
        printf("[初始化] afe_feed PSRAM 栈创建失败: ret=%ld psram=%u largest=%u\n",
               (long)feed_ret,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        return -1;
    }

    // ★ AEC 暖启动：前 3 秒滤波器未收敛，禁用唤醒词
    s_cooldown = 200;  // 200 个 fetch 周期 ≈ 3.2 秒

    ESP_LOGI(TAG, "AFE pipeline started, listening...");
    return 0;
}


void afe_set_playback_stop_detection(bool enabled)
{
    s_playback_stop_detected = false;
    s_playback_stop_detection_enabled = enabled && s_mn_handle && s_mn_data;
}


bool afe_consume_playback_stop_detected(void)
{
    bool detected = s_playback_stop_detected;
    s_playback_stop_detected = false;
    return detected;
}


bool afe_playback_stop_is_available(void)
{
    return s_mn_handle != NULL && s_mn_data != NULL;
}


void afe_capture_start(int max_samples)
{
    // ★ 借鉴 xiaozhi：临界区保护，防止 fetch task 正在 memcpy 时被 free
    portENTER_CRITICAL(&s_capture_lock);
    if (s_capture_buf) {
        int16_t *old = s_capture_buf;
        s_capture_buf = NULL;  // 先置 NULL，让 fetch task 跳过
        portEXIT_CRITICAL(&s_capture_lock);
        free(old);
    } else {
        portEXIT_CRITICAL(&s_capture_lock);
    }

    s_capture_max  = max_samples;
    s_capture_idx  = 0;
    s_capture_done = false;
    s_capture_seen_speech = false;
    s_capture_vad_speech = false;
    s_capture_vad_speech_samples = 0;
    s_capture_last_voice_idx = 0;
    bool barge_profile = max_samples <= CAPTURE_SAMPLE_RATE * 3;
    s_capture_no_speech_ms = barge_profile
        ? CAPTURE_NO_SPEECH_BARGE_MS : CAPTURE_NO_SPEECH_NORMAL_MS;
    s_capture_end_silence_ms = barge_profile
        ? CAPTURE_END_SILENCE_BARGE_MS : CAPTURE_END_SILENCE_NORMAL_MS;

    s_capture_buf  = heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!s_capture_buf) {
        ESP_LOGE(TAG, "capture malloc failed (%d)", max_samples);
        s_capture_max = 0;
        s_capture_seen_speech = false;
        s_capture_vad_speech = false;
        s_capture_vad_speech_samples = 0;
        s_capture_last_voice_idx = 0;
        s_capture_done = true;
    }
}

int16_t *afe_capture_finish(int *out_samples)
{
    // ★ 临界区：原子地取走 buffer 并置 NULL
    portENTER_CRITICAL(&s_capture_lock);
    if (out_samples) *out_samples = s_capture_idx;
    int16_t *buf = s_capture_buf;
    s_capture_buf  = NULL;
    portEXIT_CRITICAL(&s_capture_lock);

    s_capture_idx  = 0;
    s_capture_max  = 0;
    s_capture_seen_speech = false;
    s_capture_vad_speech = false;
    s_capture_vad_speech_samples = 0;
    s_capture_last_voice_idx = 0;
    s_capture_done = false;
    return buf;
}

int afe_capture_samples(void)
{
    return s_capture_idx;
}

int afe_capture_read_from(int sample_offset, int16_t *out, int max_samples)
{
    if (!out || max_samples <= 0 || sample_offset < 0) {
        return 0;
    }

    portENTER_CRITICAL(&s_capture_lock);
    int16_t *buf = s_capture_buf;
    int available = (buf && s_capture_idx > sample_offset)
                    ? s_capture_idx - sample_offset : 0;
    if (available > max_samples) available = max_samples;
    if (available > 0 && buf) {
        memcpy(out, buf + sample_offset, available * sizeof(int16_t));
    }
    portEXIT_CRITICAL(&s_capture_lock);

    return available;
}

bool afe_capture_is_done(void)
{
    return s_capture_done;
}

bool afe_capture_seen_speech(void)
{
    return s_capture_seen_speech;
}

bool afe_capture_had_speech(void)
{
    return s_capture_vad_speech;
}
