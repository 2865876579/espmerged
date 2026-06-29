#include "ws_client.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_websocket_client.h"
#include "esp_err.h"
#include "cJSON.h"
#include "audio_out.h"
#include "pump_driver.h"
#include "led_strip_driver.h"
#include "screen_anim.h"
#include "opus.h"
#include "mbedtls/base64.h"
#include "sensors.h"

static const char *TAG = "ws_client";

static esp_websocket_client_handle_t s_client = NULL;
static volatile bool s_connected = false;
static volatile bool s_tts_active = false;
static volatile TickType_t s_tts_guard_until_tick = 0;
static volatile bool s_turn_done = false;
static volatile bool s_dialog_end = false;
static volatile bool s_pending_dialog_end = false;
static QueueHandle_t s_audio_queue = NULL;  // 音频数据队列（WebSocket → Audio Task）
static volatile uint32_t s_tts_chunks_queued = 0;
static volatile uint32_t s_tts_chunks_dropped = 0;
static OpusDecoder *s_decoder = NULL;  // ★ 模块级，避免每次 TTS 懒初始化

// ── 气泵命令（独立 FreeRTOS 任务，不阻塞 audio/websocket）──
typedef enum { PUMP_NONE, PUMP_TILT, PUMP_RECOVER, PUMP_STOP, PUMP_HALT,
               PUMP_TILT_TO_KPA, PUMP_RECOVER_TO_KPA } pump_cmd_t;
static volatile pump_cmd_t s_pump_cmd = PUMP_NONE;
static volatile int        s_pump_dur = 0;
static volatile float      s_pump_target_kpa = 0;
static TaskHandle_t        s_pump_task = NULL;

#define PILLOW_PRESSURE_MIN_KPA 0.0f
#define PILLOW_PRESSURE_MAX_KPA 10.0f

static float clamp_pillow_pressure_kpa(float value)
{
    if (value < PILLOW_PRESSURE_MIN_KPA) return PILLOW_PRESSURE_MIN_KPA;
    if (value > PILLOW_PRESSURE_MAX_KPA) return PILLOW_PRESSURE_MAX_KPA;
    return value;
}

static int clamp_int_value(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static const char *led_effect_name(led_strip_effect_t effect)
{
    switch (effect) {
    case LED_STRIP_EFFECT_BLINK: return "blink";
    case LED_STRIP_EFFECT_BREATH: return "breath";
    case LED_STRIP_EFFECT_GRADIENT: return "gradient";
    case LED_STRIP_EFFECT_SOLID:
    default:
        return "solid";
    }
}

static led_strip_effect_t led_effect_from_name(const char *mode, led_strip_effect_t fallback)
{
    if (!mode) {
        return fallback;
    }
    if (strcmp(mode, "blink") == 0) {
        return LED_STRIP_EFFECT_BLINK;
    }
    if (strcmp(mode, "breath") == 0) {
        return LED_STRIP_EFFECT_BREATH;
    }
    if (strcmp(mode, "gradient") == 0) {
        return LED_STRIP_EFFECT_GRADIENT;
    }
    if (strcmp(mode, "solid") == 0) {
        return LED_STRIP_EFFECT_SOLID;
    }
    return fallback;
}

static void led_color_from_name(const char *color, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (!color || !r || !g || !b) {
        return;
    }
    if (strcmp(color, "white") == 0) {
        *r = 255; *g = 255; *b = 255;
    } else if (strcmp(color, "red") == 0) {
        *r = 255; *g = 0; *b = 0;
    } else if (strcmp(color, "orange") == 0) {
        *r = 255; *g = 90; *b = 0;
    } else if (strcmp(color, "yellow") == 0) {
        *r = 255; *g = 180; *b = 0;
    } else if (strcmp(color, "green") == 0) {
        *r = 0; *g = 255; *b = 80;
    } else if (strcmp(color, "cyan") == 0) {
        *r = 0; *g = 180; *b = 255;
    } else if (strcmp(color, "blue") == 0) {
        *r = 40; *g = 90; *b = 255;
    } else if (strcmp(color, "purple") == 0) {
        *r = 150; *g = 70; *b = 255;
    } else if (strcmp(color, "pink") == 0) {
        *r = 255; *g = 80; *b = 180;
    } else {
        *r = 255; *g = 208; *b = 150;
    }
}

static const char *led_color_name(uint8_t r, uint8_t g, uint8_t b)
{
    if (r == 255 && g == 255 && b == 255) return "white";
    if (r == 255 && g == 0 && b == 0) return "red";
    if (r == 255 && g == 90 && b == 0) return "orange";
    if (r == 255 && g == 180 && b == 0) return "yellow";
    if (r == 0 && g == 255 && b == 80) return "green";
    if (r == 0 && g == 180 && b == 255) return "cyan";
    if (r == 40 && g == 90 && b == 255) return "blue";
    if (r == 150 && g == 70 && b == 255) return "purple";
    if (r == 255 && g == 80 && b == 180) return "pink";
    if (r == 255 && g == 208 && b == 150) return "warm";
    return "custom";
}

// ── 上次泵闭环结果（供 read_sensors 回传）──
static volatile bool  s_last_pump_done    = false;
static volatile bool  s_last_pump_inflate = false;
static volatile float s_last_pump_target  = 0;
static volatile float s_last_pump_result  = 0;

static bool pump_consume_interrupt(void)
{
    pump_cmd_t pending = s_pump_cmd;
    if (pending == PUMP_HALT) {
        s_pump_cmd = PUMP_NONE;
        pump_stop();
        valve_close();
        return true;
    }
    if (pending == PUMP_STOP) {
        s_pump_cmd = PUMP_NONE;
        emergency_release();
        vTaskDelay(pdMS_TO_TICKS(3000));
        valve_close();
        return true;
    }
    return false;
}

static void pump_wait_interruptible(int duration_sec)
{
    int remaining_ms = duration_sec * 1000;
    while (remaining_ms > 0) {
        if (pump_consume_interrupt()) {
            break;
        }
        int step_ms = remaining_ms > 100 ? 100 : remaining_ms;
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        remaining_ms -= step_ms;
    }
}

static void pump_task(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pump_cmd_t cmd = s_pump_cmd; int dur = s_pump_dur; float target = s_pump_target_kpa;
        s_pump_cmd = PUMP_NONE;

        if (cmd == PUMP_TILT) {
            printf("[枕头] 充气 %ds\n", dur);
            pump_start(); pump_wait_interruptible(dur); pump_stop();
        } else if (cmd == PUMP_RECOVER) {
            printf("[枕头] 泄气 %ds\n", dur);
            valve_open(); pump_wait_interruptible(dur); valve_close();
        } else if (cmd == PUMP_HALT) {
            printf("[pillow] halt\n");
            pump_stop(); valve_close();
        } else if (cmd == PUMP_STOP) {
            printf("[枕头] 急停\n");
            emergency_release(); vTaskDelay(pdMS_TO_TICKS(3000)); valve_close();
        } else if (cmd == PUMP_TILT_TO_KPA || cmd == PUMP_RECOVER_TO_KPA) {
            // ★ 闭环+PWM比例调速：粗充(100%) → 均压 → 细调(50%/25%)
            float curr = sensor_read_pressure_kpa();
            printf("[枕头] 闭环目标 %.2f kPa, 当前 %.2f kPa\n", target, curr);

            if (curr < 0) { printf("[枕头] 传感器故障\n"); continue; }

            bool need_inflate  = (curr < target - 0.05f);
            bool need_deflate  = (curr > target + 0.05f);

            if (!need_inflate && !need_deflate) {
                printf("[枕头] 已在目标 (±50Pa)，跳过\n");
                s_last_pump_target = target; s_last_pump_result = curr;
                s_last_pump_done = true; s_last_pump_inflate = need_inflate;
                continue;
            }

            /* 动态读数虚高：用 target+1.0kPa 做刹车点，抵消气流误差 */
            int retries = 15;
            while (retries-- > 0) {
                float stop_at = clamp_pillow_pressure_kpa(need_inflate ? (target + 1.0f) : (target - 1.0f));

                if (need_inflate) {
                    pump_set_duty(100);
                    if (!pump_start()) { pump_clear_cooldown(); if(!pump_start()) break; }
                    int to = 300;
                    while (to-- > 0) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        curr = sensor_read_pressure_kpa();
                        if (curr >= stop_at || curr < 0) break;
                    }
                    pump_stop();
                } else {
                    valve_open();
                    int to = 300;
                    while (to-- > 0) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        curr = sensor_read_pressure_kpa();
                        if (curr <= stop_at || curr < 0) break;
                    }
                    valve_close();
                }

                /* 均压验证 */
                vTaskDelay(pdMS_TO_TICKS(300));
                curr = sensor_read_pressure_kpa();
                printf("[枕头] 均压 %.2f", curr);
                bool done = need_inflate ? (curr >= target) : (curr <= target);
                if (curr < 0 || done) { printf(" ✓\n"); break; }
                printf(" → 继续\n");
                pump_clear_cooldown();
            }
            printf("[枕头] 闭环完成: %.2f kPa (目标 %.2f)\n", curr, target);

            // ★ 保存结果，供 read_sensors 读取
            s_last_pump_target  = target;
            s_last_pump_result  = curr;
            s_last_pump_inflate = need_inflate;
            s_last_pump_done    = true;

            // ★ 回执
            char buf[256];
            snprintf(buf, sizeof(buf),
                "{\"type\":\"pump_result\",\"action\":\"%s\","
                "\"target_kpa\":%.2f,\"result_kpa\":%.2f}",
                need_inflate ? "tilt_to" : "recover_to", target, curr);
            ws_client_send_raw(buf);
        }
        printf("[枕头] 完成\n");
    }
}

// 借鉴 xiaozhi：用 spinlock 保护 consume 操作的原子性，避免 WebSocket 任务
// 和主循环同时 consume 标志位时丢失事件
static portMUX_TYPE s_event_spinlock = portMUX_INITIALIZER_UNLOCKED;

#define OPUS_SAMPLE_RATE    16000
#define OPUS_CHANNELS       1
#define AUDIO_QUEUE_DEPTH   128
#define AUDIO_PLAYER_STACK_BYTES 16384
#define AUDIO_QUEUE_SEND_TIMEOUT_MS 500
#define AUDIO_END_SEND_TIMEOUT_MS 5000
#define AUDIO_PLAYBACK_DRAIN_MS 80
#define TTS_WAKE_GUARD_MS 1500
#define AUDIO_SUBTITLE_MAX_BYTES 192

// 队列元素：一个 Opus 编码帧
typedef struct {
    uint8_t *data;   // heap 分配，audio task 负责 free
    size_t   len;
    char    *subtitle;
} audio_chunk_t;

static void free_audio_chunk(audio_chunk_t *chunk)
{
    if (!chunk) {
        return;
    }
    free(chunk->data);
    free(chunk->subtitle);
    chunk->data = NULL;
    chunk->subtitle = NULL;
    chunk->len = 0;
}

static void clear_audio_queue(void)
{
    if (!s_audio_queue) {
        return;
    }
    audio_chunk_t stale;
    while (xQueueReceive(s_audio_queue, &stale, 0) == pdTRUE) {
        free_audio_chunk(&stale);
    }
}

static void reset_opus_decoder(void)
{
    if (!s_decoder) {
        return;
    }
    int ret = opus_decoder_ctl(s_decoder, OPUS_RESET_STATE);
    if (ret != OPUS_OK) {
        ESP_LOGW(TAG, "Opus decoder reset failed: %d", ret);
    }
}

static void begin_tts_stream(void)
{
    clear_audio_queue();
    s_tts_active = true;
    s_tts_guard_until_tick = xTaskGetTickCount() + pdMS_TO_TICKS(TTS_WAKE_GUARD_MS);
    s_turn_done = false;
    s_pending_dialog_end = false;
    s_tts_chunks_queued = 0;
    s_tts_chunks_dropped = 0;

    // ★ 预创建 Opus 解码器，避免首帧到达时才 malloc 导致丢帧
    if (!s_decoder) {
        int err = 0;
        s_decoder = opus_decoder_create(OPUS_SAMPLE_RATE, OPUS_CHANNELS, &err);
        if (err != OPUS_OK || !s_decoder) {
            ESP_LOGE(TAG, "Opus decoder create failed: %d", err);
            s_decoder = NULL;
        }
    } else {
        reset_opus_decoder();
    }
}

static void end_tts_stream(bool dialog_end)
{
    if (dialog_end) {
        s_pending_dialog_end = true;
    }

    audio_chunk_t end = { .data = NULL, .len = 0 };
    if (xQueueSend(s_audio_queue, &end,
                   pdMS_TO_TICKS(AUDIO_END_SEND_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "TTS end marker enqueue failed");
        if (s_pending_dialog_end) {
            s_dialog_end = true;
            s_pending_dialog_end = false;
        }
        s_tts_active = false;
        s_turn_done = true;
    }
}


// ============================================================
static bool enqueue_opus_frame(const uint8_t *data, size_t len, const char *source)
{
    if (!data || len == 0 || len >= 4096) {
        s_tts_chunks_dropped++;
        return false;
    }

    uint8_t *opus_data = malloc(len);
    if (!opus_data) {
        s_tts_chunks_dropped++;
        return false;
    }
    memcpy(opus_data, data, len);

    audio_chunk_t chunk = { .data = opus_data, .len = len };
    if (xQueueSend(s_audio_queue, &chunk,
                   pdMS_TO_TICKS(AUDIO_QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
        s_tts_chunks_queued++;
        return true;
    }

    free(opus_data);
    s_tts_chunks_dropped++;
    return false;
}
//  音频播放任务 —— 独立栈，不阻塞 WebSocket
//  负责：Opus 解码 → Mono→Stereo → I2S 输出
//  借鉴 xiaozhi：TX 常开，空闲写静音填充，不产生开关跳变
// ============================================================
static bool enqueue_subtitle_marker(const char *text)
{
    if (!s_audio_queue || !text || text[0] == '\0') {
        return false;
    }

    size_t len = strlen(text);
    if (len >= AUDIO_SUBTITLE_MAX_BYTES) {
        len = AUDIO_SUBTITLE_MAX_BYTES - 1;
    }

    char *copy = malloc(len + 1);
    if (!copy) {
        return false;
    }
    memcpy(copy, text, len);
    copy[len] = '\0';

    audio_chunk_t marker = {
        .data = NULL,
        .len = 1,
        .subtitle = copy,
    };
    if (xQueueSend(s_audio_queue, &marker,
                   pdMS_TO_TICKS(AUDIO_QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
        return true;
    }

    free(copy);
    return false;
}

static void audio_player_task(void *arg)
{
    // 大数组在这个任务栈里（不影响 WebSocket 任务）
    int16_t pcm[960];           // 60ms @16kHz
    int16_t stereo[960 * 2];    // mono → stereo

    audio_chunk_t chunk;
    int played_frames = 0;
    bool tx_active = false;     // 跟踪 TX 是否已开启

    while (1) {
        // ★ 气泵命令：通知独立 pump 任务执行
        if (s_pump_cmd != PUMP_NONE && s_pump_task) {
            xTaskNotifyGive(s_pump_task);
        }

        if (xQueueReceive(s_audio_queue, &chunk, pdMS_TO_TICKS(500)) != pdTRUE) {
            // ★ xiaozhi: TX 常开，空闲时写静音填充，不产生开关跳变
            if (tx_active) {
                memset(stereo, 0, sizeof(stereo));
                audio_out_write((const uint8_t *)stereo, sizeof(stereo));
                played_frames = 0;
            }
            continue;
        }

        if (chunk.subtitle) {
            screen_anim_set_subtitle("小安", chunk.subtitle);
            free(chunk.subtitle);
            continue;
        }

        // NULL 数据 = 流结束信号
        if (chunk.data == NULL) {
            // ★ 刷静音填满整个 DMA 环，根除残留音频回绕
            audio_out_flush_silence();
            if (played_frames > 0) {
                vTaskDelay(pdMS_TO_TICKS(AUDIO_PLAYBACK_DRAIN_MS));
            }
            played_frames = 0;
            tx_active = false;
            if (s_tts_active) {
                s_tts_active = false;
                s_tts_guard_until_tick = xTaskGetTickCount() + pdMS_TO_TICKS(TTS_WAKE_GUARD_MS);
                if (s_pending_dialog_end) {
                    s_dialog_end = true;
                    s_pending_dialog_end = false;
                }
                s_turn_done = true;
            }
            continue;
        }

        // Opus → PCM（解码器由 begin_tts_stream 预创建）
        if (!s_decoder) {
            free(chunk.data);
            continue;
        }
        int samples = opus_decode(s_decoder, chunk.data, chunk.len, pcm, 960, 0);
        free(chunk.data);  // 尽早释放

        if (samples < 0) {
            ESP_LOGW(TAG, "Opus decode failed (%d), reset decoder and fill silence", samples);
            reset_opus_decoder();
            memset(pcm, 0, sizeof(pcm));
            samples = 960;
        }

        // TX 已常开，记录播放状态即可
        tx_active = true;

        // Mono → Stereo（MAX98357A 接收立体声，只取左声道也能响）
        for (int i = 0; i < samples; i++) {
            stereo[i * 2]     = pcm[i];
            stereo[i * 2 + 1] = pcm[i];
        }

        // I2S 输出 —— 这个任务可以慢慢等 DMA 空间
        audio_out_write((const uint8_t *)stereo, samples * 4);
        played_frames++;

    }
}


// ============================================================
//  WebSocket 事件回调 —— 在 websocket_task 中执行
//  只做轻量工作：JSON 解析 + base64 解码 + 入队列
//  绝不阻塞！（不调用 I2S、不做 Opus 解码）
// ============================================================
static void ws_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        s_connected = true;
        printf("[*] 已连接云端\n");
        {
            const char *hello_json =
                "{\"type\":\"hello\",\"version\":1,\"transport\":\"websocket\","
                "\"audio_params\":{\"format\":\"opus\",\"sample_rate\":16000,"
                "\"channels\":1,\"frame_duration\":60}}";
            esp_websocket_client_send_text(s_client, hello_json, strlen(hello_json),
                                           pdMS_TO_TICKS(1000));
        }
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        s_connected = false;
        s_tts_active = false;
        // 通知 audio task 重置解码器 + 停止播放
        {
            clear_audio_queue();
            audio_chunk_t end = { .data = NULL, .len = 0 };
            xQueueSend(s_audio_queue, &end, 0);
        }
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x02 && data->data_len > 0) {
            if (!s_tts_active) {
                s_tts_chunks_dropped++;
                break;
            }
            enqueue_opus_frame((const uint8_t *)data->data_ptr,
                               (size_t)data->data_len,
                               "binary");
            break;
        }

        if (data->op_code == 0x01 && data->data_len > 0) {
            cJSON *json = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (!json) break;

            cJSON *type = cJSON_GetObjectItem(json, "type");
            if (type && cJSON_IsString(type)) {

                if (strcmp(type->valuestring, "tts_audio_start") == 0) {
                    begin_tts_stream();
                }
                else if (strcmp(type->valuestring, "tts") == 0) {
                    cJSON *state = cJSON_GetObjectItem(json, "state");
                    if (state && cJSON_IsString(state)) {
                        if (strcmp(state->valuestring, "start") == 0) {
                            begin_tts_stream();
                        } else if (strcmp(state->valuestring, "stop") == 0) {
                            cJSON *dialog_end = cJSON_GetObjectItem(json, "dialog_end");
                            end_tts_stream(dialog_end && cJSON_IsTrue(dialog_end));
                        } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                            // ★ 流式显示 AI 回复文本
                            cJSON *text = cJSON_GetObjectItem(json, "text");
                            if (text && cJSON_IsString(text)) {
                                printf("[小安] %s\n", text->valuestring);
                                enqueue_subtitle_marker(text->valuestring);
                            }
                        }
                    }
                }
                else if (strcmp(type->valuestring, "tts_audio_chunk") == 0) {
                    cJSON *audio = cJSON_GetObjectItem(json, "audio");
                    if (audio && cJSON_IsString(audio)) {
                        const char *b64 = audio->valuestring;
                        size_t b64_len = strlen(b64);
                        size_t out_len = 0;
                        mbedtls_base64_decode(NULL, 0, &out_len, (const unsigned char *)b64, b64_len);

                        if (out_len > 0 && out_len < 4096) {
                            uint8_t *opus_data = malloc(out_len);
                            if (opus_data) {
                                size_t actual = 0;
                                mbedtls_base64_decode(opus_data, out_len, &actual, (const unsigned char *)b64, b64_len);

                                // 入队列（非阻塞，队满就丢，保护 websocket 任务）
                                audio_chunk_t chunk = { .data = opus_data, .len = actual };
                                if (xQueueSend(s_audio_queue, &chunk,
                                               pdMS_TO_TICKS(AUDIO_QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
                                    s_tts_chunks_queued++;
                                    opus_data = NULL;
                                } else {
                                    s_tts_chunks_dropped++;
                                }
                                free(opus_data);
                            }
                        }
                    }
                }
                else if (strcmp(type->valuestring, "tts_audio_end") == 0) {
                    cJSON *dialog_end = cJSON_GetObjectItem(json, "dialog_end");
                    end_tts_stream(dialog_end && cJSON_IsTrue(dialog_end));
                }
                else if (strcmp(type->valuestring, "status") == 0) {
                    cJSON *msg = cJSON_GetObjectItem(json, "msg");
                    if (msg && cJSON_IsString(msg)) {
                        printf("[状态] %s\n", msg->valuestring);
                    }
                    s_turn_done = true;
                }
                else if (strcmp(type->valuestring, "stt_result") == 0) {
                    cJSON *text = cJSON_GetObjectItem(json, "text");
                    if (text && cJSON_IsString(text)) {
                        printf("[你] %s\n", text->valuestring);
                        screen_anim_set_subtitle("你", text->valuestring);
                    }
                }
                else if (strcmp(type->valuestring, "led_cmd") == 0) {
                    const char *action = cJSON_GetStringValue(cJSON_GetObjectItem(json, "action"));
                    bool enabled = true;
                    uint8_t brightness = 0;
                    led_strip_effect_t effect = LED_STRIP_EFFECT_SOLID;
                    uint8_t speed_pct = 30;
                    uint8_t r = 255;
                    uint8_t g = 208;
                    uint8_t b = 150;
                    bool has_brightness = false;
                    led_strip_get_effect_state(&enabled, &brightness, &effect,
                                               &speed_pct, &r, &g, &b);

                    if (action) {
                        if (strcmp(action, "on") == 0) {
                            enabled = true;
                        } else if (strcmp(action, "off") == 0) {
                            enabled = false;
                            brightness = 0;
                            effect = LED_STRIP_EFFECT_SOLID;
                        } else if (strcmp(action, "toggle") == 0) {
                            enabled = !enabled;
                        }
                    }

                    cJSON *enabled_item = cJSON_GetObjectItem(json, "enabled");
                    if (!enabled_item) {
                        enabled_item = cJSON_GetObjectItem(json, "on");
                    }
                    if (cJSON_IsBool(enabled_item)) {
                        enabled = cJSON_IsTrue(enabled_item);
                    }

                    const char *mode = cJSON_GetStringValue(cJSON_GetObjectItem(json, "mode"));
                    effect = led_effect_from_name(mode, effect);

                    const char *color = cJSON_GetStringValue(cJSON_GetObjectItem(json, "color"));
                    led_color_from_name(color, &r, &g, &b);

                    cJSON *r_item = cJSON_GetObjectItem(json, "r");
                    cJSON *g_item = cJSON_GetObjectItem(json, "g");
                    cJSON *b_item = cJSON_GetObjectItem(json, "b");
                    if (cJSON_IsNumber(r_item)) {
                        r = (uint8_t)clamp_int_value(r_item->valueint, 0, 255);
                    }
                    if (cJSON_IsNumber(g_item)) {
                        g = (uint8_t)clamp_int_value(g_item->valueint, 0, 255);
                    }
                    if (cJSON_IsNumber(b_item)) {
                        b = (uint8_t)clamp_int_value(b_item->valueint, 0, 255);
                    }

                    cJSON *pct_item = cJSON_GetObjectItem(json, "brightness_pct");
                    if (cJSON_IsNumber(pct_item)) {
                        has_brightness = true;
                        int pct = clamp_int_value(pct_item->valueint, 0, 100);
                        brightness = (uint8_t)((pct * 255 + 50) / 100);
                    }

                    cJSON *brightness_item = cJSON_GetObjectItem(json, "brightness");
                    if (cJSON_IsNumber(brightness_item)) {
                        has_brightness = true;
                        int raw = clamp_int_value(brightness_item->valueint, 0, 255);
                        brightness = (uint8_t)raw;
                    }

                    cJSON *speed_item = cJSON_GetObjectItem(json, "speed_pct");
                    if (cJSON_IsNumber(speed_item)) {
                        speed_pct = (uint8_t)clamp_int_value(speed_item->valueint, 0, 100);
                    }

                    uint32_t duration_ms = 0;
                    cJSON *duration_item = cJSON_GetObjectItem(json, "duration_sec");
                    if (cJSON_IsNumber(duration_item)) {
                        int duration_sec = clamp_int_value(duration_item->valueint, 0, 600);
                        duration_ms = (uint32_t)duration_sec * 1000UL;
                    }

                    if (action && strcmp(action, "off") == 0) {
                        enabled = false;
                        brightness = 0;
                    } else if (enabled && brightness == 0 && !has_brightness) {
                        brightness = 56;
                    } else if (action && strcmp(action, "on") == 0 && brightness == 0) {
                        brightness = 56;
                    } else if (brightness == 0) {
                        enabled = false;
                    } else if (action && strcmp(action, "set") == 0) {
                        enabled = true;
                    }

                    led_strip_config_t config = {
                        .enabled = enabled && brightness > 0,
                        .brightness = brightness,
                        .effect = effect,
                        .speed_pct = speed_pct,
                        .duration_ms = duration_ms,
                        .r = r,
                        .g = g,
                        .b = b,
                    };

                    esp_err_t led_ret = led_strip_apply_effect(&config);
                    printf("[led] cmd action=%s enabled=%d brightness=%u mode=%s color=%s speed=%u duration=%lu ret=%d\n",
                           action ? action : "",
                           config.enabled ? 1 : 0,
                           config.brightness,
                           led_effect_name(config.effect),
                           led_color_name(config.r, config.g, config.b),
                           config.speed_pct,
                           (unsigned long)config.duration_ms,
                           led_ret);

                    bool current_enabled = false;
                    uint8_t current_brightness = 0;
                    led_strip_effect_t current_effect = LED_STRIP_EFFECT_SOLID;
                    uint8_t current_speed = 0;
                    uint8_t current_r = 0;
                    uint8_t current_g = 0;
                    uint8_t current_b = 0;
                    led_strip_get_effect_state(&current_enabled, &current_brightness,
                                               &current_effect, &current_speed,
                                               &current_r, &current_g, &current_b);
                    char led_state[256];
                    snprintf(led_state, sizeof(led_state),
                             "{\"type\":\"led_state\",\"enabled\":%s,\"brightness\":%u,\"brightness_pct\":%u,"
                             "\"mode\":\"%s\",\"speed_pct\":%u,\"color\":\"%s\",\"r\":%u,\"g\":%u,\"b\":%u}",
                             current_enabled ? "true" : "false",
                             current_brightness,
                             (unsigned)((current_brightness * 100 + 127) / 255),
                             led_effect_name(current_effect),
                             current_speed,
                             led_color_name(current_r, current_g, current_b),
                             current_r, current_g, current_b);
                    ws_client_send_raw(led_state);
                }
                else if (strcmp(type->valuestring, "pillow_cmd") == 0) {
                    const char *action = cJSON_GetStringValue(cJSON_GetObjectItem(json, "action"));
                    cJSON *target_item = cJSON_GetObjectItem(json, "target_kpa");
                    bool has_target_kpa = cJSON_IsNumber(target_item);
                    float target_kpa = has_target_kpa ? clamp_pillow_pressure_kpa((float)cJSON_GetNumberValue(target_item)) : 0.0f;

                    if (has_target_kpa) {
                        // ★ 闭环模式：有目标气压，边充/放边读传感器，到位即停
                        s_pump_target_kpa = target_kpa;
                        if (strcmp(action, "tilt") == 0) {
                            s_pump_cmd = PUMP_TILT_TO_KPA; s_pump_dur = 0;
                        } else if (strcmp(action, "recover") == 0) {
                            s_pump_cmd = PUMP_RECOVER_TO_KPA; s_pump_dur = 0;
                        }
                        printf("[枕头] LLM闭环: %s to %.2f kPa\n", action, target_kpa);
                    } else {
                        // ★ 开环模式：纯时间控制（兼容旧协议）
                        int dur = (int)cJSON_GetNumberValue(cJSON_GetObjectItem(json, "duration_sec"));
                        if (dur < 1) dur = 3;
                        if (dur > 600) dur = 600;

                        if (strcmp(action, "tilt") == 0) {
                            s_pump_cmd = PUMP_TILT; s_pump_dur = dur;
                        } else if (strcmp(action, "recover") == 0) {
                            s_pump_cmd = PUMP_RECOVER; s_pump_dur = dur;
                        } else if (strcmp(action, "stop") == 0) {
                            s_pump_cmd = PUMP_STOP; s_pump_dur = 0;
                        } else if (strcmp(action, "halt") == 0) {
                            s_pump_cmd = PUMP_HALT; s_pump_dur = 0;
                        }
                        printf("[枕头] LLM指令: %s %ds (排队中)\n", action, dur);
                    }
                }
                else if (strcmp(type->valuestring, "read_sensors") == 0) {
                    // ★ LLM 请求传感器数据：即时刷新 → 读最新数据
                    sensor_data_t sd;
                    sensor_request_refresh();
                    sensor_get_latest(&sd);

                    cJSON *resp = cJSON_CreateObject();
                    cJSON_AddStringToObject(resp, "type", "sensor_data");
                    // ★ 回传 request_id，服务器靠它匹配 Future
                    cJSON *req_id = cJSON_GetObjectItem(json, "request_id");
                    if (req_id && cJSON_IsString(req_id)) {
                        cJSON_AddStringToObject(resp, "request_id", req_id->valuestring);
                    }
                    cJSON *data_obj = cJSON_CreateObject();
                    cJSON_AddNumberToObject(data_obj, "mq135_ppm", sd.mq135_ppm);
                    cJSON_AddBoolToObject(data_obj, "mq135_valid", sd.mq135_valid);
                    cJSON_AddNumberToObject(data_obj, "pressure_kpa", sd.pressure_kpa);
                    cJSON_AddBoolToObject(data_obj, "pressure_valid", sd.pressure_valid);
                    cJSON *fsr_arr = cJSON_CreateArray();
                    for (int i = 0; i < 4; i++) {
                        cJSON *fsr = cJSON_CreateObject();
                        cJSON_AddNumberToObject(fsr, "n", sd.fsr_force_n[i]);
                        cJSON_AddBoolToObject(fsr, "valid", sd.fsr_valid[i]);
                        cJSON_AddItemToArray(fsr_arr, fsr);
                    }
                    cJSON_AddItemToObject(data_obj, "fsr", fsr_arr);
                    cJSON_AddNumberToObject(data_obj, "temperature_c", sd.temperature_c);
                    cJSON_AddNumberToObject(data_obj, "humidity_pct", sd.humidity_pct);
                    cJSON_AddBoolToObject(data_obj, "env_valid", sd.env_valid);
                    cJSON_AddNumberToObject(data_obj, "light_lux", sd.light_lux);
                    cJSON_AddBoolToObject(data_obj, "light_valid", sd.light_valid);
                    bool led_enabled = false;
                    uint8_t led_brightness = 0;
                    led_strip_get_state(&led_enabled, &led_brightness);
                    cJSON_AddBoolToObject(data_obj, "led_enabled", led_enabled);
                    cJSON_AddNumberToObject(data_obj, "led_brightness", led_brightness);
                    cJSON_AddNumberToObject(data_obj, "led_brightness_pct",
                                            (led_brightness * 100 + 127) / 255);
                    /* ★ 上次泵闭环结果 */
                    if (s_last_pump_done) {
                        cJSON *last = cJSON_CreateObject();
                        cJSON_AddStringToObject(last, "action",
                            s_last_pump_inflate ? "tilt_to" : "recover_to");
                        cJSON_AddNumberToObject(last, "target_kpa", s_last_pump_target);
                        cJSON_AddNumberToObject(last, "result_kpa", s_last_pump_result);
                        cJSON_AddItemToObject(data_obj, "last_pump", last);
                    }
                    cJSON_AddItemToObject(resp, "data", data_obj);

                    char *json_str = cJSON_PrintUnformatted(resp);
                    ws_client_send_raw(json_str);
                    free(json_str);
                    cJSON_Delete(resp);
                    printf("[传感器] 数据已回传: mq135=%.1fppm kPa=%.2f T=%.1fC H=%.1f%% lux=%.1f\n",
                           sd.mq135_ppm, sd.pressure_kpa, sd.temperature_c, sd.humidity_pct, sd.light_lux);
                }
                else if (strcmp(type->valuestring, "dialog_end") == 0) {
                    end_tts_stream(true);
                }
                else if (strcmp(type->valuestring, "pong") == 0) {
                    // 心跳，忽略
                }
            }
            cJSON_Delete(json);
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        break;

    default:
        break;
    }
}


// ============================================================
//  公开 API
// ============================================================

void ws_client_start(const char *uri)
{
    // Audio queue depth is capped to keep small queue storage out of internal RAM pressure.
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(audio_chunk_t));
    if (!s_audio_queue) {
        ESP_LOGE(TAG, "audio queue create failed");
        return;
    }

    // 音频播放任务：prio=9 高于 feed(8)，确保 DMA 不断流
    BaseType_t audio_ret = xTaskCreatePinnedToCoreWithCaps(
        audio_player_task, "audio_player", AUDIO_PLAYER_STACK_BYTES,
        NULL, 9, NULL, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (audio_ret != pdPASS) {
        ESP_LOGW(TAG, "audio_player PSRAM stack create failed, fallback internal");
        audio_ret = xTaskCreatePinnedToCore(
            audio_player_task, "audio_player", AUDIO_PLAYER_STACK_BYTES,
            NULL, 9, NULL, 1);
    }
    if (audio_ret != pdPASS) {
        ESP_LOGE(TAG, "audio_player task create failed");
        return;
    }

    // 气泵任务：独立栈 4KB，不阻塞音频和 websocket
    xTaskCreate(pump_task, "pump", 4096, NULL, 2, &s_pump_task);

    // WebSocket 客户端：16KB 栈（库内部帧解析也需要栈空间！）
    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .buffer_size = 8192,
        .reconnect_timeout_ms = 10000,
        .network_timeout_ms = 15000,
        .ping_interval_sec = 2,           // 每 3 秒发 ping 防止云 SLB 杀空闲连接
        .pingpong_timeout_sec = 0,          // 应用层 keepalive 已覆盖，协议层不管
        .disable_auto_reconnect = false,
        .task_stack = 12288,
    };

    s_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    esp_websocket_client_start(s_client);
}


void ws_client_restart(void)
{
    if (!s_client) return;

    s_connected = false;
    s_tts_active = false;
    s_tts_guard_until_tick = xTaskGetTickCount() + pdMS_TO_TICKS(TTS_WAKE_GUARD_MS);
    s_turn_done = false;
    s_dialog_end = false;
    s_pending_dialog_end = false;

    if (s_audio_queue) {
        clear_audio_queue();
        audio_chunk_t end = { .data = NULL, .len = 0 };
        xQueueSend(s_audio_queue, &end, 0);
    }

    esp_websocket_client_stop(s_client);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_websocket_client_start(s_client);
}

bool ws_client_send_text(const char *text)
{
    if (!s_client || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot send");
        return false;
    }

    cJSON *msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "type", "text");
    cJSON_AddStringToObject(msg, "text", text);
    char *json_str = cJSON_PrintUnformatted(msg);
    int json_len = strlen(json_str);

    int sent = esp_websocket_client_send_text(s_client, json_str, json_len, portMAX_DELAY);
    free(json_str);
    cJSON_Delete(msg);
    return sent >= 0;
}

bool ws_client_send_raw(const char *json_str)
{
    if (!s_client || !esp_websocket_client_is_connected(s_client)) {
        return false;
    }
    int json_len = strlen(json_str);
    int sent = esp_websocket_client_send_text(s_client, json_str, json_len, portMAX_DELAY);
    return sent >= 0;
}

bool ws_client_send_binary(const uint8_t *data, int len)
{
    if (!s_client || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot send binary");
        return false;
    }
    if (!data || len <= 0) {
        return false;
    }

    int sent = esp_websocket_client_send_bin(s_client, (const char *)data, len, portMAX_DELAY);
    return sent >= 0;
}

bool ws_client_is_connected(void)
{
    return s_connected && s_client && esp_websocket_client_is_connected(s_client);
}

bool ws_client_is_tts_active(void)
{
    return s_tts_active;
}

bool ws_client_is_tts_guard_active(void)
{
    if (s_tts_active) {
        return true;
    }
    TickType_t until = s_tts_guard_until_tick;
    if (until == 0) {
        return false;
    }
    return (int32_t)(until - xTaskGetTickCount()) > 0;
}

void ws_client_clear_events(void)
{
    // ★ 借鉴 xiaozhi：临界区保护，防止和 ws_event_handler 同时改标志位
    portENTER_CRITICAL(&s_event_spinlock);
    s_turn_done = false;
    s_dialog_end = false;
    s_pending_dialog_end = false;
    portEXIT_CRITICAL(&s_event_spinlock);
}

bool ws_client_consume_turn_done(void)
{
    // ★ 临界区保护原子 consume
    portENTER_CRITICAL(&s_event_spinlock);
    bool value = s_turn_done;
    s_turn_done = false;
    portEXIT_CRITICAL(&s_event_spinlock);
    return value;
}

bool ws_client_consume_dialog_end(void)
{
    // ★ 临界区保护原子 consume
    portENTER_CRITICAL(&s_event_spinlock);
    bool value = s_dialog_end;
    s_dialog_end = false;
    portEXIT_CRITICAL(&s_event_spinlock);
    return value;
}
