#include "ws_client.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "esp_err.h"
#include "cJSON.h"
#include "audio_out.h"
#include "pump_driver.h"
#include "led_strip_driver.h"
#include "screen_anim.h"
#include "avatar_storage.h"
#include "opus.h"
#include "mbedtls/base64.h"
#include "sensors.h"
#include "snore_detector.h"

static const char *TAG = "ws_client";

static esp_websocket_client_handle_t s_client = NULL;
static volatile bool s_connected = false;
static volatile TickType_t s_last_disconnected_tick = 0;
static volatile uint32_t s_disconnect_count = 0;
static volatile bool s_tts_active = false;
static volatile bool s_music_active = false;
static volatile bool s_music_barge_result_ready = false;
static volatile bool s_music_barge_stop = false;
static volatile TickType_t s_tts_guard_until_tick = 0;
static volatile bool s_turn_done = false;
static volatile bool s_dialog_end = false;
static volatile bool s_pending_dialog_end = false;
static volatile bool s_tts_playback_done = false;
static volatile bool s_interaction_listen_pending = false;
static volatile uint32_t s_wake_ack_id = 0;
static char s_device_id[32] = "esp32s3-unknown";
static QueueHandle_t s_audio_queue = NULL;  // 音频数据队列（WebSocket → Audio Task）
static volatile uint32_t s_tts_chunks_queued = 0;
static volatile uint32_t s_tts_chunks_dropped = 0;
static OpusDecoder *s_decoder = NULL;  // ★ 模块级，避免每次 TTS 懒初始化
static SemaphoreHandle_t s_decoder_mutex = NULL;
static SemaphoreHandle_t s_send_mutex = NULL;
static QueueHandle_t s_deferred_send_queue = NULL;
static TaskHandle_t s_deferred_send_task = NULL;
static portMUX_TYPE s_event_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define WS_FRAGMENT_MAX_BYTES 16384
#define AUDIO_OUTPUT_GAIN 0.44f
static uint8_t *s_ws_fragment_buf = NULL;
static size_t s_ws_fragment_len = 0;
static size_t s_ws_fragment_total = 0;
static uint8_t s_ws_fragment_opcode = 0;
static void clear_ws_fragment(void);
static bool append_ws_fragment(const esp_websocket_event_data_t *data);

static void deferred_send_task(void *arg)
{
    (void)arg;
    char *json = NULL;
    while (xQueueReceive(s_deferred_send_queue, &json, portMAX_DELAY) == pdTRUE) {
        if (json) {
            ws_client_send_raw(json);
            free(json);
        }
        json = NULL;
    }
    s_deferred_send_task = NULL;
    vTaskDelete(NULL);
}

static bool defer_text_send(const char *json)
{
    if (!json || !s_deferred_send_queue) {
        return false;
    }
    char *copy = strdup(json);
    if (!copy) {
        return false;
    }
    if (xQueueSend(s_deferred_send_queue, &copy, 0) != pdTRUE) {
        free(copy);
        return false;
    }
    return true;
}

typedef struct {
    char url[384];
    size_t size;
    uint32_t crc32;
} avatar_download_args_t;

static void avatar_download_task(void *arg)
{
    avatar_download_args_t *args = (avatar_download_args_t *)arg;
    if (!args) {
        vTaskDelete(NULL);
        return;
    }

    screen_anim_set_subtitle("形象", "正在同步新形象");
    esp_err_t ret = avatar_storage_download_rgb666(args->url, args->size, args->crc32);
    char msg[192];
    snprintf(msg, sizeof(msg),
             "{\"type\":\"avatar_state\",\"ok\":%s,\"ret\":%d}",
             ret == ESP_OK ? "true" : "false", ret);
    ws_client_send_raw(msg);
    screen_anim_set_subtitle("形象", ret == ESP_OK ? "新形象已同步" : "形象同步失败");
    free(args);
    vTaskDelete(NULL);
}

static void start_avatar_download(const char *url, size_t size, uint32_t crc32)
{
    if (!url || !*url || size == 0) {
        ws_client_send_raw("{\"type\":\"avatar_state\",\"ok\":false,\"ret\":-1}");
        return;
    }
    avatar_download_args_t *args = calloc(1, sizeof(*args));
    if (!args) {
        ws_client_send_raw("{\"type\":\"avatar_state\",\"ok\":false,\"ret\":-2}");
        return;
    }
    snprintf(args->url, sizeof(args->url), "%s", url);
    args->size = size;
    args->crc32 = crc32;
    if (xTaskCreatePinnedToCore(avatar_download_task,
                                "avatar_download",
                                6144,
                                args,
                                3,
                                NULL,
                                0) != pdPASS) {
        free(args);
        ws_client_send_raw("{\"type\":\"avatar_state\",\"ok\":false,\"ret\":-3}");
    }
}

// ── 气泵命令（独立 FreeRTOS 任务，不阻塞 audio/websocket）──
typedef enum { PUMP_NONE, PUMP_TILT, PUMP_RECOVER, PUMP_RELEASE, PUMP_HALT,
               PUMP_TILT_TO_KPA, PUMP_RECOVER_TO_KPA,
               PUMP_TILT_CONTINUOUS, PUMP_RECOVER_CONTINUOUS } pump_cmd_t;
typedef struct {
    pump_cmd_t cmd;
    int duration_sec;
    float target_kpa;
} pump_request_t;
static QueueHandle_t s_pump_queue = NULL;
static TaskHandle_t        s_pump_task = NULL;

#define PILLOW_PRESSURE_MIN_KPA 0.0f
#define PILLOW_PRESSURE_MAX_KPA 10.0f
#define PILLOW_PRESSURE_TOLERANCE_KPA 0.10f
#define PILLOW_ADJUST_TIMEOUT_MS 120000
#define PILLOW_RELEASE_TIMEOUT_MS 8000
#define PILLOW_RELEASE_STABLE_SAMPLES 10
#define PILLOW_MANUAL_MAX_SECONDS 2
#define PILLOW_MANUAL_MAX_RUN_MS 2000
#define PILLOW_MIN_PROGRESS_KPA 0.03f

static float clamp_pillow_pressure_kpa(float value)
{
    if (value < PILLOW_PRESSURE_MIN_KPA) return PILLOW_PRESSURE_MIN_KPA;
    if (value > PILLOW_PRESSURE_MAX_KPA) return PILLOW_PRESSURE_MAX_KPA;
    return value;
}

static bool submit_pump_request(pump_cmd_t cmd, int duration_sec, float target_kpa)
{
    if (!s_pump_queue) {
        return false;
    }
    pump_request_t request = {
        .cmd = cmd,
        .duration_sec = duration_sec,
        .target_kpa = clamp_pillow_pressure_kpa(target_kpa),
    };
    if (xQueueOverwrite(s_pump_queue, &request) != pdPASS) {
        return false;
    }
    if (s_pump_task) {
        xTaskNotifyGive(s_pump_task);
    }
    return true;
}

static bool pump_request_pending(void)
{
    return s_pump_queue && uxQueueMessagesWaiting(s_pump_queue) > 0;
}

bool ws_client_request_pillow_tilt_to_kpa(float target_kpa, const char *source)
{
    (void)source;
    target_kpa = clamp_pillow_pressure_kpa(target_kpa);
    return submit_pump_request(PUMP_TILT_TO_KPA, 0, target_kpa);
}

bool ws_client_request_pillow_recover_to_kpa(float target_kpa, const char *source)
{
    (void)source;
    target_kpa = clamp_pillow_pressure_kpa(target_kpa);
    return submit_pump_request(PUMP_RECOVER_TO_KPA, 0, target_kpa);
}

bool ws_client_request_pillow_command(const char *action, bool continuous)
{
    if (!action) {
        return false;
    }
    if (strcmp(action, "tilt") == 0) {
        return submit_pump_request(continuous ? PUMP_TILT_CONTINUOUS : PUMP_TILT,
                                   continuous ? 0 : 3, 0.0f);
    }
    if (strcmp(action, "recover") == 0) {
        return submit_pump_request(continuous ? PUMP_RECOVER_CONTINUOUS : PUMP_RECOVER,
                                   continuous ? 0 : 3, 0.0f);
    }
    if (strcmp(action, "stop") == 0 || strcmp(action, "halt") == 0) {
        return submit_pump_request(PUMP_HALT, 0, 0.0f);
    }
    if (strcmp(action, "release") == 0) {
        return submit_pump_request(PUMP_RELEASE, 0, 0.0f);
    }
    return false;
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
    if (!s_pump_queue) {
        return false;
    }
    pump_request_t request;
    if (xQueuePeek(s_pump_queue, &request, 0) != pdTRUE) {
        return false;
    }
    pump_cmd_t pending = request.cmd;
    if (pending == PUMP_HALT) {
        xQueueReceive(s_pump_queue, &request, 0);
        pump_stop();
        valve_close();
        return true;
    }
    if (pending == PUMP_RELEASE) {
        xQueueReceive(s_pump_queue, &request, 0);
        emergency_release();
        vTaskDelay(pdMS_TO_TICKS(3000));
        valve_close();
        return true;
    }
    /* Leave replacement requests queued so the task can process them next. */
    return true;
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
        pump_request_t request;
        if (!s_pump_queue || xQueueReceive(s_pump_queue, &request, 0) != pdTRUE) {
            continue;
        }
        pump_cmd_t cmd = request.cmd;
        int dur = request.duration_sec;
        float target = request.target_kpa;
        ESP_LOGI(TAG, "pump request cmd=%d duration=%d target=%.2f",
                 (int)cmd, dur, (double)target);

        if (cmd == PUMP_TILT) {
            pump_start(); pump_wait_interruptible(dur); pump_stop();
        } else if (cmd == PUMP_RECOVER) {
            valve_open(); pump_wait_interruptible(dur); valve_close();
        } else if (cmd == PUMP_TILT_CONTINUOUS) {
            if (pump_start()) {
                const int64_t deadline_us = esp_timer_get_time() +
                    (int64_t)PILLOW_MANUAL_MAX_RUN_MS * 1000LL;
                while (esp_timer_get_time() < deadline_us &&
                       !pump_consume_interrupt()) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                pump_stop();
            }
        } else if (cmd == PUMP_RECOVER_CONTINUOUS) {
            if (valve_open()) {
                while (!pump_consume_interrupt()) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                valve_close();
            }
        } else if (cmd == PUMP_HALT) {
            pump_stop(); valve_close();
        } else if (cmd == PUMP_RELEASE) {
            emergency_release(); vTaskDelay(pdMS_TO_TICKS(3000)); valve_close();
        } else if (cmd == PUMP_TILT_TO_KPA || cmd == PUMP_RECOVER_TO_KPA) {
            float curr = sensor_read_pressure_kpa();
            printf("[pressure] current=%.2f kPa target=%.2f kPa\n", curr, target);
            if (curr < 0) {
                printf("[pressure] sensor invalid\n");
                continue;
            }

            const bool need_inflate = curr < target - PILLOW_PRESSURE_TOLERANCE_KPA;
            const bool need_deflate = curr > target + PILLOW_PRESSURE_TOLERANCE_KPA;
            bool interrupted = false;
            bool stalled = false;
            bool reached = !need_inflate && !need_deflate;
            const int64_t deadline_us = esp_timer_get_time() +
                (int64_t)PILLOW_ADJUST_TIMEOUT_MS * 1000LL;

            if (need_inflate) {
                while (!reached && esp_timer_get_time() < deadline_us) {
                    if (pump_consume_interrupt()) {
                        interrupted = true;
                        break;
                    }

                    while (!pump_start() && esp_timer_get_time() < deadline_us) {
                        if (pump_consume_interrupt()) {
                            interrupted = true;
                            break;
                        }
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    if (interrupted || !pump_is_running()) break;

                    const float segment_start = curr;
                    const float gap = target - curr;
                    pump_set_duty(gap > 0.6f ? 100 : (gap > 0.25f ? 60 : 35));
                    while (pump_run_ms() < PUMP_MAX_RUN_MS) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        if (pump_consume_interrupt()) {
                            interrupted = true;
                            break;
                        }
                        curr = sensor_read_pressure_kpa();
                        if (curr < 0 || curr >= target - 0.03f) break;
                    }
                    pump_stop();
                    if (interrupted || curr < 0) break;

                    for (int i = 0; i < 5; i++) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        if (pump_consume_interrupt()) {
                            interrupted = true;
                            break;
                        }
                    }
                    if (interrupted) break;
                    curr = sensor_read_pressure_kpa();
                    printf("[pressure] settled=%.2f kPa target=%.2f kPa\n", curr, target);
                    if (curr < 0) break;
                    reached = curr >= target - PILLOW_PRESSURE_TOLERANCE_KPA;
                    if (!reached && curr < segment_start + PILLOW_MIN_PROGRESS_KPA) {
                        stalled = true;
                        ESP_LOGE(TAG,
                                 "pressure did not rise: start=%.2f settled=%.2f; stop pump",
                                 (double)segment_start, (double)curr);
                        break;
                    }
                    if (!reached) {
                        for (int i = 0; i < PUMP_COOLDOWN_MS / 100; i++) {
                            vTaskDelay(pdMS_TO_TICKS(100));
                            if (pump_consume_interrupt()) {
                                interrupted = true;
                                break;
                            }
                        }
                    }
                }
                pump_stop();
            } else if (need_deflate) {
                const bool release_fully = target <= 0.05f;
                const int64_t release_deadline_us = esp_timer_get_time() +
                    (int64_t)(release_fully ? PILLOW_RELEASE_TIMEOUT_MS
                                           : PILLOW_ADJUST_TIMEOUT_MS) * 1000LL;
                float previous = curr;
                int stable_samples = 0;
                valve_open();
                while (esp_timer_get_time() < release_deadline_us) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    if (pump_consume_interrupt()) {
                        interrupted = true;
                        break;
                    }
                    curr = sensor_read_pressure_kpa();
                    if (curr < 0) break;
                    if (!release_fully &&
                        curr <= target + PILLOW_PRESSURE_TOLERANCE_KPA) {
                        reached = true;
                        break;
                    }
                    if (fabsf(curr - previous) < 0.01f) {
                        stable_samples++;
                    } else {
                        stable_samples = 0;
                    }
                    previous = curr;
                    if (release_fully &&
                        stable_samples >= PILLOW_RELEASE_STABLE_SAMPLES) {
                        reached = true;
                        break;
                    }
                }
                valve_close();
            }

            pump_stop();
            valve_close();
            if (interrupted) continue;

            s_last_pump_target = target;
            s_last_pump_result = curr;
            s_last_pump_inflate = need_inflate;
            s_last_pump_done = true;

            char buf[256];
            snprintf(buf, sizeof(buf),
                "{\"type\":\"pump_result\",\"action\":\"%s\","
                "\"target_kpa\":%.2f,\"result_kpa\":%.2f,"
                "\"reached\":%s,\"stalled\":%s}",
                need_inflate ? "tilt_to" : "recover_to",
                target, curr, reached ? "true" : "false",
                stalled ? "true" : "false");
            ws_client_send_raw(buf);
        }
    }
}

static void avatar_restore_default_task(void *arg)
{
    (void)arg;
    screen_anim_set_subtitle("形象", "正在恢复默认形象");
    esp_err_t ret = avatar_storage_clear_custom();
    char msg[224];
    snprintf(msg, sizeof(msg),
             "{\"type\":\"avatar_state\",\"ok\":%s,\"default\":true,\"ret\":%d}",
             ret == ESP_OK ? "true" : "false", ret);
    ws_client_send_raw(msg);
    screen_anim_set_subtitle("形象", ret == ESP_OK ? "默认形象已恢复" : "默认形象恢复失败");
    vTaskDelete(NULL);
}

static void start_avatar_restore_default(void)
{
    if (xTaskCreatePinnedToCore(avatar_restore_default_task,
                                "avatar_restore",
                                3072,
                                NULL,
                                3,
                                NULL,
                                0) != pdPASS) {
        ws_client_send_raw("{\"type\":\"avatar_state\",\"ok\":false,\"default\":true,\"ret\":-2}");
    }
}

// 借鉴 xiaozhi：用 spinlock 保护 consume 操作的原子性，避免 WebSocket 任务
// 和主循环同时 consume 标志位时丢失事件
#define OPUS_SAMPLE_RATE    16000
#define OPUS_CHANNELS       1
#define AUDIO_QUEUE_DEPTH   128
#define AUDIO_PLAYER_STACK_BYTES 32768
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

static void reset_opus_decoder_locked(void)
{
    if (!s_decoder) {
        return;
    }
    int ret = opus_decoder_ctl(s_decoder, OPUS_RESET_STATE);
    if (ret != OPUS_OK) {
        ESP_LOGW(TAG, "Opus decoder reset failed: %d", ret);
    }
}

static bool lock_decoder(TickType_t timeout)
{
    return !s_decoder_mutex || xSemaphoreTake(s_decoder_mutex, timeout) == pdTRUE;
}

static void unlock_decoder(void)
{
    if (s_decoder_mutex) {
        xSemaphoreGive(s_decoder_mutex);
    }
}

static void begin_tts_stream(bool music)
{
    clear_audio_queue();
    portENTER_CRITICAL(&s_event_spinlock);
    s_tts_active = true;
    s_music_active = music;
    s_music_barge_result_ready = false;
    s_tts_guard_until_tick = xTaskGetTickCount() + pdMS_TO_TICKS(TTS_WAKE_GUARD_MS);
    s_turn_done = false;
    s_pending_dialog_end = false;
    s_tts_playback_done = false;
    portEXIT_CRITICAL(&s_event_spinlock);
    s_tts_chunks_queued = 0;
    s_tts_chunks_dropped = 0;

    // ★ 预创建 Opus 解码器，避免首帧到达时才 malloc 导致丢帧
    if (!lock_decoder(pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG, "Opus decoder busy; dropping TTS start");
        return;
    }
    if (!s_decoder) {
        int err = 0;
        s_decoder = opus_decoder_create(OPUS_SAMPLE_RATE, OPUS_CHANNELS, &err);
        if (err != OPUS_OK || !s_decoder) {
            ESP_LOGE(TAG, "Opus decoder create failed: %d", err);
            s_decoder = NULL;
        }
    } else {
        reset_opus_decoder_locked();
    }
    unlock_decoder();
}

static void stop_music_playback_now(void)
{
    clear_audio_queue();
    portENTER_CRITICAL(&s_event_spinlock);
    s_tts_active = false;
    s_music_active = false;
    s_tts_guard_until_tick = 0;
    s_pending_dialog_end = false;
    s_turn_done = true;
    s_tts_playback_done = true;
    portEXIT_CRITICAL(&s_event_spinlock);

    audio_chunk_t end = { .data = NULL, .len = 0 };
    xQueueSend(s_audio_queue, &end, 0);
}

static void end_tts_stream(bool dialog_end)
{
    if (dialog_end) {
        portENTER_CRITICAL(&s_event_spinlock);
        s_pending_dialog_end = true;
        portEXIT_CRITICAL(&s_event_spinlock);
    }

    audio_chunk_t end = { .data = NULL, .len = 0 };
    if (xQueueSend(s_audio_queue, &end,
                   pdMS_TO_TICKS(AUDIO_END_SEND_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "TTS end marker enqueue failed");
        portENTER_CRITICAL(&s_event_spinlock);
        if (s_pending_dialog_end) {
            s_dialog_end = true;
            s_pending_dialog_end = false;
        }
        s_tts_active = false;
        s_music_active = false;
        s_turn_done = true;
        s_tts_playback_done = true;
        portEXIT_CRITICAL(&s_event_spinlock);
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
    static int16_t pcm[960];           // 60ms @16kHz
    static int16_t stereo[960 * 2];    // mono → stereo

    audio_chunk_t chunk;
    int played_frames = 0;
    bool tx_active = false;     // 跟踪 TX 是否已开启

    while (1) {
        // ★ 气泵命令：通知独立 pump 任务执行
        if (pump_request_pending() && s_pump_task) {
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
            portENTER_CRITICAL(&s_event_spinlock);
            if (s_tts_active) {
                s_tts_active = false;
                s_music_active = false;
                s_tts_guard_until_tick = xTaskGetTickCount() + pdMS_TO_TICKS(TTS_WAKE_GUARD_MS);
                if (s_pending_dialog_end) {
                    s_dialog_end = true;
                    s_pending_dialog_end = false;
                }
                s_turn_done = true;
                s_tts_playback_done = true;
            }
            portEXIT_CRITICAL(&s_event_spinlock);
            continue;
        }

        // Opus → PCM（解码器由 begin_tts_stream 预创建）
        if (!lock_decoder(pdMS_TO_TICKS(100))) {
            free(chunk.data);
            s_tts_chunks_dropped++;
            continue;
        }
        if (!s_decoder) {
            unlock_decoder();
            free(chunk.data);
            continue;
        }
        int samples = opus_decode(s_decoder, chunk.data, chunk.len, pcm, 960, 0);

        if (samples < 0) {
            ESP_LOGW(TAG, "Opus decode failed (%d), reset decoder and fill silence", samples);
            reset_opus_decoder_locked();
            memset(pcm, 0, sizeof(pcm));
            samples = 960;
        }
        unlock_decoder();
        free(chunk.data);  // 尽早释放

        // TX 已常开，记录播放状态即可
        tx_active = true;

        // Mono → Stereo（MAX98357A 接收立体声，只取左声道也能响）
        for (int i = 0; i < samples; i++) {
            int16_t sample = (int16_t)((float)pcm[i] * AUDIO_OUTPUT_GAIN);
            stereo[i * 2]     = sample;
            stereo[i * 2 + 1] = sample;
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
        s_last_disconnected_tick = 0;
        {
            char hello_json[320];
            snprintf(hello_json, sizeof(hello_json),
                     "{\"type\":\"hello\",\"version\":1,\"transport\":\"websocket\","
                     "\"device_id\":\"%s\","
                     "\"free_heap\":%u,\"min_free_heap\":%u,\"disconnect_count\":%lu,"
                     "\"audio_params\":{\"format\":\"opus\",\"sample_rate\":16000,"
                     "\"channels\":1,\"frame_duration\":60}}",
                     s_device_id,
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)esp_get_minimum_free_heap_size(),
                     (unsigned long)s_disconnect_count);
            esp_websocket_client_send_text(s_client, hello_json, strlen(hello_json),
                                           pdMS_TO_TICKS(1000));
        }
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        s_connected = false;
        s_last_disconnected_tick = xTaskGetTickCount();
        s_disconnect_count++;
        clear_ws_fragment();
        portENTER_CRITICAL(&s_event_spinlock);
        s_tts_active = false;
        s_music_active = false;
        s_tts_playback_done = false;
        portEXIT_CRITICAL(&s_event_spinlock);
        ESP_LOGW(TAG, "websocket disconnected count=%lu free_heap=%u min_free_heap=%u",
                 (unsigned long)s_disconnect_count,
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)esp_get_minimum_free_heap_size());
        // 通知 audio task 重置解码器 + 停止播放
        {
            clear_audio_queue();
            audio_chunk_t end = { .data = NULL, .len = 0 };
            xQueueSend(s_audio_queue, &end, 0);
        }
        break;

    case WEBSOCKET_EVENT_DATA:
        {
        bool aggregated_fragment = false;
        esp_websocket_event_data_t aggregate;
        if (data->payload_offset != 0 || !data->fin ||
            data->payload_len > data->data_len) {
            if (!append_ws_fragment(data)) {
                break;
            }
            if (!data->fin) {
                break;
            }
            if (s_ws_fragment_len != s_ws_fragment_total) {
                clear_ws_fragment();
                break;
            }
            aggregate = *data;
            aggregate.data_ptr = (const char *)s_ws_fragment_buf;
            aggregate.data_len = (int)s_ws_fragment_len;
            aggregate.payload_len = (int)s_ws_fragment_len;
            aggregate.payload_offset = 0;
            aggregate.op_code = s_ws_fragment_opcode;
            aggregate.fin = true;
            data = &aggregate;
            aggregated_fragment = true;
        }
        if (data->op_code == 0x02 && data->data_len > 0) {
            if (!s_tts_active) {
                s_tts_chunks_dropped++;
                if (aggregated_fragment) clear_ws_fragment();
                break;
            }
            enqueue_opus_frame((const uint8_t *)data->data_ptr,
                               (size_t)data->data_len,
                               "binary");
            if (aggregated_fragment) clear_ws_fragment();
            break;
        }

        if (data->op_code == 0x01 && data->data_len > 0) {
            cJSON *json = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (!json) {
                if (aggregated_fragment) clear_ws_fragment();
                break;
            }

            cJSON *type = cJSON_GetObjectItem(json, "type");
            if (type && cJSON_IsString(type)) {

                if (strcmp(type->valuestring, "tts_audio_start") == 0) {
                    cJSON *source = cJSON_GetObjectItem(json, "source");
                    const char *source_text = cJSON_GetStringValue(source);
                    begin_tts_stream(source_text && strstr(source_text, "music"));
                }
                else if (strcmp(type->valuestring, "tts") == 0) {
                    cJSON *state = cJSON_GetObjectItem(json, "state");
                    if (state && cJSON_IsString(state)) {
                        if (strcmp(state->valuestring, "start") == 0) {
                            cJSON *source = cJSON_GetObjectItem(json, "source");
                            const char *source_text = cJSON_GetStringValue(source);
                            begin_tts_stream(source_text && strstr(source_text, "music"));
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
                else if (strcmp(type->valuestring, "listen_once") == 0) {
                    cJSON *reason = cJSON_GetObjectItem(json, "reason");
                    if (reason && cJSON_IsString(reason) &&
                        (strcmp(reason->valuestring, "environment_adjustment_reply") == 0 ||
                         strcmp(reason->valuestring, "comfort_reply") == 0)) {
                        portENTER_CRITICAL(&s_event_spinlock);
                        s_interaction_listen_pending = true;
                        portEXIT_CRITICAL(&s_event_spinlock);
                    }
                }
                else if (strcmp(type->valuestring, "status") == 0) {
                    cJSON *msg = cJSON_GetObjectItem(json, "msg");
                    if (msg && cJSON_IsString(msg)) {
                        printf("[状态] %s\n", msg->valuestring);
                        screen_anim_set_subtitle("状态", msg->valuestring);
                    }
                    portENTER_CRITICAL(&s_event_spinlock);
                    s_turn_done = true;
                    portEXIT_CRITICAL(&s_event_spinlock);
                }
                else if (strcmp(type->valuestring, "screen_status") == 0) {
                    cJSON *msg = cJSON_GetObjectItem(json, "msg");
                    if (msg && cJSON_IsString(msg)) {
                        printf("[screen_status] %s\n", msg->valuestring);
                        cJSON *duration = cJSON_GetObjectItem(json, "duration_ms");
                        int duration_ms = cJSON_IsNumber(duration) ? duration->valueint : 0;
                        if (duration_ms > 0) {
                            screen_anim_set_subtitle_timed("状态", msg->valuestring,
                                                           (uint32_t)duration_ms);
                        } else {
                            screen_anim_set_subtitle("状态", msg->valuestring);
                        }
                    }
                }
                else if (strcmp(type->valuestring, "stt_result") == 0) {
                    cJSON *text = cJSON_GetObjectItem(json, "text");
                    if (text && cJSON_IsString(text)) {
                        printf("[你] %s\n", text->valuestring);
                        screen_anim_set_subtitle("你", text->valuestring);
                    }
                }
                else if (strcmp(type->valuestring, "avatar_update") == 0) {
                    const char *url = cJSON_GetStringValue(cJSON_GetObjectItem(json, "rgb666_url"));
                    cJSON *size_item = cJSON_GetObjectItem(json, "bin_size");
                    const char *crc_text = cJSON_GetStringValue(cJSON_GetObjectItem(json, "crc32"));
                    size_t size = cJSON_IsNumber(size_item)
                                      ? (size_t)cJSON_GetNumberValue(size_item)
                                      : AVATAR_STORAGE_RGB666_SIZE;
                    uint32_t crc32 = avatar_storage_parse_crc32_text(crc_text);
                    start_avatar_download(url, size, crc32);
                }
                else if (strcmp(type->valuestring, "avatar_restore_default") == 0) {
                    start_avatar_restore_default();
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
                    (void)led_ret;

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
                else if (strcmp(type->valuestring, "ir_cmd") == 0) {
                    const char *device = cJSON_GetStringValue(cJSON_GetObjectItem(json, "device"));
                    const char *action = cJSON_GetStringValue(cJSON_GetObjectItem(json, "action"));
                    esp_err_t ir_ret = sensor_ir_control_device(device, action);
                    bool fan_on = false;
                    bool humidifier_on = false;
                    bool air_conditioner_on = false;
                    sensor_ir_get_state(&fan_on, &humidifier_on);
                    sensor_ir_get_air_conditioner_state(&air_conditioner_on);

                    char ir_state[320];
                    snprintf(ir_state, sizeof(ir_state),
                              "{\"type\":\"ir_state\",\"ok\":%s,\"device\":\"%s\",\"action\":\"%s\","
                              "\"fan_on\":%s,\"humidifier_on\":%s,"
                              "\"air_conditioner_on\":%s,\"ac_on\":%s,\"ret\":%d}",
                              ir_ret == ESP_OK ? "true" : "false",
                              device ? device : "",
                              action ? action : "",
                              fan_on ? "true" : "false",
                              humidifier_on ? "true" : "false",
                              air_conditioner_on ? "true" : "false",
                              air_conditioner_on ? "true" : "false",
                              ir_ret);
                    ws_client_send_raw(ir_state);
                }
                else if (strcmp(type->valuestring, "snore_policy") == 0) {
                    cJSON *enabled_item = cJSON_GetObjectItem(json, "enabled");
                    cJSON *sleep_item = cJSON_GetObjectItem(json, "sleep_active");
                    cJSON *target_item = cJSON_GetObjectItem(json, "target_kpa");
                    cJSON *cooldown_item = cJSON_GetObjectItem(json, "cooldown_sec");
                    bool enabled = enabled_item ? cJSON_IsTrue(enabled_item) : true;
                    bool sleep_active = sleep_item ? cJSON_IsTrue(sleep_item) : false;
                    float target_kpa = cJSON_IsNumber(target_item)
                                           ? (float)cJSON_GetNumberValue(target_item)
                                           : 4.0f;
                    int cooldown_sec = cJSON_IsNumber(cooldown_item)
                                           ? cooldown_item->valueint
                                           : 300;
                    snore_detector_set_policy(enabled, sleep_active, target_kpa, cooldown_sec);
                    printf("[snore] policy enabled=%d sleep=%d target=%.2f kPa\n",
                           enabled ? 1 : 0,
                           sleep_active ? 1 : 0,
                           (double)target_kpa);
                    if (enabled && sleep_active) {
                        screen_anim_set_subtitle("睡眠", "鼾声监测待命中");
                    }
                }
                else if (strcmp(type->valuestring, "pillow_cmd") == 0) {
                    const char *action = cJSON_GetStringValue(cJSON_GetObjectItem(json, "action"));
                    if (!action) {
                        ESP_LOGW(TAG, "pillow_cmd missing action");
                        cJSON_Delete(json);
                        if (aggregated_fragment) clear_ws_fragment();
                        break;
                    }
                    cJSON *target_item = cJSON_GetObjectItem(json, "target_kpa");
                    bool has_target_kpa = cJSON_IsNumber(target_item);
                    bool continuous = cJSON_IsTrue(cJSON_GetObjectItem(json, "continuous"));
                    float target_kpa = has_target_kpa ? clamp_pillow_pressure_kpa((float)cJSON_GetNumberValue(target_item)) : 0.0f;
                    int dur = 0;

                    if (strcmp(action, "stop") == 0 || strcmp(action, "halt") == 0) {
                        submit_pump_request(PUMP_HALT, 0, 0.0f);
                        has_target_kpa = false;
                    } else if (strcmp(action, "release") == 0) {
                        submit_pump_request(PUMP_RELEASE, 0, 0.0f);
                        has_target_kpa = false;
                    } else if (has_target_kpa) {
                        // ★ 闭环模式：有目标气压，边充/放边读传感器，到位即停
                        if (strcmp(action, "tilt") == 0) {
                            submit_pump_request(PUMP_TILT_TO_KPA, 0, target_kpa);
                        } else if (strcmp(action, "recover") == 0) {
                            submit_pump_request(PUMP_RECOVER_TO_KPA, 0, target_kpa);
                        }
                    } else {
                        // ★ 开环模式：纯时间控制（兼容旧协议）
                        if (continuous) {
                            if (strcmp(action, "tilt") == 0) {
                                submit_pump_request(PUMP_TILT_CONTINUOUS, 0, 0.0f);
                            } else if (strcmp(action, "recover") == 0) {
                                submit_pump_request(PUMP_RECOVER_CONTINUOUS, 0, 0.0f);
                            }
                        } else {
                            cJSON *duration_item = cJSON_GetObjectItem(json, "duration_sec");
                            dur = cJSON_IsNumber(duration_item)
                                      ? duration_item->valueint : 3;
                            if (dur < 1) dur = PILLOW_MANUAL_MAX_SECONDS;
                            if (dur > PILLOW_MANUAL_MAX_SECONDS) {
                                dur = PILLOW_MANUAL_MAX_SECONDS;
                            }

                            if (strcmp(action, "tilt") == 0) {
                                submit_pump_request(PUMP_TILT, dur, 0.0f);
                            } else if (strcmp(action, "recover") == 0) {
                                submit_pump_request(PUMP_RECOVER, dur, 0.0f);
                            }
                        }
                    }
                    ESP_LOGI(TAG, "pillow_cmd action=%s target=%s%.2f duration=%d continuous=%d",
                             action,
                             has_target_kpa ? "" : "none/",
                             has_target_kpa ? target_kpa : 0.0f,
                             has_target_kpa ? 0 : dur,
                             continuous ? 1 : 0);
                }
                else if (strcmp(type->valuestring, "music_barge_result") == 0) {
                    cJSON *stop = cJSON_GetObjectItem(json, "stop");
                    bool should_stop = cJSON_IsTrue(stop);
                    if (should_stop) {
                        stop_music_playback_now();
                    }
                    portENTER_CRITICAL(&s_event_spinlock);
                    s_music_barge_stop = should_stop;
                    s_music_barge_result_ready = true;
                    portEXIT_CRITICAL(&s_event_spinlock);
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
                    cJSON_AddNumberToObject(data_obj, "neck_temp_c", sd.neck_temp_c);
                    cJSON_AddBoolToObject(data_obj, "neck_temp_valid", sd.neck_temp_valid);
                    cJSON *ntc_arr = cJSON_CreateArray();
                    for (int i = 0; i < SENSOR_NTC_COUNT; i++) {
                        cJSON *ntc = cJSON_CreateObject();
                        cJSON_AddNumberToObject(ntc, "id", i + 1);
                        cJSON_AddNumberToObject(ntc, "temp_c", sd.ntc_temp_c[i]);
                        cJSON_AddBoolToObject(ntc, "valid", sd.ntc_valid[i]);
                        cJSON_AddItemToArray(ntc_arr, ntc);
                    }
                    cJSON_AddItemToObject(data_obj, "ntc", ntc_arr);
                    cJSON_AddNumberToObject(data_obj, "radar_heart_bpm", sd.radar_heart_bpm);
                    cJSON_AddNumberToObject(data_obj, "radar_breath_bpm", sd.radar_breath_bpm);
                    cJSON_AddBoolToObject(data_obj, "radar_valid", sd.radar_valid);
                    if (sd.radar_valid) {
                        /* 使用统一字段名；服务端兼容 radar_heart_bpm / radar_breath_bpm */
                        cJSON_AddNumberToObject(data_obj, "heart_rate_bpm", sd.radar_heart_bpm);
                        cJSON_AddNumberToObject(data_obj, "breath_rate_bpm", sd.radar_breath_bpm);
                    }
                    cJSON_AddNumberToObject(data_obj, "motion_level", sd.body_motion_level);
                    cJSON_AddNumberToObject(data_obj, "body_motion", sd.body_motion_level);
                    cJSON_AddBoolToObject(data_obj, "body_motion_valid", sd.body_motion_valid);
                    cJSON *fsr_arr = cJSON_CreateArray();
                    for (int i = 0; i < 4; i++) {
                        cJSON *fsr = cJSON_CreateObject();
                        cJSON_AddNumberToObject(fsr, "id", i + 1);
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
                    bool fan_on = false;
                    bool humidifier_on = false;
                    bool air_conditioner_on = false;
                    sensor_ir_get_state(&fan_on, &humidifier_on);
                    sensor_ir_get_air_conditioner_state(&air_conditioner_on);
                    cJSON_AddBoolToObject(data_obj, "fan_on", fan_on);
                    cJSON_AddBoolToObject(data_obj, "humidifier_on", humidifier_on);
                    cJSON_AddBoolToObject(data_obj, "air_conditioner_on", air_conditioner_on);
                    cJSON_AddBoolToObject(data_obj, "ac_on", air_conditioner_on);
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
                    if (json_str) {
                        if (!defer_text_send(json_str)) {
                            ESP_LOGW(TAG, "sensor_data deferred send queue full");
                        }
                        free(json_str);
                    } else {
                        ESP_LOGE(TAG, "sensor_data JSON alloc failed free_heap=%u min_free_heap=%u",
                                 (unsigned)esp_get_free_heap_size(),
                                 (unsigned)esp_get_minimum_free_heap_size());
                        defer_text_send("{\"type\":\"sensor_data\",\"data\":{\"error\":\"json_alloc_failed\"}}");
                    }
                    cJSON_Delete(resp);
                    printf("[pressure] kPa=%.2f valid=%d\n",
                           sd.pressure_kpa, sd.pressure_valid ? 1 : 0);
                }
                else if (strcmp(type->valuestring, "dialog_end") == 0) {
                    end_tts_stream(true);
                }
                else if (strcmp(type->valuestring, "wake_ack") == 0) {
                    cJSON *wake_id = cJSON_GetObjectItem(json, "wake_id");
                    if (wake_id && cJSON_IsNumber(wake_id) && wake_id->valuedouble > 0) {
                        portENTER_CRITICAL(&s_event_spinlock);
                        s_wake_ack_id = (uint32_t)wake_id->valuedouble;
                        portEXIT_CRITICAL(&s_event_spinlock);
                        printf("wake acknowledged id=%lu\n",
                               (unsigned long)s_wake_ack_id);
                    }
                }
                else if (strcmp(type->valuestring, "pong") == 0) {
                    // 心跳，忽略
                }
            }
            cJSON_Delete(json);
        }
        if (aggregated_fragment) clear_ws_fragment();
        break;
        }

    case WEBSOCKET_EVENT_ERROR: {
        s_last_disconnected_tick = xTaskGetTickCount();
        int error_type = data ? data->error_handle.error_type : -1;
        int status_code = data ? data->error_handle.esp_ws_handshake_status_code : 0;
        if (data && error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGW(TAG,
                     "websocket error type=%d esp_err=%s stack=%d sock_errno=%d status=%d free_heap=%u",
                     error_type,
                     esp_err_to_name(data->error_handle.esp_tls_last_esp_err),
                     data->error_handle.esp_tls_stack_err,
                     data->error_handle.esp_transport_sock_errno,
                     status_code,
                     (unsigned)esp_get_free_heap_size());
        } else {
            ESP_LOGW(TAG, "websocket error type=%d status=%d free_heap=%u",
                     error_type, status_code,
                     (unsigned)esp_get_free_heap_size());
        }
        break;
    }

    default:
        break;
    }
}


// ============================================================
//  公开 API
// ============================================================

void ws_client_start(const char *uri)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(s_device_id, sizeof(s_device_id),
                 "esp32s3-%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // Audio queue depth is capped to keep small queue storage out of internal RAM pressure.
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(audio_chunk_t));
    if (!s_audio_queue) {
        ESP_LOGE(TAG, "audio queue create failed");
        return;
    }

    s_pump_queue = xQueueCreate(1, sizeof(pump_request_t));
    if (!s_pump_queue) {
        ESP_LOGE(TAG, "pump queue create failed");
        vQueueDelete(s_audio_queue);
        s_audio_queue = NULL;
        return;
    }
    s_decoder_mutex = xSemaphoreCreateMutex();
    if (!s_decoder_mutex) {
        ESP_LOGE(TAG, "decoder mutex create failed");
        vQueueDelete(s_pump_queue);
        vQueueDelete(s_audio_queue);
        s_pump_queue = NULL;
        s_audio_queue = NULL;
        return;
    }
    s_send_mutex = xSemaphoreCreateMutex();
    if (!s_send_mutex) {
        ESP_LOGE(TAG, "websocket send mutex create failed");
        vSemaphoreDelete(s_decoder_mutex);
        vQueueDelete(s_pump_queue);
        vQueueDelete(s_audio_queue);
        s_decoder_mutex = NULL;
        s_pump_queue = NULL;
        s_audio_queue = NULL;
        return;
    }
    s_deferred_send_queue = xQueueCreate(4, sizeof(char *));
    if (!s_deferred_send_queue) {
        ESP_LOGE(TAG, "deferred send queue create failed");
        vSemaphoreDelete(s_send_mutex);
        vSemaphoreDelete(s_decoder_mutex);
        vQueueDelete(s_pump_queue);
        vQueueDelete(s_audio_queue);
        s_send_mutex = NULL;
        s_decoder_mutex = NULL;
        s_pump_queue = NULL;
        s_audio_queue = NULL;
        return;
    }
    if (xTaskCreate(deferred_send_task, "ws_tx", 4096, NULL, 4,
                    &s_deferred_send_task) != pdPASS) {
        ESP_LOGE(TAG, "deferred send task create failed");
        vQueueDelete(s_deferred_send_queue);
        vSemaphoreDelete(s_send_mutex);
        vSemaphoreDelete(s_decoder_mutex);
        vQueueDelete(s_pump_queue);
        vQueueDelete(s_audio_queue);
        s_deferred_send_queue = NULL;
        s_send_mutex = NULL;
        s_decoder_mutex = NULL;
        s_pump_queue = NULL;
        s_audio_queue = NULL;
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
    if (xTaskCreate(pump_task, "pump", 4096, NULL, 2, &s_pump_task) != pdPASS) {
        ESP_LOGE(TAG, "pump task create failed");
        s_pump_task = NULL;
        if (s_deferred_send_task) {
            vTaskDelete(s_deferred_send_task);
            s_deferred_send_task = NULL;
        }
        if (s_deferred_send_queue) {
            char *pending = NULL;
            while (xQueueReceive(s_deferred_send_queue, &pending, 0) == pdTRUE) {
                free(pending);
            }
            vQueueDelete(s_deferred_send_queue);
            s_deferred_send_queue = NULL;
        }
        vQueueDelete(s_pump_queue);
        vQueueDelete(s_audio_queue);
        vSemaphoreDelete(s_decoder_mutex);
        vSemaphoreDelete(s_send_mutex);
        s_pump_queue = NULL;
        s_audio_queue = NULL;
        s_decoder_mutex = NULL;
        s_send_mutex = NULL;
        s_deferred_send_queue = NULL;
        return;
    }

    // WebSocket 客户端：16KB 栈（库内部帧解析也需要栈空间！）
    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .buffer_size = 8192,
        .reconnect_timeout_ms = 1000,
        .network_timeout_ms = 15000,
        .ping_interval_sec = 10,
        .pingpong_timeout_sec = 8,
        .disable_pingpong_discon = true,
        .keep_alive_enable = true,
        .keep_alive_idle = 10,
        .keep_alive_interval = 5,
        .keep_alive_count = 3,
        .enable_close_reconnect = true,
        .disable_auto_reconnect = false,
        .task_stack = 12288,
    };

    s_client = esp_websocket_client_init(&ws_cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "WebSocket client init failed");
        return;
    }
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    esp_websocket_client_start(s_client);
}


static int ws_send_frame(const char *data, int len, bool binary)
{
    if (!data || len <= 0 || !s_client || !s_send_mutex ||
        !esp_websocket_client_is_connected(s_client)) {
        return -1;
    }
    if (xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "websocket send lock timeout");
        return -1;
    }
    int sent = -1;
    if (esp_websocket_client_is_connected(s_client)) {
        sent = binary
            ? esp_websocket_client_send_bin(s_client, data, len, pdMS_TO_TICKS(1000))
            : esp_websocket_client_send_text(s_client, data, len, pdMS_TO_TICKS(1000));
    }
    xSemaphoreGive(s_send_mutex);
    return sent;
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
    if (!json_str) {
        ESP_LOGE(TAG, "ws_client_send_text: cJSON alloc failed");
        cJSON_Delete(msg);
        return false;
    }
    int json_len = strlen(json_str);

    int sent = ws_send_frame(json_str, json_len, false);
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
    int sent = ws_send_frame(json_str, json_len, false);
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

    int sent = ws_send_frame((const char *)data, len, true);
    return sent >= 0;
}

bool ws_client_is_connected(void)
{
    return s_connected && s_client && esp_websocket_client_is_connected(s_client);
}

uint32_t ws_client_disconnected_ms(void)
{
    if (ws_client_is_connected()) {
        return 0;
    }
    TickType_t since = s_last_disconnected_tick;
    if (since == 0) {
        return UINT32_MAX;
    }
    TickType_t elapsed = xTaskGetTickCount() - since;
    return (uint32_t)(elapsed * portTICK_PERIOD_MS);
}

bool ws_client_is_tts_active(void)
{
    return s_tts_active;
}

bool ws_client_is_music_active(void)
{
    return s_music_active;
}

bool ws_client_consume_music_barge_result(bool *stop)
{
    bool ready;
    portENTER_CRITICAL(&s_event_spinlock);
    ready = s_music_barge_result_ready;
    if (ready) {
        if (stop) {
            *stop = s_music_barge_stop;
        }
        s_music_barge_result_ready = false;
    }
    portEXIT_CRITICAL(&s_event_spinlock);
    return ready;
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

bool ws_client_consume_wake_ack(uint32_t wake_id)
{
    bool matched = false;
    portENTER_CRITICAL(&s_event_spinlock);
    if (wake_id != 0 && s_wake_ack_id == wake_id) {
        s_wake_ack_id = 0;
        matched = true;
    }
    portEXIT_CRITICAL(&s_event_spinlock);
    return matched;
}

static void clear_ws_fragment(void)
{
    free(s_ws_fragment_buf);
    s_ws_fragment_buf = NULL;
    s_ws_fragment_len = 0;
    s_ws_fragment_total = 0;
    s_ws_fragment_opcode = 0;
}

static bool append_ws_fragment(const esp_websocket_event_data_t *data)
{
    if (!data || data->data_len < 0 || data->payload_len <= 0 ||
        data->payload_len > WS_FRAGMENT_MAX_BYTES || data->payload_offset < 0 ||
        (size_t)data->payload_offset + (size_t)data->data_len >
            (size_t)data->payload_len) {
        clear_ws_fragment();
        return false;
    }

    if (data->payload_offset == 0) {
        clear_ws_fragment();
        s_ws_fragment_buf = malloc((size_t)data->payload_len);
        if (!s_ws_fragment_buf) {
            return false;
        }
        s_ws_fragment_total = (size_t)data->payload_len;
        s_ws_fragment_opcode = data->op_code;
    }

    if (!s_ws_fragment_buf || (size_t)data->payload_offset != s_ws_fragment_len) {
        clear_ws_fragment();
        return false;
    }
    memcpy(s_ws_fragment_buf + s_ws_fragment_len, data->data_ptr,
           (size_t)data->data_len);
    s_ws_fragment_len += (size_t)data->data_len;
    return true;
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

bool ws_client_consume_tts_playback_done(void)
{
    portENTER_CRITICAL(&s_event_spinlock);
    bool value = s_tts_playback_done;
    s_tts_playback_done = false;
    portEXIT_CRITICAL(&s_event_spinlock);
    return value;
}

bool ws_client_consume_interaction_listen_request(void)
{
    portENTER_CRITICAL(&s_event_spinlock);
    bool value = s_interaction_listen_pending;
    s_interaction_listen_pending = false;
    portEXIT_CRITICAL(&s_event_spinlock);
    return value;
}
