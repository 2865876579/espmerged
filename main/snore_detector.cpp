#include "snore_detector.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "afe_wake_word.h"
#include "screen_anim.h"
#include "sensors.h"
#include "snore_infer_backend.h"
#include "ws_client.h"

#include "model-parameters/model_metadata.h"

static const char *TAG = "snore_ai";

static constexpr int kSampleRate = 16000;
static constexpr int kWindowSamples = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
static constexpr float kSnoreStrongThreshold = 0.60f;
static constexpr float kSnoreCandidateThreshold = 0.48f;
static constexpr float kSnoreWeakThreshold = 0.35f;
static constexpr float kSnoreAllowedNegativeMargin = 0.10f;
static constexpr float kWeakHitRmsMultiplier = 1.0f;
static constexpr float kStrongNonSnoreThreshold = 0.85f;
static constexpr int kTriggerScore = 4;
static constexpr int kMaxSnoreScore = 8;
static constexpr int kDetectionHoldWindows = 2;
static constexpr int kQuietWindowsBeforeDecay = 3;
static constexpr int kCalibrationWindows = 5;
static constexpr float kInputGain = 1.8f;
static constexpr int kBaseMinRmsForDecision = 180;
static constexpr float kNoiseRmsMultiplier = 0.35f;
static constexpr int kClipPeakReject = 30000;
static constexpr int kActivityChunks = 8;
static constexpr int kMinActiveChunksForSnore = 2;
static constexpr int kDefaultCooldownSec = 300;
static constexpr float kDefaultTargetKpa = 4.0f;

static_assert(EI_CLASSIFIER_FREQUENCY == kSampleRate, "snore model must be 16kHz");

static TaskHandle_t s_task = nullptr;
static int16_t *s_audio_window = nullptr;
static portMUX_TYPE s_policy_lock = portMUX_INITIALIZER_UNLOCKED;

static bool s_policy_enabled = false;
static bool s_sleep_active = false;
static bool s_interaction_active = false;
static float s_target_kpa = kDefaultTargetKpa;
static int s_cooldown_sec = kDefaultCooldownSec;

typedef struct {
    bool enabled;
    bool sleep_active;
    bool interaction_active;
    float target_kpa;
    int cooldown_sec;
} snore_policy_snapshot_t;

static void decay_score(int *value, int amount)
{
    if (!value || amount <= 0) return;
    *value = (*value > amount) ? (*value - amount) : 0;
}

static float clamp_target_kpa(float value)
{
    if (!isfinite(value) || value <= 0.0f) return kDefaultTargetKpa;
    if (value < 0.5f) return 0.5f;
    if (value > 5.0f) return 5.0f;
    return value;
}

static void get_policy(snore_policy_snapshot_t *out)
{
    portENTER_CRITICAL(&s_policy_lock);
    out->enabled = s_policy_enabled;
    out->sleep_active = s_sleep_active;
    out->interaction_active = s_interaction_active;
    out->target_kpa = s_target_kpa;
    out->cooldown_sec = s_cooldown_sec;
    portEXIT_CRITICAL(&s_policy_lock);
}

void snore_detector_set_policy(bool enabled,
                               bool sleep_active,
                               float target_kpa,
                               int cooldown_sec)
{
    portENTER_CRITICAL(&s_policy_lock);
    s_policy_enabled = enabled;
    s_sleep_active = sleep_active;
    s_target_kpa = clamp_target_kpa(target_kpa);
    s_cooldown_sec = cooldown_sec > 0 ? cooldown_sec : kDefaultCooldownSec;
    portEXIT_CRITICAL(&s_policy_lock);

    ESP_LOGI(TAG, "policy enabled=%d sleep=%d target=%.2f cooldown=%ds",
             enabled ? 1 : 0,
             sleep_active ? 1 : 0,
             (double)clamp_target_kpa(target_kpa),
             cooldown_sec > 0 ? cooldown_sec : kDefaultCooldownSec);
}

void snore_detector_set_interaction_active(bool active)
{
    portENTER_CRITICAL(&s_policy_lock);
    s_interaction_active = active;
    portEXIT_CRITICAL(&s_policy_lock);
}

bool snore_detector_is_enabled(void)
{
    snore_policy_snapshot_t p;
    get_policy(&p);
    return p.enabled && p.sleep_active;
}

static void calc_level(const int16_t *pcm, int samples, int *out_rms, int *out_peak)
{
    int64_t sum_sq = 0;
    int peak = 0;
    for (int i = 0; i < samples; i++) {
        int v = pcm[i];
        int a = v >= 0 ? v : -v;
        if (a > peak) peak = a;
        sum_sq += (int64_t)v * v;
    }
    if (out_rms) {
        *out_rms = (int)sqrtf((float)sum_sq / samples);
    }
    if (out_peak) {
        *out_peak = peak;
    }
}

static void remove_dc_offset(int16_t *pcm, int samples)
{
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += pcm[i];
    }
    int mean = (int)(sum / samples);
    for (int i = 0; i < samples; i++) {
        int v = (int)pcm[i] - mean;
        if (v > INT16_MAX) v = INT16_MAX;
        if (v < INT16_MIN) v = INT16_MIN;
        pcm[i] = (int16_t)v;
    }
}

static void apply_input_gain(int16_t *pcm, int samples, float gain)
{
    if (gain <= 0.0f || gain == 1.0f) return;
    for (int i = 0; i < samples; i++) {
        int v = (int)((float)pcm[i] * gain);
        if (v > INT16_MAX) v = INT16_MAX;
        if (v < INT16_MIN) v = INT16_MIN;
        pcm[i] = (int16_t)v;
    }
}

static int count_active_chunks(const int16_t *pcm, int samples, int chunks, int threshold_rms)
{
    if (!pcm || samples <= 0 || chunks <= 0) return 0;
    int active = 0;
    int chunk_size = samples / chunks;
    for (int c = 0; c < chunks; c++) {
        int start = c * chunk_size;
        int end = (c == chunks - 1) ? samples : (start + chunk_size);
        int count = end - start;
        if (count <= 0) continue;
        int64_t sum_sq = 0;
        for (int i = start; i < end; i++) {
            int v = pcm[i];
            sum_sq += (int64_t)v * v;
        }
        int chunk_rms = (int)sqrtf((float)sum_sq / count);
        if (chunk_rms >= threshold_rms) active++;
    }
    return active;
}

static void send_snore_event(bool adjusted,
                             float target_kpa,
                             float snore,
                             float non_snore,
                             int rms,
                             int peak,
                             int active_chunks)
{
    char json[384];
    snprintf(json, sizeof(json),
             "{\"type\":\"snore_event\",\"snore\":true,"
             "\"score\":%.3f,\"non_snore\":%.3f,"
             "\"rms\":%d,\"peak\":%d,\"active_chunks\":%d,"
             "\"action\":\"inflate\",\"adjusted\":%s,"
             "\"target_kpa\":%.2f,\"source\":\"local_snore_ai\"}",
             (double)snore,
             (double)non_snore,
             rms,
             peak,
             active_chunks,
             adjusted ? "true" : "false",
             (double)target_kpa);
    ws_client_send_raw(json);
}

static void snore_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "starting %s, window=%d samples", espdl_snore_adapter_name(), kWindowSamples);

    s_audio_window = (int16_t *)heap_caps_malloc(kWindowSamples * sizeof(int16_t),
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_audio_window) {
        s_audio_window = (int16_t *)malloc(kWindowSamples * sizeof(int16_t));
    }
    if (!s_audio_window) {
        ESP_LOGE(TAG, "audio window malloc failed");
        vTaskDelete(NULL);
        return;
    }

    int snore_score = 0;
    int snore_hold_windows = 0;
    int quiet_windows = 0;
    int calibration_count = 0;
    int64_t calibration_rms_sum = 0;
    int noise_rms = 0;
    int noise_floor_rms = 0;
    bool was_active = false;
    uint32_t last_total = 0;
    int64_t last_trigger_us = 0;

    while (1) {
        snore_policy_snapshot_t policy;
        get_policy(&policy);

        bool active = policy.enabled && policy.sleep_active &&
                      !policy.interaction_active &&
                      !ws_client_is_tts_active() &&
                      !ws_client_is_tts_guard_active() &&
                      sensor_person_on_bed();

        if (!active) {
            if (was_active) {
                screen_anim_set_subtitle("睡眠", "鼾声监测已暂停");
            }
            was_active = false;
            snore_score = 0;
            snore_hold_windows = 0;
            quiet_windows = 0;
            calibration_count = 0;
            calibration_rms_sum = 0;
            noise_rms = 0;
            noise_floor_rms = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!was_active) {
            ESP_LOGI(TAG, "snore monitor active");
            screen_anim_set_subtitle("睡眠", "鼾声监测已开启");
            was_active = true;
            last_total = 0;
        }

        uint32_t total = 0;
        int got = afe_recent_audio_copy_latest(s_audio_window, kWindowSamples, &total);
        if (got < kWindowSamples || (uint32_t)(total - last_total) < (uint32_t)(kWindowSamples / 2)) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        last_total = total;

        remove_dc_offset(s_audio_window, kWindowSamples);
        apply_input_gain(s_audio_window, kWindowSamples, kInputGain);

        int rms = 0;
        int peak = 0;
        calc_level(s_audio_window, kWindowSamples, &rms, &peak);

        if (calibration_count < kCalibrationWindows) {
            if (peak < kClipPeakReject) {
                calibration_rms_sum += rms;
                calibration_count++;
                noise_rms = (int)(calibration_rms_sum / calibration_count);
                if (noise_floor_rms == 0 || rms < noise_floor_rms) {
                    noise_floor_rms = rms;
                }
            }
            ESP_LOGI(TAG, "calibrating %d/%d rms=%d peak=%d noise=%d floor=%d",
                     calibration_count, kCalibrationWindows, rms, peak, noise_rms, noise_floor_rms);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        int gate_noise_rms = noise_floor_rms > 0 ? noise_floor_rms : noise_rms;
        int min_rms = (int)(gate_noise_rms * kNoiseRmsMultiplier);
        if (min_rms < kBaseMinRmsForDecision) {
            min_rms = kBaseMinRmsForDecision;
        }

        bool clipped = peak >= kClipPeakReject;
        bool loud_enough = rms >= min_rms;
        if (!loud_enough || clipped) {
            quiet_windows++;
            if (quiet_windows >= kQuietWindowsBeforeDecay) {
                decay_score(&snore_score, 1);
                quiet_windows = 0;
            }
            decay_score(&snore_hold_windows, 1);
            ESP_LOGI(TAG, "quiet rms=%d min=%d peak=%d noise=%d clip=%d score=%d",
                     rms, min_rms, peak, gate_noise_rms, clipped ? 1 : 0, snore_score);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        quiet_windows = 0;

        snore_infer_result_t infer = {};
        if (!espdl_snore_adapter_run(s_audio_window, kWindowSamples, &infer)) {
            ESP_LOGW(TAG, "inference failed");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int activity_threshold = min_rms / 2;
        if (activity_threshold < 300) activity_threshold = 300;
        int active_chunks = count_active_chunks(s_audio_window, kWindowSamples,
                                                kActivityChunks, activity_threshold);

        bool sustained_enough = active_chunks >= kMinActiveChunksForSnore;
        bool label_close_enough = infer.snore + kSnoreAllowedNegativeMargin >= infer.non_snore;
        bool strong_hit = infer.snore >= kSnoreStrongThreshold && label_close_enough && sustained_enough;
        bool candidate_hit = infer.snore >= kSnoreCandidateThreshold && label_close_enough && sustained_enough;
        bool weak_hit = infer.snore >= kSnoreWeakThreshold &&
                        rms >= (int)(min_rms * kWeakHitRmsMultiplier) &&
                        sustained_enough;
        bool strong_non_snore = infer.non_snore >= kStrongNonSnoreThreshold &&
                                infer.non_snore > infer.snore + 0.50f;

        int hit_level = 0;
        if (strong_hit) {
            snore_score += 3;
            hit_level = 3;
        } else if (candidate_hit) {
            snore_score += 2;
            hit_level = 2;
        } else if (weak_hit) {
            snore_score += 1;
            hit_level = 1;
        } else {
            decay_score(&snore_score, strong_non_snore ? 4 : 1);
        }
        if (snore_score > kMaxSnoreScore) snore_score = kMaxSnoreScore;

        if (hit_level > 0 && snore_score >= kTriggerScore) {
            snore_hold_windows = kDetectionHoldWindows;
        } else if (strong_non_snore) {
            snore_hold_windows = 0;
        } else {
            decay_score(&snore_hold_windows, 1);
        }

        ESP_LOGI(TAG,
                 "snore=%.3f non=%.3f rms=%d min=%d peak=%d act=%d hit=%d score=%d hold=%d dsp=%d cls=%d",
                 (double)infer.snore, (double)infer.non_snore, rms, min_rms, peak,
                 active_chunks, hit_level, snore_score, snore_hold_windows,
                 infer.dsp_ms, infer.classification_ms);

        int64_t now_us = esp_timer_get_time();
        bool cooldown_ok = last_trigger_us == 0 ||
                           (now_us - last_trigger_us) >= (int64_t)policy.cooldown_sec * 1000000LL;
        if (snore_hold_windows > 0 && cooldown_ok) {
            last_trigger_us = now_us;
            bool adjusted = ws_client_request_pillow_tilt_to_kpa(policy.target_kpa, "snore");
            screen_anim_set_subtitle("睡眠",
                                     adjusted ? "检测到鼾声，已自动调整枕头"
                                              : "检测到鼾声，枕头调整未执行");
            send_snore_event(adjusted, policy.target_kpa, infer.snore, infer.non_snore,
                             rms, peak, active_chunks);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void snore_detector_start(void)
{
    if (s_task) {
        return;
    }

    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        snore_task,
        "snore_ai",
        32768,
        NULL,
        1,
        &s_task,
        0,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ret != pdPASS) {
        ESP_LOGW(TAG, "PSRAM stack create failed, fallback internal");
        ret = xTaskCreatePinnedToCore(snore_task, "snore_ai", 32768, NULL, 1, &s_task, 0);
    }
    if (ret != pdPASS) {
        s_task = nullptr;
        ESP_LOGE(TAG, "snore task create failed");
    }
}
