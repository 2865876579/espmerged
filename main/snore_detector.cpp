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
#include "ei_model_wrapper.h"
#include "screen_anim.h"
#include "sensors.h"
#include "ws_client.h"

#include "model-parameters/model_metadata.h"

static const char *TAG = "snore_ai";

static constexpr int kSampleRate = 16000;
static constexpr int kWindowSamples = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
static constexpr uint32_t kInferenceStrideSamples = kSampleRate / 2;
static constexpr float kSnoringThreshold = 0.75f;
static constexpr float kSnoringClearThreshold = 0.45f;
static constexpr int kVoteWindowSize = 10;
static constexpr int kVotesRequired = 8;
static constexpr int kClearVotesRequired = 8;
static constexpr int kDefaultCooldownSec = 300;
static constexpr float kDefaultTargetKpa = 4.0f;

static_assert(EI_CLASSIFIER_FREQUENCY == kSampleRate, "snore model must be 16kHz");
static_assert(kWindowSamples <= 65536, "snore model window exceeds the audio ring");

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

typedef struct {
    uint8_t snoring[kVoteWindowSize];
    uint8_t clear[kVoteWindowSize];
    uint8_t size;
    uint8_t position;
    uint8_t snoring_count;
    uint8_t clear_count;
    bool active;
} snore_vote_state_t;

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
    const float clamped_target = clamp_target_kpa(target_kpa);
    const int effective_cooldown = cooldown_sec > 0 ? cooldown_sec : kDefaultCooldownSec;

    portENTER_CRITICAL(&s_policy_lock);
    s_policy_enabled = enabled;
    s_sleep_active = sleep_active;
    s_target_kpa = clamped_target;
    s_cooldown_sec = effective_cooldown;
    portEXIT_CRITICAL(&s_policy_lock);

    ESP_LOGI(TAG, "policy enabled=%d sleep=%d target=%.2f cooldown=%ds",
             enabled ? 1 : 0,
             sleep_active ? 1 : 0,
             (double)clamped_target,
             effective_cooldown);
}

void snore_detector_set_interaction_active(bool active)
{
    portENTER_CRITICAL(&s_policy_lock);
    s_interaction_active = active;
    portEXIT_CRITICAL(&s_policy_lock);
}

bool snore_detector_is_enabled(void)
{
    snore_policy_snapshot_t policy;
    get_policy(&policy);
    return policy.enabled && policy.sleep_active;
}

static void reset_votes(snore_vote_state_t *votes)
{
    memset(votes, 0, sizeof(*votes));
}

static void add_vote(snore_vote_state_t *votes, float snoring_probability)
{
    const uint8_t snoring = snoring_probability >= kSnoringThreshold ? 1 : 0;
    const uint8_t clear = snoring_probability < kSnoringClearThreshold ? 1 : 0;

    if (votes->size == kVoteWindowSize) {
        votes->snoring_count -= votes->snoring[votes->position];
        votes->clear_count -= votes->clear[votes->position];
    } else {
        votes->size++;
    }

    votes->snoring[votes->position] = snoring;
    votes->clear[votes->position] = clear;
    votes->snoring_count += snoring;
    votes->clear_count += clear;
    votes->position = (uint8_t)((votes->position + 1) % kVoteWindowSize);
}

static void send_snore_event(bool adjusted,
                             float target_kpa,
                             float snoring,
                             float not_snoring)
{
    char json[288];
    snprintf(json, sizeof(json),
             "{\"type\":\"snore_event\",\"snore\":true,"
             "\"score\":%.3f,\"non_snore\":%.3f,"
             "\"action\":\"inflate\",\"adjusted\":%s,"
             "\"target_kpa\":%.2f,\"source\":\"local_snore_ai_10of8\"}",
             (double)snoring,
             (double)not_snoring,
             adjusted ? "true" : "false",
             (double)target_kpa);
    ws_client_send_raw(json);
}

static void snore_task(void *arg)
{
    (void)arg;

    if (ei_model_frame_size() != (size_t)kWindowSamples) {
        ESP_LOGE(TAG, "model frame mismatch: wrapper=%u metadata=%d",
                 (unsigned)ei_model_frame_size(), kWindowSamples);
        s_task = nullptr;
        vTaskDeleteWithCaps(nullptr);
        return;
    }

    s_audio_window = (int16_t *)heap_caps_malloc(
        (size_t)kWindowSamples * sizeof(*s_audio_window),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_audio_window) {
        ESP_LOGE(TAG, "PSRAM inference window allocation failed (%u bytes)",
                 (unsigned)((size_t)kWindowSamples * sizeof(*s_audio_window)));
        s_task = nullptr;
        vTaskDeleteWithCaps(nullptr);
        return;
    }

    printf("model ready: project=%d frame=%d threshold=%.2f votes=%d/%d clear=%.2f\n",
           EI_CLASSIFIER_PROJECT_ID,
           kWindowSamples,
           (double)kSnoringThreshold,
           kVotesRequired,
           kVoteWindowSize,
           (double)kSnoringClearThreshold);

    snore_vote_state_t votes = {};
    bool monitoring = false;
    uint32_t last_total = 0;
    int64_t last_adjust_us = 0;

    while (true) {
        snore_policy_snapshot_t policy;
        get_policy(&policy);

        const bool should_monitor = policy.enabled &&
                                    policy.sleep_active &&
                                    !policy.interaction_active &&
                                    !ws_client_is_tts_active() &&
                                    !ws_client_is_tts_guard_active() &&
                                    sensor_person_on_bed();

        if (!should_monitor) {
            if (monitoring) {
                screen_anim_set_subtitle("睡眠", "鼾声监测已暂停");
                reset_votes(&votes);
                last_total = 0;
            }
            monitoring = false;
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (!monitoring) {
            monitoring = true;
            reset_votes(&votes);
            last_total = 0;
            screen_anim_set_subtitle("睡眠", "鼾声监测已开启");
            printf("snore monitor active\n");
        }

        uint32_t total = 0;
        const int copied = afe_recent_audio_copy_latest(
            s_audio_window, kWindowSamples, &total);
        if (copied < kWindowSamples ||
            (last_total != 0 && (uint32_t)(total - last_total) < kInferenceStrideSamples)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        last_total = total;

        float snoring_probability = 0.0f;
        float not_snoring_probability = 0.0f;
        const int64_t started_us = esp_timer_get_time();
        const int error = ei_model_run(s_audio_window,
                                       kWindowSamples,
                                       &snoring_probability,
                                       &not_snoring_probability);
        const int64_t inference_ms = (esp_timer_get_time() - started_us) / 1000;
        if (error != 0) {
            ESP_LOGE(TAG, "inference failed: %d (%lld ms)",
                     error, (long long)inference_ms);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        add_vote(&votes, snoring_probability);
        const bool window_full = votes.size == kVoteWindowSize;
        const bool just_detected = !votes.active && window_full &&
                                   votes.snoring_count >= kVotesRequired;
        const bool just_cleared = votes.active && window_full &&
                                  votes.clear_count >= kClearVotesRequired;

        if (just_detected) {
            votes.active = true;
        } else if (just_cleared) {
            votes.active = false;
        }

        char decision[64];
        if (just_detected) {
            snprintf(decision, sizeof(decision), ">>> SNORING DETECTED <<<");
        } else if (just_cleared) {
            snprintf(decision, sizeof(decision), "snoring cleared");
        } else if (votes.active) {
            snprintf(decision, sizeof(decision), "snoring active");
        } else if (votes.snoring_count > 0) {
            snprintf(decision, sizeof(decision),
                     "candidate %u/%d (window %u/%d)",
                     votes.snoring_count,
                     kVotesRequired,
                     votes.size,
                     kVoteWindowSize);
        } else {
            snprintf(decision, sizeof(decision), "not snoring");
        }
        printf("snoring=%.3f  not=%.3f  |  %s\n",
               (double)snoring_probability,
               (double)not_snoring_probability,
               decision);
        fflush(stdout);
        printf("snore votes=%u/%d clear=%u/%d window=%u/%d inference=%lldms\n",
               votes.snoring_count,
               kVotesRequired,
               votes.clear_count,
               kClearVotesRequired,
               votes.size,
               kVoteWindowSize,
               (long long)inference_ms);

        if (just_detected) {
            const int64_t now_us = esp_timer_get_time();
            const bool cooldown_ok = last_adjust_us == 0 ||
                (now_us - last_adjust_us) >= (int64_t)policy.cooldown_sec * 1000000LL;
            if (cooldown_ok) {
                last_adjust_us = now_us;
                const bool adjusted = ws_client_request_pillow_tilt_to_kpa(
                    policy.target_kpa, "snore");
                screen_anim_set_subtitle(
                    "睡眠",
                    adjusted ? "检测到鼾声，已自动调整枕头"
                             : "检测到鼾声，枕头调整未执行");
                send_snore_event(adjusted,
                                 policy.target_kpa,
                                 snoring_probability,
                                 not_snoring_probability);
            } else {
                screen_anim_set_subtitle("睡眠", "检测到鼾声，调整冷却中");
                ESP_LOGI(TAG, "snore detected during %ds adjustment cooldown",
                         policy.cooldown_sec);
            }
        } else if (just_cleared) {
            screen_anim_set_subtitle("睡眠", "鼾声监测中");
            const bool released = ws_client_request_pillow_recover_to_kpa(
                0.0f, "snore_clear");
            if (!released) {
                ESP_LOGW(TAG, "snoring cleared but pillow release request failed");
            }
            ESP_LOGI(TAG, "snoring cleared");
        }

        vTaskDelay(1);
    }
}

void snore_detector_start(void)
{
    if (s_task) return;

    const BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        snore_task,
        "snore_ai",
        32768,
        nullptr,
        1,
        &s_task,
        0,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ret != pdPASS) {
        s_task = nullptr;
        ESP_LOGE(TAG, "snore task PSRAM stack allocation failed");
    }
}
