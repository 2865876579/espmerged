#include "pump_driver.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pump";

// ── LEDC PWM 配置（气泵调速） ────────────────────────
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_PUMP_CHANNEL   LEDC_CHANNEL_0
#define LEDC_FREQ_HZ        5000

// ── 内部状态 ──────────────────────────────────────────
static bool     s_pump_running  = false;
static bool     s_valve_open    = false;
static int64_t  s_pump_start_us = 0;
static int64_t  s_cooldown_until_us = 0;

// ── 初始化 ───────────────────────────────────────────
void pump_driver_init(void)
{
    /* 泄气阀：普通 GPIO */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << VALVE_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(VALVE_PIN, 0);

    /* 气泵：LEDC PWM 调速 */
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = PUMP_PIN,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_PUMP_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
    };
    ledc_channel_config(&ledc_channel);

    ESP_LOGI(TAG, "driver init OK  pump=GPIO%d(PWM)  valve=GPIO%d", PUMP_PIN, VALVE_PIN);
}

// ── 气泵 PWM ─────────────────────────────────────────
void pump_set_duty(uint8_t duty_pct)
{
    if (duty_pct > 100) duty_pct = 100;
    uint32_t duty = (uint32_t)duty_pct * 255 / 100;
    ledc_set_duty(LEDC_MODE, LEDC_PUMP_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_PUMP_CHANNEL);
}

bool pump_start(void)
{
    if (s_pump_running) return true;

    int64_t now = esp_timer_get_time();
    if (now < s_cooldown_until_us) {
        ESP_LOGW(TAG, "pump cooldown — wait %lld ms", (s_cooldown_until_us - now) / 1000);
        return false;
    }

    if (s_valve_open) {
        gpio_set_level(VALVE_PIN, 0);
        s_valve_open = false;
    }

    pump_set_duty(100);  // 默认全速
    s_pump_running  = true;
    s_pump_start_us = now;
    ESP_LOGI(TAG, "pump START");
    return true;
}

void pump_stop(void)
{
    if (!s_pump_running) return;

    pump_set_duty(0);
    s_pump_running  = false;
    s_cooldown_until_us = esp_timer_get_time() + PUMP_COOLDOWN_MS * 1000LL;
    ESP_LOGI(TAG, "pump STOP  run=%lldms  cooldown=%dms",
             (esp_timer_get_time() - s_pump_start_us) / 1000, PUMP_COOLDOWN_MS);
}

// ── 泄气阀 ───────────────────────────────────────────
bool valve_open(void)
{
    if (s_valve_open) return true;

    // ★ 开阀前先停泵
    if (s_pump_running) pump_stop();

    gpio_set_level(VALVE_PIN, 1);
    s_valve_open = true;
    ESP_LOGI(TAG, "valve OPEN");
    return true;
}

void valve_close(void)
{
    if (!s_valve_open) return;

    gpio_set_level(VALVE_PIN, 0);
    s_valve_open = false;
    ESP_LOGI(TAG, "valve CLOSE");
}

// ── 紧急泄气 ─────────────────────────────────────────
void emergency_release(void)
{
    ESP_LOGW(TAG, "EMERGENCY RELEASE — stop pump + open valve %dms", VALVE_RELEASE_MS);
    pump_stop();
    gpio_set_level(VALVE_PIN, 1);
    s_valve_open = true;

    // 定时关阀（调用方可通过 FreeRTOS timer 或外部延时处理）
    // 这里为了简洁，紧急泄气后由外部调用 valve_close()
}

void pump_clear_cooldown(void)
{
    s_cooldown_until_us = 0;
}

// ── 状态查询 ─────────────────────────────────────────
bool pump_is_running(void)     { return s_pump_running; }
bool valve_is_open(void)       { return s_valve_open; }

uint32_t pump_run_ms(void)
{
    if (!s_pump_running) return 0;
    return (uint32_t)((esp_timer_get_time() - s_pump_start_us) / 1000);
}
