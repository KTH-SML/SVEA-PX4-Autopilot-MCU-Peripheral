#include "encoder_module.h"

#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "peripheral_context.h"
#include "peripheral_topics.h"
#include "svea_common.h"
#include "svea_config.h"

#define ENCODER_WHEEL_LEFT 0
#define ENCODER_WHEEL_RIGHT 1

static encoder_module_t *s_isr_module = NULL;

/* 4x quadrature decoding lookup table.
 *
 * Index = (previous_state << 2) | new_state, where each state is the 2-bit
 * value (A << 1) | B. Valid single-channel transitions map to +1 (one
 * direction) or -1 (the other direction). "No change" and illegal
 * double-channel transitions map to 0 (the illegal ones are additionally
 * counted as rejected edges in the ISR).
 *
 * A nice property of this scheme: optical bounce on one channel produces
 * alternating +1/-1 deltas that cancel out exactly, so no time-based
 * debounce is needed. Bounce still costs interrupts, though: compare
 * physical_*_ticks against the decoded counts to see how much.
 */
static DRAM_ATTR const int8_t s_quad_table[16] = {
    0,  +1, -1, 0,  /* prev 00 -> 00, 01, 10, 11 */
    -1, 0,  0,  +1, /* prev 01 -> 00, 01, 10, 11 */
    +1, 0,  0,  -1, /* prev 10 -> 00, 01, 10, 11 */
    0,  -1, +1, 0,  /* prev 11 -> 00, 01, 10, 11 */
};

static float encoder_bandpass_speed_rad_s(float speed_rad_s, float speed_mps)
{
#if ENCODER_MIN_SPEED_FILTER_ENABLE
    const float abs_speed_mps = fabsf(speed_mps);

    if (abs_speed_mps < ENCODER_MIN_SPEED_MPS || abs_speed_mps > ENCODER_MAX_SPEED_MPS) {
        return 0.0f;
    }
#else
    (void)speed_mps;
#endif

    return speed_rad_s;
}

static void IRAM_ATTR encoder_gpio_isr(void *arg)
{
    encoder_module_t *module = s_isr_module;

    if (module == NULL) {
        return;
    }

    const bool is_left = ((uint32_t)(uintptr_t)arg == ENCODER_WHEEL_LEFT);

    /* Read both channels of the affected wheel. gpio_ll_get_level is an
     * inline register read, safe from an IRAM ISR (gpio_get_level is not
     * guaranteed to be).
     */
    const uint32_t a = gpio_ll_get_level(&GPIO, is_left ? ENCODER_LEFT_A_GPIO : ENCODER_RIGHT_A_GPIO);
    const uint32_t b = gpio_ll_get_level(&GPIO, is_left ? ENCODER_LEFT_B_GPIO : ENCODER_RIGHT_B_GPIO);
    const uint8_t new_state = (uint8_t)((a << 1) | b);

    portENTER_CRITICAL_ISR(&module->ticks_lock);
    if (is_left) {
        const uint8_t prev_state = module->left_quad_state;

        if (((prev_state ^ new_state) & 0x3) == 0x3) {
            /* Both channels changed at once: illegal, count and resync. */
            module->rejected_left_edges++;
        } else {
            int32_t delta = (int32_t)s_quad_table[(prev_state << 2) | new_state];
#if ENCODER_LEFT_INVERT
            delta = -delta;
#endif
            module->left_ticks += delta;
        }

        module->left_quad_state = new_state;
        module->physical_left_ticks++;
    } else {
        const uint8_t prev_state = module->right_quad_state;

        if (((prev_state ^ new_state) & 0x3) == 0x3) {
            module->rejected_right_edges++;
        } else {
            int32_t delta = (int32_t)s_quad_table[(prev_state << 2) | new_state];
#if ENCODER_RIGHT_INVERT
            delta = -delta;
#endif
            module->right_ticks += delta;
        }

        module->right_quad_state = new_state;
        module->physical_right_ticks++;
    }
    portEXIT_CRITICAL_ISR(&module->ticks_lock);
}

static void encoder_emulate_profile(float t_s, float *linear_mps, float *yaw_rate_rps)
{
    const float two_pi = 2.0f * (float)M_PI;
    *linear_mps = 0.70f * sinf(two_pi * 0.08f * t_s) + 0.18f * sinf(two_pi * 0.27f * t_s);
    *yaw_rate_rps = 0.85f * sinf(two_pi * 0.13f * t_s + 0.9f);
}

static void encoder_emulate_ticks(encoder_module_t *module, float meters_per_tick, float dt_s)
{
    const float t_s = (float)(peripheral_now_ms() - module->emulate_start_ms) * 0.001f;
    float linear_mps = 0.0f;
    float yaw_rate_rps = 0.0f;
    encoder_emulate_profile(t_s, &linear_mps, &yaw_rate_rps);

    const float left_mps = linear_mps + (yaw_rate_rps * ENCODER_WHEELBASE_M * 0.5f);
    const float right_mps = linear_mps - (yaw_rate_rps * ENCODER_WHEELBASE_M * 0.5f);

    const float left_tick_delta_f = (left_mps * dt_s) / (meters_per_tick * ENCODER_SPEED_SCALE) + module->emulate_left_frac_ticks;
    const float right_tick_delta_f = (right_mps * dt_s) / (meters_per_tick * ENCODER_SPEED_SCALE) + module->emulate_right_frac_ticks;

    const int32_t left_tick_delta = (int32_t)left_tick_delta_f;
    const int32_t right_tick_delta = (int32_t)right_tick_delta_f;

    module->emulate_left_frac_ticks = left_tick_delta_f - (float)left_tick_delta;
    module->emulate_right_frac_ticks = right_tick_delta_f - (float)right_tick_delta;

    portENTER_CRITICAL(&module->ticks_lock);
    module->left_ticks += left_tick_delta;
    module->right_ticks += right_tick_delta;
    portEXIT_CRITICAL(&module->ticks_lock);
}

/* Throw away everything the emulation profile injected, so the published
 * counters describe real wheel motion only. prev_*_ticks must be cleared in
 * the same breath: leaving it stale would make the very next publish window
 * see one enormous fake delta and feed it straight into the speed output.
 */
static void encoder_discard_emulated_ticks(encoder_module_t *module,
                                           int32_t *discarded_left,
                                           int32_t *discarded_right)
{
    portENTER_CRITICAL(&module->ticks_lock);
    *discarded_left = module->left_ticks;
    *discarded_right = module->right_ticks;
    module->left_ticks = 0;
    module->right_ticks = 0;
    portEXIT_CRITICAL(&module->ticks_lock);

    module->prev_left_ticks = 0;
    module->prev_right_ticks = 0;
}

static bool encoder_gpio_activity_detected(encoder_module_t *module)
{
    int32_t left_ticks = 0;
    int32_t right_ticks = 0;

    portENTER_CRITICAL(&module->ticks_lock);
    left_ticks = module->physical_left_ticks;
    right_ticks = module->physical_right_ticks;
    portEXIT_CRITICAL(&module->ticks_lock);

    const bool activity = (left_ticks != module->prev_activity_left_ticks) ||
                          (right_ticks != module->prev_activity_right_ticks);

    module->prev_activity_left_ticks = left_ticks;
    module->prev_activity_right_ticks = right_ticks;
    return activity;
}

static void encoder_debug_log(encoder_module_t *module,
                              uint32_t time_ms,
                              int32_t left_ticks,
                              int32_t right_ticks,
                              int32_t d_left,
                              int32_t d_right)
{
#if ENCODER_DEBUG_LOG_ENABLE
    if ((time_ms - module->last_debug_ms) < ENCODER_DEBUG_LOG_MS) {
        return;
    }

    int32_t physical_left_ticks = 0;
    int32_t physical_right_ticks = 0;
    uint32_t rejected_left_edges = 0;
    uint32_t rejected_right_edges = 0;

    portENTER_CRITICAL(&module->ticks_lock);
    physical_left_ticks = module->physical_left_ticks;
    physical_right_ticks = module->physical_right_ticks;
    rejected_left_edges = module->rejected_left_edges;
    rejected_right_edges = module->rejected_right_edges;
    portEXIT_CRITICAL(&module->ticks_lock);

    module->last_debug_ms = time_ms;

    ESP_LOGI(SVEA_TAG,
             "encoder debug: mode=%s force=%d gpio_LA=%d gpio_LB=%d gpio_RA=%d gpio_RB=%d "
             "physical_left=%ld physical_right=%ld rejected_left=%lu rejected_right=%lu "
             "publish_left=%ld publish_right=%ld d_left=%ld d_right=%ld",
             module->use_emulation ? "emulation" : "real",
             module->force_emulation ? 1 : 0,
             gpio_get_level(ENCODER_LEFT_A_GPIO),
             gpio_get_level(ENCODER_LEFT_B_GPIO),
             gpio_get_level(ENCODER_RIGHT_A_GPIO),
             gpio_get_level(ENCODER_RIGHT_B_GPIO),
             (long)physical_left_ticks,
             (long)physical_right_ticks,
             (unsigned long)rejected_left_edges,
             (unsigned long)rejected_right_edges,
             (long)left_ticks,
             (long)right_ticks,
             (long)d_left,
             (long)d_right);
#else
    (void)module;
    (void)time_ms;
    (void)left_ticks;
    (void)right_ticks;
    (void)d_left;
    (void)d_right;
#endif
}

static void encoder_publish_task(void *arg)
{
    encoder_module_t *module = (encoder_module_t *)arg;
    const float two_pi = 2.0f * (float)M_PI;
    const float wheel_circ = (float)M_PI * ENCODER_WHEEL_DIAM_M;
    const float meters_per_tick = wheel_circ / ENCODER_TICKS_PER_REV;
    const float dt_s = (float)ENCODER_PUBLISH_MS / 1000.0f;
    const float radians_per_tick = (two_pi / ENCODER_TICKS_PER_REV) * ENCODER_SPEED_SCALE;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_PUBLISH_MS));

        if (module->use_emulation) {
#if ENCODER_AUTO_SWITCH_TO_REAL_ENABLE
            if (!module->force_emulation && encoder_gpio_activity_detected(module)) {
                int32_t discarded_left = 0;
                int32_t discarded_right = 0;

                module->use_emulation = false;
                encoder_discard_emulated_ticks(module, &discarded_left, &discarded_right);

                ESP_LOGI(SVEA_TAG,
                         "encoder activity detected, switching from emulation to real GPIO inputs: "
                         "gpio_LA=%d gpio_LB=%d gpio_RA=%d gpio_RB=%d physical_left=%ld physical_right=%ld "
                         "discarded_left=%ld discarded_right=%ld",
                         gpio_get_level(ENCODER_LEFT_A_GPIO),
                         gpio_get_level(ENCODER_LEFT_B_GPIO),
                         gpio_get_level(ENCODER_RIGHT_A_GPIO),
                         gpio_get_level(ENCODER_RIGHT_B_GPIO),
                         (long)module->prev_activity_left_ticks,
                         (long)module->prev_activity_right_ticks,
                         (long)discarded_left,
                         (long)discarded_right);
            }
#endif
            if (module->use_emulation) {
                encoder_emulate_ticks(module, meters_per_tick, dt_s);
            }
        }

        int32_t left_ticks = 0;
        int32_t right_ticks = 0;

        portENTER_CRITICAL(&module->ticks_lock);
        left_ticks = module->left_ticks;
        right_ticks = module->right_ticks;
        portEXIT_CRITICAL(&module->ticks_lock);

        const int32_t d_left = left_ticks - module->prev_left_ticks;
        const int32_t d_right = right_ticks - module->prev_right_ticks;
        module->prev_left_ticks = left_ticks;
        module->prev_right_ticks = right_ticks;

        const float right_speed_rad_s = ((float)d_right * radians_per_tick) / dt_s;
        const float left_speed_rad_s = ((float)d_left * radians_per_tick) / dt_s;
        const float right_speed_mps = ((float)d_right * meters_per_tick * ENCODER_SPEED_SCALE) / dt_s;
        const float left_speed_mps = ((float)d_left * meters_per_tick * ENCODER_SPEED_SCALE) / dt_s;
        const float filtered_right_speed_rad_s = encoder_bandpass_speed_rad_s(right_speed_rad_s, right_speed_mps);
        const float filtered_left_speed_rad_s = encoder_bandpass_speed_rad_s(left_speed_rad_s, left_speed_mps);
        const float right_angle_rad = (float)right_ticks * radians_per_tick;
        const float left_angle_rad = (float)left_ticks * radians_per_tick;
        const uint32_t sequence = module->sequence++;
        const uint32_t time_ms = peripheral_now_ms();

        encoder_debug_log(module, time_ms, left_ticks, right_ticks, d_left, d_right);

        peripheral_topic_wheel_distance_t wheel_distance = {
            .sequence = sequence,
            .time_ms = time_ms,
            .left_distance_m = (float)left_ticks * meters_per_tick * ENCODER_SPEED_SCALE,
            .right_distance_m = (float)right_ticks * meters_per_tick * ENCODER_SPEED_SCALE,
        };

        peripheral_topic_wheel_encoders_t wheel_encoders = {
            .sequence = sequence,
            .time_ms = time_ms,
            .right_wheel_speed_rad_s = filtered_right_speed_rad_s,
            .left_wheel_speed_rad_s = filtered_left_speed_rad_s,
            .right_wheel_angle_rad = right_angle_rad,
            .left_wheel_angle_rad = left_angle_rad,
        };

        if (peripheral_topic_publish_wheel_distance(module->ctx, &wheel_distance) < 0 ||
            peripheral_topic_publish_wheel_encoders(module->ctx, &wheel_encoders) < 0) {
            ESP_LOGE(SVEA_TAG, "failed to publish encoder topics");
            abort();
        }
    }
}

void encoder_module_init(encoder_module_t *module, peripheral_context_t *ctx)
{
    if (module == NULL || ctx == NULL) {
        ESP_LOGE(SVEA_TAG, "encoder init null input");
        abort();
    }

    memset(module, 0, sizeof(*module));
    module->ctx = ctx;
    module->ticks_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    module->emulate_start_ms = peripheral_now_ms();
    module->force_emulation = (ENCODER_EMULATION_ENABLE != 0);
    module->use_emulation = module->force_emulation;

    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ENCODER_LEFT_A_GPIO) | (1ULL << ENCODER_LEFT_B_GPIO) |
                        (1ULL << ENCODER_RIGHT_A_GPIO) | (1ULL << ENCODER_RIGHT_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    SVEA_CHECK(gpio_config(&cfg));

    /* Seed the quadrature state machines from current pin levels BEFORE
     * enabling interrupts, so the first real edge decodes correctly.
     */
    module->left_quad_state =
        (uint8_t)((gpio_get_level(ENCODER_LEFT_A_GPIO) << 1) | gpio_get_level(ENCODER_LEFT_B_GPIO));
    module->right_quad_state =
        (uint8_t)((gpio_get_level(ENCODER_RIGHT_A_GPIO) << 1) | gpio_get_level(ENCODER_RIGHT_B_GPIO));

    SVEA_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    s_isr_module = module;
    /* Both channels of a wheel share one handler; the argument tells the ISR
     * which wheel fired, and it re-reads both channel levels itself.
     */
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_LEFT_A_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_WHEEL_LEFT));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_LEFT_B_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_WHEEL_LEFT));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_RIGHT_A_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_WHEEL_RIGHT));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_RIGHT_B_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_WHEEL_RIGHT));

    ESP_LOGI(SVEA_TAG,
             "encoder GPIO configured (quadrature): LA=%d LB=%d RA=%d RB=%d "
             "levels LA=%d LB=%d RA=%d RB=%d pull=up intr=anyedge "
             "ticks_per_rev=%.0f min_speed_mps=%.2f max_speed_mps=%.2f",
             ENCODER_LEFT_A_GPIO,
             ENCODER_LEFT_B_GPIO,
             ENCODER_RIGHT_A_GPIO,
             ENCODER_RIGHT_B_GPIO,
             gpio_get_level(ENCODER_LEFT_A_GPIO),
             gpio_get_level(ENCODER_LEFT_B_GPIO),
             gpio_get_level(ENCODER_RIGHT_A_GPIO),
             gpio_get_level(ENCODER_RIGHT_B_GPIO),
             (double)ENCODER_TICKS_PER_REV,
             (double)ENCODER_MIN_SPEED_MPS,
             (double)ENCODER_MAX_SPEED_MPS);

    if (!module->force_emulation) {
        bool activity = false;
        const int probe_loops = ENCODER_ACTIVITY_PROBE_MS / ENCODER_PUBLISH_MS;

        for (int i = 0; i < probe_loops; i++) {
            if (encoder_gpio_activity_detected(module)) {
                activity = true;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(ENCODER_PUBLISH_MS));
        }

        module->use_emulation = !activity;
    }

    ESP_LOGI(SVEA_TAG,
             "encoder GPIO ISR up: LA=%d LB=%d RA=%d RB=%d mode=%s",
             ENCODER_LEFT_A_GPIO,
             ENCODER_LEFT_B_GPIO,
             ENCODER_RIGHT_A_GPIO,
             ENCODER_RIGHT_B_GPIO,
             module->use_emulation ? "emulation" : "real");
}

void encoder_module_start(encoder_module_t *module)
{
    if (module == NULL || module->ctx == NULL) {
        ESP_LOGE(SVEA_TAG, "encoder start invalid module");
        abort();
    }

    BaseType_t rc = xTaskCreate(encoder_publish_task, "encoder_pub", 4096, module, 12, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(SVEA_TAG, "xTaskCreate encoder_pub failed");
        abort();
    }
}