#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "peripheral_context.h"

typedef struct {
    peripheral_context_t *ctx;
    portMUX_TYPE ticks_lock;
    /* Signed net tick counts (quadrature: forward increments, reverse
     * decrements). Consumed by the publish task.
     */
    volatile int32_t left_ticks;
    volatile int32_t right_ticks;
    /* Raw "ISR fired" counters. Used to detect real hardware activity
     * (emulation vs real switching) and, in the debug log, to gauge how much
     * of the interrupt load is bounce: a healthy encoder has
     * physical_*_ticks only modestly above the decoded tick count.
     */
    volatile int32_t physical_left_ticks;
    volatile int32_t physical_right_ticks;
    /* Edges where both A and B appeared to change at once (illegal quadrature
     * transition, usually electrical noise).
     */
    volatile uint32_t rejected_left_edges;
    volatile uint32_t rejected_right_edges;
    /* Last known 2-bit quadrature state per wheel: (A << 1) | B. */
    volatile uint8_t left_quad_state;
    volatile uint8_t right_quad_state;
    uint32_t emulate_start_ms;
    uint32_t last_debug_ms;
    uint32_t sequence;
    int32_t prev_left_ticks;
    int32_t prev_right_ticks;
    int32_t prev_activity_left_ticks;
    int32_t prev_activity_right_ticks;
    float emulate_left_frac_ticks;
    float emulate_right_frac_ticks;
    bool use_emulation;
    bool force_emulation;
} encoder_module_t;

void encoder_module_init(encoder_module_t *module, peripheral_context_t *ctx);
void encoder_module_start(encoder_module_t *module);