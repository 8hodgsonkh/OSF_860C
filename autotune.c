// Minimal Auto-Tune module: sweeps duty bins and sampling offsets, scores KCL residual
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include "main.h"
#include "motor.h"
#include "current_sense.h"
#include "autotune.h"

// Safety and sweep parameters
#define AT_NUM_BINS        5
#define AT_NUM_OFFSETS     5
static const uint8_t s_bins_pct[AT_NUM_BINS] = {25, 45, 65, 80, 90};
// microseconds; convert to ticks using 4 us per tick base
static const uint16_t s_offsets_us[AT_NUM_OFFSETS] = {200, 400, 600, 800, 1000};

// Control timing: main loop calls us every ~25 ms
#define AT_PROGRESS_PERIOD_TICKS 8   // ~200 ms (8 * 25 ms)
#define AT_SAMPLES_PER_CANDIDATE 256 // ~6.4 s @25 ms/sample

typedef struct {
    bool running;
    bool aborted;
    uint8_t bin_idx;        // 0..AT_NUM_BINS-1
    uint8_t off_idx;        // 0..AT_NUM_OFFSETS-1
    uint16_t samples_left;  // countdown per candidate
    uint64_t score_accum;   // sum of residual^2
    // best offset tracking per bin
    uint32_t best_score;
    uint16_t best_us;
    // UI/report pacing
    uint8_t progress_tick;
    // saved state for restoration
    uint8_t prev_g_duty;
    uint8_t prev_fw_enabled;
} at_state_t;

static at_state_t g_at;
static uint8_t g_at_status = 0; // 0=idle,1=running,2=done,3=aborted

// Simple ASCII line TX helper provided by ebike_app.c
extern void ebike_uart_send_ascii_line(const char *s);

static inline uint16_t at_us_to_ticks(uint16_t us)
{
    // 1 tick = 4 us (HALL_SPEED_TIMER base); guard divide
    return (uint16_t)((us + 3) / 4);
}

static void at_emitf(const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    ebike_uart_send_ascii_line(buf);
}

static void at_apply_bin(uint8_t pct)
{
    // Target gentle duty. PWM_DUTY_CYCLE_MAX is 255.
    uint16_t val = ((uint16_t)PWM_DUTY_CYCLE_MAX * (uint16_t)pct) / 100u;
    if (val > PWM_DUTY_CYCLE_MAX) val = PWM_DUTY_CYCLE_MAX;
    ui8_g_duty_cycle = (uint8_t)val;
}

extern volatile uint8_t ui8_field_weakening_enabled; // from motor.c
extern uint16_t ui16_motor_speed_erps;               // from ebike_app.c

static void at_restore_user_state(void)
{
    ui8_g_duty_cycle = g_at.prev_g_duty;
    // field weakening off/on restore
    ui8_field_weakening_enabled = g_at.prev_fw_enabled;
}

void autotune_init(void)
{
    g_at.running = false;
    g_at.aborted = false;
    g_at_status = 0;
}

void autotune_start(void)
{
    if (g_at.running) return;
    g_at.running = true;
    g_at.aborted = false;
    g_at.bin_idx = 0;
    g_at.off_idx = 0;
    g_at.samples_left = AT_SAMPLES_PER_CANDIDATE;
    g_at.score_accum = 0;
    g_at.best_score = 0xFFFFFFFFu;
    g_at.best_us = s_offsets_us[0];
    g_at.progress_tick = 0;

    // Save and force safe test envelope
    g_at.prev_g_duty = ui8_g_duty_cycle;
    g_at.prev_fw_enabled = ui8_field_weakening_enabled;
    ui8_field_weakening_enabled = 0; // FW off
    at_apply_bin(s_bins_pct[g_at.bin_idx]);

    at_emitf("ATSTAT,%u,%u,0", (unsigned)g_at.bin_idx, (unsigned)s_offsets_us[g_at.off_idx]);
    g_at_status = 1; // running
}

void autotune_abort(void)
{
    if (!g_at.running) return;
    g_at.aborted = true;
    g_at.running = false;
    at_restore_user_state();
    at_emitf("ATABRT");
    g_at_status = 3; // aborted
}

bool autotune_is_running(void)
{
    return g_at.running;
}

static void at_next_candidate_or_bin(void)
{
    // Finish candidate; update best if needed
    if (g_at.score_accum < g_at.best_score) {
        g_at.best_score = (uint32_t)g_at.score_accum;
        g_at.best_us = s_offsets_us[g_at.off_idx];
    }

    // Advance offset
    g_at.off_idx++;
    if (g_at.off_idx >= AT_NUM_OFFSETS) {
        // Bin complete
        at_emitf("ATDONE,%u,%u", (unsigned)g_at.bin_idx, (unsigned)g_at.best_us);

        // Next bin
        g_at.bin_idx++;
        if (g_at.bin_idx >= AT_NUM_BINS) {
            // All done
            at_emitf("ATEND,0");
            at_restore_user_state();
            g_at.running = false;
            g_at_status = 2; // done
            return;
        }
        // Prepare next bin
        g_at.off_idx = 0;
        g_at.best_score = 0xFFFFFFFFu;
        g_at.best_us = s_offsets_us[0];
        at_apply_bin(s_bins_pct[g_at.bin_idx]);
    }

    // Reset for next candidate
    g_at.samples_left = AT_SAMPLES_PER_CANDIDATE;
    g_at.score_accum = 0;
    g_at.progress_tick = 0;
}

// Basic safety clamp: cap ERPS and keep duty modest
static void at_enforce_safety(void)
{
    if (ui16_motor_speed_erps > 200u) {
        // reduce duty mildly to keep wheel-in-air speed under control
        if (ui8_g_duty_cycle > 5) ui8_g_duty_cycle -= 5;
    }
    // Keep phase/battery current modest (~3 A). Legacy 10-bit ~0.15 A/step => ~20 steps. Use margin 24.
    {
        uint16_t mag10 = 0;
#if (TSDZ8_VADC_QUEUE_ISR)
        mag10 = cs_get_latest_phase10();
#else
        mag10 = cs_update_and_get_phase10();
#endif
        if (mag10 > 24u) {
            if (ui8_g_duty_cycle > 0) ui8_g_duty_cycle--;
        }
    }
}

bool autotune_step(void)
{
    if (!g_at.running) return false;
    if (g_at.aborted) return false;

    // Apply candidate offset in ticks (future: hook into cs_schedule if exposed)
    (void)at_us_to_ticks(s_offsets_us[g_at.off_idx]);

    // Take one sample of fused phase currents and accumulate residual^2
    cs_phases_t ph = {0};
    (void)cs_get_last_fused(&ph);
    int32_t residual = (int32_t)ph.iu + (int32_t)ph.iv + (int32_t)ph.iw;
    g_at.score_accum += (uint64_t)((int64_t)residual * (int64_t)residual);
    if (g_at.samples_left > 0) g_at.samples_left--;

    at_enforce_safety();

    // Progress pacing ~200 ms
    if (++g_at.progress_tick >= AT_PROGRESS_PERIOD_TICKS) {
        g_at.progress_tick = 0;
        at_emitf("ATSTAT,%u,%u,%lu",
                 (unsigned)g_at.bin_idx,
                 (unsigned)s_offsets_us[g_at.off_idx],
                 (unsigned long)(g_at.score_accum & 0xFFFFFFFFu));
    }

    if (g_at.samples_left == 0) {
        at_next_candidate_or_bin();
    }

    return g_at.running;
}

uint8_t autotune_status(void)
{
    return g_at_status;
}
