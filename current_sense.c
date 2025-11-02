// current_sense.c
// Robust per-phase current sensing with KCL fusion for TSDZ8 (XMC13)

#include "current_sense.h"
#include "adc.h"
#include "main.h"
#include <stddef.h>

// Use existing motor control globals to gate bias updates
extern volatile uint8_t ui8_g_duty_cycle;   // duty 0..255 (from motor.c)
extern uint8_t ui8_motor_enabled;           // 0/1 (from ebike_app.c)

// Local persistent state
static uint16_t s_bias_u = (1U << 11); // 12-bit mid-scale
static uint16_t s_bias_v = (1U << 11);
static uint16_t s_bias_w = (1U << 11);
static cs_phases_t s_last_fused = {0,0,0};
static volatile uint16_t s_phase10_latest = 0; // cached latest 10-bit magnitude
// Timestamp of last ISR fusion (HALL timer ticks @ HALL_COUNTER_FREQ)
static volatile uint16_t s_fuse_irq_tick = 0;
// Diagnostics: count how many times cache was deemed stale
static volatile uint32_t s_stale_count = 0;

void cs_init(void) {
    s_bias_u = (1U << 11);
    s_bias_v = (1U << 11);
    s_bias_w = (1U << 11);
    s_last_fused.iu = 0;
    s_last_fused.iv = 0;
    s_last_fused.iw = 0;
}

// Fixed-point scale: convert 12-bit counts to legacy 10-bit (~0.15 A/step)
// steps10 ≈ (counts12 * 39) >> 11, with rounding bias
static inline uint16_t counts12_to_steps10(uint16_t amax_counts) {
    return (uint16_t)(((uint32_t)amax_counts * 39U + (1U << 10)) >> 11);
}

// Simple, robust KCL fusion:
// 1) Read U,V,W raw 12-bit (synchronized by existing VADC config)
// 2) Maintain biases when disabled or duty=0
// 3) Compute signed iu,iv,iw (counts)
// 4) Reconstruct each phase via KCL from the other two
// 5) Compute reconstruction errors; replace the worst (highest error) with its KCL value
// 6) Publish max(|iU|,|iV|,|iW|) as legacy 10-bit magnitude
uint16_t cs_update_and_get_phase10(void) {
    // Read synchronized 12-bit ADC results for phases U, V, W
    uint16_t raw_u = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF;
    uint16_t raw_v = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF;
    uint16_t raw_w = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF;

    // Bias calibration at idle (simple LPF when motor disabled or duty=0)
    const uint8_t bias_lpf_shift = 3; // 1/8 LPF
    if ((ui8_motor_enabled == 0u) || (ui8_g_duty_cycle == 0u)) {
        s_bias_u = (uint16_t)(((uint32_t)s_bias_u * ((1U << bias_lpf_shift) - 1U) + raw_u) >> bias_lpf_shift);
        s_bias_v = (uint16_t)(((uint32_t)s_bias_v * ((1U << bias_lpf_shift) - 1U) + raw_v) >> bias_lpf_shift);
        s_bias_w = (uint16_t)(((uint32_t)s_bias_w * ((1U << bias_lpf_shift) - 1U) + raw_w) >> bias_lpf_shift);
    }

    // Signed deltas around bias
    int16_t iu = (int16_t)raw_u - (int16_t)s_bias_u;
    int16_t iv = (int16_t)raw_v - (int16_t)s_bias_v;
    int16_t iw = (int16_t)raw_w - (int16_t)s_bias_w;

    // Reconstruct each via KCL using the other two
    // iU_kcl = -(iV + iW), etc.
    int16_t iu_kcl = (int16_t)(-(int32_t)iv - (int32_t)iw);
    int16_t iv_kcl = (int16_t)(-(int32_t)iu - (int32_t)iw);
    int16_t iw_kcl = (int16_t)(-(int32_t)iu - (int32_t)iv);

    // Errors (absolute)
    uint16_t eu = (uint16_t)((iu > iu_kcl) ? (iu - iu_kcl) : (iu_kcl - iu));
    uint16_t ev = (uint16_t)((iv > iv_kcl) ? (iv - iv_kcl) : (iv_kcl - iv));
    uint16_t ew = (uint16_t)((iw > iw_kcl) ? (iw - iw_kcl) : (iw_kcl - iw));

    // Select the most suspect measurement (max error) and replace it with KCL fusion
    if ((eu >= ev) && (eu >= ew)) {
        iu = iu_kcl;
    } else if (ev >= ew) {
        iv = iv_kcl;
    } else {
        iw = iw_kcl;
    }

    // Save fused values for optional inspection
    s_last_fused.iu = iu;
    s_last_fused.iv = iv;
    s_last_fused.iw = iw;

    // Protection magnitude: conservative peak of absolute values
    uint16_t au = (iu < 0) ? (uint16_t)(-iu) : (uint16_t)iu;
    uint16_t av = (iv < 0) ? (uint16_t)(-iv) : (uint16_t)iv;
    uint16_t aw = (iw < 0) ? (uint16_t)(-iw) : (uint16_t)iw;
    uint16_t amax = au;
    if (av > amax) amax = av;
    if (aw > amax) amax = aw;

    uint16_t mag10 = counts12_to_steps10(amax);
    s_phase10_latest = mag10;
    return mag10;
}

int cs_get_last_fused(cs_phases_t* out) {
    if (out == NULL) return -1;
    out->iu = s_last_fused.iu;
    out->iv = s_last_fused.iv;
    out->iw = s_last_fused.iw;
    return 0;
}

#if (TSDZ8_VADC_QUEUE_ISR)
// Optional: IRQ handler to update cache as soon as VADC results are ready.
// Note: Ensure proper VADC event routing to this SR in adc init if enabling this path.
__RAM_FUNC void VADC0_G1_1_IRQHandler(void)
{
    // Capture timing first (free-running hall/speed timer, 250 kHz -> 4 us/tick)
    s_fuse_irq_tick = (uint16_t)XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    // Minimal work: run fusion and update cache. Results are already in result regs.
    (void)cs_update_and_get_phase10();
    // Clear pending if required by reading flags; many XMC libs auto-clear on RES read.
}

// Accessor for last magnitude
uint16_t cs_get_latest_phase10(void)
{
    return s_phase10_latest;
}

// Compute time since last VADC ISR (in microseconds), based on hall timer ticks
uint16_t cs_get_irq_age_us(void)
{
    // Read current tick (same timer used throughout motor timing)
    uint16_t now = (uint16_t)XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    // Unsigned wrap-safe delta
    uint16_t dt_ticks = (uint16_t)(now - s_fuse_irq_tick);
    // Convert to microseconds. HALL_COUNTER_FREQ is 250000 Hz (4 us per tick).
    // Use 64-bit to avoid overflow if constants change.
    uint32_t us = (uint32_t)((((uint64_t)dt_ticks) * 1000000ULL) / (uint64_t)HALL_COUNTER_FREQ);
    if (us > 0xFFFFu) us = 0xFFFFu;
    return (uint16_t)us;
}

void cs_note_stale(void)
{
    s_stale_count++;
}

uint32_t cs_get_and_clear_stale_count(void)
{
    uint32_t v = s_stale_count;
    s_stale_count = 0;
    return v;
}
#endif

// Per-period scheduling: compute a trigger tick for ADC sampling near plateau.
// Note: The CCU8->VADC trigger is already wired via external trigger in the queue config
// and currently occurs around slice-3 mid-point. This function is a placeholder to allow
// future dynamic placement (deadtime+recovery). For now it is a no-op to keep deterministic
// behavior while we scope and tune.
void cs_schedule(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w, uint16_t period_ticks)
{
    (void)duty_u; (void)duty_v; (void)duty_w; (void)period_ticks;
    // TODO: choose two lowest-duty legs and shift trigger into the quiet plateau.
    // Requires board-specific CCU8 SR routing and deadtime knowledge.
}
