// current_sense.h
// Robust per-phase current sensing with KCL fusion for TSDZ8 (XMC13)

#pragma once

#include <stdint.h>

// Public API:
// - cs_init: call once after VADC is initialized
// - cs_update_and_get_phase10: read phase ADCs, fuse using KCL, return legacy 10-bit magnitude for protection

void cs_init(void);

// Returns phase current magnitude in legacy 10-bit steps (~0.15 A/step)
uint16_t cs_update_and_get_phase10(void);

// Optional: cached latest magnitude (valid if TSDZ8_VADC_QUEUE_ISR=1 and ISR wired)
#if (TSDZ8_VADC_QUEUE_ISR)
uint16_t cs_get_latest_phase10(void);
// Returns the age (in microseconds) since the last VADC ISR updated the cache
uint16_t cs_get_irq_age_us(void);
// Increment and fetch/reset a stale-cache counter for diagnostics
void cs_note_stale(void);
uint32_t cs_get_and_clear_stale_count(void);
#endif

// Optional: expose last fused per-phase signed counts (12-bit centered around bias)
typedef struct {
    int16_t iu; // signed 12-bit centered (counts)
    int16_t iv;
    int16_t iw;
} cs_phases_t;

// Fill out with last fused values; returns 0 on success
int cs_get_last_fused(cs_phases_t* out);

// Schedule sampling window and (optionally) dynamic alias selection.
// For now this sets up timing only if needed; alias mapping remains static (U/W on G1, V on G0).
// Call once per PWM period from the compute ISR before the shadow transfer.
void cs_schedule(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w, uint16_t period_ticks);
