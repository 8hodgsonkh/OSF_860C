#pragma once
#include <stdint.h>
#include "foc_config.h"

extern volatile uint16_t adc_raw_a, adc_raw_b, adc_raw_c; // written in PWM-center ISR
extern float offA, offB, offC;

static inline void phase_to_amps(float *ia, float *ib, float *ic) {
  *ia = ((float)adc_raw_a - offA) * PHASE_AMPS_PER_COUNT;
  *ib = ((float)adc_raw_b - offB) * PHASE_AMPS_PER_COUNT;
  *ic = ((float)adc_raw_c - offC) * PHASE_AMPS_PER_COUNT;
}

void phase_offsets_calibrate_idle(void); // average 1024 samples with PWM off
