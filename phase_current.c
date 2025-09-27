#include "phase_current.h"

volatile uint16_t adc_raw_a = 0, adc_raw_b = 0, adc_raw_c = 0;
float offA = 0.0f, offB = 0.0f, offC = 0.0f;

void phase_offsets_calibrate_idle(void) {
  const int N = 1024;
  uint32_t sa = 0, sb = 0, sc = 0;
  for (int i = 0; i < N; i++) {
    sa += adc_raw_a;
    sb += adc_raw_b;
    sc += adc_raw_c;
  }
  offA = (float)(sa / N);
  offB = (float)(sb / N);
  offC = (float)(sc / N);
}
