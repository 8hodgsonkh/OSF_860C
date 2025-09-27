#pragma once
#include <math.h>

static inline void clarke(float ia, float ib, float ic, float *a, float *b) {
  (void)ic;
  *a = ia;
  *b = (ia + 2.0f * ib) * 0.57735026919f; // 1/sqrt(3)
}

static inline void park(float a, float b, float s, float c, float *d, float *q) {
  *d =  a * c + b * s;
  *q = -a * s + b * c;
}

static inline void inv_park(float d, float q, float s, float c, float *a, float *b) {
  *a = d * c - q * s;
  *b = d * s + q * c;
}
