#pragma once
#include <stdbool.h>
#include <stdint.h>

// Runtime Quiet-Window Auto-Tune (motor side)
// API
void autotune_init(void);
void autotune_start(void);
void autotune_abort(void);
bool autotune_is_running(void);
// Call every 25 ms control tick when running; returns false when finished
bool autotune_step(void);

// Compact status for display: 0=idle, 1=running, 2=done, 3=aborted
uint8_t autotune_status(void);
