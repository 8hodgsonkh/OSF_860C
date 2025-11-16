#pragma once
#include <stdbool.h>

// EKD01 defaults overlay
// Apply EKD01-specific runtime defaults after configuration/EEPROM load
// but before the display/backend begins exchanging data. If 'persist' is true,
// settings may be saved to non-volatile storage (not implemented here).
// Behavior can be locked by defining EKD01_LOCK_DEFAULTS at build time.
void ekd01_defaults_apply(bool persist);
