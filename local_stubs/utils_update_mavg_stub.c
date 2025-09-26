// local_stubs/utils_update_mavg_stub.c
#include <stdint.h>

/*
 * Weak fallback for builds that don't provide update_moving_average().
 * If a real implementation exists elsewhere, it will override this.
 * Safe passthrough (no smoothing) — keeps motor logic stable.
 */
__attribute__((weak))
uint32_t update_moving_average(uint32_t new_value)
{
    return new_value;
}
