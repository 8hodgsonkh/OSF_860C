// 860C legacy shim – provides stable symbols without altering existing logic paths.
// When DISPLAY_BACKEND==DISPLAY_BACKEND_860C these remain lightweight no-ops so the
// resulting binary should be bit-for-bit identical (provided optimization/link order stable).

#include "display_backend.h"
#if DISPLAY_BACKEND == DISPLAY_BACKEND_860C

void display_backend_init(const display_backend_config_t *cfg){ (void)cfg; }

void display_backend_update(const display_backend_inputs_t *in){ (void)in; }

uint8_t display_backend_map_error(uint8_t system_state){ return system_state; }

uint8_t display_backend_select_function_code(uint8_t menu_index){ return menu_index; }

uint8_t display_backend_battery_bar_state(uint8_t soc_pct, bool overvoltage){ (void)overvoltage; return (uint8_t)(soc_pct / 10); }

// Alias to satisfy emerging callsites expecting a 10ms tick name
#define display_backend_update_10ms display_backend_update

#endif // DISPLAY_BACKEND_860C
