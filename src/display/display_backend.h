#pragma once
#include <stdint.h>
#include <stdbool.h>

// ---- Build-time selection ---------------------------------------------------
#define DISPLAY_BACKEND_860C   0
#define DISPLAY_BACKEND_EKD01  1

#ifndef DISPLAY_BACKEND
#define DISPLAY_BACKEND DISPLAY_BACKEND_860C
#endif

// ---- Config passed at init --------------------------------------------------
typedef struct {
  uint32_t uart_port;        // e.g., 1
  uint32_t uart_baud;        // e.g., 19200
  bool     logic_is_5v;      // if true, expect level shifter on RX
  uint16_t tx_period_ms;     // cadence for TX frames
  int8_t   pin_key;          // -1 if unused
  int8_t   pin_bl;           // -1 if unused
} display_backend_config_t;

// ---- Inputs sampled periodically (controller -> display) --------------------
typedef struct {
  uint16_t battery_mv;       // millivolts
  uint16_t speed_cmh;        // cm/h (12345 = 12.345 km/h)
  uint16_t motor_amps_x5;    // A * 5 units
  uint16_t duty_pct_x10;     // duty * 10
  uint8_t  assist_level;     // 0..9
  bool     lights_on;
  uint8_t  system_state;     // error/state code
  uint16_t fault_flags;      // bitfield (implementation specific)
  uint8_t  soc_pct;          // 0..100, else 255 if unknown
  bool     overvoltage;      // true if OV condition
} display_backend_inputs_t;

// ---- Backend API ------------------------------------------------------------
// Call init once early with UART + timing config.
// Call update (or update_10ms alias) from a periodic scheduler (>=10ms cadence).
void display_backend_init(const display_backend_config_t *cfg);
void display_backend_update(const display_backend_inputs_t *in);

// Compatibility alias (EKD01/shim define a macro; keeping prototype allows static analysis)
// Some code may prefer explicit 10ms naming; underlying implementation identical.
static inline void display_backend_update_10ms(const display_backend_inputs_t *in){ display_backend_update(in); }

// Optional helpers (EKD01 implements; 860C may stub)
uint8_t display_backend_map_error(uint8_t system_state);
uint8_t display_backend_select_function_code(uint8_t menu_index);
uint8_t display_backend_battery_bar_state(uint8_t soc_pct, bool overvoltage);
