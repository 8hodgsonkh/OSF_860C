# EKD01 Backend – Integration Contract

Status: INITIAL DRAFT (placeholders marked TODO where source symbol uncertain)

## Build Selection
- `DISPLAY_BACKEND=0` (default): 860C legacy path, shim functions compile as no-ops; binary intended to remain bit-for-bit identical.
- `DISPLAY_BACKEND=1`: EKD01 UART backend enabled; source `src/display/display_backend_ekd01.c` compiled.
- Add `LOG_EKD01=1` via `CFLAGS` to enable lightweight SEGGER RTT debug messages.

## API (`src/display/display_backend.h`)
```
void display_backend_init(const display_backend_config_t* cfg);
void display_backend_update(const display_backend_inputs_t* in); // alias: display_backend_update_10ms
uint8_t display_backend_map_error(uint8_t system_state);
uint8_t display_backend_select_function_code(uint8_t menu_index);
uint8_t display_backend_battery_bar_state(uint8_t soc_pct, bool overvoltage);
```
`display_backend_update()` must be called at >=10ms cadence (recommended 10–25ms). TX frame emission internally rate-limited to `cfg->tx_period_ms` (default 50ms).

## Configuration
`display_backend_config_t` fields:
- `uart_port` (numeric logical port ID; platform provides `uart_open/uart_write/uart_read`)
- `uart_baud` (default 19200 8N1)
- `logic_is_5v` (level-shift hint; informational only)
- `tx_period_ms` (default 50ms)
- `pin_key`, `pin_bl` (optional backlight / key if needed; currently unused)

## Telemetry Inputs (`display_backend_inputs_t`)
Populate before each update call:
- `battery_mv`: from `ui16_battery_voltage_filtered_x1000` (value in mV) – cast to `uint16_t` if range fits; TODO: confirm upper bound.
- `speed_cmh`: wheel speed in cm/h. Source candidate: `(ui16_wheel_speed_x10 * 1000u)` approximates km/h*1000 -> cm/h; TODO: verify precise conversion.
- `motor_amps_x5`: from filtered current: `ui16_battery_current_filtered_x5` or `ui16_motor_current_filtered_x5` depending on requirement.
- `duty_pct_x10`: derive from duty cycle raw (`ui8_g_duty_cycle`) → `(ui8_g_duty_cycle * 1000u / 255u)` then divide by 100 for pct*10; TODO: validate scaling.
- `assist_level`: map from active mode level parameter (`ui8_riding_mode_parameter`?) or display-provided level flag; TODO: confirm symbol.
- `lights_on`: from `ui8_lights_state` (non-zero true).
- `system_state`: use `ui8_m_system_state` (error bitfield).
- `fault_flags`: reuse `ui8_m_system_state` or expanded mask for additional conditions; TODO: refine mapping.
- `soc_pct`: if SOC calculation present (search for `read_battery_soc()`); else set 255 (unknown).
- `overvoltage`: derive from dedicated status (not found) or compare `ui16_adc_voltage` against threshold; TODO.

Where a symbol is not yet confirmed, leave TODO in calling site; do not invent conversions.

## UART Protocol (Draft)
TX frame (controller → display):
```
[0] 0x55  (header)
[1] LEN (10)
[2] battery decivolts (battery_mv / 100)
[3] speed ~0.1 km/h (speed_cmh / 100)
[4] assist level (0..15)
[5] flags: bit0 lights, bit1 overvoltage, bit2 any fault
[6] motor current (low byte of motor_amps_x5)
[7] duty cycle % (duty_pct_x10 / 10)
[8] SOC (0..100 or 0xFF unknown)
[9] XOR checksum over bytes [0..8]
```
RX frame (display → controller, optional):
```
[0] 0xAA
[1] LEN (<=12)
[2] buttons bitmap (TODO map)
[3] assist delta (optional)
[4] lights toggle flag
[...] payload TBD
[last] XOR checksum over preceding bytes
```
Invalid headers/lengths/checksums are counted (`rx_bad_hdr`, `rx_bad_len`, `rx_bad_csum`) and optionally logged every 256 frames if `LOG_EKD01`.

## Logging (Optional)
Define `LOG_EKD01` (e.g. `make DISPLAY_BACKEND=1 CFLAGS+='-DLOG_EKD01'`) to enable:
- Init line: port, baud, period.
- Periodic RX stats.

## Integration Steps
1. Ensure Makefile includes conditional source:
```
DISPLAY_BACKEND ?= 0
CFLAGS += -DDISPLAY_BACKEND=$(DISPLAY_BACKEND)
SOURCES += src/display/display_backend_shim.c
ifeq ($(DISPLAY_BACKEND),1)
SOURCES += src/display/display_backend_ekd01.c
endif
```
2. Provide a scheduler call (10–25ms cadence). Example snippet (guarded to avoid baseline binary drift):
```c
#if DISPLAY_BACKEND == DISPLAY_BACKEND_EKD01
  static uint16_t last_disp_ticks;
  uint16_t now = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
  uint16_t delta = now - last_disp_ticks; // 4us ticks
  if(delta > 2500){ // ~10ms
      last_disp_ticks = now;
      display_backend_inputs_t in = {0};
      // TODO: map fields from globals (see Telemetry Inputs section)
      display_backend_update_10ms(&in);
  }
#endif
```
3. Validate baseline build unchanged (`DISPLAY_BACKEND=0`).
4. Run EKD01 build and probe UART at 19200 baud; expect 10-byte frames every ~50ms.

## Test Checklist
- Baseline (860C) build boots; no EKD01 symbols in map; binary size unchanged.
- EKD01 build emits TX frames (scope / logic analyzer: header 0x55, stable period).
- Disconnect / noise: RX stats counters increment; no blocking behavior observed.
- Optional button frame: craft RX with header 0xAA, valid checksum; handler TODO location reached.
- SOC unknown (255) produces 0xFF in byte 8.
- Overvoltage flag sets bit1 in flags and battery bar helper returns 0xF0.

## Future TODOs
- Precise speed scaling (confirm wheel speed conversion path).
- Assist level symbol / mapping table.
- Fault flag granularity vs aggregate error code.
- RX button / lights toggle dispatch integration.
- SOC calculation hookup (read_battery_soc()).

---
Generated: 2025-11-10
