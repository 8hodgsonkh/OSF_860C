# Serial Link Alignment Report (860C ↔ Motor Firmware)

This report captures the current UART settings, handshake sequence, packet shapes, and checks needed to get the 860C display past the splash screen with the motor firmware. No telemetry size increases were introduced.

## UART parameters and pins

- Baud rate: 115200 8N1 (updated from 19200)
  - Config: `bsps/TARGET_tsdz8_for_GPIO_TEST/config/GeneratedSource/cycfg_peripherals.c`
    - `CYBSP_DEBUG_UART_config.baudrate = 115200UL`
    - Dividers set with `XMC_USIC_CH_SetFractionalDivider(..., 3)` and `XMC_USIC_CH_SetBaudrateDivider(..., 1, ...)` for the updated rate
  - Oversampling: 16
  - Parity: none
- TX/RX pins (board BSP):
  - TX: P0.6 (`CYBSP_DEBUG_UART_TX_PORT`/`PIN` in `cycfg_pins.h`)
  - RX: P0.7 (`CYBSP_DEBUG_UART_RX_PORT`/`PIN` in `cycfg_pins.h`)
- FIFOs: 32 words for both RX and TX (see `cycfg_peripherals.h`)

## Handshake sequence and behavior

- Motor sends ALIVE while in `MOTOR_INIT_STATE_RESET` (see `ebike_app.c`, communications_controller())
- Display should send, in order:
  1) CONFIGURATIONS to provide wheel perimeter, voltage cutoffs, etc.
  2) Periodic frame thereafter
- Motor responds to incoming frame type via `communications_process_packages()` with:
  - ALIVE: header only (no extra payload)
  - STATUS: one byte payload at index 3 carrying `ui8_m_motor_init_status`
  - CONFIGURATIONS: parses config, disables PWM during reconfig, sets init state/status, then returns header-only response
  - PERIODIC: returns 24-byte payload with live telemetry
- Comms watchdog: if `ui8_comm_error_counter > 30` (~>750 ms), PWM disabled and ERROR_FATAL set (masked during the 10 s grace window).

Minimal diagnostics added (no payload growth):
- Every ~250 ms a SEGGER RTT line logs: init_state, init_status, comm_error_counter, last_rx_frame. See `communications_controller()` in `ebike_app.c`.

## Packet format and indices (unchanged)

- Header: `0x43` at index 0
- Length: index 1 (`len` includes header and frame type, excludes the 2-byte CRC)
- Frame type: index 2
- CRC16: appended as little-endian after `len` bytes; init 0xFFFF; same `crc16()` as existing implementation

PERIODIC payload mapping in `ebike_app.c` (indices relative to tx buffer):
- [3..4] battery voltage (10-bit packed)
- [5] battery current x5
- [6..7] wheel speed x10
- [8] brake + hall + speed/power flags
- [9] throttle ADC (10-bit >> 2)
- [10] temp or throttle adjusted
- [11] torque ADC (low 8 bits); hi bits share [7]
- [12..13] pedal torque delta (no boost)
- [14] cadence RPM
- [15] PWM duty %
- [16..17] ERPS
- [18] foc angle raw
- [19] system state (fatal masked during grace)
- [20] motor current x5
- [21..23] wheel tick counter (24-bit)
- [24..25] pedal torque delta (boost)
- [26] battery current (10-bit filtered)

Optional debug byte append remains disabled:
- Macro: `TSDZ8_APPEND_DEBUG_PERIODIC` is off by default; no size growth

## CRC details

- Function: `crc16()` with 0xFFFF seed, identical to current code
- Length covered: all bytes from index 0 to (len-1) inclusive; CRC bytes appended after

## Changes made in this alignment pass

- Raise UART baud to 115200 8N1
  - `cycfg_peripherals.c`: baud to 115200; set dividers accordingly
  - `design.modus`: baud param to 115200 (for consistency)
  - EKD01 backend default baud set to 115200 to match platform
- Diagnostics: SEGGER RTT periodic log inside `communications_controller()` (no protocol change)
- No change to payload size or field indices; fatal masking during grace already present and retained

## Bench checklist

1) Wiring: Confirm TX=P0.6 and RX=P0.7 between MCU and 860C; common ground.
2) Serial: Verify baud 115200 8N1 on both sides (display firmware must match).
3) Power-up: Observe ALIVE frames from motor; display should progress past splash once CONFIGURATIONS is sent.
4) CONFIGURATIONS: Ensure display sends within first second; motor must transition out of RESET.
5) PERIODIC: Display should request PERIODIC and receive 24-byte payload; confirm indices match above.
6) CRC: Scope/log random payloads and re-check CRC16 calculation matches.
7) Fatal grace: In first 10 s after motor init OK, fatal bit masked in TX; verify display doesn’t latch freeze in that window.
8) No extras: Confirm no extra bytes appended and frame lengths match expectations.

## Notes

- If the display still uses 19200, either revert the baud to 19200 in BSP or update the display firmware to 115200 to match.
- The divider values are tuned for the existing clock tree; if the system clock changes, re-run divider calculation.