# Motor ↔ 860C Display Synchronization Audit

Status: DRAFT (display-side code audited; baud mismatch identified; actionable citations added)
Generated: 2025-11-11

## 1. Scope
End-to-end alignment of the serial protocol, build flags, and initialization handshake between:
- Motor firmware (this repository: `860c/`)
- 860C display firmware (audited under `display_firmware/display/firmware/...`)

Objectives:
- Preserve legacy 860C behavior when `DISPLAY_BACKEND=0` (default)
- Offer EKD01 backend optionally (`DISPLAY_BACKEND=1`) without altering legacy payload size
- Ensure UART parameters, timing of CONFIGURATIONS, and PERIODIC payload mapping are identical across both sides
- Validate fatal-error grace window masking does not stall display splash
- Produce actionable diffs + checklist

## 2. Motor-Side Ground Truth

### 2.1 Build-Time Defines & Backend Selection
File: `Makefile` (lines ~57–90 and backend block ~100+)
- `DISPLAY_BACKEND ?= 0` (default)
- `DEFINES += DISPLAY_BACKEND=$(DISPLAY_BACKEND)` injects `-DDISPLAY_BACKEND=0` (or 1) into compile.
- Conditional sources:
  - Always: `src/display/display_backend_shim.c`
  - If `DISPLAY_BACKEND=1`: `src/display/display_backend_ekd01.c`
Evidence: `build/compile_commands.json` entries show `-DDISPLAY_BACKEND=0` for every compilation unit.

### 2.2 UART Parameters
Source of record: `bsps/TARGET_tsdz8_for_GPIO_TEST/config/GeneratedSource/cycfg_peripherals.c` (baud assignment) & `REPORT.md`
- Baud: 115200 8N1 (updated from 19200)
- Oversampling: 16
- Pins: TX=P0.6, RX=P0.7 (see `cycfg_pins.h`)
- RX/TX FIFO depth: 32 words

### 2.3 Protocol Constants & Buffers
File: `ebike_app.c`
- Frame type enums (lines ~32–36): `ALIVE=0`, `STATUS=1`, `PERIODIC=2`, `CONFIGURATIONS=3`, `FIRMWARE_VERSION=4`
- RX start header expectation: `0x59` (line ~2416)
- TX header fixed: `0x43` (line ~2526)
- Buffers declared (lines ~195–200): `ui8_rx_buffer[88]`, `ui8_tx_buffer[29]`
- CRC calculation: loop calling `crc16()` while ingesting bytes (lines ~2463–2470)

### 2.4 PERIODIC Payload Mapping
Documented in `REPORT.md` section "Packet format and indices"; implemented in `communications_process_packages(COMM_FRAME_TYPE_PERIODIC)` (not fully excerpted here). Indices stable; no extra debug byte because `TSDZ8_APPEND_DEBUG_PERIODIC` default 0 (macro in `main.h`).

### 2.5 Fatal-Error Grace Window
File: `ebike_app.c` (near controller top):
- Variables: `g_motor_init_done_ms`, `fatal_grace_active()`
- Masking: TX system state index (19) suppresses fatal bit while grace active (within PERIODIC builder); prevents premature splash freeze.

### 2.6 EKD01 Backend
File: `src/display/display_backend_ekd01.c`
- Default baud override if unset: line ~126 sets `g_cfg.uart_baud = 115200`.
- Uses external hooks `uart_open`, `uart_write`, `uart_read` (lines ~18–20).
- Emits fixed-length frames (header 0x55, LEN 10) – distinct from legacy 0x43 frames; isolated by backend selection.

## 3. Display-Side Ground Truth (citations)

Source tree audited:
- UART and ISR: `display_firmware/display/firmware/860C_850C/src/src/usart1.c`
- UART abstraction: `display_firmware/display/firmware/860C_850C/src/src/uart.c`
- Protocol/state machine: `display_firmware/display/firmware/common/src/state.c`
- UART sizes: `display_firmware/display/firmware/common/include/uart.h`
- CRC: `display_firmware/display/firmware/common/src/utils.c`

Key findings:
- UART parameters: `usart1_init()` sets `USART_InitStructure.USART_BaudRate = 19200;` with 8N1; DMA TX enabled; RX via ISR (`USART1_IRQHandler`). This is mismatched with motor (115200).
- RX header and packet assembly: In `USART1_IRQHandler()`, state 0 accepts only `0x43` as start-of-frame, assembles up to `len` and verifies CRC; upon success sets `ui8_received_package_flag` and copies into `ui8_rx_buffer`.
- TX header: `rt_send_tx_package()` in `state.c` sets `ui8_usart1_tx_buffer[0] = 0x59;` and `ui8_usart1_tx_buffer[2] = type`, appends CRC and sends via `uart_send_tx_buffer()`.
- Buffers and sizes: `common/include/uart.h` defines `UART_NUMBER_DATA_BYTES_TO_RECEIVE 29` and `UART_NUMBER_DATA_BYTES_TO_SEND 88` — complementary to the motor side (RX 88 / TX 29).
- CRC16: `utils.c::crc16(uint8_t, uint16_t*)` seeds with `0xFFFF` and uses polynomial `0xA001` (Modbus), matching motor.
- Init/CONFIG timing and cadence:
  - `state.c::motor_init()` transitions:
    - Waits for ALIVE (`MOTOR_INIT_WAIT_MOTOR_ALIVE`), then requests firmware (`MOTOR_INIT_GET_MOTOR_FIRMWARE_VERSION` → sends `FRAME_TYPE_FIRMWARE_VERSION` in `MOTOR_INIT_WAIT_MOTOR_FIRMWARE_VERSION`).
    - After receiving version and validating semver, enters CONFIG sequence: sets `g_motor_init_state_conf = MOTOR_INIT_CONFIG_SEND_CONFIG`, then in that substate calls `rt_send_tx_package(FRAME_TYPE_CONFIGURATIONS)` and cycles through `STATUS` polling until `MOTOR_INIT_STATUS_INIT_OK`, then transitions to `MOTOR_INIT_READY`.
  - Runtime send cadence: `state.c::communications()` ends with `if (g_motor_init_state == MOTOR_INIT_READY) rt_send_tx_package(FRAME_TYPE_PERIODIC);` ensuring continuous periodic TX once ready.

Representative code pointers (by function statements):
- `usart1.c::usart1_init` — `USART_BaudRate = 19200;` and 8N1 setup.
- `usart1.c::USART1_IRQHandler` — header check `if (ui8_byte_received == 0x43)`, CRC calculation loop with `crc16` over `len` bytes, and buffer copy when CRC matches.
- `common/src/state.c::rt_send_tx_package` — sets header `0x59`, populates PERIODIC or CONFIGURATIONS payloads, then computes CRC and transmits.
- `common/src/state.c::communications` — reads motor frames (types STATUS/PERIODIC/FW_VER) and triggers PERIODIC TX when READY.
- `common/src/state.c::motor_init` — drives init handshake and CONFIGURATIONS send path as described above.
- `common/include/uart.h` — TX/RX size macros (88/29).
- `common/src/utils.c::crc16` — seed `0xFFFF`, poly `0xA001`.

## 4. Potential Divergences & Impact
| Area | Divergence | Impact | Resolution |
|------|------------|--------|-----------|
| Baud rate | Display at 19200 vs motor at 115200 | Link silent / framing errors | Preferred: bump display to 115200 in `usart1.c` (change `USART_BaudRate` to 115200). Temporary fallback: revert motor BSP UART to 19200. |
| CONFIG timing | Display delays >1s | Motor stays in RESET, watchdog may disable PWM | Ensure CONFIG within 500 ms or add extended wait state (not recommended) |
| PERIODIC size | Display expects +1 debug byte | Misaligned indices, SOC, torque misread | Confirm macro off; update display struct; guard optional byte behind flag |
| CRC polynomial/seed | Display uses different CRC | Frames rejected; error counter climbs | Align to motor's 0xFFFF seed CRC16; provide reference implementation |
| Fatal bit interpretation | Display latches masked fatal as OK permanently | Might not surface genuine errors after grace | Ensure periodic re-evaluation post-grace; add timestamp check |
| EKD01 vs 860C | Display firmware inadvertently reads EKD01 frames | Garbage data displayed | Keep backend selection mutually exclusive; display code only compiles legacy parser |

## 5. Required Diffs (proposed)
Motor side: (already applied)
- Baud 115200 (retain unless blocked)
- Grace window masking active
Display side (proposed):
1. In `860C_850C/src/src/usart1.c::usart1_init`, set `USART_InitStructure.USART_BaudRate = 115200;` (keep 8N1).
2. Init handshake is already correct: `common/src/state.c::motor_init` sends `FRAME_TYPE_CONFIGURATIONS` and polls `STATUS` until `INIT_OK` — no change needed, just verify timing under higher baud.
3. Confirm PERIODIC payload mapping: current TX mapping in `rt_send_tx_package(FRAME_TYPE_PERIODIC)` matches motor’s expectations; ensure no extra debug byte is appended anywhere. If any legacy flag exists, keep it disabled by default.
4. CRC routine already matches motor. No change required.
5. Continue to send `FRAME_TYPE_PERIODIC` when `MOTOR_INIT_READY` as implemented; ensure cadence remains <= 100 ms period.

## 6. Build Matrix & Commands
Fish shell examples (run from repo root):
- Legacy 860C (default):
  `make -j build`
- EKD01 backend enabled:
  `make -j build DISPLAY_BACKEND=1`
- EKD01 with logging:
  `make -j build DISPLAY_BACKEND=1 CFLAGS='-DLOG_EKD01'`
- Clean + Rebuild (legacy):
  `make clean; make -j build`

## 7. Verification Checklist (Motor + Display Together)
1. Electrical: Common ground, UART pins cross-over, logic levels acceptable.
2. Serial sniff at boot: Motor emits ALIVE frames (0x43 header, small len). Display emits CONFIG (0x59 header).
3. Motor transitions `ui8_m_motor_init_state` → INIT_START_DELAY → INIT_WAIT_DELAY → INIT_OK (observe via RTT or STATUS frames) before grace expiry.
4. Within grace (10 s): Fatal events suppressed; display does not freeze if intermittent faults occur.
5. PERIODIC frames observed at correct length; CRC matches; error counter stable (<5).
6. Assist level, duty, currents render correctly (no off-by-one index due to missing debug byte).
7. After grace window: Induce a controlled fatal (e.g., simulate comm loss) and verify display shows error appropriately.
8. Baud stability: No framing errors at 115200 (scope/LA).
9. EKD01 build (if used): Distinct header 0x55 confirmed; not parsed by 860C display firmware.

## 8. Risk Assessment & Mitigations
- High baud reliability: If hardware lines marginal, revert to 57600 as fallback (update both sides); document change.
- Watchdog false positives: If display poll slower than expected, consider widening error threshold (currently >30 counts ≈750 ms). Prefer fixing poll cadence.
- Grace masking hides real faults initially: Provide RTT counter `ui8_deferred_fatal_events` to review suppressed events; escalate if non-zero after grace.

## 9. Next Actions
1. Apply the display-side baud rate change to 115200 in `usart1.c` and rebuild the display firmware.
2. Test end-to-end boot: observe ALIVE → FW_VER → CONFIGURATIONS → READY at 115200; confirm that PERIODIC frames flow and that the motor exits RESET before grace window expiry.
3. Validate no payload size mismatch: verify display TX PERIODIC length and motor RX parser alignment; keep `TSDZ8_APPEND_DEBUG_PERIODIC=0` on motor.
4. If hardware lines are noisy at 115200, optionally test 57600 on both sides and update this report.

## 10. Appendices
### A. Motor CRC Reference (Pseudo)
```
uint16_t crc16_update(uint16_t crc, uint8_t data);
// Seed 0xFFFF; iterate over all bytes excluding final CRC
```
(Exact implementation in motor code referenced via `crc16()` calls inside RX loop lines ~2463–2469 of `ebike_app.c`).

### B. Grace Window Configuration
Macros (if present in `main.h` or config header):
- `FATAL_GRACE_ENABLE` (expected 1)
- `FATAL_GRACE_MS` (expected 10000)
Provide display a note: treat system state byte changes with potential delayed fatal assertion after this window.

—
DRAFT report updated with display-side citations and required action (baud alignment).
