// EKD01 display backend (UART framed) – optional build when DISPLAY_BACKEND==DISPLAY_BACKEND_EKD01
// Contract: non-blocking, integer math only, TX every ~50ms, RX parsing opportunistic.
// Logging enabled only if LOG_EKD01 defined.
// Wiring: Caller must periodically invoke display_backend_update(...) at >=10ms cadence.

#include "display_backend.h"
#if DISPLAY_BACKEND == DISPLAY_BACKEND_EKD01

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// NOTE: These HAL headers are expected to exist in the project.
// If not available yet, replace calls with your UART/time primitives or stub out.
#include "SEGGER_RTT.h"

// Provide light-weight UART and time hooks; adapt to your platform
extern void uart_open(uint32_t port, uint32_t baud);
extern int  uart_write(uint32_t port, const void *data, int len);
extern int  uart_read(uint32_t port, void *data, int maxlen);
extern uint32_t time_ms(void);

// --- Protocol constants (adjust to EKD01 spec as you learn more) ---
#define EKD_HDR_TX  0x55     // ctrl -> disp
#define EKD_HDR_RX  0xAA     // disp -> ctrl (if present)
#define EKD_TX_LEN  10
#define EKD_RX_MAX  12

static display_backend_config_t g_cfg;
static uint8_t tx_buf[EKD_TX_LEN];
static uint32_t last_tx_ms;
static uint8_t rx_buf[EKD_RX_MAX];
static int rx_len;

// Logging macro (compile-time guard)
#ifdef LOG_EKD01
#define EKD01_LOG(fmt, ...) SEGGER_RTT_printf(0, "[EKD01] " fmt "\r\n", ##__VA_ARGS__)
#else
#define EKD01_LOG(fmt, ...)
#endif

static inline uint8_t csum_xor(const uint8_t *p, int n){
  uint8_t x=0; for(int i=0;i<n;i++) x^=p[i]; return x;
}

static void ekd01_pack_tx(const display_backend_inputs_t *in){
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = EKD_HDR_TX;
  tx_buf[1] = EKD_TX_LEN;

  // Scale to coarse integer fields; refine once EKD01 framing is finalized
  tx_buf[2] = (uint8_t)(in->battery_mv / 100);        // decivolts
  tx_buf[3] = (uint8_t)(in->speed_cmh / 100);         // ~0.1 km/h
  tx_buf[4] = (uint8_t)(in->assist_level & 0x0F);

  uint8_t flags = 0;
  if(in->lights_on)   flags |= 0x01;
  if(in->overvoltage) flags |= 0x02;
  if(in->fault_flags) flags |= 0x04;
  tx_buf[5] = flags;

  tx_buf[6] = (uint8_t)(in->motor_amps_x5 & 0xFF);    // coarse current
  tx_buf[7] = (uint8_t)(in->duty_pct_x10 / 10);       // duty %
  tx_buf[8] = (in->soc_pct <= 100) ? in->soc_pct : 0xFF;

  tx_buf[EKD_TX_LEN-1] = csum_xor(tx_buf, EKD_TX_LEN-1);
}

static void ekd01_try_tx(uint32_t now_ms){
  uint16_t period = g_cfg.tx_period_ms ? g_cfg.tx_period_ms : 50;
  if((uint16_t)(now_ms - last_tx_ms) < period) return;
  last_tx_ms = now_ms;
  (void)uart_write(g_cfg.uart_port, tx_buf, EKD_TX_LEN);
}

static uint32_t rx_bad_hdr, rx_bad_len, rx_bad_csum;

static void ekd01_try_rx(void){
  // Optional: process inbound frames (buttons/state). Safe to ignore if unused.
  uint8_t b[16];
  int n = uart_read(g_cfg.uart_port, b, (int)sizeof(b));
  if(n <= 0) return;

  for(int i=0;i<n;i++){
    if(rx_len < EKD_RX_MAX) rx_buf[rx_len++] = b[i];
    // naive frame finder
    if(rx_len >= 2){
      if(rx_buf[0] != EKD_HDR_RX){
        // Header mismatch, shift buffer left by one
        if(rx_len > 0){ memmove(rx_buf, rx_buf+1, (size_t)(--rx_len)); rx_bad_hdr++; }
        continue;
      }
      int len = rx_buf[1];
      if(len < 4 || len > EKD_RX_MAX){
        // Invalid length byte -> shift out header and length
        if(rx_len >= 2){ memmove(rx_buf, rx_buf+2, (size_t)(rx_len-2)); rx_len -= 2; rx_bad_len++; }
        continue;
      }
      if(rx_len >= len){
        // Full frame present
        uint8_t expect = csum_xor(rx_buf, len-1);
        if(rx_buf[len-1] == expect){
          // Example map: [2]=buttons, [3]=assist delta, [4]=lights toggle
          uint8_t buttons = rx_buf[2];
          (void)buttons; // TODO: dispatch to input handler (buttons, lights, assist adjust)
        } else {
          rx_bad_csum++;
        }
        // shift buffer
        int rem = rx_len - len;
        if(rem > 0) memmove(rx_buf, rx_buf+len, (size_t)rem);
        rx_len = rem;
      }
    }
  }

  // Periodic lightweight stats (every ~256 frames processed) if enabled
  static uint8_t stat_div;
  if(++stat_div == 0){
    EKD01_LOG("RX stats hdr=%lu len=%lu csum=%lu", (unsigned long)rx_bad_hdr, (unsigned long)rx_bad_len, (unsigned long)rx_bad_csum);
  }
}

void display_backend_init(const display_backend_config_t *cfg){
  g_cfg = *cfg;
  if(g_cfg.uart_baud == 0) g_cfg.uart_baud = 19200;
  if(g_cfg.tx_period_ms == 0) g_cfg.tx_period_ms = 50;
  uart_open(g_cfg.uart_port, g_cfg.uart_baud);
  last_tx_ms = 0;
  rx_len = 0;
  rx_bad_hdr = rx_bad_len = rx_bad_csum = 0;
  EKD01_LOG("init uart_port=%lu baud=%lu period=%u", (unsigned long)g_cfg.uart_port, (unsigned long)g_cfg.uart_baud, (unsigned)g_cfg.tx_period_ms);
}

void display_backend_update(const display_backend_inputs_t *in){
  ekd01_pack_tx(in);
  ekd01_try_tx(time_ms());
  ekd01_try_rx();
}

uint8_t display_backend_map_error(uint8_t system_state){
  // Basic mapping: 0 = ok, non-zero -> generic fault code 1.
  return (system_state == 0) ? 0 : 1;
}

uint8_t display_backend_select_function_code(uint8_t menu_index){
  // EKD01 specific function-code mapping if needed. Identity by default.
  return menu_index;
}

uint8_t display_backend_battery_bar_state(uint8_t soc_pct, bool overvoltage){
  if(overvoltage) return 0xF0;
  if(soc_pct > 100) return 0xFF; // unknown
  return (uint8_t)(soc_pct / 10); // 0..10 bars
}

// Provide spec alias without touching existing builds
#define display_backend_update_10ms display_backend_update

#endif // DISPLAY_BACKEND_EKD01
