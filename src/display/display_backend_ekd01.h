#pragma once
#include <stdint.h>

// EKD01 protocol constants (draft)
#define EKD01_HDR_TX  0x55
#define EKD01_HDR_RX  0xAA
#define EKD01_TX_LEN  10
#define EKD01_RX_MAX  12

// Enable LOG_EKD01 in CFLAGS to get lightweight RTT logs from EKD01 backend
// Example: make DISPLAY_BACKEND=1 CFLAGS+="-DLOG_EKD01"
