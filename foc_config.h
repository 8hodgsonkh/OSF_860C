#pragma once
// Sensing chain
#define PHASE_SHUNT_OHMS      0.003f
#define PHASE_AMP_GAIN        10.0f
#define ADC_REF_VOLTS         5.0f
#define ADC_COUNTS            4096.0f
#define PHASE_COUNTS_PER_AMP ((PHASE_SHUNT_OHMS * PHASE_AMP_GAIN) * (ADC_COUNTS / ADC_REF_VOLTS)) // 24.576
#define PHASE_AMPS_PER_COUNT (1.0f / PHASE_COUNTS_PER_AMP) // 0.04068 A/count

// Motor/electrics (tuneable but start here)
#define POLE_PAIRS            4       // from your tear-down photo
#define L_PHASE_HENRY         150e-6f // starting estimate
#define R_PHASE_OHM           0.10f   // 36V motor ~0.094 Ω / 48V ~0.125 Ω (pick your unit/build)

// Loop gains (safe starters)
#define ID_KP  0.06f
#define ID_KI  0.0015f
#define IQ_KP  0.06f
#define IQ_KI  0.0015f

// Limits
#define IQ_MAX_A        45.0f   // torque current cap (<= phase limit)
#define ID_FW_MIN_A   (-15.0f)  // max field-weakening negative Id
#define VDQV_MAX_V     0.98f    // normalize to Vbus (modulation index cap)

// FW thresholds
#define FW_ENABLE_RATIO  0.90f  // if |V| > 0.90*Vbus, start FW
#define FW_ID_STEP_A     0.2f   // step Id* more negative per control tick while saturated

// Misc
#define DEBUG_TELEM_TO_TECH_SCREEN 1  // keep if you want i_d/i_q streamed
