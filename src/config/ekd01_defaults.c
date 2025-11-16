#include "ekd01_defaults.h"
#include "ebike_app.h"

// NOTE: The 860C code presently keeps many configuration variables in static globals inside ebikc.
// Without refactoring those into an exposed struct, we touch only what is available via externs
// from existing headers. This is a placeholder that can be expanded once a formal m_config
// structure (similar to OSF) is surfaced here.

// Build lock macro; when defined, suppress runtime changes.
#ifdef EKD01_LOCK_DEFAULTS
#define EKD01_CAN_APPLY() (0)
#else
#define EKD01_CAN_APPLY() (1)
#endif

void ekd01_defaults_apply(bool persist){
    (void)persist; // persistence not implemented yet
    if(!EKD01_CAN_APPLY()) return;
    // Mapping (updated per user request):
    //   STREET (legal) mode is the default at boot with lights OFF.
    //   OFFROAD mode is engaged when lights are turned ON.
    // We no longer force any lights state here; display/user controls lights.

    // Apply controller-side EKD01 defaults so runtime matches the user's requested values.
    // Note: These are safe overlays of internal statics; persistent storage is not used here.
#if defined(DISPLAY_BACKEND) && (DISPLAY_BACKEND == 1)
    ekd01_controller_defaults_t d = {0};
    // Wheel and speed caps
    d.wheel_perimeter_mm = 2050;   // keep geometry
    d.wheel_speed_max_kmh = 60;    // off-road cap (lights ON)
    d.street_mode_speed_kmh = 15;  // new legal cap (lights OFF)

    // Battery safety
    d.battery_low_voltage_cut_off_x10 = 400; // 40.0 V
    d.battery_overcurrent_delay = 0;         // disable ADC overcurrent latch/delay

    // Assist and riding mode
    d.assist_level_flag = 1;
    d.riding_mode = TORQUE_ASSIST_MODE;      // requested default
    d.riding_mode_parameter = 50;            // suggested default

    // Smooth start
    d.smooth_start_enabled = 0;
    d.smooth_start_counter_set = 0;

    // Torque sensor tuning
    d.adc_pedal_torque_range_adj = 0;
    d.adc_pedal_torque_angle_adj = 0;
    d.pedal_torque_step_x100 = 61;

    // Throttle calibration
    d.throttle_min = 35;
    d.throttle_max = 174;
    d.throttle_legal = 0; // allow throttle without cadence; capped to 6 km/h in legal mode

    // Walk assist and misc
    d.walk_assist_parameter = 0;
    d.assist_without_pedal_rotation_enabled = 0;
    d.assist_without_pedal_rotation_threshold = 0;

    // eMTB / hybrid
    d.eMTB_based_on_power = 1;
    d.hybrid_torque_parameter = 0;

    ekd01_controller_apply_defaults(&d);
#endif
    // 3) Persistence (EEPROM) could be added here in future; currently omitted.
}
