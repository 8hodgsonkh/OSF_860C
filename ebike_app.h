/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EBIKE_APP_H_
#define _EBIKE_APP_H_

//#include <stdint.h>
#include "main.h"
// (Optional) DISPLAY_BACKEND macros may be defined via build system. We avoid including
// backend headers here to keep dependencies minimal.

// moved from ebike_app.c because used also in main.c
// from v.1.1.0
// Error state (changed)
#define NO_ERROR                                0			// "None"
#define ERROR_NOT_INIT                          1			// "Motor not init"
#define ERROR_TORQUE_SENSOR                     (1 << 1)	// "Torque Fault"
#define ERROR_CADENCE_SENSOR		    		(1 << 2)	// "Cadence fault"
#define ERROR_MOTOR_BLOCKED     				(1 << 3)	// "Motor Blocked"
#define ERROR_THROTTLE						 	(1 << 4)	// "Throttle Fault"
#define ERROR_FATAL                             (1 << 5)	// "Fatal error"  or "Undervoltage"
#define ERROR_BATTERY_OVERCURRENT               (1 << 6)	// "Overcurrent"
#define ERROR_SPEED_SENSOR	                    (1 << 7)	// "Speed fault"




// startup boost mode
#define CADENCE					0
#define SPEED						1

// for oem display
extern volatile uint8_t ui8_system_state;

// cadence sensor
extern uint16_t ui16_cadence_ticks_count_min_speed_adj;

// Torque sensor coaster brake engaged threshold value
extern uint16_t ui16_adc_coaster_brake_threshold;

// ADC motor phase current max
extern volatile uint16_t ui16_adc_motor_phase_current_max;

// Motor enabled
extern uint8_t ui8_motor_enabled;


extern uint8_t ui8_m_system_state;

// added by mstrens
extern uint8_t ui8_m_motor_init_state ;
extern uint8_t ui8_m_motor_init_status;

extern uint8_t g_clutch_active;


/*
typedef struct  _configuration_variables
{
  //uint8_t ui8_motor_power_x10; // not used
  uint32_t version;                 // added by mstrens to check the validity of configuration in flash; 32bits to use readWord
  uint8_t ui8_battery_current_max; // from  ebike_app.c
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_wheel_speed_max;
  uint8_t ui8_motor_type;
  uint8_t ui8_avaiable_for_future_use;
  // for oem display
  uint8_t ui8_assist_without_pedal_rotation_enabled;
  uint8_t ui8_assist_with_error_enabled;
  uint8_t ui8_battery_SOC_percentage_8b;
  uint8_t ui8_set_parameter_enabled;
  uint8_t ui8_street_mode_enabled;
  uint8_t ui8_riding_mode;
  uint8_t ui8_lights_configuration;
  uint8_t ui8_startup_boost_enabled;
  uint8_t ui8_auto_display_data_enabled;
  uint8_t ui8_torque_sensor_adv_enabled; 
  uint8_t ui8_soc_percent_calculation;
} struct_configuration_variables;
*/

extern uint8_t ui8_test_mode_flag ;
extern volatile uint32_t ui32_adc_battery_current_1_rotation_15b; // value in 12 +2 +1 = 15 bits (ADC + IIR + average)
extern uint8_t hall_reference_angle ;

void new_torque_sample();
void fillRxBuffer();

/*
extern uint8_t ui8_best_ref_angles1 ;
extern uint8_t ui8_best_ref_angles2 ;
extern uint8_t ui8_best_ref_angles3 ;
extern uint8_t ui8_best_ref_angles4 ;
extern uint8_t ui8_best_ref_angles5 ;
extern uint8_t ui8_best_ref_angles6 ;
*/
void ebike_app_controller(void);
//struct_configuration_variables* get_configuration_variables(void);

void ebike_app_init(void);

uint16_t read_battery_soc(void);

//static void calc_oem_wheel_speed(void);
//static void ebike_control_lights(void);


#if defined(DISPLAY_BACKEND) && (DISPLAY_BACKEND == 1)
// EKD01 boot-time controller defaults interface
typedef struct {
  // Speed / wheel geometry
  uint16_t wheel_perimeter_mm;                    // e.g., 2050
  uint8_t  wheel_speed_max_kmh;                   // off-road/default cap, e.g., 60
  uint8_t  street_mode_speed_kmh;                 // enforced when lights ON, e.g., 25

  // Battery & power safety
  uint16_t battery_low_voltage_cut_off_x10;       // e.g., 400 => 40.0V
  uint8_t  battery_overcurrent_delay;             // 0 disables overcurrent latch

  // Assist-level and riding mode
  uint8_t  assist_level_flag;                     // 1 enables motor gating from start
  uint8_t  riding_mode;                           // POWER_ASSIST_MODE, TORQUE_ASSIST_MODE, ...
  uint8_t  riding_mode_parameter;                 // assist multiplier (typ 50)

  // Smooth start / startup assist
  uint8_t  smooth_start_enabled;                  // 0/1
  uint8_t  smooth_start_counter_set;              // ramp length; 0 ok if disabled

  // Torque sensor tunables
  uint8_t  adc_pedal_torque_range_adj;            // 0..40
  uint8_t  adc_pedal_torque_angle_adj;            // 0..40
  uint8_t  pedal_torque_step_x100;                // ui8_pedal_torque_per_10_bit_ADC_step_x100

  // Throttle calibration
  uint8_t  throttle_min;                          // ADC counts (8-bit scaled)
  uint8_t  throttle_max;                          // ADC counts (8-bit scaled)
  uint8_t  throttle_legal;                        // 0/1 gate for cadence requirement

  // Walk assist & misc
  uint8_t  walk_assist_parameter;                 // 0 by default
  uint8_t  assist_without_pedal_rotation_enabled; // 0/1
  uint8_t  assist_without_pedal_rotation_threshold; // ADC threshold delta

  // eMTB / hybrid specifics
  uint8_t  eMTB_based_on_power;                   // 0 torque / 1 power
  uint8_t  hybrid_torque_parameter;               // 0 default
} ekd01_controller_defaults_t;

// Apply EKD01 controller defaults safely to internal statics
void ekd01_controller_apply_defaults(const ekd01_controller_defaults_t *d);
#endif

// -------------------------------------------------------------
// Fatal grace window helper
// -------------------------------------------------------------
// Timestamp captured at motor init completion (controller time base ms)
extern volatile uint32_t g_motor_init_done_ms;
// Count of fatal events suppressed during grace window (diagnostic)
extern uint8_t ui8_deferred_fatal_events;
// Returns true while grace window is active
bool fatal_grace_active(void);




#endif /* _EBIKE_APP_H_ */


