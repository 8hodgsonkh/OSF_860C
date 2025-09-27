/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */
#include "main.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "motor.h"
#include "ebike_app.h"
#include "common.h"
#include "adc.h"
#include "foc_config.h"
#include "phase_current.h"
#include "foc_math.h"

#include "cy_retarget_io.h"
//#include "cy_utils.h"
#if(uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
#include "ProbeScope/probe_scope.h"
#endif



// **************  to test slow rotation without using the hall sensor and so discover pattern sequence
// just to test rotation at a low speed and low power to verify the the hall sequence is OK
#define SPEED_COUNTER_MAX 19000 /360  // one electrical rotation per sec ; so 1 mecanical rotation takes 4 sec ; so 15 rpm
#define DUTY_CYCLE_TEST 30// 256 = 100% ; 40 gives a current = 1A from ADC on pin 2.8 with a 12V battery
#define ANGLE_INIT 0
// end of those test parameters


// pattern for hall sensor is 1,3,2,6,4, 5
// with full use of posif possibilities, this table should be read with expected pattern and so upload in shadow register for the next expected
// when current pattern is 1 and expected = 3 , the sadow register should be prepare for the next transition with current = 3 and exp=6 
// in current version, it is used only to detect if a transition is valid in irq0
const uint8_t expected_pattern_table[8] = {
    3, // 0 should not happen
    3, // after 1 => 3 
    6, // after 2 => 6
    2, // after 3 => 2
    5, // after 4 => 5
    1, // after 5 => 1
    4, // after 6 => 4
    1 // 7 should not happen 
};


// copied from tsdz2
#define SVM_TABLE_LEN   256
// for tsdz8 using a timer counting up to 1680 (instead of 420); values have been calculated in an XLS sheet (on google drive)
// svm table 19 Khz
static const uint16_t ui16_svm_table[SVM_TABLE_LEN] = {
1566,1576,1586,1595,1604,1612,1620,1627,1634,1640,1646,1651,1656,1661,1665,1668,1671,1674,1676,
1677,1678,1678,1678,1678,1677,1675,1673,1670,1667,1664,1659,1655,1650,1644,1638,1632,1625,1617,
1609,1601,1592,1583,1573,1556,1524,1493,1461,1428,1396,1363,1329,1295,1261,1227,1193,1158,1123,
1088,1053,1018,982,947,911,876,840,804,769,733,698,662,627,592,557,522,487,453,419,385,351,317,
284,252,219,187,156,124,107,97,88,79,71,63,55,48,42,36,30,25,21,16,13,10,7,5,3,2,2,2,2,3,4,6,9,
12,15,19,24,29,34,40,46,53,60,68,76,85,94,104,114,104,94,85,76,68,60,53,46,40,34,29,24,19,15,12,
9,6,4,3,2,2,2,2,3,5,7,10,13,16,21,25,30,36,42,48,55,63,71,79,88,97,107,124,156,187,219,252,284,
317,351,385,419,453,487,522,557,592,627,662,698,733,769,804,840,876,911,947,982,1018,1053,1088,
1123,1158,1193,1227,1261,1295,1329,1363,1396,1428,1461,1493,1524,1556,1573,1583,1592,1601,1609,
1617,1625,1632,1638,1644,1650,1655,1659,1664,1667,1670,1673,1675,1677,1678,1678,1678,1678,1677,
1676,1674,1671,1668,1665,1661,1656,1651,1646,1640,1634,1627,1620,1612,1604,1595,1586,1576};


// motor variables
uint8_t ui8_hall_360_ref_valid = 0; // fill with a hall pattern to check sequence is correct
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
static uint8_t ui8_motor_phase_absolute_angle = 0;
volatile uint16_t ui16_hall_counter_total = 0xffff; // number of tim3 ticks between 2 rotations// inTSDZ2 it was a u16
static uint16_t ui16_hall_counter_total_previous = 0;  // used to check if erps is stable
// power variables
volatile uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
volatile uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
volatile uint16_t ui16_adc_voltage_cut_off = 300*100/BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000; // 30Volt default value =  300*100/87 in TSDZ2
volatile uint8_t ui8_adc_battery_current_filtered = 0; // current in adc10 bits units (average on 1 rotation)
volatile uint32_t ui32_adc_battery_current_1_rotation_15b = 0; // value in 12 +2 +1 = 15 bits (ADC + IIR + average)
volatile uint8_t ui8_controller_adc_battery_current_target = 0;
volatile uint8_t ui8_g_duty_cycle = 0;
volatile uint8_t ui8_controller_duty_cycle_target = 0;
// Field Weakening Hall offset (added during interpolation)
volatile uint8_t ui8_fw_hall_counter_offset = 0;
volatile uint8_t ui8_fw_hall_counter_offset_max = 0;
volatile uint8_t ui8_field_weakening_enabled = 0;

// Duty cycle ramp up
static uint8_t ui8_counter_duty_cycle_ramp_up = 0;
static uint8_t ui8_counter_duty_cycle_ramp_down = 0;

// FOC angle
static uint8_t ui8_foc_angle_accumulated = 0;
/*
 * ui8_foc_flag is set once per electrical rotation (when hall pattern 0x03 is detected).
 * We no longer repurpose this flag to carry the phase‑advance value.  Instead it acts solely
 * as a “kick” to execute the hill‑climb controller below.  See the FOC update code in
 * the IRQ handler for details.
 */
static uint8_t ui8_foc_flag = 0;
volatile uint8_t ui8_g_foc_angle = 0;
uint8_t ui8_foc_angle_multiplicator = 0;

/*
 * Extremum‑seeking (hill‑climb) controller constants.
 *
 * The motor controller measures electrical rotations per second (ERPS) and battery current each
 * electrical rotation.  The goal of the optimiser is to find the FOC phase‑advance (lead angle)
 * that maximises ERPS while minimising current.  It does this by nudging the advance up or
 * down depending on whether the most recent change increased speed and/or decreased current.
 *
 * ADV_MIN / ADV_MAX: allowable range for advance (in signed 0…255 units ≈ −90…+90° electrical).
 * A cap around 18 (~25° electrical ≈ 12.5° mechanical for two pole pairs) is conservative for
 * the TSDZ8.  ADV_MIN now equals ADV_INIT, typically −10, so the optimiser may start with
 * a small negative lead at stall.  Adjust ADV_INIT and ADV_MIN to change this offset.
 * STEP_UP / STEP_DN: how far to move the target on each successful/unsuccessful step.
 * I_HYST / ER_HYST: deadbands to ignore small changes in current or ERPS to reduce noise.
 * SLEW_UP_PER_TICK / SLEW_DN_PER_TICK: maximum change applied to the actual angle per
 * electrical rotation; rising more slowly than falling helps the controller stabilise.
 * DECIMATE_N: evaluate the optimisation logic only every N electrical rotations to allow the
 * system to settle between adjustments.
 */
// Base advance range definitions.
//
// We normally start the optimiser at a small negative lead to increase low‑end torque.  One
// "count" in ui8_g_foc_angle corresponds to ~1.4° electrical, so a value of –10 counts
// equates to approximately –14° electrical (≈ –10° mechanical for a two‑pole‑pair motor).
// ADV_INIT defines this initial negative offset in counts.  The optimiser may increase
// advance above this value up to ADV_MAX.  ADV_MIN is set equal to ADV_INIT so that
// adv_target and adv_actual never drop below this starting point.  To revert to the
// original behaviour (no negative lead), set ADV_INIT to 0.
//
// NOTE: Changing ADV_INIT below –10 is generally not recommended as too much retard can
// destabilise the motor at start‑up.
#define ADV_INIT                (-10)
#define ADV_MIN                 ADV_INIT
#define ADV_MAX                 18U
// The original OSF build used:
// #define ADV_MIN                 0U
// #define ADV_MAX                 18U
// but these are commented out here because we now allow a negative starting angle.  To
// restore the original behaviour (zero or positive advance only), replace the above
// definitions with the commented ones.
#define STEP_UP                 1U
#define STEP_DN                 1U
#define I_HYST                  1U
#define ER_HYST                 1U
#define SLEW_UP_PER_TICK        1U
#define SLEW_DN_PER_TICK        2U
#define DECIMATE_N              4U

// State for the extremum‑seeking phase‑advance optimiser
// adv_target and adv_actual are signed to permit negative advance values.  They start at
// ADV_INIT (typically –10) so the motor begins with a small retarded lead.  Using signed
// types also makes it straightforward to clamp the advance between ADV_MIN and adv_max_dynamic.
// The original unsigned definitions looked like this:
// static uint8_t adv_target = 0U;
// static uint8_t adv_actual = 0U;
// static int8_t  dir_hc = 1;
// We keep them here as comments so it’s easy to revert if you wish to disable negative advance.
static float id_int = 0.0f, iq_int = 0.0f;
static float id_ref = 0.0f, iq_ref = 0.0f;
static float vd_est_prev = 0.0f, vq_est_prev = 0.0f;
static uint8_t foc_subsample = 0U;

#ifndef TWO_PI_F
#define TWO_PI_F (6.28318530717958647692f)
#endif

/*
 * The measured electrical rotations per second (ERPS) comes from ebike_app.c.  It is
 * recomputed once every 25 ms and exposed as ui16_motor_speed_erps.  Declaring it extern here
 * lets motor.c use it without introducing a circular dependency.
 */
extern uint16_t ui16_motor_speed_erps;
//static uint8_t ui8_foc_angle_multiplier = FOC_ANGLE_MULTIPLIER; //39 for 48V motor
//static uint8_t ui8_adc_foc_angle_current = 0; // use a ui16 inside the irq

// battery current variables
uint16_t ui16_adc_battery_current_acc_X4 = 0;
uint16_t ui16_adc_battery_current_filtered_X4 = 0;
volatile uint16_t ui16_adc_motor_phase_current = 0; // mstrens: it was uint8 in original code

// ADC Values
volatile uint16_t ui16_adc_voltage = 0;
volatile uint16_t ui16_adc_torque = 0;
//volatile uint16_t ui16_adc_throttle = 0; // moved to ebike_app.c
//added by mstrens
volatile uint16_t ui16_adc_torque_filtered = 0 ; // filtered adc torque
volatile uint16_t ui16_adc_torque_actual_rotation = 0;
volatile uint16_t ui16_adc_torque_previous_rotation = 0;
volatile uint8_t ui8_adc_torque_rotation_reset = 0;
    
// brakes
volatile uint8_t ui8_brake_state = 0;

// cadence sensor
#define NO_PAS_REF 5
volatile uint16_t ui16_cadence_sensor_ticks = 0;
static uint16_t ui16_cadence_sensor_ticks_counter_min = CADENCE_SENSOR_CALC_COUNTER_MIN; // initialiszed at 4270 , then varies with wheelSpeed
static uint8_t ui8_pas_state_old = 4;
static uint16_t ui16_cadence_calc_counter = 0;
static uint16_t ui16_cadence_stop_counter = 0;
static uint8_t ui8_cadence_calc_ref_state = NO_PAS_REF;
const static uint8_t ui8_pas_old_valid_state[4] = { 0x01, 0x03, 0x00, 0x02 };
//added by mstrens
uint8_t ui8_pas_counter = 0; // counter to detect a full pedal rotation (after 20 valid transitions)

// wheel speed sensor
volatile uint16_t ui16_wheel_speed_sensor_ticks = 0;
volatile uint16_t ui16_wheel_speed_sensor_ticks_counter_min = 0;
volatile uint32_t ui32_wheel_speed_sensor_ticks_total = 0;


// battery soc
volatile uint8_t ui8_battery_SOC_saved_flag = 0;
volatile uint8_t ui8_battery_SOC_reset_flag = 0;

// Hall sensor state
uint8_t current_hall_pattern = 0;
uint8_t previous_hall_pattern = 7; // Invalid value, force execution of Hall code at the first run

volatile uint8_t ui8_hall_sensors_state = 0; // name used by ebike_app.c to identify current_hall_pattern; added here for compatibility

// Hall counter value of last Hall transition 
uint16_t previous_360_ref_ticks = 0 ; 

// ----------   end of copy from tsdz2 -------------------------

uint8_t ui8_temp = 0;
uint16_t ui16_temp = 0;

volatile uint16_t ui16_a = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_b = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_c = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2

uint8_t hall_reference_angle = 0 ; // This value is initialised in ebike_app.c with DEFAULT_HALL_REFERENCE_ANGLE and m_config.global_offset_angle 

// to debug time spent in irq0 and irq1
volatile uint16_t debug_time_ccu8_irq0 = 0;
//volatile uint16_t debug_time_ccu8_irq1 = 0;
//volatile uint16_t debug_time_ccu8_irq1b = 0;
//volatile uint16_t debug_time_ccu8_irq1c = 0;
//volatile uint16_t debug_time_ccu8_irq1d = 0;
//volatile uint16_t debug_time_ccu8_irq1e = 0;
uint16_t hall_ref_angles_counter = 0;

//extern uint8_t ui8_pwm_duty_cycle_max;

uint8_t ui8_hall_ref_angles[8] = { // Sequence is 1, 3, 2 , 6, 4, 5; so angle are in theory e.g. 39, 86, 127, 167, 216, 0 (256=360°)
        0,                     // error ; index must be between 1 and 6
        24 , //    for hall pattern 1
        107, //     for hall pattern 2  106
        66, //       for hall pattern 3 65
        195, //    for hall pattern 4 
        235 , //    for hall pattern 5
        152, //     for hall pattern 6  
        0                     // error ; index must be between 1 and 6
};

// this could probably be removed
volatile uint8_t ui8_best_ref_angles[8] ; // this table is prefilled in main.c at start up
uint32_t best_ref_angles_X16bits[8] ;  // same as ui8_best_ref_angles but with 8 more bits for better filtering
uint32_t ui32_angle_per_tick_X16shift = 0 ; // 

// Hall offset for current Hall state; This offset is added in the interpolation process (so based also on the erps)
// the value is in ticks (1 ticks = 4 usec); we need  55usec/4 : 55 = 39 + 16 (39 = 3/4 of 55usec = delay between measuring and PWM change); 16=delay hall sensor
// based on the regression tests, there should probably be a correction of about 2 depending it is a rising or a falling edge of hall pattern
// still this should have only a small impact
uint8_t ui8_hall_counter_offset = 14; 

// to debug
int16_t I_u; // to check current in each phase
int16_t I_v;
int16_t I_w;
int16_t I_t;
uint16_t prev_ticks = 0;
uint16_t interval_ticks = 0;
uint8_t first_ticks = 1; // says that interval has not yet been calculated
uint16_t error_ticks_counter = 0;
uint16_t error_ticks_value;
uint16_t error_ticks_prev;
uint16_t interval_ticks_min = 0xFFFF; 
uint16_t interval_ticks_max = 0; 

uint16_t irq0_min = 0xFFFF;
uint16_t irq0_max = 0;
uint16_t irq1_min = 0xFFFF;
uint16_t irq1_max = 0;


uint16_t hall_pattern_error_counter = 0;
// used to calculate hall angles based of linear regression of all ticks intervals
// are filled in irq0 and transmitted in ebike_app.c using segger_rtt_print 
#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
uint16_t ticks_intervals[8]; // ticks intervals between 2 pattern changes;
uint8_t ticks_intervals_status; // 0 =  new data can be written; 1 data being written; 2 all data written, must be transmitted
uint16_t previous_hall_pattern_change_ticks;  // save the ticks of last pattern change
#endif

volatile uint16_t ticks_hall_pattern_irq_last = 0;
volatile uint8_t current_hall_pattern_irq = 0;

//uint32_t ui32_angle_per_tick_X16shift_new;
//uint32_t ui32_ref_angle; 
//uint32_t ui32_ref_angle_new ; // temporary calculation
//uint8_t ui8_motor_phase_absolute_angle_new; // reference for extrapolation with new algorithm 
//uint8_t ui8_svm_old = 0;
//uint8_t ui8_svm_new = 0;

//uint16_t ui16_duty_cycle_count_down = 0;
//uint16_t ui16_duty_cycle_count_up = 0;

// average current on 1 eletric rotation
uint32_t ui32_adc_battery_current_15b = 0; // value from adc
uint32_t ui32_adc_battery_current_15b_accum = 0; 
uint32_t ui32_adc_battery_current_15b_counter = 0;


// moving average
Moving_average battery_current_avg_15b;
uint32_t ui32_adc_battery_current_15b_moving_average = 0;
int battery_current_moving_avg_index = 0;
int battery_current_moving_avg_sum = 0;
int battery_current_moving_avg_buffer[64] = {0};


// to manage torque sensor using the logic of mspider in https://github.com/TSDZ2-ESP32/TSDZ2-Smart-EBike
// 1 = PAS state value changed
// 0x80  = PAS state invalid -> reset
volatile uint8_t ui8_pas_new_transition = 0;

inline uint32_t update_moving_average(uint32_t new_value){
    battery_current_moving_avg_sum -= battery_current_moving_avg_buffer[battery_current_moving_avg_index];
    battery_current_moving_avg_buffer[battery_current_moving_avg_index] = new_value;
    battery_current_moving_avg_sum += new_value;
    battery_current_moving_avg_index = (battery_current_moving_avg_index + 1) & 0x3F; 
    // Retourne la moyenne actuelle
    return (battery_current_moving_avg_sum + 32) >> 6; // divide by 64; add 32 for better rounding
}


inline uint32_t filtering_function(uint32_t ui32_temp_15b , uint32_t ui32_filtered_15b , uint32_t alpha){
    uint32_t ui32_temp_new = ui32_temp_15b * (16U - alpha);
    uint32_t ui32_temp_old =  ui32_filtered_15b * alpha;
    uint32_t ui32_filtered_value = ((ui32_temp_new + ui32_temp_old + (8)) >> 4);                    
    if (ui32_filtered_value == ui32_filtered_15b) {
        if (ui32_filtered_value < ui32_temp_15b)
            ui32_filtered_value++;
        else if (ui32_filtered_value > ui32_temp_15b)
            ui32_filtered_value--;
    }
    return ui32_filtered_value ;                  
}

void VADC0_G0_0_IRQHandler() {  // VADC is configured to compare the total current (12bits) with "1000" and generate an irq
    ui8_m_system_state |= ERROR_BATTERY_OVERCURRENT; // set the error to avoid that motor starts again
    // disable the motor
    ui8_motor_enabled = 0;
    motor_disable_pwm();
}

#if (USE_IRQ_FOR_HALL == (1))
// this irq callback occurs when posif detects a new pattern 
__RAM_FUNC void POSIF0_0_IRQHandler(){
//void POSIF0_0_IRQHandler(){
        // Capture time stamp 
    ticks_hall_pattern_irq_last = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    // capture hall pattern
    current_hall_pattern_irq = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    current_hall_pattern_irq |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    current_hall_pattern_irq |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
}
#endif



// ************************************** begin of IRQ *************************
// *************** irq 0 of ccu8
__RAM_FUNC void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
//void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)

//void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
// here we just calculate the new compare values used for the 3 slices (0,1,2) that generates the 3 PWM
#if (USE_IRQ_FOR_HALL == (1))
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    // get the current ticks
    uint16_t current_speed_timer_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    // get the last changed pattern ticks (from posif irq)
    uint16_t last_hall_pattern_change_ticks = ticks_hall_pattern_irq_last;
    // get the current hall pattern as saved duting the posif irq
    current_hall_pattern = current_hall_pattern_irq;
    XMC_ExitCriticalSection(critical_section_value);
#else // irq0 when using a XMC_CCU4_SLICE_CAPTURE
    // get the current ticks
    //uint16_t current_speed_timer_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    uint16_t current_speed_timer_ticks = (uint16_t) HALL_SPEED_TIMER_HW->TIMER;
    // get the capture register = last changed pattern = current pattern
    uint16_t last_hall_pattern_change_ticks = (uint16_t) XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW , 1);
    // get the current hall pattern
    current_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(HALL_POSIF_HW) ;
#endif
    ui8_hall_sensors_state = current_hall_pattern; // duplicate just for easier maintenance of ebike_app.c for 860c (sent to display)

    // elapsed time between now and last pattern change (used for interpolation)
    uint16_t enlapsed_time =  current_speed_timer_ticks - last_hall_pattern_change_ticks ; // ticks between now and last change
    // elapse time since last pattern 1
//    uint16_t elapsed_ticks_since_pattern_1 = current_speed_timer_ticks - previous_360_ref_ticks;

//    uint8_t new_hall_pattern = 0; // filled only when a new pattern is detected

    // to debug time in irq
    //uint16_t start_ticks = current_speed_timer_ticks; // save to calculate enlased time inside the irq // just for debug could be removed
    #define DEBUG_IRQ0_INTERVALS (0) // 1 = calculate min and max intervals between 2 irq0
    #if (DEBUG_IRQ0_INTERVALS == (1))
    interval_ticks = current_speed_timer_ticks - prev_ticks;
    if (first_ticks == 0){
        if ( (interval_ticks <=13) || (interval_ticks >= 13)) {
            error_ticks_counter++;
            //error_ticks_value = current_speed_timer_ticks;
            //error_ticks_prev = prev_ticks;
            if (interval_ticks_min > interval_ticks) interval_ticks_min = interval_ticks;
            if (interval_ticks_max < interval_ticks) interval_ticks_max = interval_ticks;
             
        }
    } else {
        first_ticks = 0; 
    }
    prev_ticks = current_speed_timer_ticks ;
    #endif
//the resistance/gain in TSDZ8 is 4X smaller than in TSDZ2; still ADC is 12 bits instead of 10; so ADC 12bits TSDZ8 = ADC 10 bits TSDZ2
        // in TSDZ2, we used only the 8 lowest bits of adc; 1 adc step = 0,16A
        // In tsdz8, the resistance is (I expect) 0.003 Ohm ; So 1A => 0,003V => 0,03V (gain aop is 10)*4096/5Vcc = 24,576 steps
        //      SO 1 adc step 12bits = 1/24,576 = 0,040A
        // For 10 A, TSDZ2 should gives 10/0,16 = 62 steps
        // For 10 A, TSDZ8 shoud give 10*24,576 steps = 246 steps
        // to convert TSDZ8 steps 12bits  in the same units as TSDZ2, we shoud take ADC12bits *62/245,76 = 0,25 and divide by 4 (or >>2)
        // current is available in gr0 result 15 in queue 0 p2.8 and/or in gr1 result 152 (p2.8)
        // both results use IIR filters and so results are in 14 bits instead of 12 bits
        // use measurement from the 2 groups
    //ui32_adc_battery_current_15b = ((XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 15 ) & 0xFFFF) +
    //                                (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 15 ) & 0xFFFF)) ;  // So here result is in 15 bits (averaging
    // changed when using infineon init for vadc (result in 12bits and in ch 1)
    ui32_adc_battery_current_15b = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_I4_RESULT_REG ) & 0xFFFF) <<3; // change from 12 to 15 digits
    //accumulate the current to calculate an average on one rotation (there are quite big variations inside each 60° sector)
    ui32_adc_battery_current_15b_accum += ui32_adc_battery_current_15b;
    ui32_adc_battery_current_15b_counter++;
        ui32_adc_battery_current_15b_moving_average = update_moving_average(ui32_adc_battery_current_15b);

    adc_raw_a = (uint16_t)(XMC_VADC_GROUP_GetResult(VADC_I1_GROUP, VADC_I1_RESULT_REG) & 0x0FFF);
    adc_raw_b = (uint16_t)(XMC_VADC_GROUP_GetResult(VADC_I2_GROUP, VADC_I2_RESULT_REG) & 0x0FFF);
    adc_raw_c = (uint16_t)(XMC_VADC_GROUP_GetResult(VADC_I3_GROUP, VADC_I3_RESULT_REG) & 0x0FFF);

    
    // to see on prove scope oscillo
    //I_t = ui32_adc_battery_current_15b >> 3; 
    // when pattern change
    if ( current_hall_pattern != previous_hall_pattern) {
        if (current_hall_pattern != expected_pattern_table[previous_hall_pattern]){ // new pattern is not the expected one
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0x00
            ui8_hall_360_ref_valid = 0;  // reset the indicator saying no error for a 360° electric rotation 
            ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            hall_pattern_error_counter++; // for debuging
    //        SEGGER_RTT_printf(0, "error %u\r\n", hall_pattern_error_counter);
//            ui32_ref_angle = 0 ; // reset the position at pattern 1
            
        } else {   // valid transition
            if (current_hall_pattern ==  0x01) {  // rotor at 210°
                if (ui8_hall_360_ref_valid) { // check that we have a full rotation without pattern sequence error
                    ui16_hall_counter_total = last_hall_pattern_change_ticks - previous_360_ref_ticks; // save the total number of tick for one electric rotation
                    // when speed is still zero, calculate the first value without correction
//                    if (ui32_angle_per_tick_X16shift_new == 0) {
//                        ui32_angle_per_tick_X16shift_new = ( 1 << 24) / (ui16_interval_second_180_ticks << 1);
//                        ui32_ref_angle = 0; // reset the reference on 0 
//                    }
                    // calculate new reference angle for next tour = theoretical position when pattern change occured
                    // it is previous reference angle + enlapsed time between 2 pattern 1 * previous speed
                    // previous reference angle is ui32_angle_pattern_1_x16shift
                    // enlapsed time = ui16_hall_counter_total
                    // previous speed = ui32_angle_per_tick_X16shift
                    // ref angle should ideally be 0 (does not include the real angle of hall at pattern 1)
                    // new ref angle = ref angle * ticks for previous tour * speed used for previous turn
//                    uint32_t ui32_ref_angle_new = ui32_ref_angle + ui16_hall_counter_total * ui32_angle_per_tick_X16shift_new;
                    // when previous ref angle > 180° (1<<23), substract 360° (1<<24) (because e.g. 350° = in fact -10°)
//                    if (ui32_ref_angle > (1<<23)) ui32_ref_angle_new -= (1<<24);
                    // check that new reference is valid (must be between 180 and 270°)
                    // when unvalid, we reset the reference to 0 and calculate the speed based on second 180° tick (so without correction) 
//                    if (( ui32_ref_angle_new < (1<<23)) || ( ui32_ref_angle_new > ((1<<24) + (1<<23)))){
//                        ui32_ref_angle = 0;
//                        ui32_angle_per_tick_X16shift_new = ( 1 << 24) / (ui16_interval_second_180_ticks << 1);
//                    } else {
                        // new speed is calculated based on ((2*360) - ref_angle_new) / "expected tick interval for next 360°"
                        // expected tick interval for next 360° = 2 X second 180 ticks
//                        ui32_angle_per_tick_X16shift_new = ( (1 << 25) - ui32_ref_angle_new ) /  (ui16_interval_second_180_ticks << 1);
                        // for next turn we start with then new ref
//                        ui32_ref_angle = ui32_ref_angle_new ; // used as reference for all following extrapolation.
                        // still when ref_angle exceed 360, we have to substract 360°
//                        if (ui32_ref_angle > (1<<24) ) ui32_ref_angle -= (1<<24);
//                    }
                    // in all cases, set the angle reference in 8 bits for next calculation 
//                    ui8_motor_phase_absolute_angle_new = (uint8_t) (ui32_ref_angle >> 16) ; // reference for new algorithm
                    
                    ui32_angle_per_tick_X16shift = ((uint32_t) ( 1 << 24)) / ui16_hall_counter_total; // new value for interpolation and updating table with reference angle
                    ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES; // 0x80 ; it says that we can interpolate because speed is known
    

                }
                ui8_hall_360_ref_valid = 0x01;
                previous_360_ref_ticks = last_hall_pattern_change_ticks ;    
            } else if (current_hall_pattern ==  0x03) {  // rotor at 150°)
                // update ui8_g_foc_angle once every ERPS (used for g_foc_angle calculation) ;
                // I do not know why this is done when hall pattern = 0X03 and not with 0X01 to avoid a test
                            ui8_foc_flag = 1;
                    // when we have had at least one full rotation
                    if (ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES){
                        // Calculate avg current per rotation once per rotation
                        ui32_adc_battery_current_1_rotation_15b = ui32_adc_battery_current_15b_accum / ui32_adc_battery_current_15b_counter;
                        ui32_adc_battery_current_15b_accum =0;
                        ui32_adc_battery_current_15b_counter = 0;
                    }        
            }    
//            new_hall_pattern = current_hall_pattern;
            /*
            #define MIN_ANGLE_PER_TICK_X16SHIFT ((1 << 24) / 5000u ) // update of hall position is done only when erps > 50 = about rpm > 750)
            // 720 rpm = 12 rps = 12*4 erps = 48 erps => 250000 tick/sec /50 = 5000 ticks per electric rotation
            if ((current_hall_pattern >1) // no need to calculate for hall pattern == 1
                    && (ui32_angle_per_tick_X16shift > MIN_ANGLE_PER_TICK_X16SHIFT) // speed must be high (no enough inertia)
                    && (ui32_angle_per_tick_X16shift)  // angle per tick must be calculated
                    && (ui16_hall_counter_total == ui16_hall_counter_total_previous)){  // speed must be stable between 2 electric rotation
                
                uint16_t ui16_measured_angle_X16bits = ( (uint32_t) ((uint16_t) (last_hall_pattern_change_ticks - previous_360_ref_ticks))
                         * ui32_angle_per_tick_X16shift) >> 8;  // here we get an angle * 256 (to get better rounding for filtering)
                ui16_measured_angle_X16bits += ((uint16_t) ui8_hall_ref_angles[1]) << 8 ;   // add the angle for hall pattern 1 *256 (with overrunning)
                //ui8_best_ref_angles[current_hall_pattern] = filtering >> 8; // set angle back in 8bits
                uint8_t ui8_measured_angle = ui16_measured_angle_X16bits >> 8;
                uint8_t delta =  ui8_measured_angle - ui8_hall_ref_angles[current_hall_pattern];
                if ((delta < 5 ) || (delta > 251 )) { // when there is no big difference 
                    // apply filter on values X16bits and update best_ref_angles
                //    best_ref_angles_X16bits[current_hall_pattern] = filtering_function((uint32_t)ui16_measured_angle_X16bits , best_ref_angles_X16bits[current_hall_pattern] ,7 );
                //    ui8_best_ref_angles[current_hall_pattern] = (best_ref_angles_X16bits[current_hall_pattern] + 128 )>> 8; // +128 for rounding
                    
                    uint32_t filtering = best_ref_angles_X16bits[current_hall_pattern];
                    #define FILTER_HALL_POSITIONS 3
                    filtering = ((filtering << FILTER_HALL_POSITIONS) - filtering + (uint32_t) ui16_measured_angle_X16bits) >> FILTER_HALL_POSITIONS ;
                    best_ref_angles_X16bits[current_hall_pattern] = filtering ; // apply new filtered value
                    ui8_best_ref_angles[current_hall_pattern] = (filtering + 128 )>> 8; // +128 for rounding
                    
                    // update the table with reference angles
                    //ui8_hall_ref_angles[current_hall_pattern] = ui8_best_ref_angles[current_hall_pattern] ;
                    hall_ref_angles_counter++;  // just to debug to see if table is/should updated at regular intervals
                }
            } 
            */
            /*
            if (current_hall_pattern == 1 ){
                ui16_hall_counter_total_previous = ui16_hall_counter_total; // save previous counter (to check if erps is stable)
            }
            */    
        }
        previous_hall_pattern = current_hall_pattern; // saved to detect future change and check for valid transition
        // set rotor angle based on best ref angles
        //ui8_motor_phase_absolute_angle = ui8_best_ref_angles[current_hall_pattern]; // use best ref instead of ui8_hall_ref_angles[]
        // set rotor angle based on fixed hall ref angles
        ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[current_hall_pattern]; // use  hall_ref_angles[]
        #if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
        if ((ticks_intervals_status == 0) && (current_hall_pattern == 1) ) {
            ticks_intervals[0] = ui16_hall_counter_total ; // save the total ticks for previous 360°
            ticks_intervals[1] = last_hall_pattern_change_ticks - previous_hall_pattern_change_ticks ;
            ticks_intervals[7] = ui8_g_duty_cycle; // save the duty cycle
            ticks_intervals_status = 1;
        } else if (ticks_intervals_status == 1)  {
            ticks_intervals[current_hall_pattern] = last_hall_pattern_change_ticks - previous_hall_pattern_change_ticks ;
            if (current_hall_pattern == 5) {
                ticks_intervals_status = 2; // stop filling the intervals (wait the values are sent)
            }    
        } 
        previous_hall_pattern_change_ticks = last_hall_pattern_change_ticks;
        #endif
    
    

    } else { // no hall patern change
        // Verify if rotor stopped (< 10 ERPS)
        if (enlapsed_time > (HALL_COUNTER_FREQ/MOTOR_ROTOR_INTERPOLATION_MIN_ERPS/6)) { //  for TSDZ2: 250000/10 /6 = 4166 ; for TSDZ8 = 8332
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0
            ui8_g_foc_angle = 0;
            ui8_hall_360_ref_valid = 0;
            ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            ui16_hall_counter_total = 0xffff;
            // when we can not use accumulated value, use the latest on and reset accum
            ui32_adc_battery_current_1_rotation_15b = ui32_adc_battery_current_15b;
            ui32_adc_battery_current_15b_accum = 0;
            ui32_adc_battery_current_15b_counter = 0; 
        }
    }

    // mstrens : moved from irq1 to irq0 to use average current over 1 rotation for regulation
    //ui8_adc_battery_current_filtered  = ui32_adc_battery_current_1_rotation_15b >> 5 ; // from 15 bits to 10 bits like TSDZ2 
    ui8_adc_battery_current_filtered = ui32_adc_battery_current_15b_moving_average  >> 5;

    /****************************************************************************/
    // - calculate interpolation angle and sine wave table index when speed is known
    uint8_t ui8_interpolation_angle = 0; // interpolation angle
    uint32_t compensated_enlapsed_time = 0; 
//    uint8_t ui8_interpolation_angle_new = 0; // interpolation angle
//    uint32_t compensated_enlapsed_time_new = 0; 
    


    if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {  // as long as hall patern are OK and motor is running 
        compensated_enlapsed_time = enlapsed_time + ui8_fw_hall_counter_offset + ui8_hall_counter_offset;
        // convert time tick to angle (256 = 360°)
           //ui8_interpolation_angle = (((uint32_t) compensated_enlapsed_time) << 8) /  ui16_hall_counter_total; // <<8 = 256 = 360 electric angle
        // convert time tick to angle (256 = 360°) using the already calculated angle per tick (avoid a division)
        // add 1<<15 for better rounding
        ui8_interpolation_angle = ((((uint32_t) compensated_enlapsed_time) *  ui32_angle_per_tick_X16shift) + (1<<15) )>> 16 ; 
        //if (ui8_interpolation_angle > 90){  // added by mstrens because interpolation should not exceed 60°
        //    ui8_interpolation_angle = 21; // 21 is about 30° so mid position between 2 hall pattern changes
        //}
//        compensated_enlapsed_time_new = elapsed_ticks_since_pattern_1 + ui8_fw_hall_counter_offset + ui8_hall_counter_offset;
//        ui8_interpolation_angle_new = (((uint32_t) compensated_enlapsed_time_new) *  ui32_angle_per_tick_X16shift_new) >> 16 ; 
    }


    uint8_t rotor_angle_counts = ui8_interpolation_angle + ui8_motor_phase_absolute_angle +
                                 hall_reference_angle + FINE_TUNE_ANGLE_OFFSET;
    float theta_e = (float)rotor_angle_counts * (TWO_PI_F / 256.0f);
    float sin_theta = sinf(theta_e);
    float cos_theta = cosf(theta_e);

    if ((ui8_motor_commutation_type == BLOCK_COMMUTATION) || (!ui8_motor_enabled)) {
        ui8_g_foc_angle = rotor_angle_counts;
        ui8_controller_duty_cycle_target = 0;
        id_ref = 0.0f;
        iq_ref = 0.0f;
        id_int = 0.0f;
        iq_int = 0.0f;
        vd_est_prev = 0.0f;
        vq_est_prev = 0.0f;
        foc_subsample = 0U;
        ui16_adc_motor_phase_current = 0;
    } else {
        foc_subsample ^= 1U;
        if (foc_subsample) {
            float ia, ib, ic;
            phase_to_amps(&ia, &ib, &ic);

            float ialpha, ibeta, id, iq;
            clarke(ia, ib, ic, &ialpha, &ibeta);
            park(ialpha, ibeta, sin_theta, cos_theta, &id, &iq);

            const float battery_step_a = ((float)BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) / 100.0f;
            float iq_cmd = (float)ui8_controller_adc_battery_current_target * battery_step_a;
            if (!ui8_motor_enabled) {
                iq_cmd = 0.0f;
            }
            iq_ref = fminf(fmaxf(iq_cmd, -IQ_MAX_A), IQ_MAX_A);

            float vmag_prev = sqrtf(vd_est_prev * vd_est_prev + vq_est_prev * vq_est_prev);
            if (vmag_prev > FW_ENABLE_RATIO) {
                id_ref = fmaxf(id_ref - FW_ID_STEP_A, ID_FW_MIN_A);
            } else {
                id_ref = 0.0f;
            }

            float err_d = id_ref - id;
            float err_q = iq_ref - iq;

            id_int += err_d;
            iq_int += err_q;

            float we = TWO_PI_F * (float)ui16_motor_speed_erps;
            float vd = ID_KP * err_d + ID_KI * id_int - we * L_PHASE_HENRY * iq;
            float vq = IQ_KP * err_q + IQ_KI * iq_int + we * L_PHASE_HENRY * id + R_PHASE_OHM * iq;

            float vmag = sqrtf(vd * vd + vq * vq);
            const float vlim = VDQV_MAX_V;
            if (vmag > vlim) {
                float scale = vlim / vmag;
                vd *= scale;
                vq *= scale;
                id_int -= err_d;
                iq_int -= err_q;
            }

            vd_est_prev = vd;
            vq_est_prev = vq;

            float valpha, vbeta;
            inv_park(vd, vq, sin_theta, cos_theta, &valpha, &vbeta);

            float angle = atan2f(vbeta, valpha);
            if (angle < 0.0f) {
                angle += TWO_PI_F;
            }
            uint8_t angle_counts = (uint8_t)lroundf(angle * (256.0f / TWO_PI_F));
            ui8_g_foc_angle = angle_counts;

            float mod_idx = sqrtf(valpha * valpha + vbeta * vbeta);
            float duty_norm = (vlim > 0.0f) ? fminf(mod_idx, vlim) / vlim : 0.0f;
            uint8_t duty_counts = (uint8_t)lroundf(fminf(duty_norm * 254.0f, 254.0f));
            ui8_controller_duty_cycle_target = duty_counts;

            float iq_abs = fabsf(iq);
            float phase_current_adc = iq_abs / battery_step_a;
            if (phase_current_adc > 1023.0f) {
                phase_current_adc = 1023.0f;
            }
            ui16_adc_motor_phase_current = (uint16_t)phase_current_adc;
        }
    }

    uint8_t ui8_svm_table_index = ui8_g_foc_angle;
    // Phase A is advanced 240 degrees over phase B
    ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 171)]; // 171 = 240 deg when 360° is coded as 256
    if (ui16_temp > MIDDLE_SVM_TABLE) { // 214 at 19 khz
        ui16_a = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8); // >>8 because duty_cycle 100% is 256
    } else {
        ui16_a = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }
    // phase B as reference phase
    ui16_temp = ui16_svm_table[ui8_svm_table_index] ;
    if (ui16_temp > MIDDLE_SVM_TABLE) {
        ui16_b = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8);
    } else {
        ui16_b = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }
    // phase C is advanced 120 degrees over phase B
    ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 85 )] ; // 85 = 120 deg
    if (ui16_temp > MIDDLE_SVM_TABLE) {
        ui16_c = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8);
    } else {
        ui16_c = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }

    #define DEBUG_IRQO_TIME (0) // 1 = calculate the time spent in irq0
    #if (DEBUG_IRQO_TIME == (1))
    uint16_t temp  = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) ;
    temp = temp - current_speed_timer_ticks;
    if (irq0_min > temp) irq0_min = temp; // store the in enlapsed time in the irq
    if (irq0_max < temp) irq0_max = temp; // store the max enlapsed time in the irq
    #endif


    // get the voltage ; done in irq0 because it is used in irq1 and irq0 takes less time
        //ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0x0FFF) >> 2; // battery gr1 ch6 result 4
    // changed to take care of infineon VADC init (result in reg 6)
    ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , VADC_VDC_RESULT_REG ) & 0x0FFF) >> 2; // battery gr1 ch6 result 6
          
    
    #if (uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
    I_u = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF;
    I_w = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF;
    I_v = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF;
    
    ProbeScope_Sampling(); // this should be moved e.g. in a interrupt that run faster
    #endif

} // end of CCU80_0_IRQHandler

#define DEBUG_IRQ1_TIME (0) // 1 = calculate time spent in irq1
// ************* irq handler 
__RAM_FUNC void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)    
//void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)    
    #if (DEBUG_IRQ1_TIME == (1))
    // to debug max time in this iSR
    uint16_t start_ticks  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    #endif

    // fill the PWM parameters with the values calculated in the other CCU8 interrupt
    //XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_a);
    //XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_V_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_b);
    //XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_W_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_c);
    PHASE_U_TIMER_HW->CR1S = (uint32_t) ui16_a;
    PHASE_V_TIMER_HW->CR1S = (uint32_t) ui16_b;
    PHASE_W_TIMER_HW->CR1S = (uint32_t) ui16_c;
    /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel */
	//XMC_CCU8_EnableShadowTransfer(ccu8_0_HW, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
	//                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
	//                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 ));
    ccu8_0_HW->GCSS = ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 );
    
    
    /****************************************************************************/
        // Read all ADC values (right aligned values).
       // adc values are reduced to 10 bits instead of 12 bits to use the same resolution as tsdz2
       // note: per vadc group, the result number is the same as the pin number (except for group 1 current sensor)
        // next line has been moved in irq 0 to save time here
        //ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0xFFF) >> 2; // battery gr1 ch6 result 4   in bg
        // next line has been moved to ebike_app.c to save time here
        //ui16_adc_torque   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 2 ) & 0xFFF) >> 2; // torque gr0 ch7 result 2 in bg p2.2
        // next line has been moved to ebike_app.c to save time in this irq
        //ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0xFFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5
        
        // update foc_angle once per electric rotation (based on fog_flag
        // foc_angle is added to the position given by hall sensor + interpolation )
        if (ui8_g_duty_cycle == 0) {
            ui16_adc_motor_phase_current = 0;
        }
        ui8_foc_flag = 0;

        // original perform also a save of some parameters (battery consumption) // to do 
    #if (DEBUG_IRQ1_TIME == (1))
    uint16_t temp1  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    temp1 = temp1 - start_ticks;
    if (irq1_min > temp1) irq1_min = temp1; // store the min enlapsed time in the irq
    if (irq1_max < temp1) irq1_max = temp1; // store the min enlapsed time in the irq
    #endif
    
    // added by mstrens to calculate torque sensor without cyclic effect using the max per current and previous rotation
    // we have several data
    // ui16_adc_torque_filtered is the actual filtered ADC torque
    // ui16_adc_torque_actual_rotation is the max during current rotation
    // ui16_adc_torque_previous_rotation is the max during previous rotation
    // ui8_pas_counter count the number of transition to detect a 360° pedal rotation

    // first reset the values per rotation when requested by ebike_app.c (because cadence is lower than a threshold)
    if (ui8_adc_torque_rotation_reset) {
        ui8_adc_torque_rotation_reset = 0; //reset the flag
        ui16_adc_torque_actual_rotation = 0;  
        ui16_adc_torque_previous_rotation = 0;
        ui8_pas_counter = 0; // reset the counter also
    }
    if (ui16_cadence_sensor_ticks > 0) { // when we have a cadence, we update data over rotation
        // actual_rotation is the max
        if (ui16_adc_torque_actual_rotation < ui16_adc_torque_filtered) ui16_adc_torque_actual_rotation = ui16_adc_torque_filtered;
        if (ui8_pas_counter >= 20) { // if we have had a full rotation
            ui8_pas_counter = 0; // reset the counter
            ui16_adc_torque_previous_rotation = ui16_adc_torque_actual_rotation;  // save the actual rotation value
            ui16_adc_torque_actual_rotation =  0; // reset the actual rotation
        }
    } else {
        ui8_pas_counter = 0 ;
        ui16_adc_torque_previous_rotation = 0;
        ui16_adc_torque_actual_rotation = 0;
    }


}  // end of CCU8_1_IRQ


void motor_enable_pwm(void) { //set posif with current position & restart the timers
    get_hall_pattern(); // refresh hall pattern in current_hall_pattern
    
    // one solution to activate is to generate an event that starts all timers in a synchronized way
    // Enable Global Start Control CCU80  in a synchronized way*/
    XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
    XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    uint32_t retry_start_counter = 10;
    while ((!XMC_CCU8_SLICE_IsTimerRunning(PHASE_U_TIMER_HW)) && (retry_start_counter > 0)){ // to be sure it is running
        XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
        XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    }
}

void motor_disable_pwm(void) {
    // we stop and clear the 3 timers that control motor PWM
    XMC_CCU8_SLICE_StopClearTimer(PHASE_U_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_V_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_W_TIMER_HW);
    // slice CCU8_3 is not stopped becauses it is required to manage some tasks (speed, torque,...) 
}

void get_hall_pattern(){  // use to initialise at power on and in motor_enable()
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    current_hall_pattern = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
    XMC_ExitCriticalSection(critical_section_value);
}

