// foc_motor_control.c  — single-file build with switchable implementations
// Toggle in main.h:  #define FOC_USE_NEW 1   // 1=new algo, 0=legacy algo

#include "main.h"
#include "foc_motor_control.h"
#include <arm_math.h>
#include "motor.h"
#include "adc.h"

// Safety typedef in case the header wasn't updated yet.
// Prefer putting this in foc_motor_control.h, but this guard avoids duplicate typedefs.
#ifndef __ELEC_ANGLE_T_DEFINED__
typedef uint16_t elec_angle_t;
#endif

// --- Externs from your project (must NOT be 'static' in motor.c)
extern volatile uint8_t  ui8_hall_state;
extern volatile uint8_t  motor_direction;
extern volatile uint16_t ui16_hall_counter;
extern volatile uint16_t ui16_hall_counter_log;
extern const uint16_t hall_position_angle_forward[8];
extern const uint16_t hall_position_angle_reverse[8];

#if (FOC_USE_NEW)

// ============================== NEW IMPLEMENTATION ==============================
// Q31 sin/cos, clean Clarke/Park, PI anti-windup, vector limiter, PWM clamps.

static const q15_t K_NEG_HALF        = (q15_t)FLOAT_TO_Q15(-0.5f);
static const q15_t K_SQRT3_DIV_2     = (q15_t)FLOAT_TO_Q15(SQRT3_DIV_2);
static const q15_t K_ONE_OVER_SQRT3  = (q15_t)FLOAT_TO_Q15(ONE_DIV_SQRT3);
static inline q31_t q31_from_elec(elec_angle_t a16) { return ((q31_t)a16) << 15; } // 0..65535 → 0..2π

// ---- HAL: ADC currents (U,W), hall angle, PWM apply ----
static void foc_hardware_read_currents(int16_t* Iu_raw, int16_t* Iw_raw) {
    *Iu_raw = (int16_t)XMC_VADC_GROUP_GetResult(VADC_I_U_GROUP, VADC_I_U_RESULT_REG);
    *Iw_raw = (int16_t)XMC_VADC_GROUP_GetResult(VADC_I_W_GROUP, VADC_I_W_RESULT_REG);
}

static elec_angle_t foc_hardware_get_electrical_angle(void) {
    uint16_t dur = ui16_hall_counter_log, now = ui16_hall_counter;
    uint16_t coarse = (motor_direction == MOTOR_DIRECTION_FORWARD)
    ? hall_position_angle_forward[ui8_hall_state]
    : hall_position_angle_reverse[ui8_hall_state];
    uint16_t fine = (dur > 50u && now < dur) ? (uint16_t)(((uint32_t)now * 60u)/dur) : 0u;
    uint16_t deg = coarse + fine; if (deg >= 360u) deg -= 360u;
    return (elec_angle_t)(((uint32_t)deg * 65536u) / 360u);
}

static inline void pwm_apply(uint16_t u, uint16_t v, uint16_t w) {
    XMC_CCU4_SLICE_SetTimerCompareMatch(SLICE_U, u);
    XMC_CCU4_SLICE_SetTimerCompareMatch(SLICE_V, v);
    XMC_CCU4_SLICE_SetTimerCompareMatch(SLICE_W, w);
}

// ---- Control helpers ----
static inline uint16_t clamp_pwm(int32_t x){
    if (x < 1) return 1; // keep some pulse (dead-time guard)
    if (x >= PWM_PERIOD_TICKS) return PWM_PERIOD_TICKS - 1;
    return (uint16_t)x;
}

static void clarke_transform(q15_t Iu, q15_t Iv, q15_t* I_alpha, q15_t* I_beta){
    *I_alpha = Iu;
    // Iβ = (Iu + 2*Iv)/√3
    q15_t tmp = (q15_t)((int32_t)Iu + ((int32_t)Iv << 1));
    *I_beta = Q15_MUL(K_ONE_OVER_SQRT3, tmp);
}

static void park_transform(q15_t I_alpha, q15_t I_beta, elec_angle_t angle, q15_t* Id, q15_t* Iq){
    q31_t s, c; arm_sin_cos_q31(q31_from_elec(angle), &s, &c);
    q15_t S = (q15_t)(s >> 16), C = (q15_t)(c >> 16);
    *Id = Q15_MUL(I_alpha, C) + Q15_MUL(I_beta,  S);
    *Iq = Q15_MUL(I_beta,  C) - Q15_MUL(I_alpha, S);
}

static void inverse_park_transform(q15_t Vd, q15_t Vq, elec_angle_t angle, q15_t* V_alpha, q15_t* V_beta){
    q31_t s, c; arm_sin_cos_q31(q31_from_elec(angle), &s, &c);
    q15_t S = (q15_t)(s >> 16), C = (q15_t)(c >> 16);
    *V_alpha = Q15_MUL(Vd, C) - Q15_MUL(Vq, S);
    *V_beta  = Q15_MUL(Vd, S) + Q15_MUL(Vq, C);
}

// Cheap magnitude limiter to avoid saturation (L∞→L2 approx)
static void limit_vector_q15(q15_t* Va, q15_t* Vb, q15_t vmax){
    q15_t a=*Va, b=*Vb;
    q15_t aa = (a>=0)?a:(q15_t)-a, bb=(b>=0)?b:(q15_t)-b;
    q15_t hi = (aa>bb)?aa:bb, lo=(aa>bb)?bb:aa;
    q15_t mag = hi + (lo>>1);
    if (mag > vmax && mag > 0){
        *Va = (q15_t)(((int32_t)*Va * vmax) / mag);
        *Vb = (q15_t)(((int32_t)*Vb * vmax) / mag);
    }
}

// Sinusoidal PWM mapping (not true SVM, but fine to start)
static void sine_pwm(q15_t V_alpha, q15_t V_beta, uint16_t* pwm_u, uint16_t* pwm_v, uint16_t* pwm_w){
    q15_t Vu = V_alpha;
    q15_t Vv = Q15_MUL(K_NEG_HALF, V_alpha) + Q15_MUL(K_SQRT3_DIV_2, V_beta);
    q15_t Vw = Q15_MUL(K_NEG_HALF, V_alpha) - Q15_MUL(K_SQRT3_DIV_2, V_beta);
    int32_t half = (PWM_PERIOD_TICKS >> 1);
    int32_t du = ((int32_t)Vu * half) >> 15; du += half;
    int32_t dv = ((int32_t)Vv * half) >> 15; dv += half;
    int32_t dw = ((int32_t)Vw * half) >> 15; dw += half;
    *pwm_u = clamp_pwm(du);
    *pwm_v = clamp_pwm(dv);
    *pwm_w = clamp_pwm(dw);
}

// PI with conditional anti-windup
static q15_t pid_run_aw(PID_Controller_t* p, q15_t e){
    q15_t up = Q15_MUL(p->Kp, e);
    q15_t ui = p->integral_term + Q15_MUL(p->Ki, e);
    q15_t u  = up + ui;
    q15_t us = u;
    if (us > p->output_max) us = p->output_max; else if (us < p->output_min) us = p->output_min;
    if (!((u > p->output_max && e > 0) || (u < p->output_min && e < 0))) p->integral_term = ui;
    return us;
}

// ---- Public API ----
void foc_init(FOC_Controller_t* foc){
    memset(foc, 0, sizeof(*foc));
    q15_t max_out = FLOAT_TO_Q15(0.95f);
    pid_init(&foc->pid_q, FLOAT_TO_Q15(0.15f), FLOAT_TO_Q15(0.02f), max_out);
    pid_init(&foc->pid_d, FLOAT_TO_Q15(0.15f), FLOAT_TO_Q15(0.02f), max_out);
    foc->Iq_target_q15 = 0;
    foc->Id_target_q15 = 0;
}

void foc_run_control_loop(FOC_Controller_t* foc){
    // 1) angle
    foc->electrical_angle = foc_hardware_get_electrical_angle();

    // 2) currents (to Q15)
    int16_t Iu_raw, Iw_raw; foc_hardware_read_currents(&Iu_raw, &Iw_raw);
    foc->I_u = (q15_t)(((int32_t)Iu_raw - (MAX_ADC_VALUE/2)) * Q15_ONE / (MAX_ADC_VALUE/2));
    foc->I_w = (q15_t)(((int32_t)Iw_raw - (MAX_ADC_VALUE/2)) * Q15_ONE / (MAX_ADC_VALUE/2));
    foc->I_v = (q15_t)(-foc->I_u - foc->I_w);

    // 3) αβ
    clarke_transform(foc->I_u, foc->I_v, &foc->I_alpha, &foc->I_beta);

    // 4) dq
    park_transform(foc->I_alpha, foc->I_beta, foc->electrical_angle, &foc->Id, &foc->Iq);

    // 5) PI
    foc->Vd = pid_run_aw(&foc->pid_d, (q15_t)(foc->Id_target_q15 - foc->Id));
    foc->Vq = pid_run_aw(&foc->pid_q, (q15_t)(foc->Iq_target_q15 - foc->Iq));

    // 6) back to αβ
    inverse_park_transform(foc->Vd, foc->Vq, foc->electrical_angle, &foc->V_alpha, &foc->V_beta);

    // 6.5) limit vector magnitude
    limit_vector_q15(&foc->V_alpha, &foc->V_beta, FLOAT_TO_Q15(0.95f));

    // 7) PWM
    sine_pwm(foc->V_alpha, foc->V_beta, &foc->pwm_u, &foc->pwm_v, &foc->pwm_w);

    // 8) apply
    pwm_apply(foc->pwm_u, foc->pwm_v, foc->pwm_w);
}

#else // ============================== LEGACY IMPLEMENTATION =======================

// This is your original version (lightly cleaned), so the toggle is 1-file.

static void foc_hardware_read_currents(int16_t* Iu_raw, int16_t* Iw_raw) {
    *Iu_raw = (int16_t)XMC_VADC_GROUP_GetResult(VADC_I_U_GROUP, VADC_I_U_RESULT_REG);
    *Iw_raw = (int16_t)XMC_VADC_GROUP_GetResult(VADC_I_W_GROUP, VADC_I_W_RESULT_REG);
}

static elec_angle_t foc_hardware_get_electrical_angle(void) {
    uint16_t coarse_angle_degrees;
    uint16_t hall_duration = ui16_hall_counter_log;
    uint16_t hall_timer_now = ui16_hall_counter;

    if (motor_direction == MOTOR_DIRECTION_FORWARD) {
        coarse_angle_degrees = hall_position_angle_forward[ui8_hall_state];
    } else {
        coarse_angle_degrees = hall_position_angle_reverse[ui8_hall_state];
    }

    uint16_t fine_angle_degrees = 0;
    if (hall_duration > 50u && hall_timer_now < hall_duration) {
        fine_angle_degrees = (uint16_t)(((uint32_t)hall_timer_now * 60u) / hall_duration);
    }

    uint16_t final_angle_degrees = coarse_angle_degrees + fine_angle_degrees;
    if (final_angle_degrees >= 360u) final_angle_degrees -= 360u;

    return (elec_angle_t)(((uint32_t)final_angle_degrees * 65536u) / 360u);
}

static void foc_hardware_update_pwm(uint16_t pwm_u, uint16_t pwm_v, uint16_t pwm_w) {
    XMC_CCU4_SLICE_SetTimerCompareMatch(SLICE_U, pwm_u);
    XMC_CCU4_SLICE_SetTimerCompareMatch(SLICE_V, pwm_v);
    XMC_CCU4_SLICE_SetTimerCompareMatch(SLICE_W, pwm_w);
}

static void pid_init(PID_Controller_t* pid, q15_t Kp, q15_t Ki, q15_t output_max) {
    pid->Kp = Kp; pid->Ki = Ki; pid->integral_term = 0; pid->previous_error = 0;
    pid->output_max = output_max; pid->output_min = -output_max;
}

static q15_t pid_run(PID_Controller_t* pid, q15_t error) {
    pid->integral_term += Q15_MUL(pid->Ki, error);
    if (pid->integral_term > pid->output_max) pid->integral_term = pid->output_max;
    else if (pid->integral_term < pid->output_min) pid->integral_term = pid->output_min;
    q15_t output = Q15_MUL(pid->Kp, error) + pid->integral_term;
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;
    return output;
}

static void clarke_transform(q15_t Iu, q15_t Iv, q15_t* I_alpha, q15_t* I_beta) {
    *I_alpha = Iu;
    *I_beta = Q15_MUL(FLOAT_TO_Q15(ONE_DIV_SQRT3), (q15_t)((int32_t)Iu + ((int32_t)Iv << 1)));
}

static void park_transform(q15_t I_alpha, q15_t I_beta, elec_angle_t angle, q15_t* Id, q15_t* Iq) {
    // Legacy used q15 trig with <<16; we keep equivalent behavior by converting to q31 then downscaling
    q31_t s, c; arm_sin_cos_q31(((q31_t)angle) << 15, &s, &c);
    q15_t sin_theta = (q15_t)(s >> 16), cos_theta = (q15_t)(c >> 16);
    *Id = Q15_MUL(I_alpha, cos_theta) + Q15_MUL(I_beta, sin_theta);
    *Iq = Q15_MUL(I_beta,  cos_theta) - Q15_MUL(I_alpha, sin_theta);
}

static void inverse_park_transform(q15_t Vd, q15_t Vq, elec_angle_t angle, q15_t* V_alpha, q15_t* V_beta) {
    q31_t s, c; arm_sin_cos_q31(((q31_t)angle) << 15, &s, &c);
    q15_t sin_theta = (q15_t)(s >> 16), cos_theta = (q15_t)(c >> 16);
    *V_alpha = Q15_MUL(Vd, cos_theta) - Q15_MUL(Vq, sin_theta);
    *V_beta  = Q15_MUL(Vd, sin_theta) + Q15_MUL(Vq, cos_theta);
}

static void space_vector_modulation(q15_t V_alpha, q15_t V_beta, uint16_t* pwm_u, uint16_t* pwm_v, uint16_t* pwm_w) {
    q15_t Vu_q15 = V_alpha;
    q15_t Vv_q15 = Q15_MUL(FLOAT_TO_Q15(-0.5f), V_alpha) + Q15_MUL(FLOAT_TO_Q15(SQRT3_DIV_2), V_beta);
    q15_t Vw_q15 = Q15_MUL(FLOAT_TO_Q15(-0.5f), V_alpha) - Q15_MUL(FLOAT_TO_Q15(SQRT3_DIV_2), V_beta);
    *pwm_u = (uint16_t)((((int32_t)Vu_q15 * (PWM_PERIOD_TICKS >> 1)) >> 15) + (PWM_PERIOD_TICKS >> 1));
    *pwm_v = (uint16_t)((((int32_t)Vv_q15 * (PWM_PERIOD_TICKS >> 1)) >> 15) + (PWM_PERIOD_TICKS >> 1));
    *pwm_w = (uint16_t)((((int32_t)Vw_q15 * (PWM_PERIOD_TICKS >> 1)) >> 15) + (PWM_PERIOD_TICKS >> 1));
    if (*pwm_u >= PWM_PERIOD_TICKS) *pwm_u = PWM_PERIOD_TICKS - 1;
    if (*pwm_v >= PWM_PERIOD_TICKS) *pwm_v = PWM_PERIOD_TICKS - 1;
    if (*pwm_w >= PWM_PERIOD_TICKS) *pwm_w = PWM_PERIOD_TICKS - 1;
}

// ---- Public API (legacy flow) ----
void foc_init(FOC_Controller_t* foc) {
    memset(foc, 0, sizeof(*foc));
    q15_t max_output = FLOAT_TO_Q15(0.95f);
    pid_init(&foc->pid_q, FLOAT_TO_Q15(0.15f), FLOAT_TO_Q15(0.02f), max_output);
    pid_init(&foc->pid_d, FLOAT_TO_Q15(0.15f), FLOAT_TO_Q15(0.02f), max_output);
    foc->Iq_target_q15 = 0;
    foc->Id_target_q15 = 0;
}

void foc_run_control_loop(FOC_Controller_t* foc) {
    foc->electrical_angle = foc_hardware_get_electrical_angle();

    int16_t Iu_raw, Iw_raw; foc_hardware_read_currents(&Iu_raw, &Iw_raw);
    foc->I_u = (q15_t)(((int32_t)Iu_raw - (MAX_ADC_VALUE / 2)) * Q15_ONE / (MAX_ADC_VALUE / 2));
    foc->I_w = (q15_t)(((int32_t)Iw_raw - (MAX_ADC_VALUE / 2)) * Q15_ONE / (MAX_ADC_VALUE / 2));
    foc->I_v = (q15_t)(-foc->I_u - foc->I_w);

    clarke_transform(foc->I_u, foc->I_v, &foc->I_alpha, &foc->I_beta);
    park_transform(foc->I_alpha, foc->I_beta, foc->electrical_angle, &foc->Id, &foc->Iq);

    q15_t Id_error = (q15_t)(foc->Id_target_q15 - foc->Id);
    q15_t Iq_error = (q15_t)(foc->Iq_target_q15 - foc->Iq);
    foc->Vd = pid_run(&foc->pid_d, Id_error);
    foc->Vq = pid_run(&foc->pid_q, Iq_error);

    inverse_park_transform(foc->Vd, foc->Vq, foc->electrical_angle, &foc->V_alpha, &foc->V_beta);
    space_vector_modulation(foc->V_alpha, foc->V_beta, &foc->pwm_u, &foc->pwm_v, &foc->pwm_w);
    foc_hardware_update_pwm(foc->pwm_u, foc->pwm_v, foc->pwm_w);
}

#endif // FOC_USE_NEW
