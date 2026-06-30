#include <math.h>
#include "interrupt.h"
#include "hall.h"
#include "foc.h"
#include "pid.h"
#include "current.h"

volatile uint16_t current_adc_a = 0;
volatile uint16_t current_adc_b = 0;
volatile uint16_t current_adc_c = 0;
volatile float phase_current_a = 0.0f;
volatile float phase_current_b = 0.0f;
volatile float phase_current_c = 0.0f;
volatile uint8_t current_sample_ready = 0;

float target_speed = 0.0f;
float uq_output = 0.0f;
float angle_el;

volatile float iq_ref = 0.0f;
volatile float id_ref = 0.0f;
volatile float measured_id = 0.0f;
volatile float measured_iq = 0.0f;
volatile float vd_output = 0.0f;
volatile float vq_output = 0.0f;

static PID_t speed_pid;
static PID_t id_pid;
static PID_t iq_pid;
static float ramp_angle = 0.0f;
static float elapsed_time = 0.0f;
static uint8_t closed_loop = 0;

static Clark_t *transform_clark;
static Park_t *transform_park;

void Pid_Control_Init(void)
{
    PID_Init(&speed_pid, SPEED_KP, SPEED_KI, 0.0f, SPEED_UQ_LIMIT);
    PID_Init(&id_pid, CURRENT_KP, CURRENT_KI, 0.0f, CURRENT_UQ_LIMIT);
    PID_Init(&iq_pid, CURRENT_KP, CURRENT_KI, 0.0f, CURRENT_UQ_LIMIT);

    ramp_angle = 0.0f;
    elapsed_time = 0.0f;
    closed_loop = 0;
    iq_ref = 0.0f;
    id_ref = 0.0f;
    uq_output = 0.0f;
}

static void Current_Loop_Run(void)
{
    angle_el = Get_Hall_Electrical_Angle();

    transform_clark = Clark(phase_current_a, phase_current_b, phase_current_c);
    transform_park = Park(transform_clark->alpha, transform_clark->beta, angle_el);

    measured_id = transform_park->Id;
    measured_iq = transform_park->Iq;

    if (!closed_loop)
    {
        PID_Reset(&id_pid);
        PID_Reset(&iq_pid);
        vd_output = 0.0f;
        vq_output = 0.0f;
        return;
    }

    vd_output = PID_Compute(&id_pid, id_ref - measured_id);
    vq_output = PID_Compute(&iq_pid, iq_ref - measured_iq);
    uq_output = vq_output;

    Set_Phase_Voltage(vq_output, vd_output, angle_el);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
        return;

    current_adc_a = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    current_adc_b = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    current_adc_c = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    if (!current_offset_calibrated)
    {
        Current_Offset_Update(current_adc_a, current_adc_b, current_adc_c);
        phase_current_a = 0.0f;
        phase_current_b = 0.0f;
        phase_current_c = 0.0f;
        current_sample_ready = 0;
        return;
    }

    phase_current_a = Current_Adc_To_Ampere(current_adc_a, 0U);
    phase_current_b = Current_Adc_To_Ampere(current_adc_b, 1U);

    /* Use two measured phases for control; keep phase C consistent. */
    phase_current_c = -phase_current_a - phase_current_b;

    Current_Loop_Run();
    current_sample_ready = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
        return;

    Hall_Update();
    elapsed_time += FOC_TS;

    if (elapsed_time < RAMP_DURATION)
    {
        ramp_angle += RAMP_START_SPEED * FOC_TS;
        if (ramp_angle >= 2.0f * PI_ANGLE)
            ramp_angle -= 2.0f * PI_ANGLE;

        Set_Phase_Voltage(0.0f, RAMP_VOLTAGE, ramp_angle);
        iq_ref = 0.0f;
        uq_output = 0.0f;
        closed_loop = 0;
        return;
    }

    if (!closed_loop)
    {
        PID_Reset(&speed_pid);
        PID_Reset(&id_pid);
        PID_Reset(&iq_pid);
        closed_loop = 1;
    }

    float actual_speed = Get_PLL_Speed();
    float speed_error = target_speed - actual_speed;

    iq_ref = PID_Compute(&speed_pid, speed_error);

    if (target_speed == 0.0f && fabsf(actual_speed) < 1.0f)
    {
        PID_Reset(&speed_pid);
        PID_Reset(&id_pid);
        PID_Reset(&iq_pid);
        iq_ref = 0.0f;
        uq_output = 0.0f;
    }
}
