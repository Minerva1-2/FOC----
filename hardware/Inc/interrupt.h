#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include <stdint.h>
#include "tim.h"
#include "usbd_cdc_if.h"

#define SPEED_KP          0.15f
#define SPEED_KI          0.05f
#define SPEED_IQ_LIMIT    5.0f
#define SPEED_UQ_LIMIT    SPEED_IQ_LIMIT

#define CURRENT_KP        0.4f
#define CURRENT_KI        0.15f
#define CURRENT_UQ_LIMIT  VOLTAGE_POWER_SUPPLY / 2

#define RAMP_DURATION     3.0f
#define RAMP_START_SPEED  20.0f
#define RAMP_VOLTAGE      3.0f

extern float target_speed;
extern float uq_output;
extern float angle_el;
void Pid_Control_Init(void);

extern volatile uint16_t current_adc_a;
extern volatile uint16_t current_adc_b;
extern volatile uint16_t current_adc_c;
extern volatile float phase_current_a;
extern volatile float phase_current_b;
extern volatile float phase_current_c;
extern volatile uint8_t current_sample_ready;

extern volatile float iq_ref;
extern volatile float id_ref;
extern volatile float measured_id;
extern volatile float measured_iq;
extern volatile float vd_output;
extern volatile float vq_output;

void Current_Sense_Start(void);

#endif
