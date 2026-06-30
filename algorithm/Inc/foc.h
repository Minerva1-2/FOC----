#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>

#define _constrain(value, min, max)     ((value >= min) ? ((value <= max) ? value : max) : min)
#define PI_ANGLE                        3.14159265f
#define FOC_TS                          100e-6f
#define PWM_PERIOD                      4200
#define VOLTAGE_POWER_SUPPLY            24.0f
#define PHASE_VOLTAGE_FILTER_ALPHA 		0.25f

extern float Ua, Ub, Uc;

typedef struct
{
    float alpha;
    float beta;
} Clark_t;

typedef struct
{
    float Id;
    float Iq;
} Park_t;

float normalizeAngle(float angle);
void Set_Phase_Voltage(float Uq, float Ud, float angle_el);
void SVPWM(float Ualpha, float Ubeta);
Clark_t *Clark(float Ia, float Ib, float Ic);
Park_t *Park(float Ialpha, float Ibeta, float angle_el);

#endif
