#ifndef __PWM_H
#define __PWM_H

#include "tim.h"

void PWM_Init(void);
void Set_PWM(float Ua, float Ub, float Uc);

#endif
