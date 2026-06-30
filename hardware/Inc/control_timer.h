#ifndef __CONTROL_TIMER_H
#define __CONTROL_TIMER_H

#include "tim.h"

/* FOC 控制环频率 (Hz)，在 CubeMX 中通过 TIM2 的 ARR 调整 */
#define FOC_CONTROL_FREQ  10000

void Control_Timer_Start(void);

#endif
