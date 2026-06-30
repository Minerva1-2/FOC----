#include "control_timer.h"

extern TIM_HandleTypeDef htim2;

void Control_Timer_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}
