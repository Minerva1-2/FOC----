#include "pwm.h"
#include "foc.h"

float dc_a = 0.0f,dc_b = 0.0f,dc_c = 0.0f;

/* 开启六路PWM */
void PWM_Init(void)
{	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
/* 设置PWM驱动电机旋转 */
void Set_PWM(float Ua, float Ub, float Uc)
{
	dc_a = _constrain(Ua / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    dc_b = _constrain(Ub / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    dc_c = _constrain(Uc / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
	
	htim1.Instance->CCR1 = (dc_a * PWM_PERIOD);
	htim1.Instance->CCR2 = (dc_b * PWM_PERIOD);
	htim1.Instance->CCR3 = (dc_c * PWM_PERIOD);
}