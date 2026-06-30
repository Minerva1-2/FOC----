#include "drv8301.h"

//开启DRV8301芯片
void DRV8301_Init(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}
//DRV8301芯片故障
uint8_t DRV8301_Fault(void)
{
	uint8_t fault_flag = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	
	return fault_flag;
		
}