#ifndef __HALL_H
#define __HALL_H

#include <stdint.h>
#include "gpio.h"
#include "foc.h"

void    Hall_Init(void);
void    Hall_Update(void);
float   Get_Hall_Electrical_Angle(void);
uint8_t Get_Hall_Count(void);

/* PLL 锁相环输出接口 */
float   Get_PLL_Angle(void);
float   Get_PLL_Speed(void);

#endif
