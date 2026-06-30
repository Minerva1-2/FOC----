#ifndef __PLL_OBSERVER_H
#define __PLL_OBSERVER_H

#include <stdint.h>

/* 电机极对数 - 请根据你的电机规格修改 */
#define POLE_PAIRS  7

/* PLL 采样周期，与 FOC 控制周期一致 (100us = 10kHz) */
#define PLL_TS      100e-6f

/**
  * @brief  初始化 PLL 观测器
  * @param  initial_angle  初始电角度 (rad)，通常从霍尔传感器获取
  */
void PLL_Init(float initial_angle);

/**
  * @brief  PLL 更新，每个控制周期 (10kHz) 调用一次
  *         内部包含 NCO 角度推算 + PI 鉴相校正
  * @param  theta_hall  霍尔插值得到的参考电角度 (rad)
  * @retval 当前 PLL 输出的平滑电角度 (rad), 范围 [0, 2*PI)
  */
float PLL_Update(float theta_hall);

/**
  * @brief  获取 PLL 估算的电角速度 (rad/s)
  */
float PLL_Get_Electrical_Speed(void);

/**
  * @brief  获取 PLL 估算的机械角速度 (rad/s)
  */
float PLL_Get_Mechanical_Speed(void);

#endif
