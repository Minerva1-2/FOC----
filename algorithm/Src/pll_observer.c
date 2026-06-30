#include "pll_observer.h"

/* ==================== PLL 环路参数 ==================== */
/*
 * PLL 本质是一个二阶 PI 锁相环:
 *   - 鉴相器: 计算 hall 参考角度与 PLL 估算角度之差
 *   - PI 控制器: 将角度误差转换为角速度修正
 *   - NCO (数控振荡器): 用角速度积分出连续角度
 *
 * 调试建议:
 *   1. KP 控制对角度误差的即时响应，太大 → 角度抖动
 *   2. KI 控制对转速的长期跟踪，太大 → 低频振荡
 *   3. 从保守值开始: KP=30, KI=500，逐步加大
 */
#define PLL_KP  30.0f
#define PLL_KI  500.0f

/* 频率积分项限幅 (rad/s), 防止 windup */
#define PLL_SPEED_LIMIT  15000.0f

/* ==================== 2*PI 常量 ==================== */
#define TWO_PI  6.28318530f

/* ==================== PLL 状态变量 ==================== */
static float pll_angle    = 0.0f;   /* 估算的电角度 (rad) */
static float pll_speed_el = 0.0f;   /* 估算的电角速度 (rad/s) */
static float pll_error_i  = 0.0f;   /* PI 积分项累积值 */

/* ==================== 角度归一化到 [0, 2*PI) ==================== */
static float NormalizeAngle(float angle)
{
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f)    angle += TWO_PI;
    return angle;
}

/* ==================== 角度差归一化到 [-PI, PI) ==================== */
static float WrapToPi(float angle)
{
    while (angle >  3.14159265f) angle -= TWO_PI;
    while (angle < -3.14159265f) angle += TWO_PI;
    return angle;
}

/* ==================== 初始化 ==================== */
void PLL_Init(float initial_angle)
{
    pll_angle    = NormalizeAngle(initial_angle);
    pll_speed_el = 0.0f;
    pll_error_i  = 0.0f;
}

/* ==================== PLL 更新 (每控制周期调用一次, 10kHz) ==================== */
/*
 * 工作流程:
 *   1. NCO 推算: 用上一周期估算的角速度，积分出新角度
 *   2. 鉴相: 计算 hall 参考角度与 NCO 推算角度之差
 *   3. PI 校正: 用角度误差修正角速度
 *   4. 输出平滑角度
 */
float PLL_Update(float theta_hall)
{
    /* 1. NCO: 角度 = 角度 + 角速度 × 采样周期 */
    pll_angle += pll_speed_el * PLL_TS;
    pll_angle = NormalizeAngle(pll_angle);

    /* 2. 鉴相器: hall 参考角度 - PLL 估算角度, 归一化到 [-PI, PI) */
    float error = WrapToPi(theta_hall - pll_angle);

    /* 3. PI 控制器: 积分项累积 + 比例项 */
    pll_error_i += PLL_KI * error * PLL_TS;

    /* 积分限幅, 防止 windup */
    if (pll_error_i > PLL_SPEED_LIMIT)       	pll_error_i = PLL_SPEED_LIMIT;
    else if (pll_error_i < -PLL_SPEED_LIMIT)	pll_error_i = -PLL_SPEED_LIMIT;

    /* 更新电角速度 = 比例 + 积分 */
    pll_speed_el = PLL_KP * error + pll_error_i;

    return pll_angle;
}

/* ==================== 获取速度 ==================== */
float PLL_Get_Electrical_Speed(void)
{
    return pll_speed_el;
}

float PLL_Get_Mechanical_Speed(void)
{
    return pll_speed_el / (float)POLE_PAIRS;
}
