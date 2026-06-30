#include "hall.h"
#include "foc.h"
#include "pll_observer.h"

/* ==================== 霍尔状态变量 ==================== */
static uint8_t g_hall_state;      /* 当前霍尔三路信号的组合值 (1~6) */
static uint8_t g_prev_hall_state; /* 上一次霍尔状态，用于检测状态变化 */

/* ==================== 霍尔状态 → 电角度查找表 (LUT) ==================== */
/*
 * 三个霍尔传感器 (U/V/W) 组合出 8 种状态 (000~111)，其中 000 和 111 无效。
 * 每种有效状态对应转子磁链所在扇区的边界角度。
 * 下标的二进制位对应 hall_1(U)、hall_2(V)、hall_3(W) 的电平。
 * 这些角度值是通过实际标定得到的，不同电机可能不同。
 */
static const float hall_angle_lut[8] = {
    0.0f,          // [0] 000 - 无效
    2.08f,         // [1] state 1 → 实测 2.08 rad
    4.16f,         // [2] state 2 → 实测 4.16 rad
    3.07f,         // [3] state 3 → 实测 3.07 rad
    6.12f,         // [4] state 4 → 实测 6.12 rad
    0.98f,         // [5] state 5 → 实测 0.98 rad
    5.17f,         // [6] state 6 → 实测 5.17 rad
    0.0f           // [7] 111 - 无效
};
/* ==================== 该电机的正转霍尔状态顺序 ==================== */
/*
 * 该电机正转时，霍尔状态按 5→1→3→2→6→4 的顺序循环。
 * 这个顺序是通过实际观测波形标定出来的，不同电机接线可能不同。
 */
static const uint8_t hall_sequence[6] = {5, 1, 3, 2, 6, 4};

/* ==================== 时间线性插值所需的变量 ==================== */
static float sector_start_angle;    /* 当前扇区的起始电角度（查 LUT 得到） */
static float sector_angle_delta;    /* 当前扇区的角度跨度，正转=+60°，反转=-60° */
static uint32_t sector_tick_start;  /* 进入当前扇区时的 DWT 周期计数值 */
static uint32_t sector_tick_delta;  /* 上一个完整扇区所耗的 DWT 周期数（用来估算转速） */
static float interpolated_angle;    /* 插值计算后的连续电角度输出 */
static float pll_output_angle;      /* PLL 输出的平滑电角度 */

/* ==================== DWT 周期计数器 (168MHz, 分辨率 ~6ns) ==================== */
/*
 * DWT (Data Watchpoint and Trace) 是 Cortex-M4 内核自带的硬件周期计数器。
 * 它以 CPU 主频 (168MHz) 计数，每过一个时钟周期 +1。
 * 比 HAL_GetTick() (1ms 分辨率) 精确约 17 万倍。
 * 32 位计数器在 168MHz 下约 25.5 秒溢出一次，足够覆盖扇区间的时间跨度。
 */
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 使能 DWT 跟踪功能 */
    DWT->CYCCNT = 0;                                  /* 将周期计数器清零 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* 启动周期计数器，开始计数 */
}

static inline uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;  /* 读取当前已经过了多少个 CPU 时钟周期 */
}

/* 在正转顺序表中查找某个霍尔状态的索引位置（0~5），找不到返回 -1 */
static int8_t hall_seq_index(uint8_t state)
{
    for (int i = 0; i < 6; i++)
    {
        if (hall_sequence[i] == state)  /* 找到了就返回它在数组中的下标 */
            return i;
    }
    return -1;  /* 无效状态 */
}

/* ==================== 角度归一化到 [0, 2π) ==================== */
/*
 * 将任意角度值限制在 0 到 2π 范围内。
 * 比如 7.0 → 0.72，-0.5 → 5.78。
 * FOC 的 sin/cos 计算需要输入角度在这个范围内。
 */
static float normalize_angle(float angle)
{
    while (angle >= 2.0f * PI_ANGLE) angle -= 2.0f * PI_ANGLE;  /* 超过 2π 就减一圈 */
    while (angle < 0.0f)             angle += 2.0f * PI_ANGLE;  /* 小于 0 就加一圈 */
    return angle;
}

/* ==================== 读取三路霍尔 GPIO 并组合成状态值 ==================== */
uint8_t Get_Hall_Count(void)
{
    uint8_t hall_1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);  /* U 相霍尔，读 0 或 1 */
    uint8_t hall_2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);  /* V 相霍尔 */
    uint8_t hall_3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);  /* W 相霍尔 */
	
    /* 将三路信号拼成一个 3 位二进制数：bit2=U, bit1=V, bit0=W，范围 0~7 */
    g_hall_state = (hall_1 << 2) | (hall_2 << 1) | (hall_3 << 0);
	
    return g_hall_state;
}

/* ==================== 初始化 ==================== */
void Hall_Init(void)
{
    DWT_Init();  /* 启动 DWT 硬件周期计数器 */

    g_prev_hall_state = Get_Hall_Count();  /* 读取电机当前的霍尔状态作为初始值 */

    sector_start_angle = hall_angle_lut[g_prev_hall_state];  /* 查表得到初始扇区边界角度 */

    sector_tick_start = DWT_GetCycles();  /* 记录进入当前扇区的时刻 */

    sector_tick_delta = 168000;  /* 给一个默认值：1ms = 168000 个周期 */
                                 /* 防止电机还没转时做除法除零 */

    sector_angle_delta = PI_ANGLE / 3.0f;  /* 默认假设正转，扇区跨度 = 60° */

    interpolated_angle = sector_start_angle;  /* 初始输出角度 = 扇区边界角度 */

    PLL_Init(interpolated_angle);  /* 用初始霍尔角度初始化 PLL */
    pll_output_angle = interpolated_angle;
}

/* ==================== 每个控制周期 (10kHz) 调用一次，更新插值角度 ==================== */
void Hall_Update(void)
{
    uint8_t current = Get_Hall_Count();  /* 读取当前霍尔状态 */

    /* 如果霍尔状态发生了变化，且新状态是有效值 (1~6)，说明转子进入了新扇区 */
    if (current != g_prev_hall_state && current >= 1 && current <= 6)
    {
        uint32_t now = DWT_GetCycles();  /* 记录此刻的精确时刻 */

        /* 计算上一个扇区走了多少个 CPU 周期，作为估算当前扇区耗时的参考 */
        sector_tick_delta = now - sector_tick_start;
        if (sector_tick_delta == 0) sector_tick_delta = 1;  /* 防止除零 */

        /* 查 LUT 得到新扇区的起始电角度 */
        sector_start_angle = hall_angle_lut[current];

        /* 通过新旧状态在正转顺序表中的位置关系，判断电机是正转还是反转 */
        int8_t cur_idx = hall_seq_index(current);       /* 新状态在顺序表中的位置 */
        int8_t prev_idx = hall_seq_index(g_prev_hall_state);  /* 旧状态的位置 */
        if (cur_idx >= 0 && prev_idx >= 0)
        {
            int diff = cur_idx - prev_idx;
            if (diff == 1 || diff == -5)       /* 正转：位置 +1（或从末尾回到开头 -5） */
                sector_angle_delta = PI_ANGLE / 3.0f;   /* 扇区跨度 = +60° */
            else if (diff == -1 || diff == 5)   /* 反转：位置 -1（或从开头跳到末尾 +5） */
                sector_angle_delta = -(PI_ANGLE / 3.0f); /* 扇区跨度 = -60° */
            else
                sector_angle_delta = PI_ANGLE / 3.0f;   /* 异常情况，默认正转 */
        }

        sector_tick_start = now;         /* 重置扇区起始时刻 */
        g_prev_hall_state = current;     /* 更新"上一次状态"为当前状态 */
    }

    /* ---- 时间线性插值：在扇区内估算当前角度 ---- */
    /*
     * 原理：假设当前扇区耗时与上一个扇区相同 (sector_tick_delta)。
     * ratio = 已过时间 / 上一扇区耗时，即转子在当前扇区内走了多少比例。
     * 当前角度 = 扇区起始角度 + 比例 × 扇区跨度。
     */
    uint32_t elapsed = DWT_GetCycles() - sector_tick_start;  /* 当前扇区已过的周期数 */
    float ratio = (float)elapsed / (float)sector_tick_delta;  /* 计算时间比例 (0.0 ~ 1.0) */
    if (ratio > 1.0f) ratio = 1.0f;  /* 如果超过了预估的扇区时长，限制在扇区末端 */

    /* 用起始角度 + 比例 × 跨度，算出当前连续电角度，再归一化到 [0, 2π) */
    interpolated_angle = normalize_angle(sector_start_angle + ratio * sector_angle_delta);

    /* 将插值角度送入 PLL 锁相环进行平滑滤波 */
    pll_output_angle = PLL_Update(interpolated_angle);
}

/* ==================== 外部接口：获取插值后的连续电角度 ==================== */
float Get_Hall_Electrical_Angle(void)
{
    return interpolated_angle;  /* 返回最近一次 Hall_Update 算出的角度 */
}
/* ==================== 外部接口：获取 PLL 估算的电角速度 ==================== */
float Get_PLL_Angle(void)
{
    return pll_output_angle;
}

float Get_PLL_Speed(void)
{
    return PLL_Get_Electrical_Speed();
}
