#include <math.h>
#include "foc.h"
#include "tim.h"
#include "pwm.h"
#include "lowpass_filter.h"

float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f, zero_electric_angle = 0.0f;

static Park_t transform_park;
static Clark_t transform_clark;

static LowPassFilter_t ua_filter;
static LowPassFilter_t ub_filter;
static LowPassFilter_t uc_filter;
static uint8_t voltage_filter_initialized = 0;

const float vdc = VOLTAGE_POWER_SUPPLY;
const float inv_sqrt3 = 0.57735026919f;

float normalizeAngle(float angle)
{
    float normal_angle = fmodf(angle, 2.0f * PI_ANGLE);
    return (normal_angle >= 0.0f) ? normal_angle : (normal_angle + 2.0f * PI_ANGLE);
}

Clark_t *Clark(float Ia, float Ib, float Ic)
{
    (void)Ic;
    transform_clark.alpha = Ia;
    transform_clark.beta = (Ia + 2.0f * Ib) / sqrtf(3.0f);
    return &transform_clark;
}

Park_t *Park(float Ialpha, float Ibeta, float angle_el)
{
    float sin_el = sinf(angle_el);
    float cos_el = cosf(angle_el);

    transform_park.Id = Ialpha * cos_el + Ibeta * sin_el;
    transform_park.Iq = -Ialpha * sin_el + Ibeta * cos_el;
    return &transform_park;
}

void Set_Phase_Voltage(float Uq, float Ud, float angle_el)
{
    angle_el = normalizeAngle(angle_el + zero_electric_angle);

    float sin_el = sinf(angle_el);
    float cos_el = cosf(angle_el);
    float Ualpha = Ud * cos_el - Uq * sin_el;
    float Ubeta = Ud * sin_el + Uq * cos_el;

    SVPWM(Ualpha, Ubeta);
}

void SVPWM(float Ualpha, float Ubeta)
{
    // 计算参考电压矢量的幅值（欧氏长度）
    float Uref = sqrtf(Ualpha * Ualpha + Ubeta * Ubeta);
    // 计算线性调制区能输出的最大相电压幅值（Vdc / √3）
    float Umax = vdc * inv_sqrt3;

    // 过调制处理：如果指令幅值超过线性区极限，则等比缩放到极限值
    if (Uref > Umax)
    {
        float scale = Umax / Uref;    // 计算缩放系数
        Ualpha *= scale;              // 缩放 α 轴分量
        Ubeta *= scale;               // 缩放 β 轴分量
    }

    // 等幅值 Clarke 逆变换：从 α-β 坐标系得到三相平衡相电压（相对于电机中性点）
    float Va = Ualpha;                                    // A 相电压
    float Vb = -0.5f * Ualpha + 0.86602540378f * Ubeta;   // B 相电压，0.866... = √3/2
    float Vc = -0.5f * Ualpha - 0.86602540378f * Ubeta;   // C 相电压

    // 找出三相电压中的最大值与最小值，为注入零序分量做准备
    float Vmax = Va;               // 暂设 A 相为最大
    float Vmin = Va;               // 暂设 A 相为最小

    if (Vb > Vmax) Vmax = Vb;      // B 相更大则更新最大值
    if (Vc > Vmax) Vmax = Vc;      // C 相更大则更新最大值

    if (Vb < Vmin) Vmin = Vb;      // B 相更小则更新最小值
    if (Vc < Vmin) Vmin = Vc;      // C 相更小则更新最小值

    // 计算注入的共模（零序）电压，将正弦调制波变为马鞍形，实现 SVPWM 效果
    float Vcom = -0.5f * (Vmax + Vmin);

    // 生成最终的 PWM 比较值：相电压 + 共模分量 + 直流偏置 Vdc/2，映射到 [0, Vdc]
    float ua_cmd = Va + Vcom + vdc * 0.5f;
    float ub_cmd = Vb + Vcom + vdc * 0.5f;
    float uc_cmd = Vc + Vcom + vdc * 0.5f;

    // 硬限幅：确保比较值不超出 PWM 的有效范围 [0, Vdc]
    ua_cmd = _constrain(ua_cmd, 0.0f, vdc);
    ub_cmd = _constrain(ub_cmd, 0.0f, vdc);
    uc_cmd = _constrain(uc_cmd, 0.0f, vdc);

    // 首次调用时初始化三相电压低通滤波器，用当前指令值初始化滤波器状态
    if (!voltage_filter_initialized)
    {
        LPF_Init(&ua_filter, PHASE_VOLTAGE_FILTER_ALPHA, ua_cmd);
        LPF_Init(&ub_filter, PHASE_VOLTAGE_FILTER_ALPHA, ub_cmd);
        LPF_Init(&uc_filter, PHASE_VOLTAGE_FILTER_ALPHA, uc_cmd);
		
        voltage_filter_initialized = 1;   // 标记已初始化，下次不再重复初始化
    }

    // 对三相电压指令进行低通滤波，得到平滑后的输出电压
    Ua = LPF_Update(&ua_filter, ua_cmd);
    Ub = LPF_Update(&ub_filter, ub_cmd);
    Uc = LPF_Update(&uc_filter, uc_cmd);

    // 将滤波后的三相电压值输出到 PWM 外设，控制逆变器开关管
    Set_PWM(Ua, Ub, Uc);
}
