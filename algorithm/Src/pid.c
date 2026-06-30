#include "pid.h"
#include "foc.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Compute(PID_t *pid, float error)
{
    /* P */
    float p_term = pid->kp * error;

    /* I - 积分并限幅防 windup */
    pid->integral += pid->ki * error * FOC_TS;
    if (pid->integral > pid->output_limit)
        pid->integral = pid->output_limit;
    else if (pid->integral < -pid->output_limit)
        pid->integral = -pid->output_limit;

    /* D */
    float d_term = pid->kd * (error - pid->prev_error) / FOC_TS;
    pid->prev_error = error;

    /* 输出求和并限幅 */
    float output = p_term + pid->integral + d_term;
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
