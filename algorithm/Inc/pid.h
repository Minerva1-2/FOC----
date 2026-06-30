#ifndef __PID_H
#define __PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_limit;
    float integral;
    float prev_error;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit);
float PID_Compute(PID_t *pid, float error);
void PID_Reset(PID_t *pid);

#endif
