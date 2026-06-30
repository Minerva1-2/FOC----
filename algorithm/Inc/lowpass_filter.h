#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H

typedef struct {
    float alpha;    /* 滤波系数 0~1, 越小滤波越强 */
    float output;
} LowPassFilter_t;

void LPF_Init(LowPassFilter_t *lpf, float alpha, float initial_value);
float LPF_Update(LowPassFilter_t *lpf, float input);

#endif
