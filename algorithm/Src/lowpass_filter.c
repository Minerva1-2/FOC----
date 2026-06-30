#include "lowpass_filter.h"

void LPF_Init(LowPassFilter_t *lpf, float alpha, float initial_value)
{
    lpf->alpha = alpha;
    lpf->output = initial_value;
}

float LPF_Update(LowPassFilter_t *lpf, float input)
{
    lpf->output = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->output;
    return lpf->output;
}
