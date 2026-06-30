#ifndef __CURRENT_H
#define __CURRENT_H

#include <stdint.h>

#define ADC_FULL_SCALE       4095.0f
#define ADC_REF_VOLTAGE      3.3f
#define CURRENT_AMP_GAIN     20.0f
#define CURRENT_SHUNT_OHM    0.0005f
#define CURRENT_OFFSET_SAMPLES 1024U

extern volatile uint8_t current_offset_calibrated;
extern volatile uint8_t adc_injected_start_status;
extern volatile uint8_t tim1_ch4_start_status;
extern volatile uint32_t adc_irq_count;
extern volatile uint32_t adc_injected_callback_count;
extern volatile uint32_t adc_sr_debug;
extern volatile uint32_t adc_cr1_debug;
extern volatile uint32_t adc_cr2_debug;
extern volatile uint32_t tim1_cc_irq_count;
extern volatile uint32_t tim1_sr_debug;
extern volatile uint32_t tim1_dier_debug;
extern volatile uint32_t tim1_ccer_debug;

void Current_Offset_Reset(void);
void Current_Offset_Update(uint16_t adc_a, uint16_t adc_b, uint16_t adc_c);
float Current_Adc_To_Ampere(uint16_t adc, uint8_t phase);

#endif
