#include "current.h"
#include "adc.h"
#include "tim.h"

volatile uint8_t current_offset_calibrated = 0;
volatile uint8_t adc_injected_start_status = 0xFF;
volatile uint8_t tim1_ch4_start_status = 0xFF;
volatile uint32_t adc_irq_count = 0;
volatile uint32_t adc_injected_callback_count = 0;

static uint32_t current_offset_count = 0;
static uint32_t current_offset_sum_a = 0;
static uint32_t current_offset_sum_b = 0;
static uint32_t current_offset_sum_c = 0;
static float current_offset_a = 2048.0f;
static float current_offset_b = 2048.0f;
static float current_offset_c = 2048.0f;
/* 开启中断和定时器1的通道4 */
void Current_Sense_Start(void)
{
    Current_Offset_Reset();
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);

}
/* 初始化参数 */
void Current_Offset_Reset(void)
{
    current_offset_count = 0;
    current_offset_sum_a = 0;
    current_offset_sum_b = 0;
    current_offset_sum_c = 0;
    current_offset_a = 2048.0f;
    current_offset_b = 2048.0f;
    current_offset_c = 2048.0f;
    current_offset_calibrated = 0;
}
/* 偏置电压采集 */
void Current_Offset_Update(uint16_t adc_a, uint16_t adc_b, uint16_t adc_c)
{
    if (current_offset_calibrated)
        return;

    current_offset_sum_a += adc_a;
    current_offset_sum_b += adc_b;
    current_offset_sum_c += adc_c;
	
    current_offset_count++;
	//采集1024次之后取平均值
    if (current_offset_count >= CURRENT_OFFSET_SAMPLES)
    {
        current_offset_a = (float)current_offset_sum_a / (float)CURRENT_OFFSET_SAMPLES;
        current_offset_b = (float)current_offset_sum_b / (float)CURRENT_OFFSET_SAMPLES;
        current_offset_c = (float)current_offset_sum_c / (float)CURRENT_OFFSET_SAMPLES;
		//状态标志位
        current_offset_calibrated = 1;
    }
}
/* 将采集到的adc数据进行计算，返回电流值 */
float Current_Adc_To_Ampere(uint16_t adc, uint8_t phase)
{
    float offset = current_offset_a;

    if (phase == 1U)
        offset = current_offset_b;
    else if (phase == 2U)
        offset = current_offset_c;

    float sense_voltage = ((float)adc - offset) * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
	
    return sense_voltage / (CURRENT_AMP_GAIN * CURRENT_SHUNT_OHM);
}
