#include "adc_user.h"

extern ADC_HandleTypeDef hadc1;

HAL_StatusTypeDef AD_DC_Injected_Once(VIRead* out)
{
    out->t_us = now_us();
    HAL_StatusTypeDef st = HAL_ADCEx_InjectedStart(&hadc1);
    if (st != HAL_OK) return st;

    st = HAL_ADCEx_InjectedPollForConversion(&hadc1, 20);
    if (st != HAL_OK) return st;
    out->volt_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

    st = HAL_ADCEx_InjectedPollForConversion(&hadc1, 20);
    if (st != HAL_OK) return st;
    out->curr_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    HAL_ADCEx_InjectedStop(&hadc1);
    return HAL_OK;
}
