#include "sensor_measure.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "stm32h5xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart6;

static const float ntc_temp_table[NTC_LUT_SIZE] = {
    -40.000000f, -35.000000f, -30.000000f, -25.000000f, -20.000000f, -15.000000f, -10.000000f, -5.000000f,
    0.000000f,   5.000000f,   10.000000f,  15.000000f,  20.000000f,  25.000000f,  30.000000f,  35.000000f,
    40.000000f,  45.000000f,  50.000000f,  55.000000f,  60.000000f,  65.000000f,  70.000000f,  75.000000f,
    80.000000f
};

static const float ntc_voltage_table[NTC_LUT_SIZE] = {
    3.197196f, 3.162500f, 3.117680f, 3.060870f, 2.988679f, 2.902410f, 2.800000f, 2.677358f,
    2.532558f, 2.357143f, 2.200000f, 1.980000f, 1.800000f, 1.650000f, 1.466259f, 1.291357f,
    1.128233f, 0.979325f, 0.845373f, 0.726901f, 0.623167f, 0.533168f, 0.455663f, 0.389177f,
    0.332641f
};

float ntc_voltage_to_temp_c(float temp_v)
{
    if (temp_v >= ntc_voltage_table[0]) {
        return ntc_temp_table[0];
    }
    if (temp_v <= ntc_voltage_table[NTC_LUT_SIZE - 1]) {
        return ntc_temp_table[NTC_LUT_SIZE - 1];
    }

    for (int i = 1; i < NTC_LUT_SIZE; ++i) {
        float v_hi = ntc_voltage_table[i - 1];
        float v_lo = ntc_voltage_table[i];

        if (temp_v <= v_hi && temp_v >= v_lo) {
            float t_hi = ntc_temp_table[i - 1];
            float t_lo = ntc_temp_table[i];
            float ratio = (v_hi - temp_v) / (v_hi - v_lo);
            return t_hi + (t_lo - t_hi) * ratio;
        }
    }

    return ntc_temp_table[NTC_LUT_SIZE - 1];
}

uint8_t ntc_voltage_to_temp_c_checked(float temp_v, float *out_temp_c)
{
    if (out_temp_c == NULL) {
        return 0u;
    }

    *out_temp_c = NTC_INVALID_TEMP_C;

    if (!isfinite(temp_v)) {
        return 0u;
    }

    if (temp_v < 0.35f || temp_v > 3.15f) {
        return 0u;
    }

    float temp_c = ntc_voltage_to_temp_c(temp_v);

    if (!isfinite(temp_c)) {
        return 0u;
    }

    if (temp_c < NTC_VALID_TEMP_MIN_C || temp_c > NTC_VALID_TEMP_MAX_C) {
        *out_temp_c = NTC_INVALID_TEMP_C;
        return 0u;
    }

    *out_temp_c = temp_c;
    return 1u;
}

uint32_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

float Convert_Voltage_ADC(uint32_t adc_value)
{
    return ((float)adc_value / ADC_MAX_COUNTS) * VREF_FIXED;
}

float Convert_Voltage_To_Current(float voltage, float offset)
{
    return (voltage - offset) / 0.137f;
}

void Print_Voltage_Current(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adc_val_current;
    char log[64];

    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_val_current = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float voltage = Convert_Voltage_ADC(adc_val_current);
    float current = Convert_Voltage_To_Current(voltage, 2.50f);

    snprintf(log, sizeof(log),
             "ADC_RAW_CURR: %lu | Voltage: %.2fV | Current: %.2fA\r\n",
             adc_val_current, voltage, current);

    HAL_UART_Transmit(&huart6, (uint8_t*)log, strlen(log), HAL_MAX_DELAY);
}
