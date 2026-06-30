#ifndef SENSOR_MEASURE_H
#define SENSOR_MEASURE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NTC_LUT_SIZE
#define NTC_LUT_SIZE 25
#endif

#ifndef ADC_MAX_COUNTS
#define ADC_MAX_COUNTS    4095.0f
#endif
#ifndef VREF_FIXED
#define VREF_FIXED        3.3f
#endif

#ifndef R_TOP_V
#define R_TOP_V           100000.0f
#endif
#ifndef R_BOT_V
#define R_BOT_V           10000.0f
#endif
#ifndef V_DIV_GAIN
#define V_DIV_GAIN        ((R_TOP_V + R_BOT_V) / R_BOT_V)
#endif

#ifndef R_SHUNT
#define R_SHUNT           0.05f
#endif
#ifndef I_AMP_GAIN
#define I_AMP_GAIN        50.0f
#endif

#ifndef K_ADC2V
#define K_ADC2V           (VREF_FIXED / ADC_MAX_COUNTS)
#endif

#define NTC_VALID_TEMP_MIN_C   (-20.0f)
#define NTC_VALID_TEMP_MAX_C   (65.0f)
#define NTC_INVALID_TEMP_C   (-999.0f)



static inline float adc_to_vsense(uint16_t raw)
{
    return ((float)raw * VREF_FIXED) / ADC_MAX_COUNTS;
}

static inline float vsense_to_vin(float v_sense)
{
    return v_sense * V_DIV_GAIN;
}

static inline float vsense_to_current(float v_sense, float offset_v)
{
    return (v_sense - offset_v) / (R_SHUNT * I_AMP_GAIN);
}

float ntc_voltage_to_temp_c(float temp_v);
uint8_t ntc_voltage_to_temp_c_checked(float temp_v, float *out_temp_c);
uint32_t Read_ADC_Channel(uint32_t channel);
float Convert_Voltage_ADC(uint32_t adc_value);
float Convert_Voltage_To_Current(float voltage, float offset);
void Print_Voltage_Current(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MEASURE_H */
