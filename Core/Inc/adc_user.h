#ifndef ADC_USER_H
#define ADC_USER_H

#include "main.h"

typedef struct { uint16_t volt_raw; uint16_t curr_raw; uint32_t t_us; } VIRead;

static inline uint32_t now_us(void) {
    extern TIM_HandleTypeDef htim2;
    return __HAL_TIM_GET_COUNTER(&htim2);
}

HAL_StatusTypeDef AD_DC_Injected_Once(VIRead* out);

#endif
