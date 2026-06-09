/*
 * ai_config.h
 *
 *  Created on: 2026. 4. 15.
 *      Author: USER
 */

#ifndef INC_AI_CONFIG_H_
#define INC_AI_CONFIG_H_

#pragma once
#include <stdint.h>
#include <stddef.h>
//#include "data_samples.h"

/* AE 입력/출력 차원 (현재 130차원 FFT+vi_i+vi_v) */
#define AE_IN_DIM   (4)
#define AE_OUT_DIM  (4)

/* --- 입력/출력 양자화 파라미터 (STM32 로그 기반) --- */
/* input:  0.003761707,-128,int8 */
#define AE_IN_SCALE       (0.003918087f)
#define AE_IN_ZERO_POINT  (-128)

/* output: O[0] ... 0.003805347,-128,int8 */
#define AE_OUT_SCALE      (0.003799048f)
#define AE_OUT_ZERO_POINT (-128)

/* --- MSE 임계값 (PC AE + MinMax 후 best-F1 threshold) --- */
//#define AE_THRESH (4.948393878318489e-06f)		//  report.json 참고
#define AE_THRESH (0.01730970f)						// 임의 보정 값

#define AE_COLS (4)  // expected 4

extern const float		AE_SAMPLES[AE_COLS];

extern const float AE_X_MIN[AE_COLS];
extern const float AE_X_MAX[AE_COLS];

void ai_minmax_scale(const float *x, float *x_scaled, int len);
void ai_minmax_inverse_scale(const float *x_scaled, float *x, int len);

static inline int ae_round_to_int(float v)
{
    return (v >= 0.0f) ? (int)(v + 0.5f) : (int)(v - 0.5f);
}

static inline void ae_quantize_in_vec(const float *x, int8_t *q, int len)
{
    for (int i = 0; i < len; ++i) {
        float v = x[i] / AE_IN_SCALE + (float)AE_IN_ZERO_POINT;
        int tmp = ae_round_to_int(v);

        if (tmp < -128) tmp = -128;
        if (tmp > 127)  tmp = 127;

        q[i] = (int8_t)tmp;
    }
}

/* int8 출력 벡터 -> float 역양자화 */
static inline void ae_dequantize_out_vec(const int8_t *q, float *y, int len)
{
    for (int i = 0; i < len; ++i) {
        y[i] = ((float)q[i] - (float)AE_OUT_ZERO_POINT) * AE_OUT_SCALE;
    }
}


#endif /* INC_AI_CONFIG_H_ */
