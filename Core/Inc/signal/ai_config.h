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

/* ON_TEST_v1: [peak_freq_khz, adc_pk_fft, voltage_v, current_a]. */
#define AE_IN_DIM   (4)
#define AE_OUT_DIM  (4)

/* Quantization from ai_model.onnx / network_generate_report.txt. */
#define AE_IN_SCALE       (0.003766168f)
#define AE_IN_ZERO_POINT  (-128)

#define AE_OUT_SCALE      (0.003898344f)
#define AE_OUT_ZERO_POINT (-128)

/* Explicit SMPS/threshold pair for this firmware build. No MID-based guessing.
 * The 100 W default preserves ON_TEST_v1; its calibration still needs review.
 * For any other rating, supply a measured AI_SMPS_MSE_THRESHOLD as well.
 * Changing this pair does not replace the model, scaler, or HI calibration. */
#ifndef AI_SMPS_RATED_WATTS
#define AI_SMPS_RATED_WATTS 100
#endif
#if AI_SMPS_RATED_WATTS <= 0 || AI_SMPS_RATED_WATTS > 65535
#error "AI_SMPS_RATED_WATTS must be a positive uint16 watt rating"
#endif
#ifndef AI_SMPS_MSE_THRESHOLD
#if AI_SMPS_RATED_WATTS == 100
#define AI_SMPS_MSE_THRESHOLD (0.011325965f)
#else
#error "Supply a validated AI_SMPS_MSE_THRESHOLD for this SMPS rating"
#endif
#endif
#define AE_THRESH AI_SMPS_MSE_THRESHOLD
#define AE_HI_LOW  (0.000452750f)
#define AE_HI_HIGH (0.000475430f)
#define RUL_CALC_INTERVAL_HOURS (1.0f)

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
