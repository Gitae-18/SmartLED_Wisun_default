#ifndef __QUANT_H__
#define __QUANT_H__

#include <stdint.h>
#include <stddef.h>

/* AE 입력/출력 차원 (현재 130차원 FFT+vi_i+vi_v) */
#define AE_IN_DIM   (130)
#define AE_OUT_DIM  (130)

/* --- 입력/출력 양자화 파라미터 (STM32 로그 기반) --- */
/* input:  I[0] ... 130/i8 1:(0.003921,-128) */
#define AE_IN_SCALE       (0.003921f)
#define AE_IN_ZERO_POINT  (-128)

/* output: O[0] ... 130/i8 1:(0.004475,-108) */
#define AE_OUT_SCALE      (0.004475f)
#define AE_OUT_ZERO_POINT (-108)

/* --- MSE 임계값 (PC AE + MinMax 후 best-F1 threshold) --- */
/* report.json: "best_f1_threshold": 1.907256568879761e-05 */
//#define AE_THRESH (1.9072566e-05f)
//#define AE_THRESH (5.8e-05f)
//#define AE_THRESH (0.032473f)
#define AE_THRESH (0.2577f)

/* --------- 헬퍼 함수: 벡터 양자화 / 역양자화 --------- */

/* float 입력 벡터 -> int8 양자화 */
static inline void ae_quantize_in_vec(const float *x, int8_t *q, int len)
{
    for (int i = 0; i < len; ++i) {
        float v = x[i] / AE_IN_SCALE + (float)AE_IN_ZERO_POINT;
        int tmp = (int)(v + 0.5f);   // 반올림
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

#endif /* __QUANT_H__ */
