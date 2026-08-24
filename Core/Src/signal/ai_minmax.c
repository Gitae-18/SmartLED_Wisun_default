/*
 * ai_minmax.c
 *
 *  Created on: 2025. 12. 2.
 *      Author: USER
 */

//#include "ai_minmax.h"
#include "signal/ai_config.h"
#include <stdint.h>

void ai_minmax_scale(const float *x, float *x_scaled, int len)
{
    for (int i = 0; i < len; i++) {
        float v    = x[i];
        float vmin = AE_X_MIN[i];
        float vmax = AE_X_MAX[i];
        float norm = 0.0f;

        if (vmax > vmin) {
            norm = (v - vmin) / (vmax - vmin);
        }

        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;

        x_scaled[i] = norm;
    }
}

void ai_minmax_inverse_scale(const float *x_scaled, float *x, int len)
{
    for (int i = 0; i < len; i++) {
        float norm = x_scaled[i];
        float vmin = AE_X_MIN[i];
        float vmax = AE_X_MAX[i];

        /* 필요하면 norm 클램핑 */
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;

        x[i] = norm * (vmax - vmin) + vmin;
    }
}

const float AE_X_MIN[AE_COLS] = {
    85.11000061035156f,
    1.5f,
    0.4000000059604645f,
    2.490000009536743f
};

const float AE_X_MAX[AE_COLS] = {
    94.87000274658203f,
    236.39999389648438f,
    1.600000023841858f,
    2.8499999046325684f
};

