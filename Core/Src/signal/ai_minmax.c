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
    80.078125f,
    0.712206006f,
    27.366035461f,
    2.729671001f
};

const float AE_X_MAX[AE_COLS] = {
    110.937507629f,
    11.872964859f,
    27.631887436f,
    3.293771982f
};

