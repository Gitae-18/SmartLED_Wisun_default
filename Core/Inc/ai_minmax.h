/*
 * ai_minmax.h
 *
 *  Created on: 2025. 12. 2.
 *      Author: USER
 */

#ifndef SRC_AI_MINMAX_H_
#define SRC_AI_MINMAX_H_

#pragma once
#include <stdint.h>
#include "data_samples.h"

extern const float AE_X_MIN[AE_COLS];
extern const float AE_X_MAX[AE_COLS];

void ai_minmax_scale(const float *x, float *x_scaled, int len);
void ai_minmax_inverse_scale(const float *x_scaled, float *x, int len);

#endif /* SRC_AI_MINMAX_H_ */
