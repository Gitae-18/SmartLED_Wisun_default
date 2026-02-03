/*
 * fft.h
 *
 *  Created on: Mar 18, 2025
 *      Author: nicek
 */
#include "arm_math.h"
#include "arm_const_structs.h"

#define FFT_SIZE 4096  // FFT 크기 정의
#define F_ULTRA   120000.0f
#define FSAMPLE   600000.0f
//define CYCLE_SAMPS  ((int)(FSAMPLE/F_ULTRA))
#define N_CYCLES     10

#ifndef INC_FFT_H_
#define INC_FFT_H_

void processFFT(float32_t *input, float32_t *output, float32_t *magnitude);

void calculateMemoryUsage(uint16_t fft_size) ;

#endif /* INC_FFT_H_ */
