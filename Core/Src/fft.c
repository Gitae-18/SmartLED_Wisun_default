#include "fft.h"
#include <stdio.h>
extern arm_rfft_fast_instance_f32 fftInstance;

/**
 * @brief FFT 변환을 수행하는 함수
 * @param input  입력 데이터 배열 (FFT_SIZE 크기)
 * @param output FFT 결과 배열 (FFT_SIZE 크기)
 */


void processFFT(float32_t *input, float32_t *output, float32_t *magnitude) {
    arm_rfft_fast_f32(&fftInstance, input, output, 0);
    arm_cmplx_mag_f32(output, magnitude, FFT_SIZE / 2);
}

/*
void calculateMemoryUsage(uint16_t fft_size) {
    // float32_t 크기 (4바이트)
    uint16_t float_size = sizeof(float32_t);

    // FFT 결과 저장 용량 (복소수 크기 벡터)
    uint32_t fft_data_size = (fft_size / 2) * float_size;

    // 바이트 -> KB 변환
    float fft_data_size_kb = fft_data_size / 1024.0;

    printf("FFT 데이터 크기: %lu 바이트 (%.2f KB)\n", fft_data_size, fft_data_size_kb);
}
*/
