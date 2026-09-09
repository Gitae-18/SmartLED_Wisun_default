#include "signal/ai_inference.h"
#include "app_x-cube-ai.h"

#include <math.h>
#include <stddef.h>

AI_Result AI_infer(const float sensor_data[AE_COLS])
{
    AI_Result result = {.status = -1};
    float input_scaled[AE_COLS];
    int8_t input_quantized[AE_COLS];
    int8_t output_quantized[AE_COLS];
    float output_dequantized[AE_COLS];
    if (sensor_data == NULL) return result;
    for (int i = 0; i < AE_COLS; ++i) {
        if (!isfinite(sensor_data[i])) return result;
    }

    ai_minmax_scale(sensor_data, input_scaled, AE_COLS);
    ae_quantize_in_vec(input_scaled, input_quantized, AE_COLS);
    result.status = AI_ModelRun(input_quantized, output_quantized);
    if (result.status != 0) return result;
    ae_dequantize_out_vec(output_quantized, output_dequantized, AE_COLS);

    /* Use the same float MSE accumulation as ON_TEST_v1. */
    for (int i = 0; i < AE_COLS; ++i) {
        float error = input_scaled[i] - output_dequantized[i];
        result.mse += error * error;
    }
    result.mse /= AE_COLS;
    result.hi = (result.mse - AE_HI_LOW) / (AE_HI_HIGH - AE_HI_LOW);
    if (result.hi < 0.0f) result.hi = 0.0f;
    if (result.hi > 1.0f) result.hi = 1.0f;
    result.is_anomaly = (result.mse >= AE_THRESH) ? 1u : 0u;
    return result;
}
