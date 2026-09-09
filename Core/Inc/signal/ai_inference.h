#ifndef SIGNAL_AI_INFERENCE_H
#define SIGNAL_AI_INFERENCE_H

#include <stdint.h>
#include "signal/ai_config.h"

typedef struct {
    int status;
    uint8_t is_anomaly;
    float mse;
    float hi; /* 0: low anomaly score, 1: high anomaly score. */
} AI_Result;

AI_Result AI_infer(const float sensor_data[AE_COLS]);

#endif
