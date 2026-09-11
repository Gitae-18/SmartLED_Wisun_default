#ifndef SIGNAL_AI_RUNTIME_H
#define SIGNAL_AI_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>
#include "signal/ai_inference.h"

typedef struct {
    float model_input[AE_COLS];
    float measured_freq_khz;
    float measured_adc_pk;
    float measured_voltage_v;
    float measured_current_a;
    float temperature_c;
    uint8_t temperature_valid;
} AI_RuntimeInput;

void AI_Runtime_Init(void);
AI_Result AI_Runtime_Process(const AI_RuntimeInput *input);
/* Main-loop retry of failed saves; never call storage from an ISR. */
void AI_Runtime_Poll(uint32_t now);
bool AI_Runtime_SaveState(void);
/* Mark a failed/skipped AI measurement (including unavailable FFT data).
 * Keeps accumulated damage intact; the next valid measurement can recover. */
void AI_Runtime_InvalidateMeasurement(void);
/* NAN before the first completed update, after invalid measurements/recovery,
 * or when an hourly update is overdue. */
float AI_Runtime_GetRULPercent(void);

#endif
