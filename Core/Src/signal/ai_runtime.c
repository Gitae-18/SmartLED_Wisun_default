#include "signal/ai_runtime.h"
#include "signal/rul.h"
#include "services/rul_storage.h"
#include "utils/debug_log.h"
#include "stm32h5xx_hal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RUL_INTERVAL_MS ((uint32_t)(RUL_CALC_INTERVAL_HOURS * 3600000.0f))
#define RUL_SAVE_RETRY_MS 60000u

static struct {
    bool initialized;
    bool state_valid;
    bool measurement_valid;
    bool dirty;
    uint32_t last_rul_tick;
    uint32_t last_save_attempt;
    uint32_t inference_count;
} g_runtime;

float AI_Runtime_GetRULPercent(void)
{
    RUL_State state;
    if (!g_runtime.initialized || !g_runtime.state_valid ||
        !g_runtime.measurement_valid ||
        (uint32_t)(HAL_GetTick() - g_runtime.last_rul_tick) >= RUL_INTERVAL_MS)
        return NAN;
    RUL_GetState(&state);
    /* A zero-damage recovery record is not a completed life calculation. */
    return state.sample_count != 0u ? RUL_GetPercent() : NAN;
}

void AI_Runtime_InvalidateMeasurement(void)
{
    g_runtime.measurement_valid = false;
}

bool AI_Runtime_SaveState(void)
{
    if (!g_runtime.state_valid) return false;
    if (!g_runtime.dirty) return true;
    RUL_State state;
    RUL_GetState(&state);
    g_runtime.last_save_attempt = HAL_GetTick();
    if (!RUL_Storage_Save(&state)) {
        uart6_log("[RUL_SAVE] failed; RAM retained, retry in 60s\r\n");
        return false;
    }
    g_runtime.dirty = false;
    uart6_log("[RUL_SAVE] ok samples=%lu RUL=%.6f%%\r\n",
              (unsigned long)state.sample_count, (double)RUL_GetPercent());
    return true;
}

void AI_Runtime_Init(void)
{
    uart6_log("[AI_CONFIG] smps_watts=%u mse_threshold=%.9f\r\n",
              (unsigned)AI_SMPS_RATED_WATTS, (double)AE_THRESH);
    memset(&g_runtime, 0, sizeof(g_runtime));
    RUL_Reset();
    RUL_State saved;
    RUL_StorageStatus status = RUL_Storage_Load(&saved);
    g_runtime.initialized = true;
    g_runtime.last_rul_tick = HAL_GetTick();
    if (status == RUL_STORAGE_LOADED) {
        g_runtime.state_valid = RUL_Restore(&saved);
        g_runtime.measurement_valid = g_runtime.state_valid;
    } else if (status == RUL_STORAGE_EMPTY) {
        g_runtime.state_valid = true;
        g_runtime.dirty = true;
        (void)AI_Runtime_SaveState(); /* First boot establishes a recovery record. */
    }
    uart6_log("[RUL_INIT] storage=%d valid=%u RUL=%.6f%% interval=1h\r\n",
              (int)status, (unsigned)g_runtime.state_valid,
              (double)AI_Runtime_GetRULPercent());
    if (!g_runtime.state_valid)
        uart6_log("[RUL_INIT] no valid saved state; RUL unavailable, AI continues\r\n");
}

void AI_Runtime_Poll(uint32_t now)
{
    if (g_runtime.initialized && g_runtime.dirty &&
        (uint32_t)(now - g_runtime.last_save_attempt) >= RUL_SAVE_RETRY_MS)
        (void)AI_Runtime_SaveState();
}

AI_Result AI_Runtime_Process(const AI_RuntimeInput *input)
{
    AI_Result result = {.status = -1};
    AI_Runtime_InvalidateMeasurement();
    if (!g_runtime.initialized || input == NULL) return result;
    result = AI_infer(input->model_input);
    if (result.status != 0) {
        uart6_log("[AI_RUNTIME] inference failed status=%d\r\n", result.status);
        return result;
    }
    ++g_runtime.inference_count;
    bool temp_valid = input->temperature_valid && isfinite(input->temperature_c);
    g_runtime.measurement_valid = temp_valid;
    uint32_t now = HAL_GetTick();
    if (g_runtime.state_valid && temp_valid &&
        (uint32_t)(now - g_runtime.last_rul_tick) >= RUL_INTERVAL_MS) {
        (void)RUL_Update(input->temperature_c, result.hi);
        /* One update at the next valid measurement. Do not replay missed hours
         * with one stale HI/temperature after disabled sensing or invalid data. */
        g_runtime.last_rul_tick = now;
        g_runtime.dirty = true;
        (void)AI_Runtime_SaveState();
    }
    uart6_log("[AI_RUNTIME] n=%lu input=%.3f,%.3f,%.3f,%.3f "
              "MSE=%.9f pred=%u HI=%.6f temp_valid=%u temp=%.2f "
              "RUL=%.6f%% saved=%u\r\n",
              (unsigned long)g_runtime.inference_count,
              (double)input->measured_freq_khz, (double)input->measured_adc_pk,
              (double)input->measured_voltage_v, (double)input->measured_current_a,
              (double)result.mse, (unsigned)result.is_anomaly, (double)result.hi,
              (unsigned)temp_valid, (double)input->temperature_c,
              (double)AI_Runtime_GetRULPercent(),
              (unsigned)(g_runtime.state_valid && !g_runtime.dirty));
    return result;
}
