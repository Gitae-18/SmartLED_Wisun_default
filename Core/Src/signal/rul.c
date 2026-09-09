#include "signal/rul.h"
#include "signal/ai_config.h"

#include <math.h>
#include <stddef.h>

#define RUL_EWMA_ALPHA    (2.0f / 25.0f)
#define RUL_DAMAGE_ALPHA  (2.0f)
#define RUL_MIN_LIFE_HOUR (1000.0f)

static RUL_State g_state;

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float life_at_temperature(float temperature_c)
{
    float life_hour;
    if (temperature_c < 70.0f) {
        life_hour = 100000.0f;
    } else if (temperature_c <= 80.0f) {
        life_hour = 100000.0f - 5500.0f * (temperature_c - 70.0f);
    } else {
        life_hour = 45000.0f - 3000.0f * (temperature_c - 80.0f);
    }
    return (life_hour < RUL_MIN_LIFE_HOUR) ? RUL_MIN_LIFE_HOUR : life_hour;
}

void RUL_Reset(void)
{
    g_state = (RUL_State){0};
}

bool RUL_StateValid(const RUL_State *state)
{
    return state != NULL && isfinite(state->cumulative_damage) &&
        isfinite(state->hi_smooth) &&
        state->cumulative_damage >= 0.0f && state->cumulative_damage <= 1.0f &&
        state->hi_smooth >= 0.0f && state->hi_smooth <= 1.0f &&
        (state->sample_count != 0u ||
         (state->cumulative_damage == 0.0f && state->hi_smooth == 0.0f));
}

bool RUL_Restore(const RUL_State *state)
{
    if (!RUL_StateValid(state)) return false;
    g_state = *state;
    return true;
}

void RUL_GetState(RUL_State *state)
{
    if (state != NULL) *state = g_state;
}

float RUL_GetPercent(void)
{
    return (1.0f - g_state.cumulative_damage) * 100.0f;
}

float RUL_Update(float temperature_c, float hi)
{
    if (!isfinite(temperature_c) || !isfinite(hi)) return RUL_GetPercent();
    hi = clampf(hi, 0.0f, 1.0f);
    if (g_state.sample_count == 0u) {
        g_state.hi_smooth = hi;
    } else {
        g_state.hi_smooth = (1.0f - RUL_EWMA_ALPHA) * g_state.hi_smooth +
                            RUL_EWMA_ALPHA * hi;
    }
    if (g_state.sample_count != UINT32_MAX) ++g_state.sample_count;
    float damage_factor = clampf(1.0f + RUL_DAMAGE_ALPHA * g_state.hi_smooth,
                                 1.0f, 3.0f);
    g_state.cumulative_damage = clampf(g_state.cumulative_damage +
        RUL_CALC_INTERVAL_HOURS / life_at_temperature(temperature_c) *
        damage_factor, 0.0f, 1.0f);
    return RUL_GetPercent();
}
