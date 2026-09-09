#ifndef SIGNAL_RUL_H
#define SIGNAL_RUL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float cumulative_damage;
    float hi_smooth;
    uint32_t sample_count;
} RUL_State;

void RUL_Reset(void);
bool RUL_StateValid(const RUL_State *state);
bool RUL_Restore(const RUL_State *state);
void RUL_GetState(RUL_State *state);
float RUL_GetPercent(void);
float RUL_Update(float temperature_c, float hi);

#endif
