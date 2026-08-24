#ifndef AI_SETTINGS_H
#define AI_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h5xx_hal.h"

typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint16_t interval_sec;
} ai_settings_t;

#define AI_SETTINGS_DEFAULT_INTERVAL_SEC 60u
#define AI_SETTINGS_MAGIC                0x41494346u

#define FLASH_AI_SETTINGS_ADDR           0x081FC000UL
#define FLASH_AI_SETTINGS_SECTOR         126u
#define FLASH_AI_SETTINGS_BANK           FLASH_BANK_2

void ai_settings_init_default(ai_settings_t *settings);
bool ai_settings_load(ai_settings_t *settings);
bool ai_settings_save(const ai_settings_t *settings);

#endif /* AI_SETTINGS_H */
