#include "services/ai_settings.h"

#include <stddef.h>
#include <string.h>

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t crc;
    ai_settings_t settings;
} ai_settings_flash_t;

static uint32_t ai_settings_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

void ai_settings_init_default(ai_settings_t *settings)
{
    if (settings == NULL) {
        return;
    }

    memset(settings, 0, sizeof(*settings));
    settings->enabled = 1u;
    settings->interval_sec = AI_SETTINGS_DEFAULT_INTERVAL_SEC;
}

bool ai_settings_load(ai_settings_t *settings)
{
    const ai_settings_flash_t *image =
        (const ai_settings_flash_t *)FLASH_AI_SETTINGS_ADDR;
    uint32_t crc;

    if (settings == NULL || image->magic != AI_SETTINGS_MAGIC) {
        return false;
    }

    crc = ai_settings_crc32(
        (const uint8_t *)&image->settings,
        sizeof(image->settings)
    );
    if (crc != image->crc) {
        return false;
    }

    *settings = image->settings;
    settings->enabled = settings->enabled ? 1u : 0u;
    if (settings->interval_sec == 0u) {
        settings->interval_sec = AI_SETTINGS_DEFAULT_INTERVAL_SEC;
    }
    return true;
}

bool ai_settings_save(const ai_settings_t *settings)
{
    ai_settings_flash_t image;
    uint8_t flash_buf[(sizeof(ai_settings_flash_t) + 15u) & ~15u];
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t erase_error = 0u;
    uint32_t address = FLASH_AI_SETTINGS_ADDR;

    if (settings == NULL) {
        return false;
    }

    memset(flash_buf, 0xFF, sizeof(flash_buf));
    image.magic = AI_SETTINGS_MAGIC;
    image.settings = *settings;
    image.settings.enabled = image.settings.enabled ? 1u : 0u;
    if (image.settings.interval_sec == 0u) {
        image.settings.interval_sec = AI_SETTINGS_DEFAULT_INTERVAL_SEC;
    }
    image.crc = ai_settings_crc32(
        (const uint8_t *)&image.settings,
        sizeof(image.settings)
    );
    memcpy(flash_buf, &image, sizeof(image));

    HAL_FLASH_Unlock();

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_AI_SETTINGS_BANK;
    erase.Sector = FLASH_AI_SETTINGS_SECTOR;
    erase.NbSectors = 1u;
    if (HAL_FLASHEx_Erase(&erase, &erase_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    for (size_t offset = 0u; offset < sizeof(flash_buf); offset += 16u) {
        if (HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_QUADWORD,
                address,
                (uint32_t)(flash_buf + offset)
            ) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        address += 16u;
    }

    HAL_FLASH_Lock();
    return true;
}
