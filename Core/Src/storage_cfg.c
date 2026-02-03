#include "storage_cfg.h"
#include "stm32h5xx_hal.h"
#include <string.h>

// ------------------------------------------------------
// CRC
// ------------------------------------------------------
uint32_t node_cfg_calc_crc(const node_cfg_t *cfg)
{
    const uint8_t *p = (const uint8_t*)cfg;
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < sizeof(node_cfg_t); i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// ------------------------------------------------------
// 기본값 초기화
// ------------------------------------------------------
void node_cfg_init_default(node_cfg_t *cfg)
{
    cfg->mode = 2;
    cfg->light_on_hour = 19;
    cfg->light_on_min  = 0;
    cfg->light_off_hour = 6;
    cfg->light_off_min  = 0;
    cfg->manual_duration_min = 30;
    cfg->snap_enable = 1;
    cfg->snap_period_min = 1;
    cfg->mid          = 0;      // 아직 미할당
    cfg->rch[0] = 0;
    cfg->rch[1] = 0;     // 부트스트랩 채널 (필요하면 매크로 사용)
    cfg->mid_assigned = 0;

    cfg->on_off_mode       = 0;
	cfg->on_corr_mode      = 0;
	cfg->on_corr_time_min  = 0;

	cfg->off_corr_mode     = 0;
	cfg->off_corr_time_min = 0;

	cfg->forced_time_min   = 0;

	cfg->saving_mode       = 0;
	cfg->saving_start_hour = 0;
	cfg->saving_start_min  = 0;
	cfg->saving_end_hour   = 0;
	cfg->saving_end_min    = 0;
}

// ------------------------------------------------------
// MID SAVE/LOAD (H5 QUADWORD write)
// ------------------------------------------------------
bool save_mid_to_flash(uint16_t mid)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t err = 0;

    erase.TypeErase  = FLASH_TYPEERASE_SECTORS;
    erase.Banks      = FLASH_BANK_USED;
    erase.Sector     = FLASH_MID_SECTOR;
    erase.NbSectors  = 1;

    if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    uint8_t buf[16] = {0xFF};
    buf[0] = (mid >> 8) & 0xFF;
    buf[1] = mid & 0xFF;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
            FLASH_MID_ADDR, (uint32_t)buf) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return false;
    }

    HAL_FLASH_Lock();
    return true;
}

bool load_mid_from_flash(uint16_t *out_mid)
{
    uint8_t *p = (uint8_t *)FLASH_MID_ADDR;

    uint16_t v = ((uint16_t)p[0] << 8) | p[1];
    if (v == 0xFFFF) return false;

    *out_mid = v;
    return true;
}

// ------------------------------------------------------
// NODE CFG SAVE/LOAD
// ------------------------------------------------------
bool save_node_cfg_to_flash(const node_cfg_t *cfg)
{
    node_cfg_flash_t image;
    image.magic = NODE_CFG_MAGIC;
    image.cfg   = *cfg;
    image.crc   = node_cfg_calc_crc(cfg);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t err = 0;

    erase.TypeErase  = FLASH_TYPEERASE_SECTORS;
    erase.Banks      = FLASH_BANK_USED;
    erase.Sector     = FLASH_NODECFG_SECTOR;
    erase.NbSectors  = 1;

    if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    uint8_t *src = (uint8_t*)&image;
    size_t size = sizeof(image);

    uint32_t addr = FLASH_NODECFG_ADDR;
    for (size_t i = 0; i < size; i += 16) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                addr, (uint32_t)(src + i)) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 16;
    }

    HAL_FLASH_Lock();
    return true;
}

bool load_node_cfg_from_flash(node_cfg_t *out)
{
    node_cfg_flash_t *f = (node_cfg_flash_t *)FLASH_NODECFG_ADDR;

    if (f->magic != NODE_CFG_MAGIC)
        return false;

    uint32_t calc = node_cfg_calc_crc(&f->cfg);
    if (calc != f->crc)
        return false;

    *out = f->cfg;
    return true;
}
