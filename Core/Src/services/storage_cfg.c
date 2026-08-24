#include "services/storage_cfg.h"
#include "stm32h5xx_hal.h"

#include <string.h>

#define NODE_CFG_PRESERVE_BYTES 128u

typedef struct {
    uint8_t  mode;
    uint8_t  light_on_hour;
    uint8_t  light_on_min;
    uint8_t  light_off_hour;
    uint8_t  light_off_min;
    uint8_t  manual_duration_min;
    uint8_t  snap_enable;
    uint8_t  snap_period_min;
    uint16_t mid;
    uint8_t  rch[2];
    uint8_t  mid_assigned;
    uint8_t  on_off_mode;
    uint8_t  on_corr_mode;
    uint16_t on_corr_time_min;
    uint8_t  off_corr_mode;
    uint16_t off_corr_time_min;
    uint16_t forced_time_min;
    uint8_t  saving_mode;
    uint8_t  saving_start_hour;
    uint8_t  saving_start_min;
    uint8_t  saving_end_hour;
    uint8_t  saving_end_min;
} legacy_node_cfg_t;

typedef struct {
    uint8_t  mode;
    uint8_t  light_on_hour;
    uint8_t  light_on_min;
    uint8_t  light_off_hour;
    uint8_t  light_off_min;
    uint8_t  manual_duration_min;
    uint8_t  snap_enable;
    uint16_t snap_interval_sec;
    uint16_t mid;
    uint8_t  rch[2];
    uint8_t  mid_assigned;
    uint8_t  on_off_mode;
    uint8_t  on_corr_mode;
    uint16_t on_corr_time_min;
    uint8_t  off_corr_mode;
    uint16_t off_corr_time_min;
    uint16_t forced_time_min;
    uint8_t  saving_mode;
    uint8_t  saving_start_hour;
    uint8_t  saving_start_min;
    uint8_t  saving_end_hour;
    uint8_t  saving_end_min;
} node_cfg_v1_t;

typedef struct __attribute__((packed)) {
    uint32_t          magic;
    uint32_t          crc;
    legacy_node_cfg_t cfg;
} legacy_node_cfg_flash_t;

typedef struct __attribute__((packed)) {
    uint32_t      magic;
    uint32_t      crc;
    node_cfg_v1_t cfg;
} node_cfg_v1_flash_t;

static uint32_t calc_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

uint32_t node_cfg_calc_crc(const node_cfg_t *cfg)
{
    return calc_crc32((const uint8_t *)cfg, sizeof(*cfg));
}

void node_cfg_init_default(node_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->mode = 1;
    cfg->on_off_mode = cfg->mode;
    cfg->light_on_hour = 19;
    cfg->light_on_min = 0;
    cfg->light_off_hour = 6;
    cfg->light_off_min = 0;
    cfg->manual_duration_min = 30;
    cfg->snap_enable = 1;
    cfg->snap_interval_sec = 60;
    cfg->coord_enable = 0;
    cfg->apply_coord_type = 0;
}

bool save_mid_to_flash(uint16_t mid)
{
    uint8_t cfg_backup[NODE_CFG_PRESERVE_BYTES];
    uint8_t mid_buf[16];

    memcpy(cfg_backup, (const void *)FLASH_NODECFG_ADDR, sizeof(cfg_backup));
    memset(mid_buf, 0xFF, sizeof(mid_buf));
    mid_buf[0] = (uint8_t)((mid >> 8) & 0xFFu);
    mid_buf[1] = (uint8_t)(mid & 0xFFu);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t err = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_USED;
    erase.Sector = FLASH_MID_SECTOR;
    erase.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    for (size_t i = 0u; i < sizeof(cfg_backup); i += 16u) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                FLASH_NODECFG_ADDR + (uint32_t)i,
                (uint32_t)(cfg_backup + i)) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
            FLASH_MID_ADDR, (uint32_t)mid_buf) != HAL_OK) {
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

    if (v == 0xFFFFu) {
        return false;
    }

    *out_mid = v;
    return true;
}

bool save_node_cfg_to_flash(const node_cfg_t *cfg)
{
    node_cfg_flash_t image;
    uint8_t flash_buf[(sizeof(node_cfg_flash_t) + 15u) & ~15u];
    uint8_t *src = flash_buf;
    size_t size = sizeof(flash_buf);
    uint8_t mid_backup[16];

    memset(flash_buf, 0xFF, sizeof(flash_buf));
    memcpy(mid_backup, (const void *)FLASH_MID_ADDR, sizeof(mid_backup));

    image.magic = NODE_CFG_MAGIC;
    image.cfg = *cfg;
    image.crc = node_cfg_calc_crc(cfg);
    memcpy(flash_buf, &image, sizeof(image));

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t err = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_USED;
    erase.Sector = FLASH_NODECFG_SECTOR;
    erase.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    {
        uint32_t addr = FLASH_NODECFG_ADDR;

        for (size_t i = 0; i < size; i += 16u) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                    addr, (uint32_t)(src + i)) != HAL_OK) {
                HAL_FLASH_Lock();
                return false;
            }
            addr += 16u;
        }
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
            FLASH_MID_ADDR, (uint32_t)mid_backup) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    HAL_FLASH_Lock();
    return true;
}

bool load_node_cfg_from_flash(node_cfg_t *out)
{
    node_cfg_flash_t *cur = (node_cfg_flash_t *)FLASH_NODECFG_ADDR;

    if (cur->magic == NODE_CFG_MAGIC) {
        uint32_t calc = node_cfg_calc_crc(&cur->cfg);

        if (calc == cur->crc) {
            *out = cur->cfg;
            out->forced_time_min = 0u;
            if (out->apply_coord_type > 1u) {
                out->apply_coord_type = 0u;
            }
            if (out->snap_interval_sec == 0u) {
                out->snap_interval_sec = 60u;
            }
            return true;
        }
    }

    {
        node_cfg_v1_flash_t *v1 = (node_cfg_v1_flash_t *)FLASH_NODECFG_ADDR;

        if (v1->magic == NODE_CFG_MAGIC &&
            calc_crc32((const uint8_t *)&v1->cfg, sizeof(v1->cfg)) == v1->crc) {
            memset(out, 0, sizeof(*out));
            out->mode = v1->cfg.mode;
            out->light_on_hour = v1->cfg.light_on_hour;
            out->light_on_min = v1->cfg.light_on_min;
            out->light_off_hour = v1->cfg.light_off_hour;
            out->light_off_min = v1->cfg.light_off_min;
            out->manual_duration_min = v1->cfg.manual_duration_min;
            out->snap_enable = v1->cfg.snap_enable;
            out->snap_interval_sec = v1->cfg.snap_interval_sec;
            out->mid = v1->cfg.mid;
            out->rch[0] = v1->cfg.rch[0];
            out->rch[1] = v1->cfg.rch[1];
            out->mid_assigned = v1->cfg.mid_assigned;
            out->on_off_mode = v1->cfg.on_off_mode;
            out->on_corr_mode = v1->cfg.on_corr_mode;
            out->on_corr_time_min = v1->cfg.on_corr_time_min;
            out->off_corr_mode = v1->cfg.off_corr_mode;
            out->off_corr_time_min = v1->cfg.off_corr_time_min;
            out->forced_time_min = 0u;
            out->saving_mode = v1->cfg.saving_mode;
            out->saving_start_hour = v1->cfg.saving_start_hour;
            out->saving_start_min = v1->cfg.saving_start_min;
            out->saving_end_hour = v1->cfg.saving_end_hour;
            out->saving_end_min = v1->cfg.saving_end_min;
            out->coord_enable = 0u;
            out->apply_coord_type = 0u;
            if (out->snap_interval_sec == 0u) {
                out->snap_interval_sec = 60u;
            }
            return true;
        }
    }

    {
        legacy_node_cfg_flash_t *legacy = (legacy_node_cfg_flash_t *)FLASH_NODECFG_ADDR;

        if (legacy->magic != NODE_CFG_MAGIC) {
            return false;
        }

        if (calc_crc32((const uint8_t *)&legacy->cfg, sizeof(legacy->cfg)) != legacy->crc) {
            return false;
        }

        memset(out, 0, sizeof(*out));
        out->mode = legacy->cfg.mode;
        out->light_on_hour = legacy->cfg.light_on_hour;
        out->light_on_min = legacy->cfg.light_on_min;
        out->light_off_hour = legacy->cfg.light_off_hour;
        out->light_off_min = legacy->cfg.light_off_min;
        out->manual_duration_min = legacy->cfg.manual_duration_min;
        out->snap_enable = legacy->cfg.snap_enable;
        out->snap_interval_sec = (uint16_t)legacy->cfg.snap_period_min * 60u;
        out->mid = legacy->cfg.mid;
        out->rch[0] = legacy->cfg.rch[0];
        out->rch[1] = legacy->cfg.rch[1];
        out->mid_assigned = legacy->cfg.mid_assigned;
        out->on_off_mode = legacy->cfg.on_off_mode;
        out->on_corr_mode = legacy->cfg.on_corr_mode;
        out->on_corr_time_min = legacy->cfg.on_corr_time_min;
        out->off_corr_mode = legacy->cfg.off_corr_mode;
        out->off_corr_time_min = legacy->cfg.off_corr_time_min;
        out->forced_time_min = 0u;
        out->saving_mode = legacy->cfg.saving_mode;
        out->saving_start_hour = legacy->cfg.saving_start_hour;
        out->saving_start_min = legacy->cfg.saving_start_min;
        out->saving_end_hour = legacy->cfg.saving_end_hour;
        out->saving_end_min = legacy->cfg.saving_end_min;
        out->apply_coord_type = 0u;
        return true;
    }
}
