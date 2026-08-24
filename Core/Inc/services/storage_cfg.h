#ifndef INC_STORAGE_CFG_H_
#define INC_STORAGE_CFG_H_

#include <stdbool.h>
#include <stdint.h>

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

    uint8_t  coord_enable;
    uint8_t  apply_coord_type;
    int32_t  standard_lat_e7;
    int32_t  standard_lon_e7;
    int32_t  install_lat_e7;
    int32_t  install_lon_e7;
} node_cfg_t;

typedef struct __attribute__((packed)) {
    uint32_t   magic;
    uint32_t   crc;
    node_cfg_t cfg;
} node_cfg_flash_t;

#define NODE_CFG_MAGIC       0x4E434647u

#define FLASH_MID_ADDR        0x081FF000UL
#define FLASH_MID_SECTOR      127

#define FLASH_NODECFG_ADDR    0x081FE000UL
#define FLASH_NODECFG_SECTOR  127

#define FLASH_BANK_USED       FLASH_BANK_2

bool save_mid_to_flash(uint16_t mid);
bool load_mid_from_flash(uint16_t *out_mid);

bool save_node_cfg_to_flash(const node_cfg_t *cfg);
bool load_node_cfg_from_flash(node_cfg_t *out);

uint32_t node_cfg_calc_crc(const node_cfg_t *cfg);
void node_cfg_init_default(node_cfg_t *cfg);

#endif /* INC_STORAGE_CFG_H_ */
