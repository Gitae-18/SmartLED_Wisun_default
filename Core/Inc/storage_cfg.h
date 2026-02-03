#ifndef INC_STORAGE_CFG_H_
#define INC_STORAGE_CFG_H_

#include <stdint.h>
#include <stdbool.h>

// -------------------------
// node_cfg_t 정의
// -------------------------
typedef struct {
    uint8_t  mode;
    uint8_t  light_on_hour;
    uint8_t  light_on_min;
    uint8_t  light_off_hour;
    uint8_t  light_off_min;
    uint8_t  manual_duration_min;
    uint8_t  snap_enable;
    uint8_t  snap_period_min;

    uint16_t mid;           // 노드 MID
    uint8_t  rch[2];
	uint8_t  mid_assigned;  // 0: 미할당, 1: 할당 완료

	uint8_t  on_off_mode;        // 0: 기본, 1: 교정 등
	uint8_t  on_corr_mode;       // ON 시각 교정 모드
	uint16_t on_corr_time_min;   // ON 교정 분(±분 단위)

	uint8_t  off_corr_mode;      // OFF 시각 교정 모드
	uint16_t off_corr_time_min;  // OFF 교정 분

	uint16_t forced_time_min;    // 강제 점등 유지 시간(분)

	uint8_t  saving_mode;        // 절전 모드 on/off
	uint8_t  saving_start_hour;  // 절전 시작 시각
	uint8_t  saving_start_min;
	uint8_t  saving_end_hour;    // 절전 종료 시각
	uint8_t  saving_end_min;
} node_cfg_t;

// flash header struct
typedef struct __attribute__((packed)) {
    uint32_t   magic;
    uint32_t   crc;
    node_cfg_t cfg;
} node_cfg_flash_t;

// constants
#define NODE_CFG_MAGIC       0x4E434647u  // "NCFG"

// flash addresses
#define FLASH_MID_ADDR        0x081FF000UL
#define FLASH_MID_SECTOR      127

#define FLASH_NODECFG_ADDR    0x081FE000UL
#define FLASH_NODECFG_SECTOR  127

#define FLASH_BANK_USED       FLASH_BANK_2

// API
bool save_mid_to_flash(uint16_t mid);
bool load_mid_from_flash(uint16_t *out_mid);

bool save_node_cfg_to_flash(const node_cfg_t *cfg);
bool load_node_cfg_from_flash(node_cfg_t *out);

uint32_t node_cfg_calc_crc(const node_cfg_t *cfg);
void node_cfg_init_default(node_cfg_t *cfg);

#endif /* INC_STORAGE_CFG_H_ */
