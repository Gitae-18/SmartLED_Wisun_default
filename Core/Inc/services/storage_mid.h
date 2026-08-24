#pragma once
#include <stdint.h>
#include <stdbool.h>

// 플래시 미사용 모드: 런타임 캐시만 사용
void     mid_storage_init(void);      // 부팅 시 MID를 AT로 받아 캐시에 로드
uint16_t mid_storage_get(void);       // 캐시값 조회 (0xFFFF면 미설정)
bool     mid_storage_set(uint16_t v); // 호환성용: 그냥 캐시만 갱신
