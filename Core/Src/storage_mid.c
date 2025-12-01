#include "storage_mid.h"
#include <stdint.h>
#include <stdbool.h>

/* 런타임 캐시만 사용 */
static uint16_t s_mid_cache = 0xFFFF;

/* 외부 제공 함수(당신 프로젝트에 이미 있을 가능성 높음):
   - AT를 통해 MID를 읽어오는 함수 (동기/타임아웃 포함) */
extern bool at_get_mid(uint16_t *out_mid);  // true=성공

void mid_storage_init(void) {
    // AT 호출 없이 캐시만 초기화
    // 실제 MID는 나중에 mid_storage_set()으로 주입
    extern uint16_t MID_INVALID; // 또는 0xFFFF 직접 사용
    (void)MID_INVALID;
    // s_mid_cache = MID_INVALID;  // 0xFFFF
}
uint16_t mid_storage_get(void)
{
    return s_mid_cache;
}

bool mid_storage_set(uint16_t v)
{
    // 플래시 저장 안 함. 호환성 위해 캐시만 갱신
    s_mid_cache = v;
    return true;
}
