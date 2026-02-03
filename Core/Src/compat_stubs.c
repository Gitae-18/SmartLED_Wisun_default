/*
 * compat_stubs.c
 *
 *  Created on: Sep 19, 2025
 *      Author: nicek
 */


// Core/Src/compat_stubs.c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---------- storage_mid 관련 (지금은 사용 안 함: 플래시 접근 제거) ---------- */
/* ---------- UID 12바이트 패킹 (메인에서 참조) ---------- */
void mid_pack_uid12(const uint32_t uid_words[3], uint8_t out12[12]) {
    // uid_words: 32비트 x3 (총 12바이트)라고 가정 → 그대로 복사
    if (!uid_words || !out12) return;
    memcpy(out12, uid_words, 12);
}

/* ---------- CBOR 출력 헬퍼 (cbor_format.c가 참조) ---------- */
/* major type 헤더 작성: put_tN(dst, cap, major(0..7), len) → 사용 바이트 수(0=실패) */
size_t put_tN(uint8_t *dst, size_t cap, uint8_t major, size_t len) {
    if (!dst || cap == 0) return 0;
    if (major > 7) return 0;
    if (len < 24) {
        if (cap < 1) return 0;
        dst[0] = (uint8_t)((major << 5) | (uint8_t)len);
        return 1;
    } else if (len <= 0xFF) {
        if (cap < 2) return 0;
        dst[0] = (uint8_t)((major << 5) | 24);
        dst[1] = (uint8_t)len;
        return 2;
    } else if (len <= 0xFFFF) {
        if (cap < 3) return 0;
        dst[0] = (uint8_t)((major << 5) | 25);
        dst[1] = (uint8_t)((len >> 8) & 0xFF);
        dst[2] = (uint8_t)(len & 0xFF);
        return 3;
    } else {
        if (cap < 5) return 0;
        dst[0] = (uint8_t)((major << 5) | 26);
        dst[1] = (uint8_t)((len >> 24) & 0xFF);
        dst[2] = (uint8_t)((len >> 16) & 0xFF);
        dst[3] = (uint8_t)((len >> 8) & 0xFF);
        dst[4] = (uint8_t)(len & 0xFF);
        return 5;
    }
}

/* text string(major=3) 쓰기: put_t3(dst, cap, s, len) → 총 바이트 수(0=실패) */
size_t put_t3(uint8_t *dst, size_t cap, const char *s, size_t len) {
    if (!dst || (!s && len)) return 0;
    size_t h = put_tN(dst, cap, 3, len);
    if (!h) return 0;
    if (cap < h + len) return 0;
    if (len) memcpy(dst + h, s, len);
    return h + len;
}
static inline size_t put_t4(uint8_t *o, size_t i,
                            char a, char b, char c, char d)
{
    o[i++] = 0x64;            // text string, length = 4 (0x60 | 4)
    o[i++] = (uint8_t)a;
    o[i++] = (uint8_t)b;
    o[i++] = (uint8_t)c;
    o[i++] = (uint8_t)d;
    return i;
}
/* byte string(major=2) 쓰기: put_bstr(dst, cap, b, len) → 총 바이트 수(0=실패) */
size_t put_bstr(uint8_t *dst, size_t cap, const uint8_t *b, size_t len) {
    if (!dst || (!b && len)) return 0;
    size_t h = put_tN(dst, cap, 2, len);
    if (!h) return 0;
    if (cap < h + len) return 0;
    if (len) memcpy(dst + h, b, len);
    return h + len;
}
