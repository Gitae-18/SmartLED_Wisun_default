// Core/Inc/wisun_frame.h
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  sig1;   // 0xAA
    uint8_t  sig2;   // 0xAB
    uint16_t tmid;   // LE
} wisun_frame_cfg_t;

typedef bool (*wisun_tx_fn)(const uint8_t *data, uint16_t len, void *user);

// 송신(이미 있음)
bool wisun_send_frame(const wisun_frame_cfg_t *cfg,
                      const uint8_t *payload, size_t payload_len,
                      wisun_tx_fn tx, void *user);

// ====== 수신 파서 추가 ======
typedef struct {
    uint16_t tmid;
    const uint8_t *data;
    uint8_t  data_len;
} wisun_frame_view_t;

// buf에 담긴 하나의 프레임을 검증/파싱해서 DATA 포인터와 길이, TMID를 반환
bool wisun_parse_frame(const uint8_t *buf, size_t len, wisun_frame_view_t *out);

#ifdef __cplusplus
}
#endif
