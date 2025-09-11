#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CBOR_FFT_MAX  64   // 필요에 맞게 조정

typedef struct {
    float freq;
    float amp;
} cbor_fft_pair_t;

typedef struct {
    // optional fields (존재 안하면 has_* = false)
    bool     has_id;
    bool     has_time;
    bool     has_volt;
    bool     has_curr;
    bool     has_fft;

    uint16_t id;      // "id": u16
    uint32_t time_us; // "t" : u32
    float    volt;    // "v" : f32
    float    curr;    // "i" : f32

    cbor_fft_pair_t fft[CBOR_FFT_MAX];
    uint16_t        fft_len;  // 실사용 길이
} cbor_packet_t;

/* 인코딩 */
bool cbor_encode_packet(uint8_t *out, size_t out_cap,
                        const cbor_packet_t *p, size_t *out_len);

/* 디코딩: 순서 무관, 선택적 필드, 모르는 키 skip */
bool cbor_decode_packet(const uint8_t *in, size_t in_len,
                        cbor_packet_t *out);

/* (선택) 간단 유틸: 상위 N개 FFT 쌍을 세팅하는 헬퍼 */
static inline void cbor_packet_set_fft(cbor_packet_t *p,
                                       const cbor_fft_pair_t *pairs,
                                       uint16_t n) {
    if (!p) return;
    if (n > CBOR_FFT_MAX) n = CBOR_FFT_MAX;
    for (uint16_t i=0; i<n; ++i) p->fft[i] = pairs[i];
    p->fft_len = n;
    p->has_fft = (n > 0);
}

#ifdef __cplusplus
}
#endif
