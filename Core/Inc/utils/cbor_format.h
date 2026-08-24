#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CBOR_FFT_MAX  64   // 필요에 맞게 조정
#define CBOR_UID_LEN 12

typedef struct {
    float freq;
    float amp;
} cbor_fft_pair_t;

typedef struct {
    // 기존 필드 (그대로)
    bool     has_id, has_time, has_volt, has_curr, has_fft, has_temp;
    uint16_t id;        // "id": u16  (레거시/옵션)
    uint32_t time_us;   // "t" : u32  (레거시/옵션)
    float    volt;      // "v" : f32
    float    curr;      // "i" : f32
    float    temp;     //"tp"
    cbor_fft_pair_t fft[CBOR_FFT_MAX];
    uint16_t        fft_len;

    // 신규 메타
    bool     has_k;     uint8_t  k;            // 0=req,1=resp,2=evt
    bool     has_topic; char     topic[16];    // "t": string (topic)
    bool     has_mid;   uint16_t mid;          // 대상/송신자 MID
    bool     has_uid;   uint8_t  uid[CBOR_UID_LEN];
    bool     has_n;     uint16_t n;            // nonce
    bool     has_ok;    uint8_t  ok;           // 0/1

    bool     p_has_on;      uint8_t  p_on;         // 스위치
    bool     p_has_new_mid; uint16_t p_new_mid;    // assign_mid용
    bool     p_has_uid;     uint8_t  p_uid[CBOR_UID_LEN]; // assign 대상 uid
    bool     p_has_start_after_ms;  uint32_t p_start_after_ms;  // 시작 지연(ms)
    bool     p_has_slot_len_ms;     uint32_t p_slot_len_ms;     // 슬롯 길이(ms)
    bool     p_has_jitter_ms;       uint32_t p_jitter_ms;       // 지터(ms)
    bool     p_has_max_mid;         uint16_t p_max_mid;
    bool     p_has_period_ms;       uint32_t p_period_ms; // 주기
    // (선택) p 안의 측정치도 허용: 디코더가 p.v/p.i/p.fft도 같게 채우도록
    bool     p_has_volt, p_has_curr, p_has_fft;
    bool     has_cmd;   uint16_t cmd;
    bool     has_raw;   uint16_t raw;
    // (옵션) 브로드캐스트/타깃도 쓰고 싶다면
    bool     has_bc;    uint8_t  bc;
    bool     has_target;uint16_t target;
    bool     has_msg_id;
	uint32_t msg_id;

	// result ("success", "error" 등)
	bool     has_result;
	char     result[16];

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
