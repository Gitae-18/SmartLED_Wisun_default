#include "wisun_frame.h"
#include <string.h>
#include <stdio.h>
#include "main.h"
extern UART_HandleTypeDef huart6;
static uint8_t xor_checksum(const uint8_t *p, size_t n){
    uint8_t x = 0;
    for (size_t i=0; i<n; ++i) x ^= p[i];
    return x;
}

bool wisun_send_frame(const wisun_frame_cfg_t *cfg,
                      const uint8_t *data, size_t data_len,
                      wisun_tx_fn tx, void *user)
{
    if (!cfg || !data || !tx) return false;
    if (data_len > 255) return false;     // DL 1바이트 제한

    uint8_t buf[PACKET_MAX_SIZE];
    size_t  i = 0;

    uint8_t sig1 = cfg->sig1 ? cfg->sig1 : 0xAA;
    uint8_t sig2 = cfg->sig2 ? cfg->sig2 : 0xAB;
    uint16_t tmid = cfg->tmid;

    buf[i++] = 0x02;              // STX
    buf[i++] = sig1;              // SIG1 (0xAA)
    buf[i++] = sig2;              // SIG2 (0xAA/0xAB)
    buf[i++] = (uint8_t)data_len; // DL = CBOR 길이
    buf[i++] = (uint8_t)(tmid & 0xFF);
    buf[i++] = (uint8_t)(tmid >> 8);

    memcpy(&buf[i], data, data_len);
    i += data_len;

    // CK: SIG1~SIG2~DL~TMID~DATA
    uint8_t ck = xor_checksum(&buf[1], (uint16_t)(2 + 1 + 2 + data_len));
    buf[i++] = ck;
    buf[i++] = 0x03;              // ETX

    // 실제 UART 전송 함수 호출
    return tx(buf, (uint16_t)i, user);
}

bool wisun_parse_frame(const uint8_t *buf, size_t len, wisun_frame_view_t *out)
{
    if (!buf || !out || len < 8) return false;
    if (buf[0] != 0x02) return false;

    uint8_t sig1 = buf[1], sig2 = buf[2];
    if (sig1 != 0xAA) return false;
    if (!(sig2 == 0xAA || sig2 == 0xAB)) return false;

    uint8_t dl = buf[3];

    uint16_t meta_len = (sig2 == 0xAA) ? 10u : 0u;
    uint16_t data_off = (sig2 == 0xAA) ? 16u : 6u; // AA: 6+10, AB:6

    // --- 1) AB는 고정: dl = DATA 길이 ---
    if (sig2 == 0xAB) {
        uint16_t ck_pos  = (uint16_t)(6u + dl);
        uint16_t etx_pos = (uint16_t)(ck_pos + 1u);
        uint16_t need    = (uint16_t)(etx_pos + 1u);
        if (len < need) return false;
        if (buf[etx_pos] != 0x03) return false;

        uint16_t ck_len = (uint16_t)(2u + 1u + 2u + dl);
        if (xor_checksum(&buf[1], ck_len) != buf[ck_pos]) return false;

        if (dl < 2u) return false;
        const uint8_t *p = &buf[data_off];
        out->tmid = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        out->data = p + 2u;
        out->data_len = (uint8_t)(dl - 2u);
        return true;
    }

    // --- 2) AA는 두 가정 중 하나가 맞는 쪽을 채택 ---
    // 가정 A: dl = DATA 길이
    uint16_t ck_pos_A  = (uint16_t)(6u + meta_len + dl);
    uint16_t etx_pos_A = (uint16_t)(ck_pos_A + 1u);
    uint16_t need_A    = (uint16_t)(etx_pos_A + 1u);

    // 가정 B: dl = (meta_len + DATA) 길이
    // => CK는 6 + dl, ETX는 그 다음
    uint16_t ck_pos_B  = (uint16_t)(6u + dl);
    uint16_t etx_pos_B = (uint16_t)(ck_pos_B + 1u);
    uint16_t need_B    = (uint16_t)(etx_pos_B + 1u);

    bool use_B = false;

    if (len >= need_A && buf[etx_pos_A] == 0x03) {
        // CK 범위는 SIG1~...~MID(2)+meta(10)+DATA(dl)
        uint16_t ck_len_A = (uint16_t)(2u + 1u + 2u + meta_len + dl);
        if (xor_checksum(&buf[1], ck_len_A) == buf[ck_pos_A]) {
            use_B = false;
        } else {
            // A는 CK 불일치면 B도 시도
            goto try_B;
        }
    } else {
        // A 자체가 길이/ETX 조건 불만족이면 B 시도
        goto try_B;
    }

    // A 사용
    if (dl < 2u) return false;
    {
        const uint8_t *p = &buf[data_off];
        out->tmid = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        out->data = p + 2u;
        out->data_len = (uint8_t)(dl - 2u);
    }
    return true;

try_B:
    if (len < need_B) return false;
    if (buf[etx_pos_B] != 0x03) return false;

    // B에서는 dl이 meta+DATA이므로, CK 범위는 SIG1~...~MID(2)+ (dl 바이트)
    // = (SIG1,SIG2,DL,MID2) = 2+1+2 =5  plus dl
    uint16_t ck_len_B = (uint16_t)(2u + 1u + 2u + dl);
    if (xor_checksum(&buf[1], ck_len_B) != buf[ck_pos_B]) return false;

    // B에서는 DATA 길이 = dl - meta_len
    if (dl < meta_len + 2u) return false;
    {
        uint16_t data_len = (uint16_t)(dl - meta_len);
        const uint8_t *p = &buf[data_off];
        out->tmid = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        out->data = p + 2u;
        out->data_len = (uint8_t)(data_len - 2u);
    }
    return true;
}


