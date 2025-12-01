#include "wisun_frame.h"

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
    if (buf[0] != 0x02) return false;  // STX

    uint8_t sig1 = buf[1], sig2 = buf[2];

    // 1) 새 표준 포맷(LEN = DL) 우선 시도
    if (sig1 == 0xAA && (sig2 == 0xAA || sig2 == 0xAB)) {
        uint8_t  dl      = buf[3];                         // DATA 길이(CBOR 전체)
        uint16_t ck_pos  = (uint16_t)(6 + dl);             // CK 위치
        uint16_t etx_pos = (uint16_t)(ck_pos + 1);
        uint16_t need    = (uint16_t)(etx_pos + 1);        // 총 길이 = 8 + DL

        if (len >= need && buf[etx_pos] == 0x03) {
            uint8_t ck_calc = xor_checksum(&buf[1],
                                           (uint16_t)(2 + 1 + 2 + dl)); // SIG1~TMID~DATA
            if (ck_calc == buf[ck_pos]) {
                out->tmid     = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
                out->data     = &buf[6];      // 여기부터 CBOR
                out->data_len = dl;           // CBOR 전체 길이
                return true;
            }
        }
        // 여기 실패하면 레거시 포맷으로 폴백
    }

    // 2) 레거시 포맷 (기존 형태 유지)
    if (sig1 == 0xAA && sig2 == 0xAA) {
        if (len < 17)              return false;
        if (buf[len - 1] != 0x03)  return false; // 마지막 ETX

        uint16_t tmid = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        const uint8_t *data = &buf[16];
        size_t data_len = len - 17;   // 마지막 ETX 제외

        out->tmid     = tmid;
        out->data     = data;
        out->data_len = (uint8_t)data_len;
        return true;
    }

    return false;
}
