#include "wisun_frame.h"

static uint8_t xor_checksum(const uint8_t *p, size_t n){
    uint8_t x = 0;
    for (size_t i=0; i<n; ++i) x ^= p[i];
    return x;
}

bool wisun_send_frame(const wisun_frame_cfg_t *cfg,
                      const uint8_t *payload, size_t payload_len,
                      wisun_tx_fn tx, void *user)
{
    if (!cfg || !payload || payload_len > 0xFD) return false; // LEN 1바이트 보호
    uint8_t buf[6 + 0xFD + 2]; // STX(1)+SIG(2)+LEN(1)+TMID(2)+DATA(≤0xFD)+CK+ETX
    size_t i = 0;

    buf[i++] = 0x02;            // STX
    buf[i++] = cfg->sig1;       // SIG1
    buf[i++] = cfg->sig2;       // SIG2
    buf[i++] = (uint8_t)(2 + payload_len + 1); // LEN = DATA + CK + ETX

    // TMID LE
    buf[i++] = (uint8_t)(cfg->tmid & 0xFF);
    buf[i++] = (uint8_t)((cfg->tmid >> 8) & 0xFF);

    // DATA
    for (size_t k=0; k<payload_len; ++k) buf[i++] = payload[k];

    // CK = XOR(SIG..DATA)
    uint8_t ck = xor_checksum(&buf[1], 2 + 1 + 2 + payload_len); // SIG~DATA
    buf[i++] = ck;
    buf[i++] = 0x03;            // ETX

    return tx(buf, (uint16_t)i, user);
}

bool wisun_parse_frame(const uint8_t *buf, size_t len, wisun_frame_view_t *out)
{
    if (!buf || len < 8 || !out) return false;

    if (buf[0] != 0x02) return false;  // STX

    uint8_t sig1 = buf[1], sig2 = buf[2];

    if (sig1 == 0xAA && sig2 == 0xAB) {
        // ✅ 신규 포맷 처리
        uint8_t L = buf[3];
        size_t need = 6 + (size_t)L;
        if (len < need) return false;

        uint16_t tmid = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        const uint8_t *data = &buf[6];
        size_t data_len = (L >= 2) ? (L - 2) : 0;
        if (data_len + 2 != L) return false;

        uint8_t ck_calc = xor_checksum(&buf[1], 2 + 1 + 2 + data_len);
        uint8_t ck_recv = buf[6 + data_len];
        if (ck_calc != ck_recv) return false;
        if (buf[7 + data_len] != 0x03) return false;

        out->tmid     = tmid;
        out->data     = data;
        out->data_len = (uint8_t)data_len;
        return true;
    }

    else if (sig1 == 0xAA && sig2 == 0xAA) {
        // ✅ 레거시 포맷 처리
        if (len < 17) return false;  // 최소한 MAC 끝까지는 있어야
        uint16_t tmid = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        const uint8_t *data = &buf[16];
        size_t data_len = len - 17; // 마지막 ETX는 제외

        if (buf[len - 1] != 0x03) return false;  // ETX 확인

        out->tmid     = tmid;
        out->data     = data;
        out->data_len = (uint8_t)data_len;
        return true;
    }

    return false;
}
