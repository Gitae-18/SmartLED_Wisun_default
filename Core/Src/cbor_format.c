#include "cbor_format.h"
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"   // MCU 맞게 수정

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern size_t put_bstr(uint8_t *dst, size_t cap, const uint8_t *b, size_t len);
/* -------------------- CBOR put helpers -------------------- */
static inline size_t put_b(uint8_t *o, size_t i, uint8_t v){ o[i]=v; return i+1; }
static inline size_t put_u16(uint8_t *o, size_t i, uint16_t v){
    o[i++] = 0x19; o[i++] = (uint8_t)(v>>8); o[i++] = (uint8_t)v; return i;
}
static inline size_t put_u32(uint8_t *o, size_t i, uint32_t v){
    o[i++] = 0x1A; o[i++] = (uint8_t)(v>>24); o[i++] = (uint8_t)(v>>16);
    o[i++] = (uint8_t)(v>>8);  o[i++] = (uint8_t)v; return i;
}
static inline size_t put_f32(uint8_t *o, size_t i, float f){
    union{ float f; uint32_t u; } u = { .f = f };
    o[i++] = 0xFA;
    o[i++] = (uint8_t)(u.u>>24); o[i++] = (uint8_t)(u.u>>16);
    o[i++] = (uint8_t)(u.u>>8);  o[i++] = (uint8_t)u.u;
    return i;
}
static inline size_t put_true(uint8_t *o, size_t i){ o[i]=0xF5; return i+1; }
static inline size_t put_false(uint8_t *o, size_t i){ o[i]=0xF4; return i+1; }
/* text strings */
static inline size_t put_t1(uint8_t *o, size_t i, char a){ o[i++]=0x61; o[i++]=(uint8_t)a; return i; }
static inline size_t put_t2(uint8_t *o, size_t i, char a, char b){ o[i++]=0x62; o[i++]=(uint8_t)a; o[i++]=(uint8_t)b; return i; }
static inline size_t put_t3(uint8_t *o, size_t i, char a, char b, char c){
    o[i++] = 0x63;        // text, len=3  (0x60 | 3)
    o[i++] = (uint8_t)a;
    o[i++] = (uint8_t)b;
    o[i++] = (uint8_t)c;
    return i;
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

/* -------------------- Encoder -------------------- */
bool cbor_encode_packet(uint8_t *out, size_t cap, const cbor_packet_t *p, size_t *olen)
{
    if (!out || !p || !olen) return false;
    size_t i = 0;
    /* --------- 0) 길이/제약 보정 --------- */
    /* topic/fw 문자열 길이 제한 (≤23) */
    char topic_buf[24] = {0};
    uint8_t topic_len = 0, fw_len = 0;
    if (p->has_topic) {
        topic_len = (uint8_t)strnlen(p->topic, 23);
        memcpy(topic_buf, p->topic, topic_len);
        topic_buf[topic_len] = '\0';
    }

    /* --------- 1) key 수 계산 --------- */
    uint8_t kv = 0;
    if (p->has_k)     kv++;
    if (p->has_topic) kv++;                     /* t:string (topic) */
    else if (p->has_time) kv++;                 /* legacy t:u32 — topic 없을 때만 */
    if (p->has_mid)   kv++;
    if (p->has_uid)   kv++;
    if (p->has_n)     kv++;
    if (p->has_ok)    kv++;
    if (p->has_id)    kv++;                     /* legacy */
    if (p->has_volt)  kv++;                     /* legacy */
    if (p->has_curr)  kv++;                     /* legacy */
    if (p->has_fft && p->fft_len>0) kv++;       /* legacy */
    if (p->has_temp)  kv++;
    if (p->has_msg_id) kv++;
    if (p->has_result) kv++;
    uint8_t pv = 0;
    if (p->p_has_on)      pv++;
    if (p->p_has_new_mid) pv++;
    if (p->p_has_uid)     pv++;
    if (p->p_has_period_ms) pv++;

    /* (선택) p.v/p.i/p.fft 지원하려면 여기서 pv++ */

    if (pv) kv++; /* "p" */

    /* 제약: kv, pv, fft_len 모두 23 이하 (short form 사용) */
    if (kv > 23) return false;
    if (pv > 23) return false;
    uint16_t fft_n = p->fft_len;
    if (p->has_fft && fft_n > 23) return false;

    /* --------- 2) top-level map header --------- */
    if (i + 1 > cap) return false;
    out[i++] = 0xA0 |(uint8_t)kv;  /* map(kv) */

    /* --------- 3) 메타/공통 필드 --------- */
    if (p->has_k) {
    	if (i + 3 > cap) return false;          // 2("k") + 1(uint)
		i = put_t1(out, i, 'k');
		i = put_b(out, i, p->k);       /* k는 0/1/2 (uint small) */
    }
    if (p->has_topic) {                          /* t:string (topic) */
        if (i + 1 + 1 + topic_len > cap) return false;
        i = put_t1(out, i, 't');
        out[i++] = 0x60 | topic_len;            // text, length = topic_len
		memcpy(&out[i], topic_buf, topic_len);
		i += topic_len;

    } else if (p->has_time) {                    /* legacy: t:u32 (topic 없을 때만) */
        if (i + 1 + 1 + 5 > cap) return false;
        i = put_t1(out, i, 't');
        i = put_u32(out, i, p->time_us);
    }
    if (p->has_mid) {
    	if (i + 7 > cap) return false;          // 4("mid") + 3(u16)
		i = put_t3(out, i, 'm','i','d');
		i = put_u16(out, i, p->mid);
    }
    if (p->has_uid) {
    	if (i + 4 + 1 + CBOR_UID_LEN > cap) return false;
		i = put_t3(out, i, 'u','i','d');
		{
			size_t written = put_bstr(out + i, cap - i, p->uid, CBOR_UID_LEN);
			if (!written) return false;
			i += written;
		}
    }
    if (p->has_n) {
        if (i + 1 + 3 > cap) return false;
        i = put_t1(out, i, 'n');
        i = put_u16(out, i, p->n);
    }
    if (p->has_ok) {
        if (i + 1 + 1 > cap) return false;
        i = put_t2(out, i, 'o','k');
        i = (p->ok ? put_true : put_false)(out, i);
    }

    /* --------- 4) 레거시(top-level) 측정치 유지 --------- */
    if (p->has_id) {
        if (i + 1 + 3 > cap) return false;
        i = put_t2(out, i, 'i','d');
        i = put_u16(out, i, p->id);
    }
    if (p->has_volt) {
        if (i + 7 > cap) return false;          // 2("v") + 5(f32)
        i = put_t1(out, i, 'v');
        i = put_f32(out, i, p->volt);
    }
    if (p->has_curr) {
        if (i + 7 > cap) return false;          // 2("i") + 5(f32)
        i = put_t1(out, i, 'i');
        i = put_f32(out, i, p->curr);
    }
   /* if (p->has_fft && fft_n > 0) {
             "fft": [ { "freq": f, "amp": a }, ... ]

            if (i + 4 > cap) return false;            key(3) + at least array hdr
            i = put_t3(out, i, 'f','f','t');

             top-level fft 배열 헤더: array(fft_n), n <= 23
            if (i + 1 > cap) return false;
            out[i++] = 0x80 | (uint8_t)fft_n;         array(n)

            for (uint16_t k = 0; k < fft_n; k++) {
                 각 요소: map(2) { "freq": float, "amp": float }
                   대략 20바이트 정도 쓰이므로 여유 체크
                if (i + 20 > cap) return false;

                 map(2)
                out[i++] = 0xA2;

                 key: "freq"
                {
                    const char *kf = "freq";
                    uint8_t Lf = 4;
                    out[i++] = 0x60 | Lf;             text(len=4)
                    memcpy(&out[i], kf, Lf); i += Lf;
                }
                 value: freq (float)
                i = put_f32(out, i, p->fft[k].freq);

                 key: "amp"
                {
                    const char *ka = "amp";
                    uint8_t La = 3;
                    out[i++] = 0x60 | La;             text(len=3)
                    memcpy(&out[i], ka, La); i += La;
                }
                 value: amp (float)
                i = put_f32(out, i, p->fft[k].amp);
            }
        }*/
    if (p->has_fft && fft_n > 0) {
        uint16_t n = fft_n;
        if (n > 23) n = 23; // CBOR 단일바이트 array 헤더 한계 고려

        i = put_t3(out, i, 'f','f','t'); // key: "fft"

        if (i + 1 > cap) return false;
        out[i++] = 0x80 | (uint8_t)n;    // array(n)

        for (uint16_t k = 0; k < n; ++k) {
            // 각 요소는 map(2) : { "freq": f32, "amp": f32 }
            if (i + 1 > cap) return false;
            out[i++] = 0xA2;             // map(2)

            // "freq"
            i = put_t4(out, i, 'f','r','e','q');
            i = put_f32(out, i, p->fft[k].freq);

            // "amp"
            i = put_t3(out, i, 'a','m','p');
            i = put_f32(out, i, p->fft[k].amp);
        }
    }
    if (p->has_temp) {
        if (i + 8 > cap) return false;          // 3("tp") + 5(f32)
        i = put_t2(out, i, 't','p');
        i = put_f32(out, i, p->temp);
    }
    if (p->has_msg_id) {
            /* "msg_id": uint (최대 32bit) */
            const char *k = "msg_id";
            uint8_t L = 6;  // strlen("msg_id")

            // key(text) + value(uint32) 자리 체크
            if (i + 1 + L + 1 + 4 > cap) return false;

            /* key: text string "msg_id" */
            out[i++] = 0x60 | L;         // text, length=L
            memcpy(&out[i], k, L); i += L;

            /* value: uint32 (0x1A + 4바이트) */
            i = put_u32(out, i, p->msg_id);
        }

        if (p->has_result) {
            /* "result": text */
            char result_buf[24] = {0};
            uint8_t result_len = (uint8_t)strnlen(p->result, 23);
            memcpy(result_buf, p->result, result_len);
            result_buf[result_len] = '\0';

            const char *k = "result";
            uint8_t K = 6;  // strlen("result")

            // key(text) + value(text) 자리 체크
            if (i + 1 + K + 1 + result_len > cap) return false;

            /* key: "result" */
            out[i++] = 0x60 | K;
            memcpy(&out[i], k, K); i += K;

            /* value: text string (len = result_len) */
            out[i++] = 0x60 | result_len;
            memcpy(&out[i], result_buf, result_len);
            i += result_len;
        }
    /* --------- 5) p 서브맵(토픽 파라미터) --------- */
    if (pv) {
    	if (i + 3 > cap) return false;           /* 'p' + map header */
		i = put_t1(out, i, 'p');
		out[i++] = 0xA0 | pv;

        if (p->p_has_on) {
		    if (i + 4 > cap) return false;          // 3("on") + 1(bool)
			i = put_t2(out, i, 'o','n');
			i = (p->p_on ? put_true : put_false)(out, i);
        }
        if (p->p_has_new_mid) {
        	if (i + 7 > cap) return false;          // 4("mid") + 3(u16)
			i = put_t3(out, i, 'm','i','d');
			i = put_u16(out, i, p->p_new_mid);
        }
        if (p->p_has_uid) {
            if (i + 4 + 1 + CBOR_UID_LEN > cap) return false;
            i = put_t3(out, i, 'u','i','d');
            {
                size_t written = put_bstr(out + i, cap - i, p->p_uid, CBOR_UID_LEN);
                if (!written) return false;
                i += written;
            }
        }
        if (p->p_has_period_ms) {
            if (i + 8 > cap) return false;          // 3("pr") + 5(u32)
            i = put_t2(out, i, 'p','r');
            i = put_u32(out, i, p->p_period_ms);
        }
        /* (선택) p.v/p.i/p.fft 지원을 원하면 여기에 추가:
           - 키는 'v','i','fft' 그대로 쓰거나, 충돌을 피하려면 다른 키로. */
    }

    *olen = i;
    {
        char dbg[64];
        int n = snprintf(dbg, sizeof(dbg),
                         "[CBOR-ENC] out[0]=%02X len=%u\r\n",
                         out[0], (unsigned)*olen);
        HAL_UART_Transmit(&huart6, (uint8_t*)dbg, (uint16_t)n, HAL_MAX_DELAY);
    }
    return true;
}

/* -------------------- Decode helpers -------------------- */
static inline bool get_b(const uint8_t *in, size_t len, size_t *i, uint8_t *b){
    if (*i >= len) return false; *b = in[*i]; (*i)++; return true;
}
static inline bool get_n(const uint8_t *in, size_t len, size_t *i, uint8_t *buf, size_t n){
    if (*i + n > len) return false; memcpy(buf, &in[*i], n); *i += n; return true;
}
static bool parse_u16(const uint8_t *in,size_t len,size_t *i,uint16_t *v){
    uint8_t b; if (!get_b(in,len,i,&b)) return false; if (b!=0x19) return false;
    uint8_t t[2]; if (!get_n(in,len,i,t,2)) return false;
    *v = (uint16_t)((t[0]<<8)|t[1]); return true;
}
static bool parse_u32(const uint8_t *in,size_t len,size_t *i,uint32_t *v){
    uint8_t b; if (!get_b(in,len,i,&b)) return false; if (b!=0x1A) return false;
    uint8_t t[4]; if (!get_n(in,len,i,t,4)) return false;
    *v = ((uint32_t)t[0]<<24)|((uint32_t)t[1]<<16)|((uint32_t)t[2]<<8)|t[3]; return true;
}
static bool parse_f32(const uint8_t *in,size_t len,size_t *i,float *f){
    uint8_t b; if (!get_b(in,len,i,&b)) return false; if (b!=0xFA) return false;
    union{ uint32_t u; float f; } u={0}; uint8_t t[4]; if (!get_n(in,len,i,t,4)) return false;
    u.u = ((uint32_t)t[0]<<24)|((uint32_t)t[1]<<16)|((uint32_t)t[2]<<8)|t[3]; *f=u.f; return true;
}
static bool parse_bool(const uint8_t *in,size_t len,size_t *i,bool *v){
    uint8_t b; if (!get_b(in,len,i,&b)) return false;
    if (b==0xF5){*v=true; return true;} if (b==0xF4){*v=false; return true;} return false;
}
static bool parse_uint_small(const uint8_t *in,size_t len,size_t *i,uint32_t *v){
    uint8_t b; if (!get_b(in,len,i,&b)) return false;
    if ((b & 0xE0) == 0x00){ /* uint major */
        uint8_t ai = b & 0x1F;
        if (ai <= 23){ *v = ai; return true; }
        else if (ai==24){ uint8_t x; if(!get_b(in,len,i,&x))return false; *v=x; return true; }
        else if (ai==25){ uint16_t x; if(!parse_u16(in,len,i,&x))return false; *v=x; return true; }
        else if (ai==26){ uint32_t x; if(!parse_u32(in,len,i,&x))return false; *v=x; return true; }
    }
    return false;
}
/* text: 0x60..0x7B (len<=27만) */
static bool read_text(const uint8_t *in,size_t len,size_t *i,char *out,uint8_t *outlen){
    uint8_t b; if (!get_b(in,len,i,&b)) return false;
    if ((b & 0xE0) != 0x60) return false;
    uint8_t L = (uint8_t)(b & 0x1F);
    if (L==0 || L>27) return false;
    if (*i + L > len) return false;
    for (uint8_t k=0;k<L;k++) out[k]=(char)in[*i + k];
    *i += L; *outlen = L; return true;
}
static bool parse_bstr_exact(const uint8_t *in,size_t len,size_t *i,uint8_t *out,uint8_t need){
    uint8_t b; if (!get_b(in,len,i,&b)) return false;
    if ((b & 0xE0) != 0x40) return false;
    uint8_t L = b & 0x1F;
    if (L != need) return false;
    if (*i + L > len) return false;
    memcpy(out, &in[*i], L); *i += L; return true;
}
/* 최소한의 skip: 우리가 다루는 타입 범위에서만 */
static bool skip_value(const uint8_t *in,size_t len,size_t *i){
    if (*i >= len) return false;
    uint8_t b = in[*i];

    /* uint/negint */
    if ((b & 0xE0)==0x00 || (b & 0xE0)==0x20){
        uint8_t ai = b & 0x1F; (*i)++;
        if      (ai<=23) return true;
        else if (ai==24){ (*i)++; return *i<=len; }
        else if (ai==25){ (*i)+=2; return *i<=len; }
        else if (ai==26){ (*i)+=4; return *i<=len; }
        else if (ai==27){ (*i)+=8; return *i<=len; }
        return false;
    }
    /* byte/text string */
    if ((b & 0xE0)==0x40 || (b & 0xE0)==0x60){
        uint8_t L = b & 0x1F; (*i)++;
        if      (L<=23){ (*i)+=L; return *i<=len; }
        else if (L==24){ if (*i>=len) return false; uint8_t n=in[*i]; (*i)++; (*i)+=n; return *i<=len; }
        return false;
    }
    /* array */
    if ((b & 0xE0)==0x80){
        uint8_t n = b & 0x1F; (*i)++;
        for (uint8_t k=0;k<n;k++) if (!skip_value(in,len,i)) return false;
        return true;
    }
    /* map */
    if ((b & 0xE0)==0xA0){
        uint8_t n = b & 0x1F; (*i)++;
        for (uint8_t k=0;k<n;k++){
            if (!skip_value(in,len,i)) return false; /* key */
            if (!skip_value(in,len,i)) return false; /* val */
        }
        return true;
    }
    /* simple/float */
    if ((b & 0xE0)==0xE0){
        (*i)++;
        if (b==0xF4 || b==0xF5 || b==0xF6 || b==0xF7) return true;
        if (b==0xF9){ (*i)+=2; return *i<=len; }
        if (b==0xFA){ (*i)+=4; return *i<=len; }
        if (b==0xFB){ (*i)+=8; return *i<=len; }
        return true;
    }
    return false;
}

/* array of pairs [ [f,a], ... ] (legacy) */
static bool parse_fft_array(const uint8_t *in,size_t len,size_t *i,
                            cbor_fft_pair_t *out, uint16_t *outn)
{
    if (*i >= len) return false;
    uint8_t b = in[*i];
    if ((b & 0xE0) != 0x80) return false; /* array */
    uint8_t n = b & 0x1F; (*i)++;
    uint16_t cnt = 0;
    for (uint8_t k=0;k<n;k++){
        if (*i >= len) return false;
        uint8_t a = in[*i]; if (a != 0x82) return false; /* array(2) */
        (*i)++;
        float f=0, a1=0;
        if (!parse_f32(in,len,i,&f))  return false;
        if (!parse_f32(in,len,i,&a1)) return false;
        if (cnt < CBOR_FFT_MAX){ out[cnt].freq=f; out[cnt].amp=a1; cnt++; }
    }
    *outn = cnt;
    return true;
}

/* -------------------- Decoder -------------------- */
bool cbor_decode_packet(const uint8_t *in, size_t in_len, cbor_packet_t *out)
{
    if (!in || !out || in_len<1) return false;
    memset(out, 0, sizeof(*out));

    size_t i=0; uint8_t head;
    if (!get_b(in,in_len,&i,&head)) return false;
    if ((head & 0xE0) != 0xA0) return false; /* top-level map */
    uint8_t pairs = head & 0x1F;

    for (uint8_t k=0;k<pairs;k++){
        char key[8]={0}; uint8_t kl=0;
        size_t before = i;
        if (!read_text(in,in_len,&i,key,&kl)){
            /* 키가 text가 아니면 통째로 skip */
            i = before;
            if (!skip_value(in,in_len,&i)) return false; /* key skip */
            if (!skip_value(in,in_len,&i)) return false; /* val skip */
            continue;
        }

        /* ===== 값 분기 ===== */
        if (kl==1 && key[0]=='k'){                 /* kind: uint small */
            uint32_t v=0; if (!parse_uint_small(in,in_len,&i,&v)) return false;
            out->k = (uint8_t)v; out->has_k = true;
        }
        else if (kl==1 && key[0]=='t'){
            /* 모호성 처리: topic(string) 또는 legacy time(u32) */
            size_t j=i; uint8_t b;
            if (!get_b(in,in_len,&j,&b)) return false;
            if ((b & 0xE0) == 0x60){               /* text → topic */
                uint8_t L=0; i++; /* 이미 b 하나 읽었으니 i를 b 뒤로 */
                if ((b & 0xE0) != 0x60) return false;
                L = (uint8_t)(b & 0x1F);
                if (L==0 || L>27) return false;
                if (i + L > in_len) return false;
                if (L >= sizeof(out->topic)) L = sizeof(out->topic)-1;
                memcpy(out->topic, &in[i], L); out->topic[L]='\0';
                i += L; out->has_topic = true;
            } else {                                /* number → legacy time */
                if (!parse_u32(in,in_len,&i,&out->time_us)) return false;
                out->has_time = true;
            }
        }
        else if (kl==3 && key[0]=='m' && key[1]=='i' && key[2]=='d'){ /* u16 */
            uint16_t v=0; if (!parse_u16(in,in_len,&i,&v)) return false;
            out->mid = v; out->has_mid = true;
        }
        else if (kl==3 && key[0]=='u' && key[1]=='i' && key[2]=='d'){ /* bstr(12) */
            if (!parse_bstr_exact(in,in_len,&i,out->uid,CBOR_UID_LEN)) return false;
            out->has_uid = true;
        }
        else if (kl==1 && key[0]=='n'){            /* u16 (nonce) */
            uint16_t v=0; if (!parse_u16(in,in_len,&i,&v)) return false;
            out->n = v; out->has_n = true;
        }
        else if (kl==2 && key[0]=='o' && key[1]=='k'){ /* bool → 0/1 */
            bool b=false; if (!parse_bool(in,in_len,&i,&b)) return false;
            out->ok = b ? 1 : 0; out->has_ok = true;
        }
        else if (kl==6 && memcmp(key,"msg_id",6)==0){  /* msg_id: uint */
			uint32_t v = 0;
			if (!parse_uint_small(in,in_len,&i,&v)) return false;
			out->msg_id     = v;
			out->has_msg_id = true;
		}
        else if (kl==1 && key[0]=='p'){            /* sub-map */
            uint8_t bh; if (!get_b(in,in_len,&i,&bh)) return false;
            if ((bh & 0xE0) != 0xA0) return false;
            uint8_t pv = bh & 0x1F;
            for (uint8_t m=0;m<pv;m++){
                char pk[8]={0}; uint8_t pl=0;
                if (!read_text(in,in_len,&i,pk,&pl)) return false;
                if (pl==2 && pk[0]=='f' && pk[1]=='w'){            /* fw: text */
                    uint8_t L=0; size_t tcur=i; uint8_t tb;
                    if (!get_b(in,in_len,&tcur,&tb)) return false;
                    if ((tb & 0xE0) != 0x60) return false;
                    L = (uint8_t)(tb & 0x1F);
                    i = tcur;
                    if (L==0 || L>27) return false;
                    if (i + 1 + L > in_len) return false;
                    i++; /* skip tb */
                }
                else if (pl==2 && pk[0]=='o' && pk[1]=='n'){       /* on: bool */
                    bool b=false; if (!parse_bool(in,in_len,&i,&b)) return false;
                    out->p_on = b?1:0; out->p_has_on = true;
                }
                else if (pl==3 && pk[0]=='m' && pk[1]=='i' && pk[2]=='d'){ /* new_mid: u16 */
                    uint16_t v=0; if (!parse_u16(in,in_len,&i,&v)) return false;
                    out->p_new_mid = v; out->p_has_new_mid = true;
                }
                else if (pl==3 && pk[0]=='u' && pk[1]=='i' && pk[2]=='d'){ /* target uid */
                    if (!parse_bstr_exact(in,in_len,&i,out->p_uid,CBOR_UID_LEN)) return false;
                    out->p_has_uid = true;
                }
                else if ((pl==2 && pk[0]=='s' && pk[1]=='a') ||                 /* "sa" */
                         (pl==16 && memcmp(pk,"p_start_after_ms",16)==0)) {     /* "p_start_after_ms" */
                    uint32_t v; if (!parse_uint_small(in,in_len,&i,&v)) return false;
                    out->p_start_after_ms = v; out->p_has_start_after_ms = true;
                }
                else if ((pl==2 && pk[0]=='s' && pk[1]=='l') ||                 /* "sl" */
                         (pl==13 && memcmp(pk,"p_slot_len_ms",13)==0)) {        /* "p_slot_len_ms" */
                    uint32_t v; if (!parse_uint_small(in,in_len,&i,&v)) return false;
                    out->p_slot_len_ms = v; out->p_has_slot_len_ms = true;
                }
                else if ((pl==2 && pk[0]=='j' && pk[1]=='i') ||                 /* "ji" */
                         (pl==12 && memcmp(pk,"p_jitter_ms",12)==0)) {          /* "p_jitter_ms" */
                    uint32_t v; if (!parse_uint_small(in,in_len,&i,&v)) return false;
                    out->p_jitter_ms = v; out->p_has_jitter_ms = true;
                }
                else if ((pl==2 && pk[0]=='m' && pk[1]=='m') ||                 /* "mm" */
                         (pl==9 && memcmp(pk,"p_max_mid",9)==0)) {              /* "p_max_mid" */
                    uint32_t v; if (!parse_uint_small(in,in_len,&i,&v)) return false;
                    out->p_max_mid = (uint16_t)v; out->p_has_max_mid = true;
                }
                else if ((pl==2 && pk[0]=='p' && pk[1]=='r') ||                 /* "pr" */
						 (pl==12 && memcmp(pk,"p_period_ms",12)==0)) {          /* "p_period_ms" */
					uint32_t v; if (!parse_uint_small(in,in_len,&i,&v)) return false;
					out->p_period_ms = v; out->p_has_period_ms = true;
				}
                else {
                    /* (선택) p.v / p.i / p.fft 등을 지원하려면 여기서 처리 */
                    if (!skip_value(in,in_len,&i)) return false;
                }
            }
        }
        /* ===== 레거시 top-level 측정치 (그대로 유지) ===== */
        else if (kl==2 && key[0]=='i' && key[1]=='d'){            /* u16 */
            uint16_t v=0; if (!parse_u16(in,in_len,&i,&v)) return false;
            out->id = v; out->has_id = true;
        }
        else if (kl==2 && key[0]=='t' && key[1]=='p'){  /* temp: float */
			 float v=0; if (!parse_f32(in,in_len,&i,&v)) return false;
			 out->temp = v; out->has_temp = true;
		 }
        else if (kl==1 && key[0]=='v'){                           /* float */
            float v=0; if (!parse_f32(in,in_len,&i,&v)) return false;
            out->volt = v; out->has_volt = true;
        }
        else if (kl==1 && key[0]=='i'){                           /* float */
            float v=0; if (!parse_f32(in,in_len,&i,&v)) return false;
            out->curr = v; out->has_curr = true;
        }
        else if (kl==3 && key[0]=='f' && key[1]=='f' && key[2]=='t'){ /* array of pairs */
            uint16_t n=0; if (!parse_fft_array(in,in_len,&i,out->fft,&n)) return false;
            out->fft_len = n; out->has_fft = (n>0);
        }
        else if (kl==3 && key[0]=='c' && key[1]=='m' && key[2]=='d'){ /* cmd: uint */
            uint32_t v=0; if (!parse_uint_small(in,in_len,&i,&v)) return false;
            out->cmd = (uint16_t)v; out->has_cmd = true;
        }
        else if (kl==3 && key[0]=='r' && key[1]=='a' && key[2]=='w'){ /* raw: uint */
            uint32_t v=0; if (!parse_uint_small(in,in_len,&i,&v)) return false;
            out->raw = (uint16_t)v; out->has_raw = true;
        }
        else if (kl==2 && key[0]=='b' && key[1]=='c'){               /* bc: uint/bool */
            uint32_t v=0; if (!parse_uint_small(in,in_len,&i,&v)) return false;
            out->bc = (uint8_t)v; out->has_bc = true;
        }
        else if (kl==6 && memcmp(key,"result",6)==0){  /* result: text */
			uint8_t b;
			if (!get_b(in,in_len,&i,&b)) return false;
			if ((b & 0xE0) != 0x60) return false;      /* text string */
			uint8_t L = (uint8_t)(b & 0x1F);
			if (L==0 || L>27) return false;
			if (i + L > in_len) return false;

			if (L >= sizeof(out->result)) L = sizeof(out->result)-1;
			memcpy(out->result, &in[i], L);
			out->result[L] = '\0';
			i += L;

			out->has_result = true;
		}
        else if (kl==6 && memcmp(key,"target",6)==0){                /* target: u16 */
            uint16_t v=0; if (!parse_u16(in,in_len,&i,&v)) return false;
            out->target = v; out->has_target = true;
        }
        else{
            /* 알 수 없는 키 → 값 skip */
            if (!skip_value(in,in_len,&i)) return false;
        }
    }
    return true;
}
