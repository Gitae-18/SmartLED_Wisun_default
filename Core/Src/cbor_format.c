#include "cbor_format.h"
#include <string.h>

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

/* -------------------- Encoder -------------------- */
bool cbor_encode_packet(uint8_t *out, size_t cap, const cbor_packet_t *p, size_t *olen)
{
    if (!out || !p || !olen) return false;
    size_t i=0;
    /* 몇 개의 key가 실제로 들어갈지 계산 */
    uint8_t kv = 0;
    if (p->has_id)   kv++;
    if (p->has_time) kv++;
    if (p->has_volt) kv++;
    if (p->has_curr) kv++;
    if (p->has_fft && p->fft_len>0) kv++;

    if (kv==0) kv=0; /* 빈 맵도 허용 가능(0xA0) */
    if (i+1 > cap) return false;
    out[i++] = 0xA0 | kv;  /* map(kv) */

    /* "id": u16 */
    if (p->has_id){
        if (i+1+2+3 > cap) return false;
        i = put_t2(out,i,'i','d');
        i = put_u16(out,i,p->id);
    }
    /* "t": u32 */
    if (p->has_time){
        if (i+1+1+5 > cap) return false;
        i = put_t1(out,i,'t');
        i = put_u32(out,i,p->time_us);
    }
    /* "v": float */
    if (p->has_volt){
        if (i+1+1+5 > cap) return false;
        i = put_t1(out,i,'v');
        i = put_f32(out,i,p->volt);
    }
    /* "i": float (current) */
    if (p->has_curr){
        if (i+1+1+5 > cap) return false;
        i = put_t1(out,i,'i');
        i = put_f32(out,i,p->curr);
    }
    /* "fft": [ [f,a], ... ] */
    if (p->has_fft && p->fft_len>0){
        uint16_t n = p->fft_len;
        if (n > CBOR_FFT_MAX) n = CBOR_FFT_MAX;

        /* key + array header */
        if (i+1+1 > cap) return false;
        i = put_t2(out,i,'f','f'); out[i++]='t'; /* "fft" = 0x63 'f' 'f' 't' */
        /* 위 한 줄을 간단히 안쪼개고: o[i++]=0x63;o[i++]='f';o[i++]='f';o[i++]='t'; */

        out[i++] = 0x80 | (uint8_t)n;  /* array(n), n<=23 가정. 더 크면 확장 필요 */
        for (uint16_t k=0;k<n;k++){
            /* 각 원소 = [freq, amp] */
            if (i+1 > cap) return false;
            out[i++] = 0x82; /* array(2) */
            if (i+5 > cap) return false; i = put_f32(out,i,p->fft[k].freq);
            if (i+5 > cap) return false; i = put_f32(out,i,p->fft[k].amp);
        }
    }

    *olen = i;
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

/* text: 0x60..0x7B (len<=27만 사용) */
static bool read_text(const uint8_t *in,size_t len,size_t *i,char *out,uint8_t *outlen){
    uint8_t b; if (!get_b(in,len,i,&b)) return false;
    if ((b & 0xE0) != 0x60) return false;
    uint8_t L = (uint8_t)(b & 0x1F);
    if (L==0 || L>27) return false;         /* 본 구현은 1~27만 지원 */
    if (*i + L > len) return false;
    for (uint8_t k=0;k<L;k++) out[k]=(char)in[*i + k];
    *i += L;
    *outlen = L;
    return true;
}

/* 최소한의 skip: 우리가 다루는 타입 범위에서만 */
static bool skip_value(const uint8_t *in,size_t len,size_t *i){
    if (*i >= len) return false;
    uint8_t b = in[*i];

    /* uint(0x00..0x1B), negint(0x20..0x3B) — 여기서는 등장 안할 예정: 간단 skip */
    if ((b & 0xE0)==0x00){ /* uint small + u8/u16/u32/u64 */
        uint8_t ai = b & 0x1F; (*i)++;
        if      (ai<=23) return true;
        else if (ai==24){ (*i)++; return *i<=len; }
        else if (ai==25){ (*i)+=2; return *i<=len; }
        else if (ai==26){ (*i)+=4; return *i<=len; }
        else if (ai==27){ (*i)+=8; return *i<=len; }
        return false;
    }
    if ((b & 0xE0)==0x20){ /* negint */
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
        return false; /* 더 큰 길이는 미지원 */
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
        if (b==0xF4 || b==0xF5 || b==0xF6 || b==0xF7) return true; /* false/true/null/undef */
        if (b==0xF9){ (*i)+=2; return *i<=len; } /* half */
        if (b==0xFA){ (*i)+=4; return *i<=len; } /* float32 */
        if (b==0xFB){ (*i)+=8; return *i<=len; } /* float64 */
        return true;
    }
    return false; /* 기타 미지원 */
}

/* array of pairs [ [f,a], ... ] 디코드 */
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
    if ((head & 0xE0) != 0xA0) return false; /* map */
    uint8_t pairs = head & 0x1F;

    for (uint8_t k=0;k<pairs;k++){
        char key[8]={0}; uint8_t kl=0;
        size_t before = i;
        if (!read_text(in,in_len,&i,key,&kl)){
            /* 키가 text가 아니면 skip */
            i = before;
            if (!skip_value(in,in_len,&i)) return false; /* key skip */
            if (!skip_value(in,in_len,&i)) return false; /* val skip */
            continue;
        }

        /* 값 분기 */
        if (kl==2 && key[0]=='i' && key[1]=='d'){            /* u16 */
            uint16_t v=0; if (!parse_u16(in,in_len,&i,&v)) return false;
            out->id = v; out->has_id = true;
        }
        else if (kl==1 && key[0]=='t'){                      /* u32 */
            uint32_t v=0; if (!parse_u32(in,in_len,&i,&v)) return false;
            out->time_us = v; out->has_time = true;
        }
        else if (kl==1 && key[0]=='v'){                      /* float */
            float v=0; if (!parse_f32(in,in_len,&i,&v)) return false;
            out->volt = v; out->has_volt = true;
        }
        else if (kl==1 && key[0]=='i'){                      /* float (current) */
            float v=0; if (!parse_f32(in,in_len,&i,&v)) return false;
            out->curr = v; out->has_curr = true;
        }
        else if (kl==3 && key[0]=='f' && key[1]=='f' && key[2]=='t'){ /* array of pairs */
            uint16_t n=0; if (!parse_fft_array(in,in_len,&i,out->fft,&n)) return false;
            out->fft_len = n; out->has_fft = (n>0);
        }
        else{
            /* 알 수 없는 키 → 값 skip */
            if (!skip_value(in,in_len,&i)) return false;
        }
    }
    return true;
}
