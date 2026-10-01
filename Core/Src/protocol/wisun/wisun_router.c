#include "protocol/wisun/wisun_router.h"

#include <string.h>

#include "main.h"
#include "utils/debug_log.h"
#include "protocol/wisun/wisun_frame.h"
#include "protocol/wisun/wisun_transport.h"

#ifndef SNAP_COMPACT_BODY_LEN
#define SNAP_COMPACT_BODY_LEN 40u
#endif

#ifndef SNAP_COMPACT_TTL_IDX
#define SNAP_COMPACT_TTL_IDX 1u
#endif

extern uint16_t my_mid;
extern hop_slot_t g_hop_q[HOP_QUEUE_SIZE];
extern uint32_t g_hop_seen_keys[HOP_SEEN_TABLE_SIZE];
extern uint32_t g_hop_seen_ticks[HOP_SEEN_TABLE_SIZE];
extern uint8_t g_hop_seen_count;
extern uint8_t g_hop_seen_pos;

uint16_t build_transport_payload(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t *out, uint16_t out_cap);

static uint16_t router_xorshift16(uint16_t x)
{
    if (x == 0u) {
        x = 0xACE1u;
    }
    x ^= (uint16_t)(x << 7);
    x ^= (uint16_t)(x >> 9);
    x ^= (uint16_t)(x << 8);
    return x;
}

uint32_t wisun_router_payload_signature(const uint8_t *data, uint16_t data_len)
{
    uint32_t hash = 2166136261u;

    if (data == NULL && data_len != 0u) {
        return 0u;
    }

    for (uint16_t i = 0u; i < data_len; ++i) {
        hash ^= data[i];
        hash *= 16777619u;
    }

    return hash;
}

void hop_tx_task_poll(void)
{
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < HOP_QUEUE_SIZE; ++i) {
        uint32_t actual_signature;

        if (!g_hop_q[i].in_use) continue;
        if ((int32_t)(now - g_hop_q[i].due_tick) < 0) continue;

        if (g_hop_q[i].data_len > HOP_MAX_FRAME) {
            uart6_log("[HOP_DROP_CORRUPT] q=%d reason=bad_len len=%u cap=%u\r\n",
                      i,
                      (unsigned)g_hop_q[i].data_len,
                      (unsigned)HOP_MAX_FRAME);
            g_hop_q[i].in_use = 0u;
            g_hop_q[i].compact_snap = 0u;
            g_hop_q[i].data_signature = 0u;
            continue;
        }

        actual_signature = wisun_router_payload_signature(g_hop_q[i].data,
                                                           g_hop_q[i].data_len);
        if (actual_signature != g_hop_q[i].data_signature) {
            uart6_log("[HOP_DROP_CORRUPT] q=%d reason=signature len=%u stored=%08lX actual=%08lX\r\n",
                      i,
                      (unsigned)g_hop_q[i].data_len,
                      (unsigned long)g_hop_q[i].data_signature,
                      (unsigned long)actual_signature);
            dbg_dump_uart6_with_tag("[HOP_CORRUPT_PAYLOAD]",
                                    g_hop_q[i].data,
                                    g_hop_q[i].data_len);
            g_hop_q[i].in_use = 0u;
            g_hop_q[i].compact_snap = 0u;
            g_hop_q[i].data_signature = 0u;
            continue;
        }

        uint8_t ttl_idx = g_hop_q[i].compact_snap ? SNAP_COMPACT_TTL_IDX : 2u;
        uint8_t min_len = g_hop_q[i].compact_snap ? SNAP_COMPACT_BODY_LEN : 3u;

        if (g_hop_q[i].data_len < min_len) {
            g_hop_q[i].in_use = 0u;
            g_hop_q[i].compact_snap = 0u;
            g_hop_q[i].data_signature = 0u;
            continue;
        }

        uint8_t ttl = g_hop_q[i].data[ttl_idx];
        if (ttl == 0u) {
            dbg_dump_uart6_with_tag("[HOP_DROP_TTL0]", g_hop_q[i].data, g_hop_q[i].data_len);
            g_hop_q[i].in_use = 0u;
            g_hop_q[i].compact_snap = 0u;
            g_hop_q[i].data_signature = 0u;
            continue;
        }

        if (!g_hop_q[i].compact_snap &&
            g_hop_q[i].data_len >= 7u &&
            g_hop_q[i].data[3] == 0x46u) {
            uint16_t queued_msg_id =
                ((uint16_t)g_hop_q[i].data[5] << 8) |
                (uint16_t)g_hop_q[i].data[6];

            uart6_log("[FW_VERSION_QUEUE_TX] q=%d msg_id=%u payload_len=%u signature=%08lX\r\n",
                      i,
                      (unsigned)queued_msg_id,
                      (unsigned)g_hop_q[i].data_len,
                      (unsigned long)actual_signature);
            dbg_dump_uart6_with_tag("[FW_VERSION_QUEUE_PAYLOAD]",
                                    g_hop_q[i].data,
                                    g_hop_q[i].data_len);
        }

        g_hop_q[i].data[ttl_idx] = (uint8_t)(ttl - 1u);

        wisun_frame_cfg_t cfg = {
            .sig1 = 0xAA,
            .sig2 = 0xAB,
            .tmid = 0x0000u,
        };

        (void)wisun_send_frame(&cfg,
                               g_hop_q[i].data,
                               g_hop_q[i].data_len,
                               wisun_tx_adapter,
                               NULL);

        g_hop_q[i].in_use = 0u;
        g_hop_q[i].compact_snap = 0u;
        g_hop_q[i].data_signature = 0u;
    }
}

uint8_t enqueue_transport_tx(uint16_t tmid, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t ttl)
{
    uint8_t payload[HOP_MAX_FRAME];
    uint16_t payload_len;
    uint32_t now;
    uint32_t base_delay;
    uint32_t jitter;

    if (tmid == 0u) {
        return 0u;
    }

    payload_len = build_transport_payload(tmid,
                                          (ttl == 0u) ? HOP_TTL_DEFAULT : ttl,
                                          cmd,
                                          flags,
                                          msg_id,
                                          body,
                                          body_len,
                                          payload,
                                          (uint16_t)sizeof(payload));
    if (payload_len == 0u) {
        return 0u;
    }

    now = HAL_GetTick();
    base_delay = 5u + (my_mid % 5u);
    jitter = router_xorshift16((uint16_t)(my_mid ^ (uint16_t)now ^ msg_id)) % 40u;

    for (int qi = 0; qi < HOP_QUEUE_SIZE; ++qi) {
        if (g_hop_q[qi].in_use) continue;

        g_hop_q[qi].tmid = 0x0000u;
        g_hop_q[qi].compact_snap = 0u;
        g_hop_q[qi].data_len = payload_len;
        memcpy(g_hop_q[qi].data, payload, payload_len);
        g_hop_q[qi].data_signature =
            wisun_router_payload_signature(g_hop_q[qi].data, payload_len);
        g_hop_q[qi].due_tick = now + base_delay + jitter;
        g_hop_q[qi].in_use = 1u;

        return 1u;
    }

    return 0u;
}

bool hop_seen_key(uint32_t key)
{
    uint32_t now = HAL_GetTick();
    uint8_t n = (g_hop_seen_count < HOP_SEEN_TABLE_SIZE) ? g_hop_seen_count : HOP_SEEN_TABLE_SIZE;
    for (uint8_t i = 0; i < n; i++) {
        if (g_hop_seen_keys[i] == key) {
            if ((uint32_t)(now - g_hop_seen_ticks[i]) <= 2000u) {
                return true;
            }
            g_hop_seen_keys[i] = 0u;
            g_hop_seen_ticks[i] = 0u;
        }
    }
    return false;
}

void hop_mark_key(uint32_t key)
{
    if (g_hop_seen_count < HOP_SEEN_TABLE_SIZE) {
        g_hop_seen_keys[g_hop_seen_count] = key;
        g_hop_seen_ticks[g_hop_seen_count] = HAL_GetTick();
        g_hop_seen_count++;
    } else {
        g_hop_seen_keys[g_hop_seen_pos] = key;
        g_hop_seen_ticks[g_hop_seen_pos] = HAL_GetTick();
        g_hop_seen_pos = (uint8_t)((g_hop_seen_pos + 1u) % HOP_SEEN_TABLE_SIZE);
    }
}
