#ifndef WISUN_ROUTER_H
#define WISUN_ROUTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HOP_QUEUE_SIZE   16u
#define HOP_MAX_FRAME    256u
#define HOP_TTL_DEFAULT  20u
#define HOP_SEEN_TABLE_SIZE 16u

typedef struct {
    uint8_t  in_use;
    uint8_t  compact_snap;
    uint16_t tmid;
    uint8_t  data[HOP_MAX_FRAME];
    uint16_t data_len;
    uint32_t due_tick;
} hop_slot_t;

uint8_t enqueue_transport_tx(uint16_t tmid,
                             uint8_t cmd,
							 uint8_t flags,
                             uint16_t msg_id,
                             const uint8_t *body,
                             uint16_t body_len,
                             uint8_t ttl);

void hop_tx_task_poll(void);

bool hop_seen_key(uint32_t key);
void hop_mark_key(uint32_t key);

uint8_t wisun_router_seen_key(uint32_t key);
void wisun_router_mark_key(uint32_t key);

uint8_t wisun_router_enqueue_payload(uint16_t tmid, const uint8_t *data, uint16_t data_len, uint8_t compact_snap, uint16_t jitter_seed);

#ifdef __cplusplus
}
#endif

#endif /* WISUN_ROUTER_H */
