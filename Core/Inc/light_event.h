#ifndef LIGHT_EVENT_H
#define LIGHT_EVENT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LIGHT_EVENT_FFT_PAIRS
#define LIGHT_EVENT_FFT_PAIRS 2u
#endif

#ifndef LIGHT_STATE_EVENT_QUEUE_SIZE
#define LIGHT_STATE_EVENT_QUEUE_SIZE 4u
#endif

#ifndef LIGHT_STATE_EVENT_CMD
#define LIGHT_STATE_EVENT_CMD 0x15u
#endif

#ifndef LIGHT_EVENT_REASON_UNKNOWN
#define LIGHT_EVENT_REASON_UNKNOWN        0u
#define LIGHT_EVENT_REASON_CMD            1u
#define LIGHT_EVENT_REASON_SET_FORCED     2u
#define LIGHT_EVENT_REASON_FORCED_EXPIRE  3u
#define LIGHT_EVENT_REASON_SCHEDULE       4u
#define LIGHT_EVENT_REASON_SAVING         5u
#define LIGHT_EVENT_REASON_RTC_UNSYNCED   6u
#define LIGHT_EVENT_REASON_TEST           7u
#define LIGHT_EVENT_REASON_NODE_CFG       8u
#endif

#define LIGHT_EVENT_VALID_LIGHT 0x01u
#define LIGHT_EVENT_VALID_VI    0x02u
#define LIGHT_EVENT_VALID_TEMP  0x04u
#define LIGHT_EVENT_VALID_FFT   0x08u
#define LIGHT_EVENT_VALID_RTC   0x10u

#define LIGHT_EVENT_REQUIRED_SENSOR_FLAGS \
    (LIGHT_EVENT_VALID_LIGHT | LIGHT_EVENT_VALID_VI | LIGHT_EVENT_VALID_TEMP | LIGHT_EVENT_VALID_FFT)

#define LIGHT_STATE_EVENT_BODY_LEN \
    (1u + 12u + 4u + 1u + 1u + 1u + 1u + 4u + 4u + 4u + 4u + 1u + \
     (8u * LIGHT_EVENT_FFT_PAIRS) + 2u + 1u + 1u + 1u + 1u + 1u + 1u)

typedef struct {
    uint8_t  pending;
    uint32_t event_id;
    uint8_t  light_on;
    uint8_t  mode;
    uint8_t  reason;
    uint32_t tick_ms;
    uint8_t  valid_flags;
    float    voltage;
    float    current;
    float    temp;
    uint8_t  fft_count;
    float    fft_freq[LIGHT_EVENT_FFT_PAIRS];
    float    fft_amp[LIGHT_EVENT_FFT_PAIRS];
    uint16_t rtc_year;
    uint8_t  rtc_month;
    uint8_t  rtc_day;
    uint8_t  rtc_hour;
    uint8_t  rtc_min;
    uint8_t  rtc_sec;
    uint8_t  rtc_synced;
} light_state_event_t;

typedef uint8_t (*light_event_send_fn)(uint16_t target_mid,
                                       uint8_t ttl,
                                       uint8_t cmd,
                                       uint8_t flags,
                                       uint16_t msg_id,
                                       const uint8_t *body,
                                       uint16_t body_len,
                                       void *user);
typedef uint8_t (*light_event_state_fn)(void *user);

void light_event_init(void);
void light_event_set_reason(uint8_t reason);
void light_state_event_note_if_changed(uint8_t before_on, uint8_t after_on);
void light_sensor_cache_update(float voltage,
                               float current,
                               float temp,
                               uint8_t fft_count,
                               const float *fft_freq,
                               const float *fft_amp,
                               uint32_t snap_count,
                               uint8_t measured_light_on);
void light_state_event_poll(void);
uint16_t light_state_event_encode(uint8_t *out, uint16_t out_cap, const light_state_event_t *ev);

void light_event_set_send_hook(light_event_send_fn fn, void *user);
void light_event_set_ready_hook(light_event_state_fn fn, void *user);
void light_event_set_tx_blocked_hook(light_event_state_fn fn, void *user);
uint8_t light_event_pending_count(void);

#ifdef __cplusplus
}
#endif

#endif /* LIGHT_EVENT_H */
