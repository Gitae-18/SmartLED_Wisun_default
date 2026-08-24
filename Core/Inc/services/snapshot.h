#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SNAP_RING_SIZE
#define SNAP_RING_SIZE 10u
#endif

#ifndef SNAP_FFT_PAIRS
#define SNAP_FFT_PAIRS 2u
#endif

#ifndef FFT_FREQ_SCALE
#define FFT_FREQ_SCALE 100.0f
#endif

#ifndef FFT_AMP_SCALE
#define FFT_AMP_SCALE 1000.0f
#endif

#ifndef AI_MSE_SCALE
#define AI_MSE_SCALE 1000000.0f
#endif

#ifndef SNAP_FFT_VALID_FREQ_MIN_HZ
#define SNAP_FFT_VALID_FREQ_MIN_HZ 80000.0f
#endif

#ifndef SNAP_FFT_VALID_FREQ_MAX_HZ
#define SNAP_FFT_VALID_FREQ_MAX_HZ 125000.0f
#endif

#ifndef SNAP_FFT_VALID_AMP_MIN
#define SNAP_FFT_VALID_AMP_MIN 0.5f
#endif

#ifndef SNAP_AFTER_LIGHT_CONTROL_HOLD_MS
#define SNAP_AFTER_LIGHT_CONTROL_HOLD_MS 1000u
#endif

#ifndef SNAP_SKIP_AFTER_LIGHT_EVENT_MS
#define SNAP_SKIP_AFTER_LIGHT_EVENT_MS 5000u
#endif

#ifndef SNAP_COMPACT_DIRECT_ENABLE
#define SNAP_COMPACT_DIRECT_ENABLE 1
#endif

#ifndef SNAP_USE_LEGACY_STRUCT_TEST
#define SNAP_USE_LEGACY_STRUCT_TEST 0
#endif

#ifndef SNAP_COMPACT_TTL_DEFAULT
#define SNAP_COMPACT_TTL_DEFAULT 3u
#endif

/* Legacy full SNAP body. */
#define SNAP_BIN_BODY_LEN \
    (1u + 12u + 4u + 4u + 4u + 1u + 1u + \
     (8u * SNAP_FFT_PAIRS) + 4u + 4u + 1u + 4u + 1u + 1u + 1u)

/*
 * Compact SNAP V1 layout (40 bytes)
 *   0       : type
 *   1       : ttl
 *   2..13   : uid[12]
 *   14..17  : voltage float32
 *   18..21  : current float32
 *   22..25  : temperature float32
 *   26      : light_on
 *   27..30  : fft frequency x100
 *   31..34  : fft amplitude x1000
 *   35..36  : snap_count uint16
 *   37..38  : AI MSE uint16
 *   39      : flags
 *
 * Compact SNAP V2 adds:
 *   40      : control mode
 *   41..42  : effective ON time, minute-of-day, little endian
 *   43..44  : effective OFF time, minute-of-day, little endian
 */
#define SNAP_COMPACT_BODY_LEN_V1       40u
#define SNAP_COMPACT_BODY_LEN_V2       45u
#define SNAP_COMPACT_BODY_LEN          SNAP_COMPACT_BODY_LEN_V2

#define SNAP_COMPACT_TTL_IDX            1u
#define SNAP_COMPACT_UID_IDX            2u
#define SNAP_COMPACT_UID_LEN           12u
#define SNAP_COMPACT_SNAP_COUNT_IDX    35u
#define SNAP_COMPACT_FLAGS_IDX         39u
#define SNAP_COMPACT_MODE_IDX          40u
#define SNAP_COMPACT_ON_TIME_IDX       41u
#define SNAP_COMPACT_OFF_TIME_IDX      43u

#define SNAP_CONTROL_TIME_INVALID 0xFFFFu
#define SNAP_COMPACT_TYPE_IDX           0u
typedef struct {
    bool     light_on;
    float    voltage;
    float    current;
    float    supersonic;
    float    temp;
    uint32_t count;
} snapshot_t;

typedef struct __attribute__((packed)) {
    uint32_t freq_x100;
    int32_t  amp_x1000;
} SnapFftScaled_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;
    uint8_t  uid[12];
    float    volt;
    float    curr;
    float    temp;
    uint8_t  light_on;
    uint8_t  fft_count;
    SnapFftScaled_t fft[SNAP_FFT_PAIRS];
    uint32_t snap_count;
    uint32_t msg_id;
    uint8_t  ai_valid;
    uint32_t ai_mse_x1000000;
    int8_t   ai_pred;
    uint8_t  ok;
    int8_t   err_code;
} SnapBin_t;

void snapshot_push(bool light_on,
                   float voltage,
                   float current,
                   float temp,
                   float supersonic);

void snapshot_reconfigure_timer_from_cfg(void);
uint8_t snapshot_enabled(void);
uint32_t snapshot_interval_ms(void);
uint8_t snapshot_current_rtc_slot(uint32_t *slot);
void snapshot_mark_tx(uint8_t was_response);
void snapshot_note_light_event_tx(void);

void snapshot_poll(uint32_t now,
                   uint8_t ultra_frame_ready,
                   uint8_t ultra_sampling_paused,
                   uint32_t last_light_control_tick);

uint8_t apply_snapshot_uart_cmd(const char *arg);

uint32_t scale_fft_freq_x100(float freq_hz);
int32_t scale_fft_amp_x1000(float amp);
uint32_t scale_ai_mse_x1000000(float mse);
float snap_round_4dp(float v);

uint16_t encode_snap_bin(uint8_t *out,
                         uint16_t out_cap,
                         const uint8_t uid12[12],
                         float volt,
                         float curr,
                         float temp,
                         uint8_t light_on,
                         uint8_t fft_count,
                         const float *fft_freq,
                         const float *fft_amp,
                         uint32_t snap_count,
                         uint32_t msg_id,
                         uint8_t ok,
                         int8_t err_code,
                         uint8_t ai_valid,
                         uint32_t ai_mse_x1000000,
                         int8_t ai_pred_value);

/*
 * Compact SNAP V2 receives control mode and effective ON/OFF times
 * calculated by the caller.
 */
uint16_t encode_snap_compact_bin(uint8_t *out,
                                 uint16_t out_cap,
                                 const uint8_t uid12[12],
                                 float volt,
                                 float curr,
                                 float temp,
                                 uint8_t light_on,
                                 uint8_t has_fft0,
                                 float fft0_freq,
                                 float fft0_amp,
                                 uint32_t snap_count,
                                 uint8_t ai_valid,
                                 uint32_t ai_mse_x1000000,
                                 int8_t ai_pred_value,
                                 uint8_t ok,
                                 uint8_t ttl,
                                 uint8_t control_mode,
                                 uint16_t on_time_min,
                                 uint16_t off_time_min);

bool is_compact_snap_body(const uint8_t *data, uint16_t len);
uint16_t compact_snap_count16(const uint8_t *data);
uint32_t compact_snap_seen_key(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* SNAPSHOT_H */
