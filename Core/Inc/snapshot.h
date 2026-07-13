#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SNAP_FFT_PAIRS
#define SNAP_FFT_PAIRS 2
#endif

#define FFT_FREQ_SCALE 100.0f
#define FFT_AMP_SCALE  1000.0f
#define AI_MSE_SCALE   1000000.0f

#define SNAP_FFT_VALID_AMP_MIN       0.5f
#define SNAP_FFT_VALID_FREQ_MIN_HZ   80000.0f
#define SNAP_FFT_VALID_FREQ_MAX_HZ   125000.0f
#define SNAP_RING_SIZE  1
#define SNAP_AFTER_LIGHT_CONTROL_HOLD_MS 3000u
#define SNAP_SKIP_AFTER_LIGHT_EVENT_MS 30000u
#define SNAP_USE_LEGACY_STRUCT_TEST 0
#define SNAP_COMPACT_DIRECT_ENABLE 1

#define SNAP_COMPACT_BODY_LEN 40u
#define SNAP_COMPACT_TTL_DEFAULT 3u
#define SNAP_COMPACT_TTL_IDX 1u
#define SNAP_COMPACT_UID_IDX 2u
#define SNAP_COMPACT_UID_LEN 12u
#define SNAP_COMPACT_SNAP_COUNT_IDX 35u
#define SNAP_AI_RESULT_LEN (1u + 4u + 1u)
#define SNAP_BIN_BODY_LEN (1u + 12u + 4u + 4u + 4u + 1u + 1u + (8u * SNAP_FFT_PAIRS) + 4u + 4u + 1u + 1u + SNAP_AI_RESULT_LEN)

typedef struct {
    bool     light_on;
    float    voltage;
    float    current;
    float    supersonic;
    float    temp;
    uint32_t count;
} snapshot_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;
    uint8_t  uid[12];

    float    volt;
    float    curr;
    float    temp;
    uint8_t  light_on;

    uint8_t  fft_count;
    struct {
        uint32_t freq_x100;
        int32_t amp_x1000;
    } fft[SNAP_FFT_PAIRS];

    uint32_t snap_count;
    uint32_t msg_id;
    uint8_t  ai_valid;
    uint32_t ai_mse_x1000000;
    int8_t   ai_pred;
    uint8_t  ok;
    int8_t   err_code;
} SnapBin_t;

void snapshot_push(bool light_on, float voltage, float current, float temp, float supersonic);
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

uint16_t encode_snap_bin(uint8_t *out, uint16_t out_cap, const uint8_t uid12[12], float volt, float curr, float temp, uint8_t light_on, uint8_t fft_count, const float *fft_freq, const float *fft_amp, uint32_t snap_count, uint32_t msg_id, uint8_t ok, int8_t err_code, uint8_t ai_valid, uint32_t ai_mse_x1000000, int8_t ai_pred_value);
uint16_t encode_snap_compact_bin(uint8_t *out, uint16_t out_cap, const uint8_t uid12[12], float volt, float curr, float temp, uint8_t light_on, uint8_t has_fft0, float fft0_freq, float fft0_amp, uint32_t snap_count, uint8_t ai_valid, uint32_t ai_mse_x1000000, int8_t ai_pred_value, uint8_t ok, uint8_t ttl);
bool is_compact_snap_body(const uint8_t *data, uint16_t len);
uint16_t compact_snap_count16(const uint8_t *data);
uint32_t compact_snap_seen_key(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* SNAPSHOT_H */
