#include "snapshot.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug_log.h"
#include "main.h"
#include "rtc_user.h"
#include "storage_cfg.h"

static snapshot_t g_snap[SNAP_RING_SIZE];
static uint8_t    g_snap_head = 0u;
static bool       g_snap_inited = false;
static uint8_t    g_snap_enable = 0u;
static uint32_t   g_snap_interval_ms = 60000u;
static uint32_t   g_snap_last_rtc_slot = UINT32_MAX;
static uint32_t   g_snap_last_tx_slot = UINT32_MAX;
static uint8_t    g_snap_last_tx_was_response = 0u;
static uint32_t   g_snap_last_light_event_tx_tick = 0u;
static uint8_t    g_snap_skip_after_light_event_pending = 0u;

extern node_cfg_t g_node_cfg;
extern uint16_t my_mid;
extern volatile uint32_t g_monitor_count;
extern RTC_HandleTypeDef hrtc;

void Ultra_StartDmaFrame(void);
bool node_is_provisioned(void);
void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id);

static uint8_t snapshot_rtc_current_slot(uint32_t interval_sec, uint32_t *slot)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    uint32_t sec_of_day;
    uint32_t day_key;

    if (slot == NULL || interval_sec == 0u) {
        return 0u;
    }

    if (!g_rtc_synced) {
        return 0u;
    }

    if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) {
        return 0u;
    }
    if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
        return 0u;
    }

    if (time.Hours > 23u || time.Minutes > 59u || time.Seconds > 59u) {
        return 0u;
    }

    sec_of_day = ((uint32_t)time.Hours * 3600u) +
                 ((uint32_t)time.Minutes * 60u) +
                 (uint32_t)time.Seconds;

    day_key = ((uint32_t)date.Year * 372u) +
              ((uint32_t)date.Month * 31u) +
              (uint32_t)date.Date;
    *slot = (day_key * 86400u + sec_of_day) / interval_sec;
    return 1u;
}

void snapshot_push(bool light_on, float voltage, float current, float temp, float supersonic)
{
    snapshot_t s = {
        .light_on   = light_on,
        .voltage    = voltage,
        .current    = current,
        .supersonic = supersonic,
        .temp       = temp,
        .count      = g_monitor_count,
    };

    g_snap[g_snap_head] = s;
    g_snap_head = (uint8_t)((g_snap_head + 1u) % SNAP_RING_SIZE);
    g_snap_inited = true;
}

static uint8_t snap_fft0_is_valid(float freq_hz, float amp)
{
    if (!isfinite(freq_hz) || !isfinite(amp)) {
        return 0u;
    }

    if (freq_hz < SNAP_FFT_VALID_FREQ_MIN_HZ ||
        freq_hz > SNAP_FFT_VALID_FREQ_MAX_HZ) {
        return 0u;
    }

    if (amp < SNAP_FFT_VALID_AMP_MIN) {
        return 0u;
    }

    return 1u;
}

void snapshot_reconfigure_timer_from_cfg(void)
{
    if (g_node_cfg.snap_enable) {
        uint32_t interval_sec = g_node_cfg.snap_interval_sec;
        g_snap_enable = 1u;

        if (interval_sec == 0u) {
            interval_sec = 60u;
            g_node_cfg.snap_interval_sec = (uint16_t)interval_sec;
        }

        if (interval_sec > 65535u) {
            interval_sec = 65535u;
            g_node_cfg.snap_interval_sec = (uint16_t)interval_sec;
        }

        g_snap_interval_ms = interval_sec * 1000u;
        g_snap_last_rtc_slot = UINT32_MAX;
        uart6_log("[SNAP_CFG] enable=%u interval_sec=%lu my_mid=%u\r\n",
                  (unsigned)g_snap_enable,
                  (unsigned long)interval_sec,
                  (unsigned)my_mid);
        (void)snapshot_rtc_current_slot(interval_sec, &g_snap_last_rtc_slot);
    } else {
        uart6_log("[SNAP_CFG] enable=0 my_mid=%u\r\n", (unsigned)my_mid);
        g_snap_enable = 0u;
        g_snap_last_rtc_slot = UINT32_MAX;
    }
}

uint8_t snapshot_enabled(void)
{
    return g_snap_enable;
}

uint32_t snapshot_interval_ms(void)
{
    return g_snap_interval_ms;
}

uint8_t snapshot_current_rtc_slot(uint32_t *slot)
{
    return snapshot_rtc_current_slot(g_snap_interval_ms / 1000u, slot);
}

void snapshot_mark_tx(uint8_t was_response)
{
    uint32_t rtc_slot = UINT32_MAX;

    if (snapshot_current_rtc_slot(&rtc_slot)) {
        g_snap_last_tx_slot = rtc_slot;
        g_snap_last_tx_was_response = was_response ? 1u : 0u;
    }
}

void snapshot_note_light_event_tx(void)
{
    g_snap_last_light_event_tx_tick = HAL_GetTick();
    g_snap_skip_after_light_event_pending = 1u;
}

void snapshot_poll(uint32_t now,
                   uint8_t ultra_frame_ready,
                   uint8_t ultra_sampling_paused,
                   uint32_t last_light_control_tick)
{
    static uint32_t snap_gate_dbg_tick = 0u;

    if (g_snap_enable && node_is_provisioned()) {
        uint32_t rtc_slot = UINT32_MAX;
        uint32_t interval_sec = g_snap_interval_ms / 1000u;

        now = HAL_GetTick();
        if (snapshot_rtc_current_slot(interval_sec, &rtc_slot)) {
            if (rtc_slot == g_snap_last_rtc_slot) {
                return;
            }

            if ((uint32_t)(now - last_light_control_tick) < SNAP_AFTER_LIGHT_CONTROL_HOLD_MS) {
                return;
            } else {
                if (g_snap_skip_after_light_event_pending &&
                    (uint32_t)(now - g_snap_last_light_event_tx_tick) <= SNAP_SKIP_AFTER_LIGHT_EVENT_MS) {
                    uart6_log("[SNAP_SKIP] reason=after_light_event rtc_slot=%lu age=%lu mid=%u\r\n",
                              (unsigned long)rtc_slot,
                              (unsigned long)(now - g_snap_last_light_event_tx_tick),
                              (unsigned)my_mid);
                    g_snap_skip_after_light_event_pending = 0u;
                    g_snap_last_rtc_slot = rtc_slot;
                    return;
                }

                if (g_snap_skip_after_light_event_pending &&
                    (uint32_t)(now - g_snap_last_light_event_tx_tick) > SNAP_SKIP_AFTER_LIGHT_EVENT_MS) {
                    g_snap_skip_after_light_event_pending = 0u;
                }

                if (rtc_slot == g_snap_last_tx_slot) {
                    uart6_log("[SNAP_SKIP] reason=slot_tx_exists rtc_slot=%lu last_was_resp=%u mid=%u\r\n",
                              (unsigned long)rtc_slot,
                              (unsigned)g_snap_last_tx_was_response,
                              (unsigned)my_mid);
                    g_snap_last_rtc_slot = rtc_slot;
                    return;
                }

                if (!ultra_frame_ready) {
                    static uint32_t wait_log_tick = 0u;

                    if ((uint32_t)(now - wait_log_tick) >= 1000u) {
                        wait_log_tick = now;

                        uart6_log("[SNAP_WAIT] reason=no_ultra_frame rtc_slot=%lu paused=%u now=%lu mid=%u\r\n",
                                (unsigned long)rtc_slot,
                                (unsigned)ultra_sampling_paused,
                                (unsigned long)now,
                                (unsigned)my_mid);
                    }

                    if (ultra_sampling_paused) {
                        uart6_log("[SNAP_WAIT_RESTART_DMA] rtc_slot=%lu mid=%u\r\n",
                                (unsigned long)rtc_slot,
                                (unsigned)my_mid);
                        Ultra_StartDmaFrame();
                    }

                    return;
                }

                uart6_log("[SNAP_POLL] enable=%u prov=%u ready=%u paused=%u now=%lu rtc_slot=%lu interval=%lu mid=%u\r\n",
                        (unsigned)g_snap_enable,
                        (unsigned)(node_is_provisioned() ? 1u : 0u),
                        (unsigned)(ultra_frame_ready ? 1u : 0u),
                        (unsigned)(ultra_sampling_paused ? 1u : 0u),
                        (unsigned long)now,
                        (unsigned long)rtc_slot,
                        (unsigned long)g_snap_interval_ms,
                        (unsigned)my_mid);

                g_snap_last_rtc_slot = rtc_slot;
                Send_Monitoring_Snapshot_JSON(0);
            }
        }
    } else {
        now = HAL_GetTick();
        if ((uint32_t)(now - snap_gate_dbg_tick) >= 10000u) {
            snap_gate_dbg_tick = now;
            uart6_log("[SNAP_GATE] enable=%u prov=%u rtc_synced=%u my_mid=%u last_slot=%lu last_tx_slot=%lu interval=%lu\r\n",
                      (unsigned)g_snap_enable,
                      (unsigned)(node_is_provisioned() ? 1u : 0u),
                      (unsigned)(g_rtc_synced ? 1u : 0u),
                      (unsigned)my_mid,
                      (unsigned long)g_snap_last_rtc_slot,
                      (unsigned long)g_snap_last_tx_slot,
                      (unsigned long)g_snap_interval_ms);
        }
    }
}

uint8_t apply_snapshot_uart_cmd(const char *arg)
{
    char *endp = NULL;
    unsigned long minutes_ul;
    uint8_t minutes;
    uint16_t interval_sec;
    char msg[96];

    if (arg == NULL) {
        debug6("[UART SNAP] missing argument\r\n");
        return 0u;
    }

    if (strcmp(arg, "OFF") == 0 || strcmp(arg, "0") == 0) {
        g_node_cfg.snap_enable = 0u;
        snapshot_reconfigure_timer_from_cfg();

        if (!save_node_cfg_to_flash(&g_node_cfg)) {
            debug6("[UART SNAP] save failed\r\n");
            return 0u;
        }

        debug6("[UART SNAP] disabled\r\n");
        return 1u;
    }

    minutes_ul = strtoul(arg, &endp, 10);
    if (endp == arg || *endp != '\0') {
        debug6("[UART SNAP] invalid value\r\n");
        return 0u;
    }

    if (minutes_ul == 0UL || minutes_ul > 120UL) {
        debug6("[UART SNAP] range is 1..120 min\r\n");
        return 0u;
    }

    minutes = (uint8_t)minutes_ul;
    interval_sec = (uint16_t)minutes * 60u;
    g_node_cfg.snap_enable = 1u;
    g_node_cfg.snap_interval_sec = interval_sec;
    snapshot_reconfigure_timer_from_cfg();

    if (!save_node_cfg_to_flash(&g_node_cfg)) {
        debug6("[UART SNAP] save failed\r\n");
        return 0u;
    }

    snprintf(msg, sizeof(msg),
             "[UART SNAP] enabled interval=%u sec (%lu ms)\r\n",
             (unsigned)g_node_cfg.snap_interval_sec,
             (unsigned long)g_snap_interval_ms);
    debug6(msg);
    return 1u;
}

uint32_t scale_fft_freq_x100(float freq_hz)
{
    double scaled;

    if (!isfinite(freq_hz) || freq_hz <= 0.0f) {
        return 0u;
    }

    scaled = ((double)freq_hz * (double)FFT_FREQ_SCALE) + 0.5;
    if (scaled >= (double)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)scaled;
}

int32_t scale_fft_amp_x1000(float amp)
{
    double scaled;

    if (!isfinite(amp)) {
        return 0;
    }

    scaled = (double)amp * (double)FFT_AMP_SCALE;
    scaled += (scaled >= 0.0) ? 0.5 : -0.5;

    if (scaled > (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < (double)INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)scaled;
}

uint32_t scale_ai_mse_x1000000(float mse)
{
    double scaled;

    if (!isfinite(mse) || mse <= 0.0f) {
        return 0u;
    }

    scaled = ((double)mse * (double)AI_MSE_SCALE) + 0.5;
    if (scaled >= (double)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)scaled;
}

float snap_round_4dp(float v)
{
    if (!isfinite(v)) {
        return 0.0f;
    }

    return roundf(v * 10000.0f) * 0.0001f;
}

uint16_t encode_snap_bin(uint8_t *out, uint16_t out_cap, const uint8_t uid12[12], float volt, float curr, float temp, uint8_t light_on, uint8_t fft_count, const float *fft_freq, const float *fft_amp, uint32_t snap_count, uint32_t msg_id, uint8_t ok, int8_t err_code, uint8_t ai_valid, uint32_t ai_mse_x1000000, int8_t ai_pred_value)
{
    uint16_t off = 0u;
    float volt_4dp = snap_round_4dp(volt);
    float curr_4dp = snap_round_4dp(curr);
    float temp_4dp = snap_round_4dp(temp);

    if (out == NULL || uid12 == NULL || out_cap < SNAP_BIN_BODY_LEN) {
        return 0u;
    }

    out[off++] = 0x01u;
    memcpy(&out[off], uid12, 12u);
    off += 12u;

    memcpy(&out[off], &volt_4dp, sizeof(volt_4dp));
    off += (uint16_t)sizeof(volt_4dp);

    memcpy(&out[off], &curr_4dp, sizeof(curr_4dp));
    off += (uint16_t)sizeof(curr_4dp);

    memcpy(&out[off], &temp_4dp, sizeof(temp_4dp));
    off += (uint16_t)sizeof(temp_4dp);

    out[off++] = light_on;
    out[off++] = fft_count;

    for (uint8_t k = 0u; k < SNAP_FFT_PAIRS; ++k) {
        float freq = 0.0f;
        float amp = 0.0f;
        uint32_t freq_x100;
        int32_t amp_x1000;

        if (k < fft_count) {
            if (fft_freq != NULL) freq = fft_freq[k];
            if (fft_amp != NULL) amp = fft_amp[k];

            if (!snap_fft0_is_valid(freq, amp)) {
                freq = 0.0f;
                amp = 0.0f;
            }
        }

        freq_x100 = scale_fft_freq_x100(freq);
        amp_x1000 = scale_fft_amp_x1000(amp);

        memcpy(&out[off], &freq_x100, sizeof(freq_x100));
        off += (uint16_t)sizeof(freq_x100);
        memcpy(&out[off], &amp_x1000, sizeof(amp_x1000));
        off += (uint16_t)sizeof(amp_x1000);
    }

    memcpy(&out[off], &snap_count, sizeof(snap_count));
    off += (uint16_t)sizeof(snap_count);

    memcpy(&out[off], &msg_id, sizeof(msg_id));
    off += (uint16_t)sizeof(msg_id);

    out[off++] = ai_valid ? 1u : 0u;
    memcpy(&out[off], &ai_mse_x1000000, sizeof(ai_mse_x1000000));
    off += (uint16_t)sizeof(ai_mse_x1000000);
    out[off++] = (uint8_t)ai_pred_value;

    out[off++] = ok;
    out[off++] = (uint8_t)err_code;

    return off;
}

uint16_t encode_snap_compact_bin(uint8_t *out, uint16_t out_cap, const uint8_t uid12[12], float volt, float curr, float temp, uint8_t light_on, uint8_t has_fft0, float fft0_freq, float fft0_amp, uint32_t snap_count, uint8_t ai_valid, uint32_t ai_mse_x1000000, int8_t ai_pred_value, uint8_t ok, uint8_t ttl)
{
    uint16_t off = 0u;
    float volt_4dp = snap_round_4dp(volt);
    float curr_4dp = snap_round_4dp(curr);
    float temp_4dp = snap_round_4dp(temp);
    uint8_t fft0_valid = (has_fft0 && isfinite(fft0_freq) && isfinite(fft0_amp) && fft0_freq > 0.0f && fft0_amp > 0.0f) ? 1u : 0u;
    uint32_t freq_x100 = fft0_valid ? scale_fft_freq_x100(fft0_freq) : 0u;
    int32_t amp_x1000 = fft0_valid ? scale_fft_amp_x1000(fft0_amp) : 0;
    uint16_t snap_count16 = (uint16_t)(snap_count & 0xFFFFu);
    uint16_t ai_mse16 = (ai_mse_x1000000 > 0xFFFFu) ? 0xFFFFu : (uint16_t)ai_mse_x1000000;
    uint8_t flags = 0u;

    if (out == NULL || uid12 == NULL || out_cap < SNAP_COMPACT_BODY_LEN) {
        return 0u;
    }

    if (ai_valid) flags |= 0x01u;
    if (ai_pred_value != 0) flags |= 0x02u;
    if (ok) flags |= 0x04u;

    out[off++] = 0x01u;
    out[off++] = ttl;
    memcpy(&out[off], uid12, 12u);
    off += 12u;

    memcpy(&out[off], &volt_4dp, sizeof(volt_4dp));
    off += (uint16_t)sizeof(volt_4dp);
    memcpy(&out[off], &curr_4dp, sizeof(curr_4dp));
    off += (uint16_t)sizeof(curr_4dp);
    memcpy(&out[off], &temp_4dp, sizeof(temp_4dp));
    off += (uint16_t)sizeof(temp_4dp);

    out[off++] = light_on ? 1u : 0u;

    memcpy(&out[off], &freq_x100, sizeof(freq_x100));
    off += (uint16_t)sizeof(freq_x100);
    memcpy(&out[off], &amp_x1000, sizeof(amp_x1000));
    off += (uint16_t)sizeof(amp_x1000);

    memcpy(&out[off], &snap_count16, sizeof(snap_count16));
    off += (uint16_t)sizeof(snap_count16);
    memcpy(&out[off], &ai_mse16, sizeof(ai_mse16));
    off += (uint16_t)sizeof(ai_mse16);

    out[off++] = flags;

    return off;
}

bool is_compact_snap_body(const uint8_t *data, uint16_t len)
{
    return data != NULL &&
           len == SNAP_COMPACT_BODY_LEN &&
           data[0] == 0x01u;
}

uint16_t compact_snap_count16(const uint8_t *data)
{
    return (uint16_t)data[SNAP_COMPACT_SNAP_COUNT_IDX] |
           ((uint16_t)data[SNAP_COMPACT_SNAP_COUNT_IDX + 1u] << 8);
}

uint32_t compact_snap_seen_key(const uint8_t *data)
{
    uint32_t h = 2166136261u;

    for (uint8_t i = 0u; i < SNAP_COMPACT_UID_LEN; ++i) {
        h ^= data[SNAP_COMPACT_UID_IDX + i];
        h *= 16777619u;
    }

    h ^= data[SNAP_COMPACT_SNAP_COUNT_IDX];
    h *= 16777619u;
    h ^= data[SNAP_COMPACT_SNAP_COUNT_IDX + 1u];
    h *= 16777619u;

    return h;
}
