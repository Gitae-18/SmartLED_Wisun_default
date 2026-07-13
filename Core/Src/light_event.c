#include "light_event.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "debug_log.h"
#include "snapshot.h"

#define FFT_FREQ_SCALE 100.0f
#define FFT_AMP_SCALE  1000.0f
#define LIGHT_EVENT_SENSOR_DELAY_MS 1000u

typedef struct {
    uint8_t  valid;
    float    voltage;
    float    current;
    float    temp;
    uint8_t  fft_count;
    float    fft_freq[LIGHT_EVENT_FFT_PAIRS];
    float    fft_amp[LIGHT_EVENT_FFT_PAIRS];
    uint32_t tick_ms;
    uint32_t snap_count;
    uint8_t  measured_light_on;
} light_event_sensor_cache_t;

extern RTC_HandleTypeDef hrtc;

__attribute__((weak)) volatile uint32_t uid_ram[3];
__attribute__((weak)) uint8_t g_rtc_synced;

__attribute__((weak)) void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id)
{
    (void)req_msg_id;
}

__attribute__((weak)) void snapshot_suppress_next_tx(void)
{
}

__attribute__((weak)) uint8_t current_control_mode(void)
{
    return 0u;
}

__attribute__((weak)) uint8_t light_event_send_transport(uint16_t target_mid,
                                                         uint8_t ttl,
                                                         uint8_t cmd,
                                                         uint8_t flags,
                                                         uint16_t msg_id,
                                                         const uint8_t *body,
                                                         uint16_t body_len)
{
    (void)target_mid;
    (void)ttl;
    (void)cmd;
    (void)flags;
    (void)msg_id;
    (void)body;
    (void)body_len;
    return 0u;
}

__attribute__((weak)) uint8_t light_event_node_ready(void)
{
    return 1u;
}

__attribute__((weak)) uint8_t light_event_tx_blocked(void)
{
    return 0u;
}

static light_state_event_t g_light_event_q[LIGHT_STATE_EVENT_QUEUE_SIZE];
static uint8_t g_light_event_head;
static uint8_t g_light_event_tail;
static uint8_t g_light_event_count;
static uint32_t g_light_event_seq;
static uint8_t g_light_event_reason_context;
static light_event_sensor_cache_t g_light_sensor_cache;

static light_event_send_fn g_send_hook;
static void *g_send_hook_user;
static light_event_state_fn g_ready_hook;
static void *g_ready_hook_user;
static light_event_state_fn g_tx_blocked_hook;
static void *g_tx_blocked_hook_user;

static void light_event_pack_uid12(uint8_t out12[12])
{
    memcpy(out12, (const void *)uid_ram, 12u);
}

static uint8_t light_state_event_refresh_rtc(light_state_event_t *ev)
{
    RTC_TimeTypeDef rtc_time = {0};
    RTC_DateTypeDef rtc_date = {0};

    if (ev == NULL) {
        return 0u;
    }

    /*
     * STM32 RTC shadow register read order:
     * HAL_RTC_GetTime() first, then HAL_RTC_GetDate().
     * Read the hardware RTC immediately before event transmission so that
     * a stale queued/cached RTC value is never sent to the gateway.
     */
    if (HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK) {
        ev->valid_flags &= (uint8_t)~LIGHT_EVENT_VALID_RTC;
        return 0u;
    }

    if (HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK) {
        ev->valid_flags &= (uint8_t)~LIGHT_EVENT_VALID_RTC;
        return 0u;
    }

    ev->rtc_year = (uint16_t)(2000u + rtc_date.Year);
    ev->rtc_month = rtc_date.Month;
    ev->rtc_day = rtc_date.Date;
    ev->rtc_hour = rtc_time.Hours;
    ev->rtc_min = rtc_time.Minutes;
    ev->rtc_sec = rtc_time.Seconds;
    ev->rtc_synced = g_rtc_synced ? 1u : 0u;
    ev->valid_flags |= LIGHT_EVENT_VALID_RTC;

    return 1u;
}

static uint32_t light_event_scale_fft_freq_x100(float freq_hz)
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

static int32_t light_event_scale_fft_amp_x1000(float amp)
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

static uint8_t light_event_is_ready(void)
{
    if (g_ready_hook != NULL) {
        return g_ready_hook(g_ready_hook_user) ? 1u : 0u;
    }

    return light_event_node_ready() ? 1u : 0u;
}

static uint8_t light_event_is_tx_blocked(void)
{
    if (g_tx_blocked_hook != NULL) {
        return g_tx_blocked_hook(g_tx_blocked_hook_user) ? 1u : 0u;
    }

    return light_event_tx_blocked() ? 1u : 0u;
}

static uint8_t light_event_send(uint16_t target_mid,
                                uint8_t ttl,
                                uint8_t cmd,
                                uint8_t flags,
                                uint16_t msg_id,
                                const uint8_t *body,
                                uint16_t body_len)
{
    if (g_send_hook != NULL) {
        return g_send_hook(target_mid, ttl, cmd, flags, msg_id, body, body_len, g_send_hook_user);
    }

    return light_event_send_transport(target_mid, ttl, cmd, flags, msg_id, body, body_len);
}

static uint8_t light_state_event_has_measurements(const light_state_event_t *ev)
{
    if (ev == NULL) {
        return 0u;
    }

    return ((ev->valid_flags & LIGHT_EVENT_REQUIRED_SENSOR_FLAGS) ==
            LIGHT_EVENT_REQUIRED_SENSOR_FLAGS) ? 1u : 0u;
}

static void light_state_event_clear_measurements(light_state_event_t *ev)
{
    if (ev == NULL) {
        return;
    }

    ev->valid_flags &= (uint8_t)~(LIGHT_EVENT_VALID_VI |
                                  LIGHT_EVENT_VALID_TEMP |
                                  LIGHT_EVENT_VALID_FFT);
    ev->voltage = 0.0f;
    ev->current = 0.0f;
    ev->temp = 0.0f;
    ev->fft_count = 0u;
    memset(ev->fft_freq, 0, sizeof(ev->fft_freq));
    memset(ev->fft_amp, 0, sizeof(ev->fft_amp));
}

void light_event_init(void)
{
    __disable_irq();
    memset(g_light_event_q, 0, sizeof(g_light_event_q));
    memset(&g_light_sensor_cache, 0, sizeof(g_light_sensor_cache));
    g_light_event_head = 0u;
    g_light_event_tail = 0u;
    g_light_event_count = 0u;
    g_light_event_seq = 0u;
    g_light_event_reason_context = LIGHT_EVENT_REASON_UNKNOWN;
    __enable_irq();
}

void light_event_set_send_hook(light_event_send_fn fn, void *user)
{
    g_send_hook = fn;
    g_send_hook_user = user;
}

void light_event_set_ready_hook(light_event_state_fn fn, void *user)
{
    g_ready_hook = fn;
    g_ready_hook_user = user;
}

void light_event_set_tx_blocked_hook(light_event_state_fn fn, void *user)
{
    g_tx_blocked_hook = fn;
    g_tx_blocked_hook_user = user;
}

uint8_t light_event_pending_count(void)
{
    uint8_t count;

    __disable_irq();
    count = g_light_event_count;
    __enable_irq();

    return count;
}

uint16_t light_state_event_encode(uint8_t *out, uint16_t out_cap, const light_state_event_t *ev)
{
    uint16_t off = 0u;

    if (out == NULL || ev == NULL || out_cap < LIGHT_STATE_EVENT_BODY_LEN) {
        return 0u;
    }

    out[off++] = LIGHT_STATE_EVENT_CMD;
    light_event_pack_uid12(&out[off]);
    off += 12u;

    memcpy(&out[off], &ev->event_id, sizeof(ev->event_id));
    off += (uint16_t)sizeof(ev->event_id);

    out[off++] = ev->valid_flags;
    out[off++] = ev->light_on;
    out[off++] = ev->mode;
    out[off++] = ev->reason;

    memcpy(&out[off], &ev->tick_ms, sizeof(ev->tick_ms));
    off += (uint16_t)sizeof(ev->tick_ms);

    memcpy(&out[off], &ev->voltage, sizeof(ev->voltage));
    off += (uint16_t)sizeof(ev->voltage);

    memcpy(&out[off], &ev->current, sizeof(ev->current));
    off += (uint16_t)sizeof(ev->current);

    memcpy(&out[off], &ev->temp, sizeof(ev->temp));
    off += (uint16_t)sizeof(ev->temp);

    out[off++] = ev->fft_count;

    for (uint8_t k = 0u; k < LIGHT_EVENT_FFT_PAIRS; ++k) {
        float freq = 0.0f;
        float amp = 0.0f;
        uint32_t freq_x100;
        int32_t amp_x1000;

        if (k < ev->fft_count) {
            freq = ev->fft_freq[k];
            amp = ev->fft_amp[k];
        }

        freq_x100 = light_event_scale_fft_freq_x100(freq);
        amp_x1000 = light_event_scale_fft_amp_x1000(amp);

        memcpy(&out[off], &freq_x100, sizeof(freq_x100));
        off += (uint16_t)sizeof(freq_x100);

        memcpy(&out[off], &amp_x1000, sizeof(amp_x1000));
        off += (uint16_t)sizeof(amp_x1000);
    }

    memcpy(&out[off], &ev->rtc_year, sizeof(ev->rtc_year));
    off += (uint16_t)sizeof(ev->rtc_year);
    out[off++] = ev->rtc_month;
    out[off++] = ev->rtc_day;
    out[off++] = ev->rtc_hour;
    out[off++] = ev->rtc_min;
    out[off++] = ev->rtc_sec;
    out[off++] = ev->rtc_synced;

    return off;
}

void light_event_set_reason(uint8_t reason)
{
    g_light_event_reason_context = reason;
}

void light_sensor_cache_update(float voltage,
                               float current,
                               float temp,
                               uint8_t fft_count,
                               const float *fft_freq,
                               const float *fft_amp,
                               uint32_t snap_count,
                               uint8_t measured_light_on)
{
    light_event_sensor_cache_t cache;

    if (fft_count == 0u || fft_freq == NULL || fft_amp == NULL) {
        return;
    }

    memset(&cache, 0, sizeof(cache));
    cache.valid = 1u;
    cache.voltage = voltage;
    cache.current = current;
    cache.temp = temp;
    cache.fft_count = (fft_count > LIGHT_EVENT_FFT_PAIRS) ? LIGHT_EVENT_FFT_PAIRS : fft_count;
    for (uint8_t k = 0u; k < cache.fft_count; ++k) {
        cache.fft_freq[k] = fft_freq[k];
        cache.fft_amp[k] = fft_amp[k];
    }
    cache.tick_ms = HAL_GetTick();
    cache.snap_count = snap_count;
    cache.measured_light_on = measured_light_on ? 1u : 0u;

    __disable_irq();
    g_light_sensor_cache = cache;
    __enable_irq();
}

static uint8_t light_state_event_fill_measurements(light_state_event_t *ev)
{
    light_event_sensor_cache_t cache;

    if (ev == NULL) {
        return 0u;
    }

    ev->valid_flags |= LIGHT_EVENT_VALID_LIGHT;

    memset(&cache, 0, sizeof(cache));
    __disable_irq();
    cache = g_light_sensor_cache;
    __enable_irq();

    if (cache.valid &&
        cache.measured_light_on == ev->light_on &&
        ((int32_t)(cache.tick_ms - ev->tick_ms) >= 0)) {
        ev->voltage = cache.voltage;
        ev->current = cache.current;
        ev->temp = cache.temp;
        ev->fft_count = cache.fft_count;
        for (uint8_t k = 0u; k < LIGHT_EVENT_FFT_PAIRS; ++k) {
            ev->fft_freq[k] = cache.fft_freq[k];
            ev->fft_amp[k] = cache.fft_amp[k];
        }
        ev->valid_flags |= LIGHT_EVENT_VALID_VI | LIGHT_EVENT_VALID_TEMP;
        if (ev->fft_count > 0u) {
            ev->valid_flags |= LIGHT_EVENT_VALID_FFT;
        }
    } else if (cache.valid) {
        static uint32_t last_state_mismatch_log;
        uint32_t now = HAL_GetTick();

        if ((uint32_t)(now - last_state_mismatch_log) >= 1000u) {
            last_state_mismatch_log = now;
            timing_log("[TLOG_LIGHT_EVT_SENSOR_STALE] t=%lu event_light=%u measured_light=%u snap=%lu cache_age=%lu event_age=%lu cache_before_event=%u\r\n",
                       (unsigned long)now,
                       (unsigned)ev->light_on,
                       (unsigned)cache.measured_light_on,
                       (unsigned long)cache.snap_count,
                       (unsigned long)(now - cache.tick_ms),
                       (unsigned long)(now - ev->tick_ms),
                       ((int32_t)(cache.tick_ms - ev->tick_ms) < 0) ? 1u : 0u);
        }
    } else {
        static uint32_t last_no_cache_log;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_no_cache_log) >= 1000u) {
            last_no_cache_log = now;
        }
    }

    return 1u;
}

void light_state_event_note_if_changed(uint8_t before_on, uint8_t after_on)
{
    light_state_event_t ev;

    if (before_on == after_on) {
        return;
    }

    memset(&ev, 0, sizeof(ev));
    ev.pending = 1u;
    ev.event_id = ++g_light_event_seq;
    if (ev.event_id == 0u) {
        ev.event_id = ++g_light_event_seq;
    }
    ev.light_on = after_on ? 1u : 0u;
    ev.mode = current_control_mode();
    ev.reason = g_light_event_reason_context;
    ev.tick_ms = HAL_GetTick();
    ev.rtc_synced = g_rtc_synced ? 1u : 0u;

    /* Capture the RTC at the state-change point. It is refreshed again
     * immediately before transmission in light_state_event_poll(). */
    (void)light_state_event_refresh_rtc(&ev);

    __disable_irq();
    if (g_light_event_count < LIGHT_STATE_EVENT_QUEUE_SIZE) {
        g_light_event_q[g_light_event_tail] = ev;
        g_light_event_tail = (uint8_t)((g_light_event_tail + 1u) % LIGHT_STATE_EVENT_QUEUE_SIZE);
        g_light_event_count++;
    } else {
        uint8_t last = (uint8_t)((g_light_event_tail + LIGHT_STATE_EVENT_QUEUE_SIZE - 1u) %
                                 LIGHT_STATE_EVENT_QUEUE_SIZE);
        g_light_event_q[last] = ev;
    }
    __enable_irq();
}

void light_state_event_poll(void)
{
    static uint32_t last_try_tick;
    static uint32_t forced_measure_event_id;
    static uint32_t forced_measure_tick;
    uint32_t now = HAL_GetTick();
    light_state_event_t ev;
    uint8_t body[LIGHT_STATE_EVENT_BODY_LEN];
    uint16_t body_len;
    uint16_t msg_id;

    if (g_light_event_count == 0u || !light_event_is_ready()) {
        return;
    }

    if (light_event_is_tx_blocked()) {
        return;
    }

    __disable_irq();
    ev = g_light_event_q[g_light_event_head];
    __enable_irq();

    if ((uint32_t)(now - ev.tick_ms) < LIGHT_EVENT_SENSOR_DELAY_MS) {
        return;
    }

    if (!light_state_event_fill_measurements(&ev)) {
        __disable_irq();
        if (g_light_event_count > 0u) {
            g_light_event_q[g_light_event_head] = ev;
        }
        __enable_irq();
        return;
    }

    if (!light_state_event_has_measurements(&ev) &&
        (forced_measure_event_id != ev.event_id ||
         (uint32_t)(now - forced_measure_tick) >= 500u)) {
        forced_measure_event_id = ev.event_id;
        forced_measure_tick = now;
        timing_log("[TLOG_LIGHT_EVT_FORCE_SNAP] t=%lu event=%lu light=%u age=%lu\r\n",
                   (unsigned long)now,
                   (unsigned long)ev.event_id,
                   (unsigned)ev.light_on,
                   (unsigned long)(now - ev.tick_ms));
        snapshot_suppress_next_tx();
        Send_Monitoring_Snapshot_JSON(0u);
        light_state_event_clear_measurements(&ev);
        (void)light_state_event_fill_measurements(&ev);
    }

    if (!light_state_event_has_measurements(&ev)) {
        __disable_irq();
        if (g_light_event_count > 0u) {
            g_light_event_q[g_light_event_head] = ev;
        }
        __enable_irq();
        return;
    }

    __disable_irq();
    if (g_light_event_count > 0u) {
        g_light_event_q[g_light_event_head] = ev;
    }
    __enable_irq();

    if (ev.reason == LIGHT_EVENT_REASON_UNKNOWN &&
        (uint32_t)(now - last_try_tick) < 100u) {
        return;
    }
    last_try_tick = now;

    /*
     * The event can remain queued while sensor data is collected.
     * Refresh directly from the hardware RTC just before encoding so the
     * gateway receives the current RTC rather than an old queued value.
     */
    if (!light_state_event_refresh_rtc(&ev)) {
        timing_log("[TLOG_LIGHT_EVT_RTC_READ_FAIL] t=%lu event=%lu sync=%u\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)ev.event_id,
                   (unsigned)ev.rtc_synced);
    }

    __disable_irq();
    if (g_light_event_count > 0u) {
        g_light_event_q[g_light_event_head] = ev;
    }
    __enable_irq();

    body_len = light_state_event_encode(body, (uint16_t)sizeof(body), &ev);
    if (body_len == 0u) {
        __disable_irq();
        if (g_light_event_count > 0u) {
            g_light_event_head = (uint8_t)((g_light_event_head + 1u) % LIGHT_STATE_EVENT_QUEUE_SIZE);
            g_light_event_count--;
        }
        __enable_irq();
        return;
    }

    msg_id = (uint16_t)(ev.event_id & 0xFFFFu);
    if (msg_id == 0u) {
        msg_id = 1u;
    }

    if (!light_event_send(0x0000u, 0u, LIGHT_STATE_EVENT_CMD, 0u, msg_id, body, body_len)) {
        return;
    }

    snapshot_note_light_event_tx();

    timing_log("[TLOG_LIGHT_EVT_TX] t=%lu event=%lu light=%u mode=%u reason=%u msg=%u flags=0x%02X temp=%f fft_count=%u fft0=%f/%f rtc=%04u-%02u-%02u %02u:%02u:%02u sync=%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)ev.event_id,
               (unsigned)ev.light_on,
               (unsigned)ev.mode,
               (unsigned)ev.reason,
               (unsigned)msg_id,
               (unsigned)ev.valid_flags,
               ev.temp,
               (unsigned)ev.fft_count,
               ev.fft_freq[0],
               ev.fft_amp[0],
               (unsigned)ev.rtc_year,
               (unsigned)ev.rtc_month,
               (unsigned)ev.rtc_day,
               (unsigned)ev.rtc_hour,
               (unsigned)ev.rtc_min,
               (unsigned)ev.rtc_sec,
               (unsigned)ev.rtc_synced);

    __disable_irq();
    if (g_light_event_count > 0u) {
        g_light_event_head = (uint8_t)((g_light_event_head + 1u) % LIGHT_STATE_EVENT_QUEUE_SIZE);
        g_light_event_count--;
    }
    __enable_irq();
}
