#include "light_control.h"

#include "main.h"
#include "storage_cfg.h"
#include "debug_log.h"

#include <stdbool.h>
#include <stdio.h>

#ifndef LIGHT_TEST_10S_ENABLE
#define LIGHT_TEST_10S_ENABLE    0
#endif

#ifndef LIGHT_TEST_10S_PERIOD_MS
#define LIGHT_TEST_10S_PERIOD_MS 10000u
#endif

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart6;
extern node_cfg_t g_node_cfg;
extern uint8_t g_light_on;
extern uint16_t g_sunrise_min;
extern uint16_t g_sunset_min;
extern uint16_t g_dawn_min;
extern uint16_t g_dusk_min;

static uint8_t  g_manual_override_active   = 0;
static uint8_t  g_manual_override_light_on = 0;
static uint32_t g_manual_override_until    = 0;
static uint8_t  g_manual_override_no_timeout = 0;
static uint8_t  g_manual_override_latch_off_on_expire = 0;
static uint32_t g_scheduler_last_gap_ms = 0;
static bool on_done_today  = false;
static bool off_done_today = false;
static uint8_t sched_last_day = 0xFF;

static GPIO_PinState light_pin_state_for(uint8_t want_on);
static int light_norm_min(int t);

void light_on(void)
{
    uint8_t before_on = light_is_on_logical();
    uint8_t after_on = 1u;

    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, light_pin_state_for(1u));
    g_light_on = after_on;

    if (before_on != after_on) {
        timing_log("[TLOG_LIGHT] t=%lu action=ON before=%u after=%u sched_gap=%lu\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned)before_on,
                   (unsigned)after_on,
                   (unsigned long)g_scheduler_last_gap_ms);
    }

    light_state_event_note_if_changed(before_on, after_on);
}

void light_off(void)
{
    uint8_t before_on = light_is_on_logical();
    uint8_t after_on = 0u;

    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, light_pin_state_for(0u));
    g_light_on = after_on;

    if (before_on != after_on) {
        timing_log("[TLOG_LIGHT] t=%lu action=OFF before=%u after=%u sched_gap=%lu\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned)before_on,
                   (unsigned)after_on,
                   (unsigned long)g_scheduler_last_gap_ms);
    }

    light_state_event_note_if_changed(before_on, after_on);
}

static GPIO_PinState light_pin_state_for(uint8_t want_on)
{
#if LIGHT_ACTIVE_LOW
    return want_on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    return want_on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
}

uint8_t light_is_on_logical(void)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(LIGHT_GPIO_Port, LIGHT_Pin);
#if LIGHT_ACTIVE_LOW
    return (raw == GPIO_PIN_RESET) ? 1u : 0u;
#else
    return (raw == GPIO_PIN_SET) ? 1u : 0u;
#endif
}

uint16_t apply_time_correction_min(uint16_t base_min, uint8_t corr_mode, uint16_t corr_delta_min)
{
    if (corr_delta_min == 0u) {
        return base_min;
    }

    switch (corr_mode) {
    case 1u:
        return (uint16_t)light_norm_min((int)base_min + (int)corr_delta_min);
    case 2u:
        return (uint16_t)light_norm_min((int)base_min - (int)corr_delta_min);
    default:
        return base_min;
    }
}

uint8_t time_window_contains(uint16_t now_min, uint16_t start_min, uint16_t end_min)
{
    if (start_min == end_min) {
        return 0u;
    }

    if (start_min < end_min) {
        return (now_min >= start_min && now_min < end_min) ? 1u : 0u;
    }

    return (now_min >= start_min || now_min < end_min) ? 1u : 0u;
}

uint8_t current_control_mode(void)
{
    return g_manual_override_active ? 3u : g_node_cfg.mode;
}

uint8_t light_control_manual_active(void)
{
    return g_manual_override_active;
}

uint16_t manual_override_duration_min(void)
{
    if (g_node_cfg.manual_duration_min > 0u) {
        return g_node_cfg.manual_duration_min;
    }

    return 30u;
}

void start_manual_override(uint8_t want_on)
{
    g_manual_override_latch_off_on_expire = 0u;
    start_manual_override_for(want_on, manual_override_duration_min());
}

void start_manual_override_for(uint8_t want_on, uint16_t duration_min)
{
    uint32_t now = HAL_GetTick();
    uint32_t duration_ms = (uint32_t)duration_min * 60000u;
    uint8_t no_timeout = (duration_min == 0u) ? 1u : 0u;

    if (duration_ms > 0x7fffffffu) {
        duration_ms = 0x7fffffffu;
    }

    /* ?대? 媛숈? 諛⑺뼢??manual override媛 ?쒖꽦 ?곹깭硫??ъ떆?묓븯吏 ?딆쓬 */
    if (g_manual_override_active &&
        g_manual_override_light_on == (want_on ? 1u : 0u) &&
        (g_manual_override_no_timeout || ((int32_t)(g_manual_override_until - now) > 0))) {
        return;
    }

    g_manual_override_active = 1u;
    g_manual_override_light_on = want_on ? 1u : 0u;
    g_manual_override_until = no_timeout ? now : (now + duration_ms);
    g_manual_override_no_timeout = no_timeout;

    if (g_manual_override_light_on) {
        light_on();
    } else {
        light_off();
    }

    if (g_manual_override_no_timeout) {
        timing_log("[TLOG_MANUAL] t=%lu state=%s hold=forever\r\n",
                   (unsigned long)HAL_GetTick(),
                   g_manual_override_light_on ? "ON" : "OFF");
    } else {
        timing_log("[TLOG_MANUAL] t=%lu state=%s hold=%u min\r\n",
                   (unsigned long)HAL_GetTick(),
                   g_manual_override_light_on ? "ON" : "OFF",
                   (unsigned)duration_min);
    }
}

void start_forced_time_control(uint16_t forced_time_min)
{
    g_manual_override_active = 0u;
    g_manual_override_no_timeout = 0u;

    if (forced_time_min == 0u) {
        g_manual_override_latch_off_on_expire = 0u;
        light_event_set_reason(LIGHT_EVENT_REASON_SET_FORCED);
        start_manual_override_for(0u, 0u);
        light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
        return;
    }

    g_manual_override_latch_off_on_expire = 1u;
    light_event_set_reason(LIGHT_EVENT_REASON_SET_FORCED);
    start_manual_override_for(1u, forced_time_min);
    light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
}

void light_control_clear_manual_override(void)
{
    g_manual_override_active = 0u;
    g_manual_override_no_timeout = 0u;
    g_manual_override_latch_off_on_expire = 0u;
}

void scheduler_poll(void)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    uint8_t want_on = 0;
    uint8_t managed = 0;
    uint32_t now_tick = HAL_GetTick();
    uint8_t control_mode = 0;
    static uint32_t last_sched_tick = 0u;
    static uint32_t last_sched_log_tick = 0u;
    static uint8_t last_log_want_on = 0xFFu;
    static uint8_t last_log_light_on = 0xFFu;
    static uint8_t last_log_manual_active = 0xFFu;
    uint32_t sched_gap = (last_sched_tick == 0u) ? 0u : (now_tick - last_sched_tick);
    last_sched_tick = now_tick;
    g_scheduler_last_gap_ms = sched_gap;

    if (g_manual_override_active) {
        if (g_manual_override_no_timeout ||
            ((int32_t)(now_tick - g_manual_override_until) < 0)) {
            if (sched_gap >= 1000u ||
                last_log_manual_active != g_manual_override_active ||
                (uint32_t)(now_tick - last_sched_log_tick) >= 5000u) {
                char msg[192];
                int n = snprintf(msg, sizeof(msg),
                                 "[SCHEDDBG_MANUAL] tick=%lu gap=%lu mode=%u manual=%u man_on=%u no_to=%u until=%lu light=%u\r\n",
                                 (unsigned long)now_tick,
                                 (unsigned long)sched_gap,
                                 (unsigned)g_node_cfg.mode,
                                 (unsigned)g_manual_override_active,
                                 (unsigned)g_manual_override_light_on,
                                 (unsigned)g_manual_override_no_timeout,
                                 (unsigned long)g_manual_override_until,
                                 (unsigned)light_is_on_logical());
                if (n > 0) {
                    HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 50);
                }
                last_sched_log_tick = now_tick;
                last_log_manual_active = g_manual_override_active;
            }

            light_event_set_reason(g_manual_override_latch_off_on_expire ?
                                   LIGHT_EVENT_REASON_SET_FORCED :
                                   LIGHT_EVENT_REASON_CMD);
            if (g_manual_override_light_on) {
                light_on();
            } else {
                light_off();
            }
            light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
            return;
        }

        if (g_manual_override_latch_off_on_expire) {
            g_manual_override_latch_off_on_expire = 0u;
            light_event_set_reason(LIGHT_EVENT_REASON_FORCED_EXPIRE);
            start_manual_override_for(0u, 0u);
            light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
            return;
        }

        g_manual_override_active = 0u;
        g_manual_override_no_timeout = 0u;
        g_manual_override_latch_off_on_expire = 0u;
    }

#if LIGHT_TEST_10S_ENABLE
    light_event_set_reason(LIGHT_EVENT_REASON_TEST);
    if (((now_tick / LIGHT_TEST_10S_PERIOD_MS) & 0x1u) != 0u) {
        light_on();
    } else {
        light_off();
    }
    light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
    return;
#endif

    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

    uint16_t now_min = (uint16_t)t.Hours * 60u + t.Minutes;
    uint16_t on_min = 0u;
    uint16_t off_min = 0u;
    uint16_t saving_start_min = (uint16_t)g_node_cfg.saving_start_hour * 60u + g_node_cfg.saving_start_min;
    uint16_t saving_end_min = (uint16_t)g_node_cfg.saving_end_hour * 60u + g_node_cfg.saving_end_min;

    if (sched_last_day != d.Date) {
        sched_last_day = d.Date;
        on_done_today  = false;
        off_done_today = false;
    }

    control_mode = current_control_mode();

    switch (control_mode) {
    case 0:
        managed = 1;
        on_min = apply_time_correction_min(g_sunset_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
        off_min = apply_time_correction_min(g_sunrise_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
        want_on = time_window_contains(now_min, on_min, off_min);
        break;

    case 1:
        managed = 1;
        on_min = apply_time_correction_min(g_dusk_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
        off_min = apply_time_correction_min(g_dawn_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
        want_on = time_window_contains(now_min, on_min, off_min);
        break;

    case 2:
        managed = 1;
        on_min  = (uint16_t)g_node_cfg.light_on_hour  * 60u + g_node_cfg.light_on_min;
        off_min = (uint16_t)g_node_cfg.light_off_hour * 60u + g_node_cfg.light_off_min;
        on_min = apply_time_correction_min(on_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
        off_min = apply_time_correction_min(off_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
        if (on_min != off_min) {
            want_on = time_window_contains(now_min, on_min, off_min);
        }
        break;

    case 3:
        managed = 1;
        want_on = 0u;
        break;

    default:
        break;
    }

    if (!managed) return;

    if (g_node_cfg.saving_mode &&
        time_window_contains(now_min, saving_start_min, saving_end_min)) {
        want_on = 0u;
    }

    {
        uint8_t current_light = light_is_on_logical();
        if (control_mode == 2u &&
            (sched_gap >= 1000u ||
             last_log_want_on != want_on ||
             last_log_light_on != current_light ||
             last_log_manual_active != g_manual_override_active ||
             (uint32_t)(now_tick - last_sched_log_tick) >= 5000u)) {
            char msg[256];
            int n = snprintf(msg, sizeof(msg),
                             "[SCHEDDBG] tick=%lu gap=%lu rtc=%02u:%02u:%02u mode=%u manual=%u man_on=%u no_to=%u until=%lu now_min=%u on=%u off=%u saving=%u want=%u light=%u\r\n",
                             (unsigned long)now_tick,
                             (unsigned long)sched_gap,
                             (unsigned)t.Hours,
                             (unsigned)t.Minutes,
                             (unsigned)t.Seconds,
                             (unsigned)control_mode,
                             (unsigned)g_manual_override_active,
                             (unsigned)g_manual_override_light_on,
                             (unsigned)g_manual_override_no_timeout,
                             (unsigned long)g_manual_override_until,
                             (unsigned)now_min,
                             (unsigned)on_min,
                             (unsigned)off_min,
                             (unsigned)g_node_cfg.saving_mode,
                             (unsigned)want_on,
                             (unsigned)current_light);
            if (n > 0) {
                HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 50);
            }
            last_sched_log_tick = now_tick;
            last_log_want_on = want_on;
            last_log_light_on = current_light;
            last_log_manual_active = g_manual_override_active;
        }
    }

    light_event_set_reason((g_node_cfg.saving_mode && time_window_contains(now_min, saving_start_min, saving_end_min)) ? LIGHT_EVENT_REASON_SAVING : LIGHT_EVENT_REASON_SCHEDULE);

    if (want_on) {
        light_on();
    } else {
        light_off();
    }
    light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
}

static int light_norm_min(int t)
{
    while (t < 0)      t += 1440;
    while (t >= 1440)  t -= 1440;
    return t;
}
