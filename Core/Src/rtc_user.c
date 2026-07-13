#include "rtc_user.h"

#include "main.h"
#include "debug_log.h"
#include "light_control.h"

#include <stdbool.h>
#include <stdio.h>

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart6;

extern bool node_is_provisioned(void);
extern uint8_t send_transport_direct(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len);
extern void update_sun_times(void);
extern uint16_t g_sunrise_min;
extern uint16_t g_sunset_min;
extern uint16_t g_dawn_min;
extern uint16_t g_dusk_min;

uint8_t g_rtc_hour;
uint8_t g_rtc_min;
uint8_t g_rtc_sec;
uint16_t g_rtc_day;
uint8_t g_rtc_month;
uint16_t g_rtc_year;
uint8_t g_rtc_synced = 0u;
volatile uint8_t g_schedule_alarm_due = 1u;
volatile uint32_t g_rtc_alarm_count = 0u;
volatile uint32_t g_rtc_alarm_last_tick = 0u;

#define RTC_SCHED_FALLBACK_CHECK_MS   1000u
#define RTC_SCHED_ALARM_STALE_MS     75000u

static uint32_t g_rtc_sched_fallback_check_tick = 0u;
static uint32_t g_rtc_sched_fallback_last_key = 0xFFFFFFFFu;

static uint8_t rtc_weekday_from_ymd(uint16_t year, uint8_t month, uint8_t day);

typedef struct {
    uint8_t valid;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint16_t sunrise_min;
    uint16_t sunset_min;
    uint16_t dawn_min;
    uint16_t dusk_min;
} rtc_gateway_sun_times_t;

static rtc_gateway_sun_times_t g_gateway_sun_times = {0};

static uint16_t rtc_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint8_t rtc_sun_min_valid(uint16_t value)
{
    return (value < 1440u) ? 1u : 0u;
}

static uint8_t rtc_store_gateway_sun_times(uint16_t year, uint8_t month, uint8_t day, const uint8_t *data, uint16_t len)
{
    uint16_t sunrise;
    uint16_t sunset;
    uint16_t dawn;
    uint16_t dusk;

    if (data == NULL || len < 15u) {
        g_gateway_sun_times.valid = 0u;
        return 0u;
    }

    sunrise = rtc_u16_be(&data[7]);
    sunset  = rtc_u16_be(&data[9]);
    dawn    = rtc_u16_be(&data[11]);
    dusk    = rtc_u16_be(&data[13]);

    if (!rtc_sun_min_valid(sunrise) ||
        !rtc_sun_min_valid(sunset) ||
        !rtc_sun_min_valid(dawn) ||
        !rtc_sun_min_valid(dusk)) {
        g_gateway_sun_times.valid = 0u;
        uart6_log("[RTC_SYNC_SUN_INVALID] sunrise=%u sunset=%u civilm=%u civile=%u\r\n",
                  (unsigned)sunrise,
                  (unsigned)sunset,
                  (unsigned)dawn,
                  (unsigned)dusk);
        return 0u;
    }

    g_gateway_sun_times.valid = 1u;
    g_gateway_sun_times.year = year;
    g_gateway_sun_times.month = month;
    g_gateway_sun_times.day = day;
    g_gateway_sun_times.sunrise_min = sunrise;
    g_gateway_sun_times.sunset_min = sunset;
    g_gateway_sun_times.dawn_min = dawn;
    g_gateway_sun_times.dusk_min = dusk;

    uart6_log("[RTC_SYNC_SUN_RX] %04u-%02u-%02u sunrise=%u sunset=%u civilm=%u civile=%u\r\n",
              (unsigned)year,
              (unsigned)month,
              (unsigned)day,
              (unsigned)sunrise,
              (unsigned)sunset,
              (unsigned)dawn,
              (unsigned)dusk);
    return 1u;
}

uint8_t rtc_apply_gateway_sun_times_if_current(void)
{
    if (!g_gateway_sun_times.valid) {
        return 0u;
    }

    if (g_gateway_sun_times.year != g_rtc_year ||
        g_gateway_sun_times.month != g_rtc_month ||
        g_gateway_sun_times.day != g_rtc_day) {
        return 0u;
    }

    g_sunrise_min = g_gateway_sun_times.sunrise_min;
    g_sunset_min = g_gateway_sun_times.sunset_min;
    g_dawn_min = g_gateway_sun_times.dawn_min;
    g_dusk_min = g_gateway_sun_times.dusk_min;
    return 1u;
}

void rtc_sync_request_poll(uint32_t now)
{
#if RTC_SYNC_REQ_ENABLE
    static uint32_t last_req_tick = 0u;
    static uint8_t req_count = 0u;
    static uint16_t req_msg_id = 0u;

    if (g_rtc_synced) {
        return;
    }

    if (!node_is_provisioned()) {
        return;
    }

    if (req_count >= RTC_SYNC_REQ_MAX_TRIES) {
        return;
    }

    if (last_req_tick == 0u) {
        if (now < RTC_SYNC_REQ_FIRST_MS) {
            return;
        }
    } else if ((uint32_t)(now - last_req_tick) < RTC_SYNC_REQ_INTERVAL_MS) {
        return;
    }

    req_msg_id++;
    if (req_msg_id == 0u) {
        req_msg_id = 1u;
    }

    if (send_transport_direct(0x0000u, 0u, SET_RTC_KST, 0u, req_msg_id, NULL, 0u)) {
        last_req_tick = now;
        req_count++;
        uart6_log("[RTC_SYNC_REQ_TX] t=%lu msg_id=%u try=%u/%u\r\n",
                  (unsigned long)now,
                  (unsigned)req_msg_id,
                  (unsigned)req_count,
                  (unsigned)RTC_SYNC_REQ_MAX_TRIES);
    }
#else
    (void)now;
#endif
}

uint8_t handle_cmd_set_rtc_kst(const uint8_t *data, uint16_t len)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    RTC_TimeTypeDef curTime = {0};
    RTC_DateTypeDef curDate = {0};
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t max_day = 31u;
    uint8_t leap = 0u;

    if (data == NULL || len < 7u) {
        debug6("[RTC_SYNC] invalid payload\r\n");
        return 1u;
    }

    /*
     * ACK-like payload 방어.
     *
     * 정상 RTC 설정 payload는:
     *   year_hi year_lo month day hour minute second ...
     *
     * 그런데 반복 로그의 payload는:
     *   2B + UID12 + msg_id32 + ok + err
     *
     * 형태라서 data[0] == 0x2B로 시작한다.
     * 이걸 날짜로 파싱하면 year=0x2B42=11074 같은 값이 되어 loop가 발생한다.
     */
    if (len >= 19u && data[0] == 0x2Bu) {
        uart6_log("[RTC_SYNC_DROP] reason=ack_like_payload len=%u first=0x%02X\r\n",
                  (unsigned)len,
                  (unsigned)data[0]);
        return 8u;
    }

    year = ((uint16_t)data[0] << 8) | data[1];
    month = data[2];
    day = data[3];
    hour = data[4];
    minute = data[5];
    second = data[6];

    {
        char parsed_dbg[128];
        int parsed_dbg_len = snprintf(
            parsed_dbg, sizeof(parsed_dbg),
            "[RTC_SYNC_PARSE] %04u-%02u-%02u %02u:%02u:%02u\r\n",
            (unsigned)year,
            (unsigned)month,
            (unsigned)day,
            (unsigned)hour,
            (unsigned)minute,
            (unsigned)second
        );
        if (parsed_dbg_len > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t*)parsed_dbg, (uint16_t)parsed_dbg_len, 100);
        }
    }

    if (year < 2000u || year > 2099u) {
        uart6_log("[RTC_SYNC_DROP] reason=bad_year year=%u len=%u\r\n",
                  (unsigned)year,
                  (unsigned)len);
        return 2u;
    }

    if (month < 1u || month > 12u) {
        uart6_log("[RTC_SYNC_DROP] reason=bad_month %04u-%02u-%02u %02u:%02u:%02u len=%u\r\n",
                  (unsigned)year,
                  (unsigned)month,
                  (unsigned)day,
                  (unsigned)hour,
                  (unsigned)minute,
                  (unsigned)second,
                  (unsigned)len);
        return 3u;
    }

    if (hour > 23u || minute > 59u || second > 59u) {
        uart6_log("[RTC_SYNC_DROP] reason=bad_time %04u-%02u-%02u %02u:%02u:%02u len=%u\r\n",
                  (unsigned)year,
                  (unsigned)month,
                  (unsigned)day,
                  (unsigned)hour,
                  (unsigned)minute,
                  (unsigned)second,
                  (unsigned)len);
        return 4u;
    }

    leap = ((year % 4u) == 0u) ? 1u : 0u;
    switch (month) {
    case 2:
        max_day = (uint8_t)(leap ? 29u : 28u);
        break;
    case 4:
    case 6:
    case 9:
    case 11:
        max_day = 30u;
        break;
    default:
        max_day = 31u;
        break;
    }

    if (day < 1u || day > max_day) {
        uart6_log("[RTC_SYNC_DROP] reason=bad_day %04u-%02u-%02u max_day=%u len=%u\r\n",
                  (unsigned)year,
                  (unsigned)month,
                  (unsigned)day,
                  (unsigned)max_day,
                  (unsigned)len);
        return 5u;
    }

    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    sDate.Year = (uint8_t)(year - 2000u);
    sDate.Month = month;
    sDate.Date = day;
    sDate.WeekDay = rtc_weekday_from_ymd(year, month, day);

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        debug6("[RTC_SYNC] HAL_RTC_SetTime failed\r\n");
        return 6u;
    }

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        debug6("[RTC_SYNC] HAL_RTC_SetDate failed\r\n");
        return 7u;
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2u);

    if (HAL_RTC_GetTime(&hrtc, &curTime, RTC_FORMAT_BIN) == HAL_OK &&
        HAL_RTC_GetDate(&hrtc, &curDate, RTC_FORMAT_BIN) == HAL_OK) {
        char applied_dbg[128];
        int applied_dbg_len = snprintf(
            applied_dbg, sizeof(applied_dbg),
            "[RTC_SYNC_APPLIED] %04u-%02u-%02u %02u:%02u:%02u wd=%u\r\n",
            (unsigned)(2000u + curDate.Year),
            (unsigned)curDate.Month,
            (unsigned)curDate.Date,
            (unsigned)curTime.Hours,
            (unsigned)curTime.Minutes,
            (unsigned)curTime.Seconds,
            (unsigned)curDate.WeekDay
        );
        if (applied_dbg_len > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t*)applied_dbg, (uint16_t)applied_dbg_len, 100);
        }
    } else {
        debug6("[RTC_SYNC] readback failed\r\n");
    }

    g_rtc_synced = 1u;
    rtc_update();

    (void)rtc_store_gateway_sun_times(year, month, day, data, len);
    update_sun_times();

    rtc_schedule_next_minute_alarm();
    scheduler_poll();

    debug6("[RTC_SYNC] scheduler resumed\r\n");
    return 0u;
}

void rtc_update(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    g_rtc_hour  = sTime.Hours;
    g_rtc_min   = sTime.Minutes;
    g_rtc_sec   = sTime.Seconds;

    g_rtc_year  = 2000 + sDate.Year;
    g_rtc_month = sDate.Month;
    g_rtc_day   = sDate.Date;
}

void rtc_schedule_next_minute_alarm(void)
{
    RTC_TimeTypeDef now_time = {0};
    RTC_DateTypeDef now_date = {0};
    RTC_AlarmTypeDef alarm = {0};
    uint8_t next_hour;
    uint8_t next_min;

    if (HAL_RTC_GetTime(&hrtc, &now_time, RTC_FORMAT_BIN) != HAL_OK) {
        return;
    }
    if (HAL_RTC_GetDate(&hrtc, &now_date, RTC_FORMAT_BIN) != HAL_OK) {
        return;
    }

    next_hour = now_time.Hours;
    next_min = (uint8_t)(now_time.Minutes + 1u);
    if (next_min >= 60u) {
        next_min = 0u;
        next_hour = (uint8_t)((next_hour + 1u) % 24u);
    }

    (void)HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

    alarm.AlarmTime.Hours = next_hour;
    alarm.AlarmTime.Minutes = next_min;
    alarm.AlarmTime.Seconds = 0u;
    alarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    alarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    alarm.AlarmDateWeekDay = 1u;
    alarm.Alarm = RTC_ALARM_A;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN) != HAL_OK) {
        uart6_log("[RTC_ALARM_SET_FAIL] now=%02u:%02u:%02u next=%02u:%02u:00\r\n",
                  (unsigned)now_time.Hours,
                  (unsigned)now_time.Minutes,
                  (unsigned)now_time.Seconds,
                  (unsigned)next_hour,
                  (unsigned)next_min);
    }
}


void rtc_scheduler_fallback_poll(uint32_t now)
{
    uint32_t alarm_age;
    uint32_t minute_key;

    if ((uint32_t)(now - g_rtc_sched_fallback_check_tick) < RTC_SCHED_FALLBACK_CHECK_MS) {
        return;
    }
    g_rtc_sched_fallback_check_tick = now;

    /* 정상 RTC Alarm이 최근에 들어왔다면 기존 Alarm 경로에 맡긴다. */
    if (g_rtc_alarm_last_tick != 0u) {
        alarm_age = (uint32_t)(now - g_rtc_alarm_last_tick);
        if (alarm_age < RTC_SCHED_ALARM_STALE_MS) {
            return;
        }
    } else if (now < RTC_SCHED_ALARM_STALE_MS) {
        /* 부팅 직후에는 최초 Alarm이 들어올 시간을 준다. */
        return;
    }

    rtc_update();

    minute_key = ((uint32_t)(g_rtc_year & 0x0FFFu) << 20) |
                 ((uint32_t)(g_rtc_month & 0x0Fu) << 16) |
                 ((uint32_t)(g_rtc_day & 0x1Fu) << 11) |
                 ((uint32_t)(g_rtc_hour & 0x1Fu) << 6) |
                 ((uint32_t)(g_rtc_min & 0x3Fu));

    if (minute_key == g_rtc_sched_fallback_last_key) {
        return;
    }
    g_rtc_sched_fallback_last_key = minute_key;

    update_sun_times();
    scheduler_poll();

    uart6_log("[RTC_SCHED_FALLBACK] alarm_count=%lu alarm_age=%lu rtc=%04u-%02u-%02u %02u:%02u:%02u\r\n",
              (unsigned long)g_rtc_alarm_count,
              (unsigned long)((g_rtc_alarm_last_tick == 0u) ? now : (now - g_rtc_alarm_last_tick)),
              (unsigned)g_rtc_year,
              (unsigned)g_rtc_month,
              (unsigned)g_rtc_day,
              (unsigned)g_rtc_hour,
              (unsigned)g_rtc_min,
              (unsigned)g_rtc_sec);

    /* Alarm 체인이 끊겼다면 다음 분 Alarm도 다시 걸어 복구를 시도한다. */
    rtc_schedule_next_minute_alarm();
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc_cb)
{
    if (hrtc_cb != NULL && hrtc_cb->Instance == RTC) {
        g_rtc_alarm_count++;
        g_rtc_alarm_last_tick = HAL_GetTick();
        g_schedule_alarm_due = 1u;
    }
}

static uint8_t rtc_weekday_from_ymd(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    uint16_t y = year;

    if (month < 1u || month > 12u || day < 1u || day > 31u) {
        return RTC_WEEKDAY_MONDAY;
    }

    if (month < 3u) {
        y--;
    }

    {
        uint8_t dow = (uint8_t)((y + y / 4u - y / 100u + y / 400u + t[month - 1u] + day) % 7u);
        return (dow == 0u) ? RTC_WEEKDAY_SUNDAY : dow;
    }
}
