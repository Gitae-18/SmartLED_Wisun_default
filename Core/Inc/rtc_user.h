#ifndef RTC_USER_H
#define RTC_USER_H

#include <stdint.h>

#define SET_RTC_KST 0x2B

#ifndef RTC_SYNC_REQ_ENABLE
#define RTC_SYNC_REQ_ENABLE      1
#endif
#ifndef RTC_SYNC_REQ_FIRST_MS
#define RTC_SYNC_REQ_FIRST_MS    3000u
#endif
#ifndef RTC_SYNC_REQ_INTERVAL_MS
#define RTC_SYNC_REQ_INTERVAL_MS 10000u
#endif
#ifndef RTC_SYNC_REQ_MAX_TRIES
#define RTC_SYNC_REQ_MAX_TRIES   6u
#endif

extern uint8_t g_rtc_hour;
extern uint8_t g_rtc_min;
extern uint8_t g_rtc_sec;
extern uint16_t g_rtc_day;
extern uint8_t g_rtc_month;
extern uint16_t g_rtc_year;
extern uint8_t g_rtc_synced;

void rtc_update(void);
void rtc_schedule_next_minute_alarm(void);
void rtc_sync_request_poll(uint32_t now);
void rtc_scheduler_poll(uint32_t now);
uint8_t rtc_apply_gateway_sun_times_if_current(void);
uint8_t handle_cmd_set_rtc_kst(const uint8_t *data, uint16_t len);

#endif /* RTC_USER_H */
