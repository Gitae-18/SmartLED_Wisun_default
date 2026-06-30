#include "wisun_app.h"

#include "light_control.h"
#include "rtc_user.h"
#include "wisun_router.h"

void boot_poll(void);
void wisun_process_rx_mainloop(void);
void update_sun_times(void);
void resp_slot_task_poll(void);
void light_state_event_poll(void);

void wisun_app_poll(uint32_t now)
{
    boot_poll();
    wisun_process_rx_mainloop();
    rtc_sync_request_poll(now);

    if (g_schedule_alarm_due) {
        g_schedule_alarm_due = 0u;
        rtc_update();
        update_sun_times();
        scheduler_poll();
        rtc_schedule_next_minute_alarm();
    }

    resp_slot_task_poll();
    light_state_event_poll();
    hop_tx_task_poll();
}
