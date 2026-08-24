#include "protocol/wisun/wisun_app.h"

#include "services/rtc_user.h"
#include "protocol/wisun/wisun_router.h"

void boot_poll(void);
void wisun_process_rx_mainloop(void);
void resp_slot_task_poll(void);
void light_state_event_poll(void);

void wisun_app_poll(uint32_t now)
{
    boot_poll();
    wisun_process_rx_mainloop();
    rtc_sync_request_poll(now);

    resp_slot_task_poll();
    light_state_event_poll();
    hop_tx_task_poll();
}
