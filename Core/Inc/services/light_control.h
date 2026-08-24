#ifndef LIGHT_CONTROL_H
#define LIGHT_CONTROL_H

#include <stdint.h>
#include "stm32h5xx_hal.h"

#define LIGHT_EVENT_REASON_UNKNOWN        0u
#define LIGHT_EVENT_REASON_CMD            1u
#define LIGHT_EVENT_REASON_SET_FORCED     2u
#define LIGHT_EVENT_REASON_FORCED_EXPIRE  3u
#define LIGHT_EVENT_REASON_SCHEDULE       4u
#define LIGHT_EVENT_REASON_SAVING         5u
#define LIGHT_EVENT_REASON_RTC_UNSYNCED   6u
#define LIGHT_EVENT_REASON_TEST           7u
#define LIGHT_EVENT_REASON_NODE_CFG       8u

#define LIGHT_Pin        GPIO_PIN_12
#define LIGHT_GPIO_Port  GPIOA
#define LIGHT_ACTIVE_LOW 0

void light_on(void);
void light_off(void);
uint8_t light_is_on_logical(void);

uint8_t current_control_mode(void);
uint8_t light_control_get_effective_schedule(uint8_t *mode_out,
                                             uint16_t *on_time_min_out,
                                             uint16_t *off_time_min_out);
uint8_t light_control_manual_active(void);
uint16_t manual_override_duration_min(void);
void start_manual_override(uint8_t want_on);
void start_manual_override_for(uint8_t want_on, uint16_t duration_min);
void start_forced_time_control(uint16_t forced_time_min);
void light_control_clear_manual_override(void);

uint16_t apply_time_correction_min(uint16_t base_min, uint8_t corr_mode, uint16_t corr_delta_min);
uint8_t time_window_contains(uint16_t now_min, uint16_t start_min, uint16_t end_min);
void scheduler_poll(void);

void light_event_set_reason(uint8_t reason);
void light_state_event_note_if_changed(uint8_t before_on, uint8_t after_on);

#endif /* LIGHT_CONTROL_H */
