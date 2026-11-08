#ifndef SIM_PICO_TIME_H
#define SIM_PICO_TIME_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef int32_t alarm_id_t;
typedef int64_t (*alarm_callback_t)(alarm_id_t id, void *user_data);
alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t callback, void *data, bool fire_if_past);
bool cancel_alarm(alarm_id_t id);
#endif
