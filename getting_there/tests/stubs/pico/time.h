#ifndef HOST_PICO_TIME_H
#define HOST_PICO_TIME_H
#include <stdbool.h>
#include <stdint.h>

typedef int32_t alarm_id_t;
// SDK 2.3.0: negative = relative to previous deadline; positive = relative
// to callback completion; zero = finished.
typedef int64_t (*alarm_callback_t)(alarm_id_t id, void *user_data);
alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t callback,
                          void *user_data, bool fire_if_past);
bool cancel_alarm(alarm_id_t id);
#endif
