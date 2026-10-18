#include "pico/stdlib.h"
typedef struct { int unused; } cyw43_wifi_scan_options_t;
typedef struct { int16_t rssi; uint8_t ssid_len; uint8_t ssid[32]; uint8_t bssid[6]; } cyw43_ev_scan_result_t;
extern int cyw43_state;
int cyw43_arch_init(void);
void cyw43_arch_enable_sta_mode(void);
int cyw43_wifi_scan(int *state, cyw43_wifi_scan_options_t *options, void *context,
                   int (*callback)(void *, const cyw43_ev_scan_result_t *));
bool cyw43_wifi_scan_active(int *state);
