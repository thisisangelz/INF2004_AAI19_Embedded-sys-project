#include "pico/stdlib.h"
typedef struct { int unused; } cyw43_wifi_scan_options_t;
typedef struct { int16_t rssi; uint8_t ssid_len; uint8_t ssid[32]; uint8_t bssid[6]; } cyw43_ev_scan_result_t;
extern int cyw43_state;
int cyw43_arch_init(void);
void cyw43_arch_deinit(void);
void cyw43_arch_enable_sta_mode(void);
int cyw43_wifi_scan(int *state, cyw43_wifi_scan_options_t *options, void *context,
                   int (*callback)(void *, const cyw43_ev_scan_result_t *));
bool cyw43_wifi_scan_active(int *state);
#define CYW43_WL_GPIO_LED_PIN 0
#define CYW43_AUTH_WPA2_AES_PSK 0
#define CYW43_NO_POWERSAVE_MODE 0
void cyw43_arch_enable_ap_mode(const char *ssid,const char *password,uint32_t auth);
void cyw43_arch_gpio_put(uint pin,bool enabled);
int cyw43_wifi_pm(int *state,uint32_t mode);
uint32_t cyw43_pm_value(uint8_t mode,uint16_t sleep,uint8_t li,uint8_t dtim,uint8_t assoc);
