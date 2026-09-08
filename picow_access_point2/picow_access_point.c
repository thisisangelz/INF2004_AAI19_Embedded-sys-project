#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#define WIFI_SSID "I AM PICO W 2"
#define WIFI_PASSWORD "pico12345"

int main()
{
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Failed to initialise Wi-Fi\n");
        return 1;
    }

    printf("Wi-Fi initialised\n");

    cyw43_arch_enable_ap_mode(
        WIFI_SSID,
        WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK
    );

    printf("Access Point started!\n");
    printf("SSID: %s\n", WIFI_SSID);
    printf("Password: %s\n", WIFI_PASSWORD);

    while (true) {
        sleep_ms(1000);
    }

    return 0;
}