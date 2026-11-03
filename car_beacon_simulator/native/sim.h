#ifndef CAR_SIM_H
#define CAR_SIM_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint64_t us, next_frame, auto_start;
    double x, y, heading, bx, by, speed, wheelbase, path, closest;
    double wifi_ref, exponent, ble_radius, stop_at, horizon;
    double gyro_angle, gyro_last_angle;
    uint64_t gyro_last_us;
    unsigned scans, wifi_reports, ads, ble_attempts, rx_bytes, tx_bytes, checkpoints;
    unsigned received_length;
    uint16_t pwm[32];
    int wifi_observed, ble_observed;
    bool wifi_active, started, finished, success, peer_verified, peer_complete;
    bool corrupt_reply;
    const char *peer_state;
} Sim;
extern Sim sim;
double sim_distance(void);
double sim_wifi(void);
double sim_ble(void);
void sim_log(const char *source, const char *message);
void sim_frame(void);
void sim_ble_tick(void);
void sim_ble_stats(void);
int sim_ble_threshold(void);
void sim_json_string(const char *s);
#endif
