#ifndef CAR_SIM_H
#define CAR_SIM_H
#include <stdint.h>
#include <stdbool.h>
#define SIM_MAX_BEACONS 3
typedef struct {
    int id;
    double x, y;
    char mat[8];
    unsigned rx_bytes, tx_bytes, received_length;
    unsigned connections, verified_events, complete_events, payload_events;
    int wifi_observed, ble_observed;
    bool verified, complete;
    const char *peer_state;
    uint16_t pwm[32];
} SimBeacon;
typedef struct { int id; double x, y; char mat[8]; } SimMat;
typedef struct {
    uint64_t us, next_frame, auto_start;
    double x, y, heading, bx, by, speed, wheelbase, path, closest;
    double wifi_ref, exponent, ble_radius, stop_at, horizon;
    double mat_radius, sensor_offset, radio_range, box_radius;
    double gyro_angle, gyro_last_angle;
    uint64_t gyro_last_us;
    unsigned scans, wifi_reports, ads, ble_attempts, rx_bytes, tx_bytes, checkpoints;
    unsigned received_length, completed_mask;
    int active_target, connected_id;
    uint16_t pwm[32];
    int wifi_observed, ble_observed;
    bool wifi_active, started, finished, success, peer_verified, peer_complete;
    bool corrupt_reply, colour_enabled, collision;
    const char *peer_state;
    SimBeacon beacons[3];
    unsigned beacon_count;
    SimMat mats[3];
    unsigned mat_count;
} Sim;
extern Sim sim;
// Hardware ownership (0=robot). Never derived from the robot's target.
extern int sim_device;
SimBeacon *sim_beacon(int id);
double sim_distance_to(int id);
double sim_distance(void);
double sim_wifi_at(int id);
double sim_ble_at(int id);
double sim_wifi(void);
double sim_ble(void);
bool sim_radio_visible(int id);
void sim_radio_address(int id, uint8_t address[6]);
int sim_radio_identity(const uint8_t address[6]);
void sim_colour_raw(uint16_t out[4], double *sx, double *sy, int *mat_id, int *band);
void sim_physics(uint32_t us);
void sim_log(const char *source, const char *message);
int sim_console_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
void sim_report_observe(const char *message);
void sim_frame(void);
void sim_ble_tick(void);
void sim_ble_stats(void);
int sim_ble_threshold(void);
int sim_active_target(void);
void sim_json_string(const char *s);
bool sim_scenario_json(const char *json);
void sim_scenario_defaults(void);
void sim_alarms_tick(void);
uint64_t sim_next_alarm(void);
#endif
