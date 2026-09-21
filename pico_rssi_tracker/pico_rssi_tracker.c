#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "transfer_protocol.h"

#define AP_COUNT 3

static const char *const AP_SSIDS[AP_COUNT] = {
    "I AM PICO W", "I AM PICO W 2", "I AM PICO W 3",
};
static const uint AP_LED_PINS[AP_COUNT] = {2, 3, 4};

#ifndef START_BUTTON_PIN
#define START_BUTTON_PIN 20
#endif
#ifndef BUZZER_PIN
#define BUZZER_PIN 18
#endif

// Adjustable first estimate only: calibrate on the hardware for 15 cm.
#ifndef BLE_CLOSE_RSSI_DBM
#define BLE_CLOSE_RSSI_DBM (-35)
#endif
#ifndef BLE_RSSI_SAMPLE_COUNT
#define BLE_RSSI_SAMPLE_COUNT 8
#endif
#ifndef BLE_CLOSE_REQUIRED_AVERAGES
#define BLE_CLOSE_REQUIRED_AVERAGES 3
#endif

#define PRINT_INTERVAL_MS 500
#define RSSI_TIMEOUT_MS 5000
#define SWITCH_HYSTERESIS_DB 3
#define BUTTON_DEBOUNCE_MS 40
#define BUZZER_TONE_HZ 2000
#define BEEP_ON_MS 80
#define BEEP_INTERVAL_FAR_MS 1000
#define BEEP_INTERVAL_NEAR_MS 100

typedef enum {
    BLE_IDLE,
    BLE_SCANNING,
    BLE_CONNECTING,
    BLE_DISCOVER_SERVICE,
    BLE_DISCOVER_CHARACTERISTIC,
    BLE_ENABLE_NOTIFICATIONS,
    BLE_WRITE_HELLO,
    BLE_WAIT_HELLO_ACK,
    BLE_WRITE_FILE_START,
    BLE_WAIT_FILE_READY,
    BLE_WRITE_FILE_DATA,
    BLE_WRITE_FILE_END,
    BLE_WAIT_FILE_RECEIVED,
    BLE_WRITE_REPLY_REQUEST,
    BLE_WAIT_REPLY_START,
    BLE_WRITE_REPLY_NEXT,
    BLE_WAIT_REPLY_PIECE,
    BLE_WRITE_COMPLETE,
    BLE_WAIT_COMPLETE_ACK,
    BLE_DISCONNECTING,
    BLE_FINISHED,
} ble_state_t;

static volatile int ap_rssi[AP_COUNT] = {-127, -127, -127};
static volatile uint32_t ap_last_seen_ms[AP_COUNT] = {0, 0, 0};

static uint buzzer_slice;
static uint16_t buzzer_wrap;
static bool buzzer_enabled;
static bool buzzer_is_on;
static uint32_t buzzer_interval_ms = BEEP_INTERVAL_FAR_MS;
static uint32_t buzzer_next_event_ms;

static int selected_ap = -1;
static volatile bool transfer_active;
static volatile bool sequence_started;
static volatile bool wifi_scans_paused;
static volatile int target_beacon;
static volatile bool beacon_complete[AP_COUNT];

static btstack_packet_callback_registration_t hci_event_registration;
static btstack_context_callback_registration_t start_scan_registration;
static btstack_context_callback_registration_t restart_registration;
static btstack_context_callback_registration_t connect_registration;
static volatile bool bluetooth_ready;
static volatile bool ble_connect_pending;
static volatile ble_state_t ble_state = BLE_IDLE;
static bd_addr_t beacon_address;
static bd_addr_type_t beacon_address_type;
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static gatt_client_service_t transfer_service;
static gatt_client_characteristic_t transfer_characteristic;
static gatt_client_notification_t notification_listener;
static bool notification_listener_registered;
static bool service_found;
static bool characteristic_found;
static bool current_transfer_succeeded;

static int ble_rssi_samples[BLE_RSSI_SAMPLE_COUNT];
static uint8_t ble_rssi_sample_count;
static uint8_t ble_rssi_sample_index;
static uint8_t qualifying_average_count;

static uint8_t tx_packet[TRANSFER_PACKET_SIZE];
static uint8_t robot_file[96];
static uint16_t robot_file_length;
static uint16_t robot_file_offset;
static uint8_t robot_file_sequence;
static uint32_t robot_file_crc;

static uint8_t reply_file[TRANSFER_MAX_FILE_SIZE];
static uint16_t reply_file_length;
static uint16_t reply_file_offset;
static uint8_t reply_file_sequence;
static uint32_t reply_file_crc;

static void gatt_client_handler(uint8_t packet_type, uint16_t channel,
                                uint8_t *packet, uint16_t size);

static bool ssid_matches(const cyw43_ev_scan_result_t *result,
                         const char *target)
{
    size_t length = strlen(target);
    return result->ssid_len == length &&
           memcmp(result->ssid, target, length) == 0;
}

static int wifi_scan_result(void *env, const cyw43_ev_scan_result_t *result)
{
    (void)env;
    if (result == NULL) return 0;
    uint32_t now_ms = (uint32_t)(time_us_64() / 1000);
    for (int i = 0; i < AP_COUNT; ++i) {
        if (ssid_matches(result, AP_SSIDS[i])) {
            ap_rssi[i] = result->rssi;
            ap_last_seen_ms[i] = now_ms;
            break;
        }
    }
    return 0;
}

static void leds_init(void)
{
    for (int i = 0; i < AP_COUNT; ++i) {
        gpio_init(AP_LED_PINS[i]);
        gpio_set_dir(AP_LED_PINS[i], GPIO_OUT);
        gpio_put(AP_LED_PINS[i], 0);
    }
}

static void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 16.0f);
    pwm_init(buzzer_slice, &config, true);
    uint32_t wrap =
        (uint32_t)(clock_get_hz(clk_sys) / (16.0f * BUZZER_TONE_HZ)) - 1;
    if (wrap > 65535) wrap = 65535;
    if (wrap < 2) wrap = 2;
    buzzer_wrap = (uint16_t)wrap;
    pwm_set_wrap(buzzer_slice, buzzer_wrap);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

static void buzzer_set_tone(bool enabled)
{
    pwm_set_gpio_level(BUZZER_PIN, enabled ? buzzer_wrap / 2 : 0);
}

static uint32_t rssi_to_beep_interval(int rssi)
{
    if (rssi < -90) rssi = -90;
    if (rssi > -30) rssi = -30;
    return BEEP_INTERVAL_FAR_MS -
           ((uint32_t)(rssi + 90) *
            (BEEP_INTERVAL_FAR_MS - BEEP_INTERVAL_NEAR_MS)) / 60;
}

static void buzzer_configure(bool enabled, uint32_t interval_ms,
                             uint32_t now_ms)
{
    if (!enabled) {
        buzzer_enabled = false;
        buzzer_is_on = false;
        buzzer_set_tone(false);
        return;
    }
    if (interval_ms < BEEP_ON_MS) interval_ms = BEEP_ON_MS;
    buzzer_interval_ms = interval_ms;
    if (!buzzer_enabled) {
        buzzer_enabled = true;
        buzzer_is_on = true;
        buzzer_set_tone(true);
        buzzer_next_event_ms = now_ms + BEEP_ON_MS;
    }
}

static void buzzer_service(uint32_t now_ms)
{
    if (transfer_active) {
        buzzer_enabled = false;
        buzzer_is_on = true;
        buzzer_set_tone(true);
        return;
    }
    if (!buzzer_enabled || (int32_t)(now_ms - buzzer_next_event_ms) < 0) return;
    if (buzzer_is_on) {
        uint32_t off_ms = buzzer_interval_ms > BEEP_ON_MS
                              ? buzzer_interval_ms - BEEP_ON_MS : 1;
        buzzer_is_on = false;
        buzzer_set_tone(false);
        buzzer_next_event_ms = now_ms + off_ms;
    } else {
        buzzer_is_on = true;
        buzzer_set_tone(true);
        buzzer_next_event_ms = now_ms + BEEP_ON_MS;
    }
}

static bool get_ap_valid(int index, uint32_t now_ms)
{
    return ap_last_seen_ms[index] != 0 &&
           (uint32_t)(now_ms - ap_last_seen_ms[index]) < RSSI_TIMEOUT_MS;
}

static bool all_access_points_visible(uint32_t now_ms)
{
    for (int i = 0; i < AP_COUNT; ++i) {
        if (!get_ap_valid(i, now_ms)) return false;
    }
    return true;
}

static void update_output(uint32_t now_ms)
{
    if (ble_state == BLE_FINISHED) {
        selected_ap = -1;
        for (int i = 0; i < AP_COUNT; ++i) gpio_put(AP_LED_PINS[i], 1);
        buzzer_configure(false, 0, now_ms);
        return;
    }

    if (sequence_started && target_beacon < AP_COUNT) {
        selected_ap = target_beacon;
    } else {
        int strongest = -1;
        for (int i = 0; i < AP_COUNT; ++i) {
            if (get_ap_valid(i, now_ms) &&
                (strongest < 0 || ap_rssi[i] > ap_rssi[strongest])) {
                strongest = i;
            }
        }
        if (strongest < 0) selected_ap = -1;
        else if (selected_ap < 0 || !get_ap_valid(selected_ap, now_ms) ||
                 (strongest != selected_ap && ap_rssi[strongest] >
                  ap_rssi[selected_ap] + SWITCH_HYSTERESIS_DB)) {
            selected_ap = strongest;
        }
    }

    for (int i = 0; i < AP_COUNT; ++i) {
        gpio_put(AP_LED_PINS[i], selected_ap == i || beacon_complete[i]);
    }
    if (transfer_active) return;
    if (selected_ap >= 0 && get_ap_valid(selected_ap, now_ms)) {
        buzzer_configure(true, rssi_to_beep_interval(ap_rssi[selected_ap]), now_ms);
    } else {
        buzzer_configure(false, 0, now_ms);
    }
}

static void print_status(uint32_t now_ms)
{
    for (int i = 0; i < AP_COUNT; ++i) {
        if (get_ap_valid(i, now_ms)) printf("AP%d: %d dBm", i + 1, ap_rssi[i]);
        else printf("AP%d: N/A", i + 1);
        if (i != AP_COUNT - 1) printf(" | ");
    }
    if (sequence_started && target_beacon < AP_COUNT) {
        printf(" | Target: AP%d | BLE state: %d%s\n", target_beacon + 1,
               ble_state, transfer_active ? " | TRANSFER/BEEP ON" : "");
    } else if (ble_state == BLE_FINISHED) {
        printf(" | ALL TRANSFERS COMPLETE\n");
    } else {
        printf(" | Press GP%d when all APs are visible\n", START_BUTTON_PIN);
    }
}

static bool advertisement_get_beacon(uint8_t *packet, uint8_t *beacon_id)
{
    const uint8_t *data = gap_event_advertising_report_get_data(packet);
    uint8_t length = gap_event_advertising_report_get_data_length(packet);
    ad_context_t context;
    for (ad_iterator_init(&context, length, data);
         ad_iterator_has_more(&context); ad_iterator_next(&context)) {
        if (ad_iterator_get_data_type(&context) !=
            BLUETOOTH_DATA_TYPE_SERVICE_DATA) continue;
        uint8_t data_length = ad_iterator_get_data_len(&context);
        const uint8_t *field = ad_iterator_get_data(&context);
        if (data_length >= 4 && transfer_read_u16(field) == BEACON_SERVICE_UUID &&
            field[2] == BEACON_PROTOCOL_VERSION) {
            *beacon_id = field[3];
            return true;
        }
    }
    return false;
}

static void reset_ble_samples(void)
{
    memset(ble_rssi_samples, 0, sizeof(ble_rssi_samples));
    ble_rssi_sample_count = 0;
    ble_rssi_sample_index = 0;
    qualifying_average_count = 0;
}

static void start_target_scan(void)
{
    current_transfer_succeeded = false;
    transfer_active = false;
    wifi_scans_paused = false;
    ble_connect_pending = false;
    reset_ble_samples();
    ble_state = BLE_SCANNING;
    gap_set_scan_parameters(0, 0x0030, 0x0030);
    gap_start_scan();
    printf("\nAP%d: BLE scan (threshold %d dBm, %d samples, %d confirmations)\n",
           target_beacon + 1, BLE_CLOSE_RSSI_DBM, BLE_RSSI_SAMPLE_COUNT,
           BLE_CLOSE_REQUIRED_AVERAGES);
}

static void start_scan_callback(void *context)
{
    (void)context;
    start_target_scan();
}

static void restart_current_target(void *context)
{
    (void)context;
    if (sequence_started && target_beacon < AP_COUNT) start_target_scan();
}

static void request_start_target_scan(void)
{
    start_scan_registration.callback = start_scan_callback;
    btstack_run_loop_execute_on_main_thread(&start_scan_registration);
}

static void connect_callback(void *context)
{
    (void)context;
    printf("Wi-Fi scan paused; connecting to AP%d over BLE\n",
           target_beacon + 1);
    gap_connect(beacon_address, beacon_address_type);
}

static void transfer_failed(const char *reason)
{
    printf("BLE transfer error for AP%d: %s; retrying\n",
           target_beacon + 1, reason);
    current_transfer_succeeded = false;
    transfer_active = false;
    wifi_scans_paused = false;
    ble_connect_pending = false;
    if (connection_handle != HCI_CON_HANDLE_INVALID) {
        ble_state = BLE_DISCONNECTING;
        gap_disconnect(connection_handle);
    } else {
        restart_registration.callback = restart_current_target;
        btstack_run_loop_execute_on_main_thread(&restart_registration);
    }
}

static bool write_packet(uint16_t length, ble_state_t writing_state)
{
    ble_state = writing_state;
    uint8_t status = gatt_client_write_value_of_characteristic(
        gatt_client_handler, connection_handle,
        transfer_characteristic.value_handle, length, tx_packet);
    if (status != ERROR_CODE_SUCCESS) {
        transfer_failed("could not start GATT write");
        return false;
    }
    return true;
}

static void write_simple(uint8_t type, ble_state_t writing_state)
{
    tx_packet[0] = type;
    write_packet(1, writing_state);
}

static void write_next_robot_chunk(void)
{
    if (robot_file_offset >= robot_file_length) {
        write_simple(MSG_FILE_END, BLE_WRITE_FILE_END);
        return;
    }
    uint16_t remaining = robot_file_length - robot_file_offset;
    uint8_t chunk = remaining > TRANSFER_DATA_BYTES
                        ? TRANSFER_DATA_BYTES : (uint8_t)remaining;
    tx_packet[0] = MSG_FILE_DATA;
    tx_packet[1] = robot_file_sequence++;
    tx_packet[2] = chunk;
    memcpy(&tx_packet[3], &robot_file[robot_file_offset], chunk);
    robot_file_offset += chunk;
    write_packet((uint16_t)(3 + chunk), BLE_WRITE_FILE_DATA);
}

static void request_next_reply_piece(void)
{
    write_simple(MSG_REPLY_NEXT, BLE_WRITE_REPLY_NEXT);
}

static void handle_notification(const uint8_t *value, uint16_t length)
{
    if (length == 0) return;
    switch (value[0]) {
    case MSG_HELLO_ACK:
        if (ble_state != BLE_WAIT_HELLO_ACK || length < 2 ||
            value[1] != target_beacon + 1) return;
        robot_file_length = (uint16_t)snprintf(
            (char *)robot_file, sizeof(robot_file),
            "Robot test file delivered to beacon AP%d", target_beacon + 1);
        robot_file_crc = transfer_crc32(robot_file, robot_file_length);
        robot_file_offset = 0;
        robot_file_sequence = 0;
        tx_packet[0] = MSG_FILE_START;
        transfer_write_u16(&tx_packet[1], robot_file_length);
        transfer_write_u32(&tx_packet[3], robot_file_crc);
        printf("AP%d handshake complete; sending %u-byte test file\n",
               target_beacon + 1, robot_file_length);
        write_packet(7, BLE_WRITE_FILE_START);
        break;

    case MSG_FILE_READY:
        if (ble_state == BLE_WAIT_FILE_READY) write_next_robot_chunk();
        break;

    case MSG_FILE_RECEIVED:
        if (ble_state != BLE_WAIT_FILE_RECEIVED || length < 6) return;
        if (value[1] != 0 || transfer_read_u32(&value[2]) != robot_file_crc) {
            transfer_failed("beacon rejected robot file or CRC");
            return;
        }
        printf("AP%d verified robot file; requesting reply file\n",
               target_beacon + 1);
        write_simple(MSG_REPLY_REQUEST, BLE_WRITE_REPLY_REQUEST);
        break;

    case MSG_REPLY_START:
        if (ble_state != BLE_WAIT_REPLY_START || length < 7) return;
        reply_file_length = transfer_read_u16(&value[1]);
        reply_file_crc = transfer_read_u32(&value[3]);
        reply_file_offset = 0;
        reply_file_sequence = 0;
        if (reply_file_length > sizeof(reply_file)) {
            transfer_failed("reply file too large");
            return;
        }
        request_next_reply_piece();
        break;

    case MSG_REPLY_DATA: {
        if (ble_state != BLE_WAIT_REPLY_PIECE || length < 3) return;
        uint8_t sequence = value[1];
        uint8_t chunk = value[2];
        if (sequence != reply_file_sequence || chunk > TRANSFER_DATA_BYTES ||
            length != (uint16_t)(3 + chunk) ||
            reply_file_offset + chunk > reply_file_length) {
            transfer_failed("invalid reply chunk");
            return;
        }
        memcpy(&reply_file[reply_file_offset], &value[3], chunk);
        reply_file_offset += chunk;
        reply_file_sequence++;
        request_next_reply_piece();
        break;
    }

    case MSG_REPLY_END:
        if (ble_state != BLE_WAIT_REPLY_PIECE || length < 5) return;
        if (reply_file_offset != reply_file_length ||
            transfer_crc32(reply_file, reply_file_length) != reply_file_crc ||
            transfer_read_u32(&value[1]) != reply_file_crc) {
            transfer_failed("reply file length or CRC failed");
            return;
        }
        printf("AP%d reply verified: %.*s\n", target_beacon + 1,
               reply_file_length, reply_file);
        write_simple(MSG_COMPLETE, BLE_WRITE_COMPLETE);
        break;

    case MSG_COMPLETE_ACK:
        if (ble_state != BLE_WAIT_COMPLETE_ACK) return;
        printf("AP%d TRANSFER COMPLETE; both boards acknowledged success\n",
               target_beacon + 1);
        current_transfer_succeeded = true;
        transfer_active = false;
        wifi_scans_paused = false;
        ble_state = BLE_DISCONNECTING;
        gap_disconnect(connection_handle);
        break;

    case MSG_ERROR:
        transfer_failed("beacon reported protocol error");
        break;
    default:
        break;
    }
}

static bool query_succeeded(uint8_t *packet)
{
    if (gatt_event_query_complete_get_att_status(packet) != ATT_ERROR_SUCCESS) {
        transfer_failed("GATT operation failed");
        return false;
    }
    return true;
}

static void gatt_client_handler(uint8_t packet_type, uint16_t channel,
                                uint8_t *packet, uint16_t size)
{
    (void)packet_type;
    (void)channel;
    (void)size;
    uint8_t event = hci_event_packet_get_type(packet);

    if (event == GATT_EVENT_NOTIFICATION) {
        handle_notification(gatt_event_notification_get_value(packet),
                            gatt_event_notification_get_value_length(packet));
        return;
    }
    if (event == GATT_EVENT_SERVICE_QUERY_RESULT &&
        ble_state == BLE_DISCOVER_SERVICE) {
        gatt_event_service_query_result_get_service(packet, &transfer_service);
        service_found = true;
        return;
    }
    if (event == GATT_EVENT_CHARACTERISTIC_QUERY_RESULT &&
        ble_state == BLE_DISCOVER_CHARACTERISTIC) {
        gatt_event_characteristic_query_result_get_characteristic(
            packet, &transfer_characteristic);
        characteristic_found = true;
        return;
    }
    if (event != GATT_EVENT_QUERY_COMPLETE) return;

    switch (ble_state) {
    case BLE_DISCOVER_SERVICE:
        if (!query_succeeded(packet)) return;
        if (!service_found) {
            transfer_failed("service not found");
            return;
        }
        characteristic_found = false;
        ble_state = BLE_DISCOVER_CHARACTERISTIC;
        gatt_client_discover_characteristics_for_service_by_uuid16(
            gatt_client_handler, connection_handle, &transfer_service,
            BEACON_CHARACTERISTIC_UUID);
        break;
    case BLE_DISCOVER_CHARACTERISTIC:
        if (!query_succeeded(packet)) return;
        if (!characteristic_found) {
            transfer_failed("characteristic not found");
            return;
        }
        notification_listener_registered = true;
        gatt_client_listen_for_characteristic_value_updates(
            &notification_listener, gatt_client_handler, connection_handle,
            &transfer_characteristic);
        ble_state = BLE_ENABLE_NOTIFICATIONS;
        gatt_client_write_client_characteristic_configuration(
            gatt_client_handler, connection_handle, &transfer_characteristic,
            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
        break;
    case BLE_ENABLE_NOTIFICATIONS:
        if (!query_succeeded(packet)) return;
        tx_packet[0] = MSG_HELLO;
        tx_packet[1] = BEACON_PROTOCOL_VERSION;
        tx_packet[2] = (uint8_t)(target_beacon + 1);
        write_packet(3, BLE_WRITE_HELLO);
        break;
    case BLE_WRITE_HELLO:
        if (query_succeeded(packet)) ble_state = BLE_WAIT_HELLO_ACK;
        break;
    case BLE_WRITE_FILE_START:
        if (query_succeeded(packet)) ble_state = BLE_WAIT_FILE_READY;
        break;
    case BLE_WRITE_FILE_DATA:
        if (query_succeeded(packet)) write_next_robot_chunk();
        break;
    case BLE_WRITE_FILE_END:
        if (query_succeeded(packet)) ble_state = BLE_WAIT_FILE_RECEIVED;
        break;
    case BLE_WRITE_REPLY_REQUEST:
        if (query_succeeded(packet)) ble_state = BLE_WAIT_REPLY_START;
        break;
    case BLE_WRITE_REPLY_NEXT:
        if (query_succeeded(packet)) ble_state = BLE_WAIT_REPLY_PIECE;
        break;
    case BLE_WRITE_COMPLETE:
        if (query_succeeded(packet)) ble_state = BLE_WAIT_COMPLETE_ACK;
        break;
    default:
        break;
    }
}

static void handle_advertisement(uint8_t *packet)
{
    uint8_t beacon_id;
    if (!advertisement_get_beacon(packet, &beacon_id) ||
        beacon_id != target_beacon + 1) return;

    int rssi = gap_event_advertising_report_get_rssi(packet);
    ble_rssi_samples[ble_rssi_sample_index] = rssi;
    ble_rssi_sample_index =
        (uint8_t)((ble_rssi_sample_index + 1) % BLE_RSSI_SAMPLE_COUNT);
    if (ble_rssi_sample_count < BLE_RSSI_SAMPLE_COUNT) ble_rssi_sample_count++;
    if (ble_rssi_sample_count < BLE_RSSI_SAMPLE_COUNT) return;

    int sum = 0;
    for (int i = 0; i < BLE_RSSI_SAMPLE_COUNT; ++i) sum += ble_rssi_samples[i];
    int average = sum / BLE_RSSI_SAMPLE_COUNT;
    if (average >= BLE_CLOSE_RSSI_DBM) qualifying_average_count++;
    else qualifying_average_count = 0;
    printf("AP%d BLE: latest %d, average %d dBm, close %u/%d\n",
           target_beacon + 1, rssi, average, qualifying_average_count,
           BLE_CLOSE_REQUIRED_AVERAGES);
    if (qualifying_average_count < BLE_CLOSE_REQUIRED_AVERAGES) return;

    transfer_active = true;
    wifi_scans_paused = true;
    ble_state = BLE_CONNECTING;
    gap_event_advertising_report_get_address(packet, beacon_address);
    beacon_address_type = gap_event_advertising_report_get_address_type(packet);
    gap_stop_scan();
    ble_connect_pending = true;
    printf("AP%d accepted as close; constant buzzer ON\n",
           target_beacon + 1);
}

static void hci_event_handler(uint8_t packet_type, uint16_t channel,
                              uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            bluetooth_ready = true;
            printf("Bluetooth ready\n");
        }
        break;
    case GAP_EVENT_ADVERTISING_REPORT:
        if (ble_state == BLE_SCANNING) handle_advertisement(packet);
        break;
    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) ==
                GAP_SUBEVENT_LE_CONNECTION_COMPLETE &&
            ble_state == BLE_CONNECTING) {
            uint8_t status = gap_subevent_le_connection_complete_get_status(packet);
            if (status != ERROR_CODE_SUCCESS) {
                connection_handle = HCI_CON_HANDLE_INVALID;
                transfer_failed("connection failed");
                return;
            }
            connection_handle =
                gap_subevent_le_connection_complete_get_connection_handle(packet);
            service_found = false;
            ble_state = BLE_DISCOVER_SERVICE;
            printf("AP%d connected; starting application handshake\n",
                   target_beacon + 1);
            gatt_client_discover_primary_services_by_uuid16(
                gatt_client_handler, connection_handle, BEACON_SERVICE_UUID);
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        connection_handle = HCI_CON_HANDLE_INVALID;
        if (notification_listener_registered) {
            notification_listener_registered = false;
            gatt_client_stop_listening_for_characteristic_value_updates(
                &notification_listener);
        }
        transfer_active = false;
        wifi_scans_paused = false;
        if (current_transfer_succeeded) {
            beacon_complete[target_beacon] = true;
            target_beacon++;
            current_transfer_succeeded = false;
            if (target_beacon >= AP_COUNT) {
                sequence_started = false;
                ble_state = BLE_FINISHED;
                selected_ap = -1;
                printf("\nALL THREE BEACON FILE TRANSFERS COMPLETE\n");
            } else {
                start_target_scan();
            }
        } else if (sequence_started) {
            start_target_scan();
        } else {
            ble_state = BLE_IDLE;
        }
        break;
    default:
        break;
    }
}

int main(void)
{
    stdio_init_all();
    leds_init();
    buzzer_init();
    gpio_init(START_BUTTON_PIN);
    gpio_set_dir(START_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(START_BUTTON_PIN);
    sleep_ms(2000);

    printf("\n========================================\n");
    printf(" Pico W Wi-Fi + BLE Three-Beacon Test\n");
    printf("========================================\n");
    printf("Start button: GP%d (active low)\n", START_BUTTON_PIN);
    printf("BLE close threshold: %d dBm\n", BLE_CLOSE_RSSI_DBM);
    printf("BLE filter: %d samples, %d qualifying averages\n\n",
           BLE_RSSI_SAMPLE_COUNT, BLE_CLOSE_REQUIRED_AVERAGES);

    if (cyw43_arch_init()) {
        printf("ERROR: failed to initialise Wi-Fi/Bluetooth\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    att_server_init(NULL, NULL, NULL);
    gatt_client_init();
    hci_event_registration.callback = hci_event_handler;
    hci_add_event_handler(&hci_event_registration);
    hci_power_control(HCI_POWER_ON);

    bool wifi_scan_in_progress = false;
    uint32_t next_wifi_scan_ms = 0;
    uint32_t next_print_ms = 0;
    bool last_button = true;
    uint32_t button_changed_ms = 0;
    bool stable_button = true;

    while (true) {
        uint32_t now_ms = (uint32_t)(time_us_64() / 1000);
        if (!wifi_scans_paused && !wifi_scan_in_progress &&
            (int32_t)(now_ms - next_wifi_scan_ms) >= 0) {
            cyw43_wifi_scan_options_t options = {0};
            int error = cyw43_wifi_scan(
                &cyw43_state, &options, NULL, wifi_scan_result);
            if (error == 0) wifi_scan_in_progress = true;
            else next_wifi_scan_ms = now_ms + 500;
        }
        if (wifi_scan_in_progress && !cyw43_wifi_scan_active(&cyw43_state)) {
            wifi_scan_in_progress = false;
            next_wifi_scan_ms = now_ms + 100;
        }
        if (ble_connect_pending && !wifi_scan_in_progress) {
            ble_connect_pending = false;
            connect_registration.callback = connect_callback;
            btstack_run_loop_execute_on_main_thread(&connect_registration);
        }

        bool raw_button = gpio_get(START_BUTTON_PIN);
        if (raw_button != last_button) {
            last_button = raw_button;
            button_changed_ms = now_ms;
        }
        if (raw_button != stable_button &&
            (uint32_t)(now_ms - button_changed_ms) >= BUTTON_DEBOUNCE_MS) {
            stable_button = raw_button;
            if (!stable_button && !sequence_started &&
                ble_state != BLE_DISCONNECTING) {
                if (!bluetooth_ready) {
                    printf("Start ignored: Bluetooth not ready\n");
                } else if (!all_access_points_visible(now_ms)) {
                    printf("Start ignored: all three Wi-Fi APs must be visible\n");
                } else {
                    for (int i = 0; i < AP_COUNT; ++i) {
                        beacon_complete[i] = false;
                    }
                    target_beacon = 0;
                    sequence_started = true;
                    printf("\nSTART accepted: AP1 -> AP2 -> AP3\n");
                    request_start_target_scan();
                }
            }
        }

        if ((int32_t)(now_ms - next_print_ms) >= 0) {
            update_output(now_ms);
            print_status(now_ms);
            next_print_ms = now_ms + PRINT_INTERVAL_MS;
        }
        buzzer_service(now_ms);
        sleep_ms(5);
    }
}
