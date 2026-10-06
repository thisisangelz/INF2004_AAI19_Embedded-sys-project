#include "payload_client.h"
#include "motion.h"
#include "run_report.h"
#include "transfer_protocol.h"
#include <stdio.h>
#include <string.h>
#include "btstack.h"
#include "pico/stdlib.h"

#define TARGET_BEACON_ID 1
#define CLOSE_RSSI_DBM (-50)
#define RSSI_SAMPLE_COUNT 12
#define REQUIRED_CLOSE_AVERAGES 3
#define SCAN_WINDOW_MS 5000
#define PHASE_TIMEOUT_MS 5000
#define CLEANUP_TIMEOUT_MS 3000
#define ATTEMPT_TIMEOUT_MS 30000

typedef enum {
    BLE_IDLE, BLE_STARTING, BLE_SCANNING, BLE_STOPPING_FOR_CONNECT,
    BLE_CONNECTING, BLE_DISCOVER_SERVICE, BLE_DISCOVER_CHARACTERISTIC,
    BLE_ENABLE_NOTIFICATIONS, BLE_WRITE_HELLO, BLE_WAIT_HELLO_ACK,
    BLE_WRITE_FILE_START, BLE_WAIT_FILE_READY, BLE_WRITE_FILE_DATA,
    BLE_WRITE_FILE_END, BLE_WAIT_FILE_RECEIVED, BLE_WRITE_REPLY_REQUEST,
    BLE_WAIT_REPLY_START, BLE_WRITE_REPLY_NEXT, BLE_WAIT_REPLY_PIECE,
    BLE_WRITE_COMPLETE, BLE_WAIT_COMPLETE_ACK, BLE_STOPPING_SCAN, BLE_CLOSING
} ble_state_t;

static volatile ble_state_t ble_state = BLE_IDLE;
static volatile bool bluetooth_ready;
static volatile bool attempt_running;
static volatile bool abort_requested;
static volatile payload_result_t attempt_result;
static payload_result_t closing_result;
static volatile uint32_t phase_started_ms;
static bool close_confirmed;
static bool scan_confirmed;
static bool payload_verified;
static btstack_packet_callback_registration_t event_registration;
static btstack_context_callback_registration_t start_registration;
static btstack_context_callback_registration_t abort_registration;
static bd_addr_t beacon_address;
static bd_addr_type_t beacon_address_type;
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static gatt_client_service_t service;
static gatt_client_characteristic_t characteristic;
static gatt_client_notification_t listener;
static bool listening;
static bool service_found;
static bool characteristic_found;
static int rssi_samples[RSSI_SAMPLE_COUNT];
static int sample_count;
static int sample_index;
static int close_count;
static uint8_t tx_packet[TRANSFER_PACKET_SIZE];
static const uint8_t robot_file[] = "Robot test file delivered to beacon AP1";
static uint16_t robot_offset;
static uint8_t robot_sequence;
static uint32_t robot_crc;
static uint8_t reply_file[TRANSFER_MAX_FILE_SIZE];
static uint16_t reply_length;
static uint16_t reply_offset;
static uint8_t reply_sequence;
static uint32_t reply_crc;
// A beacon can notify before the ATT write completion arrives.
static uint8_t pending_notification[TRANSFER_PACKET_SIZE];
static uint16_t pending_length;

static void gatt_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_notification(const uint8_t *value, uint16_t length);
static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }
static void set_state(ble_state_t state)
{
    phase_started_ms = now_ms();
    ble_state = state;
}

static void finish_attempt(void)
{
    if (listening) {
        gatt_client_stop_listening_for_characteristic_value_updates(&listener);
        listening = false;
    }
    connection_handle = HCI_CON_HANDLE_INVALID;
    set_state(BLE_IDLE);
    attempt_result = closing_result;
    attempt_running = false;
}

static void close_attempt(payload_result_t result)
{
    closing_result = result;
    pending_length = 0;
    if (ble_state == BLE_SCANNING || ble_state == BLE_STOPPING_FOR_CONNECT) {
        bool already_stopping = ble_state == BLE_STOPPING_FOR_CONNECT;
        set_state(BLE_STOPPING_SCAN);
        if (!already_stopping) gap_stop_scan();
        if (!scan_confirmed) {
            // No movement or new radio operation is allowed after this fault.
            closing_result = PAYLOAD_FATAL;
            finish_attempt();
        }
    } else if (connection_handle != HCI_CON_HANDLE_INVALID) {
        set_state(BLE_CLOSING);
        if (gap_disconnect(connection_handle) != ERROR_CODE_SUCCESS) {
            closing_result = PAYLOAD_FATAL;
            // Hold the robot still; a connection may remain live.
        }
    } else if (ble_state == BLE_CONNECTING) {
        set_state(BLE_CLOSING);
        (void)gap_connect_cancel();
    } else {
        finish_attempt();
    }
}

static void transfer_failed(const char *reason)
{
    report_printf("AP1 PAYLOAD FAILED: %s\n", reason);
    payload_verified = false;
    close_attempt(close_confirmed ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE);
}

static bool writing_state(ble_state_t state)
{
    return state == BLE_WRITE_HELLO || state == BLE_WRITE_FILE_START ||
           state == BLE_WRITE_FILE_DATA || state == BLE_WRITE_FILE_END ||
           state == BLE_WRITE_REPLY_REQUEST || state == BLE_WRITE_REPLY_NEXT ||
           state == BLE_WRITE_COMPLETE;
}

static bool write_packet(uint16_t length, ble_state_t state)
{
    set_state(state);
    uint8_t status = gatt_client_write_value_of_characteristic(
        gatt_handler, connection_handle, characteristic.value_handle, length, tx_packet);
    if (status != ERROR_CODE_SUCCESS) {
        transfer_failed("could not start GATT write");
        return false;
    }
    return true;
}

static void write_simple(uint8_t type, ble_state_t state)
{
    tx_packet[0] = type;
    (void)write_packet(1, state);
}

static void write_next_robot_chunk(void)
{
    const uint16_t length = sizeof(robot_file) - 1;
    if (robot_offset == length) {
        write_simple(MSG_FILE_END, BLE_WRITE_FILE_END);
        return;
    }
    uint16_t remaining = length - robot_offset;
    uint8_t chunk = remaining > TRANSFER_DATA_BYTES ? TRANSFER_DATA_BYTES : (uint8_t)remaining;
    tx_packet[0] = MSG_FILE_DATA;
    tx_packet[1] = robot_sequence++;
    tx_packet[2] = chunk;
    memcpy(tx_packet + 3, robot_file + robot_offset, chunk);
    robot_offset += chunk;
    (void)write_packet(3 + chunk, BLE_WRITE_FILE_DATA);
}

static void handle_notification(const uint8_t *value, uint16_t length)
{
    if (!length || ble_state == BLE_CLOSING || ble_state == BLE_IDLE) return;
    if (value[0] == MSG_ERROR) { transfer_failed("beacon reported protocol error"); return; }
    if (writing_state(ble_state)) {
        if (length > sizeof(pending_notification) || pending_length) {
            transfer_failed("unexpected notification while write is pending");
            return;
        }
        memcpy(pending_notification, value, length);
        pending_length = length;
        return;
    }
    switch (value[0]) {
    case MSG_HELLO_ACK:
        if (ble_state != BLE_WAIT_HELLO_ACK) return;
        if (length != 2 || value[1] != TARGET_BEACON_ID) {
            transfer_failed("wrong beacon identity"); return;
        }
        robot_offset = 0;
        robot_sequence = 0;
        robot_crc = transfer_crc32(robot_file, sizeof(robot_file) - 1);
        tx_packet[0] = MSG_FILE_START;
        transfer_write_u16(tx_packet + 1, sizeof(robot_file) - 1);
        transfer_write_u32(tx_packet + 3, robot_crc);
        report_printf("AP1 handshake confirmed; sending robot test file\n");
        (void)write_packet(7, BLE_WRITE_FILE_START);
        break;
    case MSG_FILE_READY:
        if (ble_state != BLE_WAIT_FILE_READY) return;
        if (length != 2 || value[1] != TARGET_BEACON_ID) {
            transfer_failed("invalid FILE_READY"); return;
        }
        write_next_robot_chunk();
        break;
    case MSG_FILE_RECEIVED:
        if (ble_state != BLE_WAIT_FILE_RECEIVED) return;
        if (length != 6 || value[1] || transfer_read_u32(value + 2) != robot_crc) {
            transfer_failed("robot file CRC rejected"); return;
        }
        write_simple(MSG_REPLY_REQUEST, BLE_WRITE_REPLY_REQUEST);
        break;
    case MSG_REPLY_START:
        if (ble_state != BLE_WAIT_REPLY_START) return;
        if (length != 7) { transfer_failed("invalid reply metadata"); return; }
        reply_length = transfer_read_u16(value + 1);
        reply_crc = transfer_read_u32(value + 3);
        reply_offset = 0;
        reply_sequence = 0;
        if (!reply_length || reply_length > sizeof(reply_file)) {
            transfer_failed("reply payload length outside 1..256 bytes"); return;
        }
        report_printf("AP1 reply announced: %u bytes, CRC %08lx\n",
                      reply_length, (unsigned long)reply_crc);
        write_simple(MSG_REPLY_NEXT, BLE_WRITE_REPLY_NEXT);
        break;
    case MSG_REPLY_DATA: {
        if (ble_state != BLE_WAIT_REPLY_PIECE) return;
        if (length < 3) { transfer_failed("short reply chunk"); return; }
        uint8_t chunk = value[2];
        if (value[1] != reply_sequence || !chunk || chunk > TRANSFER_DATA_BYTES ||
            length != 3u + chunk || reply_offset + chunk > reply_length) {
            transfer_failed("invalid reply chunk sequence or length"); return;
        }
        memcpy(reply_file + reply_offset, value + 3, chunk);
        reply_offset += chunk;
        ++reply_sequence;
        write_simple(MSG_REPLY_NEXT, BLE_WRITE_REPLY_NEXT);
        break;
    }
    case MSG_REPLY_END:
        if (ble_state != BLE_WAIT_REPLY_PIECE) return;
        if (length != 5 || reply_offset != reply_length ||
            transfer_crc32(reply_file, reply_length) != reply_crc ||
            transfer_read_u32(value + 1) != reply_crc) {
            transfer_failed("reply payload CRC or length failed"); return;
        }
        report_printf("AP1 reply payload length and CRC verified\n");
        write_simple(MSG_COMPLETE, BLE_WRITE_COMPLETE);
        break;
    case MSG_COMPLETE_ACK:
        if (ble_state != BLE_WAIT_COMPLETE_ACK) return;
        if (length != 2 || value[1] != TARGET_BEACON_ID) {
            transfer_failed("invalid final acknowledgement"); return;
        }
        payload_verified = true;
        report_printf("AP1 PAYLOAD COMPLETE: verified reply and final acknowledgement\n");
        report_printf("AP1 payload: %.*s\n", reply_length, (const char *)reply_file);
        close_attempt(PAYLOAD_COMPLETE);
        break;
    default:
        break;
    }
}

static bool query_ok(uint8_t *packet)
{
    if (gatt_event_query_complete_get_att_status(packet) == ATT_ERROR_SUCCESS) return true;
    transfer_failed("GATT operation rejected");
    return false;
}

static void gatt_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    if (type != HCI_EVENT_PACKET || abort_requested) return;
    if (ble_state == BLE_CLOSING || ble_state == BLE_IDLE) return;
    uint8_t event = hci_event_packet_get_type(packet);
    if (event == GATT_EVENT_NOTIFICATION) {
        if (gatt_event_notification_get_handle(packet) == connection_handle &&
            gatt_event_notification_get_value_handle(packet) == characteristic.value_handle) {
            handle_notification(gatt_event_notification_get_value(packet),
                                gatt_event_notification_get_value_length(packet));
        }
        return;
    }
    if (event == GATT_EVENT_SERVICE_QUERY_RESULT && ble_state == BLE_DISCOVER_SERVICE) {
        gatt_event_service_query_result_get_service(packet, &service);
        service_found = true; return;
    }
    if (event == GATT_EVENT_CHARACTERISTIC_QUERY_RESULT && ble_state == BLE_DISCOVER_CHARACTERISTIC) {
        gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
        characteristic_found = true; return;
    }
    if (event != GATT_EVENT_QUERY_COMPLETE ||
        gatt_event_query_complete_get_handle(packet) != connection_handle || !query_ok(packet)) return;
    uint8_t status = ERROR_CODE_SUCCESS;
    switch (ble_state) {
    case BLE_DISCOVER_SERVICE:
        if (!service_found) { transfer_failed("transfer service missing"); return; }
        characteristic_found = false;
        set_state(BLE_DISCOVER_CHARACTERISTIC);
        status = gatt_client_discover_characteristics_for_service_by_uuid16(
            gatt_handler, connection_handle, &service, BEACON_CHARACTERISTIC_UUID);
        break;
    case BLE_DISCOVER_CHARACTERISTIC:
        if (!characteristic_found) { transfer_failed("transfer characteristic missing"); return; }
        gatt_client_listen_for_characteristic_value_updates(&listener, gatt_handler,
                                                          connection_handle, &characteristic);
        listening = true;
        set_state(BLE_ENABLE_NOTIFICATIONS);
        status = gatt_client_write_client_characteristic_configuration(
            gatt_handler, connection_handle, &characteristic,
            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
        break;
    case BLE_ENABLE_NOTIFICATIONS:
        tx_packet[0] = MSG_HELLO;
        tx_packet[1] = BEACON_PROTOCOL_VERSION;
        tx_packet[2] = TARGET_BEACON_ID;
        (void)write_packet(3, BLE_WRITE_HELLO);
        break;
    case BLE_WRITE_HELLO: set_state(BLE_WAIT_HELLO_ACK); break;
    case BLE_WRITE_FILE_START: set_state(BLE_WAIT_FILE_READY); break;
    case BLE_WRITE_FILE_DATA: write_next_robot_chunk(); break;
    case BLE_WRITE_FILE_END: set_state(BLE_WAIT_FILE_RECEIVED); break;
    case BLE_WRITE_REPLY_REQUEST: set_state(BLE_WAIT_REPLY_START); break;
    case BLE_WRITE_REPLY_NEXT: set_state(BLE_WAIT_REPLY_PIECE); break;
    case BLE_WRITE_COMPLETE: set_state(BLE_WAIT_COMPLETE_ACK); break;
    default: return;
    }
    if (status != ERROR_CODE_SUCCESS) { transfer_failed("GATT setup could not start"); return; }
    if (pending_length && !writing_state(ble_state)) {
        uint8_t value[TRANSFER_PACKET_SIZE];
        uint16_t length = pending_length;
        memcpy(value, pending_notification, length);
        pending_length = 0;
        handle_notification(value, length);
    }
}

static bool target_advertisement(uint8_t *packet)
{
    const uint8_t *data = gap_event_advertising_report_get_data(packet);
    uint8_t length = gap_event_advertising_report_get_data_length(packet);
    ad_context_t context;
    for (ad_iterator_init(&context, length, data); ad_iterator_has_more(&context);
         ad_iterator_next(&context)) {
        if (ad_iterator_get_data_type(&context) != BLUETOOTH_DATA_TYPE_SERVICE_DATA) continue;
        const uint8_t *field = ad_iterator_get_data(&context);
        if (ad_iterator_get_data_len(&context) >= 4 &&
            transfer_read_u16(field) == BEACON_SERVICE_UUID &&
            field[2] == BEACON_PROTOCOL_VERSION && field[3] == TARGET_BEACON_ID) return true;
    }
    return false;
}

static int filtered_rssi(void)
{
    int sorted[RSSI_SAMPLE_COUNT];
    memcpy(sorted, rssi_samples, sizeof(sorted));
    for (int i = 1; i < RSSI_SAMPLE_COUNT; ++i) {
        int value = sorted[i], j = i;
        while (j > 0 && sorted[j - 1] > value) { sorted[j] = sorted[j - 1]; --j; }
        sorted[j] = value;
    }
    int sum = 0;
    const int trim = RSSI_SAMPLE_COUNT / 4;
    for (int i = trim; i < RSSI_SAMPLE_COUNT - trim; ++i) sum += sorted[i];
    const int count = RSSI_SAMPLE_COUNT - 2 * trim;
    return (sum - count / 2) / count;
}

static void advertisement_received(uint8_t *packet)
{
    if (abort_requested || !target_advertisement(packet)) return;
    int rssi = (int8_t)gap_event_advertising_report_get_rssi(packet);
    if (rssi >= 0 || rssi <= -127) return;
    rssi_samples[sample_index] = rssi;
    sample_index = (sample_index + 1) % RSSI_SAMPLE_COUNT;
    if (sample_count < RSSI_SAMPLE_COUNT) ++sample_count;
    if (sample_count < RSSI_SAMPLE_COUNT) return;
    int average = filtered_rssi();
    close_count = average >= CLOSE_RSSI_DBM ? close_count + 1 : 0;
    report_printf("AP1 BLE: filtered %d dBm, need >= %d, close %d/%d\n",
                  average, CLOSE_RSSI_DBM, close_count, REQUIRED_CLOSE_AVERAGES);
    if (close_count < REQUIRED_CLOSE_AVERAGES) return;
    close_confirmed = true;
    gap_event_advertising_report_get_address(packet, beacon_address);
    beacon_address_type = gap_event_advertising_report_get_address_type(packet);
    set_state(BLE_STOPPING_FOR_CONNECT);
    gap_stop_scan();
    report_printf("AP1 close-range check passed; motors stopped for payload transfer\n");
}

static void connection_complete(uint8_t status, hci_con_handle_t handle)
{
    if (ble_state != BLE_CONNECTING && ble_state != BLE_CLOSING) return;
    if (status != ERROR_CODE_SUCCESS) {
        connection_handle = HCI_CON_HANDLE_INVALID;
        if (ble_state == BLE_CLOSING) finish_attempt();
        else {
            // The failed connection procedure has already ended.
            set_state(BLE_IDLE);
            transfer_failed("BLE connection failed");
        }
        return;
    }
    connection_handle = handle;
    if (abort_requested || ble_state == BLE_CLOSING) {
        set_state(BLE_CLOSING);
        (void)gap_disconnect(handle);
        return;
    }
    service_found = false;
    set_state(BLE_DISCOVER_SERVICE);
    report_printf("AP1 BLE connected; discovering payload service\n");
    if (gatt_client_discover_primary_services_by_uuid16(gatt_handler, handle,
                                                       BEACON_SERVICE_UUID) != ERROR_CODE_SUCCESS) {
        transfer_failed("service discovery could not start");
    }
}

static void hci_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    if (type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        bluetooth_ready = btstack_event_state_get_state(packet) == HCI_STATE_WORKING;
        if (bluetooth_ready) report_printf("Bluetooth ready\n");
        break;
    case HCI_EVENT_COMMAND_COMPLETE:
        if (hci_event_command_complete_get_command_opcode(packet) != hci_le_set_scan_enable.opcode) break;
        if (hci_event_command_complete_get_return_parameters(packet)[0] != ERROR_CODE_SUCCESS) {
            report_printf("BLE scan command failed; stopping approach\n");
            closing_result = PAYLOAD_FATAL;
            finish_attempt();
        } else if (ble_state == BLE_SCANNING) {
            scan_confirmed = true;
        } else if (ble_state == BLE_STOPPING_FOR_CONNECT) {
            if (abort_requested) {
                closing_result = PAYLOAD_FATAL;
                finish_attempt();
            } else {
                set_state(BLE_CONNECTING);
                if (gap_connect(beacon_address, beacon_address_type) != ERROR_CODE_SUCCESS) {
                    set_state(BLE_IDLE);
                    transfer_failed("BLE connection could not start");
                }
            }
        } else if (ble_state == BLE_STOPPING_SCAN) {
            finish_attempt();
        }
        break;
    case GAP_EVENT_ADVERTISING_REPORT:
        if (ble_state == BLE_SCANNING) advertisement_received(packet);
        break;
    case HCI_EVENT_LE_META:
        if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
            connection_complete(hci_subevent_le_connection_complete_get_status(packet),
                                hci_subevent_le_connection_complete_get_connection_handle(packet));
        } else if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE_V1) {
            connection_complete(hci_subevent_le_enhanced_connection_complete_v1_get_status(packet),
                                hci_subevent_le_enhanced_connection_complete_v1_get_connection_handle(packet));
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        if (hci_event_disconnection_complete_get_connection_handle(packet) != connection_handle) break;
        connection_handle = HCI_CON_HANDLE_INVALID;
        if (ble_state != BLE_CLOSING) {
            payload_verified = false;
            closing_result = close_confirmed ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE;
            report_printf("AP1 disconnected before payload confirmation\n");
        }
        finish_attempt();
        break;
    default: break;
    }
}

static void start_attempt_callback(void *context)
{
    (void)context;
    if (abort_requested) { closing_result = PAYLOAD_FATAL; finish_attempt(); return; }
    close_confirmed = false;
    scan_confirmed = false;
    payload_verified = false;
    pending_length = 0;
    reply_length = reply_offset = 0;
    sample_count = sample_index = close_count = 0;
    memset(rssi_samples, 0, sizeof(rssi_samples));
    set_state(BLE_SCANNING);
    gap_set_scan_parameters(0, 0x0030, 0x0030);
    gap_start_scan();
    report_printf("BLE-only check: AP1, 12 readings, 3 averages >= -50 dBm; Wi-Fi paused\n");
}

static void abort_attempt_callback(void *context)
{
    (void)context;
    if (!attempt_running || ble_state == BLE_STOPPING_SCAN || ble_state == BLE_CLOSING) return;
    report_printf("BLE phase timed out or GP21 pressed; closing radio operation\n");
    close_attempt(motion_stop_requested() ? PAYLOAD_FATAL :
                  (close_confirmed ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE));
}

void payload_client_init(void)
{
    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    att_server_init(NULL, NULL, NULL);
    gatt_client_init();
    event_registration.callback = hci_handler;
    hci_add_event_handler(&event_registration);
    hci_power_control(HCI_POWER_ON);
}

bool payload_client_ready(void) { return bluetooth_ready; }

payload_result_t payload_client_attempt(void)
{
    motion_stop();
    if (!bluetooth_ready || attempt_running || motion_stop_requested()) return PAYLOAD_FATAL;
    abort_requested = false;
    attempt_running = true;
    set_state(BLE_STARTING);
    start_registration.callback = start_attempt_callback;
    btstack_run_loop_execute_on_main_thread(&start_registration);
    uint32_t start = now_ms();
    uint32_t abort_started = 0;
    while (attempt_running) {
        uint32_t elapsed = now_ms() - phase_started_ms;
        bool cleanup = ble_state == BLE_CLOSING || ble_state == BLE_STOPPING_SCAN;
        if (cleanup && elapsed >= CLEANUP_TIMEOUT_MS) {
            abort_requested = true;
            report_printf("FAILED: BLE cleanup did not finish; no further movement\n");
            return PAYLOAD_FATAL;
        }
        uint32_t timeout = ble_state == BLE_SCANNING ? SCAN_WINDOW_MS :
                           cleanup ? CLEANUP_TIMEOUT_MS : PHASE_TIMEOUT_MS;
        if (!abort_requested && (motion_stop_requested() || elapsed >= timeout ||
                                  (uint32_t)(now_ms() - start) >= ATTEMPT_TIMEOUT_MS)) {
            abort_requested = true;
            abort_started = now_ms();
            abort_registration.callback = abort_attempt_callback;
            btstack_run_loop_execute_on_main_thread(&abort_registration);
        }
        if (abort_requested && (uint32_t)(now_ms() - abort_started) >= CLEANUP_TIMEOUT_MS) {
            report_printf("FAILED: BLE cleanup did not finish; no further movement\n");
            return PAYLOAD_FATAL;
        }
        sleep_ms(10);
    }
    return motion_stop_requested() ? PAYLOAD_FATAL : attempt_result;
}

const uint8_t *payload_client_data(uint16_t *length)
{
    *length = payload_verified ? reply_length : 0;
    return payload_verified ? reply_file : NULL;
}
