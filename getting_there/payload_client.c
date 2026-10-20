#include "payload_client.h"
#include "motion.h"
#include "faults.h"
#include "packed_bits.h"
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

static volatile uint32_t phase_started_ms;
// Packed status word shared by the main loop and BTstack callbacks (see
// packed_bits.h for the atomic update rules):
//   bits 0-4   BLE state, 23 states
//   bits 5-6   result of the last attempt, 4 values
//   bits 7-8   result to report once cleanup finishes, 4 values
//   bits 9-17  one bit per flag below
#define BLE_STATE_LAST BLE_CLOSING
#define ST_STATE_SHIFT 0u
#define ST_STATE_BITS 5u
#define ST_RESULT_SHIFT 5u
#define ST_CLOSING_SHIFT 7u
#define ST_RESULT_BITS 2u
#define F_BLUETOOTH_READY (1u << 9)
#define F_ATTEMPT_RUNNING (1u << 10)
#define F_ABORT_REQUESTED (1u << 11)
#define F_CLOSE_CONFIRMED (1u << 12)
#define F_SCAN_CONFIRMED (1u << 13)
#define F_PAYLOAD_VERIFIED (1u << 14)
#define F_LISTENING (1u << 15)
#define F_SERVICE_FOUND (1u << 16)
#define F_CHARACTERISTIC_FOUND (1u << 17)
_Static_assert(ST_STATE_BITS == BITS_NEEDED(BLE_STATE_LAST), "BLE state field is not minimal");
_Static_assert(ST_RESULT_BITS == BITS_NEEDED(PAYLOAD_FATAL), "result field is not minimal");
_Static_assert(ST_CLOSING_SHIFT == ST_RESULT_SHIFT + ST_RESULT_BITS, "result fields overlap");

static volatile uint32_t status;

static ble_state_t current_state(void)
{
    return (ble_state_t)packed_get(&status, ST_STATE_SHIFT, ST_STATE_BITS);
}
static payload_result_t result_get(unsigned shift)
{
    return (payload_result_t)packed_get(&status, shift, ST_RESULT_BITS);
}
static void result_put(unsigned shift, payload_result_t result)
{
    packed_put(&status, shift, ST_RESULT_BITS, result);
}
static bool flag(uint32_t mask) { return (status & mask) != 0; }
static void flag_assign(uint32_t mask, bool value)
{
    if (value) packed_set_bits(&status, mask);
    else packed_clear_bits(&status, mask);
}

// Counters touched only from BTstack callbacks; each width is the minimum for
// its range. The main loop reads reply_length only after the attempt has ended.
static struct {
    uint32_t sample_count : 4;   // 0..RSSI_SAMPLE_COUNT
    uint32_t sample_index : 4;   // 0..RSSI_SAMPLE_COUNT-1
    uint32_t close_count : 2;    // 0..REQUIRED_CLOSE_AVERAGES
    uint32_t pending_length : 5; // 0..TRANSFER_PACKET_SIZE
    uint32_t robot_offset : 9;   // 0..TRANSFER_MAX_FILE_SIZE
    uint32_t reply_offset : 9;   // 0..TRANSFER_MAX_FILE_SIZE
    uint32_t reply_length : 9;   // 0..TRANSFER_MAX_FILE_SIZE
} counters;
_Static_assert(4 == BITS_NEEDED(RSSI_SAMPLE_COUNT), "sample_count width");
_Static_assert(4 == BITS_NEEDED(RSSI_SAMPLE_COUNT - 1), "sample_index width");
_Static_assert(2 == BITS_NEEDED(REQUIRED_CLOSE_AVERAGES), "close_count width");
_Static_assert(5 == BITS_NEEDED(TRANSFER_PACKET_SIZE), "pending_length width");
_Static_assert(9 == BITS_NEEDED(TRANSFER_MAX_FILE_SIZE), "file offset width");

static btstack_packet_callback_registration_t event_registration;
static btstack_context_callback_registration_t start_registration;
static btstack_context_callback_registration_t abort_registration;
static bd_addr_t beacon_address;
static bd_addr_type_t beacon_address_type;
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static gatt_client_service_t service;
static gatt_client_characteristic_t characteristic;
static gatt_client_notification_t listener;
// 12 readings x 7 bits, packed back to back.
static uint8_t rssi_samples[BITARRAY_BYTES(RSSI_SAMPLE_COUNT, RSSI_BITS)];
static uint8_t tx_packet[TRANSFER_PACKET_SIZE];
static const uint8_t robot_file[] = "Robot test file delivered to beacon AP1";
static uint8_t robot_sequence;
static uint32_t robot_crc;
static uint8_t reply_file[TRANSFER_MAX_FILE_SIZE];
static uint8_t reply_sequence;
static uint32_t reply_crc;
// A beacon can notify before the ATT write completion arrives.
static uint8_t pending_notification[TRANSFER_PACKET_SIZE];

static void gatt_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_notification(const uint8_t *value, uint16_t length);
static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000); }
static void set_state(ble_state_t state)
{
    phase_started_ms = now_ms();
    packed_put(&status, ST_STATE_SHIFT, ST_STATE_BITS, state);
}

static void finish_attempt(void)
{
    if (flag(F_LISTENING)) {
        gatt_client_stop_listening_for_characteristic_value_updates(&listener);
        flag_assign(F_LISTENING, false);
    }
    connection_handle = HCI_CON_HANDLE_INVALID;
    set_state(BLE_IDLE);
    result_put(ST_RESULT_SHIFT, result_get(ST_CLOSING_SHIFT));
    flag_assign(F_ATTEMPT_RUNNING, false);
}

static void close_attempt(payload_result_t result)
{
    result_put(ST_CLOSING_SHIFT, result);
    counters.pending_length = 0;
    if (current_state() == BLE_SCANNING || current_state() == BLE_STOPPING_FOR_CONNECT) {
        bool already_stopping = current_state() == BLE_STOPPING_FOR_CONNECT;
        set_state(BLE_STOPPING_SCAN);
        if (!already_stopping) gap_stop_scan();
        if (!flag(F_SCAN_CONFIRMED)) {
            // No movement or new radio operation is allowed after this fault.
            result_put(ST_CLOSING_SHIFT, PAYLOAD_FATAL);
            finish_attempt();
        }
    } else if (connection_handle != HCI_CON_HANDLE_INVALID) {
        set_state(BLE_CLOSING);
        if (gap_disconnect(connection_handle) != ERROR_CODE_SUCCESS) {
            result_put(ST_CLOSING_SHIFT, PAYLOAD_FATAL);
            // Hold the robot still; a connection may remain live.
        }
    } else if (current_state() == BLE_CONNECTING) {
        set_state(BLE_CLOSING);
        (void)gap_connect_cancel();
    } else {
        finish_attempt();
    }
}

static void transfer_failed(const char *reason)
{
    report_printf("AP1 PAYLOAD FAILED: %s\n", reason);
    fault_raise(FAULT_PAYLOAD);
    flag_assign(F_PAYLOAD_VERIFIED, false);
    close_attempt(flag(F_CLOSE_CONFIRMED) ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE);
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
    if (counters.robot_offset == length) {
        write_simple(MSG_FILE_END, BLE_WRITE_FILE_END);
        return;
    }
    uint16_t remaining = length - counters.robot_offset;
    uint8_t chunk = remaining > TRANSFER_DATA_BYTES ? TRANSFER_DATA_BYTES : (uint8_t)remaining;
    tx_packet[0] = MSG_FILE_DATA;
    tx_packet[1] = robot_sequence++;
    tx_packet[2] = chunk;
    memcpy(tx_packet + 3, robot_file + counters.robot_offset, chunk);
    counters.robot_offset += chunk;
    (void)write_packet(3 + chunk, BLE_WRITE_FILE_DATA);
}

static void handle_notification(const uint8_t *value, uint16_t length)
{
    if (!length || current_state() == BLE_CLOSING || current_state() == BLE_IDLE) return;
    if (value[0] == MSG_ERROR) { transfer_failed("beacon reported protocol error"); return; }
    if (writing_state(current_state())) {
        if (length > sizeof(pending_notification) || counters.pending_length) {
            transfer_failed("unexpected notification while write is pending");
            return;
        }
        memcpy(pending_notification, value, length);
        counters.pending_length = length;
        return;
    }
    switch (value[0]) {
    case MSG_HELLO_ACK:
        if (current_state() != BLE_WAIT_HELLO_ACK) return;
        if (length != 2 || value[1] != TARGET_BEACON_ID) {
            transfer_failed("wrong beacon identity"); return;
        }
        counters.robot_offset = 0;
        robot_sequence = 0;
        robot_crc = transfer_crc32(robot_file, sizeof(robot_file) - 1);
        tx_packet[0] = MSG_FILE_START;
        transfer_write_u16(tx_packet + 1, sizeof(robot_file) - 1);
        transfer_write_u32(tx_packet + 3, robot_crc);
        report_printf("AP1 handshake confirmed; sending robot test file\n");
        (void)write_packet(7, BLE_WRITE_FILE_START);
        break;
    case MSG_FILE_READY:
        if (current_state() != BLE_WAIT_FILE_READY) return;
        if (length != 2 || value[1] != TARGET_BEACON_ID) {
            transfer_failed("invalid FILE_READY"); return;
        }
        write_next_robot_chunk();
        break;
    case MSG_FILE_RECEIVED:
        if (current_state() != BLE_WAIT_FILE_RECEIVED) return;
        if (length != 6 || value[1] || transfer_read_u32(value + 2) != robot_crc) {
            transfer_failed("robot file CRC rejected"); return;
        }
        write_simple(MSG_REPLY_REQUEST, BLE_WRITE_REPLY_REQUEST);
        break;
    case MSG_REPLY_START:
        if (current_state() != BLE_WAIT_REPLY_START) return;
        if (length != 7) { transfer_failed("invalid reply metadata"); return; }
        // Check the 16-bit wire value before it is narrowed into the 9-bit field.
        uint16_t announced = transfer_read_u16(value + 1);
        if (!announced || announced > sizeof(reply_file)) {
            transfer_failed("reply payload length outside 1..256 bytes"); return;
        }
        counters.reply_length = announced;
        reply_crc = transfer_read_u32(value + 3);
        counters.reply_offset = 0;
        reply_sequence = 0;
        report_printf("AP1 reply announced: %u bytes, CRC %08lx\n",
                      counters.reply_length, (unsigned long)reply_crc);
        write_simple(MSG_REPLY_NEXT, BLE_WRITE_REPLY_NEXT);
        break;
    case MSG_REPLY_DATA: {
        if (current_state() != BLE_WAIT_REPLY_PIECE) return;
        if (length < 3) { transfer_failed("short reply chunk"); return; }
        uint8_t chunk = value[2];
        if (value[1] != reply_sequence || !chunk || chunk > TRANSFER_DATA_BYTES ||
            length != 3u + chunk || counters.reply_offset + chunk > counters.reply_length) {
            transfer_failed("invalid reply chunk sequence or length"); return;
        }
        memcpy(reply_file + counters.reply_offset, value + 3, chunk);
        counters.reply_offset += chunk;
        ++reply_sequence;
        write_simple(MSG_REPLY_NEXT, BLE_WRITE_REPLY_NEXT);
        break;
    }
    case MSG_REPLY_END:
        if (current_state() != BLE_WAIT_REPLY_PIECE) return;
        if (length != 5 || counters.reply_offset != counters.reply_length ||
            transfer_crc32(reply_file, counters.reply_length) != reply_crc ||
            transfer_read_u32(value + 1) != reply_crc) {
            transfer_failed("reply payload CRC or length failed"); return;
        }
        report_printf("AP1 reply payload length and CRC verified\n");
        write_simple(MSG_COMPLETE, BLE_WRITE_COMPLETE);
        break;
    case MSG_COMPLETE_ACK:
        if (current_state() != BLE_WAIT_COMPLETE_ACK) return;
        if (length != 2 || value[1] != TARGET_BEACON_ID) {
            transfer_failed("invalid final acknowledgement"); return;
        }
        flag_assign(F_PAYLOAD_VERIFIED, true);
        report_printf("AP1 PAYLOAD COMPLETE: verified reply and final acknowledgement\n");
        report_printf("AP1 payload: %.*s\n", counters.reply_length, (const char *)reply_file);
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
    if (type != HCI_EVENT_PACKET || flag(F_ABORT_REQUESTED)) return;
    if (current_state() == BLE_CLOSING || current_state() == BLE_IDLE) return;
    uint8_t event = hci_event_packet_get_type(packet);
    if (event == GATT_EVENT_NOTIFICATION) {
        if (gatt_event_notification_get_handle(packet) == connection_handle &&
            gatt_event_notification_get_value_handle(packet) == characteristic.value_handle) {
            handle_notification(gatt_event_notification_get_value(packet),
                                gatt_event_notification_get_value_length(packet));
        }
        return;
    }
    if (event == GATT_EVENT_SERVICE_QUERY_RESULT && current_state() == BLE_DISCOVER_SERVICE) {
        gatt_event_service_query_result_get_service(packet, &service);
        flag_assign(F_SERVICE_FOUND, true); return;
    }
    if (event == GATT_EVENT_CHARACTERISTIC_QUERY_RESULT && current_state() == BLE_DISCOVER_CHARACTERISTIC) {
        gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
        flag_assign(F_CHARACTERISTIC_FOUND, true); return;
    }
    if (event != GATT_EVENT_QUERY_COMPLETE ||
        gatt_event_query_complete_get_handle(packet) != connection_handle || !query_ok(packet)) return;
    uint8_t status = ERROR_CODE_SUCCESS;
    switch (current_state()) {
    case BLE_DISCOVER_SERVICE:
        if (!flag(F_SERVICE_FOUND)) { transfer_failed("transfer service missing"); return; }
        flag_assign(F_CHARACTERISTIC_FOUND, false);
        set_state(BLE_DISCOVER_CHARACTERISTIC);
        status = gatt_client_discover_characteristics_for_service_by_uuid16(
            gatt_handler, connection_handle, &service, BEACON_CHARACTERISTIC_UUID);
        break;
    case BLE_DISCOVER_CHARACTERISTIC:
        if (!flag(F_CHARACTERISTIC_FOUND)) { transfer_failed("transfer characteristic missing"); return; }
        gatt_client_listen_for_characteristic_value_updates(&listener, gatt_handler,
                                                          connection_handle, &characteristic);
        flag_assign(F_LISTENING, true);
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
    if (counters.pending_length && !writing_state(current_state())) {
        uint8_t value[TRANSFER_PACKET_SIZE];
        uint16_t length = counters.pending_length;
        memcpy(value, pending_notification, length);
        counters.pending_length = 0;
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
    for (int i = 0; i < RSSI_SAMPLE_COUNT; ++i)
        sorted[i] = rssi_decode(bitarray_get(rssi_samples, i, RSSI_BITS));
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
    if (flag(F_ABORT_REQUESTED) || !target_advertisement(packet)) return;
    int rssi = (int8_t)gap_event_advertising_report_get_rssi(packet);
    if (rssi >= 0 || rssi <= -127) return;
    bitarray_put(rssi_samples, counters.sample_index, RSSI_BITS, rssi_encode(rssi));
    counters.sample_index = (counters.sample_index + 1) % RSSI_SAMPLE_COUNT;
    if (counters.sample_count < RSSI_SAMPLE_COUNT) ++counters.sample_count;
    if (counters.sample_count < RSSI_SAMPLE_COUNT) return;
    int average = filtered_rssi();
    counters.close_count = average >= CLOSE_RSSI_DBM ? counters.close_count + 1 : 0;
    report_printf("AP1 BLE: filtered %d dBm, need >= %d, close %d/%d\n",
                  average, CLOSE_RSSI_DBM, counters.close_count, REQUIRED_CLOSE_AVERAGES);
    if (counters.close_count < REQUIRED_CLOSE_AVERAGES) return;
    flag_assign(F_CLOSE_CONFIRMED, true);
    gap_event_advertising_report_get_address(packet, beacon_address);
    beacon_address_type = gap_event_advertising_report_get_address_type(packet);
    set_state(BLE_STOPPING_FOR_CONNECT);
    gap_stop_scan();
    report_printf("AP1 close-range check passed; motors stopped for payload transfer\n");
}

static void connection_complete(uint8_t status, hci_con_handle_t handle)
{
    if (current_state() != BLE_CONNECTING && current_state() != BLE_CLOSING) return;
    if (status != ERROR_CODE_SUCCESS) {
        connection_handle = HCI_CON_HANDLE_INVALID;
        if (current_state() == BLE_CLOSING) finish_attempt();
        else {
            // The failed connection procedure has already ended.
            set_state(BLE_IDLE);
            transfer_failed("BLE connection failed");
        }
        return;
    }
    connection_handle = handle;
    if (flag(F_ABORT_REQUESTED) || current_state() == BLE_CLOSING) {
        set_state(BLE_CLOSING);
        (void)gap_disconnect(handle);
        return;
    }
    flag_assign(F_SERVICE_FOUND, false);
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
        flag_assign(F_BLUETOOTH_READY, btstack_event_state_get_state(packet) == HCI_STATE_WORKING);
        if (flag(F_BLUETOOTH_READY)) report_printf("Bluetooth ready\n");
        break;
    case HCI_EVENT_COMMAND_COMPLETE:
        if (hci_event_command_complete_get_command_opcode(packet) != hci_le_set_scan_enable.opcode) break;
        if (hci_event_command_complete_get_return_parameters(packet)[0] != ERROR_CODE_SUCCESS) {
            report_printf("BLE scan command failed; stopping approach\n");
            fault_raise(FAULT_BLE_RADIO);
            result_put(ST_CLOSING_SHIFT, PAYLOAD_FATAL);
            finish_attempt();
        } else if (current_state() == BLE_SCANNING) {
            flag_assign(F_SCAN_CONFIRMED, true);
        } else if (current_state() == BLE_STOPPING_FOR_CONNECT) {
            if (flag(F_ABORT_REQUESTED)) {
                result_put(ST_CLOSING_SHIFT, PAYLOAD_FATAL);
                finish_attempt();
            } else {
                set_state(BLE_CONNECTING);
                if (gap_connect(beacon_address, beacon_address_type) != ERROR_CODE_SUCCESS) {
                    set_state(BLE_IDLE);
                    transfer_failed("BLE connection could not start");
                }
            }
        } else if (current_state() == BLE_STOPPING_SCAN) {
            finish_attempt();
        }
        break;
    case GAP_EVENT_ADVERTISING_REPORT:
        if (current_state() == BLE_SCANNING) advertisement_received(packet);
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
        if (current_state() != BLE_CLOSING) {
            flag_assign(F_PAYLOAD_VERIFIED, false);
            result_put(ST_CLOSING_SHIFT, flag(F_CLOSE_CONFIRMED) ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE);
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
    if (flag(F_ABORT_REQUESTED)) { result_put(ST_CLOSING_SHIFT, PAYLOAD_FATAL); finish_attempt(); return; }
    flag_assign(F_CLOSE_CONFIRMED, false);
    flag_assign(F_SCAN_CONFIRMED, false);
    flag_assign(F_PAYLOAD_VERIFIED, false);
    fault_clear(FAULT_PAYLOAD);
    counters.pending_length = 0;
    counters.reply_length = counters.reply_offset = 0;
    counters.sample_count = counters.sample_index = counters.close_count = 0;
    memset(rssi_samples, 0, sizeof(rssi_samples));
    set_state(BLE_SCANNING);
    gap_set_scan_parameters(0, 0x0030, 0x0030);
    gap_start_scan();
    report_printf("BLE-only check: AP1, 12 readings, 3 averages >= -50 dBm; Wi-Fi paused\n");
}

static void abort_attempt_callback(void *context)
{
    (void)context;
    if (!flag(F_ATTEMPT_RUNNING) || current_state() == BLE_STOPPING_SCAN || current_state() == BLE_CLOSING) return;
    report_printf("BLE phase timed out or GP21 pressed; closing radio operation\n");
    close_attempt(motion_stop_requested() ? PAYLOAD_FATAL :
                  (flag(F_CLOSE_CONFIRMED) ? PAYLOAD_RETRY : PAYLOAD_NOT_CLOSE));
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

bool payload_client_ready(void) { return flag(F_BLUETOOTH_READY); }

static void (*close_callback)(void);
void payload_client_set_close_callback(void (*callback)(void)) { close_callback = callback; }

payload_result_t payload_client_attempt(void)
{
    motion_stop();
    if (!flag(F_BLUETOOTH_READY) || flag(F_ATTEMPT_RUNNING) || motion_stop_requested()) return PAYLOAD_FATAL;
    flag_assign(F_ABORT_REQUESTED, false);
    // Cleared here too, so a stale flag from the last attempt cannot fire the
    // close callback before start_attempt_callback runs.
    flag_assign(F_CLOSE_CONFIRMED, false);
    flag_assign(F_ATTEMPT_RUNNING, true);
    set_state(BLE_STARTING);
    start_registration.callback = start_attempt_callback;
    btstack_run_loop_execute_on_main_thread(&start_registration);
    uint32_t start = now_ms();
    uint32_t abort_started = 0;
    bool close_announced = false;
    while (flag(F_ATTEMPT_RUNNING)) {
        if (!close_announced && flag(F_CLOSE_CONFIRMED)) {
            close_announced = true;
            if (close_callback) close_callback();
        }
        uint32_t elapsed = now_ms() - phase_started_ms;
        bool cleanup = current_state() == BLE_CLOSING || current_state() == BLE_STOPPING_SCAN;
        if (cleanup && elapsed >= CLEANUP_TIMEOUT_MS) {
            flag_assign(F_ABORT_REQUESTED, true);
            fault_raise(FAULT_BLE_RADIO);
            report_printf("FAILED: BLE cleanup did not finish; no further movement\n");
            return PAYLOAD_FATAL;
        }
        uint32_t timeout = current_state() == BLE_SCANNING ? SCAN_WINDOW_MS :
                           cleanup ? CLEANUP_TIMEOUT_MS : PHASE_TIMEOUT_MS;
        if (!flag(F_ABORT_REQUESTED) && (motion_stop_requested() || elapsed >= timeout ||
                                  (uint32_t)(now_ms() - start) >= ATTEMPT_TIMEOUT_MS)) {
            flag_assign(F_ABORT_REQUESTED, true);
            abort_started = now_ms();
            abort_registration.callback = abort_attempt_callback;
            btstack_run_loop_execute_on_main_thread(&abort_registration);
        }
        if (flag(F_ABORT_REQUESTED) && (uint32_t)(now_ms() - abort_started) >= CLEANUP_TIMEOUT_MS) {
            fault_raise(FAULT_BLE_RADIO);
            report_printf("FAILED: BLE cleanup did not finish; no further movement\n");
            return PAYLOAD_FATAL;
        }
        sleep_ms(10);
    }
    return motion_stop_requested() ? PAYLOAD_FATAL : result_get(ST_RESULT_SHIFT);
}

const uint8_t *payload_client_data(uint16_t *length)
{
    *length = flag(F_PAYLOAD_VERIFIED) ? counters.reply_length : 0;
    return flag(F_PAYLOAD_VERIFIED) ? reply_file : NULL;
}
