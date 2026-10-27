#include <assert.h>
#include <stdarg.h>
#include "../payload_client.c"

static uint64_t sim_time;
static bool stopped;
static bool scan_ack_pending;
static bool auto_scan_ack;
static int stop_scan_calls;
static int connect_calls;
static int cancel_calls;
static int disconnect_calls;
static int write_calls;
static uint8_t write_value[20];
static uint16_t write_length;
static uint8_t write_status;
static uint8_t connect_status;
static int power_error, power_calls;
static bool defer_context_callback;
static btstack_context_callback_registration_t *queued_context_callback;
const hci_cmd_t hci_le_set_scan_enable = {0x200c, "11"};

static void scan_ack(void)
{
    uint8_t packet[] = {HCI_EVENT_COMMAND_COMPLETE, 4, 1, 0x0c, 0x20, 0};
    scan_ack_pending = false;
    hci_handler(HCI_EVENT_PACKET, 0, packet, sizeof(packet));
}
uint64_t time_us_64(void) { return sim_time; }
void sleep_ms(uint32_t ms)
{
    sim_time += (uint64_t)ms * 1000;
    if (auto_scan_ack && scan_ack_pending) scan_ack();
}
void report_printf(const char *format, ...) { (void)format; }
void report_defer_printf(const char *format, ...) { (void)format; }
void report_flush_usb(void) { }
void hci_dump_log(int level, const char *format, ...) { (void)level; (void)format; }
bool motion_stop_requested(void) { return stopped; }
void motion_stop(void) { }
void l2cap_init(void) { }
void sm_init(void) { }
void sm_set_io_capabilities(io_capability_t value) { (void)value; }
void att_server_init(const uint8_t *db, att_read_callback_t read, att_write_callback_t write)
{ (void)db; (void)read; (void)write; }
void gatt_client_init(void) { }
void hci_add_event_handler(btstack_packet_callback_registration_t *registration) { (void)registration; }
int hci_power_control(HCI_POWER_MODE mode) { (void)mode; ++power_calls; return power_error; }
void btstack_run_loop_execute_on_main_thread(btstack_context_callback_registration_t *registration)
{
    if (defer_context_callback) queued_context_callback = registration;
    else registration->callback(registration->context);
}
void gap_set_scan_parameters(uint8_t type, uint16_t interval, uint16_t window)
{ (void)type; (void)interval; (void)window; }
void gap_start_scan(void) { scan_ack_pending = true; }
void gap_stop_scan(void) { ++stop_scan_calls; scan_ack_pending = true; }
uint8_t gap_connect(const bd_addr_t address, bd_addr_type_t type)
{ (void)address; (void)type; ++connect_calls; return connect_status; }
uint8_t gap_connect_cancel(void) { ++cancel_calls; return 0; }
uint8_t gap_disconnect(hci_con_handle_t handle)
{
    ++disconnect_calls;
    uint8_t packet[] = {HCI_EVENT_DISCONNECTION_COMPLETE, 4, 0, (uint8_t)handle,
                        (uint8_t)(handle >> 8), 0x13};
    hci_handler(HCI_EVENT_PACKET, 0, packet, sizeof(packet));
    return 0;
}
uint8_t gatt_client_write_value_of_characteristic(btstack_packet_handler_t callback,
        hci_con_handle_t handle, uint16_t value_handle, uint16_t length, uint8_t *value)
{
    (void)callback; (void)handle; (void)value_handle;
    assert(length <= 20);
    ++write_calls; write_length = length;
    memcpy(write_value, value, length);
    return write_status;
}
uint8_t gatt_client_discover_primary_services_by_uuid16(btstack_packet_handler_t callback,
        hci_con_handle_t handle, uint16_t uuid)
{ (void)callback; (void)handle; (void)uuid; return 0; }
uint8_t gatt_client_discover_characteristics_for_service_by_uuid16(btstack_packet_handler_t callback,
        hci_con_handle_t handle, gatt_client_service_t *svc, uint16_t uuid)
{ (void)callback; (void)handle; (void)svc; (void)uuid; return 0; }
uint8_t gatt_client_write_client_characteristic_configuration(btstack_packet_handler_t callback,
        hci_con_handle_t handle, gatt_client_characteristic_t *chr, uint16_t configuration)
{ (void)callback; (void)handle; (void)chr; (void)configuration; return 0; }
void gatt_client_listen_for_characteristic_value_updates(gatt_client_notification_t *notification,
        btstack_packet_handler_t callback, hci_con_handle_t handle, gatt_client_characteristic_t *chr)
{ (void)notification; (void)callback; (void)handle; (void)chr; }
void gatt_client_stop_listening_for_characteristic_value_updates(gatt_client_notification_t *notification)
{ (void)notification; }
void gatt_client_deserialize_service(const uint8_t *packet, int offset, gatt_client_service_t *svc)
{ (void)packet; (void)offset; *svc = (gatt_client_service_t){.uuid16 = BEACON_SERVICE_UUID}; }
void gatt_client_deserialize_characteristic(const uint8_t *packet, int offset,
                                          gatt_client_characteristic_t *chr)
{ (void)packet; (void)offset; *chr = (gatt_client_characteristic_t){.value_handle = 7}; }

static void reset(void)
{
    sim_time = 0; stopped = scan_ack_pending = auto_scan_ack = false;
    stop_scan_calls = connect_calls = cancel_calls = disconnect_calls = write_calls = 0;
    write_status = connect_status = 0;
    power_error = power_calls = 0; defer_context_callback = false; queued_context_callback = NULL;
    flag_assign(F_STARTUP_FAILED, false);
    flag_assign(F_ATTEMPT_RUNNING, false); flag_assign(F_ABORT_REQUESTED, false); flag_assign(F_BLUETOOTH_READY, true);
    set_state(BLE_IDLE); phase_started_ms = 0;
    flag_assign(F_CLOSE_CONFIRMED, false); flag_assign(F_SCAN_CONFIRMED, false); flag_assign(F_PAYLOAD_VERIFIED, false); flag_assign(F_LISTENING, false);
    connection_handle = HCI_CON_HANDLE_INVALID;
    counters.sample_count = counters.sample_index = counters.close_count = 0;
    counters.pending_length = counters.reply_length = counters.reply_offset = 0;
    memset(rssi_samples, 0, sizeof(rssi_samples));
}
static void notification(const uint8_t *value, uint16_t length)
{
#ifdef TEST_EXTENDED_GATT_EVENTS
    const unsigned extra = 4;
#else
    const unsigned extra = 0;
#endif
    uint8_t packet[32] = {GATT_EVENT_NOTIFICATION, (uint8_t)(length + 6 + extra)};
    transfer_write_u16(packet + 2, connection_handle);
    transfer_write_u16(packet + 4 + extra, characteristic.value_handle);
    transfer_write_u16(packet + 6 + extra, length);
    memcpy(packet + 8 + extra, value, length);
    gatt_handler(HCI_EVENT_PACKET, 0, packet, 8 + extra + length);
}
static void query_complete(uint8_t status)
{
#ifdef TEST_EXTENDED_GATT_EVENTS
    uint8_t packet[] = {GATT_EVENT_QUERY_COMPLETE, 7, (uint8_t)connection_handle,
                        (uint8_t)(connection_handle >> 8), 0, 0, 0, 0, status};
#else
    uint8_t packet[] = {GATT_EVENT_QUERY_COMPLETE, 3, (uint8_t)connection_handle,
                        (uint8_t)(connection_handle >> 8), status};
#endif
    gatt_handler(HCI_EVENT_PACKET, 0, packet, sizeof(packet));
}
static void advertisement(int rssi, uint8_t id)
{
    uint8_t packet[18] = {GAP_EVENT_ADVERTISING_REPORT, 16, 0, 0, 1, 2, 3, 4, 5, 6,
                          (uint8_t)rssi, 6, 5, BLUETOOTH_DATA_TYPE_SERVICE_DATA,
                          0x20, 0xff, BEACON_PROTOCOL_VERSION, id};
    hci_handler(HCI_EVENT_PACKET, 0, packet, sizeof(packet));
}
static void start_connected(ble_state_t state)
{
    reset(); flag_assign(F_ATTEMPT_RUNNING, true); flag_assign(F_CLOSE_CONFIRMED, true);
    connection_handle = 42; characteristic.value_handle = 7; set_state(state);
}

static void successful_exchange(bool early_notifications)
{
    static const uint8_t peer_reply[] = "Reply file from beacon AP1";
    uint8_t received_robot[96] = {0};
    uint16_t received_length = 0, announced_length = 0, peer_offset = 0;
    uint32_t announced_crc = 0;
    uint8_t peer_sequence = 0;
    start_connected(BLE_ENABLE_NOTIFICATIONS);
    query_complete(0);
    for (int iterations = 0; flag(F_ATTEMPT_RUNNING); ++iterations) {
        assert(iterations < 30);
        uint8_t request[20];
        memcpy(request, write_value, write_length);
        uint8_t response[20] = {0}; uint16_t length = 0;
        switch (request[0]) {
        case MSG_HELLO:
            assert(request[1] == 1 && request[2] == 1);
            response[0] = MSG_HELLO_ACK; response[1] = 1; length = 2; break;
        case MSG_FILE_START:
            announced_length = transfer_read_u16(request + 1);
            announced_crc = transfer_read_u32(request + 3);
            response[0] = MSG_FILE_READY; response[1] = 1; length = 2; break;
        case MSG_FILE_DATA:
            memcpy(received_robot + received_length, request + 3, request[2]);
            received_length += request[2]; break;
        case MSG_FILE_END:
            assert(received_length == announced_length);
            assert(transfer_crc32(received_robot, received_length) == announced_crc);
            response[0] = MSG_FILE_RECEIVED;
            transfer_write_u32(response + 2, announced_crc); length = 6; break;
        case MSG_REPLY_REQUEST:
            response[0] = MSG_REPLY_START;
            transfer_write_u16(response + 1, sizeof(peer_reply) - 1);
            transfer_write_u32(response + 3, transfer_crc32(peer_reply, sizeof(peer_reply) - 1));
            length = 7; break;
        case MSG_REPLY_NEXT:
            if (peer_offset < sizeof(peer_reply) - 1) {
                uint16_t remaining = sizeof(peer_reply) - 1 - peer_offset;
                uint8_t chunk = remaining > 17 ? 17 : (uint8_t)remaining;
                response[0] = MSG_REPLY_DATA; response[1] = peer_sequence++; response[2] = chunk;
                memcpy(response + 3, peer_reply + peer_offset, chunk);
                peer_offset += chunk; length = 3 + chunk;
            } else {
                response[0] = MSG_REPLY_END;
                transfer_write_u32(response + 1, transfer_crc32(peer_reply, sizeof(peer_reply) - 1));
                length = 5;
            }
            break;
        case MSG_COMPLETE:
            assert(!flag(F_PAYLOAD_VERIFIED));
            response[0] = MSG_COMPLETE_ACK; response[1] = 1; length = 2; break;
        default: assert(false);
        }
        if (early_notifications && length) {
            notification(response, length);
            assert(counters.pending_length == length && !flag(F_PAYLOAD_VERIFIED));
        }
        query_complete(0);
        if (!early_notifications && length) notification(response, length);
    }
    assert(result_get(ST_RESULT_SHIFT) == PAYLOAD_COMPLETE && flag(F_PAYLOAD_VERIFIED) && disconnect_calls == 1);
    uint16_t length;
    const uint8_t *data = payload_client_data(&length);
    assert(length == sizeof(peer_reply) - 1 && memcmp(data, peer_reply, length) == 0);
}

int main(void)
{
    reset(); defer_context_callback=true; payload_client_init();
    assert(!payload_client_ready() && !payload_client_start_failed() && power_calls==0 && queued_context_callback);
    queued_context_callback->callback(queued_context_callback->context);
    assert(power_calls==1 && !payload_client_start_failed());
    uint8_t working[] = {BTSTACK_EVENT_STATE,1,HCI_STATE_WORKING};
    hci_handler(HCI_EVENT_PACKET,0,working,sizeof(working));
    assert(payload_client_ready());
    reset(); power_error=-7; payload_client_init();
    assert(power_calls==1 && !payload_client_ready() && payload_client_start_failed() && fault_test(FAULT_BLE_RADIO));
    reset(); defer_context_callback=true; payload_client_init(); stopped=true;
    queued_context_callback->callback(queued_context_callback->context);
    assert(!power_calls && payload_client_start_failed());
    reset(); flag_assign(F_ATTEMPT_RUNNING, true); start_attempt_callback(NULL); scan_ack();
    for (int i = 0; i < 20; ++i) advertisement(-70, 1);
    assert(counters.sample_count == 12 && counters.close_count == 0 && !connect_calls && current_state() == BLE_SCANNING);
    for (int i = 0; i < 13; ++i) advertisement(-50, 1);
    assert(current_state() == BLE_STOPPING_FOR_CONNECT && !connect_calls);
    assert(flag(F_ATTEMPT_RUNNING) && stop_scan_calls == 1);
    scan_ack(); assert(current_state() == BLE_CONNECTING && connect_calls == 1);
    connection_complete(0x3e, HCI_CON_HANDLE_INVALID);
    assert(!flag(F_ATTEMPT_RUNNING) && result_get(ST_RESULT_SHIFT) == PAYLOAD_RETRY && !cancel_calls);

    reset(); flag_assign(F_ATTEMPT_RUNNING, true); start_attempt_callback(NULL); scan_ack();
    for (int i = 0; i < 20; ++i) advertisement(-40, 2);
    assert(!counters.sample_count && !connect_calls);
    advertisement(0, 1); advertisement(-127, 1); assert(!counters.sample_count);
    close_attempt(PAYLOAD_NOT_CLOSE);
    assert(flag(F_ATTEMPT_RUNNING)); scan_ack();
    assert(!flag(F_ATTEMPT_RUNNING) && result_get(ST_RESULT_SHIFT) == PAYLOAD_NOT_CLOSE);

    successful_exchange(false); successful_exchange(true);

    start_connected(BLE_WAIT_HELLO_ACK);
    const uint8_t wrong_identity[] = {MSG_HELLO_ACK, 2};
    notification(wrong_identity, sizeof(wrong_identity));
    assert(!flag(F_ATTEMPT_RUNNING) && result_get(ST_RESULT_SHIFT) == PAYLOAD_RETRY && !flag(F_PAYLOAD_VERIFIED));

    start_connected(BLE_WAIT_REPLY_START);
    uint8_t metadata[7] = {MSG_REPLY_START}; transfer_write_u16(metadata + 1, 257);
    notification(metadata, sizeof(metadata)); assert(!flag(F_ATTEMPT_RUNNING) && !flag(F_PAYLOAD_VERIFIED));
    start_connected(BLE_WAIT_REPLY_START); transfer_write_u16(metadata + 1, 0);
    notification(metadata, sizeof(metadata)); assert(!flag(F_ATTEMPT_RUNNING) && !flag(F_PAYLOAD_VERIFIED));
    // 513 would wrap to 1 if it were narrowed into the 9-bit field before the range check.
    start_connected(BLE_WAIT_REPLY_START); transfer_write_u16(metadata + 1, 513);
    notification(metadata, sizeof(metadata)); assert(!flag(F_ATTEMPT_RUNNING) && !counters.reply_length);
    assert(fault_test(FAULT_PAYLOAD));

    start_connected(BLE_WAIT_REPLY_PIECE); counters.reply_length = 4; reply_sequence = 0;
    const uint8_t bad_sequence[] = {MSG_REPLY_DATA, 1, 1, 'x'};
    notification(bad_sequence, sizeof(bad_sequence)); assert(!flag(F_ATTEMPT_RUNNING) && !flag(F_PAYLOAD_VERIFIED));
    start_connected(BLE_WAIT_REPLY_PIECE); counters.reply_length = counters.reply_offset = 1;
    reply_file[0] = 'x'; reply_crc = transfer_crc32(reply_file, 1);
    const uint8_t bad_crc[] = {MSG_REPLY_END, 0, 0, 0, 0};
    notification(bad_crc, sizeof(bad_crc)); assert(!flag(F_ATTEMPT_RUNNING) && !flag(F_PAYLOAD_VERIFIED));
    start_connected(BLE_WAIT_COMPLETE_ACK);
    const uint8_t bad_ack[] = {MSG_COMPLETE_ACK, 2};
    notification(bad_ack, sizeof(bad_ack)); assert(!flag(F_ATTEMPT_RUNNING) && !flag(F_PAYLOAD_VERIFIED));

    start_connected(BLE_WRITE_HELLO);
    const uint8_t early_ack[] = {MSG_HELLO_ACK, 1};
    notification(early_ack, sizeof(early_ack));
    query_complete(ATT_ERROR_UNLIKELY_ERROR);
    assert(!flag(F_ATTEMPT_RUNNING) && !flag(F_PAYLOAD_VERIFIED) && counters.pending_length == 0);

    start_connected(BLE_ENABLE_NOTIFICATIONS); write_status = ERROR_CODE_COMMAND_DISALLOWED;
    query_complete(0); assert(!flag(F_ATTEMPT_RUNNING) && !flag(F_PAYLOAD_VERIFIED));

    reset(); auto_scan_ack = true;
    assert(payload_client_attempt() == PAYLOAD_NOT_CLOSE);
    assert(!flag(F_ATTEMPT_RUNNING) && current_state() == BLE_IDLE && sim_time >= 5000000 && !connect_calls);
    reset(); stopped = true;
    assert(payload_client_attempt() == PAYLOAD_FATAL && !flag(F_ATTEMPT_RUNNING));
    reset(); flag_assign(F_BLUETOOTH_READY, false);
    assert(payload_client_attempt() == PAYLOAD_FATAL && !flag(F_ATTEMPT_RUNNING));

    start_connected(BLE_CLOSING); connection_handle = HCI_CON_HANDLE_INVALID;
    flag_assign(F_ABORT_REQUESTED, true); result_put(ST_CLOSING_SHIFT, PAYLOAD_RETRY);
    connection_complete(0, 42);
    assert(!flag(F_ATTEMPT_RUNNING) && disconnect_calls == 1 && result_get(ST_RESULT_SHIFT) == PAYLOAD_RETRY);

    // Packed RSSI buffer: every 7-bit slot round-trips without disturbing its neighbours.
    memset(rssi_samples, 0, sizeof(rssi_samples));
    for (int i = 0; i < RSSI_SAMPLE_COUNT; ++i)
        bitarray_put(rssi_samples, i, RSSI_BITS, rssi_encode(-1 - i * 10));
    for (int i = 0; i < RSSI_SAMPLE_COUNT; ++i)
        assert(rssi_decode(bitarray_get(rssi_samples, i, RSSI_BITS)) == -1 - i * 10);
    bitarray_put(rssi_samples, 5, RSSI_BITS, rssi_encode(-126));
    assert(rssi_decode(bitarray_get(rssi_samples, 4, RSSI_BITS)) == -41);
    assert(rssi_decode(bitarray_get(rssi_samples, 5, RSSI_BITS)) == -126);
    assert(rssi_decode(bitarray_get(rssi_samples, 6, RSSI_BITS)) == -61);
    assert(sizeof(rssi_samples) == 11);

    // Shared status word: updating one field or flag must leave the others alone.
    reset();
    result_put(ST_CLOSING_SHIFT, PAYLOAD_FATAL); flag_assign(F_LISTENING, true);
    set_state(BLE_CLOSING); result_put(ST_RESULT_SHIFT, PAYLOAD_RETRY);
    assert(current_state() == BLE_CLOSING && result_get(ST_RESULT_SHIFT) == PAYLOAD_RETRY);
    assert(result_get(ST_CLOSING_SHIFT) == PAYLOAD_FATAL && flag(F_LISTENING));
    flag_assign(F_LISTENING, false); assert(!flag(F_LISTENING) && current_state() == BLE_CLOSING);
    assert(BITS_NEEDED(BLE_CLOSING) == 5 && BITS_NEEDED(5) == 3 && BITS_NEEDED(1) == 1);

    // Fault mask: bits are independent and clear individually.
    for (int f = 0; f < FAULT_COUNT; ++f) fault_clear((fault_t)f);
    fault_raise(FAULT_IMU); fault_raise(FAULT_TRANSLATION_LIMIT);
    assert(fault_mask() == ((1u << FAULT_IMU) | (1u << FAULT_TRANSLATION_LIMIT)));
    fault_clear(FAULT_IMU); assert(fault_mask() == (1u << FAULT_TRANSLATION_LIMIT));
    fault_clear(FAULT_TRANSLATION_LIMIT); assert(!fault_mask());

    puts("BLE signed RSSI, proximity gate, payload CRC, early notifications, cleanup and faults: passed");
    return 0;
}
