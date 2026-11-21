#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
static bool callback_active;
static int foreground_printf(const char *format, ...)
{ assert(!callback_active); (void)format; return 0; }
bool stdio_usb_connected(void) { assert(!callback_active); return false; }

// file_transfer.h is generated from ble_transfer/file_transfer.gatt by the
// SDK's lib/btstack/tool/compile_gatt.py and supplied on the include path.
#ifndef BEACON_ID
#define BEACON_ID 1
#endif
#define WIFI_SSID "host-test"
#define main beacon_firmware_main
#define printf foreground_printf
#include "../../ble_transfer_powerbank/beacon_server.c"
#undef printf
#undef main

static unsigned send_requests, notify_calls, completion_cues;
static uint8_t notify_error, sent[TRANSFER_PACKET_SIZE];
static uint16_t sent_length;

void hci_dump_log(int level, const char *format, ...) { (void)level; (void)format; }
void buzzer_play(const buzzer_note_t *notes, size_t count, bool repeat)
{ (void)count; (void)repeat; if (notes == cue_complete) ++completion_cues; }
void buzzer_stop(void) { }
void gap_advertisements_set_params(uint16_t minimum, uint16_t maximum,
                                  uint8_t type, uint8_t address_type,
                                  bd_addr_t address, uint8_t channels, uint8_t policy)
{
    (void)minimum; (void)maximum; (void)type; (void)address_type;
    (void)address; (void)channels; (void)policy;
}
void gap_advertisements_set_data(uint8_t length, uint8_t *data)
{ (void)length; (void)data; }
void gap_advertisements_enable(int enabled) { (void)enabled; }
void att_server_request_can_send_now_event(hci_con_handle_t handle)
{ assert(handle == 42); ++send_requests; }
uint8_t att_server_notify(hci_con_handle_t handle, uint16_t attribute,
                         const uint8_t *data, uint16_t length)
{
    assert(handle == 42 && attribute == ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE);
    assert(length <= sizeof(sent));
    ++notify_calls;
    if (!notify_error) { memcpy(sent, data, length); sent_length = length; }
    return notify_error;
}

static int write_value(uint8_t *data, uint16_t length)
{
    callback_active = true;
    int result = att_write_callback(42, ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE,
                                   ATT_TRANSACTION_MODE_NONE, 0, data, length);
    callback_active = false;
    return result;
}
static void can_send(void)
{
    uint8_t event[] = {ATT_EVENT_CAN_SEND_NOW, 2, 42, 0};
    callback_active = true;
    packet_handler(HCI_EVENT_PACKET, 0, event, sizeof(event));
    callback_active = false;
}
static void reset(void)
{
    beacon_log_init();
    reset_transfer();
    memset(received_file, 0xa5, sizeof(received_file));
    connection_handle = 42;
    notifications_enabled = true;
    pending_notification_length = 0;
    beacon_status = BEACON_STATUS_READY;
    notify_error = 0;
    send_requests = notify_calls = completion_cues = sent_length = 0;
}
static void hello(void)
{
    uint8_t command[] = {MSG_HELLO, BEACON_PROTOCOL_VERSION, BEACON_ID};
    assert(write_value(command, sizeof(command)) == 0);
    can_send();
    assert(sent_length == 2 && sent[0] == MSG_HELLO_ACK && sent[1] == BEACON_ID);
}
static void start_file(uint16_t length, uint32_t crc)
{
    uint8_t command[7] = {MSG_FILE_START};
    transfer_write_u16(command + 1, length);
    transfer_write_u32(command + 3, crc);
    assert(write_value(command, sizeof(command)) == 0);
    can_send();
}
static void simple(uint8_t command)
{
    assert(write_value(&command, 1) == 0);
    can_send();
}

int main(void)
{
    // Rejected 65535-byte metadata followed by 17-byte chunks used to write
    // beyond received_file on chunk 16. A rejection must invalidate the stream.
    reset(); hello(); start_file(UINT16_MAX, 0);
    assert(sent[0] == MSG_ERROR && expected_file_length == 0);
    uint8_t data[TRANSFER_PACKET_SIZE] = {MSG_FILE_DATA, 0, TRANSFER_DATA_BYTES};
    memset(data + 3, 'x', TRANSFER_DATA_BYTES);
    for (unsigned i = 0; i < 20; ++i) {
        data[1] = (uint8_t)i;
        assert(write_value(data, sizeof(data)) == 0);
        can_send();
        assert(sent[0] == MSG_ERROR && received_file_length == 0);
    }
    for (unsigned i = 0; i < sizeof(received_file); ++i) assert(received_file[i] == 0xa5);
    reset(); hello(); start_file(0, 0);
    assert(sent[0] == MSG_ERROR && expected_file_length == 0);

    // Prepared/cancelled writes and nonzero offsets must have no side effects,
    // including writes of the notification configuration descriptor.
    reset();
    uint8_t greeting[] = {MSG_HELLO, BEACON_PROTOCOL_VERSION, BEACON_ID};
    assert(att_write_callback(42, ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE,
        ATT_TRANSACTION_MODE_ACTIVE, 0, greeting, sizeof(greeting)) == ATT_ERROR_REQUEST_NOT_SUPPORTED);
    assert(att_write_callback(42, 0, ATT_TRANSACTION_MODE_CANCEL, 0, NULL, 0) != 0);
    assert(beacon_status == BEACON_STATUS_READY && !send_requests);
    assert(att_write_callback(42, ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE,
        ATT_TRANSACTION_MODE_NONE, 1, greeting, sizeof(greeting)) == ATT_ERROR_INVALID_OFFSET);
    uint8_t disable[] = {0, 0};
    assert(att_write_callback(42, ATT_CHARACTERISTIC_0xFF21_01_CLIENT_CONFIGURATION_HANDLE,
        ATT_TRANSACTION_MODE_ACTIVE, 0, disable, sizeof(disable)) == ATT_ERROR_REQUEST_NOT_SUPPORTED);
    assert(notifications_enabled && !send_requests);

    // Pipelining a second command cannot overwrite an unsent HELLO_ACK or
    // advance the transfer. Transport backpressure retains the exact reply.
    reset();
    assert(write_value(greeting, sizeof(greeting)) == 0);
    uint8_t metadata[7] = {MSG_FILE_START, 1};
    assert(write_value(metadata, sizeof(metadata)) == ATT_ERROR_INSUFFICIENT_RESOURCES);
    assert(beacon_status == BEACON_STATUS_HANDSHAKE && expected_file_length == 0);
    notify_error = BTSTACK_ACL_BUFFERS_FULL;
    can_send();
    assert(pending_notification_length == 2 && pending_notification[0] == MSG_HELLO_ACK);
    assert(send_requests == 2 && notify_calls == 1 && !sent_length);
    notify_error = 0; can_send();
    assert(sent[0] == MSG_HELLO_ACK && !pending_notification_length);

    // A COMPLETE command on a fresh link cannot acknowledge a nonexistent
    // transfer, and reply download is not allowed after a bad upload CRC.
    reset(); simple(MSG_COMPLETE);
    assert(sent[0] == MSG_ERROR && !completion_cues);
    reset(); hello(); start_file(1, 0);
    data[1] = 0; data[2] = 1;
    assert(write_value(data, 4) == 0);
    simple(MSG_FILE_END);
    assert(sent[0] == MSG_FILE_RECEIVED && sent[1] == 1);
    simple(MSG_REPLY_REQUEST);
    assert(sent[0] == MSG_ERROR);

    // Valid maximum-size upload, complete reply and final acknowledgement.
    // Reuse the same connection after an error via a new HELLO.
    hello();
    uint8_t file[TRANSFER_MAX_FILE_SIZE];
    for (unsigned i = 0; i < sizeof(file); ++i) file[i] = (uint8_t)('A' + i % 26);
    start_file(sizeof(file), transfer_crc32(file, sizeof(file)));
    assert(sent[0] == MSG_FILE_READY);
    uint16_t offset = 0;
    uint8_t sequence = 0;
    while (offset < sizeof(file)) {
        uint8_t chunk = sizeof(file) - offset > TRANSFER_DATA_BYTES
                      ? TRANSFER_DATA_BYTES : (uint8_t)(sizeof(file) - offset);
        data[1] = sequence++; data[2] = chunk;
        memcpy(data + 3, file + offset, chunk);
        assert(write_value(data, 3 + chunk) == 0);
        offset += chunk;
    }
    assert(received_file_length == sizeof(file) && !memcmp(received_file, file, sizeof(file)));
    simple(MSG_FILE_END);
    assert(sent[0] == MSG_FILE_RECEIVED && sent[1] == 0);
    simple(MSG_REPLY_REQUEST);
    assert(sent[0] == MSG_REPLY_START);
    uint16_t announced = transfer_read_u16(sent + 1);
    uint32_t crc = transfer_read_u32(sent + 3);
    uint8_t reply[TRANSFER_MAX_FILE_SIZE];
    offset = 0; sequence = 0;
    for (;;) {
        simple(MSG_REPLY_NEXT);
        if (sent[0] == MSG_REPLY_END) {
            assert(offset == announced && transfer_read_u32(sent + 1) == crc);
            assert(transfer_crc32(reply, offset) == crc);
            break;
        }
        assert(sent[0] == MSG_REPLY_DATA && sent[1] == sequence++);
        assert(sent[2] > 0 && offset + sent[2] <= announced && announced <= sizeof(reply));
        memcpy(reply + offset, sent + 3, sent[2]); offset += sent[2];
    }
    uint8_t complete = MSG_COMPLETE;
    assert(write_value(&complete, 1) == 0);
    assert(!completion_cues && beacon_status == BEACON_STATUS_SENDING_REPLY);
    notify_error = BTSTACK_ACL_BUFFERS_FULL; can_send();
    assert(!completion_cues && pending_notification[0] == MSG_COMPLETE_ACK);
    notify_error = 0; can_send();
    assert(sent[0] == MSG_COMPLETE_ACK && sent[1] == BEACON_ID && completion_cues == 1);
    puts("Beacon: rejected metadata, bounds, prepared writes, reply backpressure, phase guards and 256-byte exchange passed");
    return 0;
}
