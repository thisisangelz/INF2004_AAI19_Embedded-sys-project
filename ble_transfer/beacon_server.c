#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "file_transfer.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "transfer_protocol.h"

#ifndef BEACON_ID
#error "BEACON_ID must be defined as 1, 2 or 3"
#endif

#ifndef WIFI_SSID
#error "WIFI_SSID must be defined"
#endif

#define WIFI_PASSWORD "pico12345"
#define APP_AD_FLAGS 0x06

extern uint8_t const profile_data[];

static btstack_packet_callback_registration_t hci_event_registration;
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static bool notifications_enabled;
static uint8_t pending_notification[TRANSFER_PACKET_SIZE];
static uint16_t pending_notification_length;

static uint8_t received_file[TRANSFER_MAX_FILE_SIZE];
static uint16_t expected_file_length;
static uint16_t received_file_length;
static uint32_t expected_file_crc;
static uint8_t expected_sequence;

static uint8_t reply_file[64];
static uint16_t reply_length;
static uint16_t reply_offset;
static uint8_t reply_sequence;

static void queue_notification(const uint8_t *data, uint16_t length)
{
    if (!notifications_enabled || connection_handle == HCI_CON_HANDLE_INVALID) {
        return;
    }
    if (length > sizeof(pending_notification)) {
        length = sizeof(pending_notification);
    }
    memcpy(pending_notification, data, length);
    pending_notification_length = length;
    att_server_request_can_send_now_event(connection_handle);
}

static void send_simple(uint8_t type, uint8_t value)
{
    uint8_t packet[2] = {type, value};
    queue_notification(packet, sizeof(packet));
}

static void reset_transfer(void)
{
    expected_file_length = 0;
    received_file_length = 0;
    expected_file_crc = 0;
    expected_sequence = 0;
    reply_offset = 0;
    reply_sequence = 0;
}

static void send_reply_piece(void)
{
    uint8_t packet[TRANSFER_PACKET_SIZE] = {0};

    if (reply_offset == 0 && reply_sequence == 0) {
        packet[0] = MSG_REPLY_START;
        transfer_write_u16(&packet[1], reply_length);
        transfer_write_u32(&packet[3], transfer_crc32(reply_file, reply_length));
        reply_sequence = 1;
        queue_notification(packet, 7);
        return;
    }

    if (reply_offset < reply_length) {
        uint16_t remaining = reply_length - reply_offset;
        uint8_t chunk = remaining > TRANSFER_DATA_BYTES
                            ? TRANSFER_DATA_BYTES
                            : (uint8_t)remaining;
        packet[0] = MSG_REPLY_DATA;
        packet[1] = (uint8_t)(reply_sequence - 1);
        packet[2] = chunk;
        memcpy(&packet[3], &reply_file[reply_offset], chunk);
        reply_offset += chunk;
        reply_sequence++;
        queue_notification(packet, (uint16_t)(3 + chunk));
        return;
    }

    packet[0] = MSG_REPLY_END;
    transfer_write_u32(&packet[1], transfer_crc32(reply_file, reply_length));
    queue_notification(packet, 5);
}

static int att_write_callback(hci_con_handle_t con_handle,
                              uint16_t att_handle,
                              uint16_t transaction_mode,
                              uint16_t offset,
                              uint8_t *buffer,
                              uint16_t buffer_size)
{
    (void)transaction_mode;
    (void)offset;

    if (att_handle == ATT_CHARACTERISTIC_0xFF21_01_CLIENT_CONFIGURATION_HANDLE) {
        notifications_enabled =
            buffer_size >= 2 &&
            transfer_read_u16(buffer) ==
                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        connection_handle = con_handle;
        printf("BLE notifications %s\n",
               notifications_enabled ? "enabled" : "disabled");
        return 0;
    }

    if (att_handle != ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE ||
        buffer_size == 0) {
        return 0;
    }

    switch (buffer[0]) {
    case MSG_HELLO:
        if (buffer_size < 3 || buffer[1] != BEACON_PROTOCOL_VERSION ||
            buffer[2] != BEACON_ID) {
            send_simple(MSG_ERROR, 1);
            break;
        }
        reset_transfer();
        printf("Robot handshake accepted for AP%d\n", BEACON_ID);
        send_simple(MSG_HELLO_ACK, BEACON_ID);
        break;

    case MSG_FILE_START:
        if (buffer_size < 7) {
            send_simple(MSG_ERROR, 2);
            break;
        }
        expected_file_length = transfer_read_u16(&buffer[1]);
        expected_file_crc = transfer_read_u32(&buffer[3]);
        received_file_length = 0;
        expected_sequence = 0;
        if (expected_file_length > sizeof(received_file)) {
            send_simple(MSG_ERROR, 3);
            break;
        }
        printf("Receiving %u-byte test file (CRC %08lx)\n",
               expected_file_length, (unsigned long)expected_file_crc);
        send_simple(MSG_FILE_READY, BEACON_ID);
        break;

    case MSG_FILE_DATA: {
        if (buffer_size < 3) {
            send_simple(MSG_ERROR, 4);
            break;
        }
        uint8_t sequence = buffer[1];
        uint8_t chunk = buffer[2];
        if (sequence != expected_sequence ||
            chunk > TRANSFER_DATA_BYTES ||
            buffer_size != (uint16_t)(3 + chunk) ||
            received_file_length + chunk > expected_file_length) {
            send_simple(MSG_ERROR, 5);
            break;
        }
        memcpy(&received_file[received_file_length], &buffer[3], chunk);
        received_file_length += chunk;
        expected_sequence++;
        break;
    }

    case MSG_FILE_END: {
        uint32_t actual_crc =
            transfer_crc32(received_file, received_file_length);
        uint8_t response[6] = {MSG_FILE_RECEIVED, 0};
        bool valid = received_file_length == expected_file_length &&
                     actual_crc == expected_file_crc;
        response[1] = valid ? 0 : 1;
        transfer_write_u32(&response[2], actual_crc);
        printf("Robot file %s: %.*s\n", valid ? "verified" : "FAILED",
               valid ? received_file_length : 0, received_file);
        queue_notification(response, sizeof(response));
        break;
    }

    case MSG_REPLY_REQUEST:
        reply_length = (uint16_t)snprintf(
            (char *)reply_file, sizeof(reply_file),
            "Reply file from beacon AP%d", BEACON_ID);
        reply_offset = 0;
        reply_sequence = 0;
        send_reply_piece();
        break;

    case MSG_REPLY_NEXT:
        send_reply_piece();
        break;

    case MSG_COMPLETE:
        printf("TRANSFER COMPLETE with robot (AP%d)\n", BEACON_ID);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        send_simple(MSG_COMPLETE_ACK, BEACON_ID);
        break;

    default:
        send_simple(MSG_ERROR, 6);
        break;
    }

    return 0;
}

static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            uint8_t adv_data[] = {
                0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
                0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,
                (uint8_t)BEACON_SERVICE_UUID, (uint8_t)(BEACON_SERVICE_UUID >> 8),
                0x05, BLUETOOTH_DATA_TYPE_SERVICE_DATA, (uint8_t)BEACON_SERVICE_UUID,
                (uint8_t)(BEACON_SERVICE_UUID >> 8), BEACON_PROTOCOL_VERSION, BEACON_ID,
                0x0E, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
                'P', 'I', 'C', 'O', '-', 'B', 'E', 'A', 'C', 'O', 'N', '-',
                (uint8_t)('0' + BEACON_ID),
            };
            bd_addr_t null_addr = {0};
            gap_advertisements_set_params(
                160, 160, 0, 0, null_addr, 0x07, 0x00);
            gap_advertisements_set_data(sizeof(adv_data), adv_data);
            gap_advertisements_enable(1);
            printf("BLE beacon AP%d advertising\n", BEACON_ID);
        }
        break;

    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) ==
            GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
            connection_handle =
                gap_subevent_le_connection_complete_get_connection_handle(packet);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            printf("BLE robot connected\n");
        }
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        connection_handle = HCI_CON_HANDLE_INVALID;
        notifications_enabled = false;
        pending_notification_length = 0;
        reset_transfer();
        printf("BLE robot disconnected; ready for another transfer\n");
        break;

    case ATT_EVENT_CAN_SEND_NOW:
        if (pending_notification_length != 0) {
            att_server_notify(
                connection_handle,
                ATT_CHARACTERISTIC_0xFF21_01_VALUE_HANDLE,
                pending_notification,
                pending_notification_length);
            pending_notification_length = 0;
        }
        break;

    default:
        break;
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\nAP%d Wi-Fi + BLE transfer beacon\n", BEACON_ID);

    if (cyw43_arch_init()) {
        printf("ERROR: failed to initialise CYW43\n");
        return 1;
    }

    cyw43_arch_enable_ap_mode(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    printf("Wi-Fi AP started: %s\n", WIFI_SSID);

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    att_server_init(profile_data, NULL, att_write_callback);

    hci_event_registration.callback = packet_handler;
    hci_add_event_handler(&hci_event_registration);
    att_server_register_packet_handler(packet_handler);
    hci_power_control(HCI_POWER_ON);

    while (true) {
        sleep_ms(1000);
    }
}
