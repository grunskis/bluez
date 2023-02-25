// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2014  Google Inc.
 *
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#define _GNU_SOURCE
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <limits.h>
#include <errno.h>
#include <sys/time.h>

#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/l2cap.h"
#include "lib/uuid.h"

#include "src/shared/mainloop.h"
#include "src/shared/util.h"
#include "src/shared/att.h"
#include "src/shared/queue.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-client.h"

#define ATT_CID 4

static struct timeval tv;

#define LOG(...) \
  gettimeofday(&tv, NULL); printf("%ld.%ld ", tv.tv_sec, tv.tv_usec); printf(__VA_ARGS__);

#define COLOR_OFF	"\x1B[0m"
#define COLOR_RED	"\x1B[0;91m"
#define COLOR_GREEN	"\x1B[0;92m"
#define COLOR_YELLOW	"\x1B[0;93m"
#define COLOR_MAGENTA	"\x1B[0;95m"
#define COLOR_BOLDGRAY	"\x1B[1;30m"
#define COLOR_BOLDWHITE	"\x1B[1;37m"

#define NUIMO_CHAR_LED_HANDLE       0x0030
#define NUIMO_CHAR_TOUCH_HANDLE     0x0023
#define NUIMO_CHAR_BUTTON_HANDLE    0x001d
#define NUIMO_CHAR_GESTURE_HANDLE   0x0020
#define NUIMO_CHAR_ENCODER_HANDLE   0x0026
#define NUIMO_CHAR_HEARTBEAT_HANDLE 0x002b

#define NUIMO_CHAR_TOUCH_SWIPE_LEFT       0
#define NUIMO_CHAR_TOUCH_SWIPE_RIGHT      1
#define NUIMO_CHAR_TOUCH_SWIPE_UP         2
#define NUIMO_CHAR_TOUCH_SWIPE_DOWN       3
#define NUIMO_CHAR_TOUCH_TOUCH_LEFT       4
#define NUIMO_CHAR_TOUCH_TOUCH_RIGHT      5
#define NUIMO_CHAR_TOUCH_TOUCH_TOP        6
#define NUIMO_CHAR_TOUCH_TOUCH_BOTTOM     7
#define NUIMO_CHAR_TOUCH_LONGTOUCH_LEFT   8
#define NUIMO_CHAR_TOUCH_LONGTOUCH_RIGHT  9
#define NUIMO_CHAR_TOUCH_LONGTOUCH_TOP    10
#define NUIMO_CHAR_TOUCH_LONGTOUCH_BOTTOM 11

#define NUIMO_LED_MATRIX_BYTES 13

#define HEARTBEAT_INTERVAL_SEC 30

// just some random thing for now...
static const uint8_t led_icon[NUIMO_LED_MATRIX_BYTES] = {0x00, 0xd8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x0a };

static bool verbose = false;

struct client {
    int fd;
    struct bt_att *att;
    struct gatt_db *db;
    struct bt_gatt_client *gatt;

    unsigned int reliable_session_id;
};

struct chars_name_handle {
    const char *name;
    int handle;
};

void print_device_info(struct bt_gatt_client *gatt);
void print_battery_level(struct bt_gatt_client *gatt);

static void register_notify_cb(uint16_t att_ecode, void *user_data);
static void ready_cb(bool success, uint8_t att_ecode, void *user_data);
static void service_changed_cb(uint16_t start_handle, uint16_t end_handle,
                               void *user_data);
static const char *ecode_to_string(uint8_t ecode)
{
    switch (ecode) {
    case BT_ATT_ERROR_INVALID_HANDLE:
        return "Invalid Handle";
    case BT_ATT_ERROR_READ_NOT_PERMITTED:
        return "Read Not Permitted";
    case BT_ATT_ERROR_WRITE_NOT_PERMITTED:
        return "Write Not Permitted";
    case BT_ATT_ERROR_INVALID_PDU:
        return "Invalid PDU";
    case BT_ATT_ERROR_AUTHENTICATION:
        return "Authentication Required";
    case BT_ATT_ERROR_REQUEST_NOT_SUPPORTED:
        return "Request Not Supported";
    case BT_ATT_ERROR_INVALID_OFFSET:
        return "Invalid Offset";
    case BT_ATT_ERROR_AUTHORIZATION:
        return "Authorization Required";
    case BT_ATT_ERROR_PREPARE_QUEUE_FULL:
        return "Prepare Write Queue Full";
    case BT_ATT_ERROR_ATTRIBUTE_NOT_FOUND:
        return "Attribute Not Found";
    case BT_ATT_ERROR_ATTRIBUTE_NOT_LONG:
        return "Attribute Not Long";
    case BT_ATT_ERROR_INSUFFICIENT_ENCRYPTION_KEY_SIZE:
        return "Insuficient Encryption Key Size";
    case BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN:
        return "Invalid Attribute value len";
    case BT_ATT_ERROR_UNLIKELY:
        return "Unlikely Error";
    case BT_ATT_ERROR_INSUFFICIENT_ENCRYPTION:
        return "Insufficient Encryption";
    case BT_ATT_ERROR_UNSUPPORTED_GROUP_TYPE:
        return "Group type Not Supported";
    case BT_ATT_ERROR_INSUFFICIENT_RESOURCES:
        return "Insufficient Resources";
    case BT_ERROR_CCC_IMPROPERLY_CONFIGURED:
        return "CCC Improperly Configured";
    case BT_ERROR_ALREADY_IN_PROGRESS:
        return "Procedure Already in Progress";
    case BT_ERROR_OUT_OF_RANGE:
        return "Out of Range";
    default:
        return "Unknown error type";
    }
}

static void att_disconnect_cb(int err, void *user_data)
{
    printf("Device disconnected: %s\n", strerror(err));

    mainloop_quit();
}

static void att_debug_cb(const char *str, void *user_data)
{
    const char *prefix = user_data;

    LOG(COLOR_BOLDGRAY "%s" COLOR_BOLDWHITE "%s\n" COLOR_OFF, prefix, str);
}

static void gatt_debug_cb(const char *str, void *user_data)
{
    const char *prefix = user_data;

    LOG(COLOR_GREEN "%s%s\n" COLOR_OFF, prefix, str);
}

static void log_service_event(struct gatt_db_attribute *attr, const char *str)
{
    char uuid_str[MAX_LEN_UUID_STR];
    bt_uuid_t uuid;
    uint16_t start, end;

    gatt_db_attribute_get_service_uuid(attr, &uuid);
    bt_uuid_to_string(&uuid, uuid_str, sizeof(uuid_str));

    gatt_db_attribute_get_service_handles(attr, &start, &end);

    LOG("%s - UUID: %s start: 0x%04x end: 0x%04x\n", str, uuid_str,
        start, end);
}

static void service_added_cb(struct gatt_db_attribute *attr, void *user_data)
{
    log_service_event(attr, "Service Added");
}

static void service_removed_cb(struct gatt_db_attribute *attr, void *user_data)
{
    log_service_event(attr, "Service Removed");
}

static struct client *client_create(int fd, uint16_t mtu)
{
    struct client *cli;

    cli = new0(struct client, 1);
    if (!cli) {
        fprintf(stderr, "Failed to allocate memory for client\n");
        return NULL;
    }

    cli->att = bt_att_new(fd, false);
    if (!cli->att) {
        fprintf(stderr, "Failed to initialze ATT transport layer\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    if (!bt_att_set_close_on_unref(cli->att, true)) {
        fprintf(stderr, "Failed to set up ATT transport layer\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    if (!bt_att_register_disconnect(cli->att, att_disconnect_cb, NULL,
                                    NULL)) {
        fprintf(stderr, "Failed to set ATT disconnect handler\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    cli->fd = fd;
    cli->db = gatt_db_new();
    if (!cli->db) {
        fprintf(stderr, "Failed to create GATT database\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    cli->gatt = bt_gatt_client_new(cli->db, cli->att, mtu, 0);
    if (!cli->gatt) {
        fprintf(stderr, "Failed to create GATT client\n");
        gatt_db_unref(cli->db);
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    gatt_db_register(cli->db, service_added_cb, service_removed_cb,
                     NULL, NULL);

    if (verbose) {
        bt_att_set_debug(cli->att, BT_ATT_DEBUG_VERBOSE, att_debug_cb,
                         "att: ", NULL);
        bt_gatt_client_set_debug(cli->gatt, gatt_debug_cb, "gatt: ",
                                 NULL);
    }

    bt_gatt_client_ready_register(cli->gatt, ready_cb, cli, NULL);
    bt_gatt_client_set_service_changed(cli->gatt, service_changed_cb, cli,
                                       NULL);

    /* bt_gatt_client already holds a reference */
    gatt_db_unref(cli->db);

    return cli;
}

static void client_destroy(struct client *cli)
{
    bt_gatt_client_unref(cli->gatt);
    bt_att_unref(cli->att);
    free(cli);
}

static void print_uuid(const bt_uuid_t *uuid)
{
    char uuid_str[MAX_LEN_UUID_STR];
    bt_uuid_t uuid128;

    bt_uuid_to_uuid128(uuid, &uuid128);
    bt_uuid_to_string(&uuid128, uuid_str, sizeof(uuid_str));

    printf("%s\n", uuid_str);
}

static void print_incl(struct gatt_db_attribute *attr, void *user_data)
{
    struct client *cli = user_data;
    uint16_t handle, start, end;
    struct gatt_db_attribute *service;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_incl_data(attr, &handle, &start, &end))
        return;

    service = gatt_db_get_attribute(cli->db, start);
    if (!service)
        return;

    gatt_db_attribute_get_service_uuid(service, &uuid);

    printf("\t  " COLOR_GREEN "include" COLOR_OFF " - handle: "
           "0x%04x, - start: 0x%04x, end: 0x%04x,"
           "uuid: ", handle, start, end);
    print_uuid(&uuid);
}

static void print_desc(struct gatt_db_attribute *attr, void *user_data)
{
    printf("\t\t  " COLOR_MAGENTA "descr" COLOR_OFF
           " - handle: 0x%04x, uuid: ",
           gatt_db_attribute_get_handle(attr));
    print_uuid(gatt_db_attribute_get_type(attr));
}

static void print_chrc(struct gatt_db_attribute *attr, void *user_data)
{
    uint16_t handle, value_handle;
    uint8_t properties;
    uint16_t ext_prop;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_char_data(attr, &handle,
                                         &value_handle,
                                         &properties,
                                         &ext_prop,
                                         &uuid))
        return;

    printf("\t  " COLOR_YELLOW "charac" COLOR_OFF
           " - start: 0x%04x, value: 0x%04x, "
           "props: 0x%02x, ext_props: 0x%04x, uuid: ",
           handle, value_handle, properties, ext_prop);
    print_uuid(&uuid);

    gatt_db_service_foreach_desc(attr, print_desc, NULL);
}

static void print_service(struct gatt_db_attribute *attr, void *user_data)
{
    struct client *cli = user_data;
    uint16_t start, end;
    bool primary;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_service_data(attr, &start, &end, &primary,
                                            &uuid))
        return;

    printf(COLOR_RED "service" COLOR_OFF " - start: 0x%04x, "
           "end: 0x%04x, type: %s, uuid: ",
           start, end, primary ? "primary" : "secondary");
    print_uuid(&uuid);

    gatt_db_service_foreach_incl(attr, print_incl, cli);
    gatt_db_service_foreach_char(attr, print_chrc, NULL);

    printf("\n");
}

static void read_value_cb(bool success, uint8_t att_ecode, const uint8_t *value,
                          uint16_t length, void *user_data)
{
    int i;
    struct chars_name_handle *ch = user_data;

    if (!success) {
        LOG("\nRead request failed: %s (0x%02x)\n",
            ecode_to_string(att_ecode), att_ecode);
        return;
    }

    if (length == 0) {
        return;
    }

    printf("%s: ", ch->name);

    for (i = 0; i < length; i++)
        printf("%c", value[i]);

    printf("\n");
}

static void read_battery_level_cb(bool success, uint8_t att_ecode, const uint8_t *value,
                                  uint16_t length, void *user_data)
{
    struct chars_name_handle *ch = user_data;

    if (!success) {
        LOG("\nRead request failed: %s (0x%02x)\n", ecode_to_string(att_ecode), att_ecode);
        return;
    }

    if (length == 1)
        printf("%s: %d%%\n", ch->name, *value);
}


struct chars_name_handle device_info_chars[] = {
    {.handle = 0x000f, .name = "Manufacturer name"},
    {.handle = 0x0011, .name = "Model"},
    {.handle = 0x0013, .name = "Hardware revision"},
    {.handle = 0x0015, .name = "Firmware revision"},
};

void print_device_info(struct bt_gatt_client *gatt) {
    int num_handles = (int)(sizeof(device_info_chars) / sizeof(device_info_chars[0]));
    for (int i = 0; i < num_handles; i++) {
        struct chars_name_handle *user_data = &device_info_chars[i];
        if (!bt_gatt_client_read_value(gatt, user_data->handle, read_value_cb, user_data, NULL))
            printf("Failed to initiate read value procedure\n");
    }
}

static void write_led(struct bt_gatt_client *gatt, const uint8_t bytes[NUIMO_LED_MATRIX_BYTES]) {
    if (!bt_gatt_client_write_without_response(gatt,
            NUIMO_CHAR_LED_HANDLE,
            false,
            bytes,
            NUIMO_LED_MATRIX_BYTES))
        printf("Failed to initiate write without response procedure\n");
}

struct chars_name_handle battery_char = {.handle = 0x000b, .name = "Battery level"};

void print_battery_level(struct bt_gatt_client *gatt) {
    if (!bt_gatt_client_read_value(gatt, battery_char.handle, read_battery_level_cb, &battery_char,
                                   NULL))
        printf("Failed to initiate read value procedure\n");
}

static void write_cb(bool success, uint8_t att_ecode, void *user_data)
{
    if (!success) {
        LOG("\nWrite failed: %s (0x%02x)\n",
            ecode_to_string(att_ecode), att_ecode);
    }
}


static void button_press_cb(uint16_t value_handle, const uint8_t *value,
                            uint16_t length, void *user_data)
{
    if (length == 1) {
        LOG("BUTTON %d\n", *value);
    }
}

static void touch_cb(uint16_t value_handle, const uint8_t *value,
                     uint16_t length, void *user_data)
{
    struct bt_gatt_client *gatt = user_data;

    if (length == 1) {
        LOG("TOUCH %d\n", *value);
        write_led(gatt, led_icon);
    }
}

static void encoder_cb(uint16_t value_handle, const uint8_t *value,
                       uint16_t length, void *user_data)
{
    int16_t rotation;

    if (length == 2) {
        // TODO verify that this bit logic is correct
        rotation = value[0] + (value[1] << 8);
        if ((value[1] >> 7) > 0)
            rotation = rotation - (1 << 16);

        LOG("ENCODER %d\n", rotation);
    }
}

static void heartbeat_cb(uint16_t value_handle, const uint8_t *value,
                         uint16_t length, void *user_data)
{
    if (length == 1) {
        LOG("HEARTBEAT %d\n", *value);
    }
}


static void gesture_cb(uint16_t value_handle, const uint8_t *value,
                       uint16_t length, void *user_data)
{
    if (length == 2) {
        printf("GESTURE %d %d\n", value[0], value[1]);
    }
}


static void ready_cb(bool success, uint8_t att_ecode, void *user_data)
{
    unsigned int id;
    struct client *cli = user_data;
    uint8_t heartbeat_interval_sec = HEARTBEAT_INTERVAL_SEC;

    if (!success) {
        LOG("GATT discovery procedures failed - error code: 0x%02x\n",
            att_ecode);
        return;
    }

    print_device_info(cli->gatt);
    print_battery_level(cli->gatt);

    // button
    id = bt_gatt_client_register_notify(cli->gatt, NUIMO_CHAR_BUTTON_HANDLE,
                                        register_notify_cb,
                                        button_press_cb, cli->gatt, NULL);
    if (!id) {
        printf("Failed to register button press handler\n");
        return;
    }

    // touch
    id = bt_gatt_client_register_notify(cli->gatt, NUIMO_CHAR_TOUCH_HANDLE,
                                        register_notify_cb,
                                        touch_cb, cli->gatt, NULL);
    if (!id) {
        printf("Failed to register touch handler\n");
        return;
    }

    // encoder
    id = bt_gatt_client_register_notify(cli->gatt, NUIMO_CHAR_ENCODER_HANDLE,
                                        register_notify_cb,
                                        encoder_cb, cli->gatt, NULL);
    if (!id) {
        printf("Failed to register encoder handler\n");
        return;
    }

    // gesture FIXME couldn't verify that this works
    id = bt_gatt_client_register_notify(cli->gatt, NUIMO_CHAR_GESTURE_HANDLE,
                                        register_notify_cb,
                                        gesture_cb, cli->gatt, NULL);
    if (!id) {
        printf("Failed to register gesture handler\n");
        return;
    }

    // heartbeat, configure to receive every 30 seconds
    if (!bt_gatt_client_write_value(cli->gatt, NUIMO_CHAR_HEARTBEAT_HANDLE,
                                    &heartbeat_interval_sec, 1, write_cb,
                                    NULL, NULL))
        printf("Failed to initiate write procedure\n");

    id = bt_gatt_client_register_notify(cli->gatt, NUIMO_CHAR_HEARTBEAT_HANDLE,
                                        register_notify_cb, heartbeat_cb, cli->gatt, NULL);
    if (!id) {
        printf("Failed to register heartbeat handler\n");
        return;
    }
}

static void service_changed_cb(uint16_t start_handle, uint16_t end_handle,
                               void *user_data)
{
    struct client *cli = user_data;

    printf("\nService Changed handled - start: 0x%04x end: 0x%04x\n",
           start_handle, end_handle);

    gatt_db_foreach_service_in_range(cli->db, NULL, print_service, cli,
                                     start_handle, end_handle);
}

static void register_notify_cb(uint16_t att_ecode, void *user_data)
{
    if (att_ecode) {
        LOG("Failed to register notify handler "
            "- error code: 0x%02x\n", att_ecode);
        return;
    }
}

typedef void (*command_func_t)(struct client *cli, char *cmd_str);

static void signal_cb(int signum, void *user_data)
{
    switch (signum) {
    case SIGINT:
    case SIGTERM:
        mainloop_quit();
        break;
    default:
        LOG("signum: %d\n", signum);
        break;
    }
}

static int l2cap_le_att_connect(bdaddr_t *src, bdaddr_t *dst, uint8_t dst_type,
                                int sec)
{
    int sock;
    struct sockaddr_l2 srcaddr, dstaddr;
    struct bt_security btsec;

    if (verbose) {
        char srcaddr_str[18], dstaddr_str[18];

        ba2str(src, srcaddr_str);
        ba2str(dst, dstaddr_str);

        printf("nuimo-client: Opening L2CAP LE connection on ATT "
               "channel:\n\t src: %s\n\tdest: %s\n",
               srcaddr_str, dstaddr_str);
    }

    sock = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0) {
        perror("Failed to create L2CAP socket");
        return -1;
    }

    /* Set up source address */
    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.l2_family = AF_BLUETOOTH;
    srcaddr.l2_cid = htobs(ATT_CID);
    srcaddr.l2_bdaddr_type = 0;
    bacpy(&srcaddr.l2_bdaddr, src);

    if (bind(sock, (struct sockaddr *)&srcaddr, sizeof(srcaddr)) < 0) {
        perror("Failed to bind L2CAP socket");
        close(sock);
        return -1;
    }

    /* Set the security level */
    memset(&btsec, 0, sizeof(btsec));
    btsec.level = sec;
    if (setsockopt(sock, SOL_BLUETOOTH, BT_SECURITY, &btsec,
                   sizeof(btsec)) != 0) {
        fprintf(stderr, "Failed to set L2CAP security level\n");
        close(sock);
        return -1;
    }

    /* Set up destination address */
    memset(&dstaddr, 0, sizeof(dstaddr));
    dstaddr.l2_family = AF_BLUETOOTH;
    dstaddr.l2_cid = htobs(ATT_CID);
    dstaddr.l2_bdaddr_type = dst_type;
    bacpy(&dstaddr.l2_bdaddr, dst);

    printf("Connecting to device...");
    fflush(stdout);

    if (connect(sock, (struct sockaddr *) &dstaddr, sizeof(dstaddr)) < 0) {
        perror(" Failed to connect");
        close(sock);
        return -1;
    }

    printf(" Done\n");

    return sock;
}

static void usage(void)
{
    printf("nuimo-client\n");
    printf("Usage:\n\tnuimo-client [options]\n");

    printf("Options:\n"
           "\t-i, --index <id>\t\tSpecify adapter index, e.g. hci0\n"
           "\t-v, --verbose\t\t\tEnable extra logging\n"
           "\t-h, --help\t\t\tDisplay help\n");
}

static struct option main_options[] = {
    { "index",		1, 0, 'i' },
    { "verbose",	0, 0, 'v' },
    { "help",		0, 0, 'h' },
    { }
};

#define EIR_NAME_SHORT 0x08    /* shortened local name */
#define EIR_NAME_COMPLETE 0x09 /* complete local name */

static volatile int signal_received = 0;

static void eir_parse_name(uint8_t *eir, size_t eir_len, char *buf,
                           size_t buf_len) {
    size_t offset = 0;

    while (offset < eir_len) {
        uint8_t field_len = eir[0];
        size_t name_len;

        /* Check for the end of EIR */
        if (field_len == 0)
            break;

        if (offset + field_len > eir_len)
            return;

        switch (eir[1]) {
        case EIR_NAME_SHORT:
        case EIR_NAME_COMPLETE:
            name_len = field_len - 1;
            if (name_len > buf_len)
                return;

            memcpy(buf, &eir[2], name_len);
            return;
        }

        offset += field_len + 1;
        eir += field_len + 1;
    }
}

static void sigint_handler(int sig) {
    signal_received = sig;
}

static int find_nuimo_addr(int dd, bdaddr_t *nuimo_addr) {
    unsigned char buf[HCI_MAX_EVENT_SIZE], *ptr;
    struct hci_filter nf, of;
    struct sigaction sa;
    socklen_t olen;
    int len;
    char name[30];

    memset(name, 0, sizeof(name));

    olen = sizeof(of);
    if (getsockopt(dd, SOL_HCI, HCI_FILTER, &of, &olen) < 0) {
        printf("Could not get socket options\n");
        return -1;
    }

    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);

    if (setsockopt(dd, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        printf("Could not set socket options\n");
        return -1;
    }

    memset(&sa, 0, sizeof(sa));
    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = sigint_handler;
    sigaction(SIGINT, &sa, NULL);

    while (1) {
        evt_le_meta_event *meta;
        le_advertising_info *info;

        while ((len = read(dd, buf, sizeof(buf))) < 0) {
            if (errno == EINTR && signal_received == SIGINT) {
                len = 0;
                goto done;
            }

            if (errno == EAGAIN || errno == EINTR)
                continue;
            goto done;
        }

        ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        len -= (1 + HCI_EVENT_HDR_SIZE);

        meta = (void *)ptr;

        if (meta->subevent != 0x02)
            goto done;

        /* Ignoring multiple reports */
        info = (le_advertising_info *)(meta->data + 1);
        eir_parse_name(info->data, info->length, name, sizeof(name) - 1);
        if (strncmp(name, "Nuimo", 5) == 0) {
            bacpy(nuimo_addr, &info->bdaddr);
            goto done;
        }
    }

done:
    setsockopt(dd, SOL_HCI, HCI_FILTER, &of, sizeof(of));

    if (len < 0)
        return -1;

    return 0;
}

static int find_nuimo(bdaddr_t *nuimo_addr) {
    int err, dd;
    uint8_t own_type = LE_PUBLIC_ADDRESS;
    uint8_t scan_type = 0x01;
    uint8_t filter_policy = 0x00;
    uint16_t interval = htobs(0x0010);
    uint16_t window = htobs(0x0010);
    uint8_t filter_dup = 0x01;
    int dev_id;

    dev_id = hci_get_route(NULL);
    dd = hci_open_dev(dev_id);
    if (dd < 0) {
        perror("Could not open device");
        return 0;
    }

    err = hci_le_set_scan_parameters(dd, scan_type, interval, window, own_type,
                                     filter_policy, 10000);
    if (err < 0) {
        perror("Set scan parameters failed");
        return 0;
    }

    err = hci_le_set_scan_enable(dd, 0x01, filter_dup, 10000);
    if (err < 0) {
        perror("Enable scan failed");
        return 0;
    }

    printf("Searching for Nuimo... ");

    err = find_nuimo_addr(dd, nuimo_addr);
    if (err < 0) {
        perror("Could not find a Nuimo device");
        return 0;
    }

    err = hci_le_set_scan_enable(dd, 0x00, filter_dup, 10000);
    if (err < 0) {
        perror("Disable scan failed");
        return 0;
    }
    hci_close_dev(dd);

    return 1;
}


int main(int argc, char *argv[])
{
    int opt;
    uint16_t mtu = 0;
    bdaddr_t src_addr, dst_addr;
    int dev_id = -1;
    int fd;
    struct client *cli;
    char addr[18];

    while ((opt = getopt_long(argc, argv, "+hv:i:",
                              main_options, NULL)) != -1) {
        switch (opt) {
        case 'h':
            usage();
            return EXIT_SUCCESS;
        case 'v':
            verbose = true;
            break;
        case 'i':
            dev_id = hci_devid(optarg);
            if (dev_id < 0) {
                perror("Invalid adapter");
                return EXIT_FAILURE;
            }

            break;
        default:
            fprintf(stderr, "Invalid option: %c\n", opt);
            return EXIT_FAILURE;
        }
    }

    if (!argc) {
        usage();
        return EXIT_SUCCESS;
    }

    argc -= optind;
    argv += optind;
    optind = 0;

    if (argc) {
        usage();
        return EXIT_SUCCESS;
    }

    if (dev_id == -1)
        bacpy(&src_addr, BDADDR_ANY);
    else if (hci_devba(dev_id, &src_addr) < 0) {
        perror("Adapter not available");
        return EXIT_FAILURE;
    }

    /* TODO take care of case when Nuimo is not found. Probably need a timout... */
    if (find_nuimo(&dst_addr)) {
        memset(addr, 0, sizeof(addr));
        ba2str(&dst_addr, addr);
        printf("Found Nuimo %s\n", addr);
    }

    mainloop_init();

    fd = l2cap_le_att_connect(&src_addr, &dst_addr, BDADDR_LE_RANDOM, BT_SECURITY_LOW);
    if (fd < 0)
        return EXIT_FAILURE;

    cli = client_create(fd, mtu);
    if (!cli) {
        close(fd);
        return EXIT_FAILURE;
    }

    mainloop_run_with_signal(signal_cb, NULL);

    /* TODO clean up: unregister notifications etc */

    printf("\n\nShutting down...\n");

    client_destroy(cli);

    return EXIT_SUCCESS;
}
