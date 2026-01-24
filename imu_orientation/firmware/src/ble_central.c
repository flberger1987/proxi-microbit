/*
 * SPDX-License-Identifier: Apache-2.0
 * BLE Central - Xbox Controller Connection
 *
 * Implements BLE Central role to connect to Xbox Wireless Controller
 * and receive HID input reports.
 *
 * The Xbox Controller advertises as "Xbox Wireless Controller" and
 * implements BLE HID over GATT (HOGP).
 *
 * HID Service UUID: 0x1812
 * Report Characteristic UUID: 0x2A4D (with Report Reference descriptor)
 */

#include "ble_central.h"
#include "robot_state.h"
#include "smp_bt.h"  /* For pausing/resuming advertising */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>

#include <string.h>

/* Thread configuration */
#define BLE_CENTRAL_STACK_SIZE 2048
#define BLE_CENTRAL_PRIORITY 3

K_THREAD_STACK_DEFINE(ble_central_stack, BLE_CENTRAL_STACK_SIZE);
static struct k_thread ble_central_thread_data;

/* Target device names - Xbox and PS5 DualSense controllers */
static const char *target_names[] = {
    /* Xbox controllers */
    "Xbox Wireless Controller",
    "Xbox Elite Wireless Controller",
    "Xbox Adaptive Controller",
    /* PS5 DualSense controllers */
    "DualSense Wireless Controller",
    "Wireless Controller",  /* Generic name some controllers use */
    "DualSense",
    NULL
};

/* UUIDs */
#define BT_UUID_HID_VAL             0x1812
#define BT_UUID_HID_REPORT_VAL      0x2A4D
#define BT_UUID_HID_REPORT_MAP_VAL  0x2A4B
#define BT_UUID_HID_REPORT_REF_VAL  0x2908

static struct bt_uuid_16 uuid_hid = BT_UUID_INIT_16(BT_UUID_HID_VAL);
static struct bt_uuid_16 uuid_hid_report = BT_UUID_INIT_16(BT_UUID_HID_REPORT_VAL);

/* Connection state */
static struct bt_conn *controller_conn;
static volatile bool is_scanning;
static volatile bool is_connected;
static volatile bool discovery_complete;

/* GATT handles */
static uint16_t hid_start_handle;
static uint16_t hid_end_handle;
static uint16_t report_handle;
static uint16_t report_ccc_handle;

/* Discovery state machine */
enum discovery_state {
    DISC_STATE_IDLE,
    DISC_STATE_PRIMARY,
    DISC_STATE_CHARACTERISTIC,
    DISC_STATE_CCC,
};
static enum discovery_state disc_state = DISC_STATE_IDLE;

/* Callbacks */
static struct ble_central_callbacks callbacks;

/* Subscription parameters */
static struct bt_gatt_subscribe_params subscribe_params;

/* Discovery parameters */
static struct bt_gatt_discover_params discover_params;

/* Scan timeout work */
static struct k_work_delayable scan_timeout_work;
#define SCAN_TIMEOUT_MS 30000

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void start_hid_discovery(struct bt_conn *conn);

/* ============================================================================
 * HID Report Notification Handler
 * ============================================================================ */

static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (!data) {
        printk("BLE Central: Unsubscribed\n");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    /* Forward HID report to callback */
    if (callbacks.input_received && length > 0) {
        callbacks.input_received(data, length);
    }

    return BT_GATT_ITER_CONTINUE;
}

/* ============================================================================
 * GATT Discovery
 * ============================================================================ */

/* CCC UUID for descriptor discovery */
static struct bt_uuid_16 uuid_ccc = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    int err;

    if (!attr) {
        printk("BLE Central: Discovery phase complete (state=%d)\n", disc_state);

        switch (disc_state) {
        case DISC_STATE_PRIMARY:
            /* Primary service discovered, now discover characteristics */
            if (hid_start_handle && hid_end_handle) {
                printk("BLE Central: HID Service found [0x%04x-0x%04x]\n",
                       hid_start_handle, hid_end_handle);

                /* Discover HID Report characteristics */
                disc_state = DISC_STATE_CHARACTERISTIC;
                discover_params.uuid = &uuid_hid_report.uuid;
                discover_params.start_handle = hid_start_handle;
                discover_params.end_handle = hid_end_handle;
                discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

                err = bt_gatt_discover(conn, &discover_params);
                if (err) {
                    printk("BLE Central: Characteristic discovery failed (err %d)\n", err);
                    disc_state = DISC_STATE_IDLE;
                    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                }
            } else {
                printk("BLE Central: HID Service not found - not an Xbox controller?\n");
                disc_state = DISC_STATE_IDLE;
                bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }
            break;

        case DISC_STATE_CHARACTERISTIC:
            /* Characteristic discovered, now discover CCC descriptor */
            if (report_handle) {
                printk("BLE Central: HID Report found [handle 0x%04x], discovering CCC...\n",
                       report_handle);

                /* Discover CCC descriptor for the report characteristic */
                disc_state = DISC_STATE_CCC;
                discover_params.uuid = &uuid_ccc.uuid;
                discover_params.start_handle = report_handle + 1;
                discover_params.end_handle = hid_end_handle;
                discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

                err = bt_gatt_discover(conn, &discover_params);
                if (err) {
                    printk("BLE Central: CCC discovery failed (err %d)\n", err);
                    disc_state = DISC_STATE_IDLE;
                    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                }
            } else {
                printk("BLE Central: HID Report characteristic not found\n");
                disc_state = DISC_STATE_IDLE;
                bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }
            break;

        case DISC_STATE_CCC:
            /* CCC not found (no descriptors), try fallback with auto-CCC discovery */
            printk("BLE Central: CCC not found, trying auto-discovery...\n");

            subscribe_params.notify = notify_func;
            subscribe_params.value_handle = report_handle;
            subscribe_params.ccc_handle = 0;  /* Let Zephyr auto-discover */
            subscribe_params.value = BT_GATT_CCC_NOTIFY;

            err = bt_gatt_subscribe(conn, &subscribe_params);
            if (err && err != -EALREADY) {
                printk("BLE Central: Subscribe with auto-CCC failed (err %d)\n", err);
                bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            } else {
                printk("BLE Central: Subscribed (auto-CCC)!\n");
                discovery_complete = true;
                robot_set_state(ROBOT_STATE_CONNECTED);
                if (callbacks.connected) {
                    callbacks.connected();
                }
            }
            disc_state = DISC_STATE_IDLE;
            break;

        default:
            break;
        }
        return BT_GATT_ITER_STOP;
    }

    /* Process discovered attributes */
    switch (disc_state) {
    case DISC_STATE_PRIMARY:
        {
            struct bt_gatt_service_val *service = attr->user_data;
            printk("BLE Central: Found primary service at handle 0x%04x-0x%04x\n",
                   attr->handle, service->end_handle);
            hid_start_handle = attr->handle + 1;
            hid_end_handle = service->end_handle;
        }
        break;

    case DISC_STATE_CHARACTERISTIC:
        {
            struct bt_gatt_chrc *chrc = attr->user_data;
            printk("BLE Central: Found characteristic at handle 0x%04x, props=0x%02x\n",
                   chrc->value_handle, chrc->properties);
            if (bt_uuid_cmp(chrc->uuid, &uuid_hid_report.uuid) == 0) {
                /* Take first report characteristic with notify property */
                if (report_handle == 0 && (chrc->properties & BT_GATT_CHRC_NOTIFY)) {
                    report_handle = chrc->value_handle;
                    printk("BLE Central: Using this as input report\n");
                }
            }
        }
        break;

    case DISC_STATE_CCC:
        printk("BLE Central: Found CCC descriptor at handle 0x%04x\n", attr->handle);
        report_ccc_handle = attr->handle;
        /* Found CCC - immediately start subscription */
        {
            int sub_err;
            printk("BLE Central: Subscribing to HID reports...\n");

            subscribe_params.notify = notify_func;
            subscribe_params.value_handle = report_handle;
            subscribe_params.ccc_handle = report_ccc_handle;
            subscribe_params.value = BT_GATT_CCC_NOTIFY;

            sub_err = bt_gatt_subscribe(conn, &subscribe_params);
            if (sub_err && sub_err != -EALREADY) {
                printk("BLE Central: Subscribe failed (err %d)\n", sub_err);
                /* Try with auto-discovered CCC as fallback */
                subscribe_params.ccc_handle = 0;
                sub_err = bt_gatt_subscribe(conn, &subscribe_params);
                if (sub_err && sub_err != -EALREADY) {
                    printk("BLE Central: Subscribe retry failed (err %d)\n", sub_err);
                    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                } else {
                    printk("BLE Central: Subscribed (auto-CCC)!\n");
                    discovery_complete = true;
                }
            } else {
                printk("BLE Central: Subscribed!\n");
                discovery_complete = true;
            }

            if (discovery_complete) {
                printk("BLE Central: Controller ready!\n");
                robot_set_state(ROBOT_STATE_CONNECTED);
                if (callbacks.connected) {
                    callbacks.connected();
                }
            }
        }
        disc_state = DISC_STATE_IDLE;
        return BT_GATT_ITER_STOP;

    default:
        break;
    }

    return BT_GATT_ITER_CONTINUE;
}

static void start_hid_discovery(struct bt_conn *conn)
{
    int err;

    printk("BLE Central: Starting HID service discovery...\n");

    /* Reset handles */
    hid_start_handle = 0;
    hid_end_handle = 0;
    report_handle = 0;
    report_ccc_handle = 0;
    discovery_complete = false;
    disc_state = DISC_STATE_PRIMARY;

    /* Discover HID service */
    discover_params.uuid = &uuid_hid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        printk("BLE Central: Discovery failed to start (err %d)\n", err);
        disc_state = DISC_STATE_IDLE;
    }
}

/* ============================================================================
 * Connection Callbacks
 * ============================================================================ */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
        printk("BLE Central: Connection failed (err %u)\n", err);
        controller_conn = NULL;
        return;
    }

    /* Check if this is our initiated connection */
    if (controller_conn != conn) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("BLE Central: Connected to %s\n", addr);

    is_connected = true;
    is_scanning = false;

    /* Cancel scan timeout */
    k_work_cancel_delayable(&scan_timeout_work);

    /* NOTE: Stay in PAIRING state until HID discovery completes!
     * Don't set ROBOT_STATE_CONNECTED or call callbacks yet.
     */

    /* Request security (pairing/bonding) - Xbox controller requires this
     * Try L3 (authenticated encryption) for bonding, fall back to L2
     * Discovery will start after security is established via callback
     */
    int sec_err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (sec_err) {
        printk("BLE Central: Security request failed (err %d), trying discovery anyway\n", sec_err);
        start_hid_discovery(conn);
    } else {
        printk("BLE Central: Security requested, waiting for pairing/bonding...\n");
        /* Discovery will start in security_changed callback */
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (controller_conn != conn) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("BLE Central: Disconnected from %s (reason %u)\n", addr, reason);

    bt_conn_unref(controller_conn);
    controller_conn = NULL;
    is_connected = false;
    discovery_complete = false;

    /* Reset handles */
    hid_start_handle = 0;
    hid_end_handle = 0;
    report_handle = 0;

    /* Update robot state */
    robot_set_state(ROBOT_STATE_IDLE);

    /* Resume NUS advertising since Xbox controller disconnected */
    smp_bt_resume_advertising();

    /* Notify callback */
    if (callbacks.disconnected) {
        callbacks.disconnected();
    }
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (controller_conn != conn) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        printk("BLE Central: Security failed for %s (err %d)\n", addr, err);
        /* Try discovery anyway - some controllers work without full security */
        start_hid_discovery(conn);
    } else {
        printk("BLE Central: Security changed for %s: level %d\n", addr, level);
        /* Security established, now start HID discovery */
        start_hid_discovery(conn);
    }
}

BT_CONN_CB_DEFINE(central_conn_callbacks) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .security_changed = security_changed_cb,
};

/* ============================================================================
 * Scan Functions
 * ============================================================================ */

static bool name_matches_target(const char *name, size_t len)
{
    for (const char **target = target_names; *target != NULL; target++) {
        size_t target_len = strlen(*target);
        if (len >= target_len && memcmp(name, *target, target_len) == 0) {
            return true;
        }
    }
    return false;
}

static bool parse_ad_data(struct bt_data *data, void *user_data)
{
    bool *found = user_data;

    switch (data->type) {
    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_NAME_SHORTENED:
        /* Debug: Print all device names we see */
        printk("BLE: Saw device '%.*s'\n", data->data_len, data->data);

        if (name_matches_target((const char *)data->data, data->data_len)) {
            *found = true;
            return false;  /* Stop parsing */
        }
        break;
    }

    return true;  /* Continue parsing */
}

/* Store address of connectable device for later connection */
static bt_addr_le_t pending_conn_addr;
static bool pending_conn_valid;

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *buf)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bool found = false;
    int err;

    /* For connectable advertisements, store the address */
    if (adv_type == BT_GAP_ADV_TYPE_ADV_IND ||
        adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        bt_addr_le_copy(&pending_conn_addr, addr);
        pending_conn_valid = true;
    }

    /* Parse advertisement/scan response data for device name */
    bt_data_parse(buf, parse_ad_data, &found);

    if (!found) {
        return;
    }

    /* For scan response, check if we have a pending connectable address */
    if (adv_type == BT_GAP_ADV_TYPE_SCAN_RSP) {
        if (!pending_conn_valid || bt_addr_le_cmp(&pending_conn_addr, addr) != 0) {
            /* Name found in scan response but no matching connectable ad */
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            printk("BLE Central: Found name at %s but not connectable\n", addr_str);
            return;
        }
    }

    /* Found Xbox controller! */
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    printk("BLE Central: Found Xbox Controller at %s (RSSI %d)\n", addr_str, rssi);

    /* Stop scanning */
    err = bt_le_scan_stop();
    if (err) {
        printk("BLE Central: Failed to stop scan (err %d)\n", err);
        return;
    }
    is_scanning = false;

    /* Connect to the controller */
    struct bt_conn_le_create_param create_param = BT_CONN_LE_CREATE_PARAM_INIT(
        BT_CONN_LE_OPT_NONE,
        BT_GAP_SCAN_FAST_INTERVAL,
        BT_GAP_SCAN_FAST_WINDOW
    );

    /* Xbox Controller needs very relaxed connection parameters */
    struct bt_le_conn_param conn_param = BT_LE_CONN_PARAM_INIT(
        24, 48,  /* Connection interval: 30-60ms (very relaxed) */
        10,      /* Latency: allow skipping 10 events */
        1000     /* Timeout: 10s (very long supervision timeout) */
    );

    err = bt_conn_le_create(addr, &create_param, &conn_param, &controller_conn);
    if (err) {
        printk("BLE Central: Connection failed to start (err %d)\n", err);
        /* Try to restart scanning */
        ble_central_start_scan();
    }
}

static void scan_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (is_scanning) {
        printk("BLE Central: Scan timeout\n");
        bt_le_scan_stop();
        is_scanning = false;
        robot_set_state(ROBOT_STATE_IDLE);
        /* Resume NUS advertising since scan ended */
        smp_bt_resume_advertising();
    }
}

/* ============================================================================
 * BLE Central Thread
 * ============================================================================ */

static void ble_central_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("BLE Central thread started\n");

    /* Main loop - handle controller input processing */
    while (1) {
        /* For now, just keep the thread alive
         * Input processing happens in notify callback
         */
        k_msleep(100);
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int ble_central_init(const struct ble_central_callbacks *cbs)
{
    int err;

    if (cbs) {
        callbacks = *cbs;
    }

    /* Clear all old bonding info to avoid KEY_REJECTED errors */
    err = bt_unpair(BT_ID_DEFAULT, NULL);
    if (err) {
        printk("BLE Central: Failed to clear bonds (err %d)\n", err);
    } else {
        printk("BLE Central: Cleared old bonding info\n");
    }

    /* Initialize scan timeout work */
    k_work_init_delayable(&scan_timeout_work, scan_timeout_handler);

    printk("BLE Central: Initialized\n");
    return 0;
}

int ble_central_start_scan(void)
{
    int err;

    if (is_scanning) {
        return -EALREADY;
    }

    if (is_connected) {
        return -EISCONN;
    }

    /* Pause NUS advertising to avoid dual-role conflicts */
    smp_bt_pause_advertising();

    struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_ACTIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) {
        printk("BLE Central: Scan failed to start (err %d)\n", err);
        return err;
    }

    is_scanning = true;
    robot_set_state(ROBOT_STATE_PAIRING);

    /* Start scan timeout */
    k_work_schedule(&scan_timeout_work, K_MSEC(SCAN_TIMEOUT_MS));

    printk("BLE Central: Scanning for Xbox Controller...\n");
    return 0;
}

int ble_central_stop_scan(void)
{
    int err;

    if (!is_scanning) {
        return 0;
    }

    k_work_cancel_delayable(&scan_timeout_work);

    err = bt_le_scan_stop();
    if (err) {
        printk("BLE Central: Scan stop failed (err %d)\n", err);
        return err;
    }

    is_scanning = false;
    printk("BLE Central: Scan stopped\n");
    return 0;
}

int ble_central_disconnect(void)
{
    if (!controller_conn) {
        return -ENOTCONN;
    }

    return bt_conn_disconnect(controller_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

bool ble_central_is_scanning(void)
{
    return is_scanning;
}

bool ble_central_is_connected(void)
{
    return is_connected && discovery_complete;
}

void ble_central_start_thread(void)
{
    k_thread_create(&ble_central_thread_data, ble_central_stack,
                    K_THREAD_STACK_SIZEOF(ble_central_stack),
                    ble_central_thread_fn, NULL, NULL, NULL,
                    BLE_CENTRAL_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ble_central_thread_data, "ble_ctrl");
}
