/*
 * SPDX-License-Identifier: Apache-2.0
 * SMP BLE Transport for MCUmgr OTA DFU
 */

#include "smp_bt.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>

/* Track connection state */
static struct bt_conn *current_conn;
static volatile bool ble_connected;
static volatile bool advertising_paused;  /* Paused during Xbox connection */

/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        /* SMP Service UUID: 8D53DC1D-1DB7-4CD3-868B-8A527460AA84 */
        0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
        0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
};

/* Scan response data - device name */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE: Connection failed (err %u)\n", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    ble_connected = true;
    printk("BLE: Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE: Disconnected (reason %u)\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    ble_connected = false;

    /* Restart advertising only if not paused (e.g., during Xbox pairing) */
    if (!advertising_paused) {
        int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                                   sd, ARRAY_SIZE(sd));
        if (err) {
            printk("BLE: Advertising restart failed (err %d)\n", err);
        }
    } else {
        printk("BLE: Advertising paused, not restarting\n");
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int smp_bt_init(void)
{
    int err;

    printk("BLE: Initializing...\n");

    /* Initialize Bluetooth */
    err = bt_enable(NULL);
    if (err) {
        printk("BLE: bt_enable failed (err %d)\n", err);
        return err;
    }

    printk("BLE: Bluetooth initialized\n");

    /* Load persistent settings (including bond info) */
    err = settings_load();
    if (err) {
        printk("BLE: settings_load failed (err %d)\n", err);
        /* Continue anyway - bonding just won't be persistent */
    } else {
        printk("BLE: Persistent settings loaded\n");
    }

    /* Start advertising */
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                           sd, ARRAY_SIZE(sd));
    if (err) {
        printk("BLE: Advertising failed to start (err %d)\n", err);
        return err;
    }

    printk("BLE: Advertising as '%s'\n", CONFIG_BT_DEVICE_NAME);
    return 0;
}

bool smp_bt_is_connected(void)
{
    return ble_connected;
}

void smp_bt_pause_advertising(void)
{
    advertising_paused = true;
    int err = bt_le_adv_stop();
    if (err) {
        printk("BLE: Advertising stop failed (err %d)\n", err);
    } else {
        printk("BLE: Advertising paused for Xbox pairing\n");
    }
}

void smp_bt_resume_advertising(void)
{
    advertising_paused = false;
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                               sd, ARRAY_SIZE(sd));
    if (err && err != -EALREADY) {
        printk("BLE: Advertising resume failed (err %d)\n", err);
    } else {
        printk("BLE: Advertising resumed\n");
    }
}
