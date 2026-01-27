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

/* Work item to restart advertising after connection (avoids callback issues) */
static struct k_work_delayable adv_restart_work;

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

/* Track if advertising is paused (for Xbox Controller scanning) */
static volatile bool adv_paused;

/* Restart advertising work handler */
static void adv_restart_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Don't restart if advertising is paused (Xbox scan in progress) */
    if (adv_paused) {
        printk("BLE: Advertising restart skipped (paused for Xbox scan)\n");
        return;
    }

    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                               sd, ARRAY_SIZE(sd));
    if (err && err != -EALREADY) {
        printk("BLE: Advertising restart failed (err %d)\n", err);
    } else {
        printk("BLE: Advertising restarted (multi-connect)\n");
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE: Connection failed (err %u)\n", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    ble_connected = true;
    printk("BLE: Connected\n");

    /* Restart advertising after 100ms to allow additional connections */
    k_work_schedule(&adv_restart_work, K_MSEC(100));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE: Disconnected (reason %u)\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    ble_connected = false;

    /* Always restart advertising after disconnect */
    k_work_schedule(&adv_restart_work, K_MSEC(100));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int smp_bt_init(void)
{
    int err;

    printk("BLE: Initializing...\n");

    /* Initialize work item for delayed advertising restart */
    k_work_init_delayable(&adv_restart_work, adv_restart_handler);

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

    printk("BLE: Advertising as '%s' (permanent)\n", CONFIG_BT_DEVICE_NAME);
    return 0;
}

bool smp_bt_is_connected(void)
{
    return ble_connected;
}

void smp_bt_pause_advertising(void)
{
    int err;

    if (adv_paused) {
        return;
    }

    /* Cancel any pending restart work */
    k_work_cancel_delayable(&adv_restart_work);

    err = bt_le_adv_stop();
    if (err && err != -EALREADY) {
        printk("BLE: Failed to stop advertising (err %d)\n", err);
    } else {
        adv_paused = true;
        printk("BLE: Advertising paused (for Xbox scan)\n");
    }
}

void smp_bt_resume_advertising(void)
{
    if (!adv_paused) {
        return;
    }

    adv_paused = false;

    /* Schedule advertising restart with small delay */
    k_work_schedule(&adv_restart_work, K_MSEC(100));
    printk("BLE: Advertising will resume...\n");
}
