/*
 * SPDX-License-Identifier: Apache-2.0
 * BLE Output Thread using Nordic UART Service (NUS)
 */

#include "ble_output.h"
#include "sensors.h"
#include "smp_bt.h"
#include "serial_output.h"
#include "telemetry.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/nus.h>

#include <string.h>
#include <stdio.h>

/* Firmware version */
#define FW_VERSION "1.0.0-ble"

/* Output thread configuration */
#define BLE_OUTPUT_STACK_SIZE 1536
#define BLE_OUTPUT_PRIORITY 7
#define BLE_OUTPUT_PERIOD_MS 100  /* 10 Hz */

K_THREAD_STACK_DEFINE(ble_output_stack, BLE_OUTPUT_STACK_SIZE);
static struct k_thread ble_output_thread_data;

/* NUS notification state */
static volatile bool nus_notifications_enabled;

/* NUS callbacks */
static void nus_notif_enabled(bool enabled, void *ctx)
{
    ARG_UNUSED(ctx);

    if (enabled) {
        nus_notifications_enabled = true;
        printk("NUS: Notifications enabled\n");
    } else {
        nus_notifications_enabled = false;
        printk("NUS: Notifications disabled\n");
    }
}

static void nus_received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    ARG_UNUSED(ctx);

    char cmd[32];

    /* Safety check */
    if (len == 0 || len >= sizeof(cmd)) {
        return;
    }

    /* Copy and null-terminate */
    memcpy(cmd, data, len);
    cmd[len] = '\0';

    /* Remove trailing newline/carriage return */
    while (len > 0 && (cmd[len - 1] == '\n' || cmd[len - 1] == '\r')) {
        cmd[--len] = '\0';
    }

    printk("NUS: Received command: '%s'\n", cmd);

    /* Handle commands */
    if (strcmp(cmd, "CAL") == 0 || strcmp(cmd, "cal") == 0) {
        /* Start calibration */
        if (!sensors_is_calibrating()) {
            sensors_start_calibration();
            bt_nus_send(conn, "CAL:OK\r\n", 8);
            printk("NUS: Calibration started\n");
        } else {
            bt_nus_send(conn, "CAL:BUSY\r\n", 10);
        }
    } else if (strcmp(cmd, "VER") == 0 || strcmp(cmd, "ver") == 0) {
        /* Send firmware version */
        char ver_msg[32];
        int ver_len = snprintf(ver_msg, sizeof(ver_msg), "VER:%s\r\n", FW_VERSION);
        bt_nus_send(conn, ver_msg, ver_len);
        printk("NUS: Version sent\n");
    } else if (strcmp(cmd, "IMU") == 0 || strcmp(cmd, "imu") == 0) {
        /* Toggle IMU streaming */
        bool enabled = !serial_output_is_imu_enabled();
        serial_output_set_imu_enabled(enabled);
        bt_nus_send(conn, enabled ? "IMU:ON\r\n" : "IMU:OFF\r\n",
                    enabled ? 8 : 9);
    } else if (strcmp(cmd, "IMUON") == 0 || strcmp(cmd, "imuon") == 0) {
        /* Enable IMU streaming */
        serial_output_set_imu_enabled(true);
        bt_nus_send(conn, "IMU:ON\r\n", 8);
    } else if (strcmp(cmd, "IMUOFF") == 0 || strcmp(cmd, "imuoff") == 0) {
        /* Disable IMU streaming */
        serial_output_set_imu_enabled(false);
        bt_nus_send(conn, "IMU:OFF\r\n", 9);
    } else if (strcmp(cmd, "IRD") == 0 || strcmp(cmd, "ird") == 0) {
        /* Toggle IR sensor debug output */
        extern void ir_sensors_set_debug(bool);
        extern bool ir_sensors_is_calibrating(void);
        static bool ir_debug_on = true;  /* Matches default in ir_sensors.c */
        ir_debug_on = !ir_debug_on;
        ir_sensors_set_debug(ir_debug_on);
        bt_nus_send(conn, ir_debug_on ? "IRD:ON\r\n" : "IRD:OFF\r\n",
                    ir_debug_on ? 8 : 9);
    } else if (strcmp(cmd, "TELE") == 0 || strcmp(cmd, "tele") == 0) {
        /* Toggle binary telemetry streaming */
        bool enabled = !telemetry_is_enabled();
        telemetry_enable(enabled);
        bt_nus_send(conn, enabled ? "TELE:ON\r\n" : "TELE:OFF\r\n",
                    enabled ? 9 : 10);
    } else if (strcmp(cmd, "TELEON") == 0 || strcmp(cmd, "teleon") == 0) {
        /* Enable binary telemetry */
        telemetry_enable(true);
        bt_nus_send(conn, "TELE:ON\r\n", 9);
    } else if (strcmp(cmd, "TELEOFF") == 0 || strcmp(cmd, "teleoff") == 0) {
        /* Disable binary telemetry */
        telemetry_enable(false);
        bt_nus_send(conn, "TELE:OFF\r\n", 10);
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        /* List available commands */
        bt_nus_send(conn, "CMD:VER,CAL,IMU,IRD,TELE,HELP\r\n", 31);
    } else {
        /* Unknown command */
        bt_nus_send(conn, "ERR:UNKNOWN\r\n", 13);
    }
}

static struct bt_nus_cb nus_callbacks = {
    .notif_enabled = nus_notif_enabled,
    .received = nus_received,
};

static void ble_output_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct sensor_msg msg;
    char output_buf[64];
    int ret;
    int len;

    printk("BLE Output thread started\n");

    while (1) {
        /* Check if we should send data */
        if (!smp_bt_is_connected() || !nus_notifications_enabled) {
            /* Not connected or notifications disabled, just drain queue */
            k_msgq_get(&orientation_msgq, &msg, K_MSEC(BLE_OUTPUT_PERIOD_MS));
            k_msleep(BLE_OUTPUT_PERIOD_MS);
            continue;
        }

        /* Get orientation data from sensor thread */
        ret = k_msgq_get(&orientation_msgq, &msg, K_MSEC(BLE_OUTPUT_PERIOD_MS));

        if (ret == 0) {
            /* Format: IMU,<timestamp>,<roll>,<pitch>,<heading>\r\n */
            len = snprintf(output_buf, sizeof(output_buf),
                          "IMU,%u,%.1f,%.1f,%.1f\r\n",
                          msg.orientation.timestamp_ms,
                          (double)msg.orientation.roll,
                          (double)msg.orientation.pitch,
                          (double)msg.orientation.heading);

            if (len > 0 && len < (int)sizeof(output_buf)) {
                ret = bt_nus_send(NULL, output_buf, len);
                if (ret && ret != -ENOTCONN) {
                    printk("NUS: Send failed (err %d)\n", ret);
                }
            }
        }
    }
}

int ble_output_init(void)
{
    int err;

    err = bt_nus_cb_register(&nus_callbacks, NULL);
    if (err) {
        printk("NUS: Failed to register callbacks (err %d)\n", err);
        return err;
    }

    printk("NUS: Callbacks registered\n");
    return 0;
}

void ble_output_start_thread(void)
{
    k_thread_create(&ble_output_thread_data, ble_output_stack,
                    K_THREAD_STACK_SIZEOF(ble_output_stack),
                    ble_output_thread_fn, NULL, NULL, NULL,
                    BLE_OUTPUT_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ble_output_thread_data, "ble_out");
}

bool ble_output_is_enabled(void)
{
    return nus_notifications_enabled;
}
