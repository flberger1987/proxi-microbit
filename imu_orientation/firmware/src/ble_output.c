/*
 * SPDX-License-Identifier: Apache-2.0
 * BLE Output Thread using Nordic UART Service (NUS)
 */

#include "ble_output.h"
#include "version.h"
#include "sensors.h"
#include "smp_bt.h"
#include "serial_output.h"
#include "telemetry.h"
#include "autonomous_nav.h"
#include "ota_update.h"
#include "boot_control.h"

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/nus.h>

#include <string.h>
#include <stdio.h>

/* Output thread configuration */
#define BLE_OUTPUT_STACK_SIZE 1536
#define BLE_OUTPUT_PRIORITY 7
#define BLE_OUTPUT_PERIOD_MS 100  /* 10 Hz */

K_THREAD_STACK_DEFINE(ble_output_stack, BLE_OUTPUT_STACK_SIZE);
static struct k_thread ble_output_thread_data;

/* NUS notification state */
static volatile bool nus_notifications_enabled;

/* OTA work queue for async processing
 * Priority 4 = higher than sensors(5), lower than BLE TX(7)
 */
#define OTA_WORKQ_PRIORITY  4
#define OTA_WORKQ_STACK_SIZE 3072
K_THREAD_STACK_DEFINE(ota_workq_stack, OTA_WORKQ_STACK_SIZE);
static struct k_work_q ota_workq;

/* Double-buffer (ping-pong) for OTA packets
 * This prevents race conditions where a new packet overwrites
 * the buffer before the previous one is processed.
 */
static struct k_work ota_work;
static uint8_t ota_packet_buf[2][256];  /* Two buffers */
static uint16_t ota_packet_len[2];
static volatile uint8_t ota_write_idx;   /* Buffer being written by BLE callback */
static volatile uint8_t ota_read_idx;    /* Buffer being read by work handler */
static K_SEM_DEFINE(ota_buf_sem, 1, 1);  /* Semaphore to signal buffer ready */

static void ota_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Process all pending buffers */
    while (ota_packet_len[ota_read_idx] > 0) {
        uint8_t idx = ota_read_idx;
        uint16_t len = ota_packet_len[idx];

        /* Process the packet (this may do flash writes) */
        ota_process_packet(ota_packet_buf[idx], len);

        /* Mark buffer as processed */
        ota_packet_len[idx] = 0;

        /* Move to next buffer */
        ota_read_idx = (ota_read_idx + 1) % 2;
    }
}

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

    const uint8_t *byte_data = (const uint8_t *)data;

    /* Check for OTA packet (first byte is OTA_MAGIC) */
    if (len > 0 && byte_data[0] == OTA_MAGIC) {
        /* Use ping-pong buffer to avoid race conditions.
         * Copy to write buffer and submit work. */
        uint8_t idx = ota_write_idx;

        /* Check if buffer is available (previous packet processed) */
        if (ota_packet_len[idx] == 0 && len <= sizeof(ota_packet_buf[0])) {
            memcpy(ota_packet_buf[idx], byte_data, len);
            ota_packet_len[idx] = len;

            /* Switch to other buffer for next packet */
            ota_write_idx = (ota_write_idx + 1) % 2;

            /* Submit work to process the packet */
            k_work_submit_to_queue(&ota_workq, &ota_work);
        } else {
            /* Buffer busy - this shouldn't happen if Python waits for ACK */
            printk("OTA: Buffer busy, dropping packet\n");
        }
        return;
    }

    char cmd[80];  /* Increased for PID command with KD/DMAX (~60 bytes) */

    /* Safety check */
    if (len == 0 || len >= sizeof(cmd)) {
        printk("NUS: Command too long (%d bytes, max %zu)\n", len, sizeof(cmd) - 1);
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
        /* Send firmware version, slot, and boot info */
        extern uint32_t boot_control_get_firmware_version(uint8_t slot);
        char ver_msg[80];
        uint8_t slot = boot_control_get_active_slot();
        int ver_len = snprintf(ver_msg, sizeof(ver_msg),
            "VER:%s,SLOT:%c,A:%u,B:%u\r\n",
            FW_VERSION, slot == 0 ? 'A' : 'B',
            boot_control_get_firmware_version(SLOT_A),
            boot_control_get_firmware_version(SLOT_B));
        bt_nus_send(conn, ver_msg, ver_len);
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
    } else if (strncmp(cmd, "PID:", 4) == 0) {
        /* Parse PID parameters: PID:KP=0.50,KI=0.050,KD=0.10,IMAX=5.0,DMAX=5.0,YMAX=12.0 */
        float kp, ki, kd, i_max, d_max, yaw_max;
        autonav_get_pid_params(&kp, &ki, &kd, &i_max, &d_max, &yaw_max);

        char *param = cmd + 4;
        char *saveptr;
        char *token = strtok_r(param, ",", &saveptr);

        while (token != NULL) {
            if (strncmp(token, "KP=", 3) == 0) {
                kp = strtof(token + 3, NULL);
            } else if (strncmp(token, "KI=", 3) == 0) {
                ki = strtof(token + 3, NULL);
            } else if (strncmp(token, "KD=", 3) == 0) {
                kd = strtof(token + 3, NULL);
            } else if (strncmp(token, "IMAX=", 5) == 0) {
                i_max = strtof(token + 5, NULL);
            } else if (strncmp(token, "DMAX=", 5) == 0) {
                d_max = strtof(token + 5, NULL);
            } else if (strncmp(token, "YMAX=", 5) == 0) {
                yaw_max = strtof(token + 5, NULL);
            }
            token = strtok_r(NULL, ",", &saveptr);
        }

        autonav_set_pid_params(kp, ki, kd, i_max, d_max, yaw_max);

        char resp[96];
        int resp_len = snprintf(resp, sizeof(resp),
            "PID:KP=%.2f,KI=%.3f,KD=%.2f,IMAX=%.1f,DMAX=%.1f,YMAX=%.1f\r\n",
            (double)kp, (double)ki, (double)kd, (double)i_max, (double)d_max, (double)yaw_max);
        bt_nus_send(conn, resp, resp_len);
    } else if (strcmp(cmd, "PIDGET") == 0 || strcmp(cmd, "pidget") == 0) {
        /* Get current PID parameters */
        float kp, ki, kd, i_max, d_max, yaw_max;
        autonav_get_pid_params(&kp, &ki, &kd, &i_max, &d_max, &yaw_max);

        char resp[96];
        int resp_len = snprintf(resp, sizeof(resp),
            "PID:KP=%.2f,KI=%.3f,KD=%.2f,IMAX=%.1f,DMAX=%.1f,YMAX=%.1f\r\n",
            (double)kp, (double)ki, (double)kd, (double)i_max, (double)d_max, (double)yaw_max);
        bt_nus_send(conn, resp, resp_len);
    } else if (strcmp(cmd, "PIDSAVE") == 0 || strcmp(cmd, "pidsave") == 0) {
        /* Save PID parameters to flash */
        int err = autonav_save_pid_params();
        if (err == 0) {
            bt_nus_send(conn, "PID:SAVED\r\n", 11);
        } else {
            bt_nus_send(conn, "PID:ERR\r\n", 9);
        }
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        /* List available commands */
        bt_nus_send(conn, "CMD:VER,CAL,IMU,IRD,TELE,PID,PIDGET,PIDSAVE,HELP\r\n", 50);
    } else {
        /* Unknown command */
        bt_nus_send(conn, "ERR:UNKNOWN\r\n", 13);
    }
}

static struct bt_nus_cb nus_callbacks = {
    .notif_enabled = nus_notif_enabled,
    .received = nus_received,
};

/* OTA BLE send callback */
static int ota_ble_send(const uint8_t *data, uint16_t len)
{
    return bt_nus_send(NULL, data, len);
}

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

    /* Initialize OTA subsystem and set BLE send callback */
    err = ota_init();
    if (err) {
        printk("OTA: Init failed (err %d)\n", err);
        /* Continue anyway - OTA just won't work */
    } else {
        ota_set_send_callback(ota_ble_send);
        printk("OTA: Initialized with BLE send callback\n");
    }

    printk("NUS: Callbacks registered\n");
    return 0;
}

void ble_output_start_thread(void)
{
    /* Initialize dedicated OTA work queue with HIGH priority */
    k_work_queue_init(&ota_workq);
    k_work_queue_start(&ota_workq, ota_workq_stack,
                       K_THREAD_STACK_SIZEOF(ota_workq_stack),
                       OTA_WORKQ_PRIORITY, NULL);
    k_thread_name_set(&ota_workq.thread, "ota_workq");

    /* Initialize OTA work item */
    k_work_init(&ota_work, ota_work_handler);

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
