/*
 * SPDX-License-Identifier: Apache-2.0
 * BLE Telemetry Implementation
 *
 * Streams packed binary telemetry data at ~20Hz over BLE NUS.
 */

#include "telemetry.h"
#include "sensors.h"
#include "motor_driver.h"
#include "ir_sensors.h"
#include "autonomous_nav.h"
#include "smp_bt.h"
#include "ble_output.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/services/nus.h>

/* ============================================================================
 * Thread Configuration
 * ============================================================================ */

#define TELEMETRY_STACK_SIZE    1024
#define TELEMETRY_PRIORITY      8       /* Lower priority than sensors/motors */
#define TELEMETRY_INTERVAL_MS   50      /* 20 Hz update rate */

K_THREAD_STACK_DEFINE(telemetry_stack, TELEMETRY_STACK_SIZE);
static struct k_thread telemetry_thread_data;

/* ============================================================================
 * State Variables
 * ============================================================================ */

static volatile bool telemetry_enabled = true;  /* Enabled by default */

/* ============================================================================
 * Telemetry Thread
 * ============================================================================ */

static void telemetry_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct telemetry_packet pkt;
    struct orientation_data orient;
    float ir_left, ir_right;
    int8_t motor_linear, motor_angular;
    int ret;

    printk("Telemetry thread started (20 Hz)\n");

    while (1) {
        /* Only send if enabled and NUS is connected with notifications */
        if (!telemetry_enabled || !smp_bt_is_connected() || !ble_output_is_enabled()) {
            k_msleep(TELEMETRY_INTERVAL_MS);
            continue;
        }

        /* === Fill packet header === */
        pkt.magic = TELEMETRY_MAGIC;
        pkt.version = TELEMETRY_VERSION;
        pkt.timestamp_ms = k_uptime_get_32();

        /* === Orientation data (from sensors module) === */
        sensors_get_orientation(&orient);
        pkt.roll_x10 = (int16_t)(orient.roll * 10.0f);
        pkt.pitch_x10 = (int16_t)(orient.pitch * 10.0f);
        pkt.heading_x10 = (uint16_t)(orient.heading * 10.0f);

        /* === Yaw rate (Kalman-filtered from sensors module) === */
        pkt.yaw_rate_x10 = (int16_t)(sensors_get_yaw_rate() * 10.0f);

        /* === IR distance (Kalman-filtered in mm) === */
        ir_sensors_get_distance(&ir_left, &ir_right);
        pkt.ir_left_mm = (uint16_t)ir_left;
        pkt.ir_right_mm = (uint16_t)ir_right;

        /* === Motor commands === */
        motor_get_current_cmd(&motor_linear, &motor_angular);
        pkt.motor_linear = motor_linear;
        pkt.motor_angular = motor_angular;

        /* === Navigation state === */
        pkt.nav_state = (uint8_t)autonav_get_state();

        /* === Flags === */
        pkt.flags = 0;
        if (autonav_is_enabled()) {
            pkt.flags |= TELEMETRY_FLAG_AUTONAV_ENABLED;
        }
        if (motor_is_active()) {
            pkt.flags |= TELEMETRY_FLAG_MOTORS_ENABLED;
        }

        /* === Target heading === */
        float target = autonav_get_target_heading();
        if (target < 0.0f) {
            pkt.target_heading_x10 = 0xFFFF;  /* Invalid marker */
        } else {
            pkt.target_heading_x10 = (uint16_t)(target * 10.0f);
        }

        /* === Raw sensor data === */
        sensors_get_raw_accel(&pkt.raw_ax, &pkt.raw_ay, &pkt.raw_az);
        sensors_get_raw_mag(&pkt.raw_mx, &pkt.raw_my, &pkt.raw_mz);

        /* === Send packet over BLE NUS === */
        ret = bt_nus_send(NULL, (const uint8_t *)&pkt, sizeof(pkt));
        if (ret && ret != -ENOTCONN) {
            /* Don't spam errors - only log occasionally */
            static int64_t last_error_log = 0;
            int64_t now = k_uptime_get();
            if (now - last_error_log > 5000) {
                printk("Telemetry: NUS send failed (err %d)\n", ret);
                last_error_log = now;
            }
        }

        k_msleep(TELEMETRY_INTERVAL_MS);
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int telemetry_init(void)
{
    printk("Telemetry module initialized\n");
    return 0;
}

void telemetry_start_thread(void)
{
    k_thread_create(&telemetry_thread_data, telemetry_stack,
                    K_THREAD_STACK_SIZEOF(telemetry_stack),
                    telemetry_thread_fn, NULL, NULL, NULL,
                    TELEMETRY_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&telemetry_thread_data, "telemetry");
}

void telemetry_enable(bool enable)
{
    telemetry_enabled = enable;
    printk("Telemetry %s\n", enable ? "enabled" : "disabled");
}

bool telemetry_is_enabled(void)
{
    return telemetry_enabled;
}
