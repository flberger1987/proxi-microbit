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
#include "ota_update.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/services/nus.h>
#include <string.h>

/* ============================================================================
 * Thread Configuration
 * ============================================================================ */

#define TELEMETRY_STACK_SIZE    1024
#define TELEMETRY_PRIORITY      8       /* Lower priority than sensors/motors */
#define TELEMETRY_INTERVAL_MS   50      /* 20 Hz update rate */

K_THREAD_STACK_DEFINE(telemetry_stack, TELEMETRY_STACK_SIZE);
static struct k_thread telemetry_thread_data;
static k_tid_t telemetry_thread_id;

/* ============================================================================
 * State Variables
 * ============================================================================ */

static volatile bool telemetry_enabled = true;  /* Enabled by default */

/* Thread monitoring - maps thread_id to kernel thread pointer */
static struct k_thread *monitored_threads[THREAD_ID_COUNT] = {0};
static const char *thread_names[THREAD_ID_COUNT] = {
    "main", "motor", "sensor", "ble_ctrl", "telemetry", "audio",
    "ir_sensors", "autonav", "idle"
};
static uint8_t current_thread_idx = 0;  /* Round-robin index */

/* Previous runtime stats for delta calculation */
static uint64_t prev_runtime[THREAD_ID_COUNT] = {0};
static uint64_t prev_timestamp = 0;

/* ============================================================================
 * Thread Monitoring Functions
 * ============================================================================ */

#ifdef CONFIG_THREAD_MONITOR
/**
 * Callback for k_thread_foreach - match thread by name
 */
static void thread_foreach_cb(const struct k_thread *thread, void *user_data)
{
    ARG_UNUSED(user_data);
    const char *name = k_thread_name_get((k_tid_t)thread);
    if (!name) return;

    for (int i = 0; i < THREAD_ID_COUNT; i++) {
        if (strcmp(name, thread_names[i]) == 0) {
            monitored_threads[i] = (struct k_thread *)thread;
            break;
        }
    }
}
#endif

/**
 * Find threads by name and populate monitored_threads array
 */
static void discover_threads(void)
{
#ifdef CONFIG_THREAD_MONITOR
    /* Use k_thread_foreach to iterate all threads */
    k_thread_foreach(thread_foreach_cb, NULL);

    /* Count discovered threads and print names */
    int count = 0;
    for (int i = 0; i < THREAD_ID_COUNT; i++) {
        if (monitored_threads[i]) {
            count++;
            printk("  Thread[%d] '%s' found\n", i, thread_names[i]);
        } else {
            printk("  Thread[%d] '%s' NOT found\n", i, thread_names[i]);
        }
    }
    printk("Thread monitoring: %d/%d threads found\n", count, THREAD_ID_COUNT);
#else
    printk("Thread monitoring: disabled (CONFIG_THREAD_MONITOR not set)\n");
#endif
}

/**
 * Get thread stats and send packet for one thread (round-robin)
 */
static void send_thread_stats(void)
{
#if defined(CONFIG_THREAD_MONITOR) && defined(CONFIG_SCHED_THREAD_USAGE)
    /* Find next valid thread */
    int attempts = 0;
    while (attempts < THREAD_ID_COUNT) {
        if (monitored_threads[current_thread_idx] != NULL) {
            break;
        }
        current_thread_idx = (current_thread_idx + 1) % THREAD_ID_COUNT;
        attempts++;
    }

    if (attempts >= THREAD_ID_COUNT) {
        return;  /* No threads to monitor */
    }

    struct k_thread *thread = monitored_threads[current_thread_idx];
    struct thread_stats_packet pkt;

    /* Header */
    pkt.magic = TELEMETRY_THREAD_MAGIC;
    pkt.version = TELEMETRY_THREAD_VER;
    pkt.thread_id = current_thread_idx;
    pkt.thread_count = THREAD_ID_COUNT;
    pkt.timestamp_ms = k_uptime_get_32();

    /* Get thread runtime stats */
    k_thread_runtime_stats_t stats;
    uint16_t cpu_permille = 0;

    if (k_thread_runtime_stats_get((k_tid_t)thread, &stats) == 0) {
        /* Calculate CPU usage since last measurement */
        uint64_t now = k_uptime_get();
        uint64_t delta_time = now - prev_timestamp;

        if (delta_time > 0 && prev_timestamp > 0) {
            uint64_t delta_runtime = stats.execution_cycles - prev_runtime[current_thread_idx];
            /* Convert to permille (0.1%) - cycles to time ratio */
            uint64_t total_cycles = (delta_time * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) / 1000;
            if (total_cycles > 0) {
                cpu_permille = (uint16_t)((delta_runtime * 1000) / total_cycles);
                if (cpu_permille > 1000) cpu_permille = 1000;
            }
        }

        prev_runtime[current_thread_idx] = stats.execution_cycles;
        if (current_thread_idx == 0) {
            prev_timestamp = now;  /* Update timestamp once per round */
        }
    }
    pkt.cpu_permille = cpu_permille;

    /* Get stack usage */
#ifdef CONFIG_THREAD_STACK_INFO
    size_t unused = 0;
    size_t size = thread->stack_info.size;
    if (k_thread_stack_space_get(thread, &unused) == 0) {
        pkt.stack_used = (uint16_t)(size - unused);
    } else {
        pkt.stack_used = 0;
    }
#else
    pkt.stack_used = 0;
#endif

    /* Send packet */
    int ret = bt_nus_send(NULL, (const uint8_t *)&pkt, sizeof(pkt));

    /* Debug: print thread stats occasionally */
    static uint32_t send_count = 0;
    if (++send_count % 100 == 0) {  /* Every 5 seconds (100 * 50ms) */
        printk("THREAD_STAT: %s cpu=%d.%d%% stack=%d (ret=%d)\n",
               thread_names[pkt.thread_id],
               pkt.cpu_permille / 10, pkt.cpu_permille % 10,
               pkt.stack_used, ret);
    }

    /* Advance to next thread for next call */
    current_thread_idx = (current_thread_idx + 1) % THREAD_ID_COUNT;
#else
    /* Thread stats disabled */
    (void)monitored_threads;
    (void)current_thread_idx;
    (void)prev_runtime;
    (void)prev_timestamp;
#endif
}

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
        /* Note: Thread is suspended during OTA via k_thread_suspend() */

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

        /* === Send main telemetry packet over BLE NUS === */
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

        /* === Send thread stats packet (round-robin, one per cycle) === */
        send_thread_stats();

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
    /* Create telemetry thread first */
    telemetry_thread_id = k_thread_create(&telemetry_thread_data, telemetry_stack,
                                          K_THREAD_STACK_SIZEOF(telemetry_stack),
                                          telemetry_thread_fn, NULL, NULL, NULL,
                                          TELEMETRY_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(telemetry_thread_id, "telemetry");

    /* Discover threads after a delay (let all threads start and set their names) */
    k_msleep(200);
    discover_threads();
}

k_tid_t telemetry_get_thread_id(void)
{
    return telemetry_thread_id;
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
