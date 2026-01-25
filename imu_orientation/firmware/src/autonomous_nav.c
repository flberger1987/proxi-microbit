/*
 * SPDX-License-Identifier: Apache-2.0
 * Autonomous Navigation Module
 *
 * Implements autonomous navigation with obstacle avoidance.
 * Uses IR sensors for obstacle detection and IMU for heading tracking.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "autonomous_nav.h"
#include "robot_state.h"
#include "motor_driver.h"
#include "ir_sensors.h"
#include "sensors.h"
#include "audio.h"

/* ============================================================================
 * Thread Configuration
 * ============================================================================ */

#define AUTONAV_THREAD_STACK_SIZE 1024
#define AUTONAV_THREAD_PRIORITY   6
#define AUTONAV_UPDATE_INTERVAL_MS 50  /* 20 Hz update rate */

K_THREAD_STACK_DEFINE(autonav_thread_stack, AUTONAV_THREAD_STACK_SIZE);
static struct k_thread autonav_thread_data;
static k_tid_t autonav_thread_id;

/* ============================================================================
 * State Variables
 * ============================================================================ */

static volatile enum autonav_state current_state = AUTONAV_DISABLED;
static volatile bool enabled = false;

/* Timing for state transitions */
static int64_t state_start_time;

/* Yaw test tracking */
static float yaw_test_start_heading;
static float yaw_test_cw_delta;
static float yaw_test_ccw_delta;

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * Get current heading from IMU sensor data
 */
static float get_current_heading(void)
{
    return sensors_get_heading();
}

/**
 * Get full orientation for logging
 */
static void get_current_orientation(float *roll, float *pitch, float *heading)
{
    struct orientation_data orient;
    sensors_get_orientation(&orient);
    *roll = orient.roll;
    *pitch = orient.pitch;
    *heading = orient.heading;
}

/**
 * Calculate heading delta (handles wraparound at 0/360)
 */
static float heading_delta(float start, float end)
{
    float delta = end - start;

    /* Normalize to -180 to +180 range */
    while (delta > 180.0f) {
        delta -= 360.0f;
    }
    while (delta < -180.0f) {
        delta += 360.0f;
    }

    return delta;
}

/**
 * Get distance from IR sensors (returns minimum of left/right in mm)
 */
static uint16_t get_obstacle_distance(bool *left_closer, bool *right_closer)
{
    uint16_t left_raw, right_raw;
    ir_sensors_get_raw(&left_raw, &right_raw);

    /* Convert raw IR values to approximate distance
     * Using the raw_to_mm logic from ir_sensors.c:
     * Higher raw value = closer obstacle
     * For simplicity, we'll use a linear approximation here
     */

    /* Threshold for "close" detection based on raw ADC values
     * From ir_sensors.c calibration table:
     * - raw 100 = 550mm
     * - raw 350 = 350mm
     * - raw 1050 = 250mm (AUTONAV_OBSTACLE_START)
     * - raw 3000 = 180mm
     * - raw 4000 = 100mm
     */

    /* AUTONAV_OBSTACLE_START (250mm) corresponds to ~raw 1050 */
    /* AUTONAV_OBSTACLE_CRIT (150mm) corresponds to ~raw 3500 */
    #define RAW_THRESHOLD_START 800   /* ~300mm - start avoiding */
    #define RAW_THRESHOLD_CRIT  2000  /* ~200mm - back up */

    *left_closer = (left_raw > right_raw);
    *right_closer = (right_raw > left_raw);

    /* Return the higher (closer) raw value as a proxy for "closest obstacle" */
    uint16_t max_raw = (left_raw > right_raw) ? left_raw : right_raw;

    /* Approximate conversion: higher raw = closer
     * Use inverse relationship similar to ir_sensors.c calibration */
    if (max_raw >= 3500) return 100;
    if (max_raw >= 2000) return 150;
    if (max_raw >= 1050) return 250;
    if (max_raw >= 350) return 350;
    if (max_raw >= 100) return 550;
    return 999;  /* Very far or no obstacle */
}

/**
 * Set motor command for current autonomous state
 */
static void set_motor_for_state(enum autonav_state state)
{
    struct motor_cmd cmd = {0};

    switch (state) {
    case AUTONAV_EXPLORING:
        cmd.linear = AUTONAV_SPEED_LINEAR;
        cmd.angular = 0;
        break;

    case AUTONAV_AVOIDING_LEFT:
        /* Obstacle on left, turn right */
        cmd.linear = 0;
        cmd.angular = AUTONAV_SPEED_ANGULAR;
        break;

    case AUTONAV_AVOIDING_RIGHT:
        /* Obstacle on right, turn left */
        cmd.linear = 0;
        cmd.angular = -AUTONAV_SPEED_ANGULAR;
        break;

    case AUTONAV_BACKING_UP:
        cmd.linear = AUTONAV_SPEED_BACKUP;
        cmd.angular = 0;
        break;

    case AUTONAV_YAW_TEST_CW:
        /* Clockwise rotation at full speed */
        cmd.linear = 0;
        cmd.angular = AUTONAV_YAW_TEST_SPEED;
        break;

    case AUTONAV_YAW_TEST_CCW:
        /* Counter-clockwise rotation at full speed */
        cmd.linear = 0;
        cmd.angular = -AUTONAV_YAW_TEST_SPEED;
        break;

    case AUTONAV_DISABLED:
    case AUTONAV_YAW_TEST_DONE:
    default:
        /* Stop motors */
        cmd.linear = 0;
        cmd.angular = 0;
        break;
    }

    /* Send command to motor queue */
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

/**
 * Transition to a new state
 */
static void transition_to(enum autonav_state new_state)
{
    if (current_state != new_state) {
        printk("AUTONAV: %d -> %d\n", current_state, new_state);
        current_state = new_state;
        state_start_time = k_uptime_get();
        set_motor_for_state(new_state);
    }
}

/* ============================================================================
 * Yaw Test Logic
 * ============================================================================ */

static void process_yaw_test(void)
{
    int64_t elapsed = k_uptime_get() - state_start_time;
    float roll, pitch, heading;
    get_current_orientation(&roll, &pitch, &heading);

    switch (current_state) {
    case AUTONAV_YAW_TEST_CW:
        /* Log all axes every 100ms to verify which one changes */
        if ((elapsed % 100) < 50) {
            printk("YAW_CW: R=%+6.1f P=%+6.1f H=%+6.1f (%lldms)\n",
                   (double)roll, (double)pitch, (double)heading, elapsed);
        }

        /* After duration, record delta and switch to CCW */
        if (elapsed >= AUTONAV_YAW_TEST_DURATION) {
            yaw_test_cw_delta = heading_delta(yaw_test_start_heading, heading);
            printk("YAW_CW complete: heading delta=%.1f degrees\n", (double)yaw_test_cw_delta);

            /* Start CCW phase */
            yaw_test_start_heading = heading;
            transition_to(AUTONAV_YAW_TEST_CCW);
        }
        break;

    case AUTONAV_YAW_TEST_CCW:
        /* Log all axes every 100ms */
        if ((elapsed % 100) < 50) {
            printk("YAW_CCW: R=%+6.1f P=%+6.1f H=%+6.1f (%lldms)\n",
                   (double)roll, (double)pitch, (double)heading, elapsed);
        }

        /* After duration, record delta and finish */
        if (elapsed >= AUTONAV_YAW_TEST_DURATION) {
            yaw_test_ccw_delta = heading_delta(yaw_test_start_heading, heading);
            printk("YAW_CCW complete: heading delta=%.1f degrees\n", (double)yaw_test_ccw_delta);

            /* Print summary */
            printk("\n========== YAW TEST RESULTS ==========\n");
            printk("CW rotation (1s):  heading change = %+.1f degrees\n",
                   (double)yaw_test_cw_delta);
            printk("CCW rotation (1s): heading change = %+.1f degrees\n",
                   (double)yaw_test_ccw_delta);
            printk("\nExpected: CW->positive, CCW->negative\n");
            if (yaw_test_cw_delta > 0 && yaw_test_ccw_delta < 0) {
                printk("RESULT: IMU heading axis is CORRECT!\n");
            } else if (yaw_test_cw_delta < 0 && yaw_test_ccw_delta > 0) {
                printk("RESULT: IMU heading axis is INVERTED!\n");
            } else {
                printk("RESULT: INCONCLUSIVE (check IMU orientation)\n");
            }
            printk("=======================================\n\n");

            /* Play success sound and return to disabled */
            audio_play(SOUND_CONNECTED);
            transition_to(AUTONAV_YAW_TEST_DONE);

            /* After brief pause, return to disabled */
            k_msleep(500);
            transition_to(AUTONAV_DISABLED);
        }
        break;

    default:
        break;
    }
}

/* ============================================================================
 * Autonomous Navigation Logic
 * ============================================================================ */

static void process_navigation(void)
{
    bool left_closer, right_closer;
    uint16_t distance = get_obstacle_distance(&left_closer, &right_closer);
    int64_t elapsed = k_uptime_get() - state_start_time;

    uint16_t left_raw, right_raw;
    ir_sensors_get_raw(&left_raw, &right_raw);

    switch (current_state) {
    case AUTONAV_EXPLORING:
        /* Check for obstacles while moving forward */
        if (distance <= AUTONAV_OBSTACLE_CRIT) {
            /* Very close - back up */
            printk("AUTONAV: Critical obstacle at %dmm, backing up\n", distance);
            transition_to(AUTONAV_BACKING_UP);
        } else if (left_raw > RAW_THRESHOLD_START && right_raw > RAW_THRESHOLD_START) {
            /* Both sides have obstacles - back up */
            printk("AUTONAV: Obstacles on both sides, backing up\n");
            transition_to(AUTONAV_BACKING_UP);
        } else if (left_raw > RAW_THRESHOLD_START) {
            /* Obstacle on left - turn right */
            printk("AUTONAV: Obstacle on left (L=%d), turning right\n", left_raw);
            transition_to(AUTONAV_AVOIDING_LEFT);
        } else if (right_raw > RAW_THRESHOLD_START) {
            /* Obstacle on right - turn left */
            printk("AUTONAV: Obstacle on right (R=%d), turning left\n", right_raw);
            transition_to(AUTONAV_AVOIDING_RIGHT);
        }
        /* else: continue forward */
        break;

    case AUTONAV_AVOIDING_LEFT:
        /* Turning right to avoid left obstacle */
        if (elapsed >= AUTONAV_TURN_DURATION) {
            /* Done turning, check if clear */
            if (left_raw < RAW_THRESHOLD_START) {
                printk("AUTONAV: Left clear, resuming exploration\n");
                transition_to(AUTONAV_EXPLORING);
            }
            /* else: continue turning */
            state_start_time = k_uptime_get();
        }
        break;

    case AUTONAV_AVOIDING_RIGHT:
        /* Turning left to avoid right obstacle */
        if (elapsed >= AUTONAV_TURN_DURATION) {
            /* Done turning, check if clear */
            if (right_raw < RAW_THRESHOLD_START) {
                printk("AUTONAV: Right clear, resuming exploration\n");
                transition_to(AUTONAV_EXPLORING);
            }
            /* else: continue turning */
            state_start_time = k_uptime_get();
        }
        break;

    case AUTONAV_BACKING_UP:
        /* Backing up from obstacle */
        if (elapsed >= AUTONAV_BACKUP_DURATION) {
            /* Done backing up, decide which way to turn */
            if (left_raw <= right_raw) {
                printk("AUTONAV: Done backing, turning left (right more blocked)\n");
                transition_to(AUTONAV_AVOIDING_RIGHT);
            } else {
                printk("AUTONAV: Done backing, turning right (left more blocked)\n");
                transition_to(AUTONAV_AVOIDING_LEFT);
            }
        }
        break;

    default:
        break;
    }
}

/* ============================================================================
 * Thread Function
 * ============================================================================ */

static void autonav_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("Autonomous navigation thread started\n");

    while (1) {
        k_msleep(AUTONAV_UPDATE_INTERVAL_MS);

        /* Handle yaw test states */
        if (current_state == AUTONAV_YAW_TEST_CW ||
            current_state == AUTONAV_YAW_TEST_CCW) {
            process_yaw_test();
            continue;
        }

        /* Skip if disabled */
        if (!enabled || current_state == AUTONAV_DISABLED) {
            continue;
        }

        /* Process autonomous navigation */
        process_navigation();
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int autonav_init(void)
{
    printk("Autonomous navigation module initialized\n");
    return 0;
}

void autonav_start_thread(void)
{
    autonav_thread_id = k_thread_create(&autonav_thread_data, autonav_thread_stack,
                                        K_THREAD_STACK_SIZEOF(autonav_thread_stack),
                                        autonav_thread_fn, NULL, NULL, NULL,
                                        AUTONAV_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(autonav_thread_id, "autonav");
}

void autonav_enable(void)
{
    if (enabled) {
        return;  /* Already enabled */
    }

    printk("AUTONAV: Enabled\n");
    enabled = true;
    audio_play(SOUND_PAIRING_START);  /* Ascending tone */
    transition_to(AUTONAV_EXPLORING);
}

void autonav_disable(void)
{
    if (!enabled && current_state == AUTONAV_DISABLED) {
        return;  /* Already disabled */
    }

    printk("AUTONAV: Disabled\n");
    enabled = false;
    transition_to(AUTONAV_DISABLED);

    /* Stop motors */
    motor_emergency_stop();

    audio_play(SOUND_DISCONNECTED);  /* Descending tone */
}

bool autonav_is_enabled(void)
{
    return enabled;
}

enum autonav_state autonav_get_state(void)
{
    return current_state;
}

void autonav_start_yaw_test(void)
{
    /* Only start if not already running */
    if (current_state == AUTONAV_YAW_TEST_CW ||
        current_state == AUTONAV_YAW_TEST_CCW) {
        printk("AUTONAV: Yaw test already running\n");
        return;
    }

    /* Disable autonomous mode if enabled */
    if (enabled) {
        enabled = false;
    }

    float roll, pitch, heading;
    get_current_orientation(&roll, &pitch, &heading);

    printk("\n========== STARTING YAW TEST ==========\n");
    printk("Phase 1: Rotate CW for 5 seconds at 100%% PWM\n");
    printk("Phase 2: Rotate CCW for 5 seconds at 100%% PWM\n");
    printk("Watching all axes: R=Roll, P=Pitch, H=Heading\n");
    printk("Initial: R=%+.1f P=%+.1f H=%+.1f\n",
           (double)roll, (double)pitch, (double)heading);
    printk("=======================================\n\n");

    audio_play(SOUND_BUTTON_PRESS);

    /* Record starting heading */
    yaw_test_start_heading = heading;
    yaw_test_cw_delta = 0;
    yaw_test_ccw_delta = 0;

    /* Start CW rotation */
    transition_to(AUTONAV_YAW_TEST_CW);
}

bool autonav_is_yaw_test_running(void)
{
    return (current_state == AUTONAV_YAW_TEST_CW ||
            current_state == AUTONAV_YAW_TEST_CCW ||
            current_state == AUTONAV_YAW_TEST_DONE);
}

void autonav_manual_override(void)
{
    if (!enabled) {
        return;  /* Not in autonomous mode */
    }

    printk("AUTONAV: Manual override detected!\n");
    autonav_disable();
    audio_play(SOUND_OBSTACLE);  /* Warning beep */
}
