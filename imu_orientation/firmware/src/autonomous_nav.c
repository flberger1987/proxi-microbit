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
#include "yaw_controller.h"

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
 * State Variables (protected by autonav_mutex)
 * ============================================================================ */

/* Mutex for thread-safe access to autonav state */
static K_MUTEX_DEFINE(autonav_mutex);

static enum autonav_state current_state = AUTONAV_DISABLED;
static bool enabled = false;

/* Timing for state transitions */
static int64_t state_start_time;

/* Heading-hold target (degrees, 0-360) */
static float target_heading;

/* Scanning state variables */
static int scan_phase;            /* 0=scanning left (CCW), 1=scanning right (CW) */
static float scan_origin_heading; /* Heading when scan started */

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * Absolute value of a float
 */
static inline float abs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

/**
 * Get current heading from IMU sensor data
 */
static float get_current_heading(void)
{
    return sensors_get_heading();
}

/**
 * Calculate heading delta (handles wraparound at 0/360)
 * Returns the shortest path from current to target.
 */
static float heading_delta(float target, float current)
{
    float delta = target - current;

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
 * Normalize heading to 0-360 range
 */
static float normalize_heading(float heading)
{
    while (heading >= 360.0f) {
        heading -= 360.0f;
    }
    while (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}

/**
 * Set motor command for current autonomous state
 */
static void set_motor_for_state(enum autonav_state state)
{
    struct motor_cmd cmd = {0};

    switch (state) {
    case AUTONAV_HEADING_HOLD:
        /* Forward motion - angular handled by process_heading_hold */
        cmd.linear = AUTONAV_SPEED_LINEAR;
        cmd.angular = 0;
        break;

    case AUTONAV_TURNING:
    case AUTONAV_SCANNING:
        /* Turning in place - angular set by process functions */
        cmd.linear = 0;
        cmd.angular = 0;
        break;

    case AUTONAV_DISABLED:
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
 * Note: For AUTONAV_TURNING, we don't send an immediate motor command
 * because process_turning() needs to calculate the proper angular velocity.
 * Sending {0,0} here would cause a brief stop (Bug #1 fix).
 */
static void transition_to(enum autonav_state new_state)
{
    if (current_state != new_state) {
        printk("AUTONAV: %d -> %d\n", current_state, new_state);
        current_state = new_state;
        state_start_time = k_uptime_get();

        /* Don't send motor command for TURNING - process_turning() handles it
         * This prevents the 50ms stop gap that caused Bug #1 */
        if (new_state != AUTONAV_TURNING) {
            set_motor_for_state(new_state);
        }
    }
}

/* ============================================================================
 * Heading-Hold Navigation Logic
 * ============================================================================ */

/**
 * Clamp a value to a range
 */
static inline int16_t clamp_i16(int16_t val, int16_t min, int16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * Process heading-hold state - drive forward with CONTINUOUS heading correction
 *
 * Strategy:
 * - Always move forward while continuously correcting heading
 * - Outer loop: Heading error → desired yaw rate (°/s)
 * - Inner loop: Yaw rate PID controller (in yaw_controller.c)
 * - Slow down when obstacle detected
 * - Stop and scan when obstacle is too close
 */
static void process_heading_hold(void)
{
    /* Get Kalman-filtered distances in mm */
    float left_mm, right_mm;
    ir_sensors_get_distance(&left_mm, &right_mm);
    float min_dist = (left_mm < right_mm) ? left_mm : right_mm;

    /* Obstacle too close: stop and scan for clear path */
    if (min_dist < OBSTACLE_DIST_STOP) {
        printk("AUTONAV: Obstacle at %.0f mm, starting scan\n", (double)min_dist);
        scan_origin_heading = get_current_heading();
        scan_phase = 0;  /* Start scanning left (CCW) */
        yaw_controller_set_target(0.0f);
        transition_to(AUTONAV_SCANNING);
        return;
    }

    /* Calculate heading error */
    float current = get_current_heading();
    float heading_error = heading_delta(target_heading, current);

    /* Calculate desired yaw rate from heading error (outer loop)
     * error > 0 means target is CCW from current → need CCW rotation
     * Yaw controller convention: positive = CW, negative = CCW
     * Therefore: negate the error!
     */
    float desired_yaw_rate = 0.0f;
    if (abs_f(heading_error) > HEADING_TOLERANCE) {
        /* Proportional: 1°/s per degree of error, clamped to max */
        desired_yaw_rate = -heading_error * HEADING_KP;
        if (desired_yaw_rate > AUTONAV_YAW_RATE_MAX) desired_yaw_rate = AUTONAV_YAW_RATE_MAX;
        if (desired_yaw_rate < -AUTONAV_YAW_RATE_MAX) desired_yaw_rate = -AUTONAV_YAW_RATE_MAX;
    }

    /* Set target for inner yaw rate controller */
    yaw_controller_set_target(desired_yaw_rate);

    /* Calculate forward speed - slow down near obstacles */
    float speed_factor = 1.0f;
    if (min_dist < OBSTACLE_DIST_SLOW) {
        /* Linear interpolation: full speed at SLOW dist, 30% at STOP dist */
        speed_factor = 0.3f + 0.7f * (min_dist - OBSTACLE_DIST_STOP) /
                       (OBSTACLE_DIST_SLOW - OBSTACLE_DIST_STOP);
        if (speed_factor < 0.3f) speed_factor = 0.3f;
    }
    int16_t linear = (int16_t)(AUTONAV_SPEED_LINEAR * speed_factor);

    /* Debug: Log every ~2 seconds */
    static int debug_counter = 0;
    if (++debug_counter >= 40) {
        debug_counter = 0;
        printk("NAV: hdg=%.0f tgt=%.0f err=%+.0f rate=%.1f dist=%.0f spd=%d\n",
               (double)current, (double)target_heading, (double)heading_error,
               (double)desired_yaw_rate, (double)min_dist, linear);
    }

    /* Send linear speed to motor queue (angular comes from yaw controller) */
    struct motor_cmd cmd = {
        .linear = linear,
        .angular = 0,  /* Ignored - yaw controller provides angular */
    };
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

/**
 * Process turning state - rotate in place to reach target heading
 * Outer loop: Heading error → desired yaw rate
 * Inner loop: Yaw rate PID controller
 */
static void process_turning(void)
{
    int64_t elapsed = k_uptime_get() - state_start_time;

    /* Check for timeout */
    if (elapsed >= AUTONAV_TURNING_TIMEOUT) {
        printk("AUTONAV: Turning timeout! Resuming heading-hold\n");
        target_heading = get_current_heading();
        yaw_controller_set_target(0.0f);
        transition_to(AUTONAV_HEADING_HOLD);
        return;
    }

    float current = get_current_heading();
    float error = heading_delta(target_heading, current);

    /* Check if target reached */
    if (abs_f(error) < HEADING_TOLERANCE) {
        printk("AUTONAV: Target heading reached (%.1f)\n", (double)target_heading);
        yaw_controller_set_target(0.0f);
        transition_to(AUTONAV_HEADING_HOLD);
        return;
    }

    /* Calculate desired yaw rate (outer loop)
     * Proportional with minimum rate for smooth approach
     * Positive error → need CCW rotation → negative yaw rate (yaw controller convention)
     */
    float desired_yaw_rate = -error * TURNING_KP;

    /* Ensure minimum yaw rate for smooth approach
     * error > 0 → need CCW → need negative rate (yaw controller convention)
     */
    if (abs_f(desired_yaw_rate) < TURNING_MIN_YAW_RATE) {
        desired_yaw_rate = (error > 0) ? -TURNING_MIN_YAW_RATE : TURNING_MIN_YAW_RATE;
    }

    /* Clamp to max yaw rate */
    if (desired_yaw_rate > AUTONAV_YAW_RATE_MAX) desired_yaw_rate = AUTONAV_YAW_RATE_MAX;
    if (desired_yaw_rate < -AUTONAV_YAW_RATE_MAX) desired_yaw_rate = -AUTONAV_YAW_RATE_MAX;

    /* Set target for inner yaw rate controller */
    yaw_controller_set_target(desired_yaw_rate);

    /* Send zero linear (turning in place) */
    struct motor_cmd cmd = {
        .linear = 0,
        .angular = 0,  /* Ignored - yaw controller provides angular */
    };
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

/* Scanning parameters */
#define SCAN_YAW_RATE       10.0f   /* Slow scan speed (°/s) */
#define SCAN_CLEAR_DIST     400.0f  /* Distance threshold for "clear" (mm) */
#define SCAN_MAX_ANGLE      180.0f  /* Maximum scan angle before giving up */

/**
 * Process scanning state - continuous slow scan to find clear path
 *
 * Strategy:
 * 1. Turn slowly (10°/s) in scan direction
 * 2. Continuously check IR distance
 * 3. As soon as distance > 400mm → found clear path, go there
 * 4. If turned 180° without finding clear path → reverse scan direction
 * 5. If still no path after 360° total → back up and retry
 *
 * scan_phase: 0 = scanning left (CCW), 1 = scanning right (CW)
 */
static void process_scanning(void)
{
    float current = get_current_heading();
    float angle_scanned = abs_f(heading_delta(current, scan_origin_heading));

    /* Get current IR distance */
    float left_mm, right_mm;
    ir_sensors_get_distance(&left_mm, &right_mm);
    float min_dist = (left_mm < right_mm) ? left_mm : right_mm;

    /* Send linear=0 while scanning */
    struct motor_cmd cmd = { .linear = 0, .angular = 0 };
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);

    /* Check if we found a clear path */
    if (min_dist >= SCAN_CLEAR_DIST) {
        printk("SCAN: Found clear path at heading %.0f (dist=%.0f mm)\n",
               (double)current, (double)min_dist);
        yaw_controller_set_target(0.0f);
        target_heading = current;
        transition_to(AUTONAV_HEADING_HOLD);
        return;
    }

    /* Check if we've scanned far enough in current direction */
    if (angle_scanned >= SCAN_MAX_ANGLE) {
        if (scan_phase == 0) {
            /* Finished scanning left, now scan right */
            printk("SCAN: No clear path left, trying right\n");
            scan_phase = 1;
            scan_origin_heading = current;  /* Reset origin for right scan */
        } else {
            /* Scanned both directions, no clear path found */
            printk("SCAN: No clear path found! Backing up...\n");
            yaw_controller_set_target(0.0f);
            /* Just pick a direction (right) and hope for the best */
            target_heading = normalize_heading(current + 90.0f);
            transition_to(AUTONAV_TURNING);
            return;
        }
    }

    /* Continue scanning in current direction
     * scan_phase 0 = CCW → negative yaw rate (yaw controller convention)
     * scan_phase 1 = CW → positive yaw rate
     */
    float scan_rate = (scan_phase == 0) ? -SCAN_YAW_RATE : SCAN_YAW_RATE;
    yaw_controller_set_target(scan_rate);

    /* Debug output every ~1 second */
    static int scan_debug_counter = 0;
    if (++scan_debug_counter >= 20) {
        scan_debug_counter = 0;
        printk("SCAN: phase=%d angle=%.0f dist=%.0f\n",
               scan_phase, (double)angle_scanned, (double)min_dist);
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

        /* Skip if disabled */
        if (!enabled || current_state == AUTONAV_DISABLED) {
            continue;
        }

        /* Process based on current state */
        switch (current_state) {
        case AUTONAV_HEADING_HOLD:
            process_heading_hold();
            break;

        case AUTONAV_TURNING:
            process_turning();
            break;

        case AUTONAV_SCANNING:
            process_scanning();
            break;

        default:
            break;
        }
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
    k_mutex_lock(&autonav_mutex, K_FOREVER);

    if (enabled) {
        k_mutex_unlock(&autonav_mutex);
        return;  /* Already enabled */
    }

    /* Capture current heading as target */
    target_heading = get_current_heading();

    printk("AUTONAV: Enabled, target heading=%.1f\n", (double)target_heading);
    enabled = true;
    transition_to(AUTONAV_HEADING_HOLD);

    k_mutex_unlock(&autonav_mutex);

    /* Play sound outside mutex to avoid blocking */
    audio_play(SOUND_PAIRING_START);
}

void autonav_disable(void)
{
    k_mutex_lock(&autonav_mutex, K_FOREVER);

    if (!enabled && current_state == AUTONAV_DISABLED) {
        k_mutex_unlock(&autonav_mutex);
        return;  /* Already disabled */
    }

    printk("AUTONAV: Disabled\n");
    enabled = false;
    transition_to(AUTONAV_DISABLED);

    k_mutex_unlock(&autonav_mutex);

    /* Reset yaw controller target (manual control takes over via triggers) */
    yaw_controller_set_target(0.0f);

    /* Stop motors and play sound outside mutex */
    motor_emergency_stop();
    audio_play(SOUND_DISCONNECTED);
}

bool autonav_is_enabled(void)
{
    k_mutex_lock(&autonav_mutex, K_FOREVER);
    bool result = enabled;
    k_mutex_unlock(&autonav_mutex);
    return result;
}

enum autonav_state autonav_get_state(void)
{
    k_mutex_lock(&autonav_mutex, K_FOREVER);
    enum autonav_state result = current_state;
    k_mutex_unlock(&autonav_mutex);
    return result;
}

void autonav_manual_override(void)
{
    k_mutex_lock(&autonav_mutex, K_FOREVER);

    if (!enabled) {
        k_mutex_unlock(&autonav_mutex);
        return;  /* Not in autonomous mode */
    }

    printk("AUTONAV: Manual override detected!\n");
    enabled = false;
    transition_to(AUTONAV_DISABLED);

    k_mutex_unlock(&autonav_mutex);

    /* Reset yaw controller target (manual control takes over via triggers) */
    yaw_controller_set_target(0.0f);

    /* Stop motors and play sound outside mutex */
    motor_emergency_stop();
    audio_play(SOUND_OBSTACLE);
}

void autonav_turn_relative(int16_t degrees)
{
    k_mutex_lock(&autonav_mutex, K_FOREVER);

    if (!enabled) {
        k_mutex_unlock(&autonav_mutex);
        printk("AUTONAV: Turn ignored (not enabled)\n");
        return;
    }

    /* Ignore if already turning or scanning */
    if (current_state == AUTONAV_TURNING || current_state == AUTONAV_SCANNING) {
        k_mutex_unlock(&autonav_mutex);
        printk("AUTONAV: Turn ignored (busy: state=%d)\n", current_state);
        return;
    }

    /* Update target heading */
    target_heading = normalize_heading(target_heading + (float)degrees);

    printk("AUTONAV: Turn %+d degrees, new target=%.1f\n",
           degrees, (double)target_heading);

    /* Transition to turning state */
    transition_to(AUTONAV_TURNING);

    k_mutex_unlock(&autonav_mutex);

    /* Play sound outside mutex */
    audio_play(SOUND_BUTTON_PRESS);
}

float autonav_get_target_heading(void)
{
    k_mutex_lock(&autonav_mutex, K_FOREVER);

    float result = -1.0f;
    if (enabled &&
        (current_state == AUTONAV_HEADING_HOLD ||
         current_state == AUTONAV_TURNING)) {
        result = target_heading;
    }

    k_mutex_unlock(&autonav_mutex);
    return result;
}
