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

/* Heading-hold target (degrees, 0-360) */
static float target_heading;

/* Yaw test tracking */
static float yaw_test_start_heading;
static float yaw_test_cw_delta;
static float yaw_test_ccw_delta;

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
        /* Forward motion - angular handled separately by process_heading_hold */
        cmd.linear = AUTONAV_SPEED_LINEAR;
        cmd.angular = 0;
        break;

    case AUTONAV_TURNING:
        /* Turning in place - angular set by process_turning */
        cmd.linear = 0;
        cmd.angular = 0;
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
 * Process heading-hold state - drive forward while maintaining target heading
 * Uses Kalman-filtered IR distance for proportional obstacle avoidance
 */
static void process_heading_hold(void)
{
    /* Get Kalman-filtered distances in mm */
    float left_mm, right_mm;
    ir_sensors_get_distance(&left_mm, &right_mm);

    /* Find minimum distance (closest obstacle) */
    float min_dist = (left_mm < right_mm) ? left_mm : right_mm;

    /* Critical: both sides too close → back up */
    if (left_mm < OBSTACLE_DIST_CRITICAL && right_mm < OBSTACLE_DIST_CRITICAL) {
        printk("AUTONAV: Critical! L=%.0f R=%.0f mm, backing up\n",
               (double)left_mm, (double)right_mm);
        transition_to(AUTONAV_BACKING_UP);
        return;
    }

    /* Calculate base heading correction
     * Note: positive angular = turn right (CW) = heading decreases
     *       negative angular = turn left (CCW) = heading increases
     * If target > current (error > 0), we need CCW to increase heading → negative angular
     */
    float current = get_current_heading();
    float heading_error = heading_delta(target_heading, current);
    float angular_heading = -heading_error * HEADING_KP;  /* Negate: error>0 needs CCW (negative) */

    /* Debug: Log heading correction every ~1 second */
    static int debug_counter = 0;
    if (++debug_counter >= 20) {  /* 20 * 50ms = 1s */
        debug_counter = 0;
        printk("HEADING: target=%.1f current=%.1f error=%+.1f angular=%+.1f\n",
               (double)target_heading, (double)current,
               (double)heading_error, (double)angular_heading);
    }

    /* Calculate obstacle avoidance correction */
    float angular_avoid = 0.0f;

    if (min_dist < OBSTACLE_DIST_AVOID) {
        /*
         * Proportional avoidance based on distance difference:
         * - diff > 0 → right has more space → turn right (positive angular)
         * - diff < 0 → left has more space → turn left (negative angular)
         *
         * Strength increases as obstacle gets closer
         */
        float diff = right_mm - left_mm;

        /* Apply deadzone for small differences */
        if (abs_f(diff) > OBSTACLE_DIFF_DEADZONE) {
            /* Scale avoidance by proximity (closer = stronger response) */
            float proximity_factor = 1.0f - (min_dist / OBSTACLE_DIST_AVOID);
            proximity_factor = (proximity_factor < 0.0f) ? 0.0f : proximity_factor;

            angular_avoid = diff * OBSTACLE_KP_AVOID * (1.0f + proximity_factor);

            printk("AUTONAV: Avoid L=%.0f R=%.0f diff=%+.0f → angular=%+.1f\n",
                   (double)left_mm, (double)right_mm, (double)diff, (double)angular_avoid);
        }
    }

    /* Combine heading hold + obstacle avoidance */
    float angular_total = angular_heading + angular_avoid;
    int16_t angular = clamp_i16((int16_t)angular_total,
                                -AUTONAV_SPEED_ANGULAR, AUTONAV_SPEED_ANGULAR);

    /* Reduce forward speed when avoiding (proportional to avoidance effort) */
    float speed_factor = 1.0f - (abs_f(angular_avoid) / (float)AUTONAV_SPEED_ANGULAR) * 0.5f;
    if (speed_factor < 0.3f) speed_factor = 0.3f;
    int16_t linear = (int16_t)(AUTONAV_SPEED_LINEAR * speed_factor);

    struct motor_cmd cmd = {
        .linear = linear,
        .angular = angular,
    };
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

/**
 * Process turning state - rotate in place to reach target heading
 * Uses proportional control with minimum speed for smooth approach
 * Also checks for obstacles and enforces timeout
 */
static void process_turning(void)
{
    int64_t elapsed = k_uptime_get() - state_start_time;

    /* Check for timeout (60 seconds) */
    if (elapsed >= AUTONAV_TURNING_TIMEOUT) {
        printk("AUTONAV: Turning timeout! Resuming heading-hold\n");
        target_heading = get_current_heading();  /* Accept current heading */
        transition_to(AUTONAV_HEADING_HOLD);
        return;
    }

    /* Check for critical obstacles using Kalman-filtered IR */
    float left_mm, right_mm;
    ir_sensors_get_distance(&left_mm, &right_mm);

    if (left_mm < OBSTACLE_DIST_CRITICAL && right_mm < OBSTACLE_DIST_CRITICAL) {
        printk("AUTONAV: Critical obstacle while turning! L=%.0f R=%.0f mm\n",
               (double)left_mm, (double)right_mm);
        transition_to(AUTONAV_BACKING_UP);
        return;
    }

    float current = get_current_heading();
    float error = heading_delta(target_heading, current);

    /* Check if target reached */
    if (abs_f(error) < HEADING_TOLERANCE) {
        printk("AUTONAV: Target heading reached (%.1f)\n", (double)target_heading);
        transition_to(AUTONAV_HEADING_HOLD);
        return;
    }

    /* Proportional turn rate with minimum speed
     * Note: error > 0 means target > current, need CCW (negative angular)
     */
    float angular_f = -error * TURNING_KP;

    /* Apply minimum speed (maintain sign for direction)
     * error > 0 → need CCW → negative angular
     */
    if (abs_f(angular_f) < TURNING_MIN_SPEED) {
        angular_f = (error > 0) ? -TURNING_MIN_SPEED : TURNING_MIN_SPEED;
    }

    int16_t angular = clamp_i16((int16_t)angular_f,
                                -AUTONAV_SPEED_ANGULAR, AUTONAV_SPEED_ANGULAR);

    struct motor_cmd cmd = {
        .linear = 0,
        .angular = angular,
    };
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

/* ============================================================================
 * Backing Up Logic
 * ============================================================================ */

/**
 * Calculate proportional turn angle based on Kalman-filtered IR distances
 *
 * The turn angle depends on:
 * 1. Difference between left/right (bigger diff → turn more toward open side)
 * 2. Tightness of situation (less space → need bigger turn)
 *
 * @param left_mm  Kalman-filtered left distance
 * @param right_mm Kalman-filtered right distance
 * @return Turn angle in degrees to add to target_heading
 *         (positive = CCW/left = heading increases, negative = CW/right = heading decreases)
 */
static float calculate_turn_angle(float left_mm, float right_mm)
{
    float diff = right_mm - left_mm;  /* positive = more space on right */
    float min_dist = (left_mm < right_mm) ? left_mm : right_mm;

    /* Both sides similar (diff < 50mm) → U-turn */
    if (abs_f(diff) < 50.0f) {
        return 180.0f;
    }

    /*
     * Proportional turn angle calculation:
     *
     * diff_factor (0-1): How much more space on one side
     *   - 0 = equal space
     *   - 1 = 400mm+ difference (one side much freer)
     *
     * tight_factor (0-1): How tight the situation is
     *   - 0 = plenty of space (min_dist >= OBSTACLE_DIST_AVOID)
     *   - 1 = very tight (min_dist = 0)
     *
     * Final angle = 45° base + up to 45° for diff + up to 45° for tightness
     * Range: 45° to 135°
     */
    float diff_factor = abs_f(diff) / 400.0f;
    if (diff_factor > 1.0f) {
        diff_factor = 1.0f;
    }

    float tight_factor = 1.0f - (min_dist / OBSTACLE_DIST_AVOID);
    if (tight_factor < 0.0f) {
        tight_factor = 0.0f;
    }
    if (tight_factor > 1.0f) {
        tight_factor = 1.0f;
    }

    /* Calculate angle magnitude: 45° base + proportional components */
    float turn_angle = 45.0f + diff_factor * 45.0f + tight_factor * 45.0f;

    /* Apply direction:
     * - More space on right (diff > 0) → turn right (CW) → heading decreases → negative
     * - More space on left (diff < 0) → turn left (CCW) → heading increases → positive
     */
    if (diff > 0.0f) {
        turn_angle = -turn_angle;
    }

    return turn_angle;
}

/**
 * Process backing up state - reverse from obstacle, then adjust heading
 * Uses proportional turn angle based on Kalman-filtered IR distances
 */
static void process_backing_up(void)
{
    int64_t elapsed = k_uptime_get() - state_start_time;

    /* Wait for backup duration */
    if (elapsed < AUTONAV_BACKUP_DURATION) {
        return;
    }

    /* Get Kalman-filtered distance to check if safe */
    float left_mm, right_mm;
    ir_sensors_get_distance(&left_mm, &right_mm);
    float min_dist = (left_mm < right_mm) ? left_mm : right_mm;

    if (min_dist > OBSTACLE_DIST_CRITICAL) {
        /* Safe to resume - calculate proportional turn angle */
        float turn_angle = calculate_turn_angle(left_mm, right_mm);

        printk("AUTONAV: Backed up (L=%.0f R=%.0f mm), turn %.0f deg\n",
               (double)left_mm, (double)right_mm, (double)turn_angle);

        /* Update target heading */
        target_heading = normalize_heading(target_heading + turn_angle);
        printk("AUTONAV: New target heading = %.1f\n", (double)target_heading);

        /* Transition to turning to reach new heading */
        transition_to(AUTONAV_TURNING);
    } else {
        /* Still too close, keep backing */
        printk("AUTONAV: Still blocked (%.0f mm), continuing backup\n",
               (double)min_dist);
        state_start_time = k_uptime_get();
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

        /* Process based on current state */
        switch (current_state) {
        case AUTONAV_HEADING_HOLD:
            process_heading_hold();
            break;

        case AUTONAV_TURNING:
            process_turning();
            break;

        case AUTONAV_BACKING_UP:
            process_backing_up();
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
    if (enabled) {
        return;  /* Already enabled */
    }

    /* Capture current heading as target */
    target_heading = get_current_heading();

    printk("AUTONAV: Enabled, target heading=%.1f\n", (double)target_heading);
    enabled = true;
    audio_play(SOUND_PAIRING_START);  /* Ascending tone */
    transition_to(AUTONAV_HEADING_HOLD);
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

void autonav_turn_relative(int16_t degrees)
{
    if (!enabled) {
        printk("AUTONAV: Turn ignored (not enabled)\n");
        return;
    }

    /* Update target heading */
    target_heading = normalize_heading(target_heading + (float)degrees);

    printk("AUTONAV: Turn %+d degrees, new target=%.1f\n",
           degrees, (double)target_heading);

    /* Transition to turning state */
    audio_play(SOUND_BUTTON_PRESS);
    transition_to(AUTONAV_TURNING);
}

float autonav_get_target_heading(void)
{
    if (!enabled ||
        (current_state != AUTONAV_HEADING_HOLD &&
         current_state != AUTONAV_TURNING)) {
        return -1.0f;
    }
    return target_heading;
}
