/*
 * SPDX-License-Identifier: Apache-2.0
 * Autonomous Navigation Module
 *
 * Implements autonomous navigation with obstacle avoidance for the
 * Kosmos Proxi hexapod robot. Activated via D-Pad, with manual override.
 */

#ifndef AUTONOMOUS_NAV_H
#define AUTONOMOUS_NAV_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Autonomous Navigation States
 * ============================================================================ */

/**
 * Autonomous navigation state machine
 */
enum autonav_state {
    AUTONAV_DISABLED,           /* Navigation disabled, manual control */
    AUTONAV_HEADING_HOLD,       /* Driving forward with continuous heading correction */
    AUTONAV_TURNING,            /* Turning to new target heading (in place) */
    AUTONAV_SCANNING,           /* Obstacle detected: scanning left/right for best path */
};

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/* Motor speeds for autonomous navigation */
#define AUTONAV_SPEED_LINEAR     100  /* Forward speed (100%) */
#define AUTONAV_SPEED_ANGULAR    60   /* Turn speed */
#define AUTONAV_SPEED_BACKUP    -50   /* Backup speed */

/* Heading control - PI controller with anti-windup for continuous correction */
#define HEADING_KP               0.5f  /* Proportional gain (reduced from 1.0 for smoother response) */
#define HEADING_KI               0.05f /* Integral gain (eliminates steady-state error) */
#define HEADING_KD               0.1f  /* Derivative gain (damps oscillations) */
#define HEADING_D_MAX            5.0f  /* Maximum D term contribution (°/s) */
#define HEADING_I_MAX            5.0f  /* Integral windup limit (°/s contribution) */
#define HEADING_TOLERANCE        2.0f  /* Target reached when error < this (degrees) */
#define AUTONAV_YAW_RATE_MAX    12.0f  /* Maximum yaw rate for autonomous nav (°/s) - reduced for stability */

/* Turning control parameters (in-place rotation to target heading) */
#define TURNING_KP               0.8f  /* Proportional gain: 90° error → 72% speed */
#define TURNING_MIN_YAW_RATE    10.0f  /* Minimum yaw rate for turning (°/s) */

/* Obstacle parameters */
#define OBSTACLE_DIST_STOP       250.0f  /* Stop and scan if closer than this (mm) */
#define OBSTACLE_DIST_SLOW       400.0f  /* Slow down if closer than this (mm) */

/* Scanning parameters */
#define SCAN_ANGLE               90.0f  /* How far to look left/right (degrees) */

/* Timing (ms) */
#define AUTONAV_TURNING_TIMEOUT  30000 /* Max time for turning (30 sec) */
#define SCAN_SETTLE_TIME         300   /* Wait time after reaching scan position (ms) */

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * Initialize the autonomous navigation module
 *
 * @return 0 on success, negative error code on failure
 */
int autonav_init(void);

/**
 * Start the autonomous navigation thread
 */
void autonav_start_thread(void);

/**
 * Enable autonomous navigation mode
 * Robot will start exploring and avoiding obstacles
 */
void autonav_enable(void);

/**
 * Disable autonomous navigation mode
 * Returns control to manual input
 */
void autonav_disable(void);

/**
 * Check if autonomous navigation is enabled
 *
 * @return true if autonomous mode is active
 */
bool autonav_is_enabled(void);

/**
 * Get current autonomous navigation state
 *
 * @return Current state from autonav_state enum
 */
enum autonav_state autonav_get_state(void);

/**
 * Trigger manual override
 * Called when manual input is detected during autonomous mode
 */
void autonav_manual_override(void);

/**
 * Initiate a relative turn by the specified degrees
 * Only effective when autonomous mode is enabled.
 *
 * @param degrees Relative turn amount (+90 for left/CCW, -90 for right/CW, 180 for U-turn)
 */
void autonav_turn_relative(int16_t degrees);

/**
 * Get the current target heading
 *
 * @return Target heading in degrees (0-360), or -1 if not in heading-hold mode
 */
float autonav_get_target_heading(void);

/**
 * Set heading controller PID parameters (runtime tuning)
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param i_max Integral windup limit
 * @param d_max Derivative term limit
 * @param yaw_max Maximum yaw rate command
 */
void autonav_set_pid_params(float kp, float ki, float kd, float i_max, float d_max, float yaw_max);

/**
 * Get current heading controller PID parameters
 *
 * @param kp Output: Proportional gain (may be NULL)
 * @param ki Output: Integral gain (may be NULL)
 * @param kd Output: Derivative gain (may be NULL)
 * @param i_max Output: Integral windup limit (may be NULL)
 * @param d_max Output: Derivative term limit (may be NULL)
 * @param yaw_max Output: Maximum yaw rate command (may be NULL)
 */
void autonav_get_pid_params(float *kp, float *ki, float *kd, float *i_max, float *d_max, float *yaw_max);

/**
 * Save PID parameters to flash
 *
 * @return 0 on success, negative error code on failure
 */
int autonav_save_pid_params(void);

#endif /* AUTONOMOUS_NAV_H */
