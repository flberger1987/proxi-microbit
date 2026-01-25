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
    AUTONAV_YAW_TEST_CW,        /* Yaw test: rotating clockwise */
    AUTONAV_YAW_TEST_CCW,       /* Yaw test: rotating counter-clockwise */
    AUTONAV_YAW_TEST_DONE,      /* Yaw test: complete, showing results */
    AUTONAV_HEADING_HOLD,       /* Driving forward, holding target heading */
    AUTONAV_TURNING,            /* Turning to new target heading */
    AUTONAV_BACKING_UP,         /* Reversing (both sides blocked) */
};

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/* Motor speeds for autonomous navigation */
#define AUTONAV_SPEED_LINEAR     100  /* Forward speed (100%) */
#define AUTONAV_SPEED_ANGULAR    60   /* Turn speed */
#define AUTONAV_SPEED_BACKUP    -50   /* Backup speed */

/* Heading control parameters */
#define HEADING_KP               1.5f /* Proportional gain for heading correction */
#define HEADING_TOLERANCE        5.0f /* Target reached when error < this (degrees) */

/* Turning control parameters (in-place rotation to target heading) */
#define TURNING_KP               0.67f /* Proportional gain: 90° error → 60% speed */
#define TURNING_MIN_SPEED        25    /* Minimum angular speed (~20°/s) */

/* Obstacle avoidance parameters (using Kalman-filtered mm distance) */
#define OBSTACLE_DIST_CRITICAL   150.0f  /* Back up if closer than this (mm) */
#define OBSTACLE_DIST_AVOID      350.0f  /* Start avoiding if closer than this (mm) */
#define OBSTACLE_KP_AVOID        0.3f    /* Proportional gain for avoidance (angular/mm) */
#define OBSTACLE_DIFF_DEADZONE   30.0f   /* Ignore small differences (mm) */

/* Timing (ms) */
#define AUTONAV_BACKUP_DURATION  500   /* How long to back up */
#define AUTONAV_TURNING_TIMEOUT  60000 /* Max time for turning (60 sec) */
#define AUTONAV_YAW_TEST_DURATION 5000 /* Duration of each yaw test phase (5 sec) */

/* Yaw test speed (100% for clear measurements) */
#define AUTONAV_YAW_TEST_SPEED   100

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
 * Start the yaw axis test
 * Robot will rotate CW then CCW while logging heading changes
 */
void autonav_start_yaw_test(void);

/**
 * Check if yaw test is running
 *
 * @return true if yaw test is in progress
 */
bool autonav_is_yaw_test_running(void);

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

#endif /* AUTONOMOUS_NAV_H */
