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
    AUTONAV_EXPLORING,          /* Moving forward, looking for obstacles */
    AUTONAV_AVOIDING_LEFT,      /* Turning right (obstacle on left) */
    AUTONAV_AVOIDING_RIGHT,     /* Turning left (obstacle on right) */
    AUTONAV_BACKING_UP,         /* Reversing (both sides blocked) */
};

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/* Motor speeds for autonomous navigation */
#define AUTONAV_SPEED_LINEAR     50   /* Forward/backward speed */
#define AUTONAV_SPEED_ANGULAR    60   /* Turn speed */
#define AUTONAV_SPEED_BACKUP    -40   /* Backup speed */

/* Obstacle detection thresholds (mm) */
#define AUTONAV_OBSTACLE_START   250  /* Start avoiding at this distance */
#define AUTONAV_OBSTACLE_CRIT    150  /* Back up at this distance */

/* Timing (ms) */
#define AUTONAV_BACKUP_DURATION  500  /* How long to back up */
#define AUTONAV_TURN_DURATION    800  /* How long to turn */
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

#endif /* AUTONOMOUS_NAV_H */
