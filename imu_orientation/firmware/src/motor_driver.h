/*
 * SPDX-License-Identifier: Apache-2.0
 * Motor Driver - PWM Control for Kosmos Proxi
 *
 * Differential drive motor control using PWM on P0 (left) and P1 (right).
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the motor driver
 *
 * @return 0 on success, negative errno on failure
 */
int motor_driver_init(void);

/**
 * Start the motor driver thread
 * Processes motor commands from motor_cmd_q
 */
void motor_driver_start_thread(void);

/**
 * Set motor velocities directly (for testing)
 *
 * @param left_percent Left motor speed (-100 to +100)
 * @param right_percent Right motor speed (-100 to +100)
 */
void motor_set_speeds(int16_t left_percent, int16_t right_percent);

/**
 * Set differential drive velocity
 *
 * @param linear Forward/backward velocity (-100 to +100)
 * @param angular Turn velocity (-100 to +100), positive = turn right
 */
void motor_set_velocity(int16_t linear, int16_t angular);

/**
 * Emergency stop - immediately halt both motors
 */
void motor_emergency_stop(void);

/**
 * Check if motors are currently running
 *
 * @return true if any motor is active
 */
bool motor_is_active(void);

/**
 * Enable/disable motor output
 *
 * @param enable true to enable motors, false to disable
 */
void motor_enable(bool enable);

/**
 * Get current motor command values (for telemetry)
 *
 * @param linear Pointer to store linear velocity (-100 to +100)
 * @param angular Pointer to store angular velocity (-100 to +100)
 */
void motor_get_current_cmd(int8_t *linear, int8_t *angular);

#endif /* MOTOR_DRIVER_H */
