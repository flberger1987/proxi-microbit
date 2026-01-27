/*
 * SPDX-License-Identifier: Apache-2.0
 * Yaw Rate Controller
 *
 * Closed-loop yaw rate control using magnetometer feedback.
 * Maps Xbox trigger input to target yaw rate and controls
 * the turn motor to achieve the desired rate.
 */

#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* Maximum yaw rate in degrees per second */
#define YAW_RATE_MAX 40.0f

/* PID Controller gains */
#define YAW_KP 1.0f   /* Proportional gain */
#define YAW_KI 0.3f   /* Integral gain (low - feedforward does main work) */
#define YAW_KD 0.3f   /* Derivative gain (reduced from 0.8 - less oscillation) */

/**
 * Initialize the yaw rate controller
 *
 * @return 0 on success, negative errno on failure
 */
int yaw_controller_init(void);

/**
 * Enable/disable yaw rate control mode
 * When disabled, angular commands pass through directly
 *
 * @param enable true to enable closed-loop control
 */
void yaw_controller_enable(bool enable);

/**
 * Check if yaw rate control is enabled
 *
 * @return true if enabled
 */
bool yaw_controller_is_enabled(void);

/**
 * Set target yaw rate from controller triggers
 * L2 = turn left (negative), R2 = turn right (positive)
 *
 * @param left_trigger L2 value (0-1023)
 * @param right_trigger R2 value (0-1023)
 */
void yaw_controller_set_triggers(uint16_t left_trigger, uint16_t right_trigger);

/**
 * Set target yaw rate directly
 *
 * @param target_rate Target yaw rate in degrees/second (clamped to ±YAW_RATE_MAX)
 */
void yaw_controller_set_target(float target_rate);

/**
 * Get current target yaw rate
 *
 * @return Target yaw rate in degrees/second
 */
float yaw_controller_get_target(void);

/**
 * Update controller and compute motor output
 * Call this at a fixed rate (e.g., 50 Hz)
 *
 * @param measured_rate Current measured yaw rate in degrees/second
 * @return Motor angular command (-100 to +100)
 */
int16_t yaw_controller_update(float measured_rate);

/**
 * Reset controller state (clear integral term)
 */
void yaw_controller_reset(void);

/**
 * Get controller debug info
 *
 * @param error Output: current error (target - measured)
 * @param integral Output: current integral term
 * @param output Output: current motor output
 */
void yaw_controller_get_debug(float *error, float *integral, float *output);

#endif /* YAW_CONTROLLER_H */
