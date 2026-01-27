/*
 * SPDX-License-Identifier: Apache-2.0
 * Mahony AHRS Filter (simplified, no gyroscope)
 *
 * Based on Mahony's paper "Nonlinear Complementary Filters on the Special
 * Orthogonal Group" - adapted for accelerometer + magnetometer only.
 */

#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <stdbool.h>

/**
 * Mahony filter state (quaternion-based)
 */
struct mahony_filter {
    /* Quaternion orientation estimate */
    float q0, q1, q2, q3;

    /* Integral feedback terms */
    float integral_fb_x;
    float integral_fb_y;
    float integral_fb_z;

    /* Magnetic field reference (fixed at initialization) */
    float mag_ref_x;    /* Horizontal component magnitude */
    float mag_ref_z;    /* Vertical component */
    bool mag_ref_valid; /* True once reference is captured */

    /* Tuning parameters */
    float kp;           /* Proportional gain */
    float ki;           /* Integral gain */

    /* Sample period */
    float sample_period;

    /* Initialized flag */
    bool initialized;
};

/**
 * Initialize the Mahony filter
 *
 * @param filter Filter state structure
 * @param sample_freq Sample frequency in Hz (e.g., 50)
 * @param kp Proportional gain (typical: 0.5 - 2.0)
 * @param ki Integral gain (typical: 0.0 - 0.3)
 */
void mahony_init(struct mahony_filter *filter, float sample_freq,
                 float kp, float ki);

/**
 * Update filter with accelerometer and magnetometer data
 *
 * @param filter Filter state
 * @param ax, ay, az Accelerometer readings (any unit, will be normalized)
 * @param mx, my, mz Magnetometer readings (any unit, will be normalized)
 */
void mahony_update(struct mahony_filter *filter,
                   float ax, float ay, float az,
                   float mx, float my, float mz);

/**
 * Update filter with accelerometer only (no magnetometer)
 * Useful when magnetometer is unreliable
 *
 * @param filter Filter state
 * @param ax, ay, az Accelerometer readings
 */
void mahony_update_accel_only(struct mahony_filter *filter,
                              float ax, float ay, float az);

/**
 * Get Euler angles from current quaternion state
 *
 * @param filter Filter state
 * @param roll Output roll angle in degrees
 * @param pitch Output pitch angle in degrees
 * @param yaw Output yaw/heading angle in degrees (0-360)
 */
void mahony_get_euler(const struct mahony_filter *filter,
                      float *roll, float *pitch, float *yaw);

/**
 * Get raw quaternion values
 */
void mahony_get_quaternion(const struct mahony_filter *filter,
                           float *q0, float *q1, float *q2, float *q3);

/**
 * Reset filter to initial state
 */
void mahony_reset(struct mahony_filter *filter);

/**
 * Direct tilt-compensated compass calculation (no filter, no gyro needed)
 * Use this instead of mahony_update + mahony_get_euler when no gyro available.
 *
 * @param ax, ay, az Accelerometer readings (any unit)
 * @param mx, my, mz Magnetometer readings (any unit)
 * @param roll Output roll angle in degrees
 * @param pitch Output pitch angle in degrees
 * @param heading Output heading angle in degrees (0-360)
 */
void mahony_get_euler_direct(float ax, float ay, float az,
                             float mx, float my, float mz,
                             float *roll, float *pitch, float *heading);

/**
 * Compute heading using a reference gravity vector for tilt compensation.
 *
 * This function uses a slowly-filtered gravity direction (reference gravity)
 * to define the stable walking plane, instead of instantaneous accelerometer.
 * This reduces heading oscillation caused by gait-induced tilt.
 *
 * @param ref_gx, ref_gy, ref_gz Reference gravity vector (normalized)
 * @param mx, my, mz Magnetometer readings (any unit, will be normalized)
 * @return heading Heading angle in degrees (0-360)
 */
float mahony_heading_with_ref_gravity(float ref_gx, float ref_gy, float ref_gz,
                                      float mx, float my, float mz);

#endif /* MAHONY_FILTER_H */
