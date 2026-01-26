/*
 * SPDX-License-Identifier: Apache-2.0
 * Sensor reading and calibration
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <zephyr/kernel.h>
#include "orientation.h"

/* Sensor data message for inter-thread communication */
struct sensor_msg {
    struct orientation_data orientation;
    float yaw_rate;  /* Yaw rate in degrees per second (derived from heading) */
    /* Accelerometer values in USER coordinates (after transformation) */
    int16_t raw_ax;  /* User-X (forward) in milli-g */
    int16_t raw_ay;  /* User-Y (left) in milli-g */
    int16_t raw_az;  /* User-Z (up/USB) in milli-g */
    /* Magnetometer values in USER coordinates (after transformation) */
    int16_t raw_mx;  /* User-X (forward) in milli-Gauss */
    int16_t raw_my;  /* User-Y (left) in milli-Gauss */
    int16_t raw_mz;  /* User-Z (up/USB) in milli-Gauss */
};

/* Initialize sensors (accelerometer and magnetometer) */
int sensors_init(void);

/* Start the sensor reading thread (50 Hz) */
void sensors_start_thread(void);

/* Start magnetometer calibration
 * User should rotate the device in all directions
 * Calibration runs for approximately 10 seconds
 */
void sensors_start_calibration(void);

/* Check if calibration is in progress */
bool sensors_is_calibrating(void);

/* Get current calibration data */
const struct mag_calibration *sensors_get_calibration(void);

/* Get current heading (thread-safe, updated at 50Hz) */
float sensors_get_heading(void);

/* Get current yaw rate in degrees/second (thread-safe, updated at 50Hz) */
float sensors_get_yaw_rate(void);

/* Set current motor command for Kalman filter prediction (called by motor driver) */
void sensors_set_motor_cmd(float pwm_percent);

/* Get full current orientation data (thread-safe) */
void sensors_get_orientation(struct orientation_data *out);

/* Get raw accelerometer values (User coordinates, milli-g) */
void sensors_get_raw_accel(int16_t *ax, int16_t *ay, int16_t *az);

/* Get raw magnetometer values (User coordinates, milli-Gauss) */
void sensors_get_raw_mag(int16_t *mx, int16_t *my, int16_t *mz);

/* Message queue for orientation data (accessible by output thread) */
extern struct k_msgq orientation_msgq;

#endif /* SENSORS_H */
