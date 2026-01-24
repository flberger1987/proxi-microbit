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
    /* Raw accelerometer values (before axis transformation) */
    int16_t raw_ax;  /* Raw accel X in milli-g */
    int16_t raw_ay;  /* Raw accel Y in milli-g */
    int16_t raw_az;  /* Raw accel Z in milli-g */
    /* Raw magnetometer values (before axis transformation) */
    int16_t raw_mx;  /* Raw mag X in milli-Gauss */
    int16_t raw_my;  /* Raw mag Y in milli-Gauss */
    int16_t raw_mz;  /* Raw mag Z in milli-Gauss */
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

/* Message queue for orientation data (accessible by output thread) */
extern struct k_msgq orientation_msgq;

#endif /* SENSORS_H */
