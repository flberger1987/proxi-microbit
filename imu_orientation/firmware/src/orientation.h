/*
 * SPDX-License-Identifier: Apache-2.0
 * Orientation calculation from accelerometer and magnetometer data
 */

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdint.h>
#include <stdbool.h>

/* Orientation data structure */
struct orientation_data {
    float roll;           /* Roll angle in degrees (-180 to +180) */
    float pitch;          /* Pitch angle in degrees (-90 to +90) */
    float heading;        /* Heading/yaw in degrees (0 to 360) */
    uint32_t timestamp_ms;
};

/**
 * Magnetometer calibration data
 *
 * Hard-Iron: Constant offsets caused by permanent magnets nearby
 *   m_centered = m_raw - offset
 *
 * Soft-Iron: Scale factors to transform ellipsoid to sphere
 *   m_calibrated = scale * m_centered
 *
 * Full calibration: m_cal = scale * (m_raw - offset)
 */
struct mag_calibration {
    /* Hard-Iron offsets (ellipsoid center) */
    float offset_x;
    float offset_y;
    float offset_z;

    /* Soft-Iron scale factors (ellipsoid → sphere) */
    float scale_x;
    float scale_y;
    float scale_z;

    /* Calibration version for flash compatibility */
    uint8_t version;
    bool valid;
};

/* Current calibration version - increment when struct changes */
#define MAG_CAL_VERSION 2

/**
 * Calculate roll and pitch from accelerometer data
 *
 * @param ax Accelerometer X (m/s^2)
 * @param ay Accelerometer Y (m/s^2)
 * @param az Accelerometer Z (m/s^2)
 * @param roll Output roll angle in degrees
 * @param pitch Output pitch angle in degrees
 */
void orientation_calc_roll_pitch(float ax, float ay, float az,
                                  float *roll, float *pitch);

/**
 * Calculate tilt-compensated heading from magnetometer data
 *
 * @param mx Magnetometer X (Gauss)
 * @param my Magnetometer Y (Gauss)
 * @param mz Magnetometer Z (Gauss)
 * @param roll Roll angle in degrees
 * @param pitch Pitch angle in degrees
 * @param cal Magnetometer calibration data
 * @return Heading in degrees (0 to 360)
 */
float orientation_calc_heading(float mx, float my, float mz,
                                float roll, float pitch,
                                const struct mag_calibration *cal);

/**
 * Convert radians to degrees
 */
float rad_to_deg(float rad);

/**
 * Convert degrees to radians
 */
float deg_to_rad(float deg);

#endif /* ORIENTATION_H */
