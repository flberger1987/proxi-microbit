/*
 * SPDX-License-Identifier: Apache-2.0
 * Mahony AHRS Filter implementation (simplified, no gyroscope)
 */

#include "mahony_filter.h"
#include <math.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Fast inverse square root (Quake III algorithm, improved) */
static float inv_sqrt(float x)
{
    /* Use standard library for accuracy on Cortex-M4F with FPU */
    return 1.0f / sqrtf(x);
}

void mahony_init(struct mahony_filter *filter, float sample_freq,
                 float kp, float ki)
{
    memset(filter, 0, sizeof(*filter));

    /* Initial quaternion: identity (no rotation) */
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;

    /* Clear integral terms */
    filter->integral_fb_x = 0.0f;
    filter->integral_fb_y = 0.0f;
    filter->integral_fb_z = 0.0f;

    /* Magnetic reference not yet captured */
    filter->mag_ref_x = 0.0f;
    filter->mag_ref_z = 0.0f;
    filter->mag_ref_valid = false;

    /* Store parameters */
    filter->kp = kp;
    filter->ki = ki;
    filter->sample_period = 1.0f / sample_freq;

    filter->initialized = true;
}

void mahony_update(struct mahony_filter *filter,
                   float ax, float ay, float az,
                   float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* Use shorter names for readability */
    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    /* Normalise accelerometer measurement */
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* Normalise magnetometer measurement */
    recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    /* Auxiliary variables to avoid repeated arithmetic */
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    /* Reference direction of Earth's magnetic field in earth frame */
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));

    /* Capture magnetic reference on first valid reading */
    if (!filter->mag_ref_valid) {
        filter->mag_ref_x = sqrtf(hx * hx + hy * hy);
        filter->mag_ref_z = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        filter->mag_ref_valid = true;
    }

    /* Use stored magnetic reference (fixed "north") */
    bx = filter->mag_ref_x;
    bz = filter->mag_ref_z;

    /* Estimated direction of gravity and magnetic field */
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    /* Error is sum of cross product between estimated direction and measured
     * direction of gravity and magnetic field */
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    /* Compute and apply integral feedback if enabled */
    if (filter->ki > 0.0f) {
        filter->integral_fb_x += 2.0f * filter->ki * halfex * filter->sample_period;
        filter->integral_fb_y += 2.0f * filter->ki * halfey * filter->sample_period;
        filter->integral_fb_z += 2.0f * filter->ki * halfez * filter->sample_period;
    }

    /* Apply proportional and integral feedback
     * Since we have no gyro, we use the error directly as angular rate estimate */
    float gx = 2.0f * filter->kp * halfex + filter->integral_fb_x;
    float gy = 2.0f * filter->kp * halfey + filter->integral_fb_y;
    float gz = 2.0f * filter->kp * halfez + filter->integral_fb_z;

    /* Integrate rate of change of quaternion */
    gx *= 0.5f * filter->sample_period;
    gy *= 0.5f * filter->sample_period;
    gz *= 0.5f * filter->sample_period;

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalise quaternion */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q0 = q0 * recipNorm;
    filter->q1 = q1 * recipNorm;
    filter->q2 = q2 * recipNorm;
    filter->q3 = q3 * recipNorm;
}

void mahony_update_accel_only(struct mahony_filter *filter,
                              float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    /* Normalise accelerometer measurement */
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* Estimated direction of gravity */
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    /* Error is cross product between estimated and measured direction of gravity */
    halfex = ay * halfvz - az * halfvy;
    halfey = az * halfvx - ax * halfvz;
    halfez = ax * halfvy - ay * halfvx;

    /* Compute and apply integral feedback */
    if (filter->ki > 0.0f) {
        filter->integral_fb_x += 2.0f * filter->ki * halfex * filter->sample_period;
        filter->integral_fb_y += 2.0f * filter->ki * halfey * filter->sample_period;
        filter->integral_fb_z += 2.0f * filter->ki * halfez * filter->sample_period;
    }

    /* Apply feedback */
    float gx = 2.0f * filter->kp * halfex + filter->integral_fb_x;
    float gy = 2.0f * filter->kp * halfey + filter->integral_fb_y;
    float gz = 2.0f * filter->kp * halfez + filter->integral_fb_z;

    /* Integrate rate of change of quaternion */
    gx *= 0.5f * filter->sample_period;
    gy *= 0.5f * filter->sample_period;
    gz *= 0.5f * filter->sample_period;

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalise quaternion */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q0 = q0 * recipNorm;
    filter->q1 = q1 * recipNorm;
    filter->q2 = q2 * recipNorm;
    filter->q3 = q3 * recipNorm;
}

void mahony_get_euler(const struct mahony_filter *filter,
                      float *roll, float *pitch, float *yaw)
{
    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        /* Use 90 degrees if out of range */
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * (180.0f / M_PI);
    }

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);

    /* Normalize yaw to 0-360 */
    if (*yaw < 0.0f) {
        *yaw += 360.0f;
    }
}

/**
 * Direct tilt-compensated compass calculation
 * Works much better than Mahony without a gyroscope
 */
void mahony_get_euler_direct(float ax, float ay, float az,
                             float mx, float my, float mz,
                             float *roll, float *pitch, float *heading)
{
    /* Normalize accelerometer */
    float anorm = sqrtf(ax*ax + ay*ay + az*az);
    if (anorm < 0.001f) anorm = 1.0f;
    ax /= anorm;
    ay /= anorm;
    az /= anorm;

    /* Roll and Pitch from accelerometer (NED convention)
     * Roll: rotation around X (forward) axis
     * Pitch: rotation around Y (right) axis
     */
    *roll = atan2f(ay, az) * (180.0f / M_PI);
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * (180.0f / M_PI);

    /* Tilt-compensated heading from magnetometer
     * Rotate magnetometer reading to horizontal plane
     */
    float roll_rad = *roll * (M_PI / 180.0f);
    float pitch_rad = *pitch * (M_PI / 180.0f);

    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);

    /* Tilt compensation: rotate mag vector to horizontal plane */
    float mx_h = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    float my_h = my * cos_roll - mz * sin_roll;

    /* Heading from horizontal mag components */
    *heading = atan2f(-my_h, mx_h) * (180.0f / M_PI);

    /* Normalize to 0-360 */
    if (*heading < 0.0f) {
        *heading += 360.0f;
    }
}

void mahony_get_quaternion(const struct mahony_filter *filter,
                           float *q0, float *q1, float *q2, float *q3)
{
    *q0 = filter->q0;
    *q1 = filter->q1;
    *q2 = filter->q2;
    *q3 = filter->q3;
}

void mahony_reset(struct mahony_filter *filter)
{
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;

    filter->integral_fb_x = 0.0f;
    filter->integral_fb_y = 0.0f;
    filter->integral_fb_z = 0.0f;

    /* Reset magnetic reference to be recaptured */
    filter->mag_ref_valid = false;
}
