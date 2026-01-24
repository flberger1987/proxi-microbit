/*
 * SPDX-License-Identifier: Apache-2.0
 * Orientation calculation implementation
 */

#include "orientation.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float rad_to_deg(float rad)
{
    return rad * (180.0f / M_PI);
}

float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

void orientation_calc_roll_pitch(float ax, float ay, float az,
                                  float *roll, float *pitch)
{
    /* Roll: rotation around X axis
     * roll = atan2(ay, az)
     */
    *roll = rad_to_deg(atan2f(ay, az));

    /* Pitch: rotation around Y axis
     * pitch = atan2(-ax, sqrt(ay^2 + az^2))
     */
    *pitch = rad_to_deg(atan2f(-ax, sqrtf(ay * ay + az * az)));
}

float orientation_calc_heading(float mx, float my, float mz,
                                float roll, float pitch,
                                const struct mag_calibration *cal)
{
    float mx_cal, my_cal, mz_cal;
    float roll_rad, pitch_rad;
    float xh, yh;
    float heading;

    /* Apply hard-iron calibration */
    if (cal != NULL && cal->valid) {
        mx_cal = mx - cal->offset_x;
        my_cal = my - cal->offset_y;
        mz_cal = mz - cal->offset_z;
    } else {
        mx_cal = mx;
        my_cal = my;
        mz_cal = mz;
    }

    /* Convert roll and pitch to radians */
    roll_rad = deg_to_rad(roll);
    pitch_rad = deg_to_rad(pitch);

    /* Tilt compensation
     * Transform magnetometer readings to horizontal plane
     *
     * Xh = mx*cos(pitch) + my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch)
     * Yh = my*cos(roll) - mz*sin(roll)
     */
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);

    xh = mx_cal * cos_pitch +
         my_cal * sin_roll * sin_pitch +
         mz_cal * cos_roll * sin_pitch;

    yh = my_cal * cos_roll - mz_cal * sin_roll;

    /* Calculate heading
     * heading = atan2(-Yh, Xh)
     */
    heading = rad_to_deg(atan2f(-yh, xh));

    /* Normalize to 0-360 range */
    if (heading < 0.0f) {
        heading += 360.0f;
    }

    return heading;
}
