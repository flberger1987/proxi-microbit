/*
 * SPDX-License-Identifier: Apache-2.0
 * Sensor reading implementation
 */

#include "sensors.h"
#include "orientation.h"
#include "mahony_filter.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Sensor devices */
static const struct device *accel_dev;
static const struct device *magn_dev;

/* Mahony AHRS filter */
static struct mahony_filter ahrs_filter;

/* Filter tuning parameters
 * Kp: Proportional gain - higher = faster response, more noise
 * Ki: Integral gain - compensates for gyro drift (less relevant without gyro)
 * Note: Without gyro, we need higher Kp for responsiveness
 */
#define MAHONY_KP  10.0f  /* Proportional gain (high for no-gyro mode) */
#define MAHONY_KI  0.0f   /* Integral gain (disabled, no gyro drift) */

/* Low-pass filter coefficient for output smoothing
 * alpha = 1.0: no filtering (raw values)
 * alpha = 0.1: heavy filtering (slow response)
 */
#define LPF_ALPHA_ROLL    0.15f  /* Roll filtering (accelerometer noise) */
#define LPF_ALPHA_PITCH   0.15f  /* Pitch filtering (accelerometer noise) */
#define LPF_ALPHA_HEADING 1.0f   /* No LPF - Kalman does the filtering */
#define LPF_ALPHA_YAW_RATE 0.1f  /* Yaw rate smoothing */

/* Filtered orientation values */
static float filtered_roll = 0.0f;
static float filtered_pitch = 0.0f;
static float filtered_heading = 0.0f;
static float filtered_yaw_rate = 0.0f;  /* Degrees per second */

/* Savitzky-Golay filter for yaw rate derivation
 *
 * Using 5-point window with 2nd order polynomial for first derivative.
 * Coefficients: [-2, -1, 0, 1, 2] / 10
 *
 * Reference: https://www.cbcity.de/differenzieren-verrauschter-signale
 *
 * Window: 5 samples = 100ms at 50Hz
 * Effective lag: 2 samples = 40ms (centered estimate)
 */
#define SG_WINDOW_SIZE 5
static float heading_buffer[SG_WINDOW_SIZE];
static int heading_buf_idx = 0;
static int heading_buf_count = 0;

/* Savitzky-Golay coefficients for 1st derivative, 5-point, 2nd order polynomial */
static const int sg_coeff[SG_WINDOW_SIZE] = {-2, -1, 0, 1, 2};
#define SG_NORM 10

/* ============================================================================
 * 2D Kalman Filter for Heading and Yaw Rate
 *
 * State vector: x = [θ, ω]^T  (heading in degrees, yaw rate in °/s)
 *
 * System model (motor dynamics):
 *   θ[k+1] = θ[k] + ω[k] × dt
 *   ω[k+1] = A × ω[k] + B × u[k]
 *
 * Measurement: z = θ_magnetometer (direct heading measurement - no derivative!)
 *
 * Identified parameters:
 *   τ = 1.3s, K = 0.4 °/s per PWM%
 * ============================================================================ */

/* Motor model parameters
 * τ_up = 1.3s (acceleration - from sysid)
 * τ_down = 0.3s (deceleration - faster due to friction)
 *
 * Sign convention:
 *   motor_cmd > 0 → CW (right turn) → heading DECREASES
 *   motor_cmd < 0 → CCW (left turn) → heading INCREASES
 *   Therefore MOTOR_K must be NEGATIVE!
 */
#define MOTOR_TAU    0.3f          /* Time constant for stopping (faster) */
#define MOTOR_K      (-0.4f)       /* Gain: °/s per PWM% (NEGATIVE: CW = heading decreases) */
#define DT           0.1f          /* Sample time: 100ms */

/* Discretized model coefficient for ω dynamics */
#define OMEGA_A      (1.0f - DT/MOTOR_TAU)   /* 0.667 */
#define OMEGA_B      (MOTOR_K * DT/MOTOR_TAU) /* 0.133 */

/* Noise parameters */
#define Q_THETA      0.5f          /* Process noise for heading */
#define Q_OMEGA      10.0f         /* Process noise for yaw rate (high = fast response) */
#define R_THETA      3.0f          /* Measurement noise (higher = more smoothing, less mag trust) */

/* State vector [θ, ω] */
static float kalman_theta = 0.0f;  /* Heading estimate (degrees) */
static float kalman_omega = 0.0f;  /* Yaw rate estimate (°/s) */

/* Covariance matrix P (2x2, stored as 4 elements) */
static float P00 = 10.0f, P01 = 0.0f;
static float P10 = 0.0f,  P11 = 10.0f;

/* Current motor command (set by yaw controller) */
static volatile float current_motor_cmd = 0.0f;

void sensors_set_motor_cmd(float pwm_percent)
{
    current_motor_cmd = pwm_percent;
}

/**
 * Normalize angle to 0-360 range
 */
static float normalize_angle(float angle)
{
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * Compute angle difference handling wrap-around
 */
static float angle_diff(float a, float b)
{
    float diff = a - b;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

/**
 * 2D Kalman filter update
 * Input: raw magnetometer heading (degrees)
 * Updates internal state and returns filtered yaw rate
 */
static float kalman_update_2d(float measured_heading)
{
    /* ===== PREDICT STEP ===== */
    /* State prediction using motor model */
    float theta_pred = kalman_theta + kalman_omega * DT;
    float omega_pred = OMEGA_A * kalman_omega + OMEGA_B * current_motor_cmd;

    theta_pred = normalize_angle(theta_pred);

    /* Covariance prediction: P = F × P × F^T + Q
     * F = | 1   dt |
     *     | 0   A  |
     */
    float F00 = 1.0f, F01 = DT;
    float F10 = 0.0f, F11 = OMEGA_A;

    /* P_pred = F × P × F^T + Q */
    float P00_pred = F00*P00*F00 + F00*P01*F10 + F01*P10*F00 + F01*P11*F10 + Q_THETA;
    float P01_pred = F00*P00*F01 + F00*P01*F11 + F01*P10*F01 + F01*P11*F11;
    float P10_pred = F10*P00*F00 + F10*P01*F10 + F11*P10*F00 + F11*P11*F10;
    float P11_pred = F10*P00*F01 + F10*P01*F11 + F11*P10*F01 + F11*P11*F11 + Q_OMEGA;

    /* ===== UPDATE STEP ===== */
    /* Measurement: z = θ, H = [1, 0] */
    float innovation = angle_diff(measured_heading, theta_pred);

    /* S = H × P × H^T + R = P00 + R */
    float S = P00_pred + R_THETA;

    /* Kalman gain: K = P × H^T × S^(-1) = [P00/S, P10/S]^T */
    float K0 = P00_pred / S;
    float K1 = P10_pred / S;

    /* State update: x = x_pred + K × innovation */
    kalman_theta = normalize_angle(theta_pred + K0 * innovation);
    kalman_omega = omega_pred + K1 * innovation;

    /* Covariance update: P = (I - K×H) × P_pred */
    P00 = (1.0f - K0) * P00_pred;
    P01 = (1.0f - K0) * P01_pred;
    P10 = P10_pred - K1 * P00_pred;
    P11 = P11_pred - K1 * P01_pred;

    return kalman_omega;
}

/**
 * Get filtered heading from Kalman state
 */
float kalman_get_heading(void)
{
    return kalman_theta;
}

static bool filter_initialized = false;

/* Calibration state - will be populated by calibration routine or loaded from flash */
static struct mag_calibration mag_cal = {
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f,
    .scale_x = 1.0f,
    .scale_y = 1.0f,
    .scale_z = 1.0f,
    .version = MAG_CAL_VERSION,
    .valid = false
};

/* ============================================================================
 * Persistent Storage for Magnetometer Calibration
 * ============================================================================ */

static int mag_cal_settings_set(const char *name, size_t len,
                                 settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "data", &next) && !next) {
        /* Read calibration data */
        struct mag_calibration loaded_cal;
        rc = read_cb(cb_arg, &loaded_cal, len);
        if (rc < 0) {
            return rc;
        }

        /* Check version compatibility */
        if (loaded_cal.version != MAG_CAL_VERSION) {
            printk("MAG CAL: Old version %d in flash, expected %d - recalibration needed\n",
                   loaded_cal.version, MAG_CAL_VERSION);
            /* Keep default calibration (identity) */
            return 0;
        }

        /* Copy loaded calibration */
        mag_cal = loaded_cal;

        printk("MAG CAL: Loaded v%d from flash\n", mag_cal.version);
        printk("  Hard-Iron: (%.3f, %.3f, %.3f)\n",
               (double)mag_cal.offset_x,
               (double)mag_cal.offset_y,
               (double)mag_cal.offset_z);
        printk("  Soft-Iron: (%.3f, %.3f, %.3f)\n",
               (double)mag_cal.scale_x,
               (double)mag_cal.scale_y,
               (double)mag_cal.scale_z);
        return 0;
    }
    return -ENOENT;
}

static int mag_cal_settings_export(int (*cb)(const char *name,
                                              const void *value, size_t val_len))
{
    return cb("mag_cal/data", &mag_cal, sizeof(mag_cal));
}

SETTINGS_STATIC_HANDLER_DEFINE(mag_cal, "mag_cal", NULL,
                                mag_cal_settings_set, NULL,
                                mag_cal_settings_export);

static volatile bool calibrating = false;
static float cal_min_x, cal_max_x;
static float cal_min_y, cal_max_y;
static float cal_min_z, cal_max_z;
static int cal_samples;

/* Sensor thread */
#define SENSOR_STACK_SIZE 2560
#define SENSOR_PRIORITY 5
#define SENSOR_PERIOD_MS 100  /* 10 Hz - Kalman predicts between measurements */
#define CALIBRATION_SAMPLES 300   /* 30 seconds at 10 Hz */

K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_STACK_SIZE);
static struct k_thread sensor_thread_data;

/* Message queue for orientation data */
K_MSGQ_DEFINE(orientation_msgq, sizeof(struct sensor_msg), 10, 4);

/* Current orientation (updated by sensor thread, read by other modules) */
static volatile struct orientation_data current_orientation;

/* Current raw sensor values (for telemetry) */
static volatile int16_t current_raw_ax, current_raw_ay, current_raw_az;
static volatile int16_t current_raw_mx, current_raw_my, current_raw_mz;

/* ============================================================================
 * Reference Gravity Vector for Stable Heading
 *
 * The hexapod's gait causes roll/pitch oscillations at ~1.27 Hz.
 * Using instantaneous accelerometer for tilt compensation causes the
 * computed heading to oscillate ±7°.
 *
 * Solution: Use a slowly-filtered gravity direction to define the stable
 * walking plane. The filter has tau ≈ 2s (alpha=0.05 at 10Hz).
 * ============================================================================ */
#define REF_GRAV_ALPHA 0.05f  /* EMA alpha: tau ≈ (1/alpha - 1) * dt ≈ 1.9s */

static float ref_grav_x = 0.0f;
static float ref_grav_y = 0.0f;
static float ref_grav_z = 1.0f;  /* Default: gravity along Z (device flat) */
static bool ref_grav_initialized = false;

float sensors_get_heading(void)
{
    return current_orientation.heading;
}

float sensors_get_yaw_rate(void)
{
    /* Return yaw rate in MOTOR convention:
     *   positive = CW (turn right) = heading decreasing
     *   negative = CCW (turn left) = heading increasing
     *
     * Kalman internally uses PHYSICS convention (positive = CCW).
     * We negate to convert to motor convention for the yaw controller.
     */
    return -filtered_yaw_rate;
}

void sensors_get_orientation(struct orientation_data *out)
{
    if (out) {
        out->roll = current_orientation.roll;
        out->pitch = current_orientation.pitch;
        out->heading = current_orientation.heading;
        out->timestamp_ms = current_orientation.timestamp_ms;
    }
}

int sensors_init(void)
{
    struct sensor_value odr;
    int ret;

    /* Get accelerometer device (LSM303AGR accelerometer on micro:bit v2) */
    accel_dev = DEVICE_DT_GET_ONE(st_lis2dh);
    if (!device_is_ready(accel_dev)) {
        printk("ERROR: Accelerometer device not ready\n");
        return -ENODEV;
    }
    printk("Accelerometer: %s ready\n", accel_dev->name);

    /* Get magnetometer device (LSM303AGR magnetometer on micro:bit v2) */
    magn_dev = DEVICE_DT_GET_ONE(st_lis2mdl);
    if (!device_is_ready(magn_dev)) {
        printk("ERROR: Magnetometer device not ready\n");
        return -ENODEV;
    }
    printk("Magnetometer: %s ready\n", magn_dev->name);

    /* Configure magnetometer ODR to 100 Hz for fast updates */
    odr.val1 = 100;
    odr.val2 = 0;
    ret = sensor_attr_set(magn_dev, SENSOR_CHAN_MAGN_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (ret < 0) {
        printk("WARNING: Could not set magnetometer ODR: %d\n", ret);
    } else {
        printk("Magnetometer ODR set to 100 Hz\n");
    }

    /* Initialize Mahony AHRS filter */
    mahony_init(&ahrs_filter, 1000.0f / SENSOR_PERIOD_MS, MAHONY_KP, MAHONY_KI);
    printk("Mahony AHRS filter initialized (Kp=%.2f, Ki=%.2f)\n",
           (double)MAHONY_KP, (double)MAHONY_KI);

    return 0;
}

void sensors_start_calibration(void)
{
    if (calibrating) {
        return;
    }

    printk("CAL,START\n");

    /* Initialize min/max with extreme values */
    cal_min_x = cal_min_y = cal_min_z = 1000000.0f;
    cal_max_x = cal_max_y = cal_max_z = -1000000.0f;
    cal_samples = 0;

    calibrating = true;
}

bool sensors_is_calibrating(void)
{
    return calibrating;
}

const struct mag_calibration *sensors_get_calibration(void)
{
    return &mag_cal;
}

static void update_calibration(float mx, float my, float mz)
{
    /* Track min/max for each axis */
    if (mx < cal_min_x) cal_min_x = mx;
    if (mx > cal_max_x) cal_max_x = mx;
    if (my < cal_min_y) cal_min_y = my;
    if (my > cal_max_y) cal_max_y = my;
    if (mz < cal_min_z) cal_min_z = mz;
    if (mz > cal_max_z) cal_max_z = mz;

    cal_samples++;

    if (cal_samples >= CALIBRATION_SAMPLES) {
        /*
         * Hard-Iron Compensation:
         * The center of the ellipsoid is the offset caused by permanent magnets.
         * offset = (max + min) / 2
         */
        mag_cal.offset_x = (cal_max_x + cal_min_x) / 2.0f;
        mag_cal.offset_y = (cal_max_y + cal_min_y) / 2.0f;
        mag_cal.offset_z = (cal_max_z + cal_min_z) / 2.0f;

        /*
         * Soft-Iron Compensation:
         * The ellipsoid has different radii on each axis. We scale each axis
         * so that all radii become equal, transforming ellipsoid → sphere.
         *
         * radius = (max - min) / 2 = range / 2
         * avg_radius = (radius_x + radius_y + radius_z) / 3
         * scale = avg_radius / radius (normalize each axis to avg)
         */
        float range_x = cal_max_x - cal_min_x;
        float range_y = cal_max_y - cal_min_y;
        float range_z = cal_max_z - cal_min_z;

        /* Protect against division by zero (bad calibration data) */
        if (range_x < 1.0f) range_x = 1.0f;
        if (range_y < 1.0f) range_y = 1.0f;
        if (range_z < 1.0f) range_z = 1.0f;

        float avg_range = (range_x + range_y + range_z) / 3.0f;

        mag_cal.scale_x = avg_range / range_x;
        mag_cal.scale_y = avg_range / range_y;
        mag_cal.scale_z = avg_range / range_z;

        mag_cal.version = MAG_CAL_VERSION;
        mag_cal.valid = true;

        calibrating = false;

        printk("CAL,DONE\n");
        printk("  Hard-Iron (offset): %.3f, %.3f, %.3f\n",
               (double)mag_cal.offset_x,
               (double)mag_cal.offset_y,
               (double)mag_cal.offset_z);
        printk("  Soft-Iron (scale):  %.3f, %.3f, %.3f\n",
               (double)mag_cal.scale_x,
               (double)mag_cal.scale_y,
               (double)mag_cal.scale_z);
        printk("  Ranges: X=%.1f, Y=%.1f, Z=%.1f (avg=%.1f)\n",
               (double)range_x, (double)range_y, (double)range_z, (double)avg_range);

        /* Save calibration to flash for persistence across reboots */
        int err = settings_save_one("mag_cal/data", &mag_cal, sizeof(mag_cal));
        if (err) {
            printk("CAL: Failed to save to flash (err %d)\n", err);
        } else {
            printk("CAL: Saved to flash\n");
        }
    }
}

static void sensor_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct sensor_value accel[3];
    struct sensor_value magn[3];
    struct sensor_msg msg;
    int ret;

    while (1) {
        /* Read accelerometer */
        ret = sensor_sample_fetch(accel_dev);
        if (ret < 0 && ret != -EBADMSG) {
            printk("Accel fetch error: %d\n", ret);
            k_msleep(SENSOR_PERIOD_MS);
            continue;
        }

        ret = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        if (ret < 0) {
            printk("Accel channel get error: %d\n", ret);
            k_msleep(SENSOR_PERIOD_MS);
            continue;
        }

        /* Read magnetometer */
        ret = sensor_sample_fetch(magn_dev);
        if (ret < 0 && ret != -EBADMSG) {
            printk("Magn fetch error: %d\n", ret);
            k_msleep(SENSOR_PERIOD_MS);
            continue;
        }

        ret = sensor_channel_get(magn_dev, SENSOR_CHAN_MAGN_XYZ, magn);
        if (ret < 0) {
            printk("Magn channel get error: %d\n", ret);
            k_msleep(SENSOR_PERIOD_MS);
            continue;
        }

        /* Convert to float (raw sensor values in m/s² and Gauss) */
        float ax_raw = (float)sensor_value_to_double(&accel[0]);
        float ay_raw = (float)sensor_value_to_double(&accel[1]);
        float az_raw = (float)sensor_value_to_double(&accel[2]);

        float mx_raw = (float)sensor_value_to_double(&magn[0]);
        float my_raw = (float)sensor_value_to_double(&magn[1]);
        float mz_raw = (float)sensor_value_to_double(&magn[2]);

        /* Transform axes for micro:bit v2 LSM303AGR
         *
         * USER COORDINATE SYSTEM (measured via calibration):
         *   - Z+ = USB connector direction (up when USB points to ceiling)
         *   - X+ = away from user when display faces away
         *   - Y+ = left (right-hand system, toward Button A)
         *
         * ACCELEROMETER (measured sensor behavior):
         *   - Z+ up (USB up): Sensor-Y = -1g
         *   - Y+ up (Btn A up): Sensor-X = +1g
         *   - X+ up (Display up): Sensor-Z = -1g
         *
         * Transformation matrix (Accelerometer):
         *   User-X = -Sensor-Z
         *   User-Y = -Sensor-X
         *   User-Z = -Sensor-Y
         */
        float ax = -az_raw;   /* User-X = -Sensor-Z */
        float ay = -ax_raw;   /* User-Y = -Sensor-X (inverted for correct roll sign) */
        float az = -ay_raw;   /* User-Z = -Sensor-Y */

        /* MAGNETOMETER (different orientation than accelerometer!)
         *
         * Measured: When rotating around Z (USB up), Sensor-X and Sensor-Z change,
         * while Sensor-Y stays constant. This means:
         *   - Sensor-Y = vertical axis (User-Z)
         *   - Sensor-X, Sensor-Z = horizontal plane (User-X, User-Y)
         *
         * Transformation matrix (Magnetometer):
         *   User-X = -Sensor-Z (horizontal, changes with Z-rotation)
         *   User-Y = -Sensor-X (horizontal, changes with Z-rotation)
         *   User-Z = -Sensor-Y (vertical, constant during Z-rotation)
         */
        float mx = -mz_raw;   /* User-X = -Sensor-Z */
        float my = -mx_raw;   /* User-Y = -Sensor-X */
        float mz = -my_raw;   /* User-Z = -Sensor-Y */

        /* Store TRANSFORMED values as integers (milli-g and milli-Gauss)
         * These are in USER coordinates, not sensor coordinates */
        msg.raw_ax = (int16_t)(ax * 1000.0f / 9.81f);  /* m/s² to milli-g */
        msg.raw_ay = (int16_t)(ay * 1000.0f / 9.81f);
        msg.raw_az = (int16_t)(az * 1000.0f / 9.81f);
        msg.raw_mx = (int16_t)(mx * 1000.0f);  /* Gauss to milli-Gauss */
        msg.raw_my = (int16_t)(my * 1000.0f);
        msg.raw_mz = (int16_t)(mz * 1000.0f);

        /* Store for telemetry access */
        current_raw_ax = msg.raw_ax;
        current_raw_ay = msg.raw_ay;
        current_raw_az = msg.raw_az;
        current_raw_mx = msg.raw_mx;
        current_raw_my = msg.raw_my;
        current_raw_mz = msg.raw_mz;

        /* Update calibration if in progress */
        if (calibrating) {
            update_calibration(mx, my, mz);
        }

        /* Apply magnetometer calibration:
         * 1. Hard-Iron: Subtract offset (center the ellipsoid)
         * 2. Soft-Iron: Scale to transform ellipsoid → sphere
         */
        float mx_cal = mx, my_cal = my, mz_cal = mz;
        if (mag_cal.valid) {
            /* Hard-Iron: center the measurements */
            mx_cal = mx - mag_cal.offset_x;
            my_cal = my - mag_cal.offset_y;
            mz_cal = mz - mag_cal.offset_z;

            /* Soft-Iron: normalize axis ranges */
            mx_cal *= mag_cal.scale_x;
            my_cal *= mag_cal.scale_y;
            mz_cal *= mag_cal.scale_z;
        }

        /* ================================================================
         * Reference Gravity Update
         *
         * Normalize accelerometer to get gravity direction, then apply
         * slow EMA filter to track the stable walking plane.
         * This filters out gait-induced tilt oscillations (~1.27 Hz).
         * ================================================================ */
        float anorm = sqrtf(ax*ax + ay*ay + az*az);
        if (anorm > 0.5f) {  /* Valid reading (>0.5g, filters out free-fall) */
            float gx = ax / anorm;
            float gy = ay / anorm;
            float gz = az / anorm;

            if (!ref_grav_initialized) {
                /* Initialize with first valid reading */
                ref_grav_x = gx;
                ref_grav_y = gy;
                ref_grav_z = gz;
                ref_grav_initialized = true;
            } else {
                /* EMA filter update */
                ref_grav_x = REF_GRAV_ALPHA * gx + (1.0f - REF_GRAV_ALPHA) * ref_grav_x;
                ref_grav_y = REF_GRAV_ALPHA * gy + (1.0f - REF_GRAV_ALPHA) * ref_grav_y;
                ref_grav_z = REF_GRAV_ALPHA * gz + (1.0f - REF_GRAV_ALPHA) * ref_grav_z;

                /* Renormalize to unit vector (drift correction) */
                float rn = sqrtf(ref_grav_x*ref_grav_x + ref_grav_y*ref_grav_y + ref_grav_z*ref_grav_z);
                if (rn > 0.001f) {
                    ref_grav_x /= rn;
                    ref_grav_y /= rn;
                    ref_grav_z /= rn;
                }
            }
        }

        /* Calculate orientation:
         * - Roll/Pitch: from INSTANTANEOUS accelerometer (for telemetry display)
         * - Heading: from REFERENCE GRAVITY (stable walking plane)
         */
        float roll_raw, pitch_raw, heading_raw;

        /* Roll and Pitch from instantaneous accel (shows actual tilt) */
        if (anorm > 0.001f) {
            float ax_n = ax / anorm;
            float ay_n = ay / anorm;
            float az_n = az / anorm;
            roll_raw = atan2f(ay_n, az_n) * (180.0f / M_PI);
            pitch_raw = atan2f(-ax_n, sqrtf(ay_n*ay_n + az_n*az_n)) * (180.0f / M_PI);
        } else {
            roll_raw = 0.0f;
            pitch_raw = 0.0f;
        }

        /* Heading from reference gravity (stable plane, reduces gait wobble) */
        heading_raw = mahony_heading_with_ref_gravity(
            ref_grav_x, ref_grav_y, ref_grav_z,
            mx_cal, my_cal, mz_cal);

        /* Apply filters */
        if (!filter_initialized) {
            /* Initialize filter with first reading */
            filtered_roll = roll_raw;
            filtered_pitch = pitch_raw;
            /* Initialize 2D Kalman with first heading */
            kalman_theta = heading_raw;
            kalman_omega = 0.0f;
            filtered_heading = heading_raw;
            filtered_yaw_rate = 0.0f;
            filter_initialized = true;
        } else {
            /* Low-pass filter for roll and pitch */
            filtered_roll = LPF_ALPHA_ROLL * roll_raw +
                           (1.0f - LPF_ALPHA_ROLL) * filtered_roll;
            filtered_pitch = LPF_ALPHA_PITCH * pitch_raw +
                            (1.0f - LPF_ALPHA_PITCH) * filtered_pitch;

            /* 2D Kalman filter: feed raw heading, get filtered heading + yaw rate
             * No S-G derivative needed - Kalman estimates ω from θ measurements */
            filtered_yaw_rate = kalman_update_2d(heading_raw);
            filtered_heading = kalman_theta;
        }

        /* Prepare message with filtered values */
        msg.orientation.roll = filtered_roll;
        msg.orientation.pitch = filtered_pitch;
        msg.orientation.heading = filtered_heading;
        msg.orientation.timestamp_ms = k_uptime_get_32();
        msg.yaw_rate = filtered_yaw_rate;

        /* Update current orientation for direct access by other modules */
        current_orientation.roll = filtered_roll;
        current_orientation.pitch = filtered_pitch;
        current_orientation.heading = filtered_heading;
        current_orientation.timestamp_ms = msg.orientation.timestamp_ms;

        /* Send to output thread (non-blocking, drop if queue full) */
        k_msgq_put(&orientation_msgq, &msg, K_NO_WAIT);

        k_msleep(SENSOR_PERIOD_MS);
    }
}

void sensors_start_thread(void)
{
    k_thread_create(&sensor_thread_data, sensor_stack,
                    K_THREAD_STACK_SIZEOF(sensor_stack),
                    sensor_thread_fn, NULL, NULL, NULL,
                    SENSOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thread_data, "sensor");
}

void sensors_get_raw_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    if (ax) *ax = current_raw_ax;
    if (ay) *ay = current_raw_ay;
    if (az) *az = current_raw_az;
}

void sensors_get_raw_mag(int16_t *mx, int16_t *my, int16_t *mz)
{
    if (mx) *mx = current_raw_mx;
    if (my) *my = current_raw_my;
    if (mz) *mz = current_raw_mz;
}
