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

#include <math.h>

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

/* Calibration state */
static struct mag_calibration mag_cal = {
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f,
    .valid = false
};

static volatile bool calibrating = false;
static float cal_min_x, cal_max_x;
static float cal_min_y, cal_max_y;
static float cal_min_z, cal_max_z;
static int cal_samples;

/* Sensor thread */
#define SENSOR_STACK_SIZE 2048
#define SENSOR_PRIORITY 5
#define SENSOR_PERIOD_MS 20  /* 50 Hz */
#define CALIBRATION_SAMPLES 500  /* ~10 seconds at 50 Hz */

K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_STACK_SIZE);
static struct k_thread sensor_thread_data;

/* Message queue for orientation data */
K_MSGQ_DEFINE(orientation_msgq, sizeof(struct sensor_msg), 10, 4);

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
    if (mx < cal_min_x) cal_min_x = mx;
    if (mx > cal_max_x) cal_max_x = mx;
    if (my < cal_min_y) cal_min_y = my;
    if (my > cal_max_y) cal_max_y = my;
    if (mz < cal_min_z) cal_min_z = mz;
    if (mz > cal_max_z) cal_max_z = mz;

    cal_samples++;

    if (cal_samples >= CALIBRATION_SAMPLES) {
        /* Calculate hard-iron offsets: offset = (max + min) / 2 */
        mag_cal.offset_x = (cal_max_x + cal_min_x) / 2.0f;
        mag_cal.offset_y = (cal_max_y + cal_min_y) / 2.0f;
        mag_cal.offset_z = (cal_max_z + cal_min_z) / 2.0f;
        mag_cal.valid = true;

        calibrating = false;

        printk("CAL,DONE,%.3f,%.3f,%.3f\n",
               (double)mag_cal.offset_x,
               (double)mag_cal.offset_y,
               (double)mag_cal.offset_z);
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

        /* Store raw values as integers (milli-g and milli-Gauss) */
        msg.raw_ax = (int16_t)(ax_raw * 1000.0f / 9.81f);  /* m/s² to milli-g */
        msg.raw_ay = (int16_t)(ay_raw * 1000.0f / 9.81f);
        msg.raw_az = (int16_t)(az_raw * 1000.0f / 9.81f);
        msg.raw_mx = (int16_t)(mx_raw * 1000.0f);  /* Gauss to milli-Gauss */
        msg.raw_my = (int16_t)(my_raw * 1000.0f);
        msg.raw_mz = (int16_t)(mz_raw * 1000.0f);

        /* Transform axes for micro:bit v2 LSM303AGR
         *
         * USER COORDINATE SYSTEM (measured via calibration):
         *   - Z+ = USB connector direction (up when USB points to ceiling)
         *   - X+ = away from user when display faces away
         *   - Y+ = left (right-hand system, toward Button A)
         *
         * Measured sensor behavior:
         *   - Z+ up (USB up): Sensor-Y = -1g
         *   - Y+ up (Btn A up): Sensor-X = +1g
         *   - X+ up (Display up): Sensor-Z = -1g
         *
         * Transformation matrix:
         *   User-X = -Sensor-Z
         *   User-Y = +Sensor-X
         *   User-Z = -Sensor-Y
         */
        float ax = -az_raw;   /* User-X = -Sensor-Z */
        float ay = -ax_raw;   /* User-Y = -Sensor-X (inverted for correct roll sign) */
        float az = -ay_raw;   /* User-Z = -Sensor-Y */

        float mx = -mz_raw;
        float my = -mx_raw;
        float mz = -my_raw;

        /* Update calibration if in progress */
        if (calibrating) {
            update_calibration(mx, my, mz);
        }

        /* Apply hard-iron calibration to magnetometer */
        float mx_cal = mx, my_cal = my, mz_cal = mz;
        if (mag_cal.valid) {
            mx_cal = mx - mag_cal.offset_x;
            my_cal = my - mag_cal.offset_y;
            mz_cal = mz - mag_cal.offset_z;
        }

        /* Calculate orientation directly (tilt-compensated compass)
         * This works much better than Mahony filter without a gyroscope
         */
        float roll, pitch, heading;
        mahony_get_euler_direct(ax, ay, az, mx_cal, my_cal, mz_cal,
                                &roll, &pitch, &heading);


        /* Prepare message */
        msg.orientation.roll = roll;
        msg.orientation.pitch = pitch;
        msg.orientation.heading = heading;
        msg.orientation.timestamp_ms = k_uptime_get_32();

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
