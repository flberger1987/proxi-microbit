/*
 * SPDX-License-Identifier: Apache-2.0
 * Serial output thread implementation
 */

#include "serial_output.h"
#include "sensors.h"
#include "orientation.h"
#include "yaw_controller.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Output thread configuration */
#define OUTPUT_STACK_SIZE 1024
#define OUTPUT_PRIORITY 6
#define OUTPUT_PERIOD_MS 50  /* 20 Hz */

K_THREAD_STACK_DEFINE(output_stack, OUTPUT_STACK_SIZE);
static struct k_thread output_thread_data;

/* IMU streaming state (enabled for yaw testing) */
static volatile bool imu_streaming_enabled = true;

static void output_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct sensor_msg msg;
    int ret;

    while (1) {
        /* Wait for orientation data from sensor thread */
        ret = k_msgq_get(&orientation_msgq, &msg, K_MSEC(OUTPUT_PERIOD_MS));

        if (ret == 0 && imu_streaming_enabled) {
            /* Get motor output from yaw controller */
            float motor_output;
            yaw_controller_get_debug(NULL, NULL, &motor_output);

            /* Output orientation data:
             * IMU,<timestamp_ms>,<roll>,<pitch>,<heading>,<yaw_rate>,<motor_pwm>
             * Angles in degrees, yaw_rate in deg/s, motor_pwm in %
             */
            printk("IMU,%u,%.1f,%.1f,%.1f,%.1f,%.0f\r\n",
                   msg.orientation.timestamp_ms,
                   (double)msg.orientation.roll,
                   (double)msg.orientation.pitch,
                   (double)msg.orientation.heading,
                   (double)msg.yaw_rate,
                   (double)motor_output);
        }
        /* If timeout or disabled, just continue (drain queue) */
    }
}

void serial_output_start_thread(void)
{
    k_thread_create(&output_thread_data, output_stack,
                    K_THREAD_STACK_SIZEOF(output_stack),
                    output_thread_fn, NULL, NULL, NULL,
                    OUTPUT_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&output_thread_data, "serial_out");
}

void serial_output_set_imu_enabled(bool enabled)
{
    imu_streaming_enabled = enabled;
    printk("Serial IMU streaming: %s\n", enabled ? "ON" : "OFF");
}

bool serial_output_is_imu_enabled(void)
{
    return imu_streaming_enabled;
}
