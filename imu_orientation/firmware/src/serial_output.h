/*
 * SPDX-License-Identifier: Apache-2.0
 * Serial output thread for orientation data
 */

#ifndef SERIAL_OUTPUT_H
#define SERIAL_OUTPUT_H

#include <stdbool.h>

/* Start the serial output thread (20 Hz) */
void serial_output_start_thread(void);

/* Enable/disable IMU streaming (default: disabled) */
void serial_output_set_imu_enabled(bool enabled);
bool serial_output_is_imu_enabled(void);

#endif /* SERIAL_OUTPUT_H */
