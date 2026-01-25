/*
 * SPDX-License-Identifier: Apache-2.0
 * System Identification for Yaw Dynamics
 */

#ifndef SYSID_H
#define SYSID_H

#include <stdbool.h>
#include "yaw_controller.h"

/**
 * Check if system identification is running
 */
bool sysid_is_running(void);

/**
 * Abort the running test
 */
void sysid_abort_test(void);

/**
 * Run full system identification test
 * Tests multiple PWM levels and logs step responses
 *
 * Output format (via printk):
 *   SYSID,<time_ms>,<pwm_percent>,<yaw_rate_deg_s>
 *
 * Note: Disables yaw controller during test
 */
void sysid_run_full_test(void);

#endif /* SYSID_H */
