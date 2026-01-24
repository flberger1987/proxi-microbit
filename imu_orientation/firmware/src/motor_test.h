/*
 * SPDX-License-Identifier: Apache-2.0
 * Motor Test Mode - Pin testing via serial commands
 */

#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

/**
 * Initialize motor test mode
 * Configures GPIO pins for testing
 */
int motor_test_init(void);

/**
 * Start motor test thread
 * Listens for serial commands
 */
void motor_test_start_thread(void);

#endif /* MOTOR_TEST_H */
