/*
 * SPDX-License-Identifier: Apache-2.0
 * BLE Output Thread using Nordic UART Service (NUS)
 */

#ifndef BLE_OUTPUT_H
#define BLE_OUTPUT_H

#include <stdbool.h>

/**
 * Initialize BLE output module
 * Must be called after smp_bt_init()
 *
 * @return 0 on success, negative errno on failure
 */
int ble_output_init(void);

/**
 * Start the BLE output thread (10 Hz)
 * Streams orientation data over NUS when connected
 */
void ble_output_start_thread(void);

/**
 * Check if NUS notifications are enabled
 *
 * @return true if a client has enabled notifications
 */
bool ble_output_is_enabled(void);

#endif /* BLE_OUTPUT_H */
