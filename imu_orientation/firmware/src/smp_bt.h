/*
 * SPDX-License-Identifier: Apache-2.0
 * SMP BLE Transport for MCUmgr OTA DFU
 */

#ifndef SMP_BT_H
#define SMP_BT_H

#include <stdbool.h>

/**
 * Initialize BLE and start advertising for MCUmgr
 *
 * This sets up BLE with:
 * - Device name "IMU-Sensor"
 * - SMP service for firmware updates
 * - NUS service for serial data
 *
 * @return 0 on success, negative errno on failure
 */
int smp_bt_init(void);

/**
 * Check if BLE is connected
 *
 * @return true if a BLE connection is active
 */
bool smp_bt_is_connected(void);

/**
 * Pause BLE advertising (call before Xbox controller pairing)
 * Stops advertising to avoid dual-role conflicts
 */
void smp_bt_pause_advertising(void);

/**
 * Resume BLE advertising (call when Xbox controller disconnects)
 */
void smp_bt_resume_advertising(void);

#endif /* SMP_BT_H */
