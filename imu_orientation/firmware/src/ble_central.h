/*
 * SPDX-License-Identifier: Apache-2.0
 * BLE Central - Xbox Controller Connection
 *
 * Implements BLE Central role to scan for and connect to Xbox Wireless Controller.
 */

#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * BLE Central connection callback type
 */
typedef void (*ble_central_connected_cb_t)(void);
typedef void (*ble_central_disconnected_cb_t)(void);
typedef void (*ble_central_input_cb_t)(const uint8_t *data, uint16_t len);

/**
 * BLE Central callbacks
 */
struct ble_central_callbacks {
    ble_central_connected_cb_t connected;
    ble_central_disconnected_cb_t disconnected;
    ble_central_input_cb_t input_received;
};

/**
 * Initialize BLE Central mode
 * Note: This will modify BLE to support both peripheral and central roles
 *
 * @param callbacks Callback functions for events
 * @return 0 on success, negative errno on failure
 */
int ble_central_init(const struct ble_central_callbacks *callbacks);

/**
 * Start scanning for Xbox Controller
 * Scans for devices with name "Xbox Wireless Controller"
 *
 * @return 0 on success, negative errno on failure
 */
int ble_central_start_scan(void);

/**
 * Stop scanning
 *
 * @return 0 on success, negative errno on failure
 */
int ble_central_stop_scan(void);

/**
 * Disconnect from controller
 *
 * @return 0 on success, negative errno on failure
 */
int ble_central_disconnect(void);

/**
 * Check if currently scanning
 *
 * @return true if scanning is active
 */
bool ble_central_is_scanning(void);

/**
 * Check if connected to controller
 *
 * @return true if controller is connected
 */
bool ble_central_is_connected(void);

/**
 * Start the BLE central thread
 */
void ble_central_start_thread(void);

/**
 * Start auto-reconnect to bonded controller (if any)
 * Called at startup after BLE is initialized
 *
 * @return 0 if reconnect started, -ENOENT if no bonded controller
 */
int ble_central_start_reconnect(void);

/**
 * Check if a controller is bonded
 *
 * @return true if we have a bonded controller
 */
bool ble_central_has_bonded_controller(void);

/**
 * Clear bonded controller
 * Call this to forget the paired controller
 */
void ble_central_clear_bonds(void);

#endif /* BLE_CENTRAL_H */
