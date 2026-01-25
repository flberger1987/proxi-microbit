/*
 * SPDX-License-Identifier: Apache-2.0
 * Infrared Obstacle Sensors Driver
 *
 * Reads analog IR sensors on the Kosmos Proxi robot.
 * Provides obstacle detection with configurable thresholds.
 */

#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * IR sensor calibration data
 */
struct ir_calibration {
    uint16_t left_min;      /* Minimum value (no obstacle) */
    uint16_t left_max;      /* Maximum value (close obstacle) */
    uint16_t right_min;
    uint16_t right_max;
    uint16_t threshold_pct; /* Detection threshold in percent (0-100) */
};

/**
 * Initialize IR sensors
 * @return 0 on success, negative error code on failure
 */
int ir_sensors_init(void);

/**
 * Start the IR sensor thread
 */
void ir_sensors_start_thread(void);

/**
 * Get current raw ADC values
 * @param left_value Pointer to store left sensor value
 * @param right_value Pointer to store right sensor value
 */
void ir_sensors_get_raw(uint16_t *left_value, uint16_t *right_value);

/**
 * Check if obstacle is detected
 * @param left_detected Pointer to store left detection state
 * @param right_detected Pointer to store right detection state
 */
void ir_sensors_get_obstacle(bool *left_detected, bool *right_detected);

/**
 * Start calibration mode
 * In calibration mode, min/max values are tracked.
 * Move obstacles close and away from sensors during calibration.
 */
void ir_sensors_start_calibration(void);

/**
 * Stop calibration and apply captured min/max values
 */
void ir_sensors_stop_calibration(void);

/**
 * Check if calibration is active
 */
bool ir_sensors_is_calibrating(void);

/**
 * Set detection threshold (0-100%)
 * Higher values require closer obstacles to trigger
 */
void ir_sensors_set_threshold(uint16_t threshold_pct);

/**
 * Get current calibration data
 */
void ir_sensors_get_calibration(struct ir_calibration *cal);

/**
 * Enable/disable debug output
 */
void ir_sensors_set_debug(bool enabled);

/**
 * Get Kalman-filtered distance values in mm
 * @param left_mm Pointer to store left distance (mm)
 * @param right_mm Pointer to store right distance (mm)
 */
void ir_sensors_get_distance(float *left_mm, float *right_mm);

/**
 * GPIO Test Mode - find IR LED enable pin
 */
void ir_sensors_gpio_test_init(void);
void ir_sensors_gpio_test_next(void);
void ir_sensors_gpio_test_stop(void);
bool ir_sensors_gpio_test_active(void);

#endif /* IR_SENSORS_H */
