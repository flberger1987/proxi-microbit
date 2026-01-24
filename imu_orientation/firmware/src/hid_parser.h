/*
 * SPDX-License-Identifier: Apache-2.0
 * HID Parser - Xbox Controller Input Parsing
 *
 * Parses HID reports from Xbox Wireless Controller into
 * structured input data.
 */

#ifndef HID_PARSER_H
#define HID_PARSER_H

#include "robot_state.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the HID parser
 *
 * @return 0 on success, negative errno on failure
 */
int hid_parser_init(void);

/**
 * Parse raw HID report from Xbox Controller
 *
 * @param data Raw HID report data
 * @param len Length of data
 * @param input Output structure for parsed input
 * @return 0 on success, negative errno on failure
 */
int hid_parse_xbox_report(const uint8_t *data, uint16_t len, struct xbox_input *input);

/**
 * Convert Xbox input to motor command
 *
 * @param input Parsed Xbox input
 * @param cmd Output motor command
 */
void hid_input_to_motor_cmd(const struct xbox_input *input, struct motor_cmd *cmd);

/**
 * Get deadzone-adjusted stick value
 * Returns 0 for values within deadzone
 *
 * @param value Raw stick value (-32768 to 32767)
 * @param deadzone Deadzone threshold (0-32767)
 * @return Adjusted value with deadzone applied
 */
int16_t hid_apply_deadzone(int16_t value, int16_t deadzone);

/**
 * Check if a button is pressed
 *
 * @param input Parsed Xbox input
 * @param button_mask Button mask (XBOX_BTN_*)
 * @return true if button is pressed
 */
static inline bool hid_button_pressed(const struct xbox_input *input, uint16_t button_mask)
{
    return (input->buttons & button_mask) != 0;
}

/* Default deadzone (about 10% of full range) */
#define HID_DEFAULT_DEADZONE 3277

/* Trigger threshold for "pressed" state */
#define HID_TRIGGER_THRESHOLD 100

#endif /* HID_PARSER_H */
