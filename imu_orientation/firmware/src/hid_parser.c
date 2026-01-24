/*
 * SPDX-License-Identifier: Apache-2.0
 * HID Parser - Game Controller Input Parsing
 *
 * Supports both Xbox and PS5 DualSense controllers.
 *
 * PS5 DualSense BLE HID Report Format:
 * Byte 0:    Report ID (0x01)
 * Byte 1:    Left Stick X (0-255, center=128)
 * Byte 2:    Left Stick Y (0-255, center=128)
 * Byte 3:    Right Stick X (0-255, center=128)
 * Byte 4:    Right Stick Y (0-255, center=128)
 * Byte 5:    L2 Trigger (0-255)
 * Byte 6:    R2 Trigger (0-255)
 * Byte 7:    Sequence counter (low byte)
 * Byte 8:    D-pad (bits 0-3) + Square/Cross/Circle/Triangle (bits 4-7)
 * Byte 9:    L1/R1/L2btn/R2btn/Share/Options/L3/R3
 * Byte 10:   PS/Touch/Mute buttons
 *
 * Xbox Wireless Controller HID Report Format:
 * Byte 0:     Report ID (0x01)
 * Bytes 1-2:  Left Stick X (16-bit, center=32768)
 * Bytes 3-4:  Left Stick Y (16-bit, center=32768)
 * Bytes 5-6:  Right Stick X (16-bit, center=32768)
 * Bytes 7-8:  Right Stick Y (16-bit, center=32768)
 * Bytes 9-11: Triggers packed (10-bit each)
 * Byte 11+:   D-Pad + Buttons
 */

#include "hid_parser.h"
#include "robot_state.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Expected minimum report length */
#define MIN_REPORT_LEN_DUALSENSE 10
#define MIN_REPORT_LEN_XBOX 14

/* Debug: Print raw HID reports */
#define DEBUG_HID_REPORTS 1

/* Stick center values */
#define STICK_CENTER_8BIT  128U
#define STICK_CENTER_16BIT 32768U

/* Controller type detection */
enum controller_type {
    CONTROLLER_UNKNOWN,
    CONTROLLER_DUALSENSE,
    CONTROLLER_XBOX,
};

static enum controller_type detected_controller = CONTROLLER_UNKNOWN;

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

/**
 * Read 16-bit little-endian unsigned value
 */
static inline uint16_t read_le16u(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * Convert 8-bit unsigned stick value to signed 16-bit
 * Input: 0-255 (center=128)
 * Output: -32768 to 32767
 */
static inline int16_t stick_8bit_to_signed(uint8_t val)
{
    return (int16_t)((int32_t)(val - STICK_CENTER_8BIT) * 32767 / 127);
}

/**
 * Detect controller type from HID report
 * Returns detected type based on report characteristics
 */
static enum controller_type detect_controller_type(const uint8_t *data, uint16_t len)
{
    if (len < 8) {
        return CONTROLLER_UNKNOWN;
    }

    const uint8_t *p = data;

    /* Skip report ID if present */
    if (data[0] == 0x01 && len >= 10) {
        p = data + 1;
        len--;
    }

    /*
     * DualSense BLE HID format detection:
     * - 8-bit sticks (0-255, center ~128)
     * - Bytes 0-3: sticks
     * - Bytes 4-5: triggers
     * - Bytes 6+: buttons
     *
     * Xbox BLE HID format:
     * - 16-bit sticks (0-65535, center ~32768)
     * - Bytes 0-7: sticks (4 × 16-bit)
     * - Bytes 8-10: triggers (packed 10-bit)
     */

    /* Check for DualSense pattern:
     * - First 4 bytes should be near 128 (stick centers)
     * - Allow wide range for stick drift (64-192)
     */
    uint8_t b0 = p[0], b1 = p[1], b2 = p[2], b3 = p[3];

    bool bytes_near_128 =
        (b0 >= 64 && b0 <= 192) &&
        (b1 >= 64 && b1 <= 192) &&
        (b2 >= 64 && b2 <= 192) &&
        (b3 >= 64 && b3 <= 192);

    /* For Xbox 16-bit format: high bytes of sticks should be ~0x7F-0x80 */
    /* Low bytes can be anything. Pattern: [any, ~0x80, any, ~0x80, any, ~0x80, any, ~0x80] */
    bool xbox_high_bytes =
        (b1 >= 0x70 && b1 <= 0x90) &&
        (b3 >= 0x70 && b3 <= 0x90);

    /* Check trigger bytes - DualSense triggers at bytes 4-5 should be 0 when not pressed
     * Xbox triggers at bytes 8-10 are packed differently */
    uint8_t ds_l2 = p[4];
    uint8_t ds_r2 = p[5];

    /* If triggers in DualSense position are high (>50) with no apparent button state,
     * it might be Xbox format where these bytes are part of stick data */

    /* For Xbox: check if bytes 8+ are zeros (triggers not pressed) */
    bool xbox_triggers_zero = (len >= 11) && (p[8] == 0 && p[9] == 0 && p[10] == 0);

    /* Decision logic:
     * 1. If all first 4 bytes near 128 AND bytes 4-5 are low → DualSense
     * 2. If high bytes (1,3,5,7) are near 0x80 → likely Xbox 16-bit
     * 3. If report is short (≤12 bytes) → favor DualSense
     */

    /* Strong DualSense signal: sticks near 128 and triggers low */
    if (bytes_near_128 && ds_l2 < 30 && ds_r2 < 30) {
        printk("HID: Detected DualSense (sticks near 128, triggers low)\n");
        return CONTROLLER_DUALSENSE;
    }

    /* Strong Xbox signal: 16-bit pattern in high bytes */
    if (xbox_high_bytes && len >= 14) {
        uint8_t b5 = p[5], b7 = p[7];
        if ((b5 >= 0x70 && b5 <= 0x90) && (b7 >= 0x70 && b7 <= 0x90)) {
            printk("HID: Detected Xbox (16-bit stick pattern)\n");
            return CONTROLLER_XBOX;
        }
    }

    /* Ambiguous case: bytes near 128 could be either format
     * Prefer DualSense for typical BLE gaming controllers */
    if (bytes_near_128) {
        printk("HID: Assuming DualSense (8-bit stick values)\n");
        return CONTROLLER_DUALSENSE;
    }

    /* Fallback: shorter reports are typically DualSense */
    if (len <= 12) {
        printk("HID: Assuming DualSense (short report)\n");
        return CONTROLLER_DUALSENSE;
    }

    printk("HID: Assuming Xbox (default)\n");
    return CONTROLLER_XBOX;
}

/**
 * Parse PS5 DualSense HID report
 */
static int parse_dualsense_report(const uint8_t *data, uint16_t len, struct xbox_input *input)
{
    const uint8_t *p = data;

    /* Skip report ID if present */
    if (data[0] == 0x01 && len >= 10) {
        p = data + 1;
        len--;
    }

    if (len < MIN_REPORT_LEN_DUALSENSE) {
        return -EINVAL;
    }

    /* Parse 8-bit sticks and convert to signed 16-bit */
    input->left_stick_x = stick_8bit_to_signed(p[0]);
    input->left_stick_y = stick_8bit_to_signed(p[1]);
    input->right_stick_x = stick_8bit_to_signed(p[2]);
    input->right_stick_y = stick_8bit_to_signed(p[3]);

    /* Parse 8-bit triggers (0-255) - scale to 0-1023 for compatibility */
    input->left_trigger = (uint16_t)p[4] * 4;   /* Scale 0-255 to 0-1020 */
    input->right_trigger = (uint16_t)p[5] * 4;

    /* Parse buttons */
    if (len >= 9) {
        /* Byte 7: D-pad (bits 0-3) + face buttons (bits 4-7) */
        input->dpad = p[7] & 0x0F;

        /* Map DualSense buttons to our generic format:
         * Byte 7 bits 4-7: Square, Cross, Circle, Triangle
         * Byte 8 bits 0-7: L1, R1, L2, R2, Share, Options, L3, R3
         * Cross = A, Circle = B, Square = X, Triangle = Y
         */
        uint16_t buttons = 0;
        if (p[7] & 0x20) buttons |= XBOX_BTN_A;      /* Cross */
        if (p[7] & 0x40) buttons |= XBOX_BTN_B;      /* Circle */
        if (p[7] & 0x10) buttons |= XBOX_BTN_X;      /* Square */
        if (p[7] & 0x80) buttons |= XBOX_BTN_Y;      /* Triangle */

        if (len >= 10) {
            if (p[8] & 0x01) buttons |= XBOX_BTN_LB;    /* L1 */
            if (p[8] & 0x02) buttons |= XBOX_BTN_RB;    /* R1 */
            if (p[8] & 0x10) buttons |= XBOX_BTN_VIEW;  /* Share */
            if (p[8] & 0x20) buttons |= XBOX_BTN_MENU;  /* Options */
            if (p[8] & 0x40) buttons |= XBOX_BTN_LSTICK; /* L3 */
            if (p[8] & 0x80) buttons |= XBOX_BTN_RSTICK; /* R3 */
        }
        if (len >= 11) {
            if (p[9] & 0x01) buttons |= XBOX_BTN_XBOX;  /* PS button */
        }

        input->buttons = buttons;
    }

    return 0;
}

/**
 * Parse Xbox controller HID report
 *
 * Xbox Wireless Controller BLE HID Report (16 bytes, no report ID):
 * Byte  0-1:  Left Stick X  (16-bit LE, center=32768)
 * Byte  2-3:  Left Stick Y  (16-bit LE)
 * Byte  4-5:  Right Stick X (16-bit LE)
 * Byte  6-7:  Right Stick Y (16-bit LE)
 * Byte  8-9:  Left Trigger  (10-bit, little-endian)
 * Byte 10-11: Right Trigger (10-bit, little-endian)
 * Byte 12:    D-Pad (hat switch: 0=Up, 1=UpRight, 2=Right, ..., 8=neutral)
 * Byte 13:    Buttons Low  (A=0x01, B=0x02, X=0x08, Y=0x10, LB=0x40, RB=0x80)
 * Byte 14:    Buttons High (View=0x04, Menu=0x08, L3=0x20, R3=0x40)
 * Byte 15:    System buttons (Xbox=0x01)
 */
static int parse_xbox_report(const uint8_t *data, uint16_t len, struct xbox_input *input)
{
    const uint8_t *p = data;

    if (len < 14) {
        return -EINVAL;
    }

    /* Parse sticks - UNSIGNED 16-bit, convert to signed */
    uint16_t lx_raw = read_le16u(p + 0);
    uint16_t ly_raw = read_le16u(p + 2);
    uint16_t rx_raw = read_le16u(p + 4);
    uint16_t ry_raw = read_le16u(p + 6);

    input->left_stick_x = (int16_t)(lx_raw - STICK_CENTER_16BIT);
    input->left_stick_y = (int16_t)(ly_raw - STICK_CENTER_16BIT);
    input->right_stick_x = (int16_t)(rx_raw - STICK_CENTER_16BIT);
    input->right_stick_y = (int16_t)(ry_raw - STICK_CENTER_16BIT);

    /* Parse triggers - 10-bit values in bytes 8-11 */
    input->left_trigger = read_le16u(p + 8) & 0x03FF;
    input->right_trigger = read_le16u(p + 10) & 0x03FF;

    /* Parse D-Pad (byte 12) - hat switch */
    input->dpad = p[12];

    /* Parse buttons (bytes 13-15) and remap to our XBOX_BTN_* masks */
    uint8_t btn_lo = p[13];  /* A, B, X, Y, LB, RB */
    uint8_t btn_hi = p[14];  /* View, Menu, L3, R3 */
    uint8_t btn_sys = (len >= 16) ? p[15] : 0;  /* Xbox button */

    uint16_t buttons = 0;

    /* Byte 13: Face buttons and bumpers */
    if (btn_lo & 0x01) buttons |= XBOX_BTN_A;       /* A */
    if (btn_lo & 0x02) buttons |= XBOX_BTN_B;       /* B */
    if (btn_lo & 0x08) buttons |= XBOX_BTN_X;       /* X */
    if (btn_lo & 0x10) buttons |= XBOX_BTN_Y;       /* Y */
    if (btn_lo & 0x40) buttons |= XBOX_BTN_LB;      /* LB */
    if (btn_lo & 0x80) buttons |= XBOX_BTN_RB;      /* RB */

    /* Byte 14: Menu buttons, stick clicks, and Xbox button */
    if (btn_hi & 0x04) buttons |= XBOX_BTN_VIEW;   /* View (back) */
    if (btn_hi & 0x08) buttons |= XBOX_BTN_MENU;   /* Menu (start) */
    if (btn_hi & 0x10) buttons |= XBOX_BTN_XBOX;   /* Xbox button */
    if (btn_hi & 0x20) buttons |= XBOX_BTN_LSTICK; /* Left stick click */
    if (btn_hi & 0x40) buttons |= XBOX_BTN_RSTICK; /* Right stick click */

    /* Byte 15: Reserved / unused */
    (void)btn_sys;

    input->buttons = buttons;

    return 0;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int hid_parser_init(void)
{
    detected_controller = CONTROLLER_UNKNOWN;
    printk("HID Parser: Initialized (auto-detect mode)\n");
    return 0;
}

int hid_parse_xbox_report(const uint8_t *data, uint16_t len, struct xbox_input *input)
{
    int ret;

    if (!data || !input || len < 8) {
        printk("HID: Invalid report (len=%d)\n", len);
        return -EINVAL;
    }

    /* Clear output */
    memset(input, 0, sizeof(*input));

#if DEBUG_HID_REPORTS
    printk("HID[%d]: ", len);
    for (int i = 0; i < len && i < 16; i++) {
        printk("%02x ", data[i]);
    }
    printk("\n");
#endif

    /* Auto-detect controller type on first few reports */
    if (detected_controller == CONTROLLER_UNKNOWN) {
        detected_controller = detect_controller_type(data, len);
        const char *type_str = (detected_controller == CONTROLLER_DUALSENSE) ? "DualSense" :
                               (detected_controller == CONTROLLER_XBOX) ? "Xbox" : "Unknown";
        printk("HID: Detected controller type: %s\n", type_str);
    }

    /* Parse based on detected type */
    if (detected_controller == CONTROLLER_DUALSENSE) {
        ret = parse_dualsense_report(data, len, input);
    } else {
        ret = parse_xbox_report(data, len, input);
    }

    if (ret != 0) {
        return ret;
    }

#if DEBUG_HID_REPORTS
    printk("HID: LX=%d LY=%d RX=%d RY=%d LT=%d RT=%d D=%d B=0x%04x\n",
           input->left_stick_x, input->left_stick_y,
           input->right_stick_x, input->right_stick_y,
           input->left_trigger, input->right_trigger,
           input->dpad, input->buttons);
#endif

    return 0;
}

int16_t hid_apply_deadzone(int16_t value, int16_t deadzone)
{
    if (value > deadzone) {
        /* Rescale from deadzone to max */
        return (int16_t)(((int32_t)(value - deadzone) * 32767) / (32767 - deadzone));
    } else if (value < -deadzone) {
        /* Rescale from -deadzone to min */
        return (int16_t)(((int32_t)(value + deadzone) * 32767) / (32767 - deadzone));
    }
    return 0;
}

void hid_input_to_motor_cmd(const struct xbox_input *input, struct motor_cmd *cmd)
{
    if (!input || !cmd) {
        return;
    }

    /* Apply deadzone to sticks */
    int16_t left_y = hid_apply_deadzone(input->left_stick_y, HID_DEFAULT_DEADZONE);
    int16_t right_y = hid_apply_deadzone(input->right_stick_y, HID_DEFAULT_DEADZONE);

    /*
     * Control mapping (Proxi Robot):
     * - Left stick Y  → Forward/backward (linear velocity)
     * - Right stick Y → Also forward/backward (can use either)
     * - L2 trigger    → Turn LEFT (angular velocity negative)
     * - R2 trigger    → Turn RIGHT (angular velocity positive)
     *
     * Note: Y-axis is typically inverted on controllers (up = negative value)
     */

    /* Use whichever stick has more input for forward/backward */
    int32_t linear;
    if (left_y != 0 || right_y == 0) {
        linear = -left_y;  /* Invert Y axis: push up = forward */
    } else {
        linear = -right_y;
    }

    /* Scale from stick range (-32768 to 32767) to motor range (-100 to 100) */
    linear = (linear * 100) / 32767;

    /*
     * Triggers for rotation:
     * - L2 (left trigger)  → Turn LEFT  (negative angular)
     * - R2 (right trigger) → Turn RIGHT (positive angular)
     * Triggers are 0-1023 (or 0-255 for some controllers)
     */
    int32_t angular = 0;

    /* Normalize trigger values (handle both 10-bit and 8-bit ranges) */
    int32_t lt = input->left_trigger;
    int32_t rt = input->right_trigger;

    /* If values seem to be 8-bit (max ~255), scale up to 1023 range */
    if (lt <= 255 && rt <= 255 && (lt > 0 || rt > 0)) {
        lt = (lt * 1023) / 255;
        rt = (rt * 1023) / 255;
    }

    /* Apply trigger threshold */
    if (lt > HID_TRIGGER_THRESHOLD || rt > HID_TRIGGER_THRESHOLD) {
        /* L2 = turn left (negative), R2 = turn right (positive) */
        int32_t turn_left = (lt > HID_TRIGGER_THRESHOLD) ? lt : 0;
        int32_t turn_right = (rt > HID_TRIGGER_THRESHOLD) ? rt : 0;

        /* Scale triggers to -100 to +100 range */
        angular = ((turn_right - turn_left) * 100) / 1023;
    }

    /* Clamp to valid range */
    if (linear > 100) linear = 100;
    if (linear < -100) linear = -100;
    if (angular > 100) angular = 100;
    if (angular < -100) angular = -100;

    cmd->linear = (int16_t)linear;
    cmd->angular = (int16_t)angular;
    cmd->emergency_stop = false;

    /* Emergency stop: Press both bumpers (L1+R1) or both stick buttons (L3+R3) */
    if ((input->buttons & (XBOX_BTN_LB | XBOX_BTN_RB)) == (XBOX_BTN_LB | XBOX_BTN_RB)) {
        cmd->emergency_stop = true;
    }
    if ((input->buttons & (XBOX_BTN_LSTICK | XBOX_BTN_RSTICK)) == (XBOX_BTN_LSTICK | XBOX_BTN_RSTICK)) {
        cmd->emergency_stop = true;
    }
}
