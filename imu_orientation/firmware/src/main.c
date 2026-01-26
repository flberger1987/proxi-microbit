/*
 * SPDX-License-Identifier: Apache-2.0
 * Kosmos Proxi Robot - Xbox Controller Remote Control
 *
 * micro:bit v2 firmware for controlling the Kosmos Proxi robot
 * via Xbox Wireless Controller over BLE.
 *
 * Features:
 * - BLE Central: Connect to Xbox Wireless Controller
 * - BLE Peripheral: NUS for debugging/IMU data (when enabled)
 * - Differential drive motor control
 * - PWM audio feedback
 * - 5x5 LED display animations
 *
 * Button A: Long press (1s) → Start pairing mode
 * Button B: (Reserved for future use)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/input/input.h>
#include <zephyr/display/mb_display.h>

#include "robot_state.h"
#include "sensors.h"
#include "orientation.h"
#include "serial_output.h"
#include "smp_bt.h"
#include "ble_output.h"
#include "ble_central.h"
#include "hid_parser.h"
#include "motor_driver.h"
#include "audio.h"
#include "motor_test.h"
#include "ir_sensors.h"
#include "autonomous_nav.h"
#include "yaw_controller.h"
#include "telemetry.h"

/* ============================================================================
 * Display Animations
 * ============================================================================ */

/* Pulsing heart animation (~1Hz) - Idle state */
static const struct mb_image heart_animation[] = {
    /* Small heart */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 0, 1, 0, 1, 0 },
        { 0, 1, 1, 1, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* Big heart */
    MB_IMAGE(
        { 0, 1, 0, 1, 0 },
        { 1, 1, 1, 1, 1 },
        { 1, 1, 1, 1, 1 },
        { 0, 1, 1, 1, 0 },
        { 0, 0, 1, 0, 0 }
    ),
};

/* Rotating dot animation - Pairing state */
static const struct mb_image pairing_animation[] = {
    MB_IMAGE({ 1, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 1, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 1 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 1 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 1 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 1 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 1 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 1, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 1, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 1, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
    MB_IMAGE({ 0, 0, 0, 0, 0 }, { 1, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }),
};

/* Eyes animation - Connected state (blinking) - Eyes at outer edges */
static const struct mb_image eyes_animation[] = {
    /* Eyes wide open - at outer edges */
    MB_IMAGE(
        { 1, 0, 0, 0, 1 },
        { 1, 0, 0, 0, 1 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* Eyes open (held longer) */
    MB_IMAGE(
        { 1, 0, 0, 0, 1 },
        { 1, 0, 0, 0, 1 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* Eyes open (held longer) */
    MB_IMAGE(
        { 1, 0, 0, 0, 1 },
        { 1, 0, 0, 0, 1 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* Eyes half closed */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 1, 0, 0, 0, 1 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* Eyes closed (blink) */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
};

/* Calibration animation - rotating compass needle */
static const struct mb_image calibration_animation[] = {
    /* North */
    MB_IMAGE(
        { 0, 0, 1, 0, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* NE */
    MB_IMAGE(
        { 0, 0, 0, 1, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 1, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* East */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 1, 1, 1, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* SE */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 0, 1, 0 }
    ),
    /* South */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 1, 0, 0 }
    ),
    /* SW */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 1, 0, 0, 0 }
    ),
    /* West */
    MB_IMAGE(
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 1, 1, 1 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
    /* NW */
    MB_IMAGE(
        { 0, 1, 0, 0, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 0, 1, 0 },
        { 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0 }
    ),
};

/* Error X mark */
static const struct mb_image img_error = MB_IMAGE(
    { 1, 0, 0, 0, 1 },
    { 0, 1, 0, 1, 0 },
    { 0, 0, 1, 0, 0 },
    { 0, 1, 0, 1, 0 },
    { 1, 0, 0, 0, 1 }
);

/* Success checkmark */
static const struct mb_image img_check = MB_IMAGE(
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1 },
    { 0, 0, 0, 1, 0 },
    { 1, 0, 1, 0, 0 },
    { 0, 1, 0, 0, 0 }
);

/* Empty frame for blinking */
static const struct mb_image img_blank = MB_IMAGE(
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 }
);

/* Face emoji: Smile (Button A) */
static const struct mb_image face_smile = MB_IMAGE(
    { 0, 1, 0, 1, 0 },
    { 0, 1, 0, 1, 0 },
    { 0, 0, 0, 0, 0 },
    { 1, 0, 0, 0, 1 },
    { 0, 1, 1, 1, 0 }
);

/* Face emoji: Wink (Button B) */
static const struct mb_image face_wink = MB_IMAGE(
    { 0, 1, 0, 1, 0 },
    { 0, 1, 0, 0, 0 },
    { 0, 0, 0, 0, 0 },
    { 1, 0, 0, 0, 1 },
    { 0, 1, 1, 1, 0 }
);

/* Face emoji: Surprised (Button X) */
static const struct mb_image face_surprised = MB_IMAGE(
    { 0, 1, 0, 1, 0 },
    { 0, 1, 0, 1, 0 },
    { 0, 0, 0, 0, 0 },
    { 0, 1, 1, 1, 0 },
    { 0, 1, 1, 1, 0 }
);

/* Face emoji: Frown (Button Y) */
static const struct mb_image face_frown = MB_IMAGE(
    { 0, 1, 0, 1, 0 },
    { 0, 1, 0, 1, 0 },
    { 0, 0, 0, 0, 0 },
    { 0, 1, 1, 1, 0 },
    { 1, 0, 0, 0, 1 }
);

/* Arrow animation for autonomous mode (forward arrow) */
static const struct mb_image arrow_animation[] = {
    /* Frame 1: Small arrow */
    MB_IMAGE(
        { 0, 0, 1, 0, 0 },
        { 0, 1, 1, 1, 0 },
        { 1, 0, 1, 0, 1 },
        { 0, 0, 1, 0, 0 },
        { 0, 0, 1, 0, 0 }
    ),
    /* Frame 2: Large arrow */
    MB_IMAGE(
        { 0, 0, 1, 0, 0 },
        { 0, 0, 1, 0, 0 },
        { 0, 1, 1, 1, 0 },
        { 1, 0, 1, 0, 1 },
        { 0, 0, 1, 0, 0 }
    ),
};

/* ============================================================================
 * Global State
 * ============================================================================ */

static struct mb_display *disp;
static enum robot_state current_display_state = -1;  /* Force first update */

/* Long press detection */
#define LONG_PRESS_MS 1000        /* Button A: 1 second for pairing */
#define LONG_PRESS_CAL_MS 3000    /* Button B: 3 seconds for calibration */
static volatile int64_t button_a_press_time;
static volatile bool button_a_pressed;
static volatile bool long_press_triggered;

/* Button B long press for magnetometer calibration */
static volatile int64_t button_b_press_time;
static volatile bool button_b_pressed;
static volatile bool button_b_long_press_triggered;

/* Temporary emoji display (flashing for button feedback)
 * Note: Accessed from BLE callback (write) and main loop (read).
 * Race condition is benign - worst case is a single frame delay. */
#define EMOJI_DISPLAY_MS    2000  /* Total display time: 2 seconds */
#define EMOJI_FLASH_MS      250   /* Flash interval: 250ms */
static int64_t emoji_start_time;
static const struct mb_image *current_emoji;
static bool emoji_visible;

/* ============================================================================
 * Display Update
 * ============================================================================ */

/**
 * Start displaying a flashing emoji overlay.
 * The emoji will flash (250ms on/off) for 2 seconds.
 * Called from BLE callback, display update happens in main loop.
 */
static void start_emoji_flash(const struct mb_image *emoji)
{
    emoji_start_time = k_uptime_get();
    current_emoji = emoji;
    emoji_visible = true;
}

/**
 * Update the flashing emoji display.
 * Returns true if emoji is still active, false if finished.
 */
static bool update_emoji_flash(void)
{
    if (current_emoji == NULL) {
        return false;
    }

    int64_t elapsed = k_uptime_get() - emoji_start_time;

    /* Check if emoji display time is over */
    if (elapsed >= EMOJI_DISPLAY_MS) {
        current_emoji = NULL;
        emoji_visible = false;
        current_display_state = -1;  /* Force display state refresh */
        return false;
    }

    /* Calculate flash phase (250ms intervals) */
    bool should_be_visible = ((elapsed / EMOJI_FLASH_MS) % 2) == 0;

    if (should_be_visible != emoji_visible) {
        emoji_visible = should_be_visible;
        if (emoji_visible) {
            mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, SYS_FOREVER_MS,
                             current_emoji, 1);
        } else {
            mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, SYS_FOREVER_MS,
                             &img_blank, 1);
        }
    }

    return true;
}

static void update_display_for_state(enum robot_state state)
{
    if (state == current_display_state) {
        return;
    }

    current_display_state = state;

    switch (state) {
    case ROBOT_STATE_IDLE:
        mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
                         500, heart_animation, ARRAY_SIZE(heart_animation));
        break;

    case ROBOT_STATE_PAIRING:
        mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
                         100, pairing_animation, ARRAY_SIZE(pairing_animation));
        break;

    case ROBOT_STATE_CONNECTED:
        mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
                         500, eyes_animation, ARRAY_SIZE(eyes_animation));
        break;

    case ROBOT_STATE_AUTONOMOUS:
        mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
                         300, arrow_animation, ARRAY_SIZE(arrow_animation));
        break;
    }
}

/* ============================================================================
 * BLE Central Callbacks
 * ============================================================================ */

static void on_controller_connected(void)
{
    printk("Controller connected!\n");
    audio_play(SOUND_CONNECTED);
    motor_enable(true);
    yaw_controller_enable(true);  /* Enable yaw rate control */
}

/* Previous button state for edge detection */
static uint16_t prev_controller_buttons = 0;
static uint8_t prev_dpad = XBOX_DPAD_NONE;
static bool first_hid_report = true;  /* Ignore D-Pad on first report */

static void on_controller_disconnected(void)
{
    printk("Controller disconnected!\n");
    audio_play(SOUND_DISCONNECTED);
    motor_enable(false);
    motor_emergency_stop();
    yaw_controller_enable(false);
    yaw_controller_reset();

    /* Reset D-Pad state for next connection */
    first_hid_report = true;
    prev_dpad = XBOX_DPAD_NONE;
    prev_controller_buttons = 0;
}

static void on_controller_input(const uint8_t *data, uint16_t len)
{
    struct xbox_input input;
    struct motor_cmd cmd;

    /* Parse HID report */
    if (hid_parse_xbox_report(data, len, &input) != 0) {
        return;
    }

    /* Detect button press edges (just pressed this frame) */
    uint16_t just_pressed = (input.buttons ^ prev_controller_buttons) & input.buttons;

    /* Detect D-Pad edges (just pressed this frame)
     * Ignore first HID report to prevent auto-start if controller sends dpad=0 on connect */
    bool dpad_changed = (input.dpad != prev_dpad);
    bool dpad_up_pressed = false;
    bool dpad_down_pressed = false;
    bool dpad_left_pressed = false;
    bool dpad_right_pressed = false;

    if (first_hid_report) {
        /* First report - just record state, don't trigger any actions */
        first_hid_report = false;
        printk("HID: First report dpad=%d (0x%02X) (ignoring for autonav)\n", input.dpad, input.dpad);
    } else if (dpad_changed) {
        /* Log ALL D-Pad changes for debugging */
        printk("DPAD: %d->%d (0x%02X->0x%02X) prev_none=%d\n",
               prev_dpad, input.dpad, prev_dpad, input.dpad,
               (prev_dpad == XBOX_DPAD_NONE));

        /* Only trigger on transition from NONE to a direction */
        if (prev_dpad == XBOX_DPAD_NONE) {
            dpad_up_pressed = (input.dpad == XBOX_DPAD_UP);
            dpad_down_pressed = (input.dpad == XBOX_DPAD_DOWN);
            dpad_left_pressed = (input.dpad == XBOX_DPAD_LEFT);
            dpad_right_pressed = (input.dpad == XBOX_DPAD_RIGHT);

            printk("DPAD: UP=%d DOWN=%d LEFT=%d RIGHT=%d\n",
                   dpad_up_pressed, dpad_down_pressed, dpad_left_pressed, dpad_right_pressed);
        }
    }

    /* ========== D-Pad Handling (BEFORE manual override check!) ========== */
    /*
     * D-Pad controls heading direction in autonomous mode:
     * - UP: Forward (heading hold) - enable if not enabled, no effect if already forward
     * - DOWN: 180° U-turn
     * - LEFT: -90° turn (counter-clockwise)
     * - RIGHT: +90° turn (clockwise)
     *
     * Inputs are ignored while already turning (AUTONAV_TURNING state).
     * D-Pad NEVER disables autonomous mode - only stick/trigger does that.
     */
    enum autonav_state nav_state = autonav_get_state();
    bool is_turning = (nav_state == AUTONAV_TURNING || nav_state == AUTONAV_SCANNING);

    /* D-Pad UP: Enable autonomous mode / confirm forward direction */
    if (dpad_up_pressed) {
        if (!autonav_is_enabled()) {
            /* Not enabled yet - start autonomous mode */
            autonav_enable();
            robot_set_state(ROBOT_STATE_AUTONOMOUS);
        }
        /* If already in HEADING_HOLD or TURNING, UP does nothing */
    }

    /* D-Pad DOWN: 180° U-turn (only if not already turning) */
    if (dpad_down_pressed) {
        if (autonav_is_enabled() && !is_turning) {
            autonav_turn_relative(180);
        }
    }

    /* D-Pad LEFT: +90° turn CCW (only if not already turning) */
    if (dpad_left_pressed) {
        printk("DPAD LEFT: enabled=%d is_turning=%d state=%d\n",
               autonav_is_enabled(), is_turning, nav_state);
        if (autonav_is_enabled() && !is_turning) {
            autonav_turn_relative(+90);  /* CCW = heading increases */
        }
    }

    /* D-Pad RIGHT: -90° turn CW (only if not already turning) */
    if (dpad_right_pressed) {
        printk("DPAD RIGHT: enabled=%d is_turning=%d state=%d\n",
               autonav_is_enabled(), is_turning, nav_state);
        if (autonav_is_enabled() && !is_turning) {
            autonav_turn_relative(-90);  /* CW = heading decreases */
        }
    }

    /* Update previous D-Pad state */
    prev_dpad = input.dpad;

    /* ========== Manual Override Detection (AFTER D-Pad!) ========== */
    /* Skip override check if D-Pad was just pressed (intentional input) */
    if (autonav_is_enabled() && !dpad_changed) {
        /* Check for manual input that should override autonomous mode:
         * - Any stick movement outside deadzone (10% from center)
         * - Any trigger > 10%
         * - Emergency stop combo
         */
        bool stick_active = (hid_apply_deadzone(input.left_stick_y, HID_DEFAULT_DEADZONE) != 0) ||
                           (hid_apply_deadzone(input.right_stick_y, HID_DEFAULT_DEADZONE) != 0);
        bool trigger_active = (input.left_trigger > HID_TRIGGER_THRESHOLD) ||
                             (input.right_trigger > HID_TRIGGER_THRESHOLD);
        bool emergency_stop = ((input.buttons & (XBOX_BTN_LB | XBOX_BTN_RB)) == (XBOX_BTN_LB | XBOX_BTN_RB)) ||
                             ((input.buttons & (XBOX_BTN_LSTICK | XBOX_BTN_RSTICK)) == (XBOX_BTN_LSTICK | XBOX_BTN_RSTICK));

        if (stick_active || trigger_active || emergency_stop) {
            autonav_manual_override();
        }
    }

    /* ========== Face Button Handling ========== */
    /* A button (Cross on PS5) → Shoot sound + Smile face! */
    if (just_pressed & XBOX_BTN_A) {
        audio_play(SOUND_SHOOT);
        start_emoji_flash(&face_smile);
    }

    /* B button (Circle on PS5) → Click sound + Wink face */
    if (just_pressed & XBOX_BTN_B) {
        audio_play(SOUND_BUTTON_PRESS);
        start_emoji_flash(&face_wink);
    }

    /* X button (Square on PS5) → Machine gun + Surprised face! */
    if (just_pressed & XBOX_BTN_X) {
        audio_play(SOUND_MACHINEGUN);
        start_emoji_flash(&face_surprised);
    }

    /* Y button (Triangle on PS5) → Obstacle warning + Frown face */
    if (just_pressed & XBOX_BTN_Y) {
        audio_play(SOUND_OBSTACLE);
        start_emoji_flash(&face_frown);
    }

    /* Update previous button state */
    prev_controller_buttons = input.buttons;

    /* ========== Motor Control ========== */
    /* Skip sending motor commands if in autonomous mode (autonav controls motors) */
    if (autonav_is_enabled()) {
        return;
    }

    /* Set target yaw rate from triggers (closed-loop control) */
    yaw_controller_set_triggers(input.left_trigger, input.right_trigger);

    /* Convert to motor command (linear only, angular handled by yaw controller) */
    hid_input_to_motor_cmd(&input, &cmd);

    /* When yaw controller is active, zero out angular from HID parser
     * (yaw controller computes angular in motor thread) */
    if (yaw_controller_is_enabled()) {
        cmd.angular = 0;
    }

    /* Send to motor thread */
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

static struct ble_central_callbacks central_callbacks = {
    .connected = on_controller_connected,
    .disconnected = on_controller_disconnected,
    .input_received = on_controller_input,
};

/* ============================================================================
 * Button Handling
 * ============================================================================ */

static void handle_long_press_a(void)
{
    enum robot_state state = robot_get_state();

    if (state == ROBOT_STATE_IDLE) {
        /* Start pairing */
        printk("Starting pairing mode...\n");
        audio_play(SOUND_PAIRING_START);
        ble_central_start_scan();
    } else if (state == ROBOT_STATE_CONNECTED) {
        /* Disconnect */
        printk("Disconnecting controller...\n");
        ble_central_disconnect();
    } else if (state == ROBOT_STATE_PAIRING) {
        /* Cancel pairing */
        printk("Cancelling pairing...\n");
        ble_central_stop_scan();
        robot_set_state(ROBOT_STATE_IDLE);
    }
}

static void button_cb(struct input_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);

    if (evt->sync == 0) {
        return;
    }

    /* Button A handling - long press detection */
    if (evt->code == INPUT_KEY_A) {
        if (evt->value == 1) {
            /* Button pressed */
            button_a_pressed = true;
            button_a_press_time = k_uptime_get();
            long_press_triggered = false;
        } else {
            /* Button released */
            button_a_pressed = false;

            if (!long_press_triggered) {
                /* Short press - play click sound */
                audio_play(SOUND_BUTTON_PRESS);
            }
        }
    }

    /* Button B handling - short press cycles GPIO test, long press toggles mode */
    if (evt->code == INPUT_KEY_B) {
        if (evt->value == 1) {
            /* Button pressed */
            button_b_pressed = true;
            button_b_press_time = k_uptime_get();
            button_b_long_press_triggered = false;
        } else {
            /* Button released */
            button_b_pressed = false;

            if (!button_b_long_press_triggered) {
                audio_play(SOUND_BUTTON_PRESS);

                /* Short press - emergency stop */
                if (robot_get_state() == ROBOT_STATE_CONNECTED) {
                    motor_emergency_stop();
                    printk("Emergency stop triggered\n");
                }
            }
        }
    }
}

INPUT_CALLBACK_DEFINE(NULL, button_cb, NULL);

/* ============================================================================
 * Calibration State
 * ============================================================================ */

static volatile bool was_calibrating = false;
static int64_t last_calibration_beep_time = 0;
static int calibration_animation_frame = 0;
#define CALIBRATION_BEEP_INTERVAL_MS 5000  /* Beep every 5 seconds */
#define CALIBRATION_ANIM_INTERVAL_MS 500   /* Animation frame every 500ms */

/* ============================================================================
 * Main Function
 * ============================================================================ */

int main(void)
{
    int ret;

    printk("\n");
    printk("=====================================\n");
    printk("  Kosmos Proxi - Xbox Controller RC\n");
    printk("=====================================\n");
    printk("micro:bit v2 + Kosmos Proxi Robot\n");
    printk("Long-press Button A: Pair controller\n");
    printk("Short-press Button B: Emergency stop\n");
    printk("Long-press Button B: Magnetometer calibration (60s)\n");
    printk("\n");
    printk("D-Pad Controls:\n");
    printk("  UP:    Start autonomous mode (heading-hold)\n");
    printk("  DOWN:  180 U-turn (auto) / Yaw test (manual)\n");
    printk("  LEFT:  -90 turn (auto) / System ID (manual)\n");
    printk("  RIGHT: +90 turn (auto only)\n");
    printk("\n");
    printk("Exit auto mode: Stick or trigger (manual override)\n");
    printk("Triggers: Yaw rate control (max 20 deg/s)\n\n");

    /* Get display */
    disp = mb_display_get();
    if (disp == NULL) {
        printk("ERROR: Display not available\n");
        return -ENODEV;
    }

    /* Initialize sensors (IMU) */
    ret = sensors_init();
    if (ret != 0) {
        printk("WARNING: Sensors init failed (err %d)\n", ret);
        /* Continue without sensors */
    }

    /* Initialize BLE (both peripheral and central roles) */
    ret = smp_bt_init();
    if (ret != 0) {
        printk("ERROR: BLE init failed (err %d)\n", ret);
        mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, SYS_FOREVER_MS,
                         &img_error, 1);
        return ret;
    }

    /* Initialize BLE output (NUS for debugging) */
    ret = ble_output_init();
    if (ret != 0) {
        printk("WARNING: BLE output init failed (err %d)\n", ret);
    }

    /* Initialize BLE Central (for Xbox controller) */
    ret = ble_central_init(&central_callbacks);
    if (ret != 0) {
        printk("ERROR: BLE Central init failed (err %d)\n", ret);
    }

    /* Initialize HID parser */
    ret = hid_parser_init();
    if (ret != 0) {
        printk("WARNING: HID parser init failed (err %d)\n", ret);
    }

    /* Initialize motor driver */
    ret = motor_driver_init();
    if (ret != 0) {
        printk("WARNING: Motor driver init failed (err %d)\n", ret);
    }

    /* Initialize audio */
    ret = audio_init();
    if (ret != 0) {
        printk("WARNING: Audio init failed (err %d)\n", ret);
    }

    /* Initialize motor test mode */
    ret = motor_test_init();
    if (ret != 0) {
        printk("WARNING: Motor test init failed (err %d)\n", ret);
    }

    /* Initialize IR sensors */
    ret = ir_sensors_init();
    if (ret != 0) {
        printk("WARNING: IR sensors init failed (err %d)\n", ret);
    }

    /* Initialize autonomous navigation */
    ret = autonav_init();
    if (ret != 0) {
        printk("WARNING: Autonomous nav init failed (err %d)\n", ret);
    }

    /* Initialize yaw rate controller */
    ret = yaw_controller_init();
    if (ret != 0) {
        printk("WARNING: Yaw controller init failed (err %d)\n", ret);
    }

    /* Initialize telemetry */
    ret = telemetry_init();
    if (ret != 0) {
        printk("WARNING: Telemetry init failed (err %d)\n", ret);
    }

    /* Start all threads */
    sensors_start_thread();
    serial_output_start_thread();
    ble_output_start_thread();
    ble_central_start_thread();
    motor_driver_start_thread();
    audio_start_thread();
    motor_test_start_thread();
    ir_sensors_start_thread();
    autonav_start_thread();
    telemetry_start_thread();

    printk("All threads started.\n");

    /* Show startup checkmark briefly */
    mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, 1000, &img_check, 1);
    k_msleep(1000);

    /* Initial display state - heart animation */
    robot_set_state(ROBOT_STATE_IDLE);
    update_display_for_state(ROBOT_STATE_IDLE);

    /* Play startup sound */
    audio_play(SOUND_CONNECTED);

    /* Try to auto-reconnect to bonded controller */
    if (ble_central_has_bonded_controller()) {
        printk("Found bonded controller, starting auto-reconnect...\n");
        ble_central_start_reconnect();
    } else {
        printk("Ready. Waiting for controller pairing...\n\n");
    }

    /* Main loop - monitor state and handle long press */
    while (1) {
        k_msleep(50);

        /* Check for long press on Button A (pairing) */
        if (button_a_pressed && !long_press_triggered) {
            int64_t elapsed = k_uptime_get() - button_a_press_time;
            if (elapsed >= LONG_PRESS_MS) {
                long_press_triggered = true;
                handle_long_press_a();
            }
        }

        /* Check for long press on Button B (Magnetometer calibration - 3 seconds) */
        if (button_b_pressed && !button_b_long_press_triggered) {
            int64_t elapsed = k_uptime_get() - button_b_press_time;
            if (elapsed >= LONG_PRESS_CAL_MS) {
                button_b_long_press_triggered = true;

                /* Toggle magnetometer calibration */
                if (sensors_is_calibrating()) {
                    /* Already calibrating - can't stop early, just notify */
                    printk("Magnetometer calibration in progress...\n");
                } else {
                    /* Start magnetometer calibration (30 seconds) */
                    printk("Starting magnetometer calibration (30s)...\n");
                    printk("Slowly rotate the device in ALL directions!\n");
                    sensors_start_calibration();
                    audio_play(SOUND_PAIRING_START);
                }
            }
        }

        /* Sync display state with autonomous navigation */
        if (robot_is_controller_connected()) {
            if (autonav_is_enabled() && robot_get_state() != ROBOT_STATE_AUTONOMOUS) {
                robot_set_state(ROBOT_STATE_AUTONOMOUS);
            } else if (!autonav_is_enabled() && robot_get_state() == ROBOT_STATE_AUTONOMOUS) {
                robot_set_state(ROBOT_STATE_CONNECTED);
            }
        }

        /* Update flashing emoji (if active) or normal display state */
        /* Skip display update during calibration (handled separately) */
        if (!sensors_is_calibrating() && !update_emoji_flash()) {
            /* No emoji active, update display based on robot state */
            enum robot_state state = robot_get_state();
            update_display_for_state(state);
        }

        /* Handle magnetometer calibration */
        bool is_calibrating = sensors_is_calibrating();

        if (is_calibrating) {
            if (!was_calibrating) {
                /* Calibration just started */
                was_calibrating = true;
                audio_set_muted(true);  /* Mute other sounds */
                last_calibration_beep_time = k_uptime_get();
                calibration_animation_frame = 0;
                printk("Calibration started - muting other sounds\n");

                /* Show first animation frame */
                mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, SYS_FOREVER_MS,
                                 &calibration_animation[0], 1);
            }

            /* Update animation (rotating compass needle) */
            int64_t now = k_uptime_get();
            static int64_t last_anim_time = 0;
            if (now - last_anim_time >= CALIBRATION_ANIM_INTERVAL_MS) {
                last_anim_time = now;
                calibration_animation_frame = (calibration_animation_frame + 1) %
                                              ARRAY_SIZE(calibration_animation);
                mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, SYS_FOREVER_MS,
                                 &calibration_animation[calibration_animation_frame], 1);
            }

            /* Periodic beep during calibration */
            if (now - last_calibration_beep_time >= CALIBRATION_BEEP_INTERVAL_MS) {
                last_calibration_beep_time = now;
                audio_play(SOUND_CALIBRATION_BEEP);
            }
        } else if (was_calibrating) {
            /* Calibration just finished */
            was_calibrating = false;
            audio_set_muted(false);  /* Unmute sounds */
            printk("Calibration complete - unmuting sounds\n");

            /* Play success sound */
            audio_play(SOUND_CALIBRATION_DONE);

            /* Show checkmark briefly then return to current state */
            mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, 1500, &img_check, 1);
            k_msleep(1500);
            current_display_state = -1;  /* Force display update */
        }
    }

    return 0;
}
