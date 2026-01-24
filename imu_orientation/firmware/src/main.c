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

/* ============================================================================
 * Global State
 * ============================================================================ */

static struct mb_display *disp;
static enum robot_state current_display_state = -1;  /* Force first update */

/* Long press detection */
#define LONG_PRESS_MS 1000
static volatile int64_t button_a_press_time;
static volatile bool button_a_pressed;
static volatile bool long_press_triggered;

/* ============================================================================
 * Display Update
 * ============================================================================ */

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
}

static void on_controller_disconnected(void)
{
    printk("Controller disconnected!\n");
    audio_play(SOUND_DISCONNECTED);
    motor_enable(false);
    motor_emergency_stop();
}

/* Previous button state for edge detection */
static uint16_t prev_controller_buttons = 0;

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

    /* A button (Cross on PS5) → Shoot sound! */
    if (just_pressed & XBOX_BTN_A) {
        audio_play(SOUND_SHOOT);
    }

    /* B button (Circle on PS5) → Click sound */
    if (just_pressed & XBOX_BTN_B) {
        audio_play(SOUND_BUTTON_PRESS);
    }

    /* X button (Square on PS5) → Machine gun! */
    if (just_pressed & XBOX_BTN_X) {
        audio_play(SOUND_MACHINEGUN);
    }

    /* Y button (Triangle on PS5) → Obstacle warning */
    if (just_pressed & XBOX_BTN_Y) {
        audio_play(SOUND_OBSTACLE);
    }

    /* Update previous button state */
    prev_controller_buttons = input.buttons;

    /* Convert to motor command */
    hid_input_to_motor_cmd(&input, &cmd);

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

    /* Button B handling */
    if (evt->code == INPUT_KEY_B && evt->value == 1) {
        /* Reserved for future use */
        audio_play(SOUND_BUTTON_PRESS);

        /* For testing: Emergency stop */
        if (robot_get_state() == ROBOT_STATE_CONNECTED) {
            motor_emergency_stop();
            printk("Emergency stop triggered\n");
        }
    }
}

INPUT_CALLBACK_DEFINE(NULL, button_cb, NULL);

/* ============================================================================
 * Calibration (from original IMU firmware)
 * ============================================================================ */

static volatile bool was_calibrating = false;

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
    printk("Long-press Button A to pair controller\n");
    printk("Button B: Emergency stop\n\n");

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

    /* Start all threads */
    sensors_start_thread();
    serial_output_start_thread();
    ble_output_start_thread();
    ble_central_start_thread();
    motor_driver_start_thread();
    audio_start_thread();
    motor_test_start_thread();

    printk("All threads started.\n");

    /* Show startup checkmark briefly */
    mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, 1000, &img_check, 1);
    k_msleep(1000);

    /* Initial display state - heart animation */
    robot_set_state(ROBOT_STATE_IDLE);
    update_display_for_state(ROBOT_STATE_IDLE);

    /* Play startup sound */
    audio_play(SOUND_CONNECTED);

    printk("Ready. Waiting for controller pairing...\n\n");

    /* Main loop - monitor state and handle long press */
    while (1) {
        k_msleep(50);

        /* Check for long press on Button A */
        if (button_a_pressed && !long_press_triggered) {
            int64_t elapsed = k_uptime_get() - button_a_press_time;
            if (elapsed >= LONG_PRESS_MS) {
                long_press_triggered = true;
                handle_long_press_a();
            }
        }

        /* Update display based on robot state */
        enum robot_state state = robot_get_state();
        update_display_for_state(state);

        /* Handle sensor calibration (legacy feature) */
        if (was_calibrating && !sensors_is_calibrating()) {
            was_calibrating = false;
            /* Show checkmark briefly then return to current state */
            mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, 1000, &img_check, 1);
            k_msleep(1000);
            current_display_state = ROBOT_STATE_IDLE;  /* Force display update */
        }
        if (sensors_is_calibrating() && !was_calibrating) {
            was_calibrating = true;
        }
    }

    return 0;
}
