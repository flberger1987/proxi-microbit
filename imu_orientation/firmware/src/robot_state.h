/*
 * SPDX-License-Identifier: Apache-2.0
 * Robot State and Inter-Thread Communication
 *
 * Central definitions for the Kosmos Proxi robot state machine
 * and message queues for thread communication.
 */

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Robot State Machine
 * ============================================================================ */

/**
 * Robot operating states
 */
enum robot_state {
    ROBOT_STATE_IDLE,       /* Idle, showing heart animation */
    ROBOT_STATE_PAIRING,    /* BLE scanning for controller */
    ROBOT_STATE_CONNECTED,  /* Controller connected, showing eyes */
    ROBOT_STATE_AUTONOMOUS, /* Autonomous navigation active */
};

/* ============================================================================
 * Display States
 * ============================================================================ */

/**
 * Display animation states
 */
enum display_state {
    DISPLAY_HEART,      /* Pulsing heart (idle) */
    DISPLAY_PAIRING,    /* Rotating dot (pairing) */
    DISPLAY_EYES,       /* Glowing eyes (connected) */
    DISPLAY_ERROR,      /* X mark (error) */
    DISPLAY_CHECK,      /* Checkmark (success) */
};

/* ============================================================================
 * Sound Events
 * ============================================================================ */

/**
 * Sound events for audio thread
 */
enum sound_event {
    SOUND_NONE = 0,
    SOUND_PAIRING_START,    /* Ascending tone */
    SOUND_CONNECTED,        /* Success melody */
    SOUND_DISCONNECTED,     /* Descending tone */
    SOUND_OBSTACLE,         /* Warning beep */
    SOUND_BUTTON_PRESS,     /* Short click */
    SOUND_ERROR,            /* Error tone */
    SOUND_SHOOT,            /* Laser/shooting sound effect */
    SOUND_MACHINEGUN,       /* Machine gun rattle */
    SOUND_CALIBRATION_BEEP, /* Short beep during calibration */
    SOUND_CALIBRATION_DONE, /* Success jingle for calibration complete */
};

/* ============================================================================
 * Motor Commands
 * ============================================================================ */

/**
 * Motor command structure
 * Values range from -100 to +100
 */
struct motor_cmd {
    int16_t linear;     /* Forward/backward: -100 to +100 */
    int16_t angular;    /* Turn left/right: -100 to +100 */
    bool emergency_stop; /* Immediate stop */
};

/* ============================================================================
 * Controller Input
 * ============================================================================ */

/**
 * Xbox Controller button masks
 */
#define XBOX_BTN_A          0x0001
#define XBOX_BTN_B          0x0002
#define XBOX_BTN_X          0x0004
#define XBOX_BTN_Y          0x0008
#define XBOX_BTN_LB         0x0010
#define XBOX_BTN_RB         0x0020
#define XBOX_BTN_VIEW       0x0040
#define XBOX_BTN_MENU       0x0080
#define XBOX_BTN_LSTICK     0x0100
#define XBOX_BTN_RSTICK     0x0200
#define XBOX_BTN_XBOX       0x0400

/**
 * D-Pad values (hat switch) for PS5 Wireless Controller BLE
 * PS5 uses 1-indexed format with 0 as neutral:
 * 0=Neutral, 1=Up, 2=Up-Right, 3=Right, 4=Down-Right,
 * 5=Down, 6=Down-Left, 7=Left, 8=Up-Left
 *
 * Note: Xbox uses 0-indexed (0=Up, 8=Neutral) but we support PS5 here
 */
#define XBOX_DPAD_NONE      0x00  /* Neutral - no direction pressed */
#define XBOX_DPAD_UP        0x01
#define XBOX_DPAD_UP_RIGHT  0x02
#define XBOX_DPAD_RIGHT     0x03
#define XBOX_DPAD_DOWN_RIGHT 0x04
#define XBOX_DPAD_DOWN      0x05
#define XBOX_DPAD_DOWN_LEFT 0x06
#define XBOX_DPAD_LEFT      0x07
#define XBOX_DPAD_UP_LEFT   0x08

/**
 * Parsed Xbox Controller input
 */
struct xbox_input {
    uint16_t buttons;       /* Button bitmask */
    int16_t left_stick_x;   /* -32768 to 32767 */
    int16_t left_stick_y;   /* -32768 to 32767 */
    int16_t right_stick_x;  /* -32768 to 32767 */
    int16_t right_stick_y;  /* -32768 to 32767 */
    uint16_t left_trigger;  /* 0 to 1023 */
    uint16_t right_trigger; /* 0 to 1023 */
    uint8_t dpad;           /* Hat switch value */
};

/* ============================================================================
 * Obstacle Detection
 * ============================================================================ */

/**
 * Obstacle detection info from IR sensors
 */
struct obstacle_info {
    bool left_detected;     /* Obstacle on left */
    bool right_detected;    /* Obstacle on right */
    uint16_t left_value;    /* Raw ADC value */
    uint16_t right_value;   /* Raw ADC value */
};

/* ============================================================================
 * Message Queues (extern declarations)
 * ============================================================================ */

/* Controller → Kinematic: Motor commands */
extern struct k_msgq motor_cmd_q;

/* Any → Display: Display state changes */
extern struct k_msgq display_state_q;

/* Sensors → Kinematic: Obstacle detection */
extern struct k_msgq obstacle_q;

/* Any → Audio: Sound events */
extern struct k_msgq sound_q;

/* ============================================================================
 * Global State (extern declarations)
 * ============================================================================ */

/**
 * Get current robot state
 */
enum robot_state robot_get_state(void);

/**
 * Set robot state (thread-safe)
 */
void robot_set_state(enum robot_state state);

/**
 * Check if Xbox controller is connected
 */
bool robot_is_controller_connected(void);

#endif /* ROBOT_STATE_H */
