/*
 * SPDX-License-Identifier: Apache-2.0
 * Robot State and Inter-Thread Communication
 */

#include "robot_state.h"
#include <zephyr/kernel.h>

/* ============================================================================
 * Message Queue Definitions
 * ============================================================================ */

/* Controller → Kinematic: Motor commands (4 items, align 4) */
K_MSGQ_DEFINE(motor_cmd_q, sizeof(struct motor_cmd), 4, 4);

/* Any → Display: Display state changes (2 items, align 4) */
K_MSGQ_DEFINE(display_state_q, sizeof(enum display_state), 2, 4);

/* Sensors → Kinematic: Obstacle detection (4 items, align 4) */
K_MSGQ_DEFINE(obstacle_q, sizeof(struct obstacle_info), 4, 4);

/* Any → Audio: Sound events (8 items, align 4) */
K_MSGQ_DEFINE(sound_q, sizeof(enum sound_event), 8, 4);

/* ============================================================================
 * Global State
 * ============================================================================ */

static enum robot_state current_state = ROBOT_STATE_IDLE;
static struct k_mutex state_mutex;
static bool state_mutex_initialized = false;

static void ensure_mutex_init(void)
{
    if (!state_mutex_initialized) {
        k_mutex_init(&state_mutex);
        state_mutex_initialized = true;
    }
}

enum robot_state robot_get_state(void)
{
    ensure_mutex_init();

    enum robot_state state;
    k_mutex_lock(&state_mutex, K_FOREVER);
    state = current_state;
    k_mutex_unlock(&state_mutex);
    return state;
}

void robot_set_state(enum robot_state state)
{
    ensure_mutex_init();

    k_mutex_lock(&state_mutex, K_FOREVER);
    current_state = state;
    k_mutex_unlock(&state_mutex);
}

bool robot_is_controller_connected(void)
{
    return robot_get_state() == ROBOT_STATE_CONNECTED;
}
