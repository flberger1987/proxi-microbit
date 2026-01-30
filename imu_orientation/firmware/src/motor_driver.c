/*
 * SPDX-License-Identifier: Apache-2.0
 * Motor Driver for Kosmos Proxi Hexapod (PWM Version)
 *
 * The Proxi is a hexapod robot with two motors:
 * 1. Walk motor: Forward (P13) / Backward (P14)
 * 2. Turn motor: Left (P15) / Right (P16)
 *
 * Each motor is controlled via H-Bridge with PWM for speed control.
 * PWM duty cycle controls speed, direction by which pin is active.
 */

#include "motor_driver.h"
#include "robot_state.h"
#include "yaw_controller.h"
#include "sensors.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

/* Thread configuration */
#define MOTOR_STACK_SIZE 768
#define MOTOR_PRIORITY 2

K_THREAD_STACK_DEFINE(motor_stack, MOTOR_STACK_SIZE);
static struct k_thread motor_thread_data;
static k_tid_t motor_thread_id;

/*
 * PWM Motor Configuration via Device Tree
 */
#define WALK_FWD_NODE  DT_ALIAS(walk_fwd)
#define WALK_BWD_NODE  DT_ALIAS(walk_bwd)
#define TURN_LEFT_NODE DT_ALIAS(turn_left)
#define TURN_RIGHT_NODE DT_ALIAS(turn_right)

#if DT_NODE_EXISTS(WALK_FWD_NODE)
static const struct pwm_dt_spec walk_fwd = PWM_DT_SPEC_GET(WALK_FWD_NODE);
#else
static const struct pwm_dt_spec walk_fwd = {0};
#endif

#if DT_NODE_EXISTS(WALK_BWD_NODE)
static const struct pwm_dt_spec walk_bwd = PWM_DT_SPEC_GET(WALK_BWD_NODE);
#else
static const struct pwm_dt_spec walk_bwd = {0};
#endif

#if DT_NODE_EXISTS(TURN_LEFT_NODE)
static const struct pwm_dt_spec turn_left = PWM_DT_SPEC_GET(TURN_LEFT_NODE);
#else
static const struct pwm_dt_spec turn_left = {0};
#endif

#if DT_NODE_EXISTS(TURN_RIGHT_NODE)
static const struct pwm_dt_spec turn_right = PWM_DT_SPEC_GET(TURN_RIGHT_NODE);
#else
static const struct pwm_dt_spec turn_right = {0};
#endif

/* Motor state */
static volatile bool motors_enabled = false;

/* Current motor command (for telemetry) */
static volatile int8_t current_cmd_linear = 0;
static volatile int8_t current_cmd_angular = 0;

/* PWM period (1kHz = 1000us) */
#define PWM_PERIOD_NS 1000000

/* Minimum PWM to overcome motor deadzone (%) */
#define MIN_DUTY_PERCENT 20

/* Threshold for activation (avoid jitter from controller) */
#define ACTIVATION_THRESHOLD 15

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

static inline int16_t motor_clamp(int16_t val, int16_t min, int16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * Convert percent (-100 to 100) to PWM pulse width
 * Applies minimum duty cycle scaling
 */
static uint32_t percent_to_pulse(int16_t percent)
{
    if (percent <= 0) {
        return 0;
    }

    /* Scale 1-100 to MIN_DUTY_PERCENT-100 range */
    uint32_t scaled = MIN_DUTY_PERCENT +
                      ((uint32_t)percent * (100 - MIN_DUTY_PERCENT)) / 100;

    return (PWM_PERIOD_NS * scaled) / 100;
}

/**
 * Set walk motor speed and direction
 * direction: positive = forward, negative = backward, 0 = stop
 */
static void set_walk_motor(int16_t direction)
{
    if (!device_is_ready(walk_fwd.dev) || !device_is_ready(walk_bwd.dev)) {
        return;
    }

    direction = motor_clamp(direction, -100, 100);

    if (direction > ACTIVATION_THRESHOLD) {
        /* Forward */
        uint32_t pulse = percent_to_pulse(direction);
        pwm_set_dt(&walk_bwd, PWM_PERIOD_NS, 0);
        pwm_set_dt(&walk_fwd, PWM_PERIOD_NS, pulse);
    } else if (direction < -ACTIVATION_THRESHOLD) {
        /* Backward */
        uint32_t pulse = percent_to_pulse(-direction);
        pwm_set_dt(&walk_fwd, PWM_PERIOD_NS, 0);
        pwm_set_dt(&walk_bwd, PWM_PERIOD_NS, pulse);
    } else {
        /* Stop */
        pwm_set_dt(&walk_fwd, PWM_PERIOD_NS, 0);
        pwm_set_dt(&walk_bwd, PWM_PERIOD_NS, 0);
    }
}

/**
 * Set turn motor speed and direction
 * direction: positive = right, negative = left, 0 = stop
 */
static void set_turn_motor(int16_t direction)
{
    if (!device_is_ready(turn_left.dev) || !device_is_ready(turn_right.dev)) {
        return;
    }

    direction = motor_clamp(direction, -100, 100);

    if (direction > ACTIVATION_THRESHOLD) {
        /* Turn right */
        uint32_t pulse = percent_to_pulse(direction);
        pwm_set_dt(&turn_left, PWM_PERIOD_NS, 0);
        pwm_set_dt(&turn_right, PWM_PERIOD_NS, pulse);
    } else if (direction < -ACTIVATION_THRESHOLD) {
        /* Turn left */
        uint32_t pulse = percent_to_pulse(-direction);
        pwm_set_dt(&turn_right, PWM_PERIOD_NS, 0);
        pwm_set_dt(&turn_left, PWM_PERIOD_NS, pulse);
    } else {
        /* Stop */
        pwm_set_dt(&turn_left, PWM_PERIOD_NS, 0);
        pwm_set_dt(&turn_right, PWM_PERIOD_NS, 0);
    }
}

static void stop_all_motors(void)
{
    if (device_is_ready(walk_fwd.dev)) {
        pwm_set_dt(&walk_fwd, PWM_PERIOD_NS, 0);
    }
    if (device_is_ready(walk_bwd.dev)) {
        pwm_set_dt(&walk_bwd, PWM_PERIOD_NS, 0);
    }
    if (device_is_ready(turn_left.dev)) {
        pwm_set_dt(&turn_left, PWM_PERIOD_NS, 0);
    }
    if (device_is_ready(turn_right.dev)) {
        pwm_set_dt(&turn_right, PWM_PERIOD_NS, 0);
    }
}

/* ============================================================================
 * Motor Driver Thread
 * ============================================================================ */

static void motor_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct motor_cmd cmd;
    int16_t current_linear = 0;
    int16_t current_angular = 0;
    int ret;

    printk("Hexapod motor driver thread started (PWM)\n");

    while (1) {
        /* Note: Thread is suspended during OTA via k_thread_suspend() */

        /* Check for new motor command */
        ret = k_msgq_get(&motor_cmd_q, &cmd, K_MSEC(20));

        if (ret == 0) {
            if (cmd.emergency_stop) {
                current_linear = 0;
                current_angular = 0;
                current_cmd_linear = 0;
                current_cmd_angular = 0;
                stop_all_motors();
                yaw_controller_reset();
                printk("EMERGENCY STOP\n");
            } else {
                current_linear = cmd.linear;
                current_angular = cmd.angular;
                /* Note: telemetry values updated below with actual motor output */
            }
        }

        /* Update motors if enabled */
        if (motors_enabled) {
            int16_t angular_output;

            /* Use yaw rate controller if enabled */
            if (yaw_controller_is_enabled()) {
                /* Get current yaw rate from IMU */
                float measured_rate = sensors_get_yaw_rate();

                /* Update controller and get motor output */
                angular_output = yaw_controller_update(measured_rate);

                /* Debug output every 200ms for tuning */
                static int64_t last_debug = 0;
                int64_t now = k_uptime_get();
                if (now - last_debug >= 200) {
                    last_debug = now;
                    float target = yaw_controller_get_target();
                    float err, integ, out;
                    yaw_controller_get_debug(&err, &integ, &out);
                    printk("YAW,%.1f,%.1f,%d,%.1f,%.1f\n",
                           (double)target, (double)measured_rate, angular_output,
                           (double)err, (double)integ);
                }
            } else {
                /* Direct pass-through */
                angular_output = current_angular;
            }

            set_walk_motor(current_linear);
            set_turn_motor(angular_output);

            /* Update telemetry with ACTUAL output (not input command) */
            current_cmd_linear = (int8_t)current_linear;
            current_cmd_angular = (int8_t)angular_output;

            /* Feed motor command to Kalman filter for prediction */
            sensors_set_motor_cmd((float)angular_output);
        } else {
            stop_all_motors();
            sensors_set_motor_cmd(0.0f);
        }
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int motor_driver_init(void)
{
    bool walk_ok = device_is_ready(walk_fwd.dev) && device_is_ready(walk_bwd.dev);
    bool turn_ok = device_is_ready(turn_left.dev) && device_is_ready(turn_right.dev);

    if (!walk_ok) {
        printk("Motor: Walk PWM not ready\n");
    }
    if (!turn_ok) {
        printk("Motor: Turn PWM not ready\n");
    }

    /* Ensure all motors are stopped */
    stop_all_motors();

    printk("Hexapod Motor Driver initialized (PWM)\n");
    printk("  Walk: P13 (fwd) / P14 (bwd)\n");
    printk("  Turn: P15 (left) / P16 (right)\n");

    return (walk_ok && turn_ok) ? 0 : -ENODEV;
}

void motor_driver_start_thread(void)
{
    motor_thread_id = k_thread_create(&motor_thread_data, motor_stack,
                                      K_THREAD_STACK_SIZEOF(motor_stack),
                                      motor_thread_fn, NULL, NULL, NULL,
                                      MOTOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(motor_thread_id, "motor");
}

k_tid_t motor_get_thread_id(void)
{
    return motor_thread_id;
}

void motor_set_speeds(int16_t left_percent, int16_t right_percent)
{
    struct motor_cmd cmd = {
        .linear = (left_percent + right_percent) / 2,
        .angular = (right_percent - left_percent) / 2,
        .emergency_stop = false,
    };
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

void motor_set_velocity(int16_t linear, int16_t angular)
{
    struct motor_cmd cmd = {
        .linear = motor_clamp(linear, -100, 100),
        .angular = motor_clamp(angular, -100, 100),
        .emergency_stop = false,
    };
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

void motor_emergency_stop(void)
{
    struct motor_cmd cmd = {
        .linear = 0,
        .angular = 0,
        .emergency_stop = true,
    };
    k_msgq_purge(&motor_cmd_q);
    k_msgq_put(&motor_cmd_q, &cmd, K_NO_WAIT);
}

bool motor_is_active(void)
{
    return motors_enabled;
}

void motor_enable(bool enable)
{
    motors_enabled = enable;
    printk("Motors %s\n", enable ? "ENABLED" : "DISABLED");
    if (!enable) {
        stop_all_motors();
    }
}

void motor_get_current_cmd(int8_t *linear, int8_t *angular)
{
    if (linear) {
        *linear = current_cmd_linear;
    }
    if (angular) {
        *angular = current_cmd_angular;
    }
}
