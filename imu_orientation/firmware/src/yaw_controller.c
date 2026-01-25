/*
 * SPDX-License-Identifier: Apache-2.0
 * Yaw Rate Controller Implementation
 *
 * PI controller for yaw rate with anti-windup.
 */

#include "yaw_controller.h"
#include "robot_state.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Controller state */
static volatile bool controller_enabled = false;
static volatile float target_yaw_rate = 0.0f;

/* PI controller state */
static float integral_term = 0.0f;
static float last_error = 0.0f;
static float last_output = 0.0f;

/* Anti-windup limits */
#define INTEGRAL_MAX 50.0f
#define OUTPUT_MAX 100.0f

/* Trigger threshold (ignore small values) - 10% of max */
#define TRIGGER_THRESHOLD 100

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

static inline float clamp_f(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int yaw_controller_init(void)
{
    controller_enabled = false;
    target_yaw_rate = 0.0f;
    integral_term = 0.0f;
    last_error = 0.0f;
    last_output = 0.0f;

    printk("Yaw Controller: Initialized (Kp=%.1f, Ki=%.1f, max=%.0f°/s)\n",
           (double)YAW_KP, (double)YAW_KI, (double)YAW_RATE_MAX);
    return 0;
}

void yaw_controller_enable(bool enable)
{
    if (enable && !controller_enabled) {
        /* Reset controller when enabling */
        yaw_controller_reset();
    }
    controller_enabled = enable;
    printk("Yaw Controller: %s\n", enable ? "ENABLED" : "DISABLED");
}

bool yaw_controller_is_enabled(void)
{
    return controller_enabled;
}

void yaw_controller_set_triggers(uint16_t left_trigger, uint16_t right_trigger)
{
    /* Apply threshold */
    int32_t lt = (left_trigger > TRIGGER_THRESHOLD) ? left_trigger : 0;
    int32_t rt = (right_trigger > TRIGGER_THRESHOLD) ? right_trigger : 0;

    /* Map triggers to yaw rate:
     * L2 full = -YAW_RATE_MAX (turn left)
     * R2 full = +YAW_RATE_MAX (turn right)
     * Both or neither = 0
     */
    float rate = 0.0f;

    if (lt > 0 || rt > 0) {
        /* Scale from 0-1023 to 0-YAW_RATE_MAX */
        float left_rate = (float)lt * YAW_RATE_MAX / 1023.0f;
        float right_rate = (float)rt * YAW_RATE_MAX / 1023.0f;
        rate = right_rate - left_rate;
    }

    target_yaw_rate = clamp_f(rate, -YAW_RATE_MAX, YAW_RATE_MAX);
}

void yaw_controller_set_target(float target_rate)
{
    target_yaw_rate = clamp_f(target_rate, -YAW_RATE_MAX, YAW_RATE_MAX);
}

float yaw_controller_get_target(void)
{
    return target_yaw_rate;
}

int16_t yaw_controller_update(float measured_rate)
{
    if (!controller_enabled) {
        /* Pass-through mode: return 0 (no control action) */
        return 0;
    }

    /* If target is near zero (< 5°/s), don't control - just stop */
    if (target_yaw_rate > -5.0f && target_yaw_rate < 5.0f) {
        /* Reset integral to prevent windup when idle */
        integral_term = 0.0f;
        last_output = 0.0f;
        return 0;
    }

    /*
     * Feedforward + PI Controller
     *
     * Feedforward: output = (target / max_rate) * feedforward_gain
     */
    #define FEEDFORWARD_GAIN 100.0f /* 100% PWM for 40°/s max */

    float ff_term = (target_yaw_rate / YAW_RATE_MAX) * FEEDFORWARD_GAIN;

    /* Calculate error */
    float error = target_yaw_rate - measured_rate;

    /* Proportional term */
    float p_term = YAW_KP * error;

    /* Derivative term (on error change, helps with initial acceleration) */
    float error_derivative = (error - last_error) / 0.02f;  /* dt = 20ms */
    float d_term = YAW_KD * error_derivative;

    /* Integral term with anti-windup */
    bool saturated_positive = (last_output >= OUTPUT_MAX) && (error > 0);
    bool saturated_negative = (last_output <= -OUTPUT_MAX) && (error < 0);

    if (!saturated_positive && !saturated_negative) {
        /* dt = 20ms = 0.02s (50 Hz update rate) */
        integral_term += YAW_KI * error * 0.02f;
        integral_term = clamp_f(integral_term, -INTEGRAL_MAX, INTEGRAL_MAX);
    }

    /* Calculate output: PID */
    float output = ff_term + p_term + integral_term + d_term;

    /* Constrain output to same sign as target (unidirectional control) */
    if (target_yaw_rate > 0 && output < 0) {
        output = 0;
        integral_term = clamp_f(integral_term, -INTEGRAL_MAX, 0);
    } else if (target_yaw_rate < 0 && output > 0) {
        output = 0;
        integral_term = clamp_f(integral_term, 0, INTEGRAL_MAX);
    }

    /* Clamp output to motor range */
    output = clamp_f(output, -OUTPUT_MAX, OUTPUT_MAX);

    /* Store for next iteration */
    last_error = error;
    last_output = output;

    return (int16_t)output;
}

void yaw_controller_reset(void)
{
    integral_term = 0.0f;
    last_error = 0.0f;
    last_output = 0.0f;
    target_yaw_rate = 0.0f;
}

void yaw_controller_get_debug(float *error, float *integral, float *output)
{
    if (error) *error = last_error;
    if (integral) *integral = integral_term;
    if (output) *output = last_output;
}
