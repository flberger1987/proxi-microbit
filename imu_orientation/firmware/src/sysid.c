/*
 * SPDX-License-Identifier: Apache-2.0
 * System Identification for Yaw Dynamics
 *
 * Records step responses to identify motor/inertia parameters:
 * - Time constant τ
 * - Gain K (steady-state ω per PWM %)
 * - From these: J, b, k for EKF model
 */

#include "sysid.h"
#include "motor_driver.h"
#include "sensors.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Test sequence configuration */
#define SYSID_SAMPLE_RATE_MS  10   /* 100 Hz sampling */
#define SYSID_SETTLE_TIME_MS  500  /* Wait before each step */
#define SYSID_STEP_TIME_MS    2000 /* Duration of each step */
#define SYSID_PAUSE_TIME_MS   1000 /* Pause between steps */

/* PWM levels to test (%) */
static const int16_t test_levels[] = {30, 50, 70, 100, -30, -50, -70, -100};
#define NUM_TEST_LEVELS (sizeof(test_levels) / sizeof(test_levels[0]))

static volatile bool sysid_running = false;
static volatile bool sysid_abort = false;

bool sysid_is_running(void)
{
    return sysid_running;
}

void sysid_abort_test(void)
{
    sysid_abort = true;
}

/**
 * Run a single step response test
 * Outputs: SYSID,<time_ms>,<pwm>,<yaw_rate>
 */
static void run_step_test(int16_t pwm_level)
{
    int64_t start_time;
    int64_t elapsed;
    float yaw_rate;

    printk("\n# Step test: PWM = %d%%\n", pwm_level);
    printk("# Settling...\n");

    /* Ensure motor is stopped */
    motor_set_velocity(0, 0);
    k_msleep(SYSID_SETTLE_TIME_MS);

    if (sysid_abort) return;

    /* Record baseline */
    printk("# Recording baseline (500ms)...\n");
    start_time = k_uptime_get();
    while ((elapsed = k_uptime_get() - start_time) < 500) {
        yaw_rate = sensors_get_yaw_rate();
        printk("SYSID,%lld,0,%.2f\n", elapsed, (double)yaw_rate);
        k_msleep(SYSID_SAMPLE_RATE_MS);
        if (sysid_abort) return;
    }

    /* Apply step */
    printk("# Applying step: PWM = %d%%\n", pwm_level);
    motor_set_velocity(0, pwm_level);  /* angular only */

    start_time = k_uptime_get();
    while ((elapsed = k_uptime_get() - start_time) < SYSID_STEP_TIME_MS) {
        yaw_rate = sensors_get_yaw_rate();
        printk("SYSID,%lld,%d,%.2f\n", 500 + elapsed, pwm_level, (double)yaw_rate);
        k_msleep(SYSID_SAMPLE_RATE_MS);
        if (sysid_abort) return;
    }

    /* Stop and record decay */
    printk("# Motor off, recording decay...\n");
    motor_set_velocity(0, 0);

    start_time = k_uptime_get();
    while ((elapsed = k_uptime_get() - start_time) < SYSID_PAUSE_TIME_MS) {
        yaw_rate = sensors_get_yaw_rate();
        printk("SYSID,%lld,0,%.2f\n", 500 + SYSID_STEP_TIME_MS + elapsed, (double)yaw_rate);
        k_msleep(SYSID_SAMPLE_RATE_MS);
        if (sysid_abort) return;
    }
}

void sysid_run_full_test(void)
{
    if (sysid_running) {
        printk("SYSID: Already running\n");
        return;
    }

    sysid_running = true;
    sysid_abort = false;

    printk("\n");
    printk("# ============================================\n");
    printk("# SYSTEM IDENTIFICATION - Yaw Dynamics\n");
    printk("# ============================================\n");
    printk("# Format: SYSID,<time_ms>,<pwm_percent>,<yaw_rate_deg_s>\n");
    printk("# Testing %d PWM levels\n", NUM_TEST_LEVELS);
    printk("# Press Button B to abort\n");
    printk("# ============================================\n\n");

    /* Disable yaw controller for direct motor control */
    yaw_controller_enable(false);
    motor_enable(true);

    for (int i = 0; i < NUM_TEST_LEVELS && !sysid_abort; i++) {
        printk("\n# Test %d/%d\n", i + 1, NUM_TEST_LEVELS);
        run_step_test(test_levels[i]);
    }

    /* Stop motor */
    motor_set_velocity(0, 0);
    motor_enable(false);

    if (sysid_abort) {
        printk("\n# SYSID: Aborted by user\n");
    } else {
        printk("\n# ============================================\n");
        printk("# SYSID: Complete!\n");
        printk("# ============================================\n");
        printk("# Next: Plot data and fit first-order model:\n");
        printk("#   ω(t) = K * PWM * (1 - e^(-t/τ))\n");
        printk("# Parameters to extract:\n");
        printk("#   K = steady-state gain (deg/s per PWM%%)\n");
        printk("#   τ = time constant (seconds)\n");
        printk("# ============================================\n");
    }

    sysid_running = false;
}
