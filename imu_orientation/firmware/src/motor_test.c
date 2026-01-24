/*
 * SPDX-License-Identifier: Apache-2.0
 * Motor Test Mode - Pin testing via serial commands
 *
 * Commands (via serial at 115200 baud):
 *   0 1    - Set P0 HIGH (100% PWM)
 *   0 0    - Set P0 LOW (0% PWM)
 *   0 50   - Set P0 to 50% PWM
 *   1 1    - Set P1 HIGH
 *   2 1    - Set P2 HIGH
 *   ...
 *   ALL 0  - All pins LOW
 *   SCAN   - Scan through all pins one by one
 *   HELP   - Show commands
 */

#include "motor_test.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>  /* for strncasecmp */

/* Thread configuration */
#define TEST_STACK_SIZE 1024
#define TEST_PRIORITY 8

K_THREAD_STACK_DEFINE(test_stack, TEST_STACK_SIZE);
static struct k_thread test_thread_data;

/* GPIO device */
static const struct device *gpio0_dev;
static const struct device *gpio1_dev;

/* Edge connector pin mapping (micro:bit pin -> nRF GPIO) */
static const struct {
    uint8_t microbit_pin;
    uint8_t gpio_port;  /* 0 or 1 */
    uint8_t gpio_pin;
    const char *name;
} pin_map[] = {
    { 0,  0,  2, "P0  (GPIO P0.02)" },
    { 1,  0,  3, "P1  (GPIO P0.03)" },
    { 2,  0,  4, "P2  (GPIO P0.04)" },
    { 8,  0, 10, "P8  (GPIO P0.10)" },
    { 12, 0, 12, "P12 (GPIO P0.12)" },
    { 13, 0, 17, "P13 (GPIO P0.17) - SPI SCK" },
    { 14, 0,  1, "P14 (GPIO P0.01) - SPI MISO" },
    { 15, 0, 13, "P15 (GPIO P0.13) - SPI MOSI" },
    { 16, 1,  2, "P16 (GPIO P1.02)" },
};
#define NUM_PINS (sizeof(pin_map) / sizeof(pin_map[0]))

/* PWM period */
#define PWM_PERIOD_NS 1000000  /* 1kHz */

/* Current pin states */
static uint8_t pin_states[NUM_PINS];

/* UART device for commands */
static const struct device *uart_dev;

/* ============================================================================
 * GPIO Functions
 * ============================================================================ */

static const struct device *get_gpio_dev(uint8_t port)
{
    return (port == 0) ? gpio0_dev : gpio1_dev;
}

static int set_pin_output(int idx, int value)
{
    if (idx < 0 || idx >= NUM_PINS) {
        return -EINVAL;
    }

    const struct device *dev = get_gpio_dev(pin_map[idx].gpio_port);
    if (!device_is_ready(dev)) {
        printk("GPIO port %d not ready\n", pin_map[idx].gpio_port);
        return -ENODEV;
    }

    /* Configure as output */
    int ret = gpio_pin_configure(dev, pin_map[idx].gpio_pin, GPIO_OUTPUT);
    if (ret != 0) {
        printk("Failed to configure pin: %d\n", ret);
        return ret;
    }

    /* Set value */
    if (value == 0) {
        ret = gpio_pin_set(dev, pin_map[idx].gpio_pin, 0);
        pin_states[idx] = 0;
        printk("%s = LOW\n", pin_map[idx].name);
    } else if (value == 1 || value >= 100) {
        ret = gpio_pin_set(dev, pin_map[idx].gpio_pin, 1);
        pin_states[idx] = 100;
        printk("%s = HIGH\n", pin_map[idx].name);
    } else {
        /* PWM value - for now just use HIGH/LOW threshold */
        int level = (value >= 50) ? 1 : 0;
        ret = gpio_pin_set(dev, pin_map[idx].gpio_pin, level);
        pin_states[idx] = value;
        printk("%s = %d%% (GPIO %s)\n", pin_map[idx].name, value, level ? "HIGH" : "LOW");
    }

    return ret;
}

static void all_pins_low(void)
{
    printk("\n--- ALL PINS LOW ---\n");
    for (int i = 0; i < NUM_PINS; i++) {
        set_pin_output(i, 0);
    }
    printk("--------------------\n\n");
}

static void scan_pins(void)
{
    printk("\n=== PIN SCAN START ===\n");
    printk("Each pin will be HIGH for 2 seconds\n");
    printk("Watch which motor/LED reacts!\n\n");

    for (int i = 0; i < NUM_PINS; i++) {
        /* All low first */
        for (int j = 0; j < NUM_PINS; j++) {
            set_pin_output(j, 0);
        }

        printk("\n>>> Testing: %s <<<\n", pin_map[i].name);
        set_pin_output(i, 1);

        k_msleep(2000);
    }

    all_pins_low();
    printk("=== PIN SCAN DONE ===\n\n");
}

static void show_help(void)
{
    printk("\n");
    printk("=== MOTOR TEST COMMANDS ===\n");
    printk("Format: <pin> <value>\n");
    printk("  pin:   0, 1, 2, 8, 12, 13, 14, 15, 16\n");
    printk("  value: 0=LOW, 1=HIGH, 50=50%%\n");
    printk("\n");
    printk("Examples:\n");
    printk("  0 1     - P0 HIGH\n");
    printk("  0 0     - P0 LOW\n");
    printk("  1 1     - P1 HIGH\n");
    printk("  2 1     - P2 HIGH\n");
    printk("\n");
    printk("Special commands:\n");
    printk("  ALL 0   - All pins LOW\n");
    printk("  SCAN    - Test each pin for 2 sec\n");
    printk("  STATUS  - Show current pin states\n");
    printk("  HELP    - Show this help\n");
    printk("\n");
    printk("Available pins:\n");
    for (int i = 0; i < NUM_PINS; i++) {
        printk("  %d = %s\n", pin_map[i].microbit_pin, pin_map[i].name);
    }
    printk("===========================\n\n");
}

static void show_status(void)
{
    printk("\n--- PIN STATUS ---\n");
    for (int i = 0; i < NUM_PINS; i++) {
        printk("  P%d: %d%%\n", pin_map[i].microbit_pin, pin_states[i]);
    }
    printk("------------------\n\n");
}

static int find_pin_index(int microbit_pin)
{
    for (int i = 0; i < NUM_PINS; i++) {
        if (pin_map[i].microbit_pin == microbit_pin) {
            return i;
        }
    }
    return -1;
}

static void process_command(char *cmd)
{
    /* Trim whitespace */
    while (*cmd && isspace((unsigned char)*cmd)) cmd++;
    char *end = cmd + strlen(cmd) - 1;
    while (end > cmd && isspace((unsigned char)*end)) *end-- = '\0';

    if (strlen(cmd) == 0) {
        return;
    }

    printk("CMD: '%s'\n", cmd);

    /* Check special commands */
    if (strncasecmp(cmd, "HELP", 4) == 0) {
        show_help();
        return;
    }
    if (strncasecmp(cmd, "SCAN", 4) == 0) {
        scan_pins();
        return;
    }
    if (strncasecmp(cmd, "STATUS", 6) == 0) {
        show_status();
        return;
    }
    if (strncasecmp(cmd, "ALL", 3) == 0) {
        all_pins_low();
        return;
    }

    /* Parse: <pin> <value> */
    int pin, value;
    if (sscanf(cmd, "%d %d", &pin, &value) == 2) {
        int idx = find_pin_index(pin);
        if (idx >= 0) {
            set_pin_output(idx, value);
        } else {
            printk("Unknown pin: %d\n", pin);
            printk("Valid pins: 0, 1, 2, 8, 12, 13, 14, 15, 16\n");
        }
        return;
    }

    printk("Unknown command. Type HELP for usage.\n");
}

/* ============================================================================
 * Test Thread - reads serial commands
 * ============================================================================ */

static void test_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    char cmd_buf[64];
    int cmd_idx = 0;

    printk("\n");
    printk("========================================\n");
    printk("   MOTOR TEST MODE ACTIVE\n");
    printk("   Type HELP for commands\n");
    printk("========================================\n\n");

    show_help();

    while (1) {
        /* Poll for UART input */
        if (uart_dev && device_is_ready(uart_dev)) {
            uint8_t c;
            while (uart_poll_in(uart_dev, &c) == 0) {
                if (c == '\r' || c == '\n') {
                    if (cmd_idx > 0) {
                        cmd_buf[cmd_idx] = '\0';
                        process_command(cmd_buf);
                        cmd_idx = 0;
                    }
                } else if (cmd_idx < sizeof(cmd_buf) - 1) {
                    cmd_buf[cmd_idx++] = c;
                }
            }
        }

        k_msleep(50);
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int motor_test_init(void)
{
    /* Get GPIO devices */
    gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

    if (!device_is_ready(gpio0_dev)) {
        printk("Motor Test: GPIO0 not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(gpio1_dev)) {
        printk("Motor Test: GPIO1 not ready\n");
        return -ENODEV;
    }

    /* Get UART device */
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart_dev)) {
        printk("Motor Test: UART not ready\n");
        return -ENODEV;
    }

    /* Initialize all pins to low */
    memset(pin_states, 0, sizeof(pin_states));

    printk("Motor Test: Initialized\n");
    return 0;
}

void motor_test_start_thread(void)
{
    k_thread_create(&test_thread_data, test_stack,
                    K_THREAD_STACK_SIZEOF(test_stack),
                    test_thread_fn, NULL, NULL, NULL,
                    TEST_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&test_thread_data, "motor_test");
}
