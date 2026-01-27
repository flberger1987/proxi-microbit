/*
 * SPDX-License-Identifier: Apache-2.0
 * Infrared Obstacle Sensors Driver
 *
 * Uses ADC to read analog IR sensors on Kosmos Proxi.
 * Pin assignment (verified 2026-01-25):
 *   - Left sensor:  P0 (AIN0, P0.02)
 *   - Right sensor: P1 (AIN1, P0.03)
 *   - IR LED enable: P12 (P0.12, active-HIGH)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/gpio.h>

#include "ir_sensors.h"
#include "robot_state.h"

/* ============================================================================
 * Configuration
 * ============================================================================ */

/* ADC configuration for nRF52833 */
#define ADC_NODE            DT_NODELABEL(adc)
#define ADC_RESOLUTION      12              /* 12-bit resolution (0-4095) */
#define ADC_GAIN            ADC_GAIN_4      /* 4x gain for 0-0.15V range (maximum!) */
#define ADC_REFERENCE       ADC_REF_INTERNAL /* Internal 0.6V reference */
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_OVERSAMPLING    4               /* 16x oversampling (2^4) for noise reduction */

/* Sensor channels (nRF52833 AIN channels) */
#define IR_LEFT_CHANNEL     0   /* AIN0 = P0.02 = micro:bit P0 */
#define IR_RIGHT_CHANNEL    1   /* AIN1 = P0.03 = micro:bit P1 */
#define IR_EXTRA_CHANNEL    2   /* AIN2 = P0.04 = micro:bit P2 */

/* Possible IR LED enable pins (accent pins P8, P12) */
#define IR_LED_ENABLE_P8    DT_NODELABEL(gpio0)  /* P8 = P0.10 */
#define IR_LED_PIN_P8       10
#define IR_LED_ENABLE_P12   DT_NODELABEL(gpio0)  /* P12 = P0.12 */
#define IR_LED_PIN_P12      12

/* Thread configuration */
#define IR_THREAD_STACK_SIZE 1536  /* Increased for stability */
#define IR_THREAD_PRIORITY   8
#define IR_SAMPLE_INTERVAL_MS 50  /* 20 Hz sampling */

/*
 * Low-pass filter configuration (IIR 1st order)
 * y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 * For fc=10Hz cutoff at fs=20Hz sampling:
 * alpha = 2*pi*fc*dt / (2*pi*fc*dt + 1) ≈ 0.76
 *
 * Using fixed-point: alpha = 196/256 ≈ 0.766
 */
#define LP_FILTER_ALPHA     196   /* 0.766 in Q8 fixed-point (196/256) */
#define LP_FILTER_ONE_MINUS 60    /* 1 - alpha = 60/256 ≈ 0.234 */

/* Default calibration (verified 2026-01-25) */
#define DEFAULT_MIN         30      /* Typical no-obstacle value */
#define DEFAULT_MAX         3600    /* Typical close-obstacle value */
#define DEFAULT_THRESHOLD   15      /* 15% threshold (~560) for detection */

/*
 * IR raw-to-distance calibration (Least Squares Fit)
 * Measured data points:
 *   55cm=100, 35cm=350, 25cm=1050, 18cm=3000, 10cm=4000
 *
 * Power-law model: raw = A / d^n  =>  d = (A / raw)^(1/n)
 * Fitted: A ≈ 1,200,000, n ≈ 2.3, 1/n ≈ 0.435
 *
 * For integer math, use lookup table with linear interpolation.
 */
struct ir_cal_point {
    uint16_t raw;
    uint16_t mm;
};

static const struct ir_cal_point ir_cal_table[] = {
    {  100, 550 },   /* 55 cm */
    {  350, 350 },   /* 35 cm */
    { 1050, 250 },   /* 25 cm */
    { 3000, 180 },   /* 18 cm */
    { 4000, 100 },   /* 10 cm */
};
#define IR_CAL_TABLE_SIZE (sizeof(ir_cal_table) / sizeof(ir_cal_table[0]))

/* Startup bias calibration */
#define BIAS_CAL_SAMPLES    20      /* Number of samples for bias calibration */
#define BIAS_CAL_DELAY_MS   50      /* Delay between samples */

/* ============================================================================
 * Kalman Filter Configuration
 * ============================================================================
 * 1D Kalman filter for distance estimation
 * State: d (distance in mm)
 * Model: d[k+1] = d[k]  (constant position, obstacles move slowly)
 * Measurement: IR raw → mm via lookup table
 *
 * Parameters tuned for 20Hz sampling (50ms interval)
 */
#define KALMAN_Q_DISTANCE   100.0f   /* Process noise variance (mm²) - how much distance can change */
#define KALMAN_R_DISTANCE   400.0f   /* Measurement noise variance (mm²) - sensor noise */
#define KALMAN_INITIAL_P    10000.0f /* Initial estimation error variance */

/* ============================================================================
 * Thread and Stack
 * ============================================================================ */

K_THREAD_STACK_DEFINE(ir_thread_stack, IR_THREAD_STACK_SIZE);
static struct k_thread ir_thread_data;
static k_tid_t ir_thread_id;

/* ============================================================================
 * GPIO for IR LED control
 * ============================================================================ */

static const struct device *gpio0_dev;

/* Test both P8 and P12 as potential IR LED enable pins */
static const uint8_t ir_led_pins[] = {10, 12};  /* P0.10 (P8), P0.12 (P12) */
#define NUM_IR_LED_PINS ARRAY_SIZE(ir_led_pins)

/* GPIO test mode - all possible edge connector pins */
struct gpio_test_pin {
    uint8_t port;  /* 0 = gpio0, 1 = gpio1 */
    uint8_t pin;
    const char *name;
};

static const struct device *gpio1_dev;
static bool gpio_test_mode = false;
static int gpio_test_index = -1;
static bool gpio_test_active_low = false;  /* Test LOW-active pins */

/* Remaining candidate pins for IR LED enable:
 * - P0,P1 = ADC (IR sensors)
 * - P3,P4,P6,P7,P10 = LED display
 * - P5,P11 = Buttons
 * - P13,P14,P15,P16 = Motors
 * - P19,P20 = I2C (accelerometer)
 *
 * Candidates: P2, P8, P9, P12
 * P2 could be IR LED enable instead of 3rd sensor! */
static struct gpio_test_pin test_pins[] = {
    {0,  4, "P2  (P0.04) - AIN2/GPIO"},  /* Maybe IR LED enable? */
    {0, 10, "P8  (P0.10) - GPIO"},
    {0,  9, "P9  (P0.09) - NFC1"},
    {0, 12, "P12 (P0.12) - GPIO"},
};
#define NUM_TEST_PINS ARRAY_SIZE(test_pins)

/* ============================================================================
 * ADC State
 * ============================================================================ */

static const struct device *adc_dev;

/* ADC channel configurations */
static struct adc_channel_cfg left_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = IR_LEFT_CHANNEL,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0,  /* AIN0 */
};

static struct adc_channel_cfg right_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = IR_RIGHT_CHANNEL,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput1,  /* AIN1 */
};

static struct adc_channel_cfg extra_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = IR_EXTRA_CHANNEL,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput2,  /* AIN2 */
};

/* ADC sequence for reading both channels */
static int16_t adc_buffer[2];
static struct adc_sequence sequence = {
    .buffer = adc_buffer,
    .buffer_size = sizeof(adc_buffer),
    .resolution = ADC_RESOLUTION,
    .oversampling = ADC_OVERSAMPLING,  /* 16x hardware oversampling */
};

/* ============================================================================
 * Sensor State
 * ============================================================================ */

static struct ir_calibration calibration = {
    .left_min = DEFAULT_MIN,
    .left_max = DEFAULT_MAX,
    .right_min = DEFAULT_MIN,
    .right_max = DEFAULT_MAX,
    .threshold_pct = DEFAULT_THRESHOLD,
};

static volatile bool calibrating = false;
static volatile bool debug_enabled = true;  /* Enable for ir_plot.py visualization */

/* Current readings */
static volatile uint16_t current_left_raw = 0;
static volatile uint16_t current_right_raw = 0;
static volatile bool current_left_detected = false;
static volatile bool current_right_detected = false;

/* Low-pass filter state (Q8 fixed-point for precision) */
static int32_t lp_left_state = 0;   /* Filtered left value * 256 */
static int32_t lp_right_state = 0;  /* Filtered right value * 256 */
static bool lp_initialized = false;

/* Bias calibration (idle level at startup) */
static int16_t bias_left = 0;
static int16_t bias_right = 0;
static bool bias_calibrated = false;

/* Calibration tracking */
static uint16_t cal_left_min, cal_left_max;
static uint16_t cal_right_min, cal_right_max;

/* Kalman filter state for distance estimation */
static float kalman_left_d = 550.0f;   /* Estimated left distance (mm) */
static float kalman_right_d = 550.0f;  /* Estimated right distance (mm) */
static float kalman_left_p = KALMAN_INITIAL_P;   /* Left estimation error covariance */
static float kalman_right_p = KALMAN_INITIAL_P;  /* Right estimation error covariance */
static bool kalman_initialized = false;

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * Read a single ADC channel
 */
static int read_adc_channel(uint8_t channel, int16_t *value)
{
    int ret;

    sequence.channels = BIT(channel);

    ret = adc_read(adc_dev, &sequence);
    if (ret < 0) {
        return ret;
    }

    /* Convert to positive value (SAADC can return negative) */
    *value = adc_buffer[0];
    if (*value < 0) {
        *value = 0;
    }

    return 0;
}

/**
 * Convert raw IR value to distance in mm using calibration table
 * Uses linear interpolation between calibration points
 */
static uint16_t raw_to_mm(uint16_t raw)
{
    /* Below minimum raw = far away */
    if (raw <= ir_cal_table[0].raw) {
        return ir_cal_table[0].mm;  /* 550mm max */
    }

    /* Above maximum raw = very close */
    if (raw >= ir_cal_table[IR_CAL_TABLE_SIZE - 1].raw) {
        return ir_cal_table[IR_CAL_TABLE_SIZE - 1].mm;  /* 100mm min */
    }

    /* Find bracketing points and interpolate */
    for (int i = 0; i < IR_CAL_TABLE_SIZE - 1; i++) {
        if (raw >= ir_cal_table[i].raw && raw < ir_cal_table[i + 1].raw) {
            /* Linear interpolation */
            uint16_t raw_lo = ir_cal_table[i].raw;
            uint16_t raw_hi = ir_cal_table[i + 1].raw;
            uint16_t mm_lo = ir_cal_table[i].mm;
            uint16_t mm_hi = ir_cal_table[i + 1].mm;

            /* mm decreases as raw increases */
            int32_t mm = mm_lo - ((int32_t)(raw - raw_lo) * (mm_lo - mm_hi)) / (raw_hi - raw_lo);
            return (uint16_t)mm;
        }
    }

    return ir_cal_table[IR_CAL_TABLE_SIZE - 1].mm;
}

/**
 * 1D Kalman filter update for distance estimation
 * @param estimate Current estimated distance (mm)
 * @param p Current estimation error covariance
 * @param measurement New measured distance (mm)
 * @return Updated estimated distance
 */
static float kalman_update_distance(float *estimate, float *p, float measurement)
{
    /* Predict step (constant position model) */
    float d_pred = *estimate;
    float p_pred = *p + KALMAN_Q_DISTANCE;

    /* Update step */
    float k = p_pred / (p_pred + KALMAN_R_DISTANCE);  /* Kalman gain */
    *estimate = d_pred + k * (measurement - d_pred);
    *p = (1.0f - k) * p_pred;

    /* Clamp to valid range */
    if (*estimate < 50.0f) *estimate = 50.0f;
    if (*estimate > 600.0f) *estimate = 600.0f;

    return *estimate;
}

/**
 * Check if value exceeds threshold
 */
static bool check_threshold(uint16_t value, uint16_t min, uint16_t max, uint16_t threshold_pct)
{
    if (max <= min) {
        return false;  /* Invalid calibration */
    }

    /* Calculate threshold value */
    uint32_t range = max - min;
    uint32_t threshold_value = min + (range * threshold_pct) / 100;

    return value > threshold_value;
}

/* ============================================================================
 * IR Sensor Thread
 * ============================================================================ */

static void ir_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int16_t p0_off, p1_off, p2_off;
    int16_t p0_on, p1_on, p2_on;
    struct obstacle_info info;
    static bool prev_left = false, prev_right = false;

    ARG_UNUSED(p2_off);
    ARG_UNUSED(p2_on);

    printk("IR sensor thread started (P12=LED enable, P0/P1=sensors)\n");

    /* P12 is the IR LED enable pin - active-HIGH */
    #define IR_LED_PIN 12

    /*
     * Startup bias calibration: measure idle level with no obstacles
     * This runs once at boot before the main loop
     */
    if (!bias_calibrated) {
        int32_t sum_left = 0, sum_right = 0;
        int valid_samples = 0;

        printk("IR: Calibrating bias (keep sensors clear)...\n");

        for (int i = 0; i < BIAS_CAL_SAMPLES; i++) {
            /* Read with IR LEDs off */
            gpio_pin_set(gpio0_dev, IR_LED_PIN, 0);
            k_usleep(100);
            read_adc_channel(IR_LEFT_CHANNEL, &p0_off);
            read_adc_channel(IR_RIGHT_CHANNEL, &p1_off);

            /* Read with IR LEDs on */
            gpio_pin_set(gpio0_dev, IR_LED_PIN, 1);
            k_usleep(200);
            read_adc_channel(IR_LEFT_CHANNEL, &p0_on);
            read_adc_channel(IR_RIGHT_CHANNEL, &p1_on);

            gpio_pin_set(gpio0_dev, IR_LED_PIN, 0);

            int16_t diff_left = p0_on - p0_off;
            int16_t diff_right = p1_on - p1_off;

            if (diff_left >= 0 && diff_right >= 0) {
                sum_left += diff_left;
                sum_right += diff_right;
                valid_samples++;
            }

            k_msleep(BIAS_CAL_DELAY_MS);
        }

        if (valid_samples > 0) {
            bias_left = sum_left / valid_samples;
            bias_right = sum_right / valid_samples;
        }

        bias_calibrated = true;
        printk("IR: Bias calibration done: L=%d R=%d\n", bias_left, bias_right);

        /* Note: Proximity beeping disabled - display blink used instead (see main.c) */
    }

    while (1) {
        k_msleep(IR_SAMPLE_INTERVAL_MS);

        /* Step 1: Read ADC with IR LEDs OFF (P12=LOW) */
        gpio_pin_set(gpio0_dev, IR_LED_PIN, 0);  /* OFF = LOW */
        k_usleep(100);  /* Settle time */

        read_adc_channel(IR_LEFT_CHANNEL, &p0_off);
        read_adc_channel(IR_RIGHT_CHANNEL, &p1_off);
        read_adc_channel(IR_EXTRA_CHANNEL, &p2_off);

        /* Step 2: Turn on IR LEDs (P12=HIGH) */
        gpio_pin_set(gpio0_dev, IR_LED_PIN, 1);  /* ON = HIGH */
        k_usleep(200);  /* LED warm-up time */

        read_adc_channel(IR_LEFT_CHANNEL, &p0_on);
        read_adc_channel(IR_RIGHT_CHANNEL, &p1_on);
        read_adc_channel(IR_EXTRA_CHANNEL, &p2_on);

        /* Turn off to save power (P12=LOW) */
        gpio_pin_set(gpio0_dev, IR_LED_PIN, 0);

        /* Calculate reflected IR (difference = ON - ambient) */
        int16_t ir_left_raw = p0_on - p0_off;
        int16_t ir_right_raw = p1_on - p1_off;

        /* Clamp to positive values */
        if (ir_left_raw < 0) ir_left_raw = 0;
        if (ir_right_raw < 0) ir_right_raw = 0;

        /*
         * Apply low-pass filter (10Hz cutoff, IIR 1st order)
         * y[n] = alpha * x[n] + (1-alpha) * y[n-1]
         * Using Q8 fixed-point for precision
         */
        if (!lp_initialized) {
            /* Initialize filter state with first reading */
            lp_left_state = ir_left_raw << 8;
            lp_right_state = ir_right_raw << 8;
            lp_initialized = true;
        } else {
            lp_left_state = (LP_FILTER_ALPHA * ir_left_raw +
                            LP_FILTER_ONE_MINUS * (lp_left_state >> 8));
            lp_right_state = (LP_FILTER_ALPHA * ir_right_raw +
                             LP_FILTER_ONE_MINUS * (lp_right_state >> 8));
        }

        /* Extract filtered values (Q8 → integer) */
        int16_t ir_left_filt = lp_left_state >> 8;
        int16_t ir_right_filt = lp_right_state >> 8;

        /* Subtract bias (idle level calibrated at startup) */
        int16_t ir_left = ir_left_filt - bias_left;
        int16_t ir_right = ir_right_filt - bias_right;
        if (ir_left < 0) ir_left = 0;
        if (ir_right < 0) ir_right = 0;

        /* Store bias-corrected values */
        current_left_raw = (uint16_t)ir_left;
        current_right_raw = (uint16_t)ir_right;

        /*
         * Convert raw IR to distance in mm
         * Then apply Kalman filter for smoothing
         */
        float left_mm_raw = (float)raw_to_mm((uint16_t)ir_left);
        float right_mm_raw = (float)raw_to_mm((uint16_t)ir_right);

        if (!kalman_initialized) {
            kalman_left_d = left_mm_raw;
            kalman_right_d = right_mm_raw;
            kalman_left_p = KALMAN_INITIAL_P;
            kalman_right_p = KALMAN_INITIAL_P;
            kalman_initialized = true;
        }

        /* Apply Kalman filter */
        float left_mm_filt = kalman_update_distance(&kalman_left_d, &kalman_left_p, left_mm_raw);
        float right_mm_filt = kalman_update_distance(&kalman_right_d, &kalman_right_p, right_mm_raw);

        /* Debug output for plotting: IR,<ts>,<left_mm>,<right_mm>,<left_raw>,<right_raw>,<left_mm_raw>,<right_mm_raw> */
        if (debug_enabled) {
            int64_t ts = k_uptime_get();
            printk("IR,%lld,%.0f,%.0f,%d,%d,%.0f,%.0f\n",
                   ts,
                   (double)left_mm_filt, (double)right_mm_filt,
                   ir_left, ir_right,
                   (double)left_mm_raw, (double)right_mm_raw);
        }

        /* Manual calibration mode: track min/max using RAW values */
        if (calibrating) {
            if (ir_left_raw < cal_left_min) cal_left_min = ir_left_raw;
            if (ir_left_raw > cal_left_max) cal_left_max = ir_left_raw;
            if (ir_right_raw < cal_right_min) cal_right_min = ir_right_raw;
            if (ir_right_raw > cal_right_max) cal_right_max = ir_right_raw;
            continue;  /* Skip normal processing during calibration */
        }

        /* Check obstacle detection using IR difference values */
        current_left_detected = check_threshold(current_left_raw,
            calibration.left_min, calibration.left_max, calibration.threshold_pct);
        current_right_detected = check_threshold(current_right_raw,
            calibration.right_min, calibration.right_max, calibration.threshold_pct);

        /* Send to message queue on state change */
        if (current_left_detected != prev_left || current_right_detected != prev_right) {
            info.left_detected = current_left_detected;
            info.right_detected = current_right_detected;
            info.left_value = current_left_raw;
            info.right_value = current_right_raw;

            k_msgq_put(&obstacle_q, &info, K_NO_WAIT);

            /* Note: Obstacle sound disabled - display blink used instead */

            prev_left = current_left_detected;
            prev_right = current_right_detected;
        }
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int ir_sensors_init(void)
{
    int ret;

    /* Get ADC device */
    adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("IR: ADC device not ready\n");
        return -ENODEV;
    }

    /* Configure left channel */
    ret = adc_channel_setup(adc_dev, &left_channel_cfg);
    if (ret < 0) {
        printk("IR: Failed to configure left channel: %d\n", ret);
        return ret;
    }

    /* Configure right channel */
    ret = adc_channel_setup(adc_dev, &right_channel_cfg);
    if (ret < 0) {
        printk("IR: Failed to configure right channel: %d\n", ret);
        return ret;
    }

    /* Configure extra channel (P2) */
    ret = adc_channel_setup(adc_dev, &extra_channel_cfg);
    if (ret < 0) {
        printk("IR: Failed to configure extra channel: %d\n", ret);
        return ret;
    }

    printk("IR sensors initialized (P0=AIN%d, P1=AIN%d, P2=AIN%d)\n",
           IR_LEFT_CHANNEL, IR_RIGHT_CHANNEL, IR_EXTRA_CHANNEL);

    /* Setup GPIO for IR LED enable pins (P8, P12) */
    gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0_dev)) {
        printk("IR: GPIO0 device not ready\n");
        return -ENODEV;
    }

    /* Configure P8 and P12 as outputs for IR LED testing */
    for (int i = 0; i < NUM_IR_LED_PINS; i++) {
        ret = gpio_pin_configure(gpio0_dev, ir_led_pins[i], GPIO_OUTPUT_LOW);
        if (ret < 0) {
            printk("IR: Failed to configure P0.%d as output: %d\n", ir_led_pins[i], ret);
        } else {
            printk("IR: Configured P0.%d (P%d) as IR LED enable\n",
                   ir_led_pins[i], ir_led_pins[i] == 10 ? 8 : 12);
        }
    }

    return 0;
}

void ir_sensors_start_thread(void)
{
    ir_thread_id = k_thread_create(&ir_thread_data, ir_thread_stack,
                                   K_THREAD_STACK_SIZEOF(ir_thread_stack),
                                   ir_thread_fn, NULL, NULL, NULL,
                                   IR_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(ir_thread_id, "ir_sensors");
}

void ir_sensors_get_raw(uint16_t *left_value, uint16_t *right_value)
{
    if (left_value) {
        *left_value = current_left_raw;
    }
    if (right_value) {
        *right_value = current_right_raw;
    }
}

void ir_sensors_get_obstacle(bool *left_detected, bool *right_detected)
{
    if (left_detected) {
        *left_detected = current_left_detected;
    }
    if (right_detected) {
        *right_detected = current_right_detected;
    }
}

void ir_sensors_start_calibration(void)
{
    /* Initialize calibration tracking with extreme values */
    cal_left_min = 4095;
    cal_left_max = 0;
    cal_right_min = 4095;
    cal_right_max = 0;

    calibrating = true;

    printk("IR calibration started - move obstacles close and away from sensors\n");
}

void ir_sensors_stop_calibration(void)
{
    calibrating = false;

    /* Apply captured values (with some margin) */
    if (cal_left_max > cal_left_min + 100) {
        calibration.left_min = cal_left_min;
        calibration.left_max = cal_left_max;
    }
    if (cal_right_max > cal_right_min + 100) {
        calibration.right_min = cal_right_min;
        calibration.right_max = cal_right_max;
    }

    printk("IR calibration complete:\n");
    printk("  Left:  min=%d, max=%d\n", calibration.left_min, calibration.left_max);
    printk("  Right: min=%d, max=%d\n", calibration.right_min, calibration.right_max);
}

bool ir_sensors_is_calibrating(void)
{
    return calibrating;
}

void ir_sensors_set_threshold(uint16_t threshold_pct)
{
    if (threshold_pct > 100) {
        threshold_pct = 100;
    }
    calibration.threshold_pct = threshold_pct;
    printk("IR threshold set to %d%%\n", threshold_pct);
}

void ir_sensors_get_calibration(struct ir_calibration *cal)
{
    if (cal) {
        *cal = calibration;
    }
}

void ir_sensors_set_debug(bool enabled)
{
    debug_enabled = enabled;
    printk("IR debug %s\n", enabled ? "enabled" : "disabled");
}

void ir_sensors_get_distance(float *left_mm, float *right_mm)
{
    if (left_mm) {
        *left_mm = kalman_left_d;
    }
    if (right_mm) {
        *right_mm = kalman_right_d;
    }
}

/* ============================================================================
 * GPIO Test Mode
 * ============================================================================ */

static const struct device *get_gpio_dev(uint8_t port)
{
    return (port == 0) ? gpio0_dev : gpio1_dev;
}

void ir_sensors_gpio_test_init(void)
{
    int ret;
    int16_t adc_off, adc_on;
    int16_t best_diff = 0;
    int best_pin = -1;

    /* Ensure GPIO devices are ready */
    if (!gpio0_dev || !device_is_ready(gpio0_dev)) {
        gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        if (!device_is_ready(gpio0_dev)) {
            printk("ERROR: GPIO0 not ready!\n");
            return;
        }
    }

    gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    if (!device_is_ready(gpio1_dev)) {
        printk("WARNING: GPIO1 not ready\n");
    }

    printk("\n========== AUTO GPIO SCAN ==========\n");
    printk("Scanning %d pins for IR LED enable...\n", NUM_TEST_PINS);
    printk("Place an obstacle ~5cm from sensors!\n");
    printk("=====================================\n\n");

    /* Configure all pins as outputs LOW first */
    for (int i = 0; i < NUM_TEST_PINS; i++) {
        const struct device *dev = get_gpio_dev(test_pins[i].port);
        if (dev && device_is_ready(dev)) {
            ret = gpio_pin_configure(dev, test_pins[i].pin, GPIO_OUTPUT_LOW);
            if (ret < 0) {
                printk("  Config FAIL: %s (err %d)\n", test_pins[i].name, ret);
            }
        }
    }

    k_msleep(100);

    /* Test each pin: measure ADC with pin LOW, then HIGH */
    for (int i = 0; i < NUM_TEST_PINS; i++) {
        const struct device *dev = get_gpio_dev(test_pins[i].port);
        if (!dev || !device_is_ready(dev)) {
            printk("[%2d] %s: SKIP (no device)\n", i + 1, test_pins[i].name);
            continue;
        }

        /* Ensure pin is LOW, read ADC */
        gpio_pin_set(dev, test_pins[i].pin, 0);
        k_msleep(20);
        read_adc_channel(IR_LEFT_CHANNEL, &adc_off);

        /* Set pin HIGH, read ADC */
        gpio_pin_set(dev, test_pins[i].pin, 1);
        k_msleep(20);
        read_adc_channel(IR_LEFT_CHANNEL, &adc_on);

        /* Set pin LOW again */
        gpio_pin_set(dev, test_pins[i].pin, 0);

        int16_t diff = adc_on - adc_off;

        printk("[%2d] %s: OFF=%4d ON=%4d DIFF=%+5d%s\n",
               i + 1, test_pins[i].name,
               adc_off, adc_on, diff,
               (diff > 50) ? " <-- CANDIDATE!" : "");

        if (diff > best_diff) {
            best_diff = diff;
            best_pin = i;
        }
    }

    printk("\n========== SCAN COMPLETE ==========\n");
    if (best_pin >= 0 && best_diff > 50) {
        printk("BEST MATCH: %s (diff=%d)\n", test_pins[best_pin].name, best_diff);
        /* Keep this pin HIGH */
        const struct device *dev = get_gpio_dev(test_pins[best_pin].port);
        if (dev) {
            gpio_pin_set(dev, test_pins[best_pin].pin, 1);
        }
    } else {
        printk("No clear IR LED enable pin found (best diff=%d).\n", best_diff);
        printk("Try: 1) Move obstacle closer\n");
        printk("     2) IR LEDs might be active-LOW\n");
        printk("     3) IR LEDs might be always on\n");
    }
    printk("====================================\n\n");

    gpio_test_mode = true;
    gpio_test_index = -1;
}

void ir_sensors_gpio_test_next(void)
{
    if (!gpio_test_mode) {
        return;
    }

    /* Determine active/inactive states based on mode */
    int active_state = gpio_test_active_low ? 0 : 1;
    int inactive_state = gpio_test_active_low ? 1 : 0;

    /* Turn off previous pin (set to inactive state) */
    if (gpio_test_index >= 0 && gpio_test_index < NUM_TEST_PINS) {
        const struct gpio_test_pin *prev = &test_pins[gpio_test_index];
        const struct device *dev = get_gpio_dev(prev->port);
        if (dev) {
            gpio_pin_set(dev, prev->pin, inactive_state);
        }
    }

    /* Move to next pin */
    gpio_test_index++;
    if (gpio_test_index >= NUM_TEST_PINS) {
        gpio_test_index = 0;
    }

    /* Turn on current pin (set to active state) */
    const struct gpio_test_pin *curr = &test_pins[gpio_test_index];
    const struct device *dev = get_gpio_dev(curr->port);
    if (dev) {
        gpio_pin_set(dev, curr->pin, active_state);
        printk(">>> [%2d/%d] %s = %s <<<\n",
               gpio_test_index + 1, NUM_TEST_PINS, curr->name,
               gpio_test_active_low ? "LOW" : "HIGH");
    }
}

void ir_sensors_gpio_test_stop(void)
{
    /* Turn off all pins */
    for (int i = 0; i < NUM_TEST_PINS; i++) {
        const struct device *dev = get_gpio_dev(test_pins[i].port);
        if (dev) {
            gpio_pin_set(dev, test_pins[i].pin, 0);
        }
    }
    gpio_test_mode = false;
    gpio_test_index = -1;
    printk("GPIO test mode stopped\n");
}

bool ir_sensors_gpio_test_active(void)
{
    return gpio_test_mode;
}
