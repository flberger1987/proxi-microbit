/*
 * SPDX-License-Identifier: Apache-2.0
 * BLE Telemetry Interface for Robot Dashboard
 *
 * Sends packed binary telemetry data at ~20Hz over BLE NUS.
 * Connect dashboard BEFORE Xbox controller for concurrent operation.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Telemetry Packet Structure (36 bytes)
 * ============================================================================
 *
 * Wire format: Little-endian (ARM native)
 * Update rate: ~20 Hz (50ms interval)
 *
 * Python struct format: '<BBIhhHhHHbbBBHhhhhhh'
 */

#define TELEMETRY_MAGIC         0xAB
#define TELEMETRY_VERSION       0x01

/* Thread stats packet magic */
#define TELEMETRY_THREAD_MAGIC  0xAC
#define TELEMETRY_THREAD_VER    0x01

#pragma pack(push, 1)
struct telemetry_packet {
    /* Header (2 bytes) */
    uint8_t  magic;              /* 0xAB - Sync byte for packet detection */
    uint8_t  version;            /* 0x01 - Protocol version */

    /* Timestamp (4 bytes) */
    uint32_t timestamp_ms;       /* Uptime in milliseconds */

    /* Orientation (6 bytes) - scaled integers for efficiency */
    int16_t  roll_x10;           /* Roll * 10 (-1800 to +1800 for -180 to +180 deg) */
    int16_t  pitch_x10;          /* Pitch * 10 (-900 to +900 for -90 to +90 deg) */
    uint16_t heading_x10;        /* Heading * 10 (0 to 3599 for 0 to 359.9 deg) */

    /* Sensor values (6 bytes) */
    int16_t  yaw_rate_x10;       /* Yaw rate * 10 (deg/s, from Kalman filter) */
    uint16_t ir_left_mm;         /* IR left distance in mm (Kalman filtered) */
    uint16_t ir_right_mm;        /* IR right distance in mm (Kalman filtered) */

    /* Motor & Status (4 bytes) */
    int8_t   motor_linear;       /* Linear velocity command (-100 to +100) */
    int8_t   motor_angular;      /* Angular velocity command (-100 to +100) */
    uint8_t  nav_state;          /* enum autonav_state (0=DISABLED, 1=HEADING_HOLD, 2=TURNING, 3=SCANNING) */
    uint8_t  flags;              /* Bit 0: autonav_enabled, Bit 1: motors_enabled */

    /* Navigation (2 bytes) */
    uint16_t target_heading_x10; /* Target heading * 10 (0 to 3599) */

    /* Raw sensor data (12 bytes) - for debugging/visualization */
    int16_t  raw_ax;             /* Accelerometer X (milli-g, User coords) */
    int16_t  raw_ay;             /* Accelerometer Y (milli-g, User coords) */
    int16_t  raw_az;             /* Accelerometer Z (milli-g, User coords) */
    int16_t  raw_mx;             /* Magnetometer X (milli-Gauss, User coords) */
    int16_t  raw_my;             /* Magnetometer Y (milli-Gauss, User coords) */
    int16_t  raw_mz;             /* Magnetometer Z (milli-Gauss, User coords) */
};
#pragma pack(pop)

/* Verify struct size at compile time */
_Static_assert(sizeof(struct telemetry_packet) == 36,
               "Telemetry packet must be exactly 36 bytes");

/* Flag bit definitions */
#define TELEMETRY_FLAG_AUTONAV_ENABLED  0x01
#define TELEMETRY_FLAG_MOTORS_ENABLED   0x02

/* ============================================================================
 * Thread Stats Packet Structure (12 bytes)
 * ============================================================================
 *
 * Sent in round-robin: 1 thread per main packet (at 20Hz = ~2 updates/thread/s)
 * Python struct format: '<BBBBIHH'
 */

/* Thread ID enum - matches firmware thread creation order */
enum thread_id {
    THREAD_ID_MAIN = 0,
    THREAD_ID_MOTOR,
    THREAD_ID_SENSOR,
    THREAD_ID_BLE_CTRL,     /* ble_central.c sets name "ble_ctrl" */
    THREAD_ID_TELEMETRY,
    THREAD_ID_AUDIO,
    THREAD_ID_IR_SENSORS,   /* ir_sensors.c sets name "ir_sensors" */
    THREAD_ID_AUTONAV,      /* autonomous_nav.c sets name "autonav" */
    THREAD_ID_IDLE,         /* System idle thread */
    THREAD_ID_COUNT
};

#pragma pack(push, 1)
struct thread_stats_packet {
    /* Header (4 bytes) */
    uint8_t  magic;              /* 0xAC - Thread stats packet */
    uint8_t  version;            /* 0x01 - Protocol version */
    uint8_t  thread_id;          /* enum thread_id */
    uint8_t  thread_count;       /* Total threads being monitored */

    /* Timestamp (4 bytes) */
    uint32_t timestamp_ms;       /* Uptime in milliseconds */

    /* Thread Stats (4 bytes) */
    uint16_t cpu_permille;       /* CPU usage * 10 (0-1000 = 0.0% - 100.0%) */
    uint16_t stack_used;         /* Stack used in bytes */
};
#pragma pack(pop)

/* Verify struct size at compile time */
_Static_assert(sizeof(struct thread_stats_packet) == 12,
               "Thread stats packet must be exactly 12 bytes");

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * Initialize telemetry module
 *
 * @return 0 on success, negative errno on failure
 */
int telemetry_init(void);

/**
 * Start the telemetry thread (~20Hz)
 * Sends binary packets over BLE NUS when connected
 */
void telemetry_start_thread(void);

/**
 * Enable/disable telemetry streaming
 *
 * @param enable true to enable streaming, false to disable
 */
void telemetry_enable(bool enable);

/**
 * Check if telemetry streaming is enabled
 *
 * @return true if streaming is enabled
 */
bool telemetry_is_enabled(void);

#endif /* TELEMETRY_H */
