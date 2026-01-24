# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(imu_orientation)

target_sources(app PRIVATE
    # Core
    src/main.c
    src/robot_state.c

    # IMU/Sensors
    src/sensors.c
    src/orientation.c
    src/mahony_filter.c

    # Communication
    src/serial_output.c
    src/smp_bt.c
    src/ble_output.c

    # Xbox Controller
    src/ble_central.c
    src/hid_parser.c

    # Actuators
    src/motor_driver.c
    src/audio.c

    # Testing
    src/motor_test.c
)
