# SPDX-License-Identifier: Apache-2.0
#
# Mini-Bootloader for BLE OTA Dual-Slot System
# Target size: ~8KB (must fit in 0x00000-0x04000)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(mini_bootloader)

target_sources(app PRIVATE
    src/main.c
)

# Include parent src directory for boot_control.h
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
