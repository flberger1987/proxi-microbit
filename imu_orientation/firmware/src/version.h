/*
 * SPDX-License-Identifier: Apache-2.0
 * Firmware version definition
 */

#ifndef VERSION_H
#define VERSION_H

/* Build number (set by CMake from build_flash.sh) */
#ifndef BUILD_NUMBER
#define BUILD_NUMBER 0
#endif

/* Stringify macros */
#define _STR_HELPER(x) #x
#define _STR(x) _STR_HELPER(x)

/* Firmware version string */
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 0
#define FW_VERSION_SUFFIX "ble"

#define FW_VERSION _STR(FW_VERSION_MAJOR) "." _STR(FW_VERSION_MINOR) "." \
                   _STR(FW_VERSION_PATCH) "-" FW_VERSION_SUFFIX "." _STR(BUILD_NUMBER)

/* Embedded version marker for OTA tool detection.
 * Uses a unique prefix "FWVER=" that won't appear elsewhere in the binary. */
#define FW_VERSION_MARKER "FWVER=" FW_VERSION

#endif /* VERSION_H */
