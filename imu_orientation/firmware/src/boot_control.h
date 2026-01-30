/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Boot Control Block - Manages dual-slot firmware with boot pointer
 *
 * Optimized Flash Layout (nRF52833 512KB):
 *   0x00000 - 0x02000: Mini-Bootloader (8KB)
 *   0x02000 - 0x3F000: Slot A - Primary firmware (244KB)
 *   0x3F000 - 0x7C000: Slot B - Secondary firmware (244KB)
 *   0x7C000 - 0x7D000: Boot Control Block (4KB)
 *   0x7D000 - 0x7E000: OTA State (4KB)
 *   0x7E000 - 0x80000: Settings/NVS (8KB)
 */

#ifndef BOOT_CONTROL_H
#define BOOT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* Flash addresses */
#define BOOTLOADER_ADDR     0x00000000
#define BOOTLOADER_SIZE     0x00002000  /* 8KB */

#define SLOT_A_ADDR         0x00002000
#define SLOT_A_SIZE         0x0003D000  /* 244KB */

#define SLOT_B_ADDR         0x0003F000
#define SLOT_B_SIZE         0x0003D000  /* 244KB */

#define BOOT_CTRL_ADDR      0x0007C000
#define BOOT_CTRL_SIZE      0x00001000  /* 4KB */

#define OTA_STATE_ADDR      0x0007D000
#define OTA_STATE_SIZE      0x00001000  /* 4KB */

#define SETTINGS_ADDR       0x0007E000
#define SETTINGS_SIZE       0x00002000  /* 8KB */

/* Magic values */
#define BOOT_MAGIC          0xB007F1E0  /* "BOOT FILE" in leetspeak */
#define BOOT_VERSION        2  /* Incremented for version field support */

/* Fallback reason codes */
#define BOOT_FALLBACK_NONE      0  /* Normal boot, no fallback */
#define BOOT_FALLBACK_CRC_FAIL  1  /* Primary slot CRC verification failed */
#define BOOT_FALLBACK_VERSION   2  /* Using lower version (other slot invalid) */
#define BOOT_FALLBACK_BOOT_COUNT 3 /* Too many boot attempts */

/* Slot identifiers */
#define SLOT_A              0
#define SLOT_B              1

/**
 * Boot Control Block structure
 * Stored in flash at BOOT_CTRL_ADDR
 */
struct boot_control {
    uint32_t magic;           /* BOOT_MAGIC for validation */
    uint8_t  version;         /* Structure version */
    uint8_t  active_slot;     /* 0 = Slot A, 1 = Slot B */
    uint8_t  slot_a_valid;    /* 1 = Slot A contains valid firmware */
    uint8_t  slot_b_valid;    /* 1 = Slot B contains valid firmware */
    uint8_t  boot_count;      /* Counter for fallback detection */
    uint8_t  fallback_reason; /* Why fallback occurred (BOOT_FALLBACK_*) */
    uint8_t  last_boot_slot;  /* Which slot we actually booted (for diagnostics) */
    uint8_t  reserved;        /* Alignment padding */
    uint32_t slot_a_crc;      /* CRC32 of firmware in Slot A */
    uint32_t slot_b_crc;      /* CRC32 of firmware in Slot B */
    uint32_t slot_a_size;     /* Size of firmware in Slot A (bytes) */
    uint32_t slot_b_size;     /* Size of firmware in Slot B (bytes) */
    uint32_t slot_a_version;  /* Build number of firmware in Slot A */
    uint32_t slot_b_version;  /* Build number of firmware in Slot B */
    uint32_t checksum;        /* CRC32 of this structure (excluding this field) */
};

/**
 * Initialize boot control subsystem
 * Reads boot control block from flash
 *
 * @return 0 on success, negative error code on failure
 */
int boot_control_init(void);

/**
 * Get currently active slot
 *
 * @return SLOT_A or SLOT_B
 */
uint8_t boot_control_get_active_slot(void);

/**
 * Get inactive (update target) slot
 *
 * @return SLOT_A or SLOT_B
 */
uint8_t boot_control_get_inactive_slot(void);

/**
 * Get slot flash address
 *
 * @param slot SLOT_A or SLOT_B
 * @return Flash address of slot start
 */
uint32_t boot_control_get_slot_addr(uint8_t slot);

/**
 * Get slot size
 *
 * @param slot SLOT_A or SLOT_B
 * @return Size of slot in bytes
 */
uint32_t boot_control_get_slot_size(uint8_t slot);

/**
 * Check if slot contains valid firmware
 *
 * @param slot SLOT_A or SLOT_B
 * @return true if slot is valid
 */
bool boot_control_is_slot_valid(uint8_t slot);

/**
 * Get firmware size in slot
 *
 * @param slot SLOT_A or SLOT_B
 * @return Firmware size in bytes, 0 if invalid
 */
uint32_t boot_control_get_firmware_size(uint8_t slot);

/**
 * Get firmware CRC32 in slot
 *
 * @param slot SLOT_A or SLOT_B
 * @return CRC32 of firmware, 0 if invalid
 */
uint32_t boot_control_get_firmware_crc(uint8_t slot);

/**
 * Mark slot as valid after successful OTA
 *
 * @param slot SLOT_A or SLOT_B
 * @param size Firmware size in bytes
 * @param crc32 CRC32 of firmware
 * @param build_num Build number (version) of firmware
 * @return 0 on success, negative error code on failure
 */
int boot_control_mark_slot_valid(uint8_t slot, uint32_t size, uint32_t crc32, uint32_t build_num);

/**
 * Mark slot as invalid (e.g., before erasing)
 *
 * @param slot SLOT_A or SLOT_B
 * @return 0 on success, negative error code on failure
 */
int boot_control_mark_slot_invalid(uint8_t slot);

/**
 * Switch active slot pointer (called after OTA validation)
 * Does NOT reboot - caller should reboot after this
 *
 * @param new_slot SLOT_A or SLOT_B
 * @return 0 on success, negative error code on failure
 */
int boot_control_switch_slot(uint8_t new_slot);

/**
 * Confirm successful boot (resets boot_count)
 * Should be called from main after initialization is complete
 *
 * @return 0 on success, negative error code on failure
 */
int boot_control_confirm_boot(void);

/**
 * Get boot count for diagnostics
 *
 * @return Current boot count
 */
uint8_t boot_control_get_boot_count(void);

/**
 * Get fallback reason from last boot
 *
 * @return BOOT_FALLBACK_* code
 */
uint8_t boot_control_get_fallback_reason(void);

/**
 * Get firmware version (build number) in slot
 *
 * @param slot SLOT_A or SLOT_B
 * @return Build number, 0 if unknown
 */
uint32_t boot_control_get_firmware_version(uint8_t slot);

/**
 * Debug: Print boot control block info
 */
void boot_control_print_info(void);

#endif /* BOOT_CONTROL_H */
