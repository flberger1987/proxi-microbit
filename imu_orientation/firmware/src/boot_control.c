/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Boot Control Block implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>
#include <string.h>

#include "boot_control.h"

/* Flash device */
#define FLASH_DEVICE DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller))

/* Page size for nRF52833 */
#define FLASH_PAGE_SIZE 4096

/* Local copy of boot control block */
static struct boot_control boot_ctrl;
static bool boot_ctrl_initialized = false;

/* Calculate CRC32 of boot control block (excluding checksum field) */
static uint32_t calc_boot_ctrl_crc(const struct boot_control *ctrl)
{
    /* CRC everything except the last uint32_t (checksum field) */
    size_t crc_len = sizeof(struct boot_control) - sizeof(uint32_t);
    return crc32_ieee((const uint8_t *)ctrl, crc_len);
}

/* Read boot control block from flash */
static int read_boot_ctrl(struct boot_control *ctrl)
{
    const struct device *flash_dev = FLASH_DEVICE;

    if (!device_is_ready(flash_dev)) {
        printk("BOOT: Flash device not ready\n");
        return -ENODEV;
    }

    int ret = flash_read(flash_dev, BOOT_CTRL_ADDR, ctrl, sizeof(*ctrl));
    if (ret != 0) {
        printk("BOOT: Flash read failed: %d\n", ret);
        return ret;
    }

    return 0;
}

/* Write boot control block to flash */
static int write_boot_ctrl(const struct boot_control *ctrl)
{
    const struct device *flash_dev = FLASH_DEVICE;

    if (!device_is_ready(flash_dev)) {
        printk("BOOT: Flash device not ready\n");
        return -ENODEV;
    }

    /* Erase the page first */
    int ret = flash_erase(flash_dev, BOOT_CTRL_ADDR, FLASH_PAGE_SIZE);
    if (ret != 0) {
        printk("BOOT: Flash erase failed: %d\n", ret);
        return ret;
    }

    /* Write new data */
    ret = flash_write(flash_dev, BOOT_CTRL_ADDR, ctrl, sizeof(*ctrl));
    if (ret != 0) {
        printk("BOOT: Flash write failed: %d\n", ret);
        return ret;
    }

    return 0;
}

/* Validate boot control block */
static bool validate_boot_ctrl(const struct boot_control *ctrl)
{
    /* Check magic */
    if (ctrl->magic != BOOT_MAGIC) {
        return false;
    }

    /* Check version - accept both v1 and v2 for backwards compat */
    if (ctrl->version != BOOT_VERSION && ctrl->version != 1) {
        return false;
    }

    /* Check slot value */
    if (ctrl->active_slot > SLOT_B) {
        return false;
    }

    /* Verify checksum */
    uint32_t expected_crc = calc_boot_ctrl_crc(ctrl);
    if (ctrl->checksum != expected_crc) {
        return false;
    }

    return true;
}

/* Initialize with default values */
static void init_default_boot_ctrl(struct boot_control *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->magic = BOOT_MAGIC;
    ctrl->version = BOOT_VERSION;
    ctrl->active_slot = SLOT_A;
    ctrl->slot_a_valid = 1;  /* Assume current firmware is in Slot A */
    ctrl->slot_b_valid = 0;
    ctrl->boot_count = 0;
    ctrl->fallback_reason = BOOT_FALLBACK_NONE;
    ctrl->last_boot_slot = SLOT_A;
    ctrl->slot_a_crc = 0;
    ctrl->slot_b_crc = 0;
    ctrl->slot_a_size = 0;
    ctrl->slot_b_size = 0;
    ctrl->slot_a_version = 0;
    ctrl->slot_b_version = 0;
    ctrl->checksum = calc_boot_ctrl_crc(ctrl);
}

int boot_control_init(void)
{
    if (boot_ctrl_initialized) {
        return 0;
    }

    int ret = read_boot_ctrl(&boot_ctrl);
    if (ret != 0) {
        printk("BOOT: Failed to read boot control, using defaults\n");
        init_default_boot_ctrl(&boot_ctrl);
        ret = write_boot_ctrl(&boot_ctrl);
        if (ret != 0) {
            return ret;
        }
    }

    if (!validate_boot_ctrl(&boot_ctrl)) {
        printk("BOOT: Invalid boot control, initializing defaults\n");
        init_default_boot_ctrl(&boot_ctrl);
        ret = write_boot_ctrl(&boot_ctrl);
        if (ret != 0) {
            return ret;
        }
    }

    boot_ctrl_initialized = true;
    printk("BOOT: Initialized - Active slot: %c, Boot count: %d\n",
           boot_ctrl.active_slot == SLOT_A ? 'A' : 'B',
           boot_ctrl.boot_count);

    return 0;
}

uint8_t boot_control_get_active_slot(void)
{
    return boot_ctrl.active_slot;
}

uint8_t boot_control_get_inactive_slot(void)
{
    return boot_ctrl.active_slot == SLOT_A ? SLOT_B : SLOT_A;
}

uint32_t boot_control_get_slot_addr(uint8_t slot)
{
    return slot == SLOT_A ? SLOT_A_ADDR : SLOT_B_ADDR;
}

uint32_t boot_control_get_slot_size(uint8_t slot)
{
    (void)slot;
    return SLOT_A_SIZE;  /* Both slots are same size */
}

bool boot_control_is_slot_valid(uint8_t slot)
{
    if (slot == SLOT_A) {
        return boot_ctrl.slot_a_valid != 0;
    } else {
        return boot_ctrl.slot_b_valid != 0;
    }
}

uint32_t boot_control_get_firmware_size(uint8_t slot)
{
    if (slot == SLOT_A) {
        return boot_ctrl.slot_a_size;
    } else {
        return boot_ctrl.slot_b_size;
    }
}

uint32_t boot_control_get_firmware_crc(uint8_t slot)
{
    if (slot == SLOT_A) {
        return boot_ctrl.slot_a_crc;
    } else {
        return boot_ctrl.slot_b_crc;
    }
}

int boot_control_mark_slot_valid(uint8_t slot, uint32_t size, uint32_t crc32, uint32_t build_num)
{
    if (slot == SLOT_A) {
        boot_ctrl.slot_a_valid = 1;
        boot_ctrl.slot_a_size = size;
        boot_ctrl.slot_a_crc = crc32;
        boot_ctrl.slot_a_version = build_num;
    } else {
        boot_ctrl.slot_b_valid = 1;
        boot_ctrl.slot_b_size = size;
        boot_ctrl.slot_b_crc = crc32;
        boot_ctrl.slot_b_version = build_num;
    }

    boot_ctrl.checksum = calc_boot_ctrl_crc(&boot_ctrl);
    return write_boot_ctrl(&boot_ctrl);
}

int boot_control_mark_slot_invalid(uint8_t slot)
{
    if (slot == SLOT_A) {
        boot_ctrl.slot_a_valid = 0;
        boot_ctrl.slot_a_size = 0;
        boot_ctrl.slot_a_crc = 0;
        boot_ctrl.slot_a_version = 0;
    } else {
        boot_ctrl.slot_b_valid = 0;
        boot_ctrl.slot_b_size = 0;
        boot_ctrl.slot_b_crc = 0;
        boot_ctrl.slot_b_version = 0;
    }

    boot_ctrl.checksum = calc_boot_ctrl_crc(&boot_ctrl);
    return write_boot_ctrl(&boot_ctrl);
}

int boot_control_switch_slot(uint8_t new_slot)
{
    if (new_slot > SLOT_B) {
        return -EINVAL;
    }

    /* Verify target slot is valid */
    bool valid = (new_slot == SLOT_A) ? boot_ctrl.slot_a_valid : boot_ctrl.slot_b_valid;
    if (!valid) {
        printk("BOOT: Cannot switch to invalid slot %c\n",
               new_slot == SLOT_A ? 'A' : 'B');
        return -EINVAL;
    }

    boot_ctrl.active_slot = new_slot;
    boot_ctrl.boot_count = 0;  /* Reset for new slot */
    boot_ctrl.checksum = calc_boot_ctrl_crc(&boot_ctrl);

    printk("BOOT: Switching to slot %c\n", new_slot == SLOT_A ? 'A' : 'B');
    return write_boot_ctrl(&boot_ctrl);
}

int boot_control_confirm_boot(void)
{
    if (boot_ctrl.boot_count == 0) {
        /* Already confirmed */
        return 0;
    }

    boot_ctrl.boot_count = 0;
    boot_ctrl.checksum = calc_boot_ctrl_crc(&boot_ctrl);

    printk("BOOT: Boot confirmed for slot %c\n",
           boot_ctrl.active_slot == SLOT_A ? 'A' : 'B');
    return write_boot_ctrl(&boot_ctrl);
}

uint8_t boot_control_get_boot_count(void)
{
    return boot_ctrl.boot_count;
}

uint8_t boot_control_get_fallback_reason(void)
{
    return boot_ctrl.fallback_reason;
}

uint32_t boot_control_get_firmware_version(uint8_t slot)
{
    if (slot == SLOT_A) {
        return boot_ctrl.slot_a_version;
    } else {
        return boot_ctrl.slot_b_version;
    }
}

void boot_control_print_info(void)
{
    printk("=== Boot Control Block ===\n");
    printk("  Magic: 0x%08X (expected 0x%08X)\n", boot_ctrl.magic, BOOT_MAGIC);
    printk("  Version: %d\n", boot_ctrl.version);
    printk("  Active Slot: %c\n", boot_ctrl.active_slot == SLOT_A ? 'A' : 'B');
    printk("  Slot A: %s, size=%u, crc=0x%08X, ver=%u\n",
           boot_ctrl.slot_a_valid ? "VALID" : "INVALID",
           boot_ctrl.slot_a_size, boot_ctrl.slot_a_crc, boot_ctrl.slot_a_version);
    printk("  Slot B: %s, size=%u, crc=0x%08X, ver=%u\n",
           boot_ctrl.slot_b_valid ? "VALID" : "INVALID",
           boot_ctrl.slot_b_size, boot_ctrl.slot_b_crc, boot_ctrl.slot_b_version);
    printk("  Boot Count: %d\n", boot_ctrl.boot_count);
    printk("  Fallback Reason: %d\n", boot_ctrl.fallback_reason);
    printk("  Last Boot Slot: %c\n", boot_ctrl.last_boot_slot == SLOT_A ? 'A' : 'B');
    printk("  Checksum: 0x%08X\n", boot_ctrl.checksum);
}
