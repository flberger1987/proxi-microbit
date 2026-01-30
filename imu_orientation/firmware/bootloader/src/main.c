/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Mini-Bootloader for BLE OTA Dual-Slot System
 *
 * This minimal bootloader (~8KB):
 * 1. Reads Boot Control Block from flash
 * 2. Validates magic and checksum
 * 3. Selects active slot (A or B)
 * 4. Implements automatic fallback after 3 failed boots
 * 5. Jumps to selected firmware slot
 *
 * Optimized Flash Layout (nRF52833 512KB):
 *   0x00000 - 0x02000: This bootloader (8KB max)
 *   0x02000 - 0x3F000: Slot A (244KB)
 *   0x3F000 - 0x7C000: Slot B (244KB)
 *   0x7C000 - 0x7D000: Boot Control Block (4KB)
 *   0x7D000 - 0x7E000: OTA State (4KB)
 *   0x7E000 - 0x80000: Settings/NVS (8KB)
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/crc.h>
#include <string.h>

/*
 * Boot Control Block addresses and constants
 * (Duplicated from boot_control.h to keep bootloader self-contained)
 */
#define BOOTLOADER_ADDR     0x00000000
#define BOOTLOADER_SIZE     0x00002000  /* 8KB */

#define SLOT_A_ADDR         0x00002000
#define SLOT_A_SIZE         0x0003D000  /* 244KB */

#define SLOT_B_ADDR         0x0003F000
#define SLOT_B_SIZE         0x0003D000  /* 244KB */

#define BOOT_CTRL_ADDR      0x0007C000
#define BOOT_CTRL_SIZE      0x00001000  /* 4KB */

/* Magic value: 0xB007F1E0 ("BOOT FILE" in leetspeak) */
#define BOOT_MAGIC          0xB007F1E0
#define BOOT_VERSION        1

#define SLOT_A              0
#define SLOT_B              1

/* Maximum boot attempts before automatic fallback */
#define MAX_BOOT_COUNT      3

/* Fallback reason codes (must match boot_control.h) */
#define BOOT_FALLBACK_NONE      0  /* Normal boot, no fallback */
#define BOOT_FALLBACK_CRC_FAIL  1  /* Primary slot CRC verification failed */
#define BOOT_FALLBACK_VERSION   2  /* Using lower version (other slot invalid) */
#define BOOT_FALLBACK_BOOT_COUNT 3 /* Too many boot attempts */

/**
 * Boot Control Block structure (v2)
 * Must match the structure in boot_control.h
 */
struct boot_control {
    uint32_t magic;           /* BOOT_MAGIC for validation */
    uint8_t  version;         /* Structure version */
    uint8_t  active_slot;     /* 0 = Slot A, 1 = Slot B */
    uint8_t  slot_a_valid;    /* 1 = Slot A contains valid firmware */
    uint8_t  slot_b_valid;    /* 1 = Slot B contains valid firmware */
    uint8_t  boot_count;      /* Counter for fallback detection */
    uint8_t  fallback_reason; /* Why fallback occurred (BOOT_FALLBACK_*) */
    uint8_t  last_boot_slot;  /* Which slot we actually booted */
    uint8_t  reserved;        /* Alignment padding */
    uint32_t slot_a_crc;      /* CRC32 of firmware in Slot A */
    uint32_t slot_b_crc;      /* CRC32 of firmware in Slot B */
    uint32_t slot_a_size;     /* Size of firmware in Slot A (bytes) */
    uint32_t slot_b_size;     /* Size of firmware in Slot B (bytes) */
    uint32_t slot_a_version;  /* Build number of firmware in Slot A */
    uint32_t slot_b_version;  /* Build number of firmware in Slot B */
    uint32_t checksum;        /* CRC32 of this structure (excluding this field) */
};

/* Flash device */
static const struct device *flash_dev;

/* Local copy of boot control block */
static struct boot_control boot_ctrl;

/**
 * Calculate CRC32 of boot control block (excluding checksum field)
 */
static uint32_t calc_boot_ctrl_crc(const struct boot_control *ctrl)
{
    /* CRC of everything except the last 4 bytes (checksum field) */
    return crc32_ieee((const uint8_t *)ctrl,
                      sizeof(struct boot_control) - sizeof(uint32_t));
}

/**
 * Read boot control block from flash
 *
 * @return 0 on success, negative error code on failure
 */
static int read_boot_control(void)
{
    int ret;

    ret = flash_read(flash_dev, BOOT_CTRL_ADDR, &boot_ctrl, sizeof(boot_ctrl));
    if (ret != 0) {
        return ret;
    }

    return 0;
}

/**
 * Write boot control block to flash
 * Handles erase-before-write for flash
 *
 * @return 0 on success, negative error code on failure
 */
static int write_boot_control(void)
{
    int ret;

    /* Update checksum before writing */
    boot_ctrl.checksum = calc_boot_ctrl_crc(&boot_ctrl);

    /* Erase the boot control page (4KB) */
    ret = flash_erase(flash_dev, BOOT_CTRL_ADDR, BOOT_CTRL_SIZE);
    if (ret != 0) {
        return ret;
    }

    /* Write the boot control block */
    ret = flash_write(flash_dev, BOOT_CTRL_ADDR, &boot_ctrl, sizeof(boot_ctrl));
    if (ret != 0) {
        return ret;
    }

    return 0;
}

/**
 * Validate boot control block
 *
 * @return true if valid, false if corrupt/missing
 */
static bool validate_boot_control(void)
{
    uint32_t calc_crc;

    /* Check magic */
    if (boot_ctrl.magic != BOOT_MAGIC) {
        return false;
    }

    /* Check version */
    if (boot_ctrl.version != BOOT_VERSION) {
        return false;
    }

    /* Check active_slot is valid */
    if (boot_ctrl.active_slot > SLOT_B) {
        return false;
    }

    /* Verify checksum */
    calc_crc = calc_boot_ctrl_crc(&boot_ctrl);
    if (calc_crc != boot_ctrl.checksum) {
        return false;
    }

    return true;
}

/**
 * Initialize boot control with defaults (Slot A active)
 */
static void init_default_boot_control(void)
{
    memset(&boot_ctrl, 0, sizeof(boot_ctrl));
    boot_ctrl.magic = BOOT_MAGIC;
    boot_ctrl.version = BOOT_VERSION;
    boot_ctrl.active_slot = SLOT_A;
    boot_ctrl.slot_a_valid = 1;  /* Assume Slot A has valid firmware */
    boot_ctrl.slot_b_valid = 0;
    boot_ctrl.boot_count = 0;
}

/**
 * Get the other slot (for fallback)
 */
static uint8_t get_other_slot(uint8_t slot)
{
    return (slot == SLOT_A) ? SLOT_B : SLOT_A;
}

/**
 * Check if a slot is valid (has valid firmware)
 */
static bool is_slot_valid(uint8_t slot)
{
    if (slot == SLOT_A) {
        return boot_ctrl.slot_a_valid != 0;
    } else {
        return boot_ctrl.slot_b_valid != 0;
    }
}

/**
 * Get the flash address of a slot
 */
static uint32_t get_slot_addr(uint8_t slot)
{
    return (slot == SLOT_A) ? SLOT_A_ADDR : SLOT_B_ADDR;
}

/**
 * Calculate CRC32 of firmware in slot
 *
 * @param slot_addr Flash address of slot
 * @param size Size of firmware in bytes
 * @return CRC32 of firmware
 */
static uint32_t calc_firmware_crc32(uint32_t slot_addr, uint32_t size)
{
    uint32_t crc = 0;
    uint8_t buf[256];
    uint32_t offset = 0;

    while (offset < size) {
        uint32_t chunk = (size - offset > sizeof(buf)) ? sizeof(buf) : (size - offset);
        if (flash_read(flash_dev, slot_addr + offset, buf, chunk) != 0) {
            return 0;  /* Read error */
        }
        crc = crc32_ieee_update(crc, buf, chunk);
        offset += chunk;
    }

    return crc;
}

/**
 * Verify firmware CRC32 matches stored value
 *
 * @param slot SLOT_A or SLOT_B
 * @return true if CRC matches
 */
static bool verify_slot_crc(uint8_t slot)
{
    uint32_t slot_addr = get_slot_addr(slot);
    uint32_t size = (slot == SLOT_A) ? boot_ctrl.slot_a_size : boot_ctrl.slot_b_size;
    uint32_t expected_crc = (slot == SLOT_A) ? boot_ctrl.slot_a_crc : boot_ctrl.slot_b_crc;

    if (size == 0 || expected_crc == 0) {
        return false;  /* No valid firmware info */
    }

    uint32_t actual_crc = calc_firmware_crc32(slot_addr, size);
    return (actual_crc == expected_crc);
}

/**
 * Get firmware version (build number) from slot
 */
static uint32_t get_slot_version(uint8_t slot)
{
    if (slot == SLOT_A) {
        return boot_ctrl.slot_a_version;
    } else {
        return boot_ctrl.slot_b_version;
    }
}

/**
 * Validate firmware in slot by checking vector table
 *
 * A valid ARM firmware has:
 * - Initial SP at offset 0 (must be in RAM: 0x20000000 - 0x20020000)
 * - Reset vector at offset 4 (must be in slot address range)
 */
static bool validate_firmware_vectors(uint8_t slot)
{
    uint32_t vectors[2];
    uint32_t slot_addr = get_slot_addr(slot);
    uint32_t slot_size = (slot == SLOT_A) ? SLOT_A_SIZE : SLOT_B_SIZE;
    int ret;

    /* Read initial SP and reset vector */
    ret = flash_read(flash_dev, slot_addr, vectors, sizeof(vectors));
    if (ret != 0) {
        return false;
    }

    /* Check initial SP is in RAM range (nRF52833 has 128KB RAM) */
    if (vectors[0] < 0x20000000 || vectors[0] > 0x20020000) {
        return false;
    }

    /* Check reset vector is in slot address range */
    if (vectors[1] < slot_addr || vectors[1] >= (slot_addr + slot_size)) {
        return false;
    }

    /* Basic vector table looks valid */
    return true;
}

/**
 * Jump to firmware at specified address
 *
 * This function:
 * 1. Disables interrupts
 * 2. Relocates vector table to new address
 * 3. Sets stack pointer from vector table
 * 4. Jumps to reset handler
 *
 * @param addr Flash address of firmware
 */
__attribute__((noreturn))
static void jump_to_firmware(uint32_t addr)
{
    uint32_t *vector_table = (uint32_t *)addr;
    uint32_t sp = vector_table[0];      /* Initial stack pointer */
    uint32_t reset = vector_table[1];   /* Reset handler address */

    /* Disable interrupts */
    __disable_irq();

    /* Clear pending interrupts */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;  /* Disable all interrupts */
        NVIC->ICPR[i] = 0xFFFFFFFF;  /* Clear pending interrupts */
    }

    /* Relocate vector table to new firmware location */
    SCB->VTOR = addr;

    /* Ensure all memory accesses are complete */
    __DSB();
    __ISB();

    /* Set stack pointer */
    __set_MSP(sp);

    /* Jump to reset handler */
    void (*reset_handler)(void) = (void (*)(void))reset;
    reset_handler();

    /* Should never reach here */
    while (1) {
        /* Infinite loop as fallback */
    }
}

/**
 * Main bootloader entry point
 *
 * Boot decision logic:
 * 1. Verify CRC32 of both slots
 * 2. If both valid: select higher version
 * 3. If one valid: use that slot
 * 4. Set fallback_reason if not using intended slot
 */
int main(void)
{
    int ret;
    uint8_t selected_slot;
    uint8_t fallback_reason = BOOT_FALLBACK_NONE;
    bool slot_a_crc_ok = false;
    bool slot_b_crc_ok = false;

    /*
     * Step 1: Initialize flash driver
     */
    flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    if (!device_is_ready(flash_dev)) {
        /* Flash driver not ready - try to boot Slot A anyway */
        jump_to_firmware(SLOT_A_ADDR);
    }

    /*
     * Step 2: Read boot control block
     */
    ret = read_boot_control();
    if (ret != 0) {
        /* Flash read failed - boot Slot A */
        jump_to_firmware(SLOT_A_ADDR);
    }

    /*
     * Step 3: Validate boot control block
     */
    if (!validate_boot_control()) {
        /* Boot control corrupt or missing - initialize defaults and boot Slot A */
        init_default_boot_control();
        write_boot_control();
        jump_to_firmware(SLOT_A_ADDR);
    }

    /*
     * Step 4: Verify CRC32 of both slots
     * This ensures we're booting known-good firmware
     */
    if (is_slot_valid(SLOT_A) && validate_firmware_vectors(SLOT_A)) {
        slot_a_crc_ok = verify_slot_crc(SLOT_A);
    }
    if (is_slot_valid(SLOT_B) && validate_firmware_vectors(SLOT_B)) {
        slot_b_crc_ok = verify_slot_crc(SLOT_B);
    }

    /*
     * Step 5: Select slot based on CRC validity and version
     */
    if (slot_a_crc_ok && slot_b_crc_ok) {
        /* Both slots valid - select higher version */
        uint32_t ver_a = get_slot_version(SLOT_A);
        uint32_t ver_b = get_slot_version(SLOT_B);

        if (ver_b > ver_a) {
            selected_slot = SLOT_B;
        } else {
            selected_slot = SLOT_A;
        }

        /* Check if we're NOT using the intended slot */
        if (selected_slot != boot_ctrl.active_slot) {
            fallback_reason = BOOT_FALLBACK_VERSION;
        }
    } else if (slot_a_crc_ok) {
        /* Only Slot A valid */
        selected_slot = SLOT_A;
        if (boot_ctrl.active_slot == SLOT_B) {
            fallback_reason = BOOT_FALLBACK_CRC_FAIL;
        }
    } else if (slot_b_crc_ok) {
        /* Only Slot B valid */
        selected_slot = SLOT_B;
        if (boot_ctrl.active_slot == SLOT_A) {
            fallback_reason = BOOT_FALLBACK_CRC_FAIL;
        }
    } else {
        /* Neither slot has valid CRC - try vector table validation only */
        if (validate_firmware_vectors(SLOT_A)) {
            selected_slot = SLOT_A;
        } else if (validate_firmware_vectors(SLOT_B)) {
            selected_slot = SLOT_B;
        } else {
            /* No valid firmware - boot Slot A anyway */
            selected_slot = SLOT_A;
        }
        fallback_reason = BOOT_FALLBACK_CRC_FAIL;
    }

    /*
     * Step 6: Check boot count for fallback
     */
    if (boot_ctrl.boot_count >= MAX_BOOT_COUNT) {
        uint8_t other_slot = get_other_slot(selected_slot);
        bool other_crc_ok = (other_slot == SLOT_A) ? slot_a_crc_ok : slot_b_crc_ok;

        if (other_crc_ok) {
            selected_slot = other_slot;
            fallback_reason = BOOT_FALLBACK_BOOT_COUNT;
            boot_ctrl.boot_count = 0;
        } else {
            /* Other slot not valid - reset count and continue */
            boot_ctrl.boot_count = 0;
        }
    }

    /*
     * Step 7: Update boot control with selection
     */
    boot_ctrl.active_slot = selected_slot;
    boot_ctrl.fallback_reason = fallback_reason;
    boot_ctrl.last_boot_slot = selected_slot;
    boot_ctrl.boot_count++;
    write_boot_control();

    /*
     * Step 8: Jump to selected firmware
     */
    uint32_t firmware_addr = get_slot_addr(selected_slot);
    jump_to_firmware(firmware_addr);

    /* Never reached */
    return 0;
}
