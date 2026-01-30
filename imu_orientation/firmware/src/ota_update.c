/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE OTA Firmware Update Module
 *
 * Implements block-by-block firmware transfer with:
 * - CRC16 per-block validation
 * - Resume capability after connection loss
 * - Full CRC32 validation before commit
 * - Firmware backup to host
 * - Boot pointer switch (A/B slot)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/crc.h>
#include <string.h>
#include <stdlib.h>

#include "ota_update.h"
#include "boot_control.h"
#include "robot_state.h"
#include "version.h"
#include <zephyr/sys/reboot.h>

/* Thread ID getters from other modules */
extern k_tid_t sensors_get_thread_id(void);
extern k_tid_t motor_get_thread_id(void);
extern k_tid_t ir_sensors_get_thread_id(void);
extern k_tid_t autonav_get_thread_id(void);
extern k_tid_t telemetry_get_thread_id(void);

/* Track if threads are suspended */
static bool threads_suspended = false;

/* Flash device */
#define FLASH_DEVICE DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller))

/* Page size for nRF52833 */
#define FLASH_PAGE_SIZE 4096

/* OTA State persistence magic */
#define OTA_PERSIST_MAGIC 0x4F544153  /* "OTAS" */
#define OTA_PERSIST_VERSION 1

/* Page buffer for accumulating blocks before flash write */
static uint8_t page_buffer[FLASH_PAGE_SIZE] __aligned(4);
static uint16_t page_buffer_offset;
static uint32_t page_buffer_addr;  /* Flash address for current page */

/* OTA context */
static struct {
    enum ota_state state;
    uint8_t target_slot;
    uint16_t last_block;
    uint16_t total_blocks;
    uint32_t firmware_size;
    uint32_t expected_crc;
    uint32_t bytes_received;
    uint32_t slot_addr;
    uint32_t slot_size;
    char version[16];

    /* Backup state */
    uint16_t backup_block;
    uint16_t backup_total_blocks;
    uint32_t backup_size;

    /* Erase tracking */
    uint32_t erase_offset;
    uint32_t erase_total;
} ota_ctx;

/* Persistent state structure (stored at OTA_STATE_ADDR) */
struct ota_persist_state {
    uint32_t magic;
    uint8_t version;
    uint8_t state;
    uint8_t target_slot;
    uint8_t reserved;
    uint16_t last_block;
    uint16_t total_blocks;
    uint32_t firmware_size;
    uint32_t expected_crc;
    uint32_t bytes_received;
    uint32_t checksum;
} __packed;

/* Send callback */
static ota_send_callback_t send_callback;

/* Response buffer */
static uint8_t resp_buf[OTA_MAX_PACKET];

/* =========================================================================
 * Thread Suspend/Resume for OTA
 *
 * During OTA we suspend non-essential threads to give flash operations
 * CPU time and prevent interference. The scheduler handles blocking.
 * ========================================================================= */

/**
 * Suspend non-essential threads during OTA
 */
static void ota_suspend_threads(void)
{
    if (threads_suspended) {
        return;
    }

    k_tid_t tid;

    /* Suspend sensor thread */
    tid = sensors_get_thread_id();
    if (tid) {
        k_thread_suspend(tid);
    }

    /* Suspend motor thread */
    tid = motor_get_thread_id();
    if (tid) {
        k_thread_suspend(tid);
    }

    /* Suspend IR sensor thread */
    tid = ir_sensors_get_thread_id();
    if (tid) {
        k_thread_suspend(tid);
    }

    /* Suspend autonomous navigation thread */
    tid = autonav_get_thread_id();
    if (tid) {
        k_thread_suspend(tid);
    }

    /* Suspend telemetry thread */
    tid = telemetry_get_thread_id();
    if (tid) {
        k_thread_suspend(tid);
    }

    threads_suspended = true;
    printk("OTA: Threads suspended\n");
}

/**
 * Resume threads after OTA completes or aborts
 */
static void ota_resume_threads(void)
{
    if (!threads_suspended) {
        return;
    }

    k_tid_t tid;

    /* Resume in reverse order */
    tid = telemetry_get_thread_id();
    if (tid) {
        k_thread_resume(tid);
    }

    tid = autonav_get_thread_id();
    if (tid) {
        k_thread_resume(tid);
    }

    tid = ir_sensors_get_thread_id();
    if (tid) {
        k_thread_resume(tid);
    }

    tid = motor_get_thread_id();
    if (tid) {
        k_thread_resume(tid);
    }

    tid = sensors_get_thread_id();
    if (tid) {
        k_thread_resume(tid);
    }

    threads_suspended = false;
    printk("OTA: Threads resumed\n");
}

/* =========================================================================
 * CRC Functions
 * ========================================================================= */

/**
 * CRC16-CCITT (polynomial 0x1021, init 0xFFFF)
 * Used for per-block validation
 */
static uint16_t ota_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/**
 * CRC32-IEEE (uses Zephyr's crc32_ieee)
 * Used for full firmware validation
 */
static uint32_t calc_firmware_crc32(uint32_t addr, uint32_t size)
{
    const struct device *flash_dev = FLASH_DEVICE;
    uint32_t crc = 0;
    uint8_t buf[256];
    uint32_t remaining = size;
    uint32_t offset = 0;

    while (remaining > 0) {
        size_t chunk = MIN(remaining, sizeof(buf));
        int ret = flash_read(flash_dev, addr + offset, buf, chunk);
        if (ret != 0) {
            return 0;
        }

        crc = crc32_ieee_update(crc, buf, chunk);
        offset += chunk;
        remaining -= chunk;
    }

    return crc;
}

/* =========================================================================
 * Flash Operations
 * ========================================================================= */

/**
 * Flush page buffer to flash
 *
 * NOTE: Pages are pre-erased during OTA_CMD_INIT, so we only write here.
 *
 * CRITICAL: Flash write blocks the CPU (~42ms for 4KB). During this time,
 * BLE events cannot be processed, which can cause connection drops.
 * Solution: Write in smaller chunks (256 bytes = ~2.6ms each) with yields
 * between chunks to allow BLE stack to process events.
 */
#define FLASH_CHUNK_SIZE 256  /* ~2.6ms per chunk, gives BLE time to process */

static int flush_page_buffer(void)
{
    if (page_buffer_offset == 0) {
        return 0;  /* Nothing to flush */
    }

    const struct device *flash_dev = FLASH_DEVICE;

    if (!device_is_ready(flash_dev)) {
        return -ENODEV;
    }

    /* Write in chunks to avoid blocking BLE for too long */
    uint32_t write_addr = page_buffer_addr;
    uint16_t remaining = page_buffer_offset;
    uint16_t buf_offset = 0;
    int ret;

    while (remaining > 0) {
        uint16_t chunk = MIN(remaining, FLASH_CHUNK_SIZE);

        /* Write chunk */
        ret = flash_write(flash_dev, write_addr, &page_buffer[buf_offset], chunk);
        if (ret != 0) {
            printk("OTA: Flash write failed at 0x%08X: %d\n", write_addr, ret);
            return ret;
        }

        write_addr += chunk;
        buf_offset += chunk;
        remaining -= chunk;

        /* Sleep between chunks to let BLE stack process events.
         * Flash operations block CPU, preventing BLE event processing.
         * Longer sleep gives BLE more time to handle connection events. */
        if (remaining > 0) {
            k_sleep(K_MSEC(5));
        }
    }

    /* Longer sleep after page write to let BLE catch up completely */
    k_sleep(K_MSEC(30));

    /* Verify write (just first 64 bytes for speed) */
    uint8_t verify_buf[64];
    ret = flash_read(flash_dev, page_buffer_addr, verify_buf, MIN(page_buffer_offset, sizeof(verify_buf)));
    if (ret != 0 || memcmp(verify_buf, page_buffer, MIN(page_buffer_offset, sizeof(verify_buf))) != 0) {
        printk("OTA: Flash verify failed at 0x%08X\n", page_buffer_addr);
        return -EIO;
    }

    /* Move to next page */
    page_buffer_addr += FLASH_PAGE_SIZE;
    page_buffer_offset = 0;
    memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);  /* Reset to erased state */

    return 0;
}

/**
 * Add data to page buffer, flush when full
 */
static int buffer_write(const uint8_t *data, size_t len)
{
    while (len > 0) {
        size_t space = FLASH_PAGE_SIZE - page_buffer_offset;
        size_t copy = MIN(len, space);

        memcpy(&page_buffer[page_buffer_offset], data, copy);
        page_buffer_offset += copy;
        data += copy;
        len -= copy;

        if (page_buffer_offset >= FLASH_PAGE_SIZE) {
            int ret = flush_page_buffer();
            if (ret != 0) {
                return ret;
            }
        }
    }

    return 0;
}

/**
 * Erase a single page (called progressively)
 */
static int erase_next_page(void)
{
    if (ota_ctx.erase_offset >= ota_ctx.erase_total) {
        return 1;  /* Done */
    }

    const struct device *flash_dev = FLASH_DEVICE;
    uint32_t addr = ota_ctx.slot_addr + ota_ctx.erase_offset;

    int ret = flash_erase(flash_dev, addr, FLASH_PAGE_SIZE);
    if (ret != 0) {
        return ret;
    }

    ota_ctx.erase_offset += FLASH_PAGE_SIZE;

    return (ota_ctx.erase_offset >= ota_ctx.erase_total) ? 1 : 0;
}

/* =========================================================================
 * State Persistence
 * ========================================================================= */

/**
 * Calculate checksum for persist state
 */
static uint32_t calc_persist_checksum(const struct ota_persist_state *ps)
{
    size_t len = sizeof(*ps) - sizeof(ps->checksum);
    return crc32_ieee((const uint8_t *)ps, len);
}

/**
 * Save OTA state to flash for resume capability
 */
static int save_ota_state(void)
{
    const struct device *flash_dev = FLASH_DEVICE;
    struct ota_persist_state ps;

    if (!device_is_ready(flash_dev)) {
        return -ENODEV;
    }

    ps.magic = OTA_PERSIST_MAGIC;
    ps.version = OTA_PERSIST_VERSION;
    ps.state = ota_ctx.state;
    ps.target_slot = ota_ctx.target_slot;
    ps.reserved = 0;
    ps.last_block = ota_ctx.last_block;
    ps.total_blocks = ota_ctx.total_blocks;
    ps.firmware_size = ota_ctx.firmware_size;
    ps.expected_crc = ota_ctx.expected_crc;
    ps.bytes_received = ota_ctx.bytes_received;
    ps.checksum = calc_persist_checksum(&ps);

    int ret = flash_erase(flash_dev, OTA_STATE_ADDR, OTA_STATE_SIZE);
    if (ret != 0) {
        return ret;
    }

    ret = flash_write(flash_dev, OTA_STATE_ADDR, &ps, sizeof(ps));
    if (ret != 0) {
        return ret;
    }
    return 0;
}

/**
 * Load OTA state from flash
 */
static int load_ota_state(void)
{
    const struct device *flash_dev = FLASH_DEVICE;
    struct ota_persist_state ps;

    if (!device_is_ready(flash_dev)) {
        return -ENODEV;
    }

    int ret = flash_read(flash_dev, OTA_STATE_ADDR, &ps, sizeof(ps));
    if (ret != 0) {
        return ret;
    }

    /* Validate magic and checksum */
    if (ps.magic != OTA_PERSIST_MAGIC) {
        return -ENOENT;
    }

    if (ps.version != OTA_PERSIST_VERSION) {
        return -EINVAL;
    }

    if (ps.checksum != calc_persist_checksum(&ps)) {
        return -EINVAL;
    }

    /* Restore state */
    ota_ctx.state = ps.state;
    ota_ctx.target_slot = ps.target_slot;
    ota_ctx.last_block = ps.last_block;
    ota_ctx.total_blocks = ps.total_blocks;
    ota_ctx.firmware_size = ps.firmware_size;
    ota_ctx.expected_crc = ps.expected_crc;
    ota_ctx.bytes_received = ps.bytes_received;
    ota_ctx.slot_addr = boot_control_get_slot_addr(ota_ctx.target_slot);
    ota_ctx.slot_size = boot_control_get_slot_size(ota_ctx.target_slot);

    /* Setup page buffer for next write position */
    uint32_t written = ota_ctx.bytes_received;
    page_buffer_addr = ota_ctx.slot_addr + (written / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    page_buffer_offset = written % FLASH_PAGE_SIZE;

    /* If we have a partial page, we need to read it back */
    if (page_buffer_offset > 0) {
        ret = flash_read(flash_dev, page_buffer_addr, page_buffer, page_buffer_offset);
        if (ret != 0) {
            return ret;
        }
    }

    return 0;
}

/**
 * Clear saved OTA state
 */
static int clear_ota_state(void)
{
    const struct device *flash_dev = FLASH_DEVICE;

    if (!device_is_ready(flash_dev)) {
        return -ENODEV;
    }

    return flash_erase(flash_dev, OTA_STATE_ADDR, OTA_STATE_SIZE);
}

/* =========================================================================
 * Response Helpers
 * ========================================================================= */

/**
 * Send response packet
 */
static int send_response(const uint8_t *data, uint16_t len)
{
    if (!send_callback) {
        return -EINVAL;
    }
    return send_callback(data, len);
}

/**
 * Send simple error response
 */
static int send_error(uint8_t error_code)
{
    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = OTA_RESP_ERROR;
    resp_buf[2] = error_code;
    return send_response(resp_buf, 3);
}

/**
 * Send ACK for received block
 */
static int send_ack(uint16_t block_num)
{
    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = OTA_RESP_ACK;
    resp_buf[2] = block_num & 0xFF;
    resp_buf[3] = (block_num >> 8) & 0xFF;
    return send_response(resp_buf, 4);
}

/**
 * Send NAK for failed block
 */
static int send_nak(uint16_t block_num, uint8_t error)
{
    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = OTA_RESP_NAK;
    resp_buf[2] = block_num & 0xFF;
    resp_buf[3] = (block_num >> 8) & 0xFF;
    resp_buf[4] = error;
    return send_response(resp_buf, 5);
}

/**
 * Send current status
 */
static int send_status(uint8_t resp_type)
{
    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = resp_type;

    struct ota_status *status = (struct ota_status *)&resp_buf[2];
    status->state = ota_ctx.state;
    status->target_slot = ota_ctx.target_slot;
    status->last_block = ota_ctx.last_block;
    status->total_blocks = ota_ctx.total_blocks;
    status->firmware_size = ota_ctx.firmware_size;
    status->expected_crc = ota_ctx.expected_crc;
    status->bytes_received = ota_ctx.bytes_received;

    return send_response(resp_buf, 2 + sizeof(struct ota_status));
}

/* =========================================================================
 * Command Handlers
 * ========================================================================= */

/**
 * Handle OTA_CMD_INIT - Start new OTA transfer
 */
static int handle_cmd_init(const uint8_t *data, uint16_t len)
{
    if (len < 2 + sizeof(struct ota_init_payload)) {
        return send_error(OTA_ERR_INVALID);
    }

    if (ota_ctx.state != OTA_STATE_IDLE && ota_ctx.state != OTA_STATE_SUSPENDED) {
        return send_error(OTA_ERR_STATE);
    }

    const struct ota_init_payload *init = (const struct ota_init_payload *)&data[2];

    /* Validate size */
    ota_ctx.target_slot = boot_control_get_inactive_slot();
    ota_ctx.slot_addr = boot_control_get_slot_addr(ota_ctx.target_slot);
    ota_ctx.slot_size = boot_control_get_slot_size(ota_ctx.target_slot);

    if (init->size > ota_ctx.slot_size || init->size == 0) {
        return send_error(OTA_ERR_SIZE);
    }

    /* Store parameters */
    ota_ctx.firmware_size = init->size;
    ota_ctx.expected_crc = init->crc32;
    ota_ctx.total_blocks = (init->size + OTA_BLOCK_SIZE - 1) / OTA_BLOCK_SIZE;
    ota_ctx.last_block = 0;
    ota_ctx.bytes_received = 0;
    memcpy(ota_ctx.version, init->version, sizeof(ota_ctx.version));
    ota_ctx.version[15] = '\0';

    /* Setup page buffer */
    page_buffer_addr = ota_ctx.slot_addr;
    page_buffer_offset = 0;
    memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);

    /* Setup erase */
    ota_ctx.erase_offset = 0;
    ota_ctx.erase_total = (init->size + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1);

    /* Mark target slot as invalid before erasing */
    boot_control_mark_slot_invalid(ota_ctx.target_slot);

    printk("OTA: slot %c, %u B\n", ota_ctx.target_slot == SLOT_A ? 'A' : 'B', ota_ctx.firmware_size);

    /* Disable motors and suspend non-essential threads during OTA */
    extern void motor_enable(bool enable);
    extern void motor_emergency_stop(void);
    motor_emergency_stop();
    motor_enable(false);

    /* Switch display to OTA mode BEFORE erase to prevent display flickering.
     * Main thread is NOT suspended, so it will keep updating display. */
    robot_set_state(ROBOT_STATE_OTA);
    ota_display_progress(0);

    /* Suspend threads - scheduler will block them until OTA completes */
    ota_suspend_threads();

    /* Start erasing */
    ota_ctx.state = OTA_STATE_ERASE;

    /* Erase all pages (this may take a while) */
    while (ota_ctx.erase_offset < ota_ctx.erase_total) {
        int ret = erase_next_page();
        if (ret < 0) {
            ota_ctx.state = OTA_STATE_IDLE;
            ota_resume_threads();  /* CRITICAL: Resume threads on error! */
            motor_enable(true);    /* Re-enable motors */
            robot_set_state(ROBOT_STATE_IDLE);
            return send_error(OTA_ERR_FLASH);
        }
        /* Sleep to let BLE stack process events and maintain connection.
         * 50ms is long enough for BLE to process multiple events without
         * risking supervision timeout (typically 4-6 seconds). */
        k_sleep(K_MSEC(50));
    }

    ota_ctx.state = OTA_STATE_TRANSFER;

    /* Save state for resume */
    save_ota_state();

    return send_status(OTA_RESP_READY);
}

/**
 * Handle OTA_CMD_DATA - Receive data block
 */
static int handle_cmd_data(const uint8_t *data, uint16_t len)
{
    if (ota_ctx.state != OTA_STATE_TRANSFER) {
        return send_error(OTA_ERR_STATE);
    }

    /* Minimum: magic(1) + cmd(1) + block_num(2) + crc16(2) + at least 1 byte data */
    if (len < 7) {
        return send_error(OTA_ERR_INVALID);
    }

    uint16_t block_num = data[2] | (data[3] << 8);
    uint16_t block_crc = data[4] | (data[5] << 8);
    const uint8_t *block_data = &data[6];
    uint16_t block_len = len - 6;

    /* Check block number */
    uint16_t expected_block = ota_ctx.last_block + 1;

    if (block_num == ota_ctx.last_block) {
        return send_ack(block_num);
    }

    if (block_num != expected_block || block_num > ota_ctx.total_blocks) {
        return send_nak(block_num, OTA_ERR_SEQUENCE);
    }

    /* Verify block CRC16 */
    uint16_t calc_crc = ota_crc16_ccitt(block_data, block_len);
    if (calc_crc != block_crc) {
        return send_nak(block_num, OTA_ERR_CRC);
    }

    /* Check if this is the last block with padding */
    uint32_t remaining = ota_ctx.firmware_size - ota_ctx.bytes_received;
    if (remaining < block_len) {
        block_len = remaining;  /* Truncate to actual firmware size */
    }

    /* Write to page buffer */
    int ret = buffer_write(block_data, block_len);
    if (ret != 0) {
        /* CRITICAL: Resume threads on flash error! */
        ota_resume_threads();
        extern void motor_enable(bool enable);
        motor_enable(true);
        robot_set_state(ROBOT_STATE_IDLE);
        ota_ctx.state = OTA_STATE_IDLE;
        clear_ota_state();
        return send_nak(block_num, OTA_ERR_FLASH);
    }

    ota_ctx.last_block = block_num;
    ota_ctx.bytes_received += block_len;

    /* Debug: Log first, milestone, and last blocks */
    if (block_num == 1 || block_num == ota_ctx.total_blocks) {
        printk("OTA: blk %u/%u, %u/%u B\n",
               block_num, ota_ctx.total_blocks,
               ota_ctx.bytes_received, ota_ctx.firmware_size);
    }

    /* Update progress display only every 44 blocks (~4% = 1 LED per update) */
    if ((block_num % 44) == 0 || block_num == ota_ctx.total_blocks) {
        ota_display_progress(ota_get_progress());
    }

    /* Save state periodically for resume (every 100 blocks) */
    if ((block_num % 100) == 0) {
        save_ota_state();
    }

    /* Check if transfer complete */
    if (block_num == ota_ctx.total_blocks) {
        /* Flush remaining data */
        ret = flush_page_buffer();
        if (ret != 0) {
            /* CRITICAL: Resume threads on final flush error! */
            ota_resume_threads();
            extern void motor_enable(bool enable);
            motor_enable(true);
            robot_set_state(ROBOT_STATE_IDLE);
            ota_ctx.state = OTA_STATE_IDLE;
            clear_ota_state();
            return send_nak(block_num, OTA_ERR_FLASH);
        }

        ota_ctx.state = OTA_STATE_VALIDATE;
        save_ota_state();
    }

    return send_ack(block_num);
}

/**
 * Handle OTA_CMD_QUERY - Return status for resume
 */
static int handle_cmd_query(const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    /* If we were suspended, try to load saved state */
    if (ota_ctx.state == OTA_STATE_IDLE) {
        int ret = load_ota_state();
        if (ret == 0 && ota_ctx.state == OTA_STATE_TRANSFER) {
            ota_ctx.state = OTA_STATE_SUSPENDED;
        }
    }

    /* If suspended, move back to transfer on query */
    if (ota_ctx.state == OTA_STATE_SUSPENDED) {
        ota_ctx.state = OTA_STATE_TRANSFER;
    }

    return send_status(OTA_RESP_STATUS);
}

/**
 * Handle OTA_CMD_ABORT - Cancel current OTA
 */
static int handle_cmd_abort(const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    ota_ctx.state = OTA_STATE_ABORT;
    clear_ota_state();

    /* Reset context */
    memset(&ota_ctx, 0, sizeof(ota_ctx));
    ota_ctx.state = OTA_STATE_IDLE;

    /* Return to idle display */
    robot_set_state(ROBOT_STATE_IDLE);

    /* Resume threads after OTA abort */
    ota_resume_threads();

    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = OTA_RESP_OK;
    return send_response(resp_buf, 2);
}

/**
 * Handle OTA_CMD_VALIDATE - Calculate and verify CRC32
 */
static int handle_cmd_validate(const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    if (ota_ctx.state != OTA_STATE_VALIDATE) {
        return send_error(OTA_ERR_STATE);
    }

    printk("OTA: Validating %u bytes at 0x%08X...\n",
           ota_ctx.firmware_size, ota_ctx.slot_addr);

    uint32_t actual_crc = calc_firmware_crc32(ota_ctx.slot_addr, ota_ctx.firmware_size);

    printk("OTA: CRC exp=0x%08X act=0x%08X\n", ota_ctx.expected_crc, actual_crc);

    if (actual_crc != ota_ctx.expected_crc) {
        printk("OTA: FAIL - CRC mismatch!\n");

        /* Show error animation */
        ota_display_error();

        resp_buf[0] = OTA_MAGIC;
        resp_buf[1] = OTA_RESP_VALID_FAIL;
        memcpy(&resp_buf[2], &ota_ctx.expected_crc, 4);
        memcpy(&resp_buf[6], &actual_crc, 4);

        ota_ctx.state = OTA_STATE_IDLE;
        robot_set_state(ROBOT_STATE_IDLE);  /* Return to idle display */
        clear_ota_state();

        /* Resume threads after OTA failure */
        ota_resume_threads();

        return send_response(resp_buf, 10);
    }

    printk("OTA: OK\n");

    /* Show success animation */
    ota_display_success();

    /* Parse build number from version string (format: "1.0.0-ble.N")
     * Find last '.' and parse the number after it */
    uint32_t build_num = 0;
    const char *last_dot = strrchr(ota_ctx.version, '.');
    if (last_dot != NULL) {
        build_num = (uint32_t)strtoul(last_dot + 1, NULL, 10);
    }
    printk("OTA: Version=%s, Build=%u\n", ota_ctx.version, build_num);

    /* Mark slot as valid with build number for version tracking */
    int ret = boot_control_mark_slot_valid(ota_ctx.target_slot,
                                           ota_ctx.firmware_size,
                                           ota_ctx.expected_crc,
                                           build_num);
    if (ret != 0) {
        /* CRITICAL: Resume threads on boot control error! */
        ota_resume_threads();
        extern void motor_enable(bool enable);
        motor_enable(true);
        robot_set_state(ROBOT_STATE_IDLE);
        ota_ctx.state = OTA_STATE_IDLE;
        return send_error(OTA_ERR_FLASH);
    }

    /* Switch boot pointer to new slot */
    ret = boot_control_switch_slot(ota_ctx.target_slot);
    if (ret != 0) {
        /* CRITICAL: Resume threads on boot control error! */
        ota_resume_threads();
        extern void motor_enable(bool enable);
        motor_enable(true);
        robot_set_state(ROBOT_STATE_IDLE);
        ota_ctx.state = OTA_STATE_IDLE;
        return send_error(OTA_ERR_FLASH);
    }

    /* Clear OTA state - transfer complete */
    clear_ota_state();

    ota_ctx.state = OTA_STATE_COMMIT;

    /* Send success response */
    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = OTA_RESP_VALID_OK;
    send_response(resp_buf, 2);

    /* Give BLE time to send the response before rebooting */
    k_sleep(K_MSEC(500));

    /* Reboot to new firmware */
    sys_reboot(SYS_REBOOT_COLD);

    /* Never reached */
    return 0;
}

/**
 * Handle OTA_CMD_BACKUP_REQ - Start backup of active slot
 */
static int handle_cmd_backup_req(const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    if (ota_ctx.state != OTA_STATE_IDLE) {
        return send_error(OTA_ERR_STATE);
    }

    uint8_t active_slot = boot_control_get_active_slot();
    ota_ctx.backup_size = boot_control_get_firmware_size(active_slot);

    if (ota_ctx.backup_size == 0) {
        return send_error(OTA_ERR_INVALID);
    }

    ota_ctx.backup_block = 0;
    ota_ctx.backup_total_blocks = (ota_ctx.backup_size + OTA_BLOCK_SIZE - 1) / OTA_BLOCK_SIZE;
    ota_ctx.slot_addr = boot_control_get_slot_addr(active_slot);
    ota_ctx.state = OTA_STATE_BACKUP;

    /* Send READY response with size and block count */
    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = OTA_RESP_READY;
    memcpy(&resp_buf[2], &ota_ctx.backup_size, 4);
    memcpy(&resp_buf[6], &ota_ctx.backup_total_blocks, 2);

    return send_response(resp_buf, 8);
}

/**
 * Handle OTA_CMD_BACKUP_ACK - Send next backup block
 *
 * Block numbering is 0-based to match Python client expectations.
 * backup_block tracks the NEXT block to send (starts at 0).
 */
static int handle_cmd_backup_ack(const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    if (ota_ctx.state != OTA_STATE_BACKUP) {
        return send_error(OTA_ERR_STATE);
    }

    /* Check if all blocks have been sent */
    if (ota_ctx.backup_block >= ota_ctx.backup_total_blocks) {
        /* Backup complete */
        uint32_t crc = calc_firmware_crc32(ota_ctx.slot_addr, ota_ctx.backup_size);

        resp_buf[0] = OTA_MAGIC;
        resp_buf[1] = OTA_RESP_BACKUP_DONE;
        memcpy(&resp_buf[2], &ota_ctx.backup_size, 4);
        memcpy(&resp_buf[6], &crc, 4);

        ota_ctx.state = OTA_STATE_IDLE;
        return send_response(resp_buf, 10);
    }

    /* Calculate block offset and length (0-based indexing) */
    uint32_t offset = ota_ctx.backup_block * OTA_BLOCK_SIZE;
    uint16_t block_len = OTA_BLOCK_SIZE;

    if (offset + block_len > ota_ctx.backup_size) {
        block_len = ota_ctx.backup_size - offset;
    }

    /* Read block from flash */
    const struct device *flash_dev = FLASH_DEVICE;
    uint8_t block_data[OTA_BLOCK_SIZE];

    int ret = flash_read(flash_dev, ota_ctx.slot_addr + offset, block_data, block_len);
    if (ret != 0) {
        ota_ctx.state = OTA_STATE_IDLE;
        return send_error(OTA_ERR_FLASH);
    }

    /* Calculate CRC16 */
    uint16_t crc = ota_crc16_ccitt(block_data, block_len);

    /* Build response with current block number (0-based) */
    resp_buf[0] = OTA_MAGIC;
    resp_buf[1] = OTA_RESP_BACKUP_DATA;
    resp_buf[2] = ota_ctx.backup_block & 0xFF;
    resp_buf[3] = (ota_ctx.backup_block >> 8) & 0xFF;
    resp_buf[4] = crc & 0xFF;
    resp_buf[5] = (crc >> 8) & 0xFF;
    memcpy(&resp_buf[6], block_data, block_len);

    /* Increment block counter AFTER sending */
    ota_ctx.backup_block++;

    return send_response(resp_buf, 6 + block_len);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

int ota_init(void)
{
    memset(&ota_ctx, 0, sizeof(ota_ctx));
    ota_ctx.state = OTA_STATE_IDLE;

    /* Check for saved state (interrupted transfer) */
    int ret = load_ota_state();
    if (ret == 0 && (ota_ctx.state == OTA_STATE_TRANSFER ||
                     ota_ctx.state == OTA_STATE_VALIDATE)) {
        ota_ctx.state = OTA_STATE_SUSPENDED;
    } else {
        ota_ctx.state = OTA_STATE_IDLE;
    }

    return 0;
}

int ota_process_packet(const uint8_t *data, uint16_t len)
{
    if (!data || len < 2) {
        return -EINVAL;
    }

    if (data[0] != OTA_MAGIC) {
        return -EINVAL;  /* Not an OTA packet */
    }

    uint8_t cmd = data[1];

    switch (cmd) {
    case OTA_CMD_INIT:
        return handle_cmd_init(data, len);

    case OTA_CMD_DATA:
        return handle_cmd_data(data, len);

    case OTA_CMD_QUERY:
        return handle_cmd_query(data, len);

    case OTA_CMD_ABORT:
        return handle_cmd_abort(data, len);

    case OTA_CMD_VALIDATE:
        return handle_cmd_validate(data, len);

    case OTA_CMD_BACKUP_REQ:
        return handle_cmd_backup_req(data, len);

    case OTA_CMD_BACKUP_ACK:
        return handle_cmd_backup_ack(data, len);

    default:
        return send_error(OTA_ERR_INVALID);
    }
}

enum ota_state ota_get_state(void)
{
    return ota_ctx.state;
}

bool ota_is_active(void)
{
    return ota_ctx.state != OTA_STATE_IDLE;
}

void ota_on_disconnect(void)
{
    if (ota_ctx.state == OTA_STATE_TRANSFER) {
        ota_ctx.state = OTA_STATE_SUSPENDED;
        save_ota_state();
    } else if (ota_ctx.state == OTA_STATE_BACKUP) {
        ota_ctx.state = OTA_STATE_IDLE;
    }
}

void ota_on_connect(void)
{
    /* Nothing to do - QUERY will resume */
}

int ota_abort(void)
{
    if (ota_ctx.state == OTA_STATE_IDLE) {
        return 0;  /* Nothing to abort */
    }

    return handle_cmd_abort(NULL, 0);
}

uint8_t ota_get_progress(void)
{
    if (ota_ctx.total_blocks == 0) {
        return 0;
    }

    return (uint8_t)((ota_ctx.last_block * 100) / ota_ctx.total_blocks);
}

void ota_set_send_callback(ota_send_callback_t cb)
{
    send_callback = cb;
}

const char *ota_state_name(enum ota_state state)
{
    switch (state) {
    case OTA_STATE_IDLE:      return "IDLE";
    case OTA_STATE_BACKUP:    return "BACKUP";
    case OTA_STATE_ERASE:     return "ERASE";
    case OTA_STATE_TRANSFER:  return "TRANSFER";
    case OTA_STATE_SUSPENDED: return "SUSPENDED";
    case OTA_STATE_VALIDATE:  return "VALIDATE";
    case OTA_STATE_COMMIT:    return "COMMIT";
    case OTA_STATE_ABORT:     return "ABORT";
    default:                  return "UNKNOWN";
    }
}
