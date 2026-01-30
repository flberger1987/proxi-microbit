/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE OTA Firmware Update Module
 *
 * Features:
 * - Block-by-block transfer with CRC16 validation
 * - Resume capability after connection loss
 * - Full CRC32 validation before commit
 * - Firmware backup to host before update
 * - Boot pointer switch (no copy needed)
 */

#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol constants */
#define OTA_MAGIC           0xF0        /* First byte of OTA packets */
#define OTA_BLOCK_SIZE      224         /* Data bytes per block */
#define OTA_MAX_PACKET      244         /* Max BLE packet (MTU - overhead) */

/* OTA Commands (Host → Device) */
#define OTA_CMD_INIT        0x01        /* Start OTA: size(4), crc32(4), version(16) */
#define OTA_CMD_DATA        0x02        /* Data block: block_num(2), crc16(2), data(224) */
#define OTA_CMD_QUERY       0x03        /* Query status for resume */
#define OTA_CMD_ABORT       0x04        /* Abort current OTA */
#define OTA_CMD_VALIDATE    0x05        /* Trigger final validation */
#define OTA_CMD_BACKUP_REQ  0x06        /* Request firmware backup */
#define OTA_CMD_BACKUP_ACK  0x07        /* Acknowledge backup block received */

/* OTA Responses (Device → Host) */
#define OTA_RESP_OK         0x00        /* Generic success */
#define OTA_RESP_READY      0x01        /* Ready for transfer (with status) */
#define OTA_RESP_ACK        0x02        /* Block received OK: block_num(2) */
#define OTA_RESP_NAK        0x03        /* Block CRC failed: block_num(2), error(1) */
#define OTA_RESP_STATUS     0x04        /* Current status (for resume) */
#define OTA_RESP_VALID_OK   0x05        /* Validation passed */
#define OTA_RESP_VALID_FAIL 0x06        /* Validation failed: expected(4), actual(4) */
#define OTA_RESP_BACKUP_DATA 0x07       /* Backup block: block_num(2), crc16(2), data(224) */
#define OTA_RESP_BACKUP_DONE 0x08       /* Backup complete: size(4), crc32(4) */
#define OTA_RESP_ERROR      0x10        /* Error: error_code(1) */

/* Error codes */
#define OTA_ERR_NONE        0x00
#define OTA_ERR_CRC         0x01        /* CRC mismatch */
#define OTA_ERR_SEQUENCE    0x02        /* Wrong block number */
#define OTA_ERR_SIZE        0x03        /* Size mismatch */
#define OTA_ERR_FLASH       0x04        /* Flash operation failed */
#define OTA_ERR_STATE       0x05        /* Invalid state for operation */
#define OTA_ERR_TIMEOUT     0x06        /* Operation timed out */
#define OTA_ERR_INVALID     0x07        /* Invalid packet/parameter */

/* OTA State Machine */
enum ota_state {
    OTA_STATE_IDLE = 0,         /* Ready for new OTA */
    OTA_STATE_BACKUP,           /* Sending firmware backup to host */
    OTA_STATE_ERASE,            /* Erasing target slot */
    OTA_STATE_TRANSFER,         /* Receiving blocks */
    OTA_STATE_SUSPENDED,        /* Connection lost, waiting for resume */
    OTA_STATE_VALIDATE,         /* Validating received firmware */
    OTA_STATE_COMMIT,           /* Switching boot pointer */
    OTA_STATE_ABORT             /* Aborting, returning to idle */
};

/* OTA Status structure (for RESP_STATUS and RESP_READY) */
struct ota_status {
    uint8_t  state;             /* Current OTA state */
    uint8_t  target_slot;       /* Target slot (A or B) */
    uint16_t last_block;        /* Last successfully received block */
    uint16_t total_blocks;      /* Total blocks expected */
    uint32_t firmware_size;     /* Total firmware size in bytes */
    uint32_t expected_crc;      /* Expected CRC32 */
    uint32_t bytes_received;    /* Bytes received so far */
} __packed;

/* OTA Init packet payload */
struct ota_init_payload {
    uint32_t size;              /* Firmware size in bytes */
    uint32_t crc32;             /* Expected CRC32 of firmware */
    char     version[16];       /* Version string (null-terminated) */
} __packed;

/**
 * Initialize OTA subsystem
 * Called once at startup
 *
 * @return 0 on success, negative error code on failure
 */
int ota_init(void);

/**
 * Process incoming OTA packet
 * Called from BLE NUS receive handler
 *
 * @param data Packet data (first byte is OTA_MAGIC, second is command)
 * @param len  Packet length
 * @return 0 on success, negative error code on failure
 */
int ota_process_packet(const uint8_t *data, uint16_t len);

/**
 * Get current OTA state
 *
 * @return Current state enum
 */
enum ota_state ota_get_state(void);

/**
 * Check if OTA is in progress
 *
 * @return true if OTA is active (not IDLE)
 */
bool ota_is_active(void);

/**
 * Handle BLE disconnection
 * Moves to SUSPENDED state if transfer was in progress
 */
void ota_on_disconnect(void);

/**
 * Handle BLE reconnection
 * Stays in SUSPENDED state until host sends QUERY
 */
void ota_on_connect(void);

/**
 * Abort current OTA operation
 * Can be called externally (e.g., on user button press)
 *
 * @return 0 on success
 */
int ota_abort(void);

/**
 * Get transfer progress percentage
 *
 * @return 0-100 percentage, or 0 if not transferring
 */
uint8_t ota_get_progress(void);

/**
 * Callback type for sending OTA responses
 * Must be set before OTA can function
 */
typedef int (*ota_send_callback_t)(const uint8_t *data, uint16_t len);

/**
 * Set the callback for sending OTA responses over BLE
 *
 * @param cb Callback function
 */
void ota_set_send_callback(ota_send_callback_t cb);

/**
 * Get human-readable state name
 *
 * @param state OTA state
 * @return State name string
 */
const char *ota_state_name(enum ota_state state);

#endif /* OTA_UPDATE_H */
