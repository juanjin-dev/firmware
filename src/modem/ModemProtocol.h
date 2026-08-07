#pragma once

/*
 * Host <-> modem wire protocol definitions. See README.md for the specification.
 *
 * Must not depend on Meshtastic types: a host implementation restates these
 * from the specification, not from this source.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Negotiated in HELLO. Governs the meaning of message types and payloads.
#define MODEM_PROTOCOL_VERSION 1

/// Doubles as the framing generation marker. An incompatible frame layout takes
/// a new value, and peers built against the old one discard rather than misparse.
#define MODEM_SOF 0xA5

#define MODEM_MAX_PAYLOAD 512
#define MODEM_HEADER_SIZE 5
#define MODEM_CRC_SIZE 2
#define MODEM_FRAME_OVERHEAD (MODEM_HEADER_SIZE + MODEM_CRC_SIZE)
#define MODEM_MAX_FRAME_SIZE (MODEM_FRAME_OVERHEAD + MODEM_MAX_PAYLOAD)

#define MODEM_TYPE_RESPONSE_BIT 0x80
#define MODEM_TYPE_MODEM_BIT 0x40
#define MODEM_TYPE_KIND_MASK 0xC0

typedef enum {
    MODEM_CMD_HELLO = 0x01,
    MODEM_CMD_GET_MODEM_STATUS = 0x02,

    MODEM_CMD_SET_IDENTITY = 0x10,
    MODEM_CMD_SET_REGION_PROFILE = 0x11,
    MODEM_CMD_SET_TX_PARAMS = 0x12,
    MODEM_CMD_SET_CHANNEL = 0x13,
    MODEM_CMD_SET_SECURITY = 0x14,
    MODEM_CMD_SET_RX_POLICY = 0x15,
    MODEM_CMD_GET_IDENTITY = 0x1E,
    MODEM_CMD_GET_CONFIG_DIGEST = 0x1F,

    MODEM_CMD_SET_POSITION = 0x20,
    MODEM_CMD_SEND_TELEMETRY = 0x21,
    MODEM_CMD_SEND_BINARY = 0x22,
    MODEM_CMD_SEND_TEXT = 0x23,
    MODEM_CMD_ANNOUNCE = 0x24,

    MODEM_CMD_GET_MESH_STATUS = 0x30,
    MODEM_CMD_GET_NODE_LIST = 0x31,
    MODEM_CMD_GET_NODE_INFO = 0x32,

    MODEM_CMD_ENTER_BOOTLOADER = 0x3F,
} ModemCommand;

typedef enum {
    MODEM_EVT_REBOOTED = 0x40,
    MODEM_EVT_HEALTH = 0x41,
    MODEM_EVT_RX_DATA = 0x42,
    MODEM_EVT_RX_TEXT = 0x43,
    MODEM_EVT_TX_STATUS = 0x44,
    MODEM_EVT_MESH_STATE = 0x45,

    MODEM_QRY_TELEMETRY = 0x60,
    MODEM_QRY_POSITION = 0x61,
} ModemEvent;

typedef enum {
    MODEM_OK = 0x00,
    MODEM_ERR_UNKNOWN_TYPE = 0x01,
    MODEM_ERR_BAD_LENGTH = 0x02,
    MODEM_ERR_BAD_PARAM = 0x03,
    MODEM_ERR_NOT_HANDSHAKED = 0x04,
    MODEM_ERR_BUSY = 0x05,
    MODEM_ERR_TX_QUEUE_FULL = 0x06,
    MODEM_ERR_NOT_SUPPORTED = 0x07,
    MODEM_ERR_RADIO = 0x08,
    MODEM_ERR_STORAGE = 0x09,
    MODEM_ERR_RATE_LIMITED = 0x0A,
} ModemStatus;

#define MODEM_T_FRAME_MS 100
#define MODEM_T_RESPONSE_MS 1000
/// Separate from MODEM_T_RESPONSE_MS: committing to LittleFS stalls instruction
/// fetch for an internal flash page erase, which can outlast a transmission.
#define MODEM_T_RESPONSE_WRITE_MS 3000
#define MODEM_T_EVENT_ACK_MS 500
#define MODEM_T_HEALTH_MS 10000
#define MODEM_T_IDLE_MS 30000
#define MODEM_T_QUERY_MS 400
/// Upper bounds on how long a transmission may sit in one state before the
/// modem gives up on it and reports a terminal outcome.
#define MODEM_T_TX_SEND_MS 30000
#define MODEM_T_TX_ACK_MS 60000

#define MODEM_EVENT_ACK_ATTEMPTS 3

#define MODEM_SEND_WANT_ACK 0x01
#define MODEM_SEND_WANT_RESPONSE 0x02
#define MODEM_SEND_PKI 0x04

/// What one mesh frame carries, and what encrypting to a recipient costs out of
/// it. A payload is refused up front rather than failing after the modem has
/// already answered that it accepted the send.
#define MODEM_MAX_MESH_PAYLOAD 233
#define MODEM_PKI_OVERHEAD 12

/// A retried command must not execute twice: a repeated SEND_* would otherwise
/// put the same measurement on the mesh a second time.
#define MODEM_RESPONSE_CACHE_DEPTH 8

/// Guards ENTER_MODEM_BOOTLOADER, which cannot be undone from the host side.
#define MODEM_BOOTLOADER_MAGIC 0x1EB00710u

typedef enum {
    MODEM_REBOOT_POWER_ON = 0,
    MODEM_REBOOT_RESET_PIN = 1,
    MODEM_REBOOT_WATCHDOG = 2,
    MODEM_REBOOT_SOFTWARE = 3,
    MODEM_REBOOT_BROWNOUT = 4,
} ModemRebootReason;

typedef enum {
    MODEM_TEXT_RX_ALL = 0,
    MODEM_TEXT_RX_ALLOWLIST = 1,
    MODEM_TEXT_RX_BLOCK = 2,
} ModemTextRxMode;

typedef enum {
    MODEM_PORTNUM_ALL = 0,
    MODEM_PORTNUM_ALLOWLIST = 1,
} ModemPortnumMode;

typedef enum {
    MODEM_TX_QUEUED = 0,
    MODEM_TX_SENT = 1,
    MODEM_TX_ACKED = 2,
    MODEM_TX_TIMEOUT = 3,
    MODEM_TX_FAILED = 4,
} ModemTxState;

typedef enum {
    MODEM_STATE_NORMAL = 0,
    MODEM_STATE_CONGESTED = 1,
    MODEM_STATE_UNDER_ATTACK = 2,
    MODEM_STATE_QUARANTINE = 3,
} ModemHealthState;

static inline bool modemTypeIsCommand(uint8_t type)
{
    return (type & MODEM_TYPE_KIND_MASK) == 0x00 && type != 0x00;
}

static inline bool modemTypeIsEvent(uint8_t type)
{
    return (type & MODEM_TYPE_KIND_MASK) == MODEM_TYPE_MODEM_BIT;
}

static inline bool modemTypeIsCommandResponse(uint8_t type)
{
    return (type & MODEM_TYPE_KIND_MASK) == MODEM_TYPE_RESPONSE_BIT;
}

static inline bool modemTypeIsHostResponse(uint8_t type)
{
    return (type & MODEM_TYPE_KIND_MASK) == MODEM_TYPE_KIND_MASK;
}

/// RX events carry mesh packets already received; lost on the UART they are
/// gone. State events are recoverable by polling and go unanswered.
static inline bool modemEventNeedsAck(uint8_t type)
{
    return type == MODEM_EVT_RX_DATA || type == MODEM_EVT_RX_TEXT;
}

static inline uint8_t modemResponseFor(uint8_t request)
{
    return (uint8_t)(request | MODEM_TYPE_RESPONSE_BIT);
}

static inline uint8_t modemRequestFor(uint8_t response)
{
    return (uint8_t)(response & ~MODEM_TYPE_RESPONSE_BIT);
}
