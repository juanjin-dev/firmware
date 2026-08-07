#pragma once

#include "ModemProtocol.h"

/*
 * Payload encoding and decoding. Every layout mirrors an offset table in the
 * meshkit wire protocol specification.
 *
 * Structs are never written to the wire directly: a compiler is free to pad
 * them, and the protocol is not padded. Each field is placed by hand.
 *
 * Decoders point variable-length fields back into the caller's buffer rather
 * than copying, so they stay valid only while that buffer does - which for a
 * received frame is until the next call to ModemFrameDecoder::next.
 */

#define MODEM_HELLO_SIZE 8
#define MODEM_HELLO_REPLY_SIZE 17
#define MODEM_REGION_PROFILE_SIZE 16
#define MODEM_TX_PARAMS_SIZE 4
#define MODEM_POSITION_SIZE 40
#define MODEM_CONFIG_DIGEST_SIZE 16
#define MODEM_MESH_STATUS_SIZE 32
#define MODEM_MESH_STATE_SIZE 8
#define MODEM_REBOOTED_SIZE 8
#define MODEM_SEND_REPLY_SIZE 8
#define MODEM_HEALTH_SIZE 20
#define MODEM_TX_STATUS_SIZE 8
#define MODEM_RX_DATA_HEADER_SIZE 18
#define MODEM_RX_TEXT_HEADER_SIZE 17
#define MODEM_NODE_LIST_HEAD_SIZE 4
#define MODEM_NODE_ENTRY_SIZE 16
#define MODEM_IDENTITY_REPLY_HEAD_SIZE 12
#define MODEM_NODE_INFO_HEAD_SIZE 36
#define MODEM_ENTER_BOOTLOADER_SIZE 4
#define MODEM_QUERY_TELEMETRY_SIZE 8
#define MODEM_QUERY_POSITION_SIZE 8

typedef struct {
    uint16_t proto_version;
    uint16_t abi;
} ModemHello;

typedef struct {
    uint8_t role;
    bool is_unmessagable;
    const uint8_t *long_name;
    uint8_t long_name_len;
    const uint8_t *short_name;
    uint8_t short_name_len;
} ModemIdentity;

typedef struct {
    uint8_t region;
    bool use_preset;
    uint8_t modem_preset;
    uint8_t channel_num;
    uint32_t bandwidth_hz;
    uint8_t spread_factor;
    uint8_t coding_rate;
    bool override_duty_cycle;
    uint32_t override_freq_hz;
} ModemRegionProfile;

typedef struct {
    int8_t tx_power_dbm;
    uint8_t hop_limit;
    bool tx_enabled;
} ModemTxParams;

typedef struct {
    uint8_t index;
    uint8_t role;
    const uint8_t *psk;
    uint8_t psk_len;
    const uint8_t *name;
    uint8_t name_len;
} ModemChannel;

typedef struct {
    const uint8_t *private_key;
    uint8_t key_len;
    bool generate;
} ModemSecurity;

typedef struct {
    uint8_t text_rx_mode;
    uint8_t portnum_mode;
    uint16_t max_payload_len;
    uint16_t uart_budget_pps;
    uint16_t per_sender_pps_q8;
    const uint8_t *portnums;
    uint8_t portnum_count;
    const uint8_t *senders; /* sender_count little-endian u32 values */
    uint8_t sender_count;
} ModemRxPolicy;

typedef struct {
    int32_t latitude_i;
    int32_t longitude_i;
    int32_t altitude_m;
    int32_t altitude_hae;
    uint32_t timestamp_unix;
    uint32_t ground_speed_mmps;
    uint32_t ground_track;
    uint16_t hdop;
    uint16_t pdop;
    uint16_t gps_accuracy_mm;
    uint8_t precision_bits;
    uint8_t fix_type;
    uint8_t sats_in_view;
    uint8_t loc_source;
    uint8_t alt_source;
    uint8_t flags;
} ModemPosition;

typedef struct {
    uint8_t metric_id;
    int8_t scale10;
    int32_t value;
} ModemMetric;

typedef struct {
    uint8_t variant;
    const uint8_t *metrics; /* metric_count packed 6-byte records */
    const uint8_t *utf8;
    uint8_t metric_count;
    uint8_t text_len;
} ModemTelemetry;

typedef struct {
    uint32_t dest_node;
    uint16_t portnum;
    uint8_t flags;
    uint8_t hop_limit;
    uint8_t channel_index;
    const uint8_t *payload;
    uint16_t payload_len;
} ModemSendBinary;

typedef struct {
    uint32_t dest_node;
    uint8_t channel_index;
    uint8_t flags;
    const uint8_t *utf8;
    uint16_t text_len;
} ModemSendText;

typedef struct {
    uint32_t from_node;
    uint8_t portnum;
    uint8_t channel_index;
    uint8_t hop_away;
    uint8_t flags;
    int16_t rssi_dbm;
    int8_t snr_q2;
    uint32_t rx_time;
} ModemRxDataHeader;

typedef struct {
    uint32_t from_node;
    uint8_t channel_index;
    uint8_t hop_away;
    int16_t rssi_dbm;
    int8_t snr_q2;
    uint32_t rx_time;
} ModemRxTextHeader;

typedef struct {
    uint32_t session_epoch;
    uint32_t uptime_s;
    uint16_t node_count;
    uint16_t tx_queue_depth;
    uint16_t rx_dropped;
    uint8_t channel_util_pct;
    uint8_t air_util_tx_pct;
    uint8_t state;
} ModemHealth;

typedef struct {
    uint32_t tx_id;
    uint8_t state;
    uint8_t err;
    uint8_t hops_used;
} ModemTxStatus;

typedef struct {
    uint8_t region;
    uint8_t modem_preset;
    uint8_t channel_count;
    uint32_t node_num;
    uint32_t config_crc32;
    uint16_t channel_num;
    uint8_t hop_limit;
    int8_t tx_power_dbm;
} ModemConfigDigest;

typedef struct {
    uint8_t channel_util_pct;
    uint8_t air_util_tx_pct;
    uint16_t tx_queue_depth;
    uint16_t rx_queue_depth;
    uint32_t rx_total;
    uint32_t rx_dropped_policy;
    uint32_t rx_dropped_rate_limit;
    uint32_t rx_decrypt_failed;
    uint32_t tx_total;
    uint32_t tx_failed;
} ModemMeshStatus;

typedef struct {
    uint32_t nodedb_generation;
    uint16_t node_count;
} ModemMeshState;

typedef struct {
    uint32_t session_epoch;
    uint8_t firmware_version[3];
    uint8_t reason;
} ModemRebooted;

typedef struct {
    uint32_t node_num;
    uint32_t last_heard;
    int8_t snr_q2;
    uint8_t hops_away;
    uint8_t role;
    uint8_t flags;
} ModemNodeEntry;

typedef struct {
    uint32_t node_num;
    uint8_t role;
    bool is_unmessagable;
    uint8_t hw_model;
} ModemIdentityInfo;

typedef struct {
    uint32_t node_num;
    uint32_t last_heard;
    int32_t latitude_i;
    int32_t longitude_i;
    int32_t altitude_m;
    uint32_t position_time;
    uint16_t voltage_mv;
    int8_t snr_q2;
    uint8_t hops_away;
    uint8_t role;
    uint8_t flags;
    uint8_t battery_level;
} ModemNodeInfo;

/* Decoders: host -> modem. Return false on a short or malformed payload. */

bool modemDecodeHello(const uint8_t *buf, uint16_t len, ModemHello *out);
bool modemDecodeIdentity(const uint8_t *buf, uint16_t len, ModemIdentity *out);
bool modemDecodeRegionProfile(const uint8_t *buf, uint16_t len, ModemRegionProfile *out);
bool modemDecodeTxParams(const uint8_t *buf, uint16_t len, ModemTxParams *out);
bool modemDecodeChannel(const uint8_t *buf, uint16_t len, ModemChannel *out);
bool modemDecodeSecurity(const uint8_t *buf, uint16_t len, ModemSecurity *out);
bool modemDecodeRxPolicy(const uint8_t *buf, uint16_t len, ModemRxPolicy *out);
bool modemDecodePosition(const uint8_t *buf, uint16_t len, ModemPosition *out);
bool modemDecodeTelemetry(const uint8_t *buf, uint16_t len, ModemTelemetry *out);
bool modemDecodeSendBinary(const uint8_t *buf, uint16_t len, ModemSendBinary *out);
bool modemDecodeSendText(const uint8_t *buf, uint16_t len, ModemSendText *out);
bool modemDecodeEnterBootloader(const uint8_t *buf, uint16_t len);

/// True when the bytes are well-formed UTF-8: no truncated or overlong
/// sequences, no surrogates, nothing past U+10FFFF.
bool modemIsValidUtf8(const uint8_t *buf, uint16_t len);

/// Reads metric *index* out of a telemetry payload; false when out of range.
bool modemTelemetryMetric(const ModemTelemetry *telemetry, uint8_t index, ModemMetric *out);

/// Reads sender *index* out of a receive policy; false when out of range.
bool modemRxPolicySender(const ModemRxPolicy *policy, uint8_t index, uint32_t *out);

/* Encoders: modem -> host. Return bytes written, or 0 if the buffer is short. */

uint16_t modemEncodeHelloReply(uint8_t *buf, uint16_t cap, uint8_t status, uint16_t proto_version,
                               uint16_t abi, uint32_t session_epoch, uint32_t node_num, const uint8_t firmware_version[3],
                               uint8_t capability_count);
uint16_t modemEncodeConfigDigest(uint8_t *buf, uint16_t cap, uint8_t status, const ModemConfigDigest *in);
uint16_t modemEncodeMeshStatus(uint8_t *buf, uint16_t cap, uint8_t status, const ModemMeshStatus *in);
uint16_t modemEncodeSendReply(uint8_t *buf, uint16_t cap, uint8_t status, uint32_t tx_id);
uint16_t modemEncodeHealth(uint8_t *buf, uint16_t cap, const ModemHealth *in);
uint16_t modemEncodeTxStatus(uint8_t *buf, uint16_t cap, const ModemTxStatus *in);
uint16_t modemEncodeMeshState(uint8_t *buf, uint16_t cap, const ModemMeshState *in);
uint16_t modemEncodeRebooted(uint8_t *buf, uint16_t cap, const ModemRebooted *in);
uint16_t modemEncodeRxData(uint8_t *buf, uint16_t cap, const ModemRxDataHeader *header, const uint8_t *payload,
                           uint16_t payload_len);
uint16_t modemEncodeRxText(uint8_t *buf, uint16_t cap, const ModemRxTextHeader *header, const uint8_t *utf8,
                           uint16_t text_len);
uint16_t modemEncodeNodeListHead(uint8_t *buf, uint16_t cap, uint8_t status, uint8_t count, uint8_t total);
uint16_t modemEncodeNodeEntry(uint8_t *buf, uint16_t cap, const ModemNodeEntry *in);
uint16_t modemEncodeQueryTelemetry(uint8_t *buf, uint16_t cap, uint32_t from_node, uint8_t variant);
uint16_t modemEncodeQueryPosition(uint8_t *buf, uint16_t cap, uint32_t from_node);
uint16_t modemEncodePosition(uint8_t *buf, uint16_t cap, const ModemPosition *in);
uint16_t modemEncodeIdentityReply(uint8_t *buf, uint16_t cap, uint8_t status, const ModemIdentityInfo *in,
                                  const char *long_name, const char *short_name);
uint16_t modemEncodeNodeInfo(uint8_t *buf, uint16_t cap, uint8_t status, const ModemNodeInfo *in, const char *long_name,
                             const char *short_name);
bool modemDecodeNodeInfoRequest(const uint8_t *buf, uint16_t len, uint32_t *out_node_num);

/// Error responses carry the offending field's offset in byte 1.
uint16_t modemEncodeBadParam(uint8_t *buf, uint16_t cap, uint8_t field_offset);
