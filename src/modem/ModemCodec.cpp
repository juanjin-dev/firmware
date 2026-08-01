#include "ModemCodec.h"

#include <string.h>

static inline uint8_t rd8(const uint8_t *p)
{
    return p[0];
}

static inline uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void wr8(uint8_t *p, uint8_t v)
{
    p[0] = v;
}

static inline void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

// ---------------------------------------------------------------------------
// Decoders

bool modemDecodeHello(const uint8_t *buf, uint16_t len, ModemHello *out)
{
    if (len != MODEM_HELLO_SIZE)
        return false;
    out->proto_version = rd16(&buf[0]);
    out->abi = rd16(&buf[2]);
    return true;
}

bool modemDecodeIdentity(const uint8_t *buf, uint16_t len, ModemIdentity *out)
{
    if (len < 4)
        return false;

    const uint8_t long_len = buf[2];
    const uint8_t short_len = buf[3];
    if ((uint32_t)len < 4u + long_len + short_len)
        return false;

    out->role = buf[0];
    out->is_unmessagable = buf[1] != 0;
    out->long_name = &buf[4];
    out->long_name_len = long_len;
    out->short_name = &buf[4 + long_len];
    out->short_name_len = short_len;
    return true;
}

bool modemDecodeRegionProfile(const uint8_t *buf, uint16_t len, ModemRegionProfile *out)
{
    if (len != MODEM_REGION_PROFILE_SIZE)
        return false;

    out->region = buf[0];
    out->use_preset = buf[1] != 0;
    out->modem_preset = buf[2];
    out->channel_num = buf[3];
    out->bandwidth_hz = rd32(&buf[4]);
    out->spread_factor = buf[8];
    out->coding_rate = buf[9];
    out->override_duty_cycle = buf[10] != 0;
    out->override_freq_hz = rd32(&buf[12]);
    return true;
}

bool modemDecodeTxParams(const uint8_t *buf, uint16_t len, ModemTxParams *out)
{
    if (len != MODEM_TX_PARAMS_SIZE)
        return false;
    out->tx_power_dbm = (int8_t)buf[0];
    out->hop_limit = buf[1];
    out->tx_enabled = buf[2] != 0;
    return true;
}

bool modemDecodeChannel(const uint8_t *buf, uint16_t len, ModemChannel *out)
{
    if (len < 4)
        return false;

    const uint8_t psk_len = buf[2];
    const uint8_t name_len = buf[3];
    if (psk_len != 0 && psk_len != 1 && psk_len != 16 && psk_len != 32)
        return false;
    if ((uint32_t)len < 4u + psk_len + name_len)
        return false;

    out->index = buf[0];
    out->role = buf[1];
    out->psk = &buf[4];
    out->psk_len = psk_len;
    out->name = &buf[4 + psk_len];
    out->name_len = name_len;
    return true;
}

bool modemDecodeSecurity(const uint8_t *buf, uint16_t len, ModemSecurity *out)
{
    if (len < 4)
        return false;

    const uint8_t key_len = buf[0];
    if (key_len != 0 && key_len != 32)
        return false;
    if ((uint32_t)len < 4u + key_len)
        return false;

    out->key_len = key_len;
    out->generate = (buf[1] & 0x01) != 0;
    out->private_key = &buf[4];
    return true;
}

bool modemDecodeRxPolicy(const uint8_t *buf, uint16_t len, ModemRxPolicy *out)
{
    if (len < 12)
        return false;

    const uint8_t portnum_count = buf[2];
    const uint8_t sender_count = buf[3];
    if ((uint32_t)len < 12u + portnum_count + (uint32_t)sender_count * 4u)
        return false;

    out->text_rx_mode = buf[0];
    out->portnum_mode = buf[1];
    out->portnum_count = portnum_count;
    out->sender_count = sender_count;
    out->max_payload_len = rd16(&buf[4]);
    out->uart_budget_pps = rd16(&buf[6]);
    out->per_sender_pps_q8 = rd16(&buf[8]);
    out->portnums = &buf[12];
    out->senders = &buf[12 + portnum_count];
    return true;
}

bool modemRxPolicySender(const ModemRxPolicy *policy, uint8_t index, uint32_t *out)
{
    if (index >= policy->sender_count)
        return false;
    *out = rd32(&policy->senders[(uint32_t)index * 4u]);
    return true;
}

bool modemDecodePosition(const uint8_t *buf, uint16_t len, ModemPosition *out)
{
    if (len != MODEM_POSITION_SIZE)
        return false;

    out->latitude_i = (int32_t)rd32(&buf[0]);
    out->longitude_i = (int32_t)rd32(&buf[4]);
    out->altitude_m = (int32_t)rd32(&buf[8]);
    out->timestamp_unix = rd32(&buf[12]);
    out->precision_bits = buf[16];
    out->fix_type = buf[17];
    out->sats_in_view = buf[18];
    out->flags = buf[19];
    return true;
}

bool modemDecodeTelemetry(const uint8_t *buf, uint16_t len, ModemTelemetry *out)
{
    if (len < 2)
        return false;

    const uint8_t count = buf[1];
    if ((uint32_t)len < 2u + (uint32_t)count * 6u)
        return false;

    out->variant = buf[0];
    out->metric_count = count;
    out->metrics = &buf[2];
    return true;
}

bool modemTelemetryMetric(const ModemTelemetry *telemetry, uint8_t index, ModemMetric *out)
{
    if (index >= telemetry->metric_count)
        return false;

    const uint8_t *p = &telemetry->metrics[(uint32_t)index * 6u];
    out->metric_id = p[0];
    out->scale10 = (int8_t)p[1];
    out->value = (int32_t)rd32(&p[2]);
    return true;
}

bool modemDecodeSendBinary(const uint8_t *buf, uint16_t len, ModemSendBinary *out)
{
    if (len < 10)
        return false;

    const uint16_t payload_len = rd16(&buf[8]);
    if ((uint32_t)len < 10u + payload_len)
        return false;

    out->dest_node = rd32(&buf[0]);
    out->portnum = buf[4];
    out->flags = buf[5];
    out->hop_limit = buf[6];
    out->channel_index = buf[7];
    out->payload = &buf[10];
    out->payload_len = payload_len;
    return true;
}

bool modemDecodeSendText(const uint8_t *buf, uint16_t len, ModemSendText *out)
{
    if (len < 8)
        return false;

    const uint16_t text_len = rd16(&buf[6]);
    if ((uint32_t)len < 8u + text_len)
        return false;

    out->dest_node = rd32(&buf[0]);
    out->channel_index = buf[4];
    out->flags = buf[5];
    out->utf8 = &buf[8];
    out->text_len = text_len;
    return true;
}

bool modemDecodeEnterBootloader(const uint8_t *buf, uint16_t len)
{
    return len == MODEM_ENTER_BOOTLOADER_SIZE && rd32(buf) == MODEM_BOOTLOADER_MAGIC;
}

// ---------------------------------------------------------------------------
// Encoders

uint16_t modemEncodeHelloReply(uint8_t *buf, uint16_t cap, uint8_t status, uint16_t proto_version, uint16_t abi,
                               uint32_t session_epoch, uint32_t node_num, const uint8_t firmware_version[3],
                               uint8_t capability_count)
{
    if (cap < MODEM_HELLO_REPLY_SIZE)
        return 0;

    wr8(&buf[0], status);
    wr16(&buf[1], proto_version);
    wr16(&buf[3], abi);
    wr32(&buf[5], session_epoch);
    wr32(&buf[9], node_num);
    buf[13] = firmware_version[0];
    buf[14] = firmware_version[1];
    buf[15] = firmware_version[2];
    wr8(&buf[16], capability_count);
    return MODEM_HELLO_REPLY_SIZE;
}

uint16_t modemEncodeConfigDigest(uint8_t *buf, uint16_t cap, uint8_t status, const ModemConfigDigest *in)
{
    if (cap < MODEM_CONFIG_DIGEST_SIZE)
        return 0;

    wr8(&buf[0], status);
    wr8(&buf[1], in->region);
    wr8(&buf[2], in->modem_preset);
    wr8(&buf[3], in->channel_count);
    wr32(&buf[4], in->node_num);
    wr32(&buf[8], in->config_crc32);
    wr16(&buf[12], in->channel_num);
    wr8(&buf[14], in->hop_limit);
    wr8(&buf[15], (uint8_t)in->tx_power_dbm);
    return MODEM_CONFIG_DIGEST_SIZE;
}

uint16_t modemEncodeMeshStatus(uint8_t *buf, uint16_t cap, uint8_t status, const ModemMeshStatus *in)
{
    if (cap < MODEM_MESH_STATUS_SIZE)
        return 0;

    wr8(&buf[0], status);
    wr8(&buf[1], in->channel_util_pct);
    wr8(&buf[2], in->air_util_tx_pct);
    wr8(&buf[3], 0);
    wr16(&buf[4], in->tx_queue_depth);
    wr16(&buf[6], in->rx_queue_depth);
    wr32(&buf[8], in->rx_total);
    wr32(&buf[12], in->rx_dropped_policy);
    wr32(&buf[16], in->rx_dropped_rate_limit);
    wr32(&buf[20], in->rx_decrypt_failed);
    wr32(&buf[24], in->tx_total);
    wr32(&buf[28], in->tx_failed);
    return MODEM_MESH_STATUS_SIZE;
}

uint16_t modemEncodeSendReply(uint8_t *buf, uint16_t cap, uint8_t status, uint32_t tx_id)
{
    if (cap < MODEM_SEND_REPLY_SIZE)
        return 0;

    wr8(&buf[0], status);
    buf[1] = buf[2] = buf[3] = 0;
    wr32(&buf[4], status == MODEM_OK ? tx_id : 0u);
    return MODEM_SEND_REPLY_SIZE;
}

uint16_t modemEncodeHealth(uint8_t *buf, uint16_t cap, const ModemHealth *in)
{
    if (cap < MODEM_HEALTH_SIZE)
        return 0;

    wr32(&buf[0], in->session_epoch);
    wr32(&buf[4], in->uptime_s);
    wr16(&buf[8], in->node_count);
    wr16(&buf[10], in->tx_queue_depth);
    wr16(&buf[12], in->rx_dropped);
    wr8(&buf[14], in->channel_util_pct);
    wr8(&buf[15], in->air_util_tx_pct);
    wr8(&buf[16], in->state);
    buf[17] = buf[18] = buf[19] = 0;
    return MODEM_HEALTH_SIZE;
}

uint16_t modemEncodeTxStatus(uint8_t *buf, uint16_t cap, const ModemTxStatus *in)
{
    if (cap < MODEM_TX_STATUS_SIZE)
        return 0;

    wr32(&buf[0], in->tx_id);
    wr8(&buf[4], in->state);
    wr8(&buf[5], in->err);
    wr8(&buf[6], in->hops_used);
    wr8(&buf[7], 0);
    return MODEM_TX_STATUS_SIZE;
}

uint16_t modemEncodeMeshState(uint8_t *buf, uint16_t cap, const ModemMeshState *in)
{
    if (cap < MODEM_MESH_STATE_SIZE)
        return 0;

    wr32(&buf[0], in->nodedb_generation);
    wr16(&buf[4], in->node_count);
    wr16(&buf[6], 0);
    return MODEM_MESH_STATE_SIZE;
}

uint16_t modemEncodeRebooted(uint8_t *buf, uint16_t cap, const ModemRebooted *in)
{
    if (cap < MODEM_REBOOTED_SIZE)
        return 0;

    wr32(&buf[0], in->session_epoch);
    buf[4] = in->firmware_version[0];
    buf[5] = in->firmware_version[1];
    buf[6] = in->firmware_version[2];
    wr8(&buf[7], in->reason);
    return MODEM_REBOOTED_SIZE;
}

uint16_t modemEncodeRxData(uint8_t *buf, uint16_t cap, const ModemRxDataHeader *header, const uint8_t *payload,
                           uint16_t payload_len)
{
    const uint32_t total = MODEM_RX_DATA_HEADER_SIZE + (uint32_t)payload_len;
    if (cap < total || total > MODEM_MAX_PAYLOAD)
        return 0;

    wr32(&buf[0], header->from_node);
    wr8(&buf[4], header->portnum);
    wr8(&buf[5], header->channel_index);
    wr8(&buf[6], header->hop_away);
    wr8(&buf[7], header->flags);
    wr16(&buf[8], (uint16_t)header->rssi_dbm);
    wr8(&buf[10], (uint8_t)header->snr_q2);
    wr8(&buf[11], 0);
    wr32(&buf[12], header->rx_time);
    wr16(&buf[16], payload_len);
    if (payload_len)
        memcpy(&buf[MODEM_RX_DATA_HEADER_SIZE], payload, payload_len);
    return (uint16_t)total;
}

uint16_t modemEncodeRxText(uint8_t *buf, uint16_t cap, const ModemRxTextHeader *header, const uint8_t *utf8,
                           uint16_t text_len)
{
    const uint32_t total = MODEM_RX_TEXT_HEADER_SIZE + (uint32_t)text_len;
    if (cap < total || total > MODEM_MAX_PAYLOAD)
        return 0;

    wr32(&buf[0], header->from_node);
    wr8(&buf[4], header->channel_index);
    wr8(&buf[5], header->hop_away);
    wr8(&buf[6], 0);
    wr16(&buf[7], (uint16_t)header->rssi_dbm);
    wr8(&buf[9], (uint8_t)header->snr_q2);
    wr8(&buf[10], 0);
    wr32(&buf[11], header->rx_time);
    wr16(&buf[15], text_len);
    if (text_len)
        memcpy(&buf[MODEM_RX_TEXT_HEADER_SIZE], utf8, text_len);
    return (uint16_t)total;
}

uint16_t modemEncodeNodeListHead(uint8_t *buf, uint16_t cap, uint8_t status, uint8_t count, uint8_t total)
{
    if (cap < MODEM_NODE_LIST_HEAD_SIZE)
        return 0;

    wr8(&buf[0], status);
    wr8(&buf[1], count);
    wr8(&buf[2], total);
    wr8(&buf[3], 0);
    return MODEM_NODE_LIST_HEAD_SIZE;
}

uint16_t modemEncodeNodeEntry(uint8_t *buf, uint16_t cap, const ModemNodeEntry *in)
{
    if (cap < MODEM_NODE_ENTRY_SIZE)
        return 0;

    wr32(&buf[0], in->node_num);
    wr32(&buf[4], in->last_heard);
    wr8(&buf[8], (uint8_t)in->snr_q2);
    wr8(&buf[9], in->hops_away);
    wr8(&buf[10], in->role);
    wr8(&buf[11], in->flags);
    buf[12] = buf[13] = buf[14] = buf[15] = 0;
    return MODEM_NODE_ENTRY_SIZE;
}

static uint16_t appendNames(uint8_t *buf, uint16_t cap, uint16_t head, const char *long_name, const char *short_name,
                            uint8_t long_len_offset)
{
    const size_t long_len = long_name ? strnlen(long_name, 0xFF) : 0;
    const size_t short_len = short_name ? strnlen(short_name, 0xFF) : 0;
    if ((size_t)cap < head + long_len + short_len)
        return 0;

    wr8(&buf[long_len_offset], (uint8_t)long_len);
    wr8(&buf[long_len_offset + 1], (uint8_t)short_len);
    memcpy(&buf[head], long_name, long_len);
    memcpy(&buf[head + long_len], short_name, short_len);
    return (uint16_t)(head + long_len + short_len);
}

uint16_t modemEncodeIdentityReply(uint8_t *buf, uint16_t cap, uint8_t status, const ModemIdentityInfo *in,
                                  const char *long_name, const char *short_name)
{
    if (cap < MODEM_IDENTITY_REPLY_HEAD_SIZE)
        return 0;

    wr8(&buf[0], status);
    wr8(&buf[1], in->role);
    wr8(&buf[2], in->is_unmessagable ? 1 : 0);
    wr8(&buf[3], in->hw_model);
    wr32(&buf[4], in->node_num);
    buf[10] = buf[11] = 0;
    return appendNames(buf, cap, MODEM_IDENTITY_REPLY_HEAD_SIZE, long_name, short_name, 8);
}

uint16_t modemEncodeNodeInfo(uint8_t *buf, uint16_t cap, uint8_t status, const ModemNodeInfo *in, const char *long_name,
                             const char *short_name)
{
    if (cap < MODEM_NODE_INFO_HEAD_SIZE)
        return 0;

    wr8(&buf[0], status);
    wr8(&buf[1], in->role);
    wr8(&buf[2], in->hops_away);
    wr8(&buf[3], in->flags);
    wr32(&buf[4], in->node_num);
    wr32(&buf[8], in->last_heard);
    wr32(&buf[12], (uint32_t)in->latitude_i);
    wr32(&buf[16], (uint32_t)in->longitude_i);
    wr32(&buf[20], (uint32_t)in->altitude_m);
    wr32(&buf[24], in->position_time);
    wr8(&buf[28], (uint8_t)in->snr_q2);
    wr8(&buf[29], in->battery_level);
    wr16(&buf[30], in->voltage_mv);
    buf[34] = buf[35] = 0;
    return appendNames(buf, cap, MODEM_NODE_INFO_HEAD_SIZE, long_name, short_name, 32);
}

bool modemDecodeNodeInfoRequest(const uint8_t *buf, uint16_t len, uint32_t *out_node_num)
{
    if (len != 4)
        return false;

    *out_node_num = rd32(&buf[0]);
    return true;
}

uint16_t modemEncodeBadParam(uint8_t *buf, uint16_t cap, uint8_t field_offset)
{
    if (cap < 2)
        return 0;

    wr8(&buf[0], MODEM_ERR_BAD_PARAM);
    wr8(&buf[1], field_offset);
    return 2;
}
