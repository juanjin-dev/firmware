#include "IndustrialModemModule.h"

#include "Channels.h"
#include "MeshService.h"
#include "HardwareRNG.h"
#include "RadioLibInterface.h"
#include "Router.h"
#include "NodeDB.h"
#include "airtime.h"
#include "modules/NodeInfoModule.h"
#include "main.h"
#include "TypeConversions.h"
#include "RTC.h"
#include "mesh-pb-constants.h"
#include "meshUtils.h"
#include <math.h>
#include <string.h>

/// Long enough for the response to reach the host before the radio restarts.
#define MODEM_REBOOT_DELAY_MS 1500

IndustrialModemModule *industrialModemModule;

#define MODEM_UART Serial1
#define MODEM_BAUD 115200
#define MODEM_ABI 0

/// Poll interval. The receive buffer holds about 22 ms at 115200 baud, so the
/// service has to run well inside that or bytes are lost while the scheduler is
/// elsewhere.
#define MODEM_SERVICE_MS 5

IndustrialModemModule::IndustrialModemModule() : MeshModule("IndustrialModem"), concurrency::OSThread("IndustrialModem") {}

int32_t IndustrialModemModule::runOnce()
{
    if (!started) {
        MODEM_UART.begin(MODEM_BAUD);
        decoder.reset();

        // Seeded here rather than in platform setup because the entropy comes from
        // the radio's wideband noise, and the radio is not up until setup returns.
        uint32_t entropy = 0;
        if (!HardwareRNG::seed(entropy))
            entropy = micros();
        randomSeed(entropy);

        sessionEpoch = random(1, INT32_MAX);
        started = true;

        ModemRebooted rebooted = {};
        rebooted.session_epoch = sessionEpoch;
        rebooted.firmware_version[0] = 2;
        rebooted.firmware_version[1] = 7;
        rebooted.firmware_version[2] = 27;
        rebooted.reason = MODEM_REBOOT_POWER_ON;

        uint8_t payload[MODEM_REBOOTED_SIZE];
        const uint16_t len = modemEncodeRebooted(payload, sizeof(payload), &rebooted);
        emit(MODEM_EVT_REBOOTED, payload, len);
    }

    service();
    expirePendingTx();
    expirePendingQuery();
    return MODEM_SERVICE_MS;
}

bool IndustrialModemModule::wantPacket(const meshtastic_MeshPacket *p)
{
    if (p->which_payload_variant != meshtastic_MeshPacket_decoded_tag)
        return false;

    return p->decoded.portnum == meshtastic_PortNum_ROUTING_APP ||
           p->decoded.portnum == meshtastic_PortNum_TELEMETRY_APP ||
           p->decoded.portnum == meshtastic_PortNum_POSITION_APP;
}

ProcessMessage IndustrialModemModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.decoded.portnum == meshtastic_PortNum_ROUTING_APP)
        onRouting(mp);
    else if (mp.decoded.portnum == meshtastic_PortNum_TELEMETRY_APP)
        recordPeerTelemetry(mp);
    return ProcessMessage::CONTINUE;
}

void IndustrialModemModule::recordPeerTelemetry(const meshtastic_MeshPacket &mp)
{
    if (!nodeDB)
        return;

    meshtastic_Telemetry heard = meshtastic_Telemetry_init_default;
    if (!pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Telemetry_msg, &heard))
        return;
    if (heard.which_variant != meshtastic_Telemetry_device_metrics_tag)
        return;

    nodeDB->updateTelemetry(getFrom(&mp), heard, RX_SRC_RADIO);
}

void IndustrialModemModule::onRouting(const meshtastic_MeshPacket &mp)
{
    const uint32_t id = mp.decoded.request_id;
    if (!id)
        return;

    meshtastic_Routing routing = meshtastic_Routing_init_default;
    if (!pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Routing_msg, &routing))
        return;

    const uint8_t hopsUsed = mp.hop_start >= mp.hop_limit ? (uint8_t)(mp.hop_start - mp.hop_limit) : 0;

    switch (routing.error_reason) {
    case meshtastic_Routing_Error_NONE:
        completeTx(id, MODEM_TX_ACKED, 0, hopsUsed);
        break;
    case meshtastic_Routing_Error_MAX_RETRANSMIT:
        completeTx(id, MODEM_TX_TIMEOUT, (uint8_t)routing.error_reason, hopsUsed);
        break;
    default:
        completeTx(id, MODEM_TX_FAILED, (uint8_t)routing.error_reason, hopsUsed);
        break;
    }
}

void IndustrialModemModule::trackTx(uint32_t id, bool wantAck)
{
    PendingTx *slot = nullptr;
    for (uint8_t i = 0; i < MODEM_MAX_PENDING_TX; i++) {
        if (!pending[i].id) {
            slot = &pending[i];
            break;
        }
    }
    if (!slot) {
        slot = &pending[0];
        for (uint8_t i = 1; i < MODEM_MAX_PENDING_TX; i++)
            if (pending[i].expiresAt < slot->expiresAt)
                slot = &pending[i];
    }

    slot->id = id;
    slot->wantAck = wantAck;
    slot->state = MODEM_TX_QUEUED;
    slot->expiresAt = millis() + (wantAck ? MODEM_T_TX_ACK_MS : MODEM_T_TX_SEND_MS);
    emitTxStatus(id, MODEM_TX_QUEUED, 0, 0);
}

void IndustrialModemModule::completeTx(uint32_t id, uint8_t state, uint8_t err, uint8_t hopsUsed)
{
    for (uint8_t i = 0; i < MODEM_MAX_PENDING_TX; i++) {
        if (pending[i].id != id)
            continue;
        pending[i].id = 0;
        emitTxStatus(id, state, err, hopsUsed);
        return;
    }
}

void IndustrialModemModule::expirePendingTx()
{
    if (!nodeDB)
        return;

    const uint32_t now = millis();
    const uint32_t self = nodeDB->getNodeNum();

    for (uint8_t i = 0; i < MODEM_MAX_PENDING_TX; i++) {
        PendingTx &tx = pending[i];
        if (!tx.id)
            continue;

        if (tx.state == MODEM_TX_QUEUED && RadioLibInterface::instance &&
            !RadioLibInterface::instance->isTxPending(self, tx.id)) {
            tx.state = MODEM_TX_SENT;
            emitTxStatus(tx.id, MODEM_TX_SENT, 0, 0);
            if (!tx.wantAck) {
                tx.id = 0;
                continue;
            }
        }

        if ((int32_t)(now - tx.expiresAt) >= 0) {
            const uint32_t id = tx.id;
            tx.id = 0;
            emitTxStatus(id, MODEM_TX_TIMEOUT, 0, 0);
        }
    }
}

void IndustrialModemModule::emitTxStatus(uint32_t id, uint8_t state, uint8_t err, uint8_t hopsUsed)
{
    ModemTxStatus status = {};
    status.tx_id = id;
    status.state = state;
    status.err = err;
    status.hops_used = hopsUsed;

    uint8_t payload[MODEM_TX_STATUS_SIZE];
    emit(MODEM_EVT_TX_STATUS, payload, modemEncodeTxStatus(payload, sizeof(payload), &status));
}

void IndustrialModemModule::service()
{
    const uint32_t now = millis();

    while (MODEM_UART.available()) {
        uint8_t chunk[64];
        size_t got = 0;
        while (got < sizeof(chunk) && MODEM_UART.available())
            chunk[got++] = (uint8_t)MODEM_UART.read();

        size_t offset = 0;
        while (offset < got) {
            const size_t took = decoder.feed(&chunk[offset], got - offset, now);
            offset += took;
            if (took == 0) {
                // Buffer full: drain before accepting the rest of this chunk.
                while (true) {
                    const ModemFrameResult result = decoder.next(now);
                    if (result == MODEM_FRAME_NONE)
                        break;
                    if (result == MODEM_FRAME_OK)
                        onFrame(decoder.frame());
                }
            }
        }
    }

    while (true) {
        const ModemFrameResult result = decoder.next(now);
        if (result == MODEM_FRAME_NONE)
            return;
        if (result == MODEM_FRAME_OK)
            onFrame(decoder.frame());
    }
}

void IndustrialModemModule::onFrame(const ModemFrameView &frame)
{
    if (modemTypeIsCommand(frame.type))
        onCommand(frame);
    else if (modemTypeIsHostResponse(frame.type))
        onHostResponse(frame);
    // Command responses and modem-initiated frames are ours to send, never to receive.
}

void IndustrialModemModule::onHostResponse(const ModemFrameView &frame)
{
    const uint8_t asked = modemRequestFor(frame.type);
    if (asked != MODEM_QRY_TELEMETRY && asked != MODEM_QRY_POSITION)
        return;
    if (frame.seq != querySeq || !queryPending)
        return;

    queryPending = false;
    if (frame.payloadLen < 1 || frame.payload[0] != MODEM_OK)
        return;

    if (asked == MODEM_QRY_POSITION) {
        ModemPosition fresh;
        if (!modemDecodePosition(&frame.payload[1], (uint16_t)(frame.payloadLen - 1), &fresh))
            return;

        lastPosition = fresh;
        havePosition = true;

        uint8_t encoded[MODEM_MAX_MESH_PAYLOAD];
        answerPendingQuery(encoded, encodePosition(fresh, encoded, sizeof(encoded)));
        return;
    }

    ModemTelemetry answer;
    if (!modemDecodeTelemetry(&frame.payload[1], (uint16_t)(frame.payloadLen - 1), &answer))
        return;
    if (answer.variant != query.variant)
        return;

    uint8_t encoded[MODEM_MAX_MESH_PAYLOAD];
    const uint16_t len = encodeTelemetry(answer, encoded, sizeof(encoded));
    if (!len)
        return;

    cacheTelemetry(answer.variant, encoded, len);
    answerPendingQuery(encoded, len);
}

void IndustrialModemModule::onCommand(const ModemFrameView &frame)
{
    // Every command except HELLO is refused until the handshake completes, so a
    // frame from an unknown peer version can never reach a handler.
    if (!handshaked && frame.type != MODEM_CMD_HELLO) {
        replyStatus(frame.type, frame.seq, MODEM_ERR_NOT_HANDSHAKED);
        return;
    }

    switch (frame.type) {
    case MODEM_CMD_HELLO: {
        ModemHello hello;
        if (!modemDecodeHello(frame.payload, frame.payloadLen, &hello)) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }

        const uint8_t firmware[3] = {2, 7, 27};
        const uint32_t nodeNum = nodeDB ? nodeDB->getNodeNum() : 0;
        const uint16_t len = modemEncodeHelloReply(scratch, sizeof(scratch), MODEM_OK, MODEM_PROTOCOL_VERSION, MODEM_ABI,
                                                   sessionEpoch, nodeNum, firmware, 0);
        handshaked = true;
        reply(frame.type, frame.seq, scratch, len);
        break;
    }

    case MODEM_CMD_GET_MODEM_STATUS: {
        if (frame.payloadLen != 0) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }

        ModemHealth health;
        fillHealth(&health);

        scratch[0] = MODEM_OK;
        const uint16_t len = modemEncodeHealth(&scratch[1], sizeof(scratch) - 1, &health);
        reply(frame.type, frame.seq, scratch, (uint16_t)(1 + len));
        break;
    }

    case MODEM_CMD_SET_REGION_PROFILE: {
        ModemRegionProfile profile;
        if (!modemDecodeRegionProfile(frame.payload, frame.payloadLen, &profile)) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }
        if (profile.region > (uint8_t)_meshtastic_Config_LoRaConfig_RegionCode_MAX) {
            reply(frame.type, frame.seq, scratch, modemEncodeBadParam(scratch, sizeof(scratch), 0));
            return;
        }

        config.lora.region = (meshtastic_Config_LoRaConfig_RegionCode)profile.region;
        config.lora.use_preset = profile.use_preset;
        config.lora.modem_preset = (meshtastic_Config_LoRaConfig_ModemPreset)profile.modem_preset;
        config.lora.channel_num = profile.channel_num;
        config.lora.override_duty_cycle = profile.override_duty_cycle;
        config.lora.override_frequency = (float)profile.override_freq_hz / 1e6f;
        if (!profile.use_preset) {
            config.lora.bandwidth = profile.bandwidth_hz / 1000; // the stack stores kHz
            config.lora.spread_factor = profile.spread_factor;
            config.lora.coding_rate = profile.coding_rate;
        }

        // Answer first: the radio is torn down and rebuilt on the way back up,
        // and the host needs the reply before the port goes quiet.
        replyStatus(frame.type, frame.seq, MODEM_OK);
        MODEM_UART.flush();

        if (::service)
            ::service->reloadConfig(SEGMENT_CONFIG);
        rebootAtMsec = millis() + MODEM_REBOOT_DELAY_MS;
        break;
    }

    case MODEM_CMD_SET_IDENTITY: {
        ModemIdentity identity;
        if (!modemDecodeIdentity(frame.payload, frame.payloadLen, &identity)) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }
        replyStatus(frame.type, frame.seq, setIdentity(identity));
        break;
    }

    case MODEM_CMD_GET_IDENTITY: {
        if (frame.payloadLen != 0) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }

        ModemIdentityInfo info = {};
        info.node_num = nodeDB ? nodeDB->getNodeNum() : 0;
        info.role = (uint8_t)owner.role;
        info.is_unmessagable = owner.has_is_unmessagable && owner.is_unmessagable;
        info.hw_model = (uint8_t)owner.hw_model;

        reply(frame.type, frame.seq,
              scratch, modemEncodeIdentityReply(scratch, sizeof(scratch), MODEM_OK, &info, owner.long_name, owner.short_name));
        break;
    }

    case MODEM_CMD_GET_CONFIG_DIGEST: {
        if (frame.payloadLen != 0) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }

        ModemConfigDigest digest = {};
        digest.region = (uint8_t)config.lora.region;
        digest.modem_preset = (uint8_t)config.lora.modem_preset;
        digest.channel_count = (uint8_t)channels.getNumChannels();
        digest.node_num = nodeDB ? nodeDB->getNodeNum() : 0;
        digest.config_crc32 = configCrc();
        digest.channel_num = (uint16_t)config.lora.channel_num;
        digest.hop_limit = (uint8_t)config.lora.hop_limit;
        digest.tx_power_dbm = (int8_t)config.lora.tx_power;

        reply(frame.type, frame.seq, scratch, modemEncodeConfigDigest(scratch, sizeof(scratch), MODEM_OK, &digest));
        break;
    }

    case MODEM_CMD_GET_MESH_STATUS: {
        if (frame.payloadLen != 0) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }

        ModemMeshStatus status;
        fillMeshStatus(&status);
        reply(frame.type, frame.seq, scratch, modemEncodeMeshStatus(scratch, sizeof(scratch), MODEM_OK, &status));
        break;
    }

    case MODEM_CMD_GET_NODE_LIST:
        replyNodeList(frame);
        break;

    case MODEM_CMD_GET_NODE_INFO:
        replyNodeInfo(frame);
        break;

    case MODEM_CMD_SET_POSITION: {
        ModemPosition position;
        if (!modemDecodePosition(frame.payload, frame.payloadLen, &position)) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }
        replyStatus(frame.type, frame.seq, sendPosition(position));
        break;
    }

    case MODEM_CMD_SEND_TELEMETRY: {
        ModemTelemetry telemetry;
        if (!modemDecodeTelemetry(frame.payload, frame.payloadLen, &telemetry)) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }
        uint32_t txId = 0;
        const uint8_t status = sendTelemetry(telemetry, &txId);
        reply(frame.type, frame.seq, scratch, modemEncodeSendReply(scratch, sizeof(scratch), status, txId));
        break;
    }

    case MODEM_CMD_SEND_BINARY: {
        ModemSendBinary message;
        if (!modemDecodeSendBinary(frame.payload, frame.payloadLen, &message)) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }
        uint32_t txId = 0;
        const uint8_t status = sendBinary(message, &txId);
        reply(frame.type, frame.seq, scratch, modemEncodeSendReply(scratch, sizeof(scratch), status, txId));
        break;
    }

    case MODEM_CMD_SEND_TEXT: {
        ModemSendText message;
        if (!modemDecodeSendText(frame.payload, frame.payloadLen, &message)) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }
        uint32_t txId = 0;
        const uint8_t status = sendText(message, &txId);
        reply(frame.type, frame.seq, scratch, modemEncodeSendReply(scratch, sizeof(scratch), status, txId));
        break;
    }

    case MODEM_CMD_ANNOUNCE: {
        if (frame.payloadLen != 1) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
            return;
        }
        if (!nodeInfoModule) {
            replyStatus(frame.type, frame.seq, MODEM_ERR_BUSY);
            return;
        }
        nodeInfoModule->sendOurNodeInfo(NODENUM_BROADCAST, frame.payload[0] != 0, 0, true);
        replyStatus(frame.type, frame.seq, MODEM_OK);
        break;
    }

    default:
        replyStatus(frame.type, frame.seq, MODEM_ERR_UNKNOWN_TYPE);
        break;
    }
}

void IndustrialModemModule::replyNodeList(const ModemFrameView &frame)
{
    if (frame.payloadLen != 2) {
        replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
        return;
    }
    if (!nodeDB) {
        replyStatus(frame.type, frame.seq, MODEM_ERR_BUSY);
        return;
    }

    const uint8_t offset = frame.payload[0];
    const size_t total = nodeDB->getNumMeshNodes();
    const uint16_t capacity = (uint16_t)((sizeof(scratch) - MODEM_NODE_LIST_HEAD_SIZE) / MODEM_NODE_ENTRY_SIZE);

    uint8_t wanted = frame.payload[1];
    if (wanted == 0 || wanted > capacity)
        wanted = (uint8_t)capacity;

    uint32_t previous = 0;
    bool havePrevious = false;
    for (uint8_t skipped = 0; skipped < offset; skipped++) {
        uint32_t best = 0;
        bool found = false;
        for (size_t i = 0; i < total; i++) {
            const uint32_t num = nodeDB->getMeshNodeByIndex(i)->num;
            if (havePrevious && num <= previous)
                continue;
            if (!found || num < best) {
                best = num;
                found = true;
            }
        }
        if (!found) {
            reply(frame.type, frame.seq, scratch,
                  modemEncodeNodeListHead(scratch, sizeof(scratch), MODEM_OK, 0, (uint8_t)total));
            return;
        }
        previous = best;
        havePrevious = true;
    }

    uint16_t written = modemEncodeNodeListHead(scratch, sizeof(scratch), MODEM_OK, 0, (uint8_t)total);
    uint8_t count = 0;

    while (count < wanted) {
        uint32_t best = 0;
        bool found = false;
        for (size_t i = 0; i < total; i++) {
            const uint32_t num = nodeDB->getMeshNodeByIndex(i)->num;
            if (havePrevious && num <= previous)
                continue;
            if (!found || num < best) {
                best = num;
                found = true;
            }
        }
        if (!found)
            break;

        const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(best);
        if (!node)
            break;

        ModemNodeEntry entry = {};
        entry.node_num = best;
        entry.last_heard = node->last_heard;
        entry.snr_q2 = (int8_t)lroundf(node->snr * 4.0f);
        entry.hops_away = node->has_hops_away ? node->hops_away : 0;
        entry.role = (uint8_t)node->user.role;
        if (node->has_position)
            entry.flags |= 0x01;
        if (node->user.public_key.size)
            entry.flags |= 0x02;
        if (node->is_favorite)
            entry.flags |= 0x04;
        if (best == nodeDB->getNodeNum())
            entry.flags |= 0x08;

        written += modemEncodeNodeEntry(&scratch[written], (uint16_t)(sizeof(scratch) - written), &entry);
        count++;
        previous = best;
        havePrevious = true;
    }

    modemEncodeNodeListHead(scratch, sizeof(scratch), MODEM_OK, count, (uint8_t)total);
    reply(frame.type, frame.seq, scratch, written);
}

void IndustrialModemModule::replyNodeInfo(const ModemFrameView &frame)
{
    uint32_t requested = 0;
    if (!modemDecodeNodeInfoRequest(frame.payload, frame.payloadLen, &requested)) {
        replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_LENGTH);
        return;
    }
    if (!nodeDB) {
        replyStatus(frame.type, frame.seq, MODEM_ERR_BUSY);
        return;
    }

    if (requested == 0)
        requested = nodeDB->getNodeNum();

    const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(requested);
    if (!node) {
        replyStatus(frame.type, frame.seq, MODEM_ERR_BAD_PARAM);
        return;
    }

    ModemNodeInfo info = {};
    info.node_num = node->num;
    info.last_heard = node->last_heard;
    info.snr_q2 = (int8_t)lroundf(node->snr * 4.0f);
    info.hops_away = node->has_hops_away ? node->hops_away : 0;
    info.role = (uint8_t)node->user.role;
    info.battery_level = node->has_device_metrics ? (uint8_t)node->device_metrics.battery_level : 0;
    info.voltage_mv = node->has_device_metrics ? (uint16_t)lroundf(node->device_metrics.voltage * 1000.0f) : 0;
    if (node->has_position) {
        info.flags |= 0x01;
        info.latitude_i = node->position.latitude_i;
        info.longitude_i = node->position.longitude_i;
        info.altitude_m = node->position.altitude;
        info.position_time = node->position.time;
    }
    if (node->user.public_key.size)
        info.flags |= 0x02;
    if (node->is_favorite)
        info.flags |= 0x04;
    if (node->num == nodeDB->getNodeNum())
        info.flags |= 0x08;
    if (node->user.has_is_unmessagable && node->user.is_unmessagable)
        info.flags |= 0x10;

    reply(frame.type, frame.seq, scratch,
          modemEncodeNodeInfo(scratch, sizeof(scratch), MODEM_OK, &info, node->user.long_name, node->user.short_name));
}

uint8_t IndustrialModemModule::setIdentity(const ModemIdentity &in)
{
    if (in.long_name_len >= sizeof(owner.long_name) || in.short_name_len >= sizeof(owner.short_name))
        return MODEM_ERR_BAD_PARAM;
    if (in.role > _meshtastic_Config_DeviceConfig_Role_MAX)
        return MODEM_ERR_BAD_PARAM;

    if (in.long_name_len) {
        memcpy(owner.long_name, in.long_name, in.long_name_len);
        owner.long_name[in.long_name_len] = '\0';
        sanitizeUtf8(owner.long_name, sizeof(owner.long_name));
    }
    if (in.short_name_len) {
        memcpy(owner.short_name, in.short_name, in.short_name_len);
        owner.short_name[in.short_name_len] = '\0';
        sanitizeUtf8(owner.short_name, sizeof(owner.short_name));
    }
    owner.role = (meshtastic_Config_DeviceConfig_Role)in.role;
    owner.has_is_unmessagable = true;
    owner.is_unmessagable = in.is_unmessagable;
    config.device.role = owner.role;

    if (nodeDB) {
        meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
        if (self)
            self->user = TypeConversions::ConvertToUserLite(owner);
        nodeDB->saveToDisk(SEGMENT_DEVICESTATE | SEGMENT_CONFIG);
    }
    if (nodeInfoModule)
        nodeInfoModule->sendOurNodeInfo(NODENUM_BROADCAST, false, 0, true);

    return MODEM_OK;
}

static uint8_t telemetryVariantForTag(pb_size_t tag)
{
    switch (tag) {
    case meshtastic_Telemetry_device_metrics_tag:
        return 1;
    case meshtastic_Telemetry_environment_metrics_tag:
        return 2;
    case meshtastic_Telemetry_power_metrics_tag:
        return 3;
    case meshtastic_Telemetry_air_quality_metrics_tag:
        return 5;
    case meshtastic_Telemetry_host_metrics_tag:
        return 6;
    default:
        return 0;
    }
}

void IndustrialModemModule::cacheTelemetry(uint8_t variant, const uint8_t *payload, uint16_t len)
{
    if (len > sizeof(telemetry[0].payload))
        return;

    TelemetrySlot *slot = nullptr;
    for (uint8_t i = 0; i < MODEM_TELEMETRY_SLOTS; i++) {
        if (telemetry[i].variant == variant || telemetry[i].variant == 0) {
            slot = &telemetry[i];
            break;
        }
    }
    if (!slot)
        return;

    memcpy(slot->payload, payload, len);
    slot->len = len;
    slot->variant = variant;
}

const IndustrialModemModule::TelemetrySlot *IndustrialModemModule::cachedTelemetry(uint8_t variant) const
{
    for (uint8_t i = 0; i < MODEM_TELEMETRY_SLOTS; i++)
        if (telemetry[i].variant == variant && telemetry[i].len)
            return &telemetry[i];
    return nullptr;
}

meshtastic_MeshPacket *IndustrialModemModule::allocReply()
{
    if (!currentRequest || !router || query.active)
        return nullptr;

    const meshtastic_MeshPacket &request = *currentRequest;

    // The reading belongs to the host, so ask it. Answering here would block the
    // receive path for a UART round trip; the reply goes out later, correlated by
    // request id, and the mesh is told to expect nothing from this call.
    if (request.decoded.portnum == meshtastic_PortNum_TELEMETRY_APP)
        return replyToTelemetryRequest(request);
    if (request.decoded.portnum == meshtastic_PortNum_POSITION_APP)
        return replyToPositionRequest(request);
    return nullptr;
}

meshtastic_MeshPacket *IndustrialModemModule::replyToTelemetryRequest(const meshtastic_MeshPacket &request)
{
    meshtastic_Telemetry asked = meshtastic_Telemetry_init_default;
    if (!pb_decode_from_bytes(request.decoded.payload.bytes, request.decoded.payload.size, &meshtastic_Telemetry_msg,
                              &asked))
        return nullptr;

    const uint8_t variant = telemetryVariantForTag(asked.which_variant);
    if (!variant)
        return nullptr;

    askHost(MODEM_QRY_TELEMETRY, request, variant);
    ignoreRequest = true;
    return nullptr;
}

meshtastic_MeshPacket *IndustrialModemModule::replyToPositionRequest(const meshtastic_MeshPacket &request)
{
    askHost(MODEM_QRY_POSITION, request, 0);
    ignoreRequest = true;
    return nullptr;
}

void IndustrialModemModule::askHost(uint8_t queryType, const meshtastic_MeshPacket &request, uint8_t variant)
{
    query.from = getFrom(&request);
    query.requestId = request.id;
    query.channel = request.channel;
    query.type = queryType;
    query.variant = variant;
    query.deadline = millis() + MODEM_T_QUERY_MS;
    query.active = true;

    uint8_t payload[MODEM_QUERY_TELEMETRY_SIZE];
    const uint16_t len = queryType == MODEM_QRY_POSITION
                             ? modemEncodeQueryPosition(payload, sizeof(payload), query.from)
                             : modemEncodeQueryTelemetry(payload, sizeof(payload), query.from, variant);
    querySeq = eventSeq;
    queryPending = true;
    emit(queryType, payload, len);
}

void IndustrialModemModule::answerPendingQuery(const uint8_t *payload, uint16_t len)
{
    if (!query.active)
        return;
    query.active = false;
    queryPending = false;

    if (!router || !::service || !len)
        return;

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;

    p->to = query.from;
    p->channel = query.channel;
    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    p->decoded.request_id = query.requestId;
    memcpy(p->decoded.payload.bytes, payload, len);
    p->decoded.payload.size = len;
    ::service->sendToMesh(p, RX_SRC_LOCAL, true);
}

void IndustrialModemModule::expirePendingQuery()
{
    if (!query.active || (int32_t)(millis() - query.deadline) < 0)
        return;

    if (query.type == MODEM_QRY_POSITION) {
        uint8_t encoded[MODEM_MAX_MESH_PAYLOAD];
        const uint16_t len = havePosition ? encodePosition(lastPosition, encoded, sizeof(encoded)) : 0;
        answerPendingQuery(encoded, len);
        return;
    }

    // Device metrics describe the radio, so the modem can always answer them even
    // when the host declines to add the battery and rail voltage it alone knows.
    if (query.variant == 1) {
        ModemTelemetry own = {};
        own.variant = 1;
        uint8_t encoded[MODEM_MAX_MESH_PAYLOAD];
        answerPendingQuery(encoded, encodeTelemetry(own, encoded, sizeof(encoded)));
        return;
    }

    const TelemetrySlot *slot = cachedTelemetry(query.variant);
    if (slot)
        answerPendingQuery(slot->payload, slot->len);
    else
        answerPendingQuery(nullptr, 0);
}

static bool applyDeviceMetric(meshtastic_DeviceMetrics *out, uint8_t id, float v)
{
    switch (id) {
    case 1:
        out->battery_level = (uint32_t)lroundf(v);
        out->has_battery_level = true;
        return true;
    case 2:
        out->voltage = v;
        out->has_voltage = true;
        return true;
    case 3:
        out->channel_utilization = v;
        out->has_channel_utilization = true;
        return true;
    case 4:
        out->air_util_tx = v;
        out->has_air_util_tx = true;
        return true;
    case 5:
        out->uptime_seconds = (uint32_t)lroundf(v);
        out->has_uptime_seconds = true;
        return true;
    default:
        return false;
    }
}

static bool applyEnvironmentMetric(meshtastic_EnvironmentMetrics *out, uint8_t id, float v)
{
    switch (id) {
    case 1:
        out->temperature = v;
        out->has_temperature = true;
        return true;
    case 2:
        out->relative_humidity = v;
        out->has_relative_humidity = true;
        return true;
    case 3:
        out->barometric_pressure = v;
        out->has_barometric_pressure = true;
        return true;
    case 4:
        out->gas_resistance = v;
        out->has_gas_resistance = true;
        return true;
    case 5:
        out->voltage = v;
        out->has_voltage = true;
        return true;
    case 6:
        out->current = v;
        out->has_current = true;
        return true;
    case 7:
        out->iaq = (uint16_t)lroundf(v);
        out->has_iaq = true;
        return true;
    case 8:
        out->distance = v;
        out->has_distance = true;
        return true;
    case 9:
        out->lux = v;
        out->has_lux = true;
        return true;
    case 10:
        out->white_lux = v;
        out->has_white_lux = true;
        return true;
    case 11:
        out->ir_lux = v;
        out->has_ir_lux = true;
        return true;
    case 12:
        out->uv_lux = v;
        out->has_uv_lux = true;
        return true;
    case 13:
        out->wind_direction = (uint16_t)lroundf(v);
        out->has_wind_direction = true;
        return true;
    case 14:
        out->wind_speed = v;
        out->has_wind_speed = true;
        return true;
    case 15:
        out->weight = v;
        out->has_weight = true;
        return true;
    case 16:
        out->wind_gust = v;
        out->has_wind_gust = true;
        return true;
    case 17:
        out->wind_lull = v;
        out->has_wind_lull = true;
        return true;
    case 18:
        out->radiation = v;
        out->has_radiation = true;
        return true;
    case 19:
        out->rainfall_1h = v;
        out->has_rainfall_1h = true;
        return true;
    case 20:
        out->rainfall_24h = v;
        out->has_rainfall_24h = true;
        return true;
    case 21:
        out->soil_moisture = (uint8_t)lroundf(v);
        out->has_soil_moisture = true;
        return true;
    case 22:
        out->soil_temperature = v;
        out->has_soil_temperature = true;
        return true;
    default:
        return false;
    }
}

static bool applyPowerMetric(meshtastic_PowerMetrics *out, uint8_t id, float v)
{
    if (id < 1 || id > 16)
        return false;

    float *voltages[] = {&out->ch1_voltage, &out->ch2_voltage, &out->ch3_voltage, &out->ch4_voltage,
                         &out->ch5_voltage, &out->ch6_voltage, &out->ch7_voltage, &out->ch8_voltage};
    float *currents[] = {&out->ch1_current, &out->ch2_current, &out->ch3_current, &out->ch4_current,
                         &out->ch5_current, &out->ch6_current, &out->ch7_current, &out->ch8_current};
    bool *hasVoltage[] = {&out->has_ch1_voltage, &out->has_ch2_voltage, &out->has_ch3_voltage, &out->has_ch4_voltage,
                          &out->has_ch5_voltage, &out->has_ch6_voltage, &out->has_ch7_voltage, &out->has_ch8_voltage};
    bool *hasCurrent[] = {&out->has_ch1_current, &out->has_ch2_current, &out->has_ch3_current, &out->has_ch4_current,
                          &out->has_ch5_current, &out->has_ch6_current, &out->has_ch7_current, &out->has_ch8_current};

    const uint8_t channel = (uint8_t)((id - 1) / 2);
    if (id & 1) {
        *voltages[channel] = v;
        *hasVoltage[channel] = true;
    } else {
        *currents[channel] = v;
        *hasCurrent[channel] = true;
    }
    return true;
}

static bool applyAirQualityMetric(meshtastic_AirQualityMetrics *out, uint8_t id, float v)
{
    const uint32_t whole = (uint32_t)lroundf(v);
    switch (id) {
    case 1:
        out->pm10_standard = whole;
        out->has_pm10_standard = true;
        return true;
    case 2:
        out->pm25_standard = whole;
        out->has_pm25_standard = true;
        return true;
    case 3:
        out->pm40_standard = whole;
        out->has_pm40_standard = true;
        return true;
    case 4:
        out->pm100_standard = whole;
        out->has_pm100_standard = true;
        return true;
    case 5:
        out->pm10_environmental = whole;
        out->has_pm10_environmental = true;
        return true;
    case 6:
        out->pm25_environmental = whole;
        out->has_pm25_environmental = true;
        return true;
    case 7:
        out->pm100_environmental = whole;
        out->has_pm100_environmental = true;
        return true;
    case 8:
        out->particles_03um = whole;
        out->has_particles_03um = true;
        return true;
    case 9:
        out->particles_05um = whole;
        out->has_particles_05um = true;
        return true;
    case 10:
        out->particles_10um = whole;
        out->has_particles_10um = true;
        return true;
    case 11:
        out->particles_25um = whole;
        out->has_particles_25um = true;
        return true;
    case 12:
        out->particles_40um = whole;
        out->has_particles_40um = true;
        return true;
    case 13:
        out->particles_50um = whole;
        out->has_particles_50um = true;
        return true;
    case 14:
        out->particles_100um = whole;
        out->has_particles_100um = true;
        return true;
    case 15:
        out->particles_tps = v;
        out->has_particles_tps = true;
        return true;
    case 16:
        out->co2 = whole;
        out->has_co2 = true;
        return true;
    case 17:
        out->co2_temperature = v;
        out->has_co2_temperature = true;
        return true;
    case 18:
        out->co2_humidity = v;
        out->has_co2_humidity = true;
        return true;
    case 19:
        out->form_formaldehyde = v;
        out->has_form_formaldehyde = true;
        return true;
    case 20:
        out->form_temperature = v;
        out->has_form_temperature = true;
        return true;
    case 21:
        out->form_humidity = v;
        out->has_form_humidity = true;
        return true;
    case 22:
        out->pm_temperature = v;
        out->has_pm_temperature = true;
        return true;
    case 23:
        out->pm_humidity = v;
        out->has_pm_humidity = true;
        return true;
    case 24:
        out->pm_voc_idx = v;
        out->has_pm_voc_idx = true;
        return true;
    case 25:
        out->pm_nox_idx = v;
        out->has_pm_nox_idx = true;
        return true;
    default:
        return false;
    }
}

static bool applyHostMetric(meshtastic_HostMetrics *out, uint8_t id, float v)
{
    switch (id) {
    case 1:
        out->uptime_seconds = (uint32_t)llroundf(v);
        return true;
    case 2:
        out->freemem_bytes = (uint64_t)llroundf(v);
        return true;
    case 3:
        out->diskfree1_bytes = (uint64_t)llroundf(v);
        return true;
    case 4:
        out->diskfree2_bytes = (uint64_t)llroundf(v);
        out->has_diskfree2_bytes = true;
        return true;
    case 5:
        out->diskfree3_bytes = (uint64_t)llroundf(v);
        out->has_diskfree3_bytes = true;
        return true;
    case 6:
        out->load1 = (uint16_t)lroundf(v * 100.0f);
        return true;
    case 7:
        out->load5 = (uint16_t)lroundf(v * 100.0f);
        return true;
    case 8:
        out->load15 = (uint16_t)lroundf(v * 100.0f);
        return true;
    default:
        return false;
    }
}

uint16_t IndustrialModemModule::encodeTelemetry(const ModemTelemetry &in, uint8_t *out, uint16_t cap)
{
    meshtastic_Telemetry telemetry = meshtastic_Telemetry_init_default;
    switch (in.variant) {
    case 1:
        telemetry.which_variant = meshtastic_Telemetry_device_metrics_tag;
        break;
    case 2:
        telemetry.which_variant = meshtastic_Telemetry_environment_metrics_tag;
        break;
    case 3:
        telemetry.which_variant = meshtastic_Telemetry_power_metrics_tag;
        break;
    case 5:
        telemetry.which_variant = meshtastic_Telemetry_air_quality_metrics_tag;
        break;
    case 6:
        telemetry.which_variant = meshtastic_Telemetry_host_metrics_tag;
        break;
    default:
        return 0;
    }

    if (in.variant == 1) {
        meshtastic_DeviceMetrics &device = telemetry.variant.device_metrics;
        device.uptime_seconds = millis() / 1000u;
        device.has_uptime_seconds = true;
        if (airTime) {
            device.channel_utilization = airTime->channelUtilizationPercent();
            device.has_channel_utilization = true;
            device.air_util_tx = airTime->utilizationTXPercent();
            device.has_air_util_tx = true;
        }
    }

    if (in.text_len && in.variant != 6)
        return 0;
    if (in.text_len >= sizeof(telemetry.variant.host_metrics.user_string))
        return 0;
    if (in.text_len && !modemIsValidUtf8(in.utf8, in.text_len))
        return 0;

    for (uint8_t i = 0; i < in.metric_count; i++) {
        ModemMetric metric;
        if (!modemTelemetryMetric(&in, i, &metric))
            return 0;

        const float value = (float)metric.value * powf(10.0f, (float)metric.scale10);
        bool applied = false;
        switch (in.variant) {
        case 1:
            applied = applyDeviceMetric(&telemetry.variant.device_metrics, metric.metric_id, value);
            break;
        case 2:
            applied = applyEnvironmentMetric(&telemetry.variant.environment_metrics, metric.metric_id, value);
            break;
        case 3:
            applied = applyPowerMetric(&telemetry.variant.power_metrics, metric.metric_id, value);
            break;
        case 5:
            applied = applyAirQualityMetric(&telemetry.variant.air_quality_metrics, metric.metric_id, value);
            break;
        default:
            applied = applyHostMetric(&telemetry.variant.host_metrics, metric.metric_id, value);
            break;
        }
        if (!applied)
            return 0;
    }

    if (in.text_len) {
        memcpy(telemetry.variant.host_metrics.user_string, in.utf8, in.text_len);
        telemetry.variant.host_metrics.user_string[in.text_len] = '\0';
        telemetry.variant.host_metrics.has_user_string = true;
    }

    telemetry.time = getValidTime(RTCQualityFromNet);

    return (uint16_t)pb_encode_to_bytes(out, cap, &meshtastic_Telemetry_msg, &telemetry);
}

uint8_t IndustrialModemModule::sendTelemetry(const ModemTelemetry &in, uint32_t *outTxId)
{
    if (!router || !::service)
        return MODEM_ERR_BUSY;

    uint8_t encoded[MODEM_MAX_MESH_PAYLOAD];
    const uint16_t len = encodeTelemetry(in, encoded, sizeof(encoded));
    if (!len)
        return MODEM_ERR_BAD_PARAM;

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return MODEM_ERR_TX_QUEUE_FULL;

    p->to = NODENUM_BROADCAST;
    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    memcpy(p->decoded.payload.bytes, encoded, len);
    p->decoded.payload.size = len;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    cacheTelemetry(in.variant, encoded, len);

    *outTxId = p->id;
    trackTx(p->id, false);
    ::service->sendToMesh(p, RX_SRC_LOCAL, true);
    return MODEM_OK;
}

static bool portnumIsReservedForModem(uint16_t portnum)
{
    switch (portnum) {
    case meshtastic_PortNum_TEXT_MESSAGE_APP:
    case meshtastic_PortNum_POSITION_APP:
    case meshtastic_PortNum_NODEINFO_APP:
    case meshtastic_PortNum_ROUTING_APP:
    case meshtastic_PortNum_ADMIN_APP:
    case meshtastic_PortNum_TELEMETRY_APP:
    case meshtastic_PortNum_TRACEROUTE_APP:
        return true;
    default:
        return false;
    }
}

uint8_t IndustrialModemModule::sendPayload(uint32_t dest, uint16_t portnum, uint8_t flags, uint8_t hopLimit,
                                           uint8_t channelIndex, const uint8_t *payload, uint16_t payloadLen,
                                           uint32_t *outTxId)
{
    if (!router || !::service || !nodeDB)
        return MODEM_ERR_BUSY;

    const bool wantAck = (flags & MODEM_SEND_WANT_ACK) != 0;
    const bool wantResponse = (flags & MODEM_SEND_WANT_RESPONSE) != 0;
    const bool pki = (flags & MODEM_SEND_PKI) != 0;
    const bool broadcast = dest == NODENUM_BROADCAST;

    if (portnum == 0 || portnum > _meshtastic_PortNum_MAX)
        return MODEM_ERR_BAD_PARAM;
    if (channelIndex >= channels.getNumChannels())
        return MODEM_ERR_BAD_PARAM;
    if (broadcast && (wantAck || wantResponse))
        return MODEM_ERR_BAD_PARAM;

    const uint16_t ceiling = pki ? MODEM_MAX_MESH_PAYLOAD - MODEM_PKI_OVERHEAD : MODEM_MAX_MESH_PAYLOAD;
    if (payloadLen > ceiling)
        return MODEM_ERR_BAD_PARAM;

    if (pki) {
        if (broadcast || config.security.private_key.size != 32)
            return MODEM_ERR_NOT_SUPPORTED;
        const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(dest);
        if (!node || node->user.public_key.size != 32)
            return MODEM_ERR_BAD_PARAM;
    }

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return MODEM_ERR_TX_QUEUE_FULL;

    p->to = dest;
    p->channel = channelIndex;
    p->want_ack = wantAck;
    p->pki_encrypted = pki;
    p->decoded.portnum = (meshtastic_PortNum)portnum;
    p->decoded.want_response = wantResponse;
    p->decoded.payload.size = payloadLen;
    memcpy(p->decoded.payload.bytes, payload, payloadLen);
    if (hopLimit)
        p->hop_limit = hopLimit;
    p->priority = wantAck ? meshtastic_MeshPacket_Priority_RELIABLE : meshtastic_MeshPacket_Priority_DEFAULT;

    *outTxId = p->id;
    trackTx(p->id, wantAck);
    ::service->sendToMesh(p, RX_SRC_LOCAL, true);
    return MODEM_OK;
}

uint8_t IndustrialModemModule::sendBinary(const ModemSendBinary &in, uint32_t *outTxId)
{
    if (portnumIsReservedForModem(in.portnum))
        return MODEM_ERR_NOT_SUPPORTED;

    return sendPayload(in.dest_node, in.portnum, in.flags, in.hop_limit, in.channel_index, in.payload, in.payload_len,
                       outTxId);
}

uint8_t IndustrialModemModule::sendText(const ModemSendText &in, uint32_t *outTxId)
{
    if (!modemIsValidUtf8(in.utf8, in.text_len))
        return MODEM_ERR_BAD_PARAM;

    return sendPayload(in.dest_node, meshtastic_PortNum_TEXT_MESSAGE_APP, in.flags, 0, in.channel_index, in.utf8,
                       in.text_len, outTxId);
}

meshtastic_Position IndustrialModemModule::buildPosition(const ModemPosition &in)
{
    meshtastic_Position position = meshtastic_Position_init_default;
    position.latitude_i = in.latitude_i;
    position.has_latitude_i = true;
    position.longitude_i = in.longitude_i;
    position.has_longitude_i = true;
    position.altitude = in.altitude_m;
    position.has_altitude = true;
    position.time = in.timestamp_unix;
    position.timestamp = in.timestamp_unix;
    position.altitude_hae = in.altitude_hae;
    position.has_altitude_hae = in.altitude_hae != 0;
    position.ground_speed = in.ground_speed_mmps * 36u / 10000u;
    position.has_ground_speed = in.ground_speed_mmps != 0;
    position.ground_track = in.ground_track;
    position.has_ground_track = in.ground_track != 0;
    position.HDOP = in.hdop;
    position.PDOP = in.pdop;
    position.gps_accuracy = in.gps_accuracy_mm;
    position.fix_type = in.fix_type;
    position.location_source = (meshtastic_Position_LocSource)in.loc_source;
    position.altitude_source = (meshtastic_Position_AltSource)in.alt_source;
    position.precision_bits = in.precision_bits ? in.precision_bits : 32;
    position.sats_in_view = in.sats_in_view;
    position.seq_number = ++positionSeq;
    return position;
}

uint16_t IndustrialModemModule::encodePosition(const ModemPosition &in, uint8_t *out, uint16_t cap)
{
    meshtastic_Position position = buildPosition(in);
    return (uint16_t)pb_encode_to_bytes(out, cap, &meshtastic_Position_msg, &position);
}

uint8_t IndustrialModemModule::sendPosition(const ModemPosition &in)
{
    if (!router || !::service || !nodeDB)
        return MODEM_ERR_BUSY;

    meshtastic_Position position = buildPosition(in);

    lastPosition = in;
    havePosition = true;

    nodeDB->setLocalPosition(position);

    meshtastic_NodeInfoLite *self = ::service->refreshLocalMeshNode();
    if (self) {
        self->position = TypeConversions::ConvertToPositionLite(position);
        self->has_position = true;
    }

    uint8_t encoded[MODEM_MAX_MESH_PAYLOAD];
    const uint16_t len = (uint16_t)pb_encode_to_bytes(encoded, sizeof(encoded), &meshtastic_Position_msg, &position);
    if (!len)
        return MODEM_ERR_BAD_PARAM;

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return MODEM_ERR_TX_QUEUE_FULL;

    p->to = NODENUM_BROADCAST;
    p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
    memcpy(p->decoded.payload.bytes, encoded, len);
    p->decoded.payload.size = len;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    ::service->sendToMesh(p, RX_SRC_LOCAL, true);
    return MODEM_OK;
}

void IndustrialModemModule::fillMeshStatus(ModemMeshStatus *out) const
{
    memset(out, 0, sizeof(*out));

    if (airTime) {
        out->channel_util_pct = (uint8_t)airTime->channelUtilizationPercent();
        out->air_util_tx_pct = (uint8_t)airTime->utilizationTXPercent();
    }
    if (router) {
        const meshtastic_QueueStatus queue = router->getQueueStatus();
        out->tx_queue_depth = (uint16_t)(queue.maxlen - queue.free);
    }
    if (RadioLibInterface::instance) {
        out->rx_total = RadioLibInterface::instance->rxGood + RadioLibInterface::instance->rxBad;
        out->rx_decrypt_failed = RadioLibInterface::instance->rxBad;
        out->tx_total = RadioLibInterface::instance->txGood;
    }
    // rx_queue_depth, rx_dropped_policy, rx_dropped_rate_limit and tx_failed stay
    // zero: the receive filter that would feed them is not implemented yet.
}

/// A cheap fingerprint of the settings the host can push, so it can tell whether
/// what it sent survived a reboot without reading every field back.
uint32_t IndustrialModemModule::configCrc() const
{
    uint32_t crc = 2166136261u; // FNV-1a
    const uint8_t bytes[] = {
        (uint8_t)config.lora.region,        (uint8_t)config.lora.modem_preset,
        (uint8_t)config.lora.use_preset,    (uint8_t)config.lora.channel_num,
        (uint8_t)config.lora.hop_limit,     (uint8_t)config.lora.tx_power,
        (uint8_t)config.lora.bandwidth,     (uint8_t)config.lora.spread_factor,
        (uint8_t)config.lora.coding_rate,   (uint8_t)config.lora.override_duty_cycle,
        (uint8_t)config.device.role,        (uint8_t)channels.getNumChannels(),
    };
    for (uint8_t b : bytes) {
        crc ^= b;
        crc *= 16777619u;
    }
    return crc;
}

void IndustrialModemModule::fillHealth(ModemHealth *out) const
{
    memset(out, 0, sizeof(*out));
    out->session_epoch = sessionEpoch;
    out->uptime_s = millis() / 1000;
    out->state = MODEM_STATE_NORMAL;

    // setupModules() runs before airTime exists, and the host is expected to ask
    // for status the moment it sees MODEM_REBOOTED - so both of these can still
    // be null on the first few polls.
    if (nodeDB)
        out->node_count = (uint16_t)nodeDB->getNumMeshNodes();
    if (airTime) {
        out->channel_util_pct = (uint8_t)airTime->channelUtilizationPercent();
        out->air_util_tx_pct = (uint8_t)airTime->utilizationTXPercent();
    }
}

void IndustrialModemModule::reply(uint8_t command, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[MODEM_MAX_FRAME_SIZE];
    const size_t written = modemFrameEncode(frame, sizeof(frame), modemResponseFor(command), seq, payload, len);
    if (written)
        MODEM_UART.write(frame, written);
}

void IndustrialModemModule::replyStatus(uint8_t command, uint8_t seq, uint8_t status)
{
    reply(command, seq, &status, 1);
}

void IndustrialModemModule::emit(uint8_t event, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[MODEM_MAX_FRAME_SIZE];
    const size_t written = modemFrameEncode(frame, sizeof(frame), event, eventSeq, payload, len);
    eventSeq++;
    if (written)
        MODEM_UART.write(frame, written);
}
