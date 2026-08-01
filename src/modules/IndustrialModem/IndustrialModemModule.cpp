#include "IndustrialModemModule.h"

#include "Channels.h"
#include "MeshService.h"
#include "RadioLibInterface.h"
#include "Router.h"
#include "NodeDB.h"
#include "airtime.h"
#include "modules/NodeInfoModule.h"
#include "main.h"
#include "TypeConversions.h"
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

IndustrialModemModule::IndustrialModemModule() : concurrency::OSThread("IndustrialModem") {}

int32_t IndustrialModemModule::runOnce()
{
    if (!started) {
        MODEM_UART.begin(MODEM_BAUD);
        decoder.reset();
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
    return MODEM_SERVICE_MS;
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
    if (modemTypeIsCommand(frame.type) || frame.type == MODEM_CMD_EVENT_ACK)
        onCommand(frame);
    // Responses and events are the modem's to send, never to receive.
}

void IndustrialModemModule::onCommand(const ModemFrameView &frame)
{
    if (frame.type == MODEM_CMD_EVENT_ACK)
        return; // No data events are emitted yet, so nothing to retire.

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

uint8_t IndustrialModemModule::sendPosition(const ModemPosition &in)
{
    if (!router || !::service || !nodeDB)
        return MODEM_ERR_BUSY;

    meshtastic_Position position = meshtastic_Position_init_default;
    position.latitude_i = in.latitude_i;
    position.has_latitude_i = true;
    position.longitude_i = in.longitude_i;
    position.has_longitude_i = true;
    position.altitude = in.altitude_m;
    position.has_altitude = true;
    position.time = in.timestamp_unix;
    position.timestamp = in.timestamp_unix;
    position.location_source = meshtastic_Position_LocSource_LOC_MANUAL;
    position.precision_bits = in.precision_bits ? in.precision_bits : 32;
    position.sats_in_view = in.sats_in_view;

    nodeDB->setLocalPosition(position);

    meshtastic_NodeInfoLite *self = ::service->refreshLocalMeshNode();
    if (self) {
        self->position = TypeConversions::ConvertToPositionLite(position);
        self->has_position = true;
    }

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return MODEM_ERR_TX_QUEUE_FULL;

    p->to = NODENUM_BROADCAST;
    p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
    p->decoded.payload.size = pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes),
                                                 &meshtastic_Position_msg, &position);
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
