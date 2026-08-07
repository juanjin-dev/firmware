#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"
#include "mesh/MeshModule.h"
#include "modem/ModemCodec.h"
#include "modem/ModemFrame.h"
#include <Arduino.h>

#define MODEM_MAX_PENDING_TX 8
#define MODEM_TELEMETRY_SLOTS 5

/**
 * Serves the meshkit wire protocol on UART1.
 *
 * This is the only place the protocol meets Meshtastic. Everything under
 * src/modem/ is deliberately free of mesh types so a host implementation can be
 * written from the specification alone; this class is the adapter that turns
 * decoded requests into calls on the stack.
 */
class IndustrialModemModule : public MeshModule, private concurrency::OSThread
{
  public:
    IndustrialModemModule();

  protected:
    virtual int32_t runOnce() override;
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override;
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    virtual meshtastic_MeshPacket *allocReply() override;

  private:
    struct TelemetrySlot {
        uint8_t payload[MODEM_MAX_MESH_PAYLOAD];
        uint16_t len;
        uint8_t variant;
    };

    struct PendingQuery {
        uint32_t from;
        uint32_t requestId;
        uint32_t deadline;
        uint8_t channel;
        uint8_t type;
        uint8_t variant;
        bool active;
    };

    struct PendingTx {
        uint32_t id;
        uint32_t expiresAt;
        uint8_t state;
        bool wantAck;
    };

    void trackTx(uint32_t id, bool wantAck);
    void completeTx(uint32_t id, uint8_t state, uint8_t err, uint8_t hopsUsed);
    void expirePendingTx();
    void emitTxStatus(uint32_t id, uint8_t state, uint8_t err, uint8_t hopsUsed);
    void onRouting(const meshtastic_MeshPacket &mp);
    void recordPeerTelemetry(const meshtastic_MeshPacket &mp);
    void cacheTelemetry(uint8_t variant, const uint8_t *payload, uint16_t len);
    const TelemetrySlot *cachedTelemetry(uint8_t variant) const;
    void askHost(uint8_t queryType, const meshtastic_MeshPacket &request, uint8_t variant);
    meshtastic_MeshPacket *replyToTelemetryRequest(const meshtastic_MeshPacket &request);
    meshtastic_MeshPacket *replyToPositionRequest(const meshtastic_MeshPacket &request);
    void answerPendingQuery(const uint8_t *payload, uint16_t len);
    void expirePendingQuery();

    void service();
    void onFrame(const ModemFrameView &frame);
    void onCommand(const ModemFrameView &frame);
    void onHostResponse(const ModemFrameView &frame);

    void reply(uint8_t command, uint8_t seq, const uint8_t *payload, uint16_t len);
    void replyStatus(uint8_t command, uint8_t seq, uint8_t status);
    void emit(uint8_t event, const uint8_t *payload, uint16_t len);

    void fillHealth(ModemHealth *out) const;
    uint8_t sendPosition(const ModemPosition &in);
    meshtastic_Position buildPosition(const ModemPosition &in);
    uint16_t encodePosition(const ModemPosition &in, uint8_t *out, uint16_t cap);
    uint8_t sendTelemetry(const ModemTelemetry &in, uint32_t *outTxId);
    uint16_t encodeTelemetry(const ModemTelemetry &in, uint8_t *out, uint16_t cap);
    uint8_t sendBinary(const ModemSendBinary &in, uint32_t *outTxId);
    uint8_t sendText(const ModemSendText &in, uint32_t *outTxId);
    uint8_t sendPayload(uint32_t dest, uint16_t portnum, uint8_t flags, uint8_t hopLimit, uint8_t channelIndex,
                        const uint8_t *payload, uint16_t payloadLen, uint32_t *outTxId);
    uint8_t setIdentity(const ModemIdentity &in);
    void fillMeshStatus(ModemMeshStatus *out) const;
    uint32_t configCrc() const;

    void replyNodeList(const ModemFrameView &frame);
    void replyNodeInfo(const ModemFrameView &frame);

    bool started = false;
    bool handshaked = false;
    uint32_t sessionEpoch = 0;
    uint8_t eventSeq = 0;
    uint32_t positionSeq = 0;
    uint8_t querySeq = 0;
    bool queryPending = false;

    ModemFrameDecoder decoder;
    PendingTx pending[MODEM_MAX_PENDING_TX] = {};
    TelemetrySlot telemetry[MODEM_TELEMETRY_SLOTS] = {};
    PendingQuery query = {};
    ModemPosition lastPosition = {};
    bool havePosition = false;
    uint8_t scratch[MODEM_MAX_FRAME_SIZE];
};

extern IndustrialModemModule *industrialModemModule;
