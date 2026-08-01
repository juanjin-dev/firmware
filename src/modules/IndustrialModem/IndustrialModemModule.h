#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"
#include "modem/ModemCodec.h"
#include "modem/ModemFrame.h"
#include <Arduino.h>

/**
 * Serves the meshkit wire protocol on UART1.
 *
 * This is the only place the protocol meets Meshtastic. Everything under
 * src/modem/ is deliberately free of mesh types so a host implementation can be
 * written from the specification alone; this class is the adapter that turns
 * decoded requests into calls on the stack.
 */
class IndustrialModemModule : private concurrency::OSThread
{
  public:
    IndustrialModemModule();

  protected:
    virtual int32_t runOnce() override;

  private:
    void service();
    void onFrame(const ModemFrameView &frame);
    void onCommand(const ModemFrameView &frame);

    void reply(uint8_t command, uint8_t seq, const uint8_t *payload, uint16_t len);
    void replyStatus(uint8_t command, uint8_t seq, uint8_t status);
    void emit(uint8_t event, const uint8_t *payload, uint16_t len);

    void fillHealth(ModemHealth *out) const;
    uint8_t sendPosition(const ModemPosition &in);
    uint8_t setIdentity(const ModemIdentity &in);
    void fillMeshStatus(ModemMeshStatus *out) const;
    uint32_t configCrc() const;

    void replyNodeList(const ModemFrameView &frame);
    void replyNodeInfo(const ModemFrameView &frame);

    bool started = false;
    bool handshaked = false;
    uint32_t sessionEpoch = 0;
    uint8_t eventSeq = 0;

    ModemFrameDecoder decoder;
    uint8_t scratch[MODEM_MAX_FRAME_SIZE];
};

extern IndustrialModemModule *industrialModemModule;
