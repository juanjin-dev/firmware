#pragma once

#include "ModemProtocol.h"

uint16_t modemCrc16(const uint8_t *data, size_t len);

size_t modemFrameEncode(uint8_t *out, size_t outSize, uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t payloadLen);

typedef enum {
    MODEM_FRAME_NONE,
    MODEM_FRAME_OK,
    MODEM_FRAME_ERR_CRC,
    MODEM_FRAME_ERR_OVERSIZE,
    MODEM_FRAME_ERR_TIMEOUT,
} ModemFrameResult;

struct ModemFrameView {
    uint8_t type;
    uint8_t seq;
    const uint8_t *payload;
    uint16_t payloadLen;
};

class ModemFrameDecoder
{
  public:
    void reset();

    size_t feed(const uint8_t *data, size_t len, uint32_t nowMs);

    ModemFrameResult next(uint32_t nowMs);

    const ModemFrameView &frame() const { return view; }

    size_t pending() const { return used; }

  private:
    void discardLeadingByte();
    bool findSof();

    uint8_t buf[MODEM_MAX_FRAME_SIZE] = {};
    size_t used = 0;
    size_t consume = 0;
    uint32_t lastByteMs = 0;
    ModemFrameView view = {};
};
