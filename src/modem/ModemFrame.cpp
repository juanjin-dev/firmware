#include "ModemFrame.h"

#include <string.h>

#define MODEM_CRC_POLY 0x8408
#define MODEM_CRC_INIT 0xFFFF

uint16_t modemCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = MODEM_CRC_INIT;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ MODEM_CRC_POLY) : (uint16_t)(crc >> 1);
        }
    }

    return (uint16_t)~crc;
}

size_t modemFrameEncode(uint8_t *out, size_t outSize, uint8_t type, uint8_t seq, const uint8_t *payload, uint16_t payloadLen)
{
    if (payloadLen > MODEM_MAX_PAYLOAD)
        return 0;
    if (payloadLen > 0 && payload == NULL)
        return 0;

    const size_t total = MODEM_FRAME_OVERHEAD + payloadLen;
    if (out == NULL || outSize < total)
        return 0;

    out[0] = MODEM_SOF;
    out[1] = (uint8_t)(payloadLen & 0xFF);
    out[2] = (uint8_t)(payloadLen >> 8);
    out[3] = type;
    out[4] = seq;

    if (payloadLen > 0)
        memcpy(&out[MODEM_HEADER_SIZE], payload, payloadLen);

    const uint16_t crc = modemCrc16(&out[1], (size_t)(MODEM_HEADER_SIZE - 1) + payloadLen);
    out[MODEM_HEADER_SIZE + payloadLen] = (uint8_t)(crc & 0xFF);
    out[MODEM_HEADER_SIZE + payloadLen + 1] = (uint8_t)(crc >> 8);

    return total;
}

void ModemFrameDecoder::reset()
{
    used = 0;
    consume = 0;
    lastByteMs = 0;
    view = ModemFrameView();
}

size_t ModemFrameDecoder::feed(const uint8_t *data, size_t len, uint32_t nowMs)
{
    if (data == NULL || len == 0)
        return 0;

    const size_t room = sizeof(buf) - used;
    const size_t take = len < room ? len : room;

    if (take > 0) {
        memcpy(&buf[used], data, take);
        used += take;
        lastByteMs = nowMs;
    }

    return take;
}

void ModemFrameDecoder::discardLeadingByte()
{
    if (used > 0) {
        memmove(buf, &buf[1], used - 1);
        used--;
    }
}

bool ModemFrameDecoder::findSof()
{
    size_t i = 0;
    while (i < used && buf[i] != MODEM_SOF)
        i++;

    if (i > 0) {
        memmove(buf, &buf[i], used - i);
        used -= i;
    }

    return used > 0;
}

ModemFrameResult ModemFrameDecoder::next(uint32_t nowMs)
{
    if (consume > 0) {
        memmove(buf, &buf[consume], used - consume);
        used -= consume;
        consume = 0;
    }

    if (!findSof())
        return MODEM_FRAME_NONE;

    const bool stalled = (uint32_t)(nowMs - lastByteMs) > MODEM_T_FRAME_MS;

    if (used < MODEM_HEADER_SIZE) {
        if (stalled) {
            discardLeadingByte();
            return MODEM_FRAME_ERR_TIMEOUT;
        }
        return MODEM_FRAME_NONE;
    }

    const uint16_t payloadLen = (uint16_t)(buf[1] | ((uint16_t)buf[2] << 8));
    if (payloadLen > MODEM_MAX_PAYLOAD) {
        discardLeadingByte();
        return MODEM_FRAME_ERR_OVERSIZE;
    }

    const size_t total = MODEM_FRAME_OVERHEAD + payloadLen;
    if (used < total) {
        if (stalled) {
            discardLeadingByte();
            return MODEM_FRAME_ERR_TIMEOUT;
        }
        return MODEM_FRAME_NONE;
    }

    const uint16_t want = modemCrc16(&buf[1], (size_t)(MODEM_HEADER_SIZE - 1) + payloadLen);
    const uint16_t got = (uint16_t)(buf[total - 2] | ((uint16_t)buf[total - 1] << 8));

    if (want != got) {
        discardLeadingByte();
        return MODEM_FRAME_ERR_CRC;
    }

    view.type = buf[3];
    view.seq = buf[4];
    view.payload = &buf[MODEM_HEADER_SIZE];
    view.payloadLen = payloadLen;
    consume = total;

    return MODEM_FRAME_OK;
}
