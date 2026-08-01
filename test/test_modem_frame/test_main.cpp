#include "TestUtil.h"
#include <Arduino.h>
#include <unity.h>

#include "modem/ModemFrame.h"
#include <cstdio>
#include <cstring>
#include <vector>

static constexpr uint64_t BASE_SEED = 0x000D3E11ULL;
static constexpr unsigned FUZZ_ITERS = 3000;

static uint64_t rngState = BASE_SEED;

static inline void rngSeed(uint64_t s)
{
    rngState = s ? s : 1;
}

static inline uint32_t rngNext()
{
    rngState = rngState * 6364136223846793005ULL + 1442695040888963407ULL;
    return (uint32_t)(rngState >> 33);
}

static inline uint8_t rngByte()
{
    return (uint8_t)(rngNext() & 0xFF);
}

static inline uint32_t rngRange(uint32_t n)
{
    return n ? (rngNext() % n) : 0;
}

static std::vector<uint8_t> encodeOk(uint8_t type, uint8_t seq, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> out(MODEM_MAX_FRAME_SIZE);
    const size_t n =
        modemFrameEncode(out.data(), out.size(), type, seq, payload.empty() ? NULL : payload.data(), (uint16_t)payload.size());
    TEST_ASSERT_EQUAL_size_t(MODEM_FRAME_OVERHEAD + payload.size(), n);
    out.resize(n);
    return out;
}

static void feedAll(ModemFrameDecoder &d, const std::vector<uint8_t> &bytes, uint32_t nowMs)
{
    size_t off = 0;
    while (off < bytes.size()) {
        const size_t took = d.feed(&bytes[off], bytes.size() - off, nowMs);
        TEST_ASSERT_GREATER_THAN_size_t(0, took);
        off += took;
    }
}

// ---------------------------------------------------------------------------
// Group F1 - encoding and CRC

static void test_F1a_crc_check_vector()
{
    const uint8_t v[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x906E, modemCrc16(v, sizeof(v)));
}

static void test_F1b_encode_layout()
{
    const std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF};
    const std::vector<uint8_t> f = encodeOk(MODEM_CMD_SEND_BINARY, 0x42, payload);

    TEST_ASSERT_EQUAL_HEX8(MODEM_SOF, f[0]);
    TEST_ASSERT_EQUAL_HEX8(0x04, f[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, f[2]);
    TEST_ASSERT_EQUAL_HEX8(MODEM_CMD_SEND_BINARY, f[3]);
    TEST_ASSERT_EQUAL_HEX8(0x42, f[4]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload.data(), &f[MODEM_HEADER_SIZE], payload.size());

    const uint16_t crc = modemCrc16(&f[1], MODEM_HEADER_SIZE - 1 + payload.size());
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(crc & 0xFF), f[f.size() - 2]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(crc >> 8), f[f.size() - 1]);
}

static void test_F1c_encode_rejects_bad_args()
{
    uint8_t out[MODEM_MAX_FRAME_SIZE];
    uint8_t payload[8] = {0};

    TEST_ASSERT_EQUAL_size_t(0, modemFrameEncode(out, sizeof(out), 1, 0, payload, MODEM_MAX_PAYLOAD + 1));
    TEST_ASSERT_EQUAL_size_t(0, modemFrameEncode(out, MODEM_FRAME_OVERHEAD, 1, 0, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_size_t(0, modemFrameEncode(out, sizeof(out), 1, 0, NULL, 4));
    TEST_ASSERT_EQUAL_size_t(MODEM_FRAME_OVERHEAD, modemFrameEncode(out, sizeof(out), 1, 0, NULL, 0));
}

static void test_F1d_roundtrip_all_lengths()
{
    const uint16_t lengths[] = {0, 1, 2, 7, 64, 233, 511, MODEM_MAX_PAYLOAD};

    for (uint16_t len : lengths) {
        std::vector<uint8_t> payload(len);
        for (uint16_t i = 0; i < len; i++)
            payload[i] = (uint8_t)(i * 31 + len);

        ModemFrameDecoder d;
        d.reset();
        feedAll(d, encodeOk(MODEM_EVT_RX_DATA, (uint8_t)len, payload), 1000);

        TEST_ASSERT_EQUAL(MODEM_FRAME_OK, d.next(1000));
        TEST_ASSERT_EQUAL_HEX8(MODEM_EVT_RX_DATA, d.frame().type);
        TEST_ASSERT_EQUAL_HEX8((uint8_t)len, d.frame().seq);
        TEST_ASSERT_EQUAL_UINT16(len, d.frame().payloadLen);
        if (len > 0)
            TEST_ASSERT_EQUAL_HEX8_ARRAY(payload.data(), d.frame().payload, len);

        TEST_ASSERT_EQUAL(MODEM_FRAME_NONE, d.next(1000));
    }
}

// ---------------------------------------------------------------------------
// Group F2 - resynchronisation

static void test_F2a_leading_garbage_discarded()
{
    ModemFrameDecoder d;
    d.reset();

    std::vector<uint8_t> stream = {0x00, 0xFF, 0x13, 0x37};
    const std::vector<uint8_t> f = encodeOk(MODEM_CMD_HELLO, 9, {1, 2, 3});
    stream.insert(stream.end(), f.begin(), f.end());

    feedAll(d, stream, 500);
    TEST_ASSERT_EQUAL(MODEM_FRAME_OK, d.next(500));
    TEST_ASSERT_EQUAL_HEX8(MODEM_CMD_HELLO, d.frame().type);
    TEST_ASSERT_EQUAL_HEX8(9, d.frame().seq);
}

static void test_F2b_back_to_back_frames()
{
    ModemFrameDecoder d;
    d.reset();

    std::vector<uint8_t> stream;
    for (uint8_t i = 0; i < 4; i++) {
        const std::vector<uint8_t> f = encodeOk(MODEM_EVT_TX_STATUS, i, {i, (uint8_t)(i + 1)});
        stream.insert(stream.end(), f.begin(), f.end());
    }

    feedAll(d, stream, 100);
    for (uint8_t i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL(MODEM_FRAME_OK, d.next(100));
        TEST_ASSERT_EQUAL_HEX8(i, d.frame().seq);
        TEST_ASSERT_EQUAL_UINT16(2, d.frame().payloadLen);
        TEST_ASSERT_EQUAL_HEX8(i, d.frame().payload[0]);
    }
    TEST_ASSERT_EQUAL(MODEM_FRAME_NONE, d.next(100));
}

static void test_F2c_crc_error_then_recovery()
{
    ModemFrameDecoder d;
    d.reset();

    std::vector<uint8_t> bad = encodeOk(MODEM_CMD_SEND_TEXT, 1, {'h', 'i'});
    bad[bad.size() - 1] ^= 0xFF;

    const std::vector<uint8_t> good = encodeOk(MODEM_CMD_SEND_TEXT, 2, {'o', 'k'});

    std::vector<uint8_t> stream = bad;
    stream.insert(stream.end(), good.begin(), good.end());
    feedAll(d, stream, 10);

    TEST_ASSERT_EQUAL(MODEM_FRAME_ERR_CRC, d.next(10));

    ModemFrameResult r;
    while ((r = d.next(10)) == MODEM_FRAME_ERR_CRC || r == MODEM_FRAME_ERR_OVERSIZE)
        ;

    TEST_ASSERT_EQUAL(MODEM_FRAME_OK, r);
    TEST_ASSERT_EQUAL_HEX8(2, d.frame().seq);
}

static void test_F2d_oversize_length_discards_one_byte_only()
{
    ModemFrameDecoder d;
    d.reset();

    std::vector<uint8_t> stream = {MODEM_SOF, 0xFF, 0xFF, 0x01, 0x00};
    const std::vector<uint8_t> good = encodeOk(MODEM_CMD_GET_MODEM_STATUS, 7, {});
    stream.insert(stream.end(), good.begin(), good.end());

    feedAll(d, stream, 0);
    TEST_ASSERT_EQUAL(MODEM_FRAME_ERR_OVERSIZE, d.next(0));

    ModemFrameResult r;
    while ((r = d.next(0)) == MODEM_FRAME_ERR_CRC || r == MODEM_FRAME_ERR_OVERSIZE)
        ;

    TEST_ASSERT_EQUAL(MODEM_FRAME_OK, r);
    TEST_ASSERT_EQUAL_HEX8(MODEM_CMD_GET_MODEM_STATUS, d.frame().type);
    TEST_ASSERT_EQUAL_HEX8(7, d.frame().seq);
}

static void test_F2e_interbyte_timeout()
{
    ModemFrameDecoder d;
    d.reset();

    const std::vector<uint8_t> f = encodeOk(MODEM_CMD_SEND_BINARY, 3, {1, 2, 3, 4, 5, 6, 7, 8});
    const std::vector<uint8_t> head(f.begin(), f.begin() + 6);
    feedAll(d, head, 1000);

    TEST_ASSERT_EQUAL(MODEM_FRAME_NONE, d.next(1000 + MODEM_T_FRAME_MS));
    TEST_ASSERT_EQUAL(MODEM_FRAME_ERR_TIMEOUT, d.next(1000 + MODEM_T_FRAME_MS + 1));

    feedAll(d, f, 2000);
    ModemFrameResult r;
    while ((r = d.next(2000)) == MODEM_FRAME_ERR_CRC || r == MODEM_FRAME_ERR_OVERSIZE || r == MODEM_FRAME_ERR_TIMEOUT)
        ;
    TEST_ASSERT_EQUAL(MODEM_FRAME_OK, r);
    TEST_ASSERT_EQUAL_HEX8(3, d.frame().seq);
}

static void test_F2f_byte_at_a_time()
{
    ModemFrameDecoder d;
    d.reset();

    std::vector<uint8_t> payload(200);
    for (size_t i = 0; i < payload.size(); i++)
        payload[i] = (uint8_t)i;

    const std::vector<uint8_t> f = encodeOk(MODEM_EVT_RX_TEXT, 0x5A, payload);

    unsigned complete = 0;
    for (size_t i = 0; i < f.size(); i++) {
        TEST_ASSERT_EQUAL_size_t(1, d.feed(&f[i], 1, 50));
        if (d.next(50) == MODEM_FRAME_OK)
            complete++;
    }

    TEST_ASSERT_EQUAL_UINT(1, complete);
    TEST_ASSERT_EQUAL_HEX8(MODEM_EVT_RX_TEXT, d.frame().type);
    TEST_ASSERT_EQUAL_UINT16(payload.size(), d.frame().payloadLen);
}

static void test_F2g_feed_reports_short_accept_when_full()
{
    ModemFrameDecoder d;
    d.reset();

    std::vector<uint8_t> junk(MODEM_MAX_FRAME_SIZE + 64, 0x00);
    const size_t took = d.feed(junk.data(), junk.size(), 0);

    TEST_ASSERT_EQUAL_size_t(MODEM_MAX_FRAME_SIZE, took);
    TEST_ASSERT_EQUAL(MODEM_FRAME_NONE, d.next(0));
    TEST_ASSERT_EQUAL_size_t(0, d.pending());
}

// ---------------------------------------------------------------------------
// Group F3 - fuzz

static void test_F3a_random_bytes_never_crash()
{
    rngSeed(BASE_SEED);
    ModemFrameDecoder d;
    d.reset();

    for (unsigned it = 0; it < FUZZ_ITERS; it++) {
        uint8_t chunk[64];
        const uint32_t n = rngRange(sizeof(chunk)) + 1;
        for (uint32_t i = 0; i < n; i++)
            chunk[i] = rngByte();

        size_t off = 0;
        while (off < n) {
            const size_t took = d.feed(&chunk[off], n - off, it);
            off += took;
            if (took == 0) {
                while (d.next(it) != MODEM_FRAME_NONE)
                    ;
            }
        }

        ModemFrameResult r;
        while ((r = d.next(it)) != MODEM_FRAME_NONE) {
            if (r == MODEM_FRAME_OK) {
                TEST_ASSERT_NOT_NULL(d.frame().payload);
                TEST_ASSERT_LESS_OR_EQUAL_UINT16(MODEM_MAX_PAYLOAD, d.frame().payloadLen);
            }
        }
    }
}

static void test_F3b_frames_survive_random_garbage()
{
    rngSeed(BASE_SEED ^ 0x5A5AULL);

    for (unsigned it = 0; it < 500; it++) {
        const uint16_t len = (uint16_t)rngRange(64);
        std::vector<uint8_t> payload(len);
        for (uint16_t i = 0; i < len; i++)
            payload[i] = rngByte();

        const uint8_t seq = rngByte();
        const std::vector<uint8_t> f = encodeOk(MODEM_EVT_RX_DATA, seq, payload);

        std::vector<uint8_t> stream;
        const uint32_t leading = rngRange(32);
        for (uint32_t i = 0; i < leading; i++) {
            const uint8_t b = rngByte();
            stream.push_back(b == MODEM_SOF ? (uint8_t)(b + 1) : b);
        }
        stream.insert(stream.end(), f.begin(), f.end());

        ModemFrameDecoder d;
        d.reset();
        feedAll(d, stream, it);

        bool found = false;
        ModemFrameResult r;
        while ((r = d.next(it)) != MODEM_FRAME_NONE) {
            if (r == MODEM_FRAME_OK && d.frame().seq == seq && d.frame().payloadLen == len) {
                found = true;
                break;
            }
        }

        TEST_ASSERT_TRUE_MESSAGE(found, "a well-formed frame after non-SOF garbage must be recovered");
    }
}

// ---------------------------------------------------------------------------
void setUp(void) {}
void tearDown(void) {}

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();

    printf("\n=== Group F1: encoding and CRC ===\n");
    RUN_TEST(test_F1a_crc_check_vector);
    RUN_TEST(test_F1b_encode_layout);
    RUN_TEST(test_F1c_encode_rejects_bad_args);
    RUN_TEST(test_F1d_roundtrip_all_lengths);

    printf("\n=== Group F2: resynchronisation ===\n");
    RUN_TEST(test_F2a_leading_garbage_discarded);
    RUN_TEST(test_F2b_back_to_back_frames);
    RUN_TEST(test_F2c_crc_error_then_recovery);
    RUN_TEST(test_F2d_oversize_length_discards_one_byte_only);
    RUN_TEST(test_F2e_interbyte_timeout);
    RUN_TEST(test_F2f_byte_at_a_time);
    RUN_TEST(test_F2g_feed_reports_short_accept_when_full);

    printf("\n=== Group F3: fuzz ===\n");
    RUN_TEST(test_F3a_random_bytes_never_crash);
    RUN_TEST(test_F3b_frames_survive_random_garbage);

    exit(UNITY_END());
}

void loop() {}
