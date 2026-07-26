#include "m6_wire.h"
#include "generated/m6_v2_wire_vectors.inc"

#include <assert.h>
#include <string.h>

static void test_crc_and_session(void)
{
    assert(M6Wire_Crc16(m6_v2_crc_empty_input,
        M6_V2_CRC_EMPTY_INPUT_LENGTH) == M6_V2_CRC_EMPTY_EXPECTED);
    assert(M6Wire_Crc16(m6_v2_crc_check_123456789_input,
        M6_V2_CRC_CHECK_123456789_INPUT_LENGTH) ==
        M6_V2_CRC_CHECK_123456789_EXPECTED);
    assert(M6Wire_Crc16(m6_v2_crc_bytes_00_to_ff_input,
        M6_V2_CRC_BYTES_00_TO_FF_INPUT_LENGTH) ==
        M6_V2_CRC_BYTES_00_TO_FF_EXPECTED);
    assert(M6Wire_DeriveSession(UINT64_C(0x0123456789ABCDEF),
        0x10203040UL, 0x0000000CUL, 7U, 1U) ==
        M6_V2_SESSION_NOMINAL_EXPECTED_SESSION);
    assert(M6Wire_Crc32(m6_v2_session_zero_maps_to_one_input,
        M6_V2_SESSION_ZERO_MAPS_TO_ONE_INPUT_LENGTH) ==
        M6_V2_SESSION_ZERO_MAPS_TO_ONE_RAW_CRC32);
    assert(M6Wire_DeriveSession(UINT64_C(0x0123456789ABCDEF),
        0x10203040UL, 0xA21D170FUL, 7U, 1U) ==
        M6_V2_SESSION_ZERO_MAPS_TO_ONE_EXPECTED_SESSION);
}

static M6Frame_t ping_frame(void)
{
    M6Frame_t frame = {0};
    frame.version = M6_WIRE_VERSION;
    frame.type = M6_FRAME_RPC_REQUEST;
    frame.session = 0x141C7206UL;
    frame.sequence = 1U;
    frame.payload_length = 6U;
    frame.payload[0] = 1U;
    frame.payload[1] = 0U;
    frame.payload[2] = 0x7EU;
    frame.payload[3] = 0x20U;
    frame.payload[4] = 0x53U;
    frame.payload[5] = 0x7DU;
    return frame;
}

static void test_encode_and_parse(void)
{
    uint8_t wire[M6_MAX_WIRE_FRAME];
    M6Frame_t source = ping_frame();
    M6Frame_t decoded;
    M6Parser_t parser;
    size_t length = M6Wire_Encode(&source, wire, sizeof(wire));
    bool complete = false;

    assert(length == M6_V2_PING_REQUEST_WIRE_LENGTH);
    assert(memcmp(wire, m6_v2_ping_request_wire, length) == 0);
    M6Parser_Init(&parser, 0U, 0U);
    for (size_t i = 0U; i < length; i++) {
        if (M6Parser_Feed(&parser, wire[i], 10U, 0U, 0U, &decoded)) {
            assert(!complete);
            complete = true;
        }
    }
    assert(complete);
    assert(decoded.type == source.type);
    assert(decoded.flags == source.flags);
    assert(decoded.session == source.session);
    assert(decoded.sequence == source.sequence);
    assert(decoded.payload_length == source.payload_length);
    assert(memcmp(decoded.payload, source.payload, source.payload_length) == 0);
    assert(parser.state == M6_PARSE_COLLECT);
}

typedef struct {
    const uint8_t *wire;
    size_t length;
} GoldenFrame_t;

static void assert_golden_round_trip(const GoldenFrame_t *golden)
{
    uint8_t encoded[M6_MAX_WIRE_FRAME];
    M6Parser_t parser;
    M6Frame_t decoded;
    size_t encoded_length;
    bool complete = false;

    M6Parser_Init(&parser, 0U, 0U);
    for (size_t i = 0U; i < golden->length; i++) {
        if (M6Parser_Feed(&parser, golden->wire[i], 0U, 0U, 0U,
                          &decoded)) {
            assert(!complete);
            complete = true;
        }
    }
    assert(complete);
    encoded_length = M6Wire_Encode(&decoded, encoded, sizeof(encoded));
    assert(encoded_length == golden->length);
    assert(memcmp(encoded, golden->wire, encoded_length) == 0);
}

static void test_all_golden_frames(void)
{
    static const GoldenFrame_t frames[] = {
        {m6_v2_hello_request_wire, M6_V2_HELLO_REQUEST_WIRE_LENGTH},
        {m6_v2_ready_wire, M6_V2_READY_WIRE_LENGTH},
        {m6_v2_ready_bad_host_payload_64_wire,
         M6_V2_READY_BAD_HOST_PAYLOAD_64_WIRE_LENGTH},
        {m6_v2_ping_request_wire, M6_V2_PING_REQUEST_WIRE_LENGTH},
        {m6_v2_ping_response_wire, M6_V2_PING_RESPONSE_WIRE_LENGTH},
        {m6_v2_ping_response_replay_wire,
         M6_V2_PING_RESPONSE_REPLAY_WIRE_LENGTH},
        {m6_v2_unknown_type_wire, M6_V2_UNKNOWN_TYPE_WIRE_LENGTH},
        {m6_v2_unsupported_type_error_wire,
         M6_V2_UNSUPPORTED_TYPE_ERROR_WIRE_LENGTH},
        {m6_v2_core_telemetry_gap_wire,
         M6_V2_CORE_TELEMETRY_GAP_WIRE_LENGTH},
        {m6_v2_max_payload_bulk_gap_wire,
         M6_V2_MAX_PAYLOAD_BULK_GAP_WIRE_LENGTH},
    };

    for (size_t i = 0U; i < sizeof(frames) / sizeof(frames[0]); i++)
        assert_golden_round_trip(&frames[i]);
}

static void test_stream_boundaries(void)
{
    M6Parser_t parser;
    M6Frame_t frame;
    M6Parser_Init(&parser, 0U, 0U);
    assert(!M6Parser_Feed(&parser, 0x7EU, 0U, 0U, 0U, &frame));
    assert(!M6Parser_Feed(&parser, 0x02U, 0U, 0U, 0U, &frame));
    assert(!M6Parser_Feed(&parser, 0x7DU, 0U, 0U, 0U, &frame));
    assert(!M6Parser_Feed(&parser, 0x01U, 0U, 0U, 0U, &frame));
    assert(parser.state == M6_PARSE_DISCARD);
    assert(parser.counters.escape_error == 1U);
    M6Parser_Tick(&parser, 100U, 0U, 0U);
    assert(parser.state == M6_PARSE_HUNT);
    assert(parser.counters.total_timeout == 1U);

    M6Parser_Init(&parser, 0U, 0U);
    for (size_t i = 0U; i < 10U; i++)
        (void)M6Parser_Feed(&parser, m6_v2_ping_request_wire[i],
                           0xFFFFFFF0UL, 0U, 0U, &frame);
    M6Parser_Tick(&parser, 33U, 0U, 0U);
    assert(parser.state != M6_PARSE_HUNT);
    M6Parser_Tick(&parser, 34U, 0U, 0U);
    assert(parser.state == M6_PARSE_HUNT);
    assert(parser.counters.interbyte_timeout == 1U);

    M6Parser_Init(&parser, 0U, 0U);
    for (size_t i = 0U; i < 10U; i++)
        (void)M6Parser_Feed(&parser, m6_v2_ping_request_wire[i],
                           5U, 0U, 0U, &frame);
    M6Parser_Tick(&parser, 6U, 1U, 0U);
    assert(parser.state == M6_PARSE_HUNT);
    assert(parser.counters.epoch_abort == 1U);
}

int main(void)
{
    test_crc_and_session();
    test_encode_and_parse();
    test_all_golden_frames();
    test_stream_boundaries();
    return 0;
}
