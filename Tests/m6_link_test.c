#include "m6_link.h"
#include "generated/m6_v2_wire_vectors.inc"

#include <assert.h>
#include <string.h>

static uint8_t writes[12][M6_MAX_WIRE_FRAME];
static uint16_t write_lengths[12];
static uint8_t write_count;

static bool write_frame(const uint8_t *data, uint16_t length, void *context)
{
    (void)context;
    assert(write_count < 12U);
    memcpy(writes[write_count], data, length);
    write_lengths[write_count++] = length;
    return true;
}

static bool tx_idle(void *context)
{
    (void)context;
    return true;
}

static M6Result_t execute_rpc(uint16_t opcode, const uint8_t *body,
                              uint16_t body_length, uint8_t *success,
                              uint16_t *success_length, uint32_t *detail,
                              void *context)
{
    (void)context;
    *detail = 0U;
    if ((opcode == 2U) && (body_length == 0U))
        return M6_EXECUTOR_PENDING;
    if ((opcode != 1U) || (body_length != 4U) || (*success_length < 12U))
        return M6_RESULT_UNSUPPORTED_OPCODE;
    memcpy(success, body, 4U);
    success[4] = 0x7EU;
    success[5] = 0x7DU;
    success[6] = 0x53U;
    success[7] = 0x20U;
    success[8] = 0x40U;
    success[9] = 0x30U;
    success[10] = 0x20U;
    success[11] = 0x10U;
    *success_length = 12U;
    return M6_RESULT_OK;
}

static M6Frame_t decode(const uint8_t *wire, uint16_t length)
{
    M6Parser_t parser;
    M6Frame_t result = {0};
    bool complete = false;
    M6Parser_Init(&parser, 0U, 0U);
    for (uint16_t i = 0U; i < length; i++)
        if (M6Parser_Feed(&parser, wire[i], 0U, 0U, 0U, &result))
            complete = true;
    assert(complete);
    return result;
}

static void pump(uint32_t now_ms)
{
    M6Link_Service(now_ms, 0U, 0U); /* select */
    M6Link_Service(now_ms, 0U, 0U); /* submit */
    M6Link_Service(now_ms, 0U, 0U); /* physical completion */
}

static void feed(const uint8_t *wire, uint16_t length, uint32_t now_ms)
{
    for (uint16_t i = 0U; i < length; i++)
        M6Link_ProcessByte(wire[i], now_ms, 0U, 0U);
}

int main(void)
{
    M6LinkConfig_t config = {
        .boot_id = 0x10203040UL,
        .raw_reset_cause = 0x0000000CUL,
        .session_generation = 6U,
        .parameter_schema = 1U,
        .capabilities = M6_DEVICE_CAPABILITIES
    };
    M6LinkStats_t stats;
    M6Frame_t frame;
    uint8_t hello[M6_MAX_WIRE_FRAME];
    uint16_t hello_length = M6_V2_HELLO_REQUEST_WIRE_LENGTH;
    uint8_t event[4] = {1U, 0U, 1U, 0U};

    M6Link_Init(&config, execute_rpc, write_frame, tx_idle, NULL, 0U, 0U);
    assert(M6Link_BeginV2(UINT64_C(0x1111111111111111), 0U));
    M6Link_Service(M6_V1_DRAIN_TIMEOUT_MS - 1U, 0U, 0U);
    assert(M6Link_Mode() == M6_LINK_V2_DRAIN);
    M6Link_Service(M6_V1_DRAIN_TIMEOUT_MS, 0U, 0U);
    assert(M6Link_Mode() == M6_LINK_V2_DRAIN);
    assert(M6Link_DrainFailed());
    M6Link_NotifyV1ReadyComplete(M6_V1_DRAIN_TIMEOUT_MS);
    assert(M6Link_Mode() == M6_LINK_V1_RESYNC);

    M6Link_Init(&config, execute_rpc, write_frame, tx_idle, NULL, 0U, 0U);
    assert(M6Link_BeginV2(UINT64_C(0x0123456789ABCDEF), 0U));
    M6Link_NotifyV1ReadyComplete(0U);
    {
        M6Frame_t bad_hello;
        M6Parser_t parser;
        bool decoded = false;
        M6Parser_Init(&parser, 0U, 0U);
        for (size_t i = 0U; i < M6_V2_HELLO_REQUEST_WIRE_LENGTH; i++)
            if (M6Parser_Feed(&parser, m6_v2_hello_request_wire[i],
                              0U, 0U, 0U, &bad_hello)) decoded = true;
        assert(decoded);
        bad_hello.payload[36] = 2U;
        hello_length = (uint16_t)M6Wire_Encode(
            &bad_hello, hello, sizeof(hello));
        feed(hello, hello_length, 1U);
        pump(1U);
        assert(write_count == 1U);
        frame = decode(writes[0], write_lengths[0]);
        assert(frame.payload[6] == M6_RESULT_BAD_LENGTH);
        assert(frame.payload[8] == 2U && frame.payload[9] == 0U &&
               frame.payload[10] == 1U && frame.payload[11] == 0U);
        write_count = 0U;
    }

    M6Link_Init(&config, execute_rpc, write_frame, tx_idle, NULL, 0U, 0U);
    assert(M6Link_BeginV2(UINT64_C(0x0123456789ABCDEF), 0U));
    M6Link_NotifyV1ReadyComplete(0U);
    memcpy(hello, m6_v2_hello_request_wire, hello_length);
    /* Golden HELLO asks for capabilities not published by this r15 slice.
     * Limit optional to the actual device mask and fix its CRC by encoding. */
    {
        M6Frame_t h;
        M6Parser_t p;
        bool ok = false;
        M6Parser_Init(&p, 0U, 0U);
        for (size_t i = 0U; i < hello_length; i++)
            if (M6Parser_Feed(&p, hello[i], 0U, 0U, 0U, &h)) ok = true;
        assert(ok);
        {
            uint64_t optional = M6_DEVICE_CAPABILITIES &
                ~M6_HANDSHAKE_REQUIRED;
            for (uint8_t i = 0U; i < 8U; i++)
                h.payload[24U + i] = (uint8_t)(optional >> (8U * i));
        }
        hello_length = (uint16_t)M6Wire_Encode(&h, hello, sizeof(hello));
        assert(hello_length != 0U);
    }
    feed(hello, hello_length, 1U);
    pump(1U);
    assert(write_count == 1U);
    frame = decode(writes[0], write_lengths[0]);
    assert(frame.type == M6_FRAME_READY);
    assert(frame.payload_length == 64U);
    assert(frame.payload[6] == 0U && frame.payload[7] == 0U);
    M6Link_GetStats(&stats);
    assert(stats.mode == M6_LINK_V2_ESTABLISHED);
    assert(stats.session == 0x141C7206UL);

    feed(m6_v2_ping_request_wire, M6_V2_PING_REQUEST_WIRE_LENGTH, 2U);
    pump(2U);
    assert(write_count == 2U);
    assert(write_lengths[1] == M6_V2_PING_RESPONSE_WIRE_LENGTH);
    assert(memcmp(writes[1], m6_v2_ping_response_wire,
                  write_lengths[1]) == 0);

    feed(m6_v2_ping_request_wire, M6_V2_PING_REQUEST_WIRE_LENGTH, 3U);
    pump(3U);
    assert(write_count == 3U);
    assert(write_lengths[2] == M6_V2_PING_RESPONSE_REPLAY_WIRE_LENGTH);
    assert(memcmp(writes[2], m6_v2_ping_response_replay_wire,
                  write_lengths[2]) == 0);

    assert(M6Link_Publish(M6_TX_P2, M6_FRAME_TELEMETRY, 1U,
                          event, sizeof(event)));
    assert(M6Link_Publish(M6_TX_P0, M6_FRAME_EVENT, 1U,
                          event, sizeof(event)));
    pump(4U);
    frame = decode(writes[3], write_lengths[3]);
    assert(frame.type == M6_FRAME_EVENT);
    pump(5U);
    frame = decode(writes[4], write_lengths[4]);
    assert(frame.type == M6_FRAME_TELEMETRY);
    M6Link_GetStats(&stats);
    assert(stats.replay_count == 1U);
    assert(stats.p0_high_water == 1U);
    assert(stats.p2_high_water == 1U);

    assert(M6Link_Publish(M6_TX_P2, M6_FRAME_TELEMETRY, 2U,
                          event, sizeof(event)));
    assert(M6Link_ClearTelemetry(1U << 1, M6_GAP_PRODUCER_DROP) == 1U);
    assert(M6Link_Publish(M6_TX_P2, M6_FRAME_TELEMETRY, 2U,
                          event, sizeof(event)));
    assert(M6Link_Publish(M6_TX_P3, M6_FRAME_BULK, 20U,
                          event, sizeof(event)));
    M6Link_Emergency(1U);
    M6Link_Service(6U, 0U, 0U);
    M6Link_GetStats(&stats);
    assert(stats.mode == M6_LINK_V2_ESTABLISHED);
    assert(stats.session == 0x141C7206UL);
    assert(stats.p2_drop == 2U);
    assert(stats.p3_drop == 1U);
    assert(write_count == 5U);

    memset(&frame, 0, sizeof(frame));
    frame.version = M6_WIRE_VERSION;
    frame.type = M6_FRAME_RPC_REQUEST;
    frame.session = stats.session;
    frame.sequence = 2U;
    frame.payload_length = 2U;
    frame.payload[0] = 2U;
    {
        uint8_t pending_wire[M6_MAX_WIRE_FRAME];
        uint16_t pending_length = (uint16_t)M6Wire_Encode(
            &frame, pending_wire, sizeof(pending_wire));
        assert(pending_length != 0U);
        feed(pending_wire, pending_length, 7U);
        pump(7U);
        assert(write_count == 5U);
        feed(pending_wire, pending_length, 8U);
        pump(8U);
        assert(write_count == 5U);
    }
    assert(M6Link_CompletePending(2U, M6_RESULT_OK, 0U,
                                  event, sizeof(event)));
    pump(9U);
    assert(write_count == 6U);
    frame = decode(writes[5], write_lengths[5]);
    assert(frame.type == M6_FRAME_RPC_RESPONSE);
    assert(frame.sequence == 2U);
    assert(frame.payload_length == 12U);
    assert(frame.payload[0] == 2U && frame.payload[1] == 0U);
    M6Link_GetStats(&stats);
    assert(stats.sequence_errors == 0U);

    memset(&frame, 0, sizeof(frame));
    frame.version = M6_WIRE_VERSION;
    frame.type = M6_FRAME_RPC_REQUEST;
    frame.flags = 0x80U;
    frame.session = stats.session ^ 1U;
    frame.sequence = 3U;
    frame.payload_length = 1U;
    {
        uint8_t bad_wire[M6_MAX_WIRE_FRAME];
        uint16_t bad_length = (uint16_t)M6Wire_Encode(
            &frame, bad_wire, sizeof(bad_wire));
        feed(bad_wire, bad_length, 10U);
        pump(10U);
        assert(write_count == 6U);
    }
    M6Link_GetStats(&stats);
    assert(stats.session_mismatch == 1U);

    frame.type = 0x55U;
    frame.flags = 0U;
    frame.session = stats.session;
    frame.payload_length = 0U;
    {
        uint8_t bad_wire[M6_MAX_WIRE_FRAME];
        uint16_t bad_length = (uint16_t)M6Wire_Encode(
            &frame, bad_wire, sizeof(bad_wire));
        feed(bad_wire, bad_length, 10U);
        pump(10U);
        assert(write_count == 7U);
        feed(bad_wire, bad_length, 50U);
        pump(50U);
        assert(write_count == 7U);
        feed(bad_wire, bad_length, 110U);
        pump(110U);
        assert(write_count == 8U);
    }
    M6Link_GetStats(&stats);
    assert(stats.control_error_drop == 1U);

    {
        uint32_t sequence_before = stats.next_async_sequence;
        assert(M6Link_Publish(M6_TX_P2, M6_FRAME_TELEMETRY, 3U,
                              event, sizeof(event)));
        assert(M6Link_Publish(M6_TX_P3, M6_FRAME_BULK, 21U,
                              event, sizeof(event)));
        M6Link_TestSetTokens(0U, 0U);
        pump(110U);
        M6Link_GetStats(&stats);
        assert(stats.next_async_sequence == sequence_before);
        assert(write_count == 8U);
        M6Link_Service(110U, 0U, 0U);
        M6Link_GetStats(&stats);
        assert(stats.next_async_sequence == sequence_before);
    }

    assert(M6Link_Publish(M6_TX_P0, M6_FRAME_EVENT, 1U,
                          event, sizeof(event)));
    M6Link_TestSetNextAsyncSequence(0xFFFFFFFEUL);
    pump(111U);
    assert(write_count == 9U);
    frame = decode(writes[8], write_lengths[8]);
    assert(frame.type == M6_FRAME_EVENT);
    assert(frame.sequence == 0xFFFFFFFEUL);
    assert(frame.payload[0] == M6_EVENT_LINK_STATE);
    assert(frame.payload[1] == 0U);
    M6Link_GetStats(&stats);
    assert(stats.next_async_sequence == 0xFFFFFFFFUL);
    return 0;
}
