#include "m6_wire.h"

#include <string.h>

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void put_u64(uint8_t *p, uint64_t value)
{
    for (uint8_t i = 0U; i < 8U; i++) p[i] = (uint8_t)(value >> (8U * i));
}

static bool must_escape(uint8_t byte)
{
    return (byte == 0x20U) || (byte == 0x53U) ||
           (byte == M6_FRAME_ESCAPE) || (byte == M6_FRAME_DELIMITER);
}

uint16_t M6Wire_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++)
            crc = (crc & 0x8000U) != 0U ?
                (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint32_t M6Wire_Crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
            crc = (crc & 1U) != 0U ?
                (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
    }
    return crc ^ 0xFFFFFFFFUL;
}

uint32_t M6Wire_DeriveSession(uint64_t challenge, uint32_t boot_id,
                              uint32_t raw_reset_cause,
                              uint32_t session_generation,
                              uint16_t parameter_schema)
{
    uint8_t input[30] = {'M','6','V','2','S','E','S','S'};
    uint32_t result;
    put_u64(&input[8], challenge);
    put_u32(&input[16], boot_id);
    put_u32(&input[20], raw_reset_cause);
    put_u32(&input[24], session_generation);
    put_u16(&input[28], parameter_schema);
    result = M6Wire_Crc32(input, sizeof(input));
    return result == 0U ? 1U : result;
}

size_t M6Wire_Encode(const M6Frame_t *frame, uint8_t *wire,
                     size_t capacity)
{
    uint8_t decoded[M6_MAX_DECODED_FRAME];
    size_t decoded_length;
    size_t out = 0U;
    uint16_t crc;

    if ((frame == NULL) || (wire == NULL) ||
        (frame->payload_length > M6_MAX_PAYLOAD)) return 0U;
    decoded[0] = M6_WIRE_VERSION;
    decoded[1] = frame->type;
    decoded[2] = frame->flags;
    put_u32(&decoded[3], frame->session);
    put_u32(&decoded[7], frame->sequence);
    put_u16(&decoded[11], frame->payload_length);
    if (frame->payload_length != 0U)
        memcpy(&decoded[M6_FRAME_HEADER_SIZE], frame->payload,
               frame->payload_length);
    decoded_length = M6_FRAME_HEADER_SIZE + frame->payload_length;
    crc = M6Wire_Crc16(decoded, decoded_length);
    put_u16(&decoded[decoded_length], crc);
    decoded_length += M6_FRAME_CRC_SIZE;

    if (capacity == 0U) return 0U;
    wire[out++] = M6_FRAME_DELIMITER;
    for (size_t i = 0U; i < decoded_length; i++) {
        if (must_escape(decoded[i])) {
            if ((out + 2U) > capacity) return 0U;
            wire[out++] = M6_FRAME_ESCAPE;
            wire[out++] = decoded[i] ^ M6_ESCAPE_XOR;
        } else {
            if ((out + 1U) > capacity) return 0U;
            wire[out++] = decoded[i];
        }
    }
    if ((out + 1U) > capacity) return 0U;
    wire[out++] = M6_FRAME_DELIMITER;
    return out;
}

static void parser_hunt(M6Parser_t *parser)
{
    parser->state = M6_PARSE_HUNT;
    parser->decoded_length = 0U;
    parser->wire_length = 0U;
    parser->timing_active = false;
}

static void parser_open(M6Parser_t *parser)
{
    parser->state = M6_PARSE_COLLECT;
    parser->decoded_length = 0U;
    parser->wire_length = 1U;
    parser->timing_active = false;
}

static bool parser_epoch(M6Parser_t *parser, uint32_t emergency_epoch,
                         uint32_t overflow_epoch)
{
    if ((emergency_epoch == parser->emergency_epoch) &&
        (overflow_epoch == parser->overflow_epoch)) return false;
    parser->emergency_epoch = emergency_epoch;
    parser->overflow_epoch = overflow_epoch;
    if (parser->state != M6_PARSE_HUNT) parser->counters.epoch_abort++;
    parser_hunt(parser);
    return true;
}

static bool parser_timeout(M6Parser_t *parser, uint32_t now_ms)
{
    if (!parser->timing_active) return false;
    if ((uint32_t)(now_ms - parser->started_ms) >= M6_TOTAL_TIMEOUT_MS) {
        parser->counters.total_timeout++;
        parser_hunt(parser);
        return true;
    }
    if ((uint32_t)(now_ms - parser->last_byte_ms) >=
        M6_INTERBYTE_TIMEOUT_MS) {
        parser->counters.interbyte_timeout++;
        parser_hunt(parser);
        return true;
    }
    return false;
}

static void parser_mark_byte(M6Parser_t *parser, uint32_t now_ms)
{
    if (!parser->timing_active) {
        parser->timing_active = true;
        parser->started_ms = now_ms;
    }
    parser->last_byte_ms = now_ms;
}

static bool decode_candidate(M6Parser_t *parser, M6Frame_t *frame)
{
    uint16_t payload_length;
    uint16_t expected_crc;
    uint16_t received_crc;

    if (parser->decoded_length <
        (M6_FRAME_HEADER_SIZE + M6_FRAME_CRC_SIZE)) {
        parser->counters.short_frame++;
        return false;
    }
    payload_length = read_u16(&parser->decoded[11]);
    if ((payload_length > M6_MAX_PAYLOAD) ||
        (parser->decoded_length !=
         (uint16_t)(M6_FRAME_HEADER_SIZE + payload_length +
                    M6_FRAME_CRC_SIZE))) {
        parser->counters.length_error++;
        return false;
    }
    received_crc = read_u16(&parser->decoded[parser->decoded_length - 2U]);
    expected_crc = M6Wire_Crc16(parser->decoded,
                                parser->decoded_length - 2U);
    if (received_crc != expected_crc) {
        parser->counters.crc_error++;
        return false;
    }
    if (frame != NULL) {
        frame->version = parser->decoded[0];
        frame->type = parser->decoded[1];
        frame->flags = parser->decoded[2];
        frame->session = read_u32(&parser->decoded[3]);
        frame->sequence = read_u32(&parser->decoded[7]);
        frame->payload_length = payload_length;
        if (payload_length != 0U)
            memcpy(frame->payload, &parser->decoded[M6_FRAME_HEADER_SIZE],
                   payload_length);
    }
    return true;
}

void M6Parser_Init(M6Parser_t *parser, uint32_t emergency_epoch,
                   uint32_t overflow_epoch)
{
    if (parser == NULL) return;
    memset(parser, 0, sizeof(*parser));
    parser->state = M6_PARSE_HUNT;
    parser->emergency_epoch = emergency_epoch;
    parser->overflow_epoch = overflow_epoch;
}

bool M6Parser_Feed(M6Parser_t *parser, uint8_t byte, uint32_t now_ms,
                   uint32_t emergency_epoch, uint32_t overflow_epoch,
                   M6Frame_t *frame)
{
    bool completed = false;
    if (parser == NULL) return false;
    (void)parser_epoch(parser, emergency_epoch, overflow_epoch);
    (void)parser_timeout(parser, now_ms);

    if (parser->state == M6_PARSE_HUNT) {
        if (byte == M6_FRAME_DELIMITER) parser_open(parser);
        return false;
    }
    if (byte == M6_FRAME_DELIMITER) {
        if (parser->state == M6_PARSE_ESCAPED) {
            parser->counters.escape_error++;
        } else if ((parser->state == M6_PARSE_COLLECT) &&
                   (parser->decoded_length != 0U)) {
            completed = decode_candidate(parser, frame);
        }
        parser_open(parser);
        return completed;
    }

    if (parser->state == M6_PARSE_DISCARD) {
        parser_mark_byte(parser, now_ms);
        return false;
    }
    /* Every accepted internal wire byte must still leave room for closing
     * delimiter.  wire_length already includes the opening delimiter. */
    if (parser->wire_length >= (M6_MAX_WIRE_FRAME - 1U)) {
        parser->counters.wire_overflow++;
        parser->state = M6_PARSE_DISCARD;
        parser_mark_byte(parser, now_ms);
        return false;
    }
    parser->wire_length++;
    parser_mark_byte(parser, now_ms);

    if (parser->state == M6_PARSE_ESCAPED) {
        if ((byte != 0x00U) && (byte != 0x73U) &&
            (byte != 0x5DU) && (byte != 0x5EU)) {
            parser->counters.escape_error++;
            parser->state = M6_PARSE_DISCARD;
            return false;
        }
        byte ^= M6_ESCAPE_XOR;
        parser->state = M6_PARSE_COLLECT;
    } else if (byte == M6_FRAME_ESCAPE) {
        parser->state = M6_PARSE_ESCAPED;
        return false;
    }

    if (parser->decoded_length >= M6_MAX_DECODED_FRAME) {
        parser->counters.decoded_overflow++;
        parser->state = M6_PARSE_DISCARD;
        return false;
    }
    parser->decoded[parser->decoded_length++] = byte;
    return false;
}

void M6Parser_Tick(M6Parser_t *parser, uint32_t now_ms,
                   uint32_t emergency_epoch, uint32_t overflow_epoch)
{
    if (parser == NULL) return;
    if (parser_epoch(parser, emergency_epoch, overflow_epoch)) return;
    (void)parser_timeout(parser, now_ms);
}
