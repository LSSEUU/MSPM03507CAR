#include "m6_link.h"

#include <string.h>

#define M6_FLAG_REPLAY       0x01U
#define M6_FLAG_GAP_BEFORE   0x02U
#define M6_READY_MAGIC       0x59443252UL
#define M6_HELLO_MAGIC       0x3256364DUL
#define M6_HOST_LAST_SEQUENCE 0xFFFFFFFEUL
#define M6_ASYNC_TERMINAL_SEQUENCE 0xFFFFFFFEUL

typedef enum {
    REPLAY_FREE = 0,
    REPLAY_QUEUED,
    REPLAY_IN_FLIGHT,
    REPLAY_PENDING,
    REPLAY_COMPLETED
} ReplayState_t;

typedef struct {
    uint32_t sequence;
    uint32_t age;
    uint8_t request[M6_MAX_RPC_REQUEST_PAYLOAD];
    uint8_t response[M6_MAX_PAYLOAD];
    uint8_t request_length;
    uint8_t response_length;
    uint8_t state;
    uint8_t replay;
} ReplayEntry_t;

typedef struct {
    uint16_t topic;
    uint8_t type;
    uint8_t flags;
    uint8_t length;
    uint8_t occupied;
    uint8_t payload[48];
} EventSlot_t;

typedef struct {
    uint16_t topic;
    uint8_t type;
    uint8_t flags;
    uint8_t length;
    uint8_t occupied;
    uint8_t payload[72];
} TelemetrySlot_t;

typedef struct {
    uint16_t topic;
    uint8_t type;
    uint8_t flags;
    uint8_t length;
    uint8_t occupied;
    uint8_t payload[M6_MAX_PAYLOAD];
} BulkSlot_t;

typedef enum {
    FLIGHT_NONE = 0,
    FLIGHT_READY_SUCCESS,
    FLIGHT_READY_FAILURE,
    FLIGHT_REPLAY,
    FLIGHT_CONTROL,
    FLIGHT_P0,
    FLIGHT_P1,
    FLIGHT_P2,
    FLIGHT_P3,
    FLIGHT_TERMINAL,
    FLIGHT_RESYNC
} FlightKind_t;

typedef struct {
    M6LinkConfig_t config;
    M6RpcExecutor_t executor;
    M6LinkWrite_t write;
    M6LinkTxIdle_t tx_idle;
    void *context;
    M6Parser_t parser;
    M6LinkStats_t stats;
    uint64_t challenge;
    uint64_t enabled_capabilities;
    uint32_t mode_started_ms;
    uint32_t replay_age;
    ReplayEntry_t replay[M6_REPLAY_ENTRY_COUNT];
    EventSlot_t p0[M6_P0_TOPIC_COUNT];
    EventSlot_t p1[M6_P1_SLOT_COUNT];
    TelemetrySlot_t p2[M6_P2_SLOT_COUNT];
    BulkSlot_t p3[M6_P3_SLOT_COUNT];
    M6Frame_t ready;
    bool ready_pending;
    bool ready_success;
    M6Frame_t control;
    bool control_pending;
    uint8_t wire[M6_MAX_WIRE_FRAME];
    uint16_t wire_length;
    bool in_flight;
    FlightKind_t flight_kind;
    uint8_t flight_index;
    uint16_t flight_topic;
    uint32_t last_rx_ms;
    uint8_t resync_index;
    bool resync_quiet;
    uint8_t resync_reply[41];
    bool resync_reply_pending;
    bool drain_failed;
    bool async_stopped;
    uint16_t p2_gap_mask;
    uint16_t p2_tokens;
    uint16_t p3_tokens;
    uint16_t p2_token_remainder;
    uint16_t p3_token_remainder;
    uint32_t token_updated_ms;
    uint32_t current_rpc_sequence;
    uint32_t last_protocol_error_ms;
    bool protocol_error_sent;
    volatile uint32_t signaled_emergency_epoch;
    uint32_t handled_emergency_epoch;
} M6Link_t;

static M6Link_t s_link;

_Static_assert(sizeof(ReplayEntry_t) <= 152U,
               "frozen replay entry RAM bound exceeded");
_Static_assert(sizeof(s_link) <= 2560U,
               "frozen m6-v2 new static RAM bound exceeded");

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t get_u64(const uint8_t *p)
{
    uint64_t value = 0U;
    for (uint8_t i = 0U; i < 8U; i++) value |= (uint64_t)p[i] << (8U * i);
    return value;
}

static void set_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void set_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void set_u64(uint8_t *p, uint64_t value)
{
    for (uint8_t i = 0U; i < 8U; i++) p[i] = (uint8_t)(value >> (8U * i));
}

static uint8_t count_occupied(const uint8_t *slots, uint8_t count,
                              size_t stride, size_t occupied_offset)
{
    uint8_t used = 0U;
    for (uint8_t i = 0U; i < count; i++)
        used += slots[(size_t)i * stride + occupied_offset] != 0U;
    return used;
}

static uint8_t lowest_bit(uint64_t value)
{
    uint8_t bit = 0U;
    while ((bit < 64U) && ((value & (UINT64_C(1) << bit)) == 0U)) bit++;
    return bit;
}

static void enter_resync(uint32_t now_ms)
{
    s_link.stats.mode = M6_LINK_V1_RESYNC;
    s_link.stats.session = 0U;
    s_link.enabled_capabilities = 0U;
    s_link.mode_started_ms = now_ms;
    s_link.last_rx_ms = now_ms;
    s_link.resync_index = 0U;
    s_link.resync_quiet = false;
    s_link.control_pending = false;
    s_link.async_stopped = false;
    if (!s_link.in_flight) {
        s_link.flight_kind = FLIGHT_NONE;
        s_link.wire_length = 0U;
    }
    memset(s_link.p1, 0, sizeof(s_link.p1));
    memset(s_link.p2, 0, sizeof(s_link.p2));
    memset(s_link.p3, 0, sizeof(s_link.p3));
    M6Parser_Init(&s_link.parser, s_link.parser.emergency_epoch,
                  s_link.parser.overflow_epoch);
}

static void build_ready(M6Result_t result, uint32_t detail,
                        uint64_t required, uint64_t optional)
{
    uint8_t *p = s_link.ready.payload;
    bool success = result == M6_RESULT_OK;
    memset(&s_link.ready, 0, sizeof(s_link.ready));
    s_link.ready.version = M6_WIRE_VERSION;
    s_link.ready.type = M6_FRAME_READY;
    s_link.ready.payload_length = 64U;
    set_u32(&p[0], M6_READY_MAGIC);
    set_u16(&p[4], success ? M6_REGISTRY_REVISION : 0U);
    set_u16(&p[6], (uint16_t)result);
    set_u32(&p[8], detail);
    set_u64(&p[12], s_link.challenge);
    if (success) {
        s_link.config.session_generation++;
        s_link.stats.session = M6Wire_DeriveSession(
            s_link.challenge, s_link.config.boot_id,
            s_link.config.raw_reset_cause, s_link.config.session_generation,
            s_link.config.parameter_schema);
        s_link.enabled_capabilities = s_link.config.capabilities &
            (required | optional);
        set_u32(&p[20], s_link.stats.session);
    }
    set_u32(&p[24], s_link.config.boot_id);
    set_u32(&p[28], s_link.config.raw_reset_cause);
    set_u32(&p[32], s_link.config.session_generation);
    set_u16(&p[36], s_link.config.parameter_schema);
    set_u64(&p[40], s_link.config.capabilities);
    if (success) {
        set_u64(&p[48], s_link.enabled_capabilities);
        set_u16(&p[56], M6_MAX_PAYLOAD);
        set_u16(&p[58], M6_MAX_RPC_REQUEST_PAYLOAD);
        p[60] = 1U;
    }
    s_link.ready_pending = true;
    s_link.ready_success = success;
}

static void handle_hello(const M6Frame_t *frame, uint32_t now_ms)
{
    const uint8_t *p = frame->payload;
    uint64_t required;
    uint64_t optional;
    uint64_t missing;
    M6Result_t result = M6_RESULT_OK;
    uint32_t detail = 0U;

    if ((frame->flags != 0U) || (frame->session != 0U) ||
        (frame->sequence != 0U) || (frame->payload_length != 40U)) {
        s_link.stats.handshake_errors++;
        enter_resync(now_ms);
        return;
    }
    required = get_u64(&p[16]);
    optional = get_u64(&p[24]);
    /* Frozen validation precedence. */
    if (get_u32(&p[0]) != M6_HELLO_MAGIC) {
        result = M6_RESULT_MALFORMED;
    } else if (get_u64(&p[8]) != s_link.challenge) {
        result = M6_RESULT_INVALID_ARGUMENT;
        detail = 8U;
    } else if (get_u16(&p[4]) > get_u16(&p[6])) {
        result = M6_RESULT_INVALID_ARGUMENT;
        detail = 4U;
    } else if ((p[37] | p[38] | p[39]) != 0U) {
        result = M6_RESULT_MALFORMED;
        detail = p[37] != 0U ? 37U : (p[38] != 0U ? 38U : 39U);
    } else if (p[36] != 1U) {
        result = M6_RESULT_BAD_LENGTH;
        detail = (1UL << 16) | p[36];
    } else if (!((get_u16(&p[4]) <= M6_REGISTRY_REVISION) &&
                 (get_u16(&p[6]) >= M6_REGISTRY_REVISION))) {
        result = M6_RESULT_INCOMPATIBLE_REGISTRY;
        detail = M6_REGISTRY_REVISION;
    } else if ((missing = M6_HANDSHAKE_REQUIRED & ~required) != 0U) {
        result = M6_RESULT_UNSUPPORTED_CAPABILITY;
        detail = lowest_bit(missing);
    } else if ((required & optional) != 0U) {
        result = M6_RESULT_INVALID_ARGUMENT;
        detail = lowest_bit(required & optional);
    } else if ((missing = required & ~s_link.config.capabilities) != 0U) {
        result = M6_RESULT_UNSUPPORTED_CAPABILITY;
        detail = lowest_bit(missing);
    } else if (get_u16(&p[32]) < M6_MAX_PAYLOAD) {
        if (get_u16(&p[32]) < 64U) {
            s_link.stats.handshake_errors++;
            enter_resync(now_ms);
            return;
        }
        result = M6_RESULT_BAD_LENGTH;
        detail = ((uint32_t)M6_MAX_PAYLOAD << 16) | get_u16(&p[32]);
    } else if (get_u16(&p[34]) < M6_MAX_RPC_REQUEST_PAYLOAD) {
        result = M6_RESULT_BAD_LENGTH;
        detail = ((uint32_t)M6_MAX_RPC_REQUEST_PAYLOAD << 16) |
            get_u16(&p[34]);
    }
    if (result != M6_RESULT_OK) s_link.stats.handshake_errors++;
    build_ready(result, detail, required, optional);
}

static ReplayEntry_t *find_sequence(uint32_t sequence)
{
    for (uint8_t i = 0U; i < M6_REPLAY_ENTRY_COUNT; i++)
        if ((s_link.replay[i].state != REPLAY_FREE) &&
            (s_link.replay[i].sequence == sequence)) return &s_link.replay[i];
    return NULL;
}

static ReplayEntry_t *reserve_replay(void)
{
    ReplayEntry_t *oldest = NULL;
    for (uint8_t i = 0U; i < M6_REPLAY_ENTRY_COUNT; i++) {
        ReplayEntry_t *entry = &s_link.replay[i];
        if (entry->state == REPLAY_FREE) return entry;
        if ((entry->state == REPLAY_COMPLETED) &&
            ((oldest == NULL) || (entry->age < oldest->age))) oldest = entry;
    }
    return oldest;
}

static bool rpc_unfinished(void)
{
    for (uint8_t i = 0U; i < M6_REPLAY_ENTRY_COUNT; i++)
        if ((s_link.replay[i].state == REPLAY_QUEUED) ||
            (s_link.replay[i].state == REPLAY_IN_FLIGHT) ||
            (s_link.replay[i].state == REPLAY_PENDING)) return true;
    return false;
}

static void queue_control_response(uint32_t sequence, uint16_t opcode,
                                   M6Result_t result, uint32_t detail)
{
    if (s_link.control_pending) {
        s_link.stats.control_error_drop++;
        return;
    }
    memset(&s_link.control, 0, sizeof(s_link.control));
    s_link.control.version = M6_WIRE_VERSION;
    s_link.control.type = M6_FRAME_RPC_RESPONSE;
    s_link.control.session = s_link.stats.session;
    s_link.control.sequence = sequence;
    s_link.control.payload_length = 8U;
    set_u16(&s_link.control.payload[0], opcode);
    set_u16(&s_link.control.payload[2], (uint16_t)result);
    set_u32(&s_link.control.payload[4], detail);
    s_link.control_pending = true;
}

static void queue_protocol_error(const M6Frame_t *frame, M6Result_t result,
                                 uint32_t detail, uint32_t now_ms)
{
    if (s_link.control_pending ||
        (s_link.protocol_error_sent &&
         ((uint32_t)(now_ms - s_link.last_protocol_error_ms) < 100U))) {
        s_link.stats.control_error_drop++;
        return;
    }
    memset(&s_link.control, 0, sizeof(s_link.control));
    s_link.control.version = M6_WIRE_VERSION;
    s_link.control.type = M6_FRAME_PROTOCOL_ERROR;
    s_link.control.session = s_link.stats.session;
    s_link.control.sequence = frame->sequence;
    s_link.control.payload_length = 8U;
    set_u16(&s_link.control.payload[0], (uint16_t)result);
    s_link.control.payload[2] = frame->type;
    s_link.control.payload[3] = frame->flags;
    set_u32(&s_link.control.payload[4], detail);
    s_link.control_pending = true;
    s_link.protocol_error_sent = true;
    s_link.last_protocol_error_ms = now_ms;
}

static void execute_rpc(ReplayEntry_t *entry)
{
    uint16_t opcode = get_u16(entry->request);
    uint16_t success_length = M6_MAX_PAYLOAD - 8U;
    uint32_t detail = 0U;
    M6Result_t result = M6_RESULT_UNSUPPORTED_OPCODE;
    s_link.current_rpc_sequence = entry->sequence;
    if (s_link.executor != NULL)
        result = s_link.executor(opcode, &entry->request[2],
            (uint16_t)(entry->request_length - 2U), &entry->response[8],
            &success_length, &detail, s_link.context);
    s_link.current_rpc_sequence = 0U;
    if (result == M6_EXECUTOR_PENDING) {
        entry->response_length = 0U;
        entry->state = REPLAY_PENDING;
        return;
    }
    if ((result != M6_RESULT_OK) ||
        (success_length > (M6_MAX_PAYLOAD - 8U))) success_length = 0U;
    set_u16(&entry->response[0], opcode);
    set_u16(&entry->response[2], (uint16_t)result);
    set_u32(&entry->response[4], detail);
    entry->response_length = (uint8_t)(8U + success_length);
    entry->state = REPLAY_QUEUED;
}

static void handle_rpc(const M6Frame_t *frame, uint32_t now_ms)
{
    ReplayEntry_t *entry;
    uint16_t opcode = frame->payload_length >= 2U ?
        get_u16(frame->payload) : 0U;
    if (frame->flags != 0U) {
        queue_protocol_error(frame, M6_RESULT_BAD_FLAGS, frame->flags,
                             now_ms);
        return;
    }
    if ((frame->payload_length < 2U) ||
        (frame->payload_length > M6_MAX_RPC_REQUEST_PAYLOAD)) {
        queue_protocol_error(frame, M6_RESULT_BAD_LENGTH,
                             frame->payload_length, now_ms);
        return;
    }
    if (frame->sequence == 0U) {
        s_link.stats.sequence_errors++;
        queue_control_response(0U, opcode, M6_RESULT_SEQUENCE_ZERO,
                               s_link.stats.next_host_sequence);
        return;
    }
    if (frame->sequence > s_link.stats.next_host_sequence) {
        s_link.stats.sequence_errors++;
        queue_control_response(frame->sequence, opcode, M6_RESULT_SEQUENCE_GAP,
                               s_link.stats.next_host_sequence);
        return;
    }
    if (frame->sequence < s_link.stats.next_host_sequence) {
        entry = find_sequence(frame->sequence);
        if (entry == NULL) {
            s_link.stats.sequence_errors++;
            queue_control_response(frame->sequence, opcode,
                M6_RESULT_REPLAY_EXPIRED, s_link.stats.next_host_sequence);
        } else if ((entry->request_length != frame->payload_length) ||
                   (memcmp(entry->request, frame->payload,
                           frame->payload_length) != 0)) {
            s_link.stats.sequence_errors++;
            queue_control_response(frame->sequence, opcode,
                M6_RESULT_SEQUENCE_CONFLICT, s_link.stats.next_host_sequence);
        } else if (entry->state == REPLAY_COMPLETED) {
            entry->replay = 1U;
            entry->state = REPLAY_QUEUED;
            s_link.stats.replay_count++;
        }
        return;
    }
    if (rpc_unfinished()) {
        s_link.stats.busy_count++;
        queue_control_response(frame->sequence, opcode, M6_RESULT_BUSY, 1U);
        return;
    }
    if (frame->sequence > M6_HOST_LAST_SEQUENCE) {
        s_link.stats.sequence_errors++;
        queue_control_response(frame->sequence, opcode,
            M6_RESULT_SEQUENCE_EXHAUSTED, s_link.stats.next_host_sequence);
        return;
    }
    entry = reserve_replay();
    if (entry == NULL) {
        s_link.stats.busy_count++;
        queue_control_response(frame->sequence, opcode, M6_RESULT_BUSY, 1U);
        return;
    }
    memset(entry, 0, sizeof(*entry));
    entry->sequence = frame->sequence;
    entry->request_length = (uint8_t)frame->payload_length;
    memcpy(entry->request, frame->payload, frame->payload_length);
    s_link.stats.next_host_sequence++;
    execute_rpc(entry);
}

static void handle_frame(const M6Frame_t *frame, uint32_t now_ms)
{
    s_link.stats.rx_frames++;
    if (frame->version != M6_WIRE_VERSION) {
        s_link.stats.frame_errors++;
        if (s_link.stats.mode == M6_LINK_V2_ARMED) enter_resync(now_ms);
        return;
    }
    if (s_link.stats.mode == M6_LINK_V2_ARMED) {
        if (frame->type != M6_FRAME_HELLO) {
            s_link.stats.handshake_errors++;
            enter_resync(now_ms);
            return;
        }
        handle_hello(frame, now_ms);
        return;
    }
    if (s_link.stats.mode != M6_LINK_V2_ESTABLISHED) return;
    if (frame->session != s_link.stats.session) {
        s_link.stats.session_mismatch++;
        return;
    }
    if (frame->type == M6_FRAME_RPC_REQUEST) {
        handle_rpc(frame, now_ms);
        return;
    }
    if ((frame->type == M6_FRAME_HELLO) ||
        (frame->type == M6_FRAME_READY) ||
        (frame->type == M6_FRAME_RPC_RESPONSE) ||
        (frame->type == M6_FRAME_PROTOCOL_ERROR) ||
        (frame->type == M6_FRAME_EVENT) ||
        (frame->type == M6_FRAME_TELEMETRY) ||
        (frame->type == M6_FRAME_BULK))
        queue_protocol_error(frame, M6_RESULT_UNSUPPORTED_TYPE, frame->type,
                             now_ms);
    else
        queue_protocol_error(frame, M6_RESULT_UNSUPPORTED_TYPE, frame->type,
                             now_ms);
}

static int find_slot(uint8_t *slots, uint8_t count, size_t stride,
                     size_t topic_offset, size_t occupied_offset,
                     uint16_t topic)
{
    for (uint8_t i = 0U; i < count; i++) {
        uint8_t *slot = &slots[(size_t)i * stride];
        uint16_t slot_topic;
        memcpy(&slot_topic, &slot[topic_offset], sizeof(slot_topic));
        if (slot[occupied_offset] && (slot_topic == topic)) return (int)i;
    }
    return -1;
}

static int free_slot(uint8_t *slots, uint8_t count, size_t stride,
                     size_t occupied_offset)
{
    for (uint8_t i = 0U; i < count; i++)
        if (!slots[(size_t)i * stride + occupied_offset]) return (int)i;
    return -1;
}

static bool store_async(uint8_t *slots, uint8_t count, size_t stride,
                        size_t payload_capacity, uint16_t topic,
                        uint8_t type, const uint8_t *payload, uint16_t length,
                        bool coalesce, bool event_occurrence,
                        uint32_t *drop, uint8_t *high_water)
{
    EventSlot_t *slot;
    uint16_t occurrence = 0U;
    bool merged = false;
    int index;
    if (length > payload_capacity) return false;
    index = coalesce ? find_slot(slots, count, stride,
        offsetof(EventSlot_t, topic), offsetof(EventSlot_t, occupied), topic) : -1;
    if (index >= 0) {
        merged = true;
        slot = (EventSlot_t *)&slots[(size_t)index * stride];
        if (event_occurrence && (slot->length >= 12U))
            occurrence = get_u16(&slot->payload[8]);
        slot->flags |= M6_FLAG_GAP_BEFORE;
        (*drop)++;
    } else {
        index = free_slot(slots, count, stride,
                          offsetof(EventSlot_t, occupied));
        if (index < 0) {
            (*drop)++;
            return false;
        }
        slot = (EventSlot_t *)&slots[(size_t)index * stride];
    }
    slot->topic = topic;
    slot->type = type;
    slot->length = (uint8_t)length;
    slot->occupied = 1U;
    memcpy(slot->payload, payload, length);
    if (event_occurrence && (length >= 12U)) {
        if (occurrence == 0U) occurrence = get_u16(&slot->payload[8]);
        if (occurrence == 0U) occurrence = 1U;
        else if (merged && (occurrence != 0xFFFFU)) occurrence++;
        set_u16(&slot->payload[8], occurrence);
    }
    {
        uint8_t used = count_occupied(slots, count, stride,
                                     offsetof(EventSlot_t, occupied));
        if (used > *high_water) *high_water = used;
    }
    return true;
}

bool M6Link_Publish(M6TxPriority_t priority, uint8_t frame_type,
                    uint16_t topic, const uint8_t *payload,
                    uint16_t payload_length)
{
    if ((s_link.stats.mode != M6_LINK_V2_ESTABLISHED) ||
        (payload == NULL) || (payload_length > M6_MAX_PAYLOAD)) return false;
    if (priority == M6_TX_P0) {
        if ((topic < 1U) || (topic > M6_P0_TOPIC_COUNT)) return false;
        return store_async((uint8_t *)s_link.p0, M6_P0_TOPIC_COUNT,
            sizeof(s_link.p0[0]), sizeof(s_link.p0[0].payload), topic,
            frame_type, payload, payload_length, true, true,
            &s_link.stats.p0_drop,
            &s_link.stats.p0_high_water);
    }
    if (priority == M6_TX_P1)
        return store_async((uint8_t *)s_link.p1, M6_P1_SLOT_COUNT,
            sizeof(s_link.p1[0]), sizeof(s_link.p1[0].payload), topic,
            frame_type, payload, payload_length, true, true,
            &s_link.stats.p1_drop,
            &s_link.stats.p1_high_water);
    if (priority == M6_TX_P2) {
        bool stored = store_async((uint8_t *)s_link.p2, M6_P2_SLOT_COUNT,
            sizeof(s_link.p2[0]), sizeof(s_link.p2[0].payload), topic,
            frame_type, payload, payload_length, true, false,
            &s_link.stats.p2_drop,
            &s_link.stats.p2_high_water);
        int index = find_slot((uint8_t *)s_link.p2, M6_P2_SLOT_COUNT,
            sizeof(s_link.p2[0]), offsetof(TelemetrySlot_t, topic),
            offsetof(TelemetrySlot_t, occupied), topic);
        if (stored && (index >= 0) && (topic >= 1U) && (topic <= 16U) &&
            ((s_link.p2_gap_mask & (uint16_t)(1U << (topic - 1U))) != 0U)) {
            s_link.p2[index].flags |= M6_FLAG_GAP_BEFORE;
            s_link.p2_gap_mask &= (uint16_t)~(1U << (topic - 1U));
        }
        return stored;
    }
    return store_async((uint8_t *)s_link.p3, M6_P3_SLOT_COUNT,
        sizeof(s_link.p3[0]), sizeof(s_link.p3[0].payload), topic,
        frame_type, payload, payload_length, false, false,
        &s_link.stats.p3_drop,
        &s_link.stats.p3_high_water);
}

bool M6Link_CompletePending(uint32_t sequence, M6Result_t result,
                            uint32_t detail, const uint8_t *success_body,
                            uint16_t success_length)
{
    ReplayEntry_t *entry = find_sequence(sequence);
    uint16_t opcode;
    if ((entry == NULL) || (entry->state != REPLAY_PENDING) ||
        (result == M6_EXECUTOR_PENDING) ||
        (success_length > (M6_MAX_PAYLOAD - 8U)) ||
        ((success_length != 0U) && (success_body == NULL))) return false;
    opcode = get_u16(entry->request);
    if (result != M6_RESULT_OK) success_length = 0U;
    set_u16(&entry->response[0], opcode);
    set_u16(&entry->response[2], (uint16_t)result);
    set_u32(&entry->response[4], detail);
    if (success_length != 0U)
        memcpy(&entry->response[8], success_body, success_length);
    entry->response_length = (uint8_t)(8U + success_length);
    entry->state = REPLAY_QUEUED;
    return true;
}

uint8_t M6Link_ClearTelemetry(uint16_t stream_mask, M6GapReason_t reason)
{
    uint8_t cleared = 0U;
    (void)reason;
    for (uint8_t i = 0U; i < M6_P2_SLOT_COUNT; i++) {
        uint16_t topic;
        if (!s_link.p2[i].occupied) continue;
        topic = s_link.p2[i].topic;
        if ((topic < 1U) || (topic > 16U) ||
            ((stream_mask & (uint16_t)(1U << (topic - 1U))) == 0U))
            continue;
        memset(&s_link.p2[i], 0, sizeof(s_link.p2[i]));
        s_link.p2_gap_mask |= (uint16_t)(1U << (topic - 1U));
        s_link.stats.p2_drop++;
        cleared++;
    }
    if (!s_link.in_flight && (s_link.flight_kind == FLIGHT_P2) &&
        (s_link.flight_topic >= 1U) && (s_link.flight_topic <= 16U) &&
        ((stream_mask &
          (uint16_t)(1U << (s_link.flight_topic - 1U))) != 0U)) {
        s_link.p2_gap_mask |=
            (uint16_t)(1U << (s_link.flight_topic - 1U));
        s_link.stats.p2_drop++;
        s_link.flight_kind = FLIGHT_NONE;
        s_link.wire_length = 0U;
        cleared++;
    }
    return cleared;
}

static bool choose_async(uint8_t *slots, uint8_t count, size_t stride,
                         FlightKind_t kind, bool clear, M6Frame_t *frame)
{
    for (uint8_t i = 0U; i < count; i++) {
        EventSlot_t *slot = (EventSlot_t *)&slots[(size_t)i * stride];
        if (!slot->occupied) continue;
        memset(frame, 0, sizeof(*frame));
        frame->version = M6_WIRE_VERSION;
        frame->type = slot->type;
        frame->flags = slot->flags;
        frame->session = s_link.stats.session;
        frame->sequence = s_link.stats.next_async_sequence;
        frame->payload_length = slot->length;
        memcpy(frame->payload, slot->payload, slot->length);
        s_link.flight_topic = slot->topic;
        if (clear) memset(slot, 0, stride);
        s_link.flight_kind = kind;
        s_link.flight_index = i;
        return true;
    }
    return false;
}

static bool choose_frame(M6Frame_t *frame, uint32_t now_ms)
{
    if (s_link.ready_pending) {
        *frame = s_link.ready;
        s_link.flight_kind = s_link.ready_success ?
            FLIGHT_READY_SUCCESS : FLIGHT_READY_FAILURE;
        return true;
    }
    if (!s_link.async_stopped &&
        (s_link.stats.next_async_sequence != M6_ASYNC_TERMINAL_SEQUENCE) &&
        choose_async((uint8_t *)s_link.p0, M6_P0_TOPIC_COUNT,
                     sizeof(s_link.p0[0]), FLIGHT_P0, true, frame))
        return true;
    for (uint8_t i = 0U; i < M6_REPLAY_ENTRY_COUNT; i++) {
        ReplayEntry_t *entry = &s_link.replay[i];
        if (entry->state != REPLAY_QUEUED) continue;
        memset(frame, 0, sizeof(*frame));
        frame->version = M6_WIRE_VERSION;
        frame->type = M6_FRAME_RPC_RESPONSE;
        frame->flags = entry->replay ? M6_FLAG_REPLAY : 0U;
        frame->session = s_link.stats.session;
        frame->sequence = entry->sequence;
        frame->payload_length = entry->response_length;
        memcpy(frame->payload, entry->response, entry->response_length);
        entry->state = REPLAY_IN_FLIGHT;
        s_link.flight_kind = FLIGHT_REPLAY;
        s_link.flight_index = i;
        return true;
    }
    if (s_link.control_pending) {
        *frame = s_link.control;
        s_link.flight_kind = FLIGHT_CONTROL;
        return true;
    }
    if (!s_link.async_stopped &&
        (s_link.stats.next_async_sequence == M6_ASYNC_TERMINAL_SEQUENCE)) {
        memset(frame, 0, sizeof(*frame));
        frame->version = M6_WIRE_VERSION;
        frame->type = M6_FRAME_EVENT;
        frame->session = s_link.stats.session;
        frame->sequence = M6_ASYNC_TERMINAL_SEQUENCE;
        frame->payload_length = 28U;
        set_u16(&frame->payload[0], 8U); /* LINK_STATE */
        frame->payload[2] = 1U;         /* WARNING */
        frame->payload[3] = 1U;
        set_u32(&frame->payload[4], now_ms);
        set_u16(&frame->payload[8], 1U);
        frame->payload[10] = 16U;
        frame->payload[12] = (uint8_t)M6_LINK_V2_ESTABLISHED;
        frame->payload[13] = 7U;        /* SEQUENCE_EXHAUSTED */
        set_u32(&frame->payload[16], s_link.stats.session);
        set_u32(&frame->payload[20], s_link.stats.next_host_sequence);
        set_u32(&frame->payload[24], 0xFFFFFFFFUL);
        s_link.stats.next_async_sequence = 0xFFFFFFFFUL;
        s_link.flight_kind = FLIGHT_TERMINAL;
        return true;
    }
    if (s_link.async_stopped) return false;
    if (choose_async((uint8_t *)s_link.p1, M6_P1_SLOT_COUNT,
                     sizeof(s_link.p1[0]), FLIGHT_P1, true, frame))
        return true;
    if (choose_async((uint8_t *)s_link.p2, M6_P2_SLOT_COUNT,
                     sizeof(s_link.p2[0]), FLIGHT_P2, true, frame))
        return true;
    return choose_async((uint8_t *)s_link.p3, M6_P3_SLOT_COUNT,
                        sizeof(s_link.p3[0]), FLIGHT_P3, false, frame);
}

static void complete_flight(uint32_t now_ms)
{
    bool close_after = false;
    switch (s_link.flight_kind) {
    case FLIGHT_READY_SUCCESS:
        s_link.ready_pending = false;
        s_link.stats.mode = M6_LINK_V2_ESTABLISHED;
        s_link.stats.next_host_sequence = 1U;
        s_link.stats.next_async_sequence = 1U;
        s_link.async_stopped = false;
        s_link.p2_gap_mask = 0U;
        break;
    case FLIGHT_READY_FAILURE:
        s_link.ready_pending = false;
        enter_resync(now_ms);
        break;
    case FLIGHT_REPLAY:
        close_after = get_u16(&s_link.replay[s_link.flight_index].response[0]) ==
            M6_RPC_SESSION_CLOSE &&
            get_u16(&s_link.replay[s_link.flight_index].response[2]) == 0U;
        s_link.replay[s_link.flight_index].state = REPLAY_COMPLETED;
        s_link.replay[s_link.flight_index].replay = 0U;
        s_link.replay[s_link.flight_index].age = ++s_link.replay_age;
        s_link.stats.replay_used = 0U;
        for (uint8_t i = 0U; i < M6_REPLAY_ENTRY_COUNT; i++)
            s_link.stats.replay_used +=
                s_link.replay[i].state != REPLAY_FREE;
        break;
    case FLIGHT_CONTROL: s_link.control_pending = false; break;
    case FLIGHT_P0:
    case FLIGHT_P1:
    case FLIGHT_P2:
    case FLIGHT_P3:
        break;
    case FLIGHT_TERMINAL: s_link.async_stopped = true; break;
    case FLIGHT_RESYNC:
        s_link.resync_reply_pending = false;
        s_link.stats.mode = M6_LINK_V1;
        break;
    default: break;
    }
    s_link.in_flight = false;
    s_link.flight_kind = FLIGHT_NONE;
    s_link.wire_length = 0U;
    s_link.stats.tx_frames++;
    if (close_after) {
        memset(s_link.replay, 0, sizeof(s_link.replay));
        s_link.stats.replay_used = 0U;
        enter_resync(now_ms);
    }
}

static void service_tx(uint32_t now_ms)
{
    M6Frame_t frame;
    size_t length;
    if (s_link.in_flight) {
        if ((s_link.tx_idle != NULL) && s_link.tx_idle(s_link.context))
            complete_flight(now_ms);
        return;
    }
    if (s_link.flight_kind != FLIGHT_NONE) {
        if ((s_link.write != NULL) &&
            s_link.write(s_link.wire, s_link.wire_length, s_link.context))
            s_link.in_flight = true;
        return;
    }
    if (s_link.resync_reply_pending) {
        memcpy(s_link.wire, s_link.resync_reply, sizeof(s_link.resync_reply));
        s_link.wire_length = sizeof(s_link.resync_reply);
        s_link.flight_kind = FLIGHT_RESYNC;
        return;
    }
    if ((s_link.stats.mode == M6_LINK_V1) ||
        (s_link.stats.mode == M6_LINK_V2_DRAIN) ||
        (s_link.stats.mode == M6_LINK_V1_RESYNC)) return;
    if (!choose_frame(&frame, now_ms)) return;
    length = M6Wire_Encode(&frame, s_link.wire, sizeof(s_link.wire));
    if (length == 0U) {
        if (s_link.flight_kind == FLIGHT_REPLAY)
            s_link.replay[s_link.flight_index].state = REPLAY_QUEUED;
        s_link.flight_kind = FLIGHT_NONE;
        return;
    }
    if (s_link.flight_kind == FLIGHT_P2) {
        if (length > s_link.p2_tokens) {
            s_link.stats.p2_drop++;
            if ((s_link.flight_topic >= 1U) && (s_link.flight_topic <= 16U))
                s_link.p2_gap_mask |=
                    (uint16_t)(1U << (s_link.flight_topic - 1U));
            s_link.flight_kind = FLIGHT_NONE;
            return;
        }
        s_link.p2_tokens = (uint16_t)(s_link.p2_tokens - length);
    } else if (s_link.flight_kind == FLIGHT_P3) {
        if (length > s_link.p3_tokens) {
            s_link.flight_kind = FLIGHT_NONE;
            return;
        }
        s_link.p3_tokens = (uint16_t)(s_link.p3_tokens - length);
        memset(&s_link.p3[s_link.flight_index], 0,
               sizeof(s_link.p3[s_link.flight_index]));
    }
    if ((s_link.flight_kind == FLIGHT_P0) ||
        (s_link.flight_kind == FLIGHT_P1) ||
        (s_link.flight_kind == FLIGHT_P2) ||
        (s_link.flight_kind == FLIGHT_P3))
        s_link.stats.next_async_sequence++;
    s_link.wire_length = (uint16_t)length;
}

void M6Link_Init(const M6LinkConfig_t *config, M6RpcExecutor_t executor,
                 M6LinkWrite_t write, M6LinkTxIdle_t tx_idle, void *context,
                 uint32_t emergency_epoch, uint32_t overflow_epoch)
{
    memset(&s_link, 0, sizeof(s_link));
    if (config != NULL) s_link.config = *config;
    s_link.config.capabilities &= M6_DEVICE_CAPABILITIES;
    s_link.executor = executor;
    s_link.write = write;
    s_link.tx_idle = tx_idle;
    s_link.context = context;
    s_link.stats.mode = M6_LINK_V1;
    s_link.stats.next_host_sequence = 1U;
    s_link.stats.next_async_sequence = 1U;
    s_link.p2_tokens = 448U;
    s_link.p3_tokens = 224U;
    s_link.signaled_emergency_epoch = emergency_epoch;
    s_link.handled_emergency_epoch = emergency_epoch;
    M6Parser_Init(&s_link.parser, emergency_epoch, overflow_epoch);
}

M6LinkMode_t M6Link_Mode(void) { return s_link.stats.mode; }
uint64_t M6Link_Capabilities(void) { return s_link.config.capabilities; }
uint64_t M6Link_EnabledCapabilities(void)
{
    return s_link.enabled_capabilities;
}

bool M6Link_BeginV2(uint64_t challenge, uint32_t now_ms)
{
    if ((s_link.stats.mode != M6_LINK_V1) || (challenge == 0U)) return false;
    s_link.challenge = challenge;
    s_link.drain_failed = false;
    s_link.stats.mode = M6_LINK_V2_DRAIN;
    s_link.mode_started_ms = now_ms;
    return true;
}

bool M6Link_DrainFailed(void) { return s_link.drain_failed; }

void M6Link_NotifyV1ReadyComplete(uint32_t now_ms)
{
    if (s_link.stats.mode != M6_LINK_V2_DRAIN) return;
    if (s_link.drain_failed) {
        enter_resync(now_ms);
        return;
    }
    s_link.stats.mode = M6_LINK_V2_ARMED;
    s_link.mode_started_ms = now_ms;
    M6Parser_Init(&s_link.parser, s_link.parser.emergency_epoch,
                  s_link.parser.overflow_epoch);
}

static void process_resync(uint8_t byte, uint32_t now_ms)
{
    static const uint8_t probe[] = ":1,get,meta.capabilities\r\n";
    static const char hex[] = "0123456789abcdef";
    if ((uint32_t)(now_ms - s_link.last_rx_ms) >= 100U)
        s_link.resync_quiet = true;
    s_link.last_rx_ms = now_ms;
    if (!s_link.resync_quiet) return;
    if (byte != probe[s_link.resync_index]) {
        s_link.resync_quiet = false;
        s_link.resync_index = 0U;
        return;
    }
    s_link.resync_index++;
    if (s_link.resync_index != sizeof(probe) - 1U) return;
    memcpy(s_link.resync_reply, "ok,1,meta.capabilities,", 23U);
    for (uint8_t i = 0U; i < 16U; i++)
        s_link.resync_reply[23U + i] =
            (uint8_t)hex[(s_link.config.capabilities >> ((15U - i) * 4U)) & 0xFU];
    s_link.resync_reply[39] = '\r';
    s_link.resync_reply[40] = '\n';
    s_link.resync_reply_pending = true;
    s_link.resync_index = 0U;
}

void M6Link_ProcessByte(uint8_t byte, uint32_t now_ms,
                        uint32_t emergency_epoch, uint32_t overflow_epoch)
{
    M6Frame_t frame;
    if (s_link.stats.mode == M6_LINK_V1_RESYNC) {
        process_resync(byte, now_ms);
        return;
    }
    if ((s_link.stats.mode != M6_LINK_V2_ARMED) &&
        (s_link.stats.mode != M6_LINK_V2_ESTABLISHED)) return;
    if (M6Parser_Feed(&s_link.parser, byte, now_ms, emergency_epoch,
                      overflow_epoch, &frame)) handle_frame(&frame, now_ms);
}

void M6Link_Service(uint32_t now_ms, uint32_t emergency_epoch,
                    uint32_t overflow_epoch)
{
    uint32_t elapsed = now_ms - s_link.token_updated_ms;
    uint32_t generated;
    uint32_t before = s_link.parser.counters.total_timeout +
        s_link.parser.counters.interbyte_timeout;
    if ((s_link.stats.mode == M6_LINK_V2_DRAIN) && !s_link.drain_failed &&
        ((uint32_t)(now_ms - s_link.mode_started_ms) >=
         M6_V1_DRAIN_TIMEOUT_MS)) {
        s_link.stats.handshake_errors++;
        s_link.drain_failed = true;
    }
    if (emergency_epoch != s_link.handled_emergency_epoch)
        s_link.signaled_emergency_epoch = emergency_epoch;
    if (s_link.handled_emergency_epoch != s_link.signaled_emergency_epoch) {
        uint8_t p2_used = count_occupied((const uint8_t *)s_link.p2,
            M6_P2_SLOT_COUNT, sizeof(s_link.p2[0]),
            offsetof(TelemetrySlot_t, occupied));
        uint8_t p3_used = count_occupied((const uint8_t *)s_link.p3,
            M6_P3_SLOT_COUNT, sizeof(s_link.p3[0]),
            offsetof(BulkSlot_t, occupied));
        s_link.stats.p2_drop += p2_used;
        s_link.stats.p3_drop += p3_used;
        if (!s_link.in_flight && (s_link.flight_kind == FLIGHT_P2)) {
            s_link.stats.p2_drop++;
            s_link.flight_kind = FLIGHT_NONE;
            s_link.wire_length = 0U;
        } else if (!s_link.in_flight && (s_link.flight_kind == FLIGHT_P3)) {
            s_link.flight_kind = FLIGHT_NONE;
            s_link.wire_length = 0U;
        }
        memset(s_link.p2, 0, sizeof(s_link.p2));
        memset(s_link.p3, 0, sizeof(s_link.p3));
        s_link.handled_emergency_epoch = s_link.signaled_emergency_epoch;
        if (s_link.stats.mode == M6_LINK_V2_ARMED) enter_resync(now_ms);
    }
    if (elapsed != 0U) {
        generated = elapsed * 7000U + s_link.p2_token_remainder;
        s_link.p2_tokens = (uint16_t)((s_link.p2_tokens + generated / 1000U) >
            448U ? 448U : s_link.p2_tokens + generated / 1000U);
        s_link.p2_token_remainder = (uint16_t)(generated % 1000U);
        generated = elapsed * 1000U + s_link.p3_token_remainder;
        s_link.p3_tokens = (uint16_t)((s_link.p3_tokens + generated / 1000U) >
            224U ? 224U : s_link.p3_tokens + generated / 1000U);
        s_link.p3_token_remainder = (uint16_t)(generated % 1000U);
        s_link.token_updated_ms = now_ms;
    }
    if ((s_link.stats.mode == M6_LINK_V2_ARMED) ||
        (s_link.stats.mode == M6_LINK_V2_ESTABLISHED)) {
        M6Parser_Tick(&s_link.parser, now_ms, emergency_epoch, overflow_epoch);
        if ((s_link.stats.mode == M6_LINK_V2_ARMED) &&
            (((uint32_t)(now_ms - s_link.mode_started_ms) >=
              M6_HELLO_TIMEOUT_MS) ||
             (before != s_link.parser.counters.total_timeout +
                        s_link.parser.counters.interbyte_timeout))) {
            s_link.stats.handshake_errors++;
            enter_resync(now_ms);
        }
    }
    service_tx(now_ms);
}

void M6Link_Emergency(uint32_t emergency_epoch)
{
    /* ISR-safe notification only.  The single-owner main service performs
     * parser and queue mutation after Safety has already stopped the motors. */
    s_link.signaled_emergency_epoch = emergency_epoch;
}

void M6Link_GetStats(M6LinkStats_t *stats)
{
    if (stats == NULL) return;
    s_link.stats.parser = s_link.parser.counters;
    *stats = s_link.stats;
}

uint32_t M6Link_CurrentRpcSequence(void) { return s_link.current_rpc_sequence; }
uint32_t M6Link_CurrentSession(void) { return s_link.stats.session; }

#ifdef M6_LINK_HOST_TEST
void M6Link_TestSetNextAsyncSequence(uint32_t sequence)
{
    s_link.stats.next_async_sequence = sequence;
}
void M6Link_TestSetTokens(uint16_t p2_tokens, uint16_t p3_tokens)
{
    s_link.p2_tokens = p2_tokens;
    s_link.p3_tokens = p3_tokens;
}
#endif
