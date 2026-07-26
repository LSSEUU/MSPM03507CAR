#ifndef M6_LINK_H
#define M6_LINK_H

#include "m6_wire.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define M6_REPLAY_ENTRY_COUNT 4U
#define M6_P0_TOPIC_COUNT     3U
#define M6_P1_SLOT_COUNT      4U
#define M6_P2_SLOT_COUNT      4U
#define M6_P3_SLOT_COUNT      2U

/* Executor-only sentinel. It is never serialized as an RPC result. */
#define M6_EXECUTOR_PENDING ((M6Result_t)0xFFFEU)

typedef enum {
    M6_LINK_V1 = 0,
    M6_LINK_V2_DRAIN = 1,
    M6_LINK_V2_ARMED = 2,
    M6_LINK_V2_ESTABLISHED = 3,
    M6_LINK_V1_RESYNC = 4
} M6LinkMode_t;

typedef enum {
    M6_RESULT_OK = 0,
    M6_RESULT_MALFORMED = 1,
    M6_RESULT_BAD_FLAGS = 2,
    M6_RESULT_BAD_LENGTH = 3,
    M6_RESULT_UNSUPPORTED_TYPE = 4,
    M6_RESULT_UNSUPPORTED_OPCODE = 5,
    M6_RESULT_INCOMPATIBLE_REGISTRY = 6,
    M6_RESULT_UNSUPPORTED_CAPABILITY = 7,
    M6_RESULT_SESSION_REQUIRED = 8,
    M6_RESULT_SESSION_MISMATCH = 9,
    M6_RESULT_SEQUENCE_ZERO = 10,
    M6_RESULT_SEQUENCE_GAP = 11,
    M6_RESULT_SEQUENCE_CONFLICT = 12,
    M6_RESULT_REPLAY_EXPIRED = 13,
    M6_RESULT_SEQUENCE_EXHAUSTED = 14,
    M6_RESULT_BUSY = 15,
    M6_RESULT_SESSION_CLOSING = 16,
    M6_RESULT_INVALID_ARGUMENT = 256,
    M6_RESULT_UNKNOWN_PARAMETER = 257,
    M6_RESULT_READ_ONLY = 258,
    M6_RESULT_INVALID_VALUE = 259,
    M6_RESULT_OUT_OF_RANGE = 260,
    M6_RESULT_RELATION = 261,
    M6_RESULT_STALE_ACTIVE_REVISION = 262,
    M6_RESULT_STALE_STAGE_REVISION = 263,
    M6_RESULT_STALE_GENERATION = 264,
    M6_RESULT_WRONG_STATE = 265,
    M6_RESULT_SAFETY_FAULT = 266,
    M6_RESULT_ACTION_ACTIVE = 267,
    M6_RESULT_NOT_ARMED = 268,
    M6_RESULT_ARM_EXPIRED = 269,
    M6_RESULT_SENSOR_NOT_READY = 270,
    M6_RESULT_UNSUPPORTED_RATE = 271,
    M6_RESULT_BANDWIDTH_EXCEEDED = 272,
    M6_RESULT_NOT_FOUND = 273,
    M6_RESULT_STORAGE_IO = 512,
    M6_RESULT_STORAGE_VERIFY = 513,
    M6_RESULT_STORAGE_SCHEMA = 514,
    M6_RESULT_INTERNAL = 32767
} M6Result_t;

typedef enum {
    M6_TX_P0 = 0,
    M6_TX_P1 = 1,
    M6_TX_P2 = 2,
    M6_TX_P3 = 3
} M6TxPriority_t;

typedef struct {
    M6LinkMode_t mode;
    uint32_t session;
    uint32_t next_host_sequence;
    uint32_t next_async_sequence;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t frame_errors;
    uint32_t handshake_errors;
    uint32_t session_mismatch;
    uint32_t sequence_errors;
    uint32_t replay_count;
    uint32_t busy_count;
    uint32_t p0_drop;
    uint32_t p1_drop;
    uint32_t p2_drop;
    uint32_t p3_drop;
    uint32_t control_error_drop;
    uint8_t replay_used;
    uint8_t p0_high_water;
    uint8_t p1_high_water;
    uint8_t p2_high_water;
    uint8_t p3_high_water;
    M6ParserCounters_t parser;
} M6LinkStats_t;

typedef struct {
    uint32_t boot_id;
    uint32_t raw_reset_cause;
    uint32_t session_generation;
    uint16_t parameter_schema;
    uint64_t capabilities;
} M6LinkConfig_t;

typedef M6Result_t (*M6RpcExecutor_t)(
    uint16_t opcode, const uint8_t *body, uint16_t body_length,
    uint8_t *success_body, uint16_t *success_length, uint32_t *detail,
    void *context);
typedef bool (*M6LinkWrite_t)(const uint8_t *data, uint16_t length,
                              void *context);
typedef bool (*M6LinkTxIdle_t)(void *context);

void M6Link_Init(const M6LinkConfig_t *config, M6RpcExecutor_t executor,
                 M6LinkWrite_t write, M6LinkTxIdle_t tx_idle, void *context,
                 uint32_t emergency_epoch, uint32_t overflow_epoch);
M6LinkMode_t M6Link_Mode(void);
uint64_t M6Link_Capabilities(void);
uint64_t M6Link_EnabledCapabilities(void);
bool M6Link_BeginV2(uint64_t challenge, uint32_t now_ms);
bool M6Link_DrainFailed(void);
void M6Link_NotifyV1ReadyComplete(uint32_t now_ms);
void M6Link_ProcessByte(uint8_t byte, uint32_t now_ms,
                        uint32_t emergency_epoch, uint32_t overflow_epoch);
void M6Link_Service(uint32_t now_ms, uint32_t emergency_epoch,
                    uint32_t overflow_epoch);
bool M6Link_Publish(M6TxPriority_t priority, uint8_t frame_type,
                    uint16_t topic, const uint8_t *payload,
                    uint16_t payload_length);
bool M6Link_CompletePending(uint32_t sequence, M6Result_t result,
                            uint32_t detail, const uint8_t *success_body,
                            uint16_t success_length);
uint8_t M6Link_ClearTelemetry(uint16_t stream_mask, M6GapReason_t reason);
void M6Link_Emergency(uint32_t emergency_epoch);
void M6Link_GetStats(M6LinkStats_t *stats);
uint32_t M6Link_CurrentRpcSequence(void);
uint32_t M6Link_CurrentSession(void);

#ifdef M6_LINK_HOST_TEST
void M6Link_TestSetNextAsyncSequence(uint32_t sequence);
void M6Link_TestSetTokens(uint16_t p2_tokens, uint16_t p3_tokens);
#endif

#endif
