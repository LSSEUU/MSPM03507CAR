#ifndef M6_WIRE_H
#define M6_WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Generated contract constants: Tests/vectors/m6_v2_wire_v1.json
 * registry-sha256: 8f47a5525b9fc7ef348c070d359c07ccf59c366d7703174af05274f6632dec67 */
#define M6_WIRE_VERSION                 2U
#define M6_REGISTRY_REVISION            1U
#define M6_FRAME_DELIMITER              0x7EU
#define M6_FRAME_ESCAPE                 0x7DU
#define M6_ESCAPE_XOR                   0x20U
#define M6_FRAME_HEADER_SIZE            13U
#define M6_FRAME_CRC_SIZE               2U
#define M6_MAX_PAYLOAD                  96U
#define M6_MAX_RPC_REQUEST_PAYLOAD      32U
#define M6_MAX_DECODED_FRAME            111U
#define M6_MAX_WIRE_FRAME               224U
#define M6_INTERBYTE_TIMEOUT_MS         50U
#define M6_TOTAL_TIMEOUT_MS             100U
#define M6_HELLO_TIMEOUT_MS             500U
#define M6_V1_DRAIN_TIMEOUT_MS          250U
#define M6_HANDSHAKE_REQUIRED           UINT64_C(0x000000000000600F)
/* Project extension: duration 0 means continuous only for LINE_FOLLOW. */
#define M6_LINE_ACTION_CONTINUOUS_DURATION_MS 0U

#define M6_CAP_WIRE_R1                  (UINT64_C(1) << 0)
#define M6_CAP_RAW_ESTOP_S_SPACE        (UINT64_C(1) << 1)
#define M6_CAP_SESSION_DEDUP             (UINT64_C(1) << 2)
#define M6_CAP_PRIORITY_TX               (UINT64_C(1) << 3)
#define M6_CAP_RESET_DIAG                (UINT64_C(1) << 4)
#define M6_CAP_HEALTH_STATS              (UINT64_C(1) << 5)
#define M6_CAP_PARAM_READ                (UINT64_C(1) << 6)
#define M6_CAP_PARAM_STAGE               (UINT64_C(1) << 7)
#define M6_CAP_PARAM_BOUNDARY_APPLY      (UINT64_C(1) << 8)
#define M6_CAP_PARAM_PERSIST             (UINT64_C(1) << 9)
#define M6_CAP_TELEMETRY_SUBSCRIBE       (UINT64_C(1) << 10)
#define M6_CAP_ACTION_TWO_PHASE          (UINT64_C(1) << 11)
#define M6_CAP_LEGACY_V1                 (UINT64_C(1) << 13)
#define M6_CAP_HANDSHAKE_RESYNC          (UINT64_C(1) << 14)
#define M6_CAP_FAULT_CLEAR               (UINT64_C(1) << 15)
#define M6_CAP_WHEEL_DIAG_ACTION         (UINT64_C(1) << 16)
#define M6_CAP_HEADING_ACTION            (UINT64_C(1) << 17)
#define M6_CAP_LINE_ACTION               (UINT64_C(1) << 18)
#define M6_CAP_HEADING_CALIBRATION       (UINT64_C(1) << 20)
#define M6_CAP_CONFIGURABLE_TURN         (UINT64_C(1) << 22)
#define M6_CAP_HEADING_RATE_ACTION       (UINT64_C(1) << 24)
#define M6_CAP_MANUAL_DRIVE              (UINT64_C(1) << 25)

/* Deliberately excludes trace/bulk/capture/self-test. */
#define M6_DEVICE_CAPABILITIES (M6_HANDSHAKE_REQUIRED | M6_CAP_RESET_DIAG | \
    M6_CAP_HEALTH_STATS | M6_CAP_PARAM_READ | M6_CAP_PARAM_STAGE | \
    M6_CAP_PARAM_BOUNDARY_APPLY | M6_CAP_PARAM_PERSIST | \
    M6_CAP_TELEMETRY_SUBSCRIBE | M6_CAP_ACTION_TWO_PHASE | \
    M6_CAP_FAULT_CLEAR | M6_CAP_WHEEL_DIAG_ACTION | M6_CAP_HEADING_ACTION | \
    M6_CAP_LINE_ACTION | \
    M6_CAP_HEADING_CALIBRATION | M6_CAP_CONFIGURABLE_TURN | \
    M6_CAP_HEADING_RATE_ACTION | M6_CAP_MANUAL_DRIVE)

typedef enum {
    M6_FRAME_HELLO = 0x01,
    M6_FRAME_READY = 0x02,
    M6_FRAME_RPC_REQUEST = 0x10,
    M6_FRAME_RPC_RESPONSE = 0x11,
    M6_FRAME_PROTOCOL_ERROR = 0x12,
    M6_FRAME_EVENT = 0x20,
    M6_FRAME_TELEMETRY = 0x30,
    M6_FRAME_BULK = 0x31
} M6FrameType_t;

typedef enum {
    M6_RPC_PING = 0x0001,
    M6_RPC_IDENTITY_GET = 0x0002,
    M6_RPC_STATUS_GET = 0x0003,
    M6_RPC_RESET_GET = 0x0004,
    M6_RPC_TASK_STATS_GET = 0x0005,
    M6_RPC_UART_STATS_GET = 0x0006,
    M6_RPC_LINK_STATS_GET = 0x0007,
    M6_RPC_ACTION_GET = 0x0008,
    M6_RPC_SUBSCRIPTION_GET = 0x0009,
    M6_RPC_SNAPSHOT_GET = 0x000A,
    M6_RPC_MANUAL_DRIVE_GET = 0x000B,
    M6_RPC_PARAM_SUMMARY_GET = 0x0100,
    M6_RPC_PARAM_DESCRIPTOR_GET = 0x0101,
    M6_RPC_PARAM_VALUE_GET = 0x0102,
    M6_RPC_SESSION_CLOSE = 0x8000,
    M6_RPC_FAULT_CLEAR = 0x8001,
    M6_RPC_PARAM_STAGE_SET = 0x8100,
    M6_RPC_PARAM_APPLY = 0x8101,
    M6_RPC_PARAM_REVERT = 0x8102,
    M6_RPC_PARAM_LOAD = 0x8103,
    M6_RPC_PARAM_DEFAULTS = 0x8104,
    M6_RPC_PARAM_SAVE = 0x8105,
    M6_RPC_SUBSCRIPTION_SET = 0x8200,
    M6_RPC_ACTION_ARM = 0x8300,
    M6_RPC_ACTION_CONFIRM = 0x8301,
    M6_RPC_ACTION_CANCEL = 0x8302,
    M6_RPC_HEADING_CALIBRATE = 0x8303,
    M6_RPC_MANUAL_DRIVE_SET = 0x8305
} M6RpcOpcode_t;

typedef enum {
    M6_ACTION_KIND_NONE = 0,
    M6_ACTION_KIND_WHEEL_SPEED_TEST = 1,
    M6_ACTION_KIND_HEADING_HOLD = 2,
    M6_ACTION_KIND_HEADING_TURN = 3,
    M6_ACTION_KIND_HEADING_RATE = 4,
    M6_ACTION_KIND_LINE_FOLLOW = 5
} M6ActionKind_t;

typedef enum {
    M6_STREAM_CORE = 1,
    M6_STREAM_WHEEL = 2,
    M6_STREAM_LINE = 3,
    M6_STREAM_HEALTH = 4
} M6TelemetryStream_t;

typedef enum {
    M6_EVENT_RESET = 1,
    M6_EVENT_SAFETY_FAULT = 2,
    M6_EVENT_EMERGENCY_STOP = 3,
    M6_EVENT_ACTION_STATE = 4,
    M6_EVENT_PARAM_COMMITTED = 5,
    M6_EVENT_TELEMETRY_GAP = 6,
    M6_EVENT_STORAGE_FAULT = 7,
    M6_EVENT_LINK_STATE = 8,
    M6_EVENT_GRAYSCALE_READY = 9,
    M6_EVENT_MANUAL_DRIVE_STATE = 10
} M6EventCode_t;

typedef enum {
    M6_MANUAL_DRIVE_SOURCE_NONE = 0,
    M6_MANUAL_DRIVE_SOURCE_PB11_IR = 1,
    M6_MANUAL_DRIVE_SOURCE_UART1_VISION = 2,
    M6_MANUAL_DRIVE_SOURCE_HOST_RPC = 3
} M6ManualDriveSource_t;

typedef enum {
    M6_MANUAL_DRIVE_STOP_NONE = 0,
    M6_MANUAL_DRIVE_STOP_RELEASE = 1,
    M6_MANUAL_DRIVE_STOP_LEASE_EXPIRED = 2,
    M6_MANUAL_DRIVE_STOP_PREEMPTED = 3,
    M6_MANUAL_DRIVE_STOP_FAULT = 4,
    M6_MANUAL_DRIVE_STOP_ACTION_TAKEOVER = 5
} M6ManualDriveStopReason_t;

#define M6_MANUAL_DRIVE_FLAG_TAKEOVER   (1U << 0)
#define M6_MANUAL_DRIVE_FLAG_RELEASE    (1U << 1)

typedef enum {
    M6_GAP_QUEUE_OVERWRITE = 1,
    M6_GAP_TOKEN_BUDGET = 2,
    M6_GAP_PRODUCER_DROP = 3,
    M6_GAP_EMERGENCY_FLUSH = 4,
    M6_GAP_SESSION_TEARDOWN = 5
} M6GapReason_t;

typedef enum {
    M6_PARSE_HUNT = 0,
    M6_PARSE_COLLECT,
    M6_PARSE_ESCAPED,
    M6_PARSE_DISCARD
} M6ParserState_t;

typedef struct {
    uint32_t crc_error;
    uint32_t short_frame;
    uint32_t length_error;
    uint32_t escape_error;
    uint32_t decoded_overflow;
    uint32_t wire_overflow;
    uint32_t interbyte_timeout;
    uint32_t total_timeout;
    uint32_t epoch_abort;
} M6ParserCounters_t;

typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t flags;
    uint32_t session;
    uint32_t sequence;
    uint16_t payload_length;
    uint8_t payload[M6_MAX_PAYLOAD];
} M6Frame_t;

typedef struct {
    M6ParserState_t state;
    uint8_t decoded[M6_MAX_DECODED_FRAME];
    uint16_t decoded_length;
    uint16_t wire_length;
    uint32_t started_ms;
    uint32_t last_byte_ms;
    uint32_t emergency_epoch;
    uint32_t overflow_epoch;
    bool timing_active;
    M6ParserCounters_t counters;
} M6Parser_t;

uint16_t M6Wire_Crc16(const uint8_t *data, size_t length);
uint32_t M6Wire_Crc32(const uint8_t *data, size_t length);
uint32_t M6Wire_DeriveSession(uint64_t challenge, uint32_t boot_id,
                              uint32_t raw_reset_cause,
                              uint32_t session_generation,
                              uint16_t parameter_schema);
size_t M6Wire_Encode(const M6Frame_t *frame, uint8_t *wire,
                     size_t capacity);
void M6Parser_Init(M6Parser_t *parser, uint32_t emergency_epoch,
                   uint32_t overflow_epoch);
bool M6Parser_Feed(M6Parser_t *parser, uint8_t byte, uint32_t now_ms,
                   uint32_t emergency_epoch, uint32_t overflow_epoch,
                   M6Frame_t *frame);
void M6Parser_Tick(M6Parser_t *parser, uint32_t now_ms,
                   uint32_t emergency_epoch, uint32_t overflow_epoch);

#endif
