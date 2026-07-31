#include "zdt_stepper.h"

#include "zdt_stepper_uart.h"

#include <stddef.h>
#include <string.h>

#define ZDT_FUNCTION_ZERO_POSITION       0x0AU
#define ZDT_FUNCTION_CLEAR_PROTECTION    0x0EU
#define ZDT_FUNCTION_READ_ENCODER        0x31U
#define ZDT_FUNCTION_READ_SPEED          0x35U
#define ZDT_FUNCTION_READ_POSITION       0x36U
#define ZDT_FUNCTION_READ_MOTOR_STATUS   0x3AU
#define ZDT_FUNCTION_READ_HOME_STATUS    0x3BU
#define ZDT_FUNCTION_READ_CONFIG         0x42U
#define ZDT_FUNCTION_START_HOMING        0x9AU
#define ZDT_FUNCTION_ABORT_HOMING        0x9CU
#define ZDT_FUNCTION_ENABLE              0xF3U
#define ZDT_FUNCTION_VELOCITY            0xF6U
#define ZDT_FUNCTION_POSITION            0xFDU
#define ZDT_FUNCTION_STOP                0xFEU

#define ZDT_RESPONSE_ACCEPTED            0x02U
#define ZDT_RESPONSE_AT_ORIGIN           0x12U
#define ZDT_RESPONSE_LIMIT_ACTIVE        0x22U
#define ZDT_RESPONSE_COMPLETED           0x9FU
#define ZDT_RESPONSE_PARAMETER_ERROR     0xE2U
#define ZDT_RESPONSE_FORMAT_ERROR        0xEEU

#define ZDT_RESPONSE_BUFFER_SIZE           48U
#define ZDT_POLL_CHUNK_SIZE                32U

static ZdtStepperSnapshot_t stepper;
static uint8_t response_buffer[ZDT_RESPONSE_BUFFER_SIZE];
static uint8_t response_length;

static ZdtStepperResult_t SendFrame(const uint8_t *frame, size_t length)
{
    return ZdtStepperUart_Write(frame, length) ?
        ZDT_STEPPER_OK : ZDT_STEPPER_TX_FULL;
}

static bool IsDirectionValid(ZdtStepperDirection_t direction)
{
    return (direction == ZDT_STEPPER_DIRECTION_CW) ||
           (direction == ZDT_STEPPER_DIRECTION_CCW);
}

static bool IsPositionModeValid(ZdtStepperPositionMode_t mode)
{
    return (mode == ZDT_STEPPER_POSITION_RELATIVE_LAST_TARGET) ||
           (mode == ZDT_STEPPER_POSITION_ABSOLUTE_ZERO) ||
           (mode == ZDT_STEPPER_POSITION_RELATIVE_CURRENT);
}

static bool IsHomeModeValid(ZdtStepperHomeMode_t mode)
{
    return (uint32_t)mode <=
        (uint32_t)ZDT_STEPPER_HOME_POWER_LOSS_POSITION;
}

static bool IsActionFunction(uint8_t function)
{
    switch (function) {
        case ZDT_FUNCTION_ZERO_POSITION:
        case ZDT_FUNCTION_CLEAR_PROTECTION:
        case ZDT_FUNCTION_START_HOMING:
        case ZDT_FUNCTION_ABORT_HOMING:
        case ZDT_FUNCTION_ENABLE:
        case ZDT_FUNCTION_VELOCITY:
        case ZDT_FUNCTION_POSITION:
        case ZDT_FUNCTION_STOP:
            return true;
        default:
            return false;
    }
}

static bool IsWireResponse(uint8_t response)
{
    switch (response) {
        case ZDT_RESPONSE_ACCEPTED:
        case ZDT_RESPONSE_AT_ORIGIN:
        case ZDT_RESPONSE_LIMIT_ACTIVE:
        case ZDT_RESPONSE_COMPLETED:
        case ZDT_RESPONSE_PARAMETER_ERROR:
        case ZDT_RESPONSE_FORMAT_ERROR:
            return true;
        default:
            return false;
    }
}

static ZdtStepperResponse_t DecodeWireResponse(uint8_t response)
{
    switch (response) {
        case ZDT_RESPONSE_ACCEPTED:
            return ZDT_STEPPER_RESPONSE_ACCEPTED;
        case ZDT_RESPONSE_AT_ORIGIN:
            return ZDT_STEPPER_RESPONSE_AT_ORIGIN;
        case ZDT_RESPONSE_LIMIT_ACTIVE:
            return ZDT_STEPPER_RESPONSE_LIMIT_ACTIVE;
        case ZDT_RESPONSE_COMPLETED:
            return ZDT_STEPPER_RESPONSE_COMPLETED;
        case ZDT_RESPONSE_PARAMETER_ERROR:
            return ZDT_STEPPER_RESPONSE_PARAMETER_OR_STATE_ERROR;
        case ZDT_RESPONSE_FORMAT_ERROR:
            return ZDT_STEPPER_RESPONSE_FORMAT_ERROR;
        default:
            return ZDT_STEPPER_RESPONSE_NONE;
    }
}

static uint8_t ExpectedFrameLength(void)
{
    uint8_t function;
    if (response_length < 2U) return 0U;
    function = response_buffer[1];

    if (IsActionFunction(function)) return 4U;
    if ((function == ZDT_FUNCTION_READ_MOTOR_STATUS) ||
        (function == ZDT_FUNCTION_READ_HOME_STATUS)) return 4U;
    if (function == ZDT_FUNCTION_READ_ENCODER) {
        if ((response_length >= 4U) &&
            (response_buffer[3] == ZDT_STEPPER_CHECK_BYTE) &&
            IsWireResponse(response_buffer[2])) return 4U;
        return 5U;
    }
    if (function == ZDT_FUNCTION_READ_CONFIG) {
        /* 0x42 replies are addr,function,length,payload...,check.  Keep
         * accepting the standard four-byte error response as well. */
        if ((response_length >= 3U) &&
            IsWireResponse(response_buffer[2])) {
            if (response_length < 4U) return 0xFEU;
            return response_buffer[3] == ZDT_STEPPER_CHECK_BYTE ? 4U : 0xFFU;
        }
        if (response_length < 3U) return 0xFEU;
        /* Emm V5.0 defines this byte as the complete response length,
         * including address, function, length and trailing check byte. */
        if ((response_buffer[2] < 5U) ||
            (response_buffer[2] > ZDT_RESPONSE_BUFFER_SIZE)) return 0xFFU;
        return response_buffer[2];
    }
    if ((function == ZDT_FUNCTION_READ_SPEED) ||
        (function == ZDT_FUNCTION_READ_POSITION)) {
        /* Read commands may return a short four-byte error frame. */
        if ((response_length >= 4U) &&
            (response_buffer[3] == ZDT_STEPPER_CHECK_BYTE) &&
            IsWireResponse(response_buffer[2])) return 4U;
        return (function == ZDT_FUNCTION_READ_SPEED) ? 6U : 8U;
    }
    return 0U;
}

static void DiscardPrefixByte(void)
{
    if (response_length == 0U) return;
    memmove(response_buffer, response_buffer + 1U,
            (size_t)(response_length - 1U));
    response_length--;
    stepper.stats.discarded_rx_bytes++;
}

static bool ParseDataFrame(const uint8_t *frame, uint8_t length)
{
    uint8_t function = frame[1];

    if (IsActionFunction(function)) {
        if (!IsWireResponse(frame[2])) return false;
        stepper.last_response = DecodeWireResponse(frame[2]);
    } else if (function == ZDT_FUNCTION_READ_ENCODER) {
        if (length == 4U) {
            stepper.last_response = DecodeWireResponse(frame[2]);
        } else {
            stepper.encoder_raw = (uint16_t)(((uint16_t)frame[2] << 8) |
                                             (uint16_t)frame[3]);
            stepper.encoder_valid = true;
            stepper.encoder_sequence++;
            stepper.last_response = ZDT_STEPPER_RESPONSE_DATA;
        }
    } else if (function == ZDT_FUNCTION_READ_SPEED) {
        if (length == 4U) {
            stepper.last_response = DecodeWireResponse(frame[2]);
        } else {
            if (frame[2] > 1U) return false;
            stepper.speed_negative = (frame[2] != 0U);
            stepper.speed_rpm = (uint16_t)(((uint16_t)frame[3] << 8) |
                                            (uint16_t)frame[4]);
            stepper.speed_valid = true;
            stepper.last_response = ZDT_STEPPER_RESPONSE_DATA;
        }
    } else if (function == ZDT_FUNCTION_READ_POSITION) {
        if (length == 4U) {
            stepper.last_response = DecodeWireResponse(frame[2]);
        } else {
            if (frame[2] > 1U) return false;
            stepper.position_negative = (frame[2] != 0U);
            stepper.position_raw = ((uint32_t)frame[3] << 24) |
                                   ((uint32_t)frame[4] << 16) |
                                   ((uint32_t)frame[5] << 8) |
                                   (uint32_t)frame[6];
            stepper.position_valid = true;
            stepper.position_sequence++;
            stepper.last_response = ZDT_STEPPER_RESPONSE_DATA;
        }
    } else if (function == ZDT_FUNCTION_READ_CONFIG) {
        uint8_t payload_length;
        uint32_t fingerprint = 2166136261UL;
        uint8_t index;
        if (length == 4U) {
            stepper.last_response = DecodeWireResponse(frame[2]);
        } else {
            payload_length = frame[2];
            memset(&stepper.driver_config, 0,
                   sizeof(stepper.driver_config));
            stepper.driver_config.length = payload_length;
            if (payload_length >= 8U) {
                stepper.driver_config.parameter_count = frame[3];
                stepper.driver_config.motor_type = frame[4];
                stepper.driver_config.pulse_mode = frame[5];
                stepper.driver_config.communication_mode = frame[6];
                stepper.driver_config.enable_mode = frame[7];
                stepper.driver_config.direction = frame[8];
                stepper.driver_config.subdivision = frame[9];
            }
            if (payload_length >= 9U)
                stepper.driver_config.interpolation = frame[10];
            if (payload_length >= 21U)
                stepper.driver_config.response_mode = frame[22];
            if (payload_length >= 22U)
                stepper.driver_config.stall_protection = frame[23];
            /* Fingerprint the declared length and every configuration byte,
             * excluding only address/function and the trailing check byte. */
            for (index = 2U; index < (uint8_t)(length - 1U); index++) {
                fingerprint ^= frame[index];
                fingerprint *= 16777619UL;
            }
            stepper.driver_config_fingerprint = fingerprint;
            stepper.driver_config_valid = true;
            stepper.driver_config_sequence++;
            stepper.last_response = ZDT_STEPPER_RESPONSE_DATA;
        }
    } else if (function == ZDT_FUNCTION_READ_MOTOR_STATUS) {
        if ((frame[2] == ZDT_RESPONSE_PARAMETER_ERROR) ||
            (frame[2] == ZDT_RESPONSE_FORMAT_ERROR)) {
            stepper.last_response = DecodeWireResponse(frame[2]);
        } else {
            /* Bit 6 is reserved and must not be asserted by valid data. */
            if ((frame[2] & (1U << 6)) != 0U) return false;
            stepper.motor_status = frame[2];
            stepper.motor_status_valid = true;
            stepper.motor_status_sequence++;
            stepper.last_response = ZDT_STEPPER_RESPONSE_DATA;
        }
    } else if (function == ZDT_FUNCTION_READ_HOME_STATUS) {
        if ((frame[2] == ZDT_RESPONSE_PARAMETER_ERROR) ||
            (frame[2] == ZDT_RESPONSE_FORMAT_ERROR)) {
            stepper.last_response = DecodeWireResponse(frame[2]);
        } else {
            if ((frame[2] & 0xC0U) != 0U) return false;
            stepper.homing_status = frame[2];
            stepper.homing_status_valid = true;
            stepper.last_response = ZDT_STEPPER_RESPONSE_DATA;
        }
    } else {
        return false;
    }

    stepper.last_function = function;
    stepper.response_sequence++;
    return true;
}

static void ProcessBufferedResponses(void)
{
    while (response_length != 0U) {
        uint8_t expected;
        if (response_buffer[0] != ZDT_STEPPER_ADDRESS) {
            DiscardPrefixByte();
            continue;
        }
        if (response_length < 2U) return;
        expected = ExpectedFrameLength();
        if (expected == 0xFEU) return;
        if ((expected == 0U) || (expected == 0xFFU)) {
            stepper.stats.invalid_frames++;
            DiscardPrefixByte();
            continue;
        }
        if (response_length < expected) return;
        if (response_buffer[expected - 1U] != ZDT_STEPPER_CHECK_BYTE) {
            stepper.stats.checksum_errors++;
            DiscardPrefixByte();
            continue;
        }
        if (!ParseDataFrame(response_buffer, expected))
            stepper.stats.invalid_frames++;
        memmove(response_buffer, response_buffer + expected,
                (size_t)(response_length - expected));
        response_length = (uint8_t)(response_length - expected);
    }
}

static void PushResponseByte(uint8_t byte)
{
    if (response_length == ZDT_RESPONSE_BUFFER_SIZE) {
        stepper.stats.invalid_frames++;
        DiscardPrefixByte();
    }
    response_buffer[response_length++] = byte;
    ProcessBufferedResponses();
}

void ZdtStepper_Init(void)
{
    memset(&stepper, 0, sizeof(stepper));
    memset(response_buffer, 0, sizeof(response_buffer));
    response_length = 0U;
    ZdtStepperUart_Init();
}

void ZdtStepper_Poll(void)
{
    uint8_t data[ZDT_POLL_CHUNK_SIZE];
    size_t length;
    size_t index;
    do {
        length = ZdtStepperUart_Read(data, sizeof(data));
        for (index = 0U; index < length; index++) PushResponseByte(data[index]);
    } while (length == sizeof(data));
}

void ZdtStepper_GetSnapshot(ZdtStepperSnapshot_t *snapshot)
{
    ZdtStepperUartStats_t uart;
    if (snapshot == NULL) return;
    ZdtStepperUart_GetStats(&uart);
    *snapshot = stepper;
    snapshot->stats.tx_bytes = uart.tx_bytes;
    snapshot->stats.rx_bytes = uart.rx_bytes;
    snapshot->stats.tx_overflow = uart.tx_overflow;
    snapshot->stats.rx_overflow = uart.rx_overflow;
    snapshot->stats.hardware_errors = uart.hardware_errors;
    snapshot->stats.preempted_frames = uart.preempted_frames;
    snapshot->stats.tx_queued = uart.tx_queued;
    snapshot->stats.rx_queued = uart.rx_queued;
    snapshot->stats.tx_high_water = uart.tx_high_water;
    snapshot->stats.rx_high_water = uart.rx_high_water;
}

ZdtStepperResult_t ZdtStepper_Enable(bool enabled)
{
    const uint8_t frame[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_ENABLE, 0xABU,
        enabled ? 0x01U : 0x00U, 0x00U, ZDT_STEPPER_CHECK_BYTE
    };
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_Stop(void)
{
    const uint8_t frame[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_STOP, 0x98U, 0x00U,
        ZDT_STEPPER_CHECK_BYTE
    };
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_EmergencyStop(bool disable_after_stop)
{
    static const uint8_t stop[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_STOP, 0x98U, 0x00U,
        ZDT_STEPPER_CHECK_BYTE
    };
    static const uint8_t disable[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_ENABLE, 0xABU, 0x00U, 0x00U,
        ZDT_STEPPER_CHECK_BYTE
    };
    const uint8_t *frames[2] = {stop, disable};
    const uint8_t lengths[2] = {(uint8_t)sizeof(stop),
                                (uint8_t)sizeof(disable)};
    return ZdtStepperUart_WriteEmergencyBatch(frames, lengths,
                                               disable_after_stop ? 2U : 1U) ?
        ZDT_STEPPER_OK : ZDT_STEPPER_TX_FULL;
}

ZdtStepperResult_t ZdtStepper_EmergencyDisable(void)
{
    static const uint8_t disable[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_ENABLE, 0xABU, 0x00U, 0x00U,
        ZDT_STEPPER_CHECK_BYTE
    };
    const uint8_t *frames[1] = {disable};
    const uint8_t lengths[1] = {(uint8_t)sizeof(disable)};
    return ZdtStepperUart_WriteEmergencyBatch(frames, lengths, 1U) ?
        ZDT_STEPPER_OK : ZDT_STEPPER_TX_FULL;
}

ZdtStepperResult_t ZdtStepper_RunVelocity(ZdtStepperDirection_t direction,
                                          uint16_t speed_rpm,
                                          uint8_t acceleration)
{
    uint8_t frame[8];
    if (!IsDirectionValid(direction) ||
        (speed_rpm > ZDT_STEPPER_MAX_SPEED_RPM))
        return ZDT_STEPPER_INVALID_ARGUMENT;
    frame[0] = ZDT_STEPPER_ADDRESS;
    frame[1] = ZDT_FUNCTION_VELOCITY;
    frame[2] = (uint8_t)direction;
    frame[3] = (uint8_t)(speed_rpm >> 8);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = 0x00U;
    frame[7] = ZDT_STEPPER_CHECK_BYTE;
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_MovePosition(
    ZdtStepperDirection_t direction, uint16_t speed_rpm,
    uint8_t acceleration, uint32_t pulses,
    ZdtStepperPositionMode_t mode)
{
    uint8_t frame[13];
    if (!IsDirectionValid(direction) || !IsPositionModeValid(mode) ||
        (speed_rpm > ZDT_STEPPER_MAX_SPEED_RPM))
        return ZDT_STEPPER_INVALID_ARGUMENT;
    frame[0] = ZDT_STEPPER_ADDRESS;
    frame[1] = ZDT_FUNCTION_POSITION;
    frame[2] = (uint8_t)direction;
    frame[3] = (uint8_t)(speed_rpm >> 8);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(pulses >> 24);
    frame[7] = (uint8_t)(pulses >> 16);
    frame[8] = (uint8_t)(pulses >> 8);
    frame[9] = (uint8_t)pulses;
    frame[10] = (uint8_t)mode;
    frame[11] = 0x00U;
    frame[12] = ZDT_STEPPER_CHECK_BYTE;
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_ZeroCurrentPosition(void)
{
    const uint8_t frame[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_ZERO_POSITION, 0x6DU,
        ZDT_STEPPER_CHECK_BYTE
    };
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_ClearProtection(void)
{
    const uint8_t frame[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_CLEAR_PROTECTION, 0x52U,
        ZDT_STEPPER_CHECK_BYTE
    };
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_StartHoming(ZdtStepperHomeMode_t mode)
{
    uint8_t frame[5];
    if (!IsHomeModeValid(mode)) return ZDT_STEPPER_INVALID_ARGUMENT;
    frame[0] = ZDT_STEPPER_ADDRESS;
    frame[1] = ZDT_FUNCTION_START_HOMING;
    frame[2] = (uint8_t)mode;
    frame[3] = 0x00U;
    frame[4] = ZDT_STEPPER_CHECK_BYTE;
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_AbortHoming(void)
{
    const uint8_t frame[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_ABORT_HOMING, 0x48U,
        ZDT_STEPPER_CHECK_BYTE
    };
    return SendFrame(frame, sizeof(frame));
}

static ZdtStepperResult_t SendRead(uint8_t function)
{
    const uint8_t frame[] = {
        ZDT_STEPPER_ADDRESS, function, ZDT_STEPPER_CHECK_BYTE
    };
    return SendFrame(frame, sizeof(frame));
}

ZdtStepperResult_t ZdtStepper_ReadSpeed(void)
{
    return SendRead(ZDT_FUNCTION_READ_SPEED);
}

ZdtStepperResult_t ZdtStepper_ReadEncoder(void)
{
    return SendRead(ZDT_FUNCTION_READ_ENCODER);
}

ZdtStepperResult_t ZdtStepper_ReadPosition(void)
{
    return SendRead(ZDT_FUNCTION_READ_POSITION);
}

ZdtStepperResult_t ZdtStepper_ReadMotorStatus(void)
{
    return SendRead(ZDT_FUNCTION_READ_MOTOR_STATUS);
}

ZdtStepperResult_t ZdtStepper_ReadHomingStatus(void)
{
    return SendRead(ZDT_FUNCTION_READ_HOME_STATUS);
}

ZdtStepperResult_t ZdtStepper_ReadDriverConfig(void)
{
    const uint8_t frame[] = {
        ZDT_STEPPER_ADDRESS, ZDT_FUNCTION_READ_CONFIG, 0x6CU,
        ZDT_STEPPER_CHECK_BYTE
    };
    return SendFrame(frame, sizeof(frame));
}

int32_t ZdtStepper_CircularDelta(uint16_t value, uint16_t reference)
{
    uint16_t wrapped = (uint16_t)(value - reference);
    return (wrapped < 0x8000U) ? (int32_t)wrapped :
                                (int32_t)wrapped - 65536L;
}
