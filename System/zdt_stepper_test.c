#include "zdt_stepper_test.h"

#include <limits.h>
#include <string.h>

#define COMMAND_TIMEOUT_MS       500U
#define FEEDBACK_TIMEOUT_MS      500U
#define ACTIVE_QUERY_PERIOD_MS   100U
#define MOTOR_UNSAFE_MASK (ZDT_STEPPER_MOTOR_STALLED | \
    ZDT_STEPPER_MOTOR_STALL_PROTECTION | ZDT_STEPPER_MOTOR_LEFT_LIMIT | \
    ZDT_STEPPER_MOTOR_RIGHT_LIMIT | ZDT_STEPPER_MOTOR_POWER_LOSS)

typedef struct {
    ZdtStepperTestSnapshot_t snapshot;
    uint32_t seen_sequence;
    uint32_t response_ms;
    uint32_t motor_ms;
    uint32_t feedback_ms;
    uint32_t command_sent_ms;
    uint32_t query_sent_ms;
    uint32_t deadline_ms;
    uint32_t next_query_ms;
    uint32_t next_test_id;
    uint8_t expected_function;
    uint8_t refresh_pending;
    uint8_t query_function;
    bool query_inflight;
} Controller_t;

static Controller_t s_controller;

static bool Reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static uint8_t CommandFunction(ZdtStepperTestCommand_t command)
{
    if (command == ZDT_TEST_COMMAND_ENABLE ||
        command == ZDT_TEST_COMMAND_DISABLE) return 0xF3U;
    if (command == ZDT_TEST_COMMAND_STOP) return 0xFEU;
    if (command == ZDT_TEST_COMMAND_VELOCITY) return 0xF6U;
    if (command == ZDT_TEST_COMMAND_POSITION) return 0xFDU;
    return 0U;
}

static bool IsResponseError(ZdtStepperResponse_t response)
{
    return (response == ZDT_STEPPER_RESPONSE_LIMIT_ACTIVE) ||
        (response == ZDT_STEPPER_RESPONSE_PARAMETER_OR_STATE_ERROR) ||
        (response == ZDT_STEPPER_RESPONSE_FORMAT_ERROR);
}

static void EmergencyStop(ZdtStepperTestState_t state,
                          ZdtStepperTestStopReason_t reason)
{
    (void)ZdtStepper_EmergencyStop(false);
    s_controller.snapshot.state = state;
    s_controller.snapshot.stop_reason = reason;
    s_controller.snapshot.remaining_ms = 0U;
    s_controller.expected_function = 0U;
}

static void ObserveResponse(uint32_t now_ms)
{
    ZdtStepperSnapshot_t device;
    ZdtStepper_GetSnapshot(&device);
    if (device.response_sequence == s_controller.seen_sequence) return;
    s_controller.seen_sequence = device.response_sequence;
    s_controller.response_ms = now_ms;
    s_controller.feedback_ms = now_ms;
    if (device.last_function == 0x3AU) s_controller.motor_ms = now_ms;
    if (s_controller.query_inflight &&
        (device.last_function == s_controller.query_function))
        s_controller.query_inflight = false;

    if ((s_controller.expected_function != 0U) &&
        (device.last_function == s_controller.expected_function)) {
        if (IsResponseError(device.last_response)) {
            EmergencyStop(ZDT_TEST_STATE_ERROR,
                          ZDT_TEST_STOP_DEVICE_REJECTED);
        } else if ((s_controller.snapshot.kind == ZDT_TEST_KIND_POSITION) &&
                   (device.last_response == ZDT_STEPPER_RESPONSE_COMPLETED)) {
            EmergencyStop(ZDT_TEST_STATE_COMPLETED,
                          ZDT_TEST_STOP_POSITION_REACHED);
        } else {
            s_controller.expected_function = 0U;
            s_controller.snapshot.state =
                (s_controller.snapshot.kind == ZDT_TEST_KIND_NONE) ?
                ZDT_TEST_STATE_IDLE : ZDT_TEST_STATE_RUNNING;
        }
    }
    if ((s_controller.snapshot.kind == ZDT_TEST_KIND_POSITION) &&
        device.motor_status_valid &&
        ((device.motor_status & ZDT_STEPPER_MOTOR_POSITION_REACHED) != 0U))
        EmergencyStop(ZDT_TEST_STATE_COMPLETED,
                      ZDT_TEST_STOP_POSITION_REACHED);
}

static void QueueNextQuery(uint32_t now_ms)
{
    ZdtStepperResult_t result = ZDT_STEPPER_OK;
    uint8_t bit = 0U;
    if (s_controller.expected_function != 0U) return;
    if (s_controller.query_inflight) {
        if (Reached(now_ms, s_controller.query_sent_ms +
                             COMMAND_TIMEOUT_MS))
            s_controller.query_inflight = false;
        else
            return;
    }
    if ((s_controller.refresh_pending & ZDT_STEPPER_REFRESH_SPEED) != 0U) {
        bit = ZDT_STEPPER_REFRESH_SPEED; s_controller.query_function = 0x35U;
        result = ZdtStepper_ReadSpeed();
    } else if ((s_controller.refresh_pending &
                ZDT_STEPPER_REFRESH_POSITION) != 0U) {
        bit = ZDT_STEPPER_REFRESH_POSITION; s_controller.query_function = 0x36U;
        result = ZdtStepper_ReadPosition();
    } else if ((s_controller.refresh_pending &
                ZDT_STEPPER_REFRESH_MOTOR) != 0U) {
        bit = ZDT_STEPPER_REFRESH_MOTOR; s_controller.query_function = 0x3AU;
        result = ZdtStepper_ReadMotorStatus();
    } else if ((s_controller.refresh_pending &
                ZDT_STEPPER_REFRESH_HOMING) != 0U) {
        bit = ZDT_STEPPER_REFRESH_HOMING; s_controller.query_function = 0x3BU;
        result = ZdtStepper_ReadHomingStatus();
    }
    if ((bit != 0U) && (result == ZDT_STEPPER_OK)) {
        s_controller.refresh_pending &= (uint8_t)~bit;
        s_controller.query_inflight = true;
        s_controller.query_sent_ms = now_ms;
    }
}

void ZdtStepperTest_Init(uint32_t now_ms)
{
    memset(&s_controller, 0, sizeof(s_controller));
    ZdtStepper_Init();
    s_controller.next_test_id = 1U;
    s_controller.response_ms = UINT32_MAX;
    s_controller.motor_ms = UINT32_MAX;
    s_controller.feedback_ms = now_ms;
    s_controller.snapshot.state = ZDT_TEST_STATE_IDLE;
    s_controller.snapshot.kind = ZDT_TEST_KIND_NONE;
    s_controller.snapshot.stop_reason = ZDT_TEST_STOP_STARTUP;
    (void)ZdtStepper_EmergencyStop(true);
}

void ZdtStepperTest_RequestRefresh(uint8_t mask)
{
    s_controller.refresh_pending |= (uint8_t)(mask & ZDT_STEPPER_REFRESH_ALL);
}

void ZdtStepperTest_Poll(uint32_t now_ms)
{
    ZdtStepper_Poll();
    ObserveResponse(now_ms);

    if ((s_controller.snapshot.state == ZDT_TEST_STATE_WAITING_ACK) &&
        Reached(now_ms, s_controller.command_sent_ms + COMMAND_TIMEOUT_MS)) {
        EmergencyStop(ZDT_TEST_STATE_ERROR, ZDT_TEST_STOP_COMMUNICATION);
    }
    if ((s_controller.snapshot.state == ZDT_TEST_STATE_RUNNING) ||
        (s_controller.snapshot.state == ZDT_TEST_STATE_WAITING_ACK &&
         s_controller.snapshot.kind != ZDT_TEST_KIND_NONE)) {
        if ((s_controller.snapshot.state == ZDT_TEST_STATE_RUNNING) &&
            Reached(now_ms, s_controller.next_query_ms)) {
            s_controller.refresh_pending |= ZDT_STEPPER_REFRESH_ALL;
            s_controller.next_query_ms = now_ms + ACTIVE_QUERY_PERIOD_MS;
        }
        if (Reached(now_ms, s_controller.feedback_ms + FEEDBACK_TIMEOUT_MS)) {
            EmergencyStop(ZDT_TEST_STATE_ERROR,
                          ZDT_TEST_STOP_COMMUNICATION);
        } else if (Reached(now_ms, s_controller.deadline_ms)) {
            EmergencyStop(ZDT_TEST_STATE_COMPLETED,
                s_controller.snapshot.kind == ZDT_TEST_KIND_VELOCITY ?
                ZDT_TEST_STOP_DURATION : ZDT_TEST_STOP_TIMEOUT);
        }
    }
    QueueNextQuery(now_ms);
}

static bool MotorReady(uint32_t now_ms)
{
    ZdtStepperSnapshot_t device;
    ZdtStepper_GetSnapshot(&device);
    return device.motor_status_valid &&
        (s_controller.motor_ms != UINT32_MAX) &&
        !Reached(now_ms, s_controller.motor_ms + FEEDBACK_TIMEOUT_MS + 1U) &&
        ((device.motor_status & ZDT_STEPPER_MOTOR_ENABLED) != 0U) &&
        ((device.motor_status & MOTOR_UNSAFE_MASK) == 0U);
}

ZdtStepperResult_t ZdtStepperTest_Command(
    const ZdtStepperTestRequest_t *request, uint32_t now_ms)
{
    ZdtStepperResult_t result;
    if (request == NULL) return ZDT_STEPPER_INVALID_ARGUMENT;
    if ((request->command < ZDT_TEST_COMMAND_ENABLE) ||
        (request->command > ZDT_TEST_COMMAND_POSITION))
        return ZDT_STEPPER_INVALID_ARGUMENT;
    if (((request->command == ZDT_TEST_COMMAND_ENABLE) ||
         (request->command == ZDT_TEST_COMMAND_VELOCITY) ||
         (request->command == ZDT_TEST_COMMAND_POSITION)) &&
        !request->confirmed) return ZDT_STEPPER_INVALID_ARGUMENT;

    if (request->command == ZDT_TEST_COMMAND_STOP ||
        request->command == ZDT_TEST_COMMAND_DISABLE) {
        result = ZdtStepper_EmergencyStop(
            request->command == ZDT_TEST_COMMAND_DISABLE);
        if (result != ZDT_STEPPER_OK) return result;
        s_controller.snapshot.command_id = request->command_id;
        s_controller.snapshot.kind = ZDT_TEST_KIND_NONE;
        s_controller.snapshot.state = ZDT_TEST_STATE_IDLE;
        s_controller.snapshot.stop_reason =
            request->command == ZDT_TEST_COMMAND_DISABLE ?
            ZDT_TEST_STOP_DISABLED : ZDT_TEST_STOP_USER;
        s_controller.expected_function = 0U;
        return ZDT_STEPPER_OK;
    }

    if (request->command == ZDT_TEST_COMMAND_ENABLE) {
        result = ZdtStepper_Enable(true);
        if (result != ZDT_STEPPER_OK) return result;
        s_controller.snapshot.kind = ZDT_TEST_KIND_NONE;
    } else {
        if ((request->direction != ZDT_STEPPER_DIRECTION_CW) &&
            (request->direction != ZDT_STEPPER_DIRECTION_CCW))
            return ZDT_STEPPER_INVALID_ARGUMENT;
        if ((request->speed_rpm < 1U) || (request->speed_rpm > 300U) ||
            !MotorReady(now_ms)) return ZDT_STEPPER_INVALID_ARGUMENT;
        if (request->command == ZDT_TEST_COMMAND_VELOCITY) {
            if ((request->duration_or_timeout_ms < 100U) ||
                (request->duration_or_timeout_ms > 5000U) ||
                (request->pulses != 0U) ||
                (request->position_mode !=
                 ZDT_STEPPER_POSITION_RELATIVE_LAST_TARGET))
                return ZDT_STEPPER_INVALID_ARGUMENT;
            result = ZdtStepper_RunVelocity(request->direction,
                request->speed_rpm, request->acceleration);
            s_controller.snapshot.kind = ZDT_TEST_KIND_VELOCITY;
        } else {
            if ((request->duration_or_timeout_ms < 500U) ||
                (request->duration_or_timeout_ms > 10000U) ||
                (request->pulses < 1U) || (request->pulses > 3200U) ||
                (request->position_mode !=
                 ZDT_STEPPER_POSITION_RELATIVE_CURRENT))
                return ZDT_STEPPER_INVALID_ARGUMENT;
            result = ZdtStepper_MovePosition(request->direction,
                request->speed_rpm, request->acceleration, request->pulses,
                request->position_mode);
            s_controller.snapshot.kind = ZDT_TEST_KIND_POSITION;
        }
        if (result != ZDT_STEPPER_OK) return result;
        s_controller.snapshot.test_id = s_controller.next_test_id++;
        s_controller.deadline_ms = now_ms + request->duration_or_timeout_ms;
        s_controller.feedback_ms = now_ms;
        s_controller.next_query_ms = now_ms;
    }
    s_controller.snapshot.command_id = request->command_id;
    s_controller.snapshot.direction = request->direction;
    s_controller.snapshot.acceleration = request->acceleration;
    s_controller.snapshot.requested_speed_rpm = request->speed_rpm;
    s_controller.snapshot.position_mode = request->position_mode;
    s_controller.snapshot.pulses = request->pulses;
    s_controller.snapshot.stop_reason = ZDT_TEST_STOP_NONE;
    s_controller.snapshot.state = ZDT_TEST_STATE_WAITING_ACK;
    s_controller.expected_function = CommandFunction(request->command);
    s_controller.command_sent_ms = now_ms;
    return ZDT_STEPPER_OK;
}

void ZdtStepperTest_GetSnapshot(ZdtStepperTestSnapshot_t *snapshot,
                                uint32_t now_ms)
{
    if (snapshot == NULL) return;
    *snapshot = s_controller.snapshot;
    ZdtStepper_GetSnapshot(&snapshot->device);
    snapshot->time_ms = now_ms;
    snapshot->response_age_ms = (s_controller.response_ms == UINT32_MAX) ?
        UINT32_MAX : now_ms - s_controller.response_ms;
    snapshot->valid_flags = 0U;
    if (snapshot->device.speed_valid) snapshot->valid_flags |= 1U;
    if (snapshot->device.position_valid) snapshot->valid_flags |= 2U;
    if (snapshot->device.motor_status_valid) snapshot->valid_flags |= 4U;
    if (snapshot->device.homing_status_valid) snapshot->valid_flags |= 8U;
    snapshot->speed_rpm = snapshot->device.speed_negative ?
        -(int16_t)snapshot->device.speed_rpm :
        (int16_t)snapshot->device.speed_rpm;
    snapshot->position_raw = snapshot->device.position_negative ?
        -(int64_t)snapshot->device.position_raw :
        (int64_t)snapshot->device.position_raw;
    if ((snapshot->state == ZDT_TEST_STATE_RUNNING) ||
        (snapshot->state == ZDT_TEST_STATE_WAITING_ACK &&
         snapshot->kind != ZDT_TEST_KIND_NONE))
        snapshot->remaining_ms = Reached(now_ms, s_controller.deadline_ms) ?
            0U : s_controller.deadline_ms - now_ms;
    else
        snapshot->remaining_ms = 0U;
}
