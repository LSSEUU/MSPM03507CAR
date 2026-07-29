#include "zdt_stepper_test.h"

#include <assert.h>
#include <string.h>

static ZdtStepperSnapshot_t device;
static unsigned emergency_count;
static bool emergency_disable;
static unsigned query_count[4];

void ZdtStepper_Init(void) { memset(&device, 0, sizeof(device)); }
void ZdtStepper_Poll(void) {}
void ZdtStepper_GetSnapshot(ZdtStepperSnapshot_t *snapshot) { *snapshot = device; }
ZdtStepperResult_t ZdtStepper_EmergencyStop(bool disable)
{
    emergency_count++; emergency_disable = disable; return ZDT_STEPPER_OK;
}
ZdtStepperResult_t ZdtStepper_Enable(bool enabled)
{ (void)enabled; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_RunVelocity(ZdtStepperDirection_t direction,
    uint16_t speed, uint8_t acceleration)
{ (void)direction; (void)speed; (void)acceleration; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_MovePosition(ZdtStepperDirection_t direction,
    uint16_t speed, uint8_t acceleration, uint32_t pulses,
    ZdtStepperPositionMode_t mode)
{ (void)direction; (void)speed; (void)acceleration; (void)pulses; (void)mode; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadSpeed(void) { query_count[0]++; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadPosition(void) { query_count[1]++; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadMotorStatus(void) { query_count[2]++; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadHomingStatus(void) { query_count[3]++; return ZDT_STEPPER_OK; }

static void respond(uint32_t sequence, uint8_t function,
                    ZdtStepperResponse_t response, uint8_t motor)
{
    device.response_sequence = sequence;
    device.last_function = function;
    device.last_response = response;
    if (function == 0x3AU) {
        device.motor_status_valid = true;
        device.motor_status = motor;
    }
}

static ZdtStepperTestRequest_t velocity(uint32_t id, uint16_t duration)
{
    ZdtStepperTestRequest_t r;
    memset(&r, 0, sizeof(r));
    r.command_id = id; r.command = ZDT_TEST_COMMAND_VELOCITY;
    r.confirmed = true; r.direction = ZDT_STEPPER_DIRECTION_CW;
    r.speed_rpm = 60U; r.acceleration = 20U;
    r.duration_or_timeout_ms = duration;
    r.position_mode = ZDT_STEPPER_POSITION_RELATIVE_LAST_TARGET;
    return r;
}

static void make_motor_ready(uint32_t now)
{
    respond(device.response_sequence + 1U, 0x3AU, ZDT_STEPPER_RESPONSE_DATA,
            ZDT_STEPPER_MOTOR_ENABLED);
    ZdtStepperTest_Poll(now);
}

static void test_startup_and_queries(void)
{
    memset(query_count, 0, sizeof(query_count)); emergency_count = 0U;
    ZdtStepperTest_Init(0U);
    assert(emergency_count == 1U && emergency_disable);
    ZdtStepperTest_RequestRefresh(ZDT_STEPPER_REFRESH_ALL);
    ZdtStepperTest_RequestRefresh(ZDT_STEPPER_REFRESH_ALL);
    ZdtStepperTest_Poll(1U); assert(query_count[0] == 1U);
    respond(1U, 0x35U, ZDT_STEPPER_RESPONSE_DATA, 0U);
    ZdtStepperTest_Poll(2U); assert(query_count[1] == 1U);
    respond(2U, 0x36U, ZDT_STEPPER_RESPONSE_DATA, 0U);
    ZdtStepperTest_Poll(3U); assert(query_count[2] == 1U);
    respond(3U, 0x3AU, ZDT_STEPPER_RESPONSE_DATA, ZDT_STEPPER_MOTOR_ENABLED);
    ZdtStepperTest_Poll(4U); assert(query_count[3] == 1U);
    respond(4U, 0x3BU, ZDT_STEPPER_RESPONSE_DATA, 0U);
    ZdtStepperTest_Poll(5U);
    assert(query_count[0] == 1U && query_count[1] == 1U &&
           query_count[2] == 1U && query_count[3] == 1U);
}

static void test_velocity_deadline_and_stale_feedback(void)
{
    ZdtStepperTestRequest_t request;
    ZdtStepperTestSnapshot_t snapshot;
    ZdtStepperTest_Init(0U); make_motor_ready(1U);
    request = velocity(10U, 100U);
    assert(ZdtStepperTest_Command(&request, 2U) == ZDT_STEPPER_OK);
    respond(2U, 0xF6U, ZDT_STEPPER_RESPONSE_ACCEPTED,
            ZDT_STEPPER_MOTOR_ENABLED);
    ZdtStepperTest_Poll(3U);
    ZdtStepperTest_GetSnapshot(&snapshot, 3U);
    assert(snapshot.state == ZDT_TEST_STATE_RUNNING);
    ZdtStepperTest_Poll(102U);
    ZdtStepperTest_GetSnapshot(&snapshot, 102U);
    assert(snapshot.state == ZDT_TEST_STATE_COMPLETED);
    assert(snapshot.stop_reason == ZDT_TEST_STOP_DURATION);

    ZdtStepperTest_Init(0U); make_motor_ready(1U);
    request = velocity(11U, 1000U);
    assert(ZdtStepperTest_Command(&request, 2U) == ZDT_STEPPER_OK);
    respond(2U, 0xF6U, ZDT_STEPPER_RESPONSE_ACCEPTED,
            ZDT_STEPPER_MOTOR_ENABLED);
    ZdtStepperTest_Poll(3U); ZdtStepperTest_Poll(504U);
    ZdtStepperTest_GetSnapshot(&snapshot, 504U);
    assert(snapshot.state == ZDT_TEST_STATE_ERROR);
    assert(snapshot.stop_reason == ZDT_TEST_STOP_COMMUNICATION);
}

static void test_position_completion_and_validation(void)
{
    ZdtStepperTestRequest_t request;
    ZdtStepperTestSnapshot_t snapshot;
    ZdtStepperTest_Init(0U); make_motor_ready(1U);
    request = velocity(20U, 99U);
    assert(ZdtStepperTest_Command(&request, 2U) == ZDT_STEPPER_INVALID_ARGUMENT);
    request.duration_or_timeout_ms = 100U; request.speed_rpm = 301U;
    assert(ZdtStepperTest_Command(&request, 2U) == ZDT_STEPPER_INVALID_ARGUMENT);

    memset(&request, 0, sizeof(request)); request.command_id = 21U;
    request.command = ZDT_TEST_COMMAND_POSITION; request.confirmed = true;
    request.direction = ZDT_STEPPER_DIRECTION_CCW; request.speed_rpm = 60U;
    request.pulses = 100U; request.duration_or_timeout_ms = 500U;
    request.position_mode = ZDT_STEPPER_POSITION_RELATIVE_CURRENT;
    assert(ZdtStepperTest_Command(&request, 3U) == ZDT_STEPPER_OK);
    respond(2U, 0xFDU, ZDT_STEPPER_RESPONSE_ACCEPTED,
            ZDT_STEPPER_MOTOR_ENABLED);
    ZdtStepperTest_Poll(4U);
    respond(3U, 0x3AU, ZDT_STEPPER_RESPONSE_DATA,
            ZDT_STEPPER_MOTOR_ENABLED | ZDT_STEPPER_MOTOR_POSITION_REACHED);
    ZdtStepperTest_Poll(20U);
    ZdtStepperTest_GetSnapshot(&snapshot, 20U);
    assert(snapshot.state == ZDT_TEST_STATE_COMPLETED);
    assert(snapshot.stop_reason == ZDT_TEST_STOP_POSITION_REACHED);
}

int main(void)
{
    test_startup_and_queries();
    test_velocity_deadline_and_stale_feedback();
    test_position_completion_and_validation();
    return 0;
}
