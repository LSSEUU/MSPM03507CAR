#include "lever_actuator.h"

#include "lever_motion_config.h"
#include "lever_calibration.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "zdt_stepper.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static LeverCalibrationValues_t calibration;
static bool calibration_valid;
static bool session_valid;
static uint16_t session_reference;
static int32_t session_horizontal;
static ZdtStepperSnapshot_t device;
static uint32_t now_ms;
static uint32_t enable_calls;
static uint32_t move_calls;
static uint32_t stop_calls;
static uint32_t disable_stop_calls;
static uint32_t begin_action_calls;
static uint32_t end_action_calls;
static uint32_t read_position_calls;
static uint32_t last_move_pulses;
static uint16_t last_move_speed;
static ZdtStepperDirection_t last_move_direction;
static SystemState_t system_state;
static bool safety_fault;

#define TEST_BOOT_POSITION_ORIGIN (-1467L)

bool LeverCalibration_GetActive(LeverCalibrationValues_t *value)
{
    if (!calibration_valid) return false;
    *value = calibration;
    return true;
}
bool LeverCalibration_GetSessionReference(uint16_t *reference, int32_t *horizontal)
{
    if (!session_valid) return false;
    if (reference != NULL) *reference = session_reference;
    if (horizontal != NULL) *horizontal = session_horizontal;
    return true;
}
bool LeverCalibration_IsActive(void) { return false; }
void LeverCalibration_InvalidateDriverConfig(uint32_t actual_fingerprint)
{
    (void)actual_fingerprint;
    calibration_valid = false;
    session_valid = false;
}
uint32_t SystemTime_Now(void) { return now_ms; }
SystemState_t SystemState_Get(void) { return system_state; }
bool SystemState_RequestRun(void)
{
    if (system_state != SYSTEM_STATE_IDLE) return false;
    system_state = SYSTEM_STATE_RUN;
    return true;
}
bool SystemState_RequestIdle(void)
{
    if (system_state != SYSTEM_STATE_RUN) return false;
    system_state = SYSTEM_STATE_IDLE;
    return true;
}
bool Safety_HasFault(void) { return safety_fault; }
void Safety_BeginAction(uint32_t start_ms, uint32_t timeout_ms,
                        uint32_t required_sensor_mask)
{
    (void)start_ms; (void)timeout_ms; (void)required_sensor_mask;
    begin_action_calls++;
}
void Safety_EndAction(void) { end_action_calls++; }
void ZdtStepper_GetSnapshot(ZdtStepperSnapshot_t *snapshot) { *snapshot = device; }
int32_t ZdtStepper_CircularDelta(uint16_t value, uint16_t reference)
{
    uint16_t wrapped = (uint16_t)(value - reference);
    return wrapped < 0x8000U ? (int32_t)wrapped : (int32_t)wrapped - 65536L;
}
ZdtStepperResult_t ZdtStepper_ReadEncoder(void) { return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadPosition(void)
{ read_position_calls++; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadMotorStatus(void) { return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadDriverConfig(void) { return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_Enable(bool enabled)
{
    assert(enabled);
    enable_calls++;
    return ZDT_STEPPER_OK;
}
ZdtStepperResult_t ZdtStepper_MovePosition(
    ZdtStepperDirection_t direction, uint16_t speed, uint8_t acceleration,
    uint32_t pulses, ZdtStepperPositionMode_t mode)
{
    (void)acceleration; (void)mode;
    last_move_direction = direction;
    last_move_speed = speed;
    last_move_pulses = pulses;
    move_calls++;
    return ZDT_STEPPER_OK;
}
ZdtStepperResult_t ZdtStepper_Stop(void) { stop_calls++; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_EmergencyStop(bool disable_after_stop)
{
    if (disable_after_stop) disable_stop_calls++;
    stop_calls++;
    return ZDT_STEPPER_OK;
}

static void make_calibration(void)
{
    uint8_t i;
    memset(&calibration, 0, sizeof(calibration));
    calibration.data_version = LEVER_CALIBRATION_DATA_VERSION;
    calibration.driver_config_fingerprint = 0x12345678UL;
    calibration.soft_min_angle_mdeg = -6000;
    calibration.soft_max_angle_mdeg = 6000;
    calibration.soft_min_counts = -600;
    calibration.soft_max_counts = 600;
    for (i = 0U; i < LEVER_CALIBRATION_POINT_COUNT; i++) {
        calibration.angle_mdeg[i] = -8000 + (int32_t)i * 2000;
        calibration.increasing_counts[i] = -800 + (int32_t)i * 200;
        calibration.decreasing_counts[i] = -798 + (int32_t)i * 200;
    }
}

static void reset_fixture(void)
{
    memset(&device, 0, sizeof(device));
    make_calibration();
    calibration_valid = false;
    session_valid = false;
    session_reference = 1000U;
    session_horizontal = 0;
    now_ms = 0U;
    enable_calls = move_calls = stop_calls = 0U;
    disable_stop_calls = 0U;
    begin_action_calls = end_action_calls = 0U;
    read_position_calls = 0U;
    last_move_pulses = 0U;
    last_move_speed = 0U;
    last_move_direction = ZDT_STEPPER_DIRECTION_CW;
    system_state = SYSTEM_STATE_IDLE;
    safety_fault = false;
    LeverActuator_Init(now_ms);
}

static void make_feedback(int32_t counts, uint32_t fingerprint, uint8_t motor)
{
    device.encoder_valid = true;
    device.encoder_raw = (uint16_t)(session_reference + counts);
    device.encoder_sequence++;
    device.position_valid = true;
    device.position_negative = true;
    device.position_raw = 11017U;
    device.position_sequence++;
    device.motor_status_valid = true;
    device.motor_status = motor;
    device.motor_status_sequence++;
    device.driver_config_valid = true;
    device.driver_config_fingerprint = fingerprint;
    device.driver_config.subdivision = 16U;
    device.driver_config_sequence++;
    LeverActuator_Task(now_ms);
}

static void make_boot_feedback(int32_t counts, uint8_t motor,
                               bool refresh_config)
{
    int32_t position = TEST_BOOT_POSITION_ORIGIN + counts;
    uint32_t magnitude = position < 0 ? (uint32_t)(-position) :
                                        (uint32_t)position;
    device.encoder_valid = true;
    device.encoder_raw = (uint16_t)(LEVER_ABSOLUTE_BOTTOM_ENCODER + counts);
    device.encoder_sequence++;
    device.position_valid = true;
    device.position_negative = position < 0;
    device.position_raw = magnitude;
    device.position_sequence++;
    device.motor_status_valid = true;
    device.motor_status = motor;
    device.motor_status_sequence++;
    if (refresh_config || !device.driver_config_valid) {
        device.driver_config_valid = true;
        device.driver_config_fingerprint = 0x12345678UL;
        device.driver_config.subdivision = 16U;
        device.driver_config.direction = 0U;
        device.driver_config_sequence++;
    }
    LeverActuator_Task(now_ms);
}

static void test_unsafe_states_emit_no_motion(void)
{
    reset_fixture();
    assert(LeverActuator_ArmHorizontal() == LEVER_ACTUATOR_NOT_CALIBRATED);
    assert(enable_calls == 0U && move_calls == 0U);

    calibration_valid = true;
    LeverActuator_Init(now_ms);
    assert(LeverActuator_ArmHorizontal() == LEVER_ACTUATOR_SESSION_INVALID);
    assert(enable_calls == 0U && move_calls == 0U);

    session_valid = true;
    make_feedback(0, 0x99999999UL, 0U);
    assert(LeverActuator_ArmHorizontal() == LEVER_ACTUATOR_NOT_CALIBRATED);
    assert(enable_calls == 0U && move_calls == 0U);

    reset_fixture();
    calibration_valid = true;
    session_valid = true;
    LeverActuator_Init(now_ms);
    make_feedback(700, 0x12345678UL, 0U);
    assert(LeverActuator_ArmHorizontal() == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(enable_calls == 0U && move_calls == 0U);
}

static void test_limits_timeout_and_config_change_stop(void)
{
    LeverActuatorSnapshot_t snapshot;
    reset_fixture();
    calibration_valid = true;
    session_valid = true;
    LeverActuator_Init(now_ms);
    make_feedback(0, 0x12345678UL, 0U);
    assert(LeverActuator_ArmHorizontal() == LEVER_ACTUATOR_OK);
    assert(enable_calls == 1U);
    make_feedback(0, 0x12345678UL, ZDT_STEPPER_MOTOR_ENABLED);
    assert(LeverActuator_SetTilt(3000) == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(move_calls == 0U);

    now_ms = 101U;
    assert(LeverActuator_SetTilt(1000) == LEVER_ACTUATOR_FEEDBACK_STALE);
    assert(move_calls == 0U && stop_calls >= 1U);

    reset_fixture();
    calibration_valid = true;
    session_valid = true;
    LeverActuator_Init(now_ms);
    make_feedback(0, 0x12345678UL, 0U);
    assert(LeverActuator_ArmHorizontal() == LEVER_ACTUATOR_OK);
    make_feedback(0, 0x12345678UL, ZDT_STEPPER_MOTOR_ENABLED);
    device.driver_config_fingerprint = 0x87654321UL;
    device.driver_config_sequence++;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_CONFIG_CHANGED);
    assert(!snapshot.armed && stop_calls >= 1U);

    reset_fixture();
    calibration_valid = true;
    session_valid = true;
    LeverActuator_Init(now_ms);
    make_feedback(0, 0x12345678UL, 0U);
    assert(LeverActuator_ArmHorizontal() == LEVER_ACTUATOR_OK);
    make_feedback(0, 0x12345678UL, ZDT_STEPPER_MOTOR_ENABLED);
    assert(LeverActuator_SetTilt(1000) == LEVER_ACTUATOR_OK);
    assert(move_calls == 1U);
    make_feedback(-20, 0x12345678UL, ZDT_STEPPER_MOTOR_ENABLED);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_DIRECTION_FAULT);
    assert(!snapshot.armed && !snapshot.moving && stop_calls >= 1U);
}

static void test_boot_returns_from_gravity_reference_to_horizontal(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    assert(!calibration_valid);
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(0, 0U, sample == 0U);
        now_ms += 60U;
    }
    assert(enable_calls == 1U);
    assert(begin_action_calls == 1U);
    assert(system_state == SYSTEM_STATE_RUN);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_WAIT_ENABLE);

    make_boot_feedback(0, ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    assert(last_move_direction == ZDT_STEPPER_DIRECTION_CW);
    assert(last_move_speed == LEVER_BOOT_HOME_SPEED_RPM);
    assert(last_move_pulses == 276U);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state ==
           LEVER_ACTUATOR_BOOT_MOVING_HORIZONTAL);

    now_ms += 60U;
    make_boot_feedback(1500,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 60U;
    make_boot_feedback(3500,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 3,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state ==
           LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL);
    assert(snapshot.boot_homed && !snapshot.moving);
    assert(stop_calls == 1U);
    assert(end_action_calls == 1U);
    assert(system_state == SYSTEM_STATE_IDLE);
}

static void test_boot_rejects_unknown_start_and_reverse_motion(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    make_boot_feedback(0, 0U, true);
    now_ms += 60U;
    make_boot_feedback(100, 0U, false);
    now_ms += 60U;
    make_boot_feedback(0, 0U, false);
    now_ms += LEVER_BOOT_REFERENCE_TIMEOUT_MS + 1U;
    make_boot_feedback(100, 0U, true);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(enable_calls == 0U && move_calls == 0U);

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(0, 0U, sample == 0U);
        now_ms += 60U;
    }
    make_boot_feedback(0, ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    now_ms += 60U;
    make_boot_feedback(-19,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_DIRECTION_FAULT);
    assert(disable_stop_calls == 1U);
    assert(!snapshot.boot_homed && !snapshot.moving);
}

static void test_boot_returns_from_upper_limit_and_rejects_outside(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_MECHANICAL_MAX_COUNTS, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    assert(enable_calls == 1U);
    make_boot_feedback(LEVER_MECHANICAL_MAX_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    assert(last_move_direction == ZDT_STEPPER_DIRECTION_CCW);
    assert(last_move_pulses == 408U);

    now_ms += 60U;
    make_boot_feedback(10000, ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS + 3,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_homed);
    assert(snapshot.boot_state ==
           LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL);

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    make_boot_feedback(LEVER_MECHANICAL_MAX_COUNTS +
                       LEVER_MECHANICAL_TOLERANCE_COUNTS + 1,
                       0U, true);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(enable_calls == 0U && move_calls == 0U);
}

static void test_horizontal_hold_tolerance_is_bounded(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_HORIZONTAL_COUNTS, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_homed);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS -
                       LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS -
                       LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS - 1,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(disable_stop_calls == 1U);
}

static void test_boot_feedback_requires_continuous_timeout(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_HORIZONTAL_COUNTS, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_WAIT_ENABLE);

    now_ms += 200U;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_homed && snapshot.fault_latched == 0U);

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_HORIZONTAL_COUNTS, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    now_ms += LEVER_BOOT_ACTIVE_FEEDBACK_TIMEOUT_MS + 1U;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_FEEDBACK_STALE);
    assert(disable_stop_calls == 1U);
}

int main(void)
{
    test_unsafe_states_emit_no_motion();
    test_limits_timeout_and_config_change_stop();
    test_boot_returns_from_gravity_reference_to_horizontal();
    test_boot_rejects_unknown_start_and_reverse_motion();
    test_boot_returns_from_upper_limit_and_rejects_outside();
    test_horizontal_hold_tolerance_is_bounded();
    test_boot_feedback_requires_continuous_timeout();
    return 0;
}
