#include "lever_actuator.h"

#include "lever_motion_config.h"
#include "h3_control_config.h"
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
static uint32_t velocity_calls;
static uint32_t stop_calls;
static uint32_t disable_stop_calls;
static uint32_t emergency_stop_calls;
static uint32_t begin_action_calls;
static uint32_t end_action_calls;
static uint32_t read_encoder_calls;
static uint32_t read_position_calls;
static uint32_t read_motor_status_calls;
static uint32_t read_config_calls;
static uint32_t last_move_pulses;
static uint16_t last_move_speed;
static ZdtStepperDirection_t last_move_direction;
static uint16_t last_velocity_speed;
static ZdtStepperDirection_t last_velocity_direction;
static SystemState_t system_state;
static bool safety_fault;
static uint8_t last_tx_function;

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
ZdtStepperResult_t ZdtStepper_ReadEncoder(void)
{ read_encoder_calls++; last_tx_function = 0x31U; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadPosition(void)
{ read_position_calls++; last_tx_function = 0x36U; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadMotorStatus(void)
{ read_motor_status_calls++; last_tx_function = 0x3AU; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_ReadDriverConfig(void)
{ read_config_calls++; last_tx_function = 0x42U; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_Enable(bool enabled)
{
    assert(enabled);
    enable_calls++;
    last_tx_function = 0xF3U;
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
    last_tx_function = 0xFDU;
    return ZDT_STEPPER_OK;
}
ZdtStepperResult_t ZdtStepper_RunVelocity(
    ZdtStepperDirection_t direction, uint16_t speed, uint8_t acceleration)
{
    (void)acceleration;
    last_velocity_direction = direction;
    last_velocity_speed = speed;
    velocity_calls++;
    last_tx_function = 0xF6U;
    return ZDT_STEPPER_OK;
}
ZdtStepperResult_t ZdtStepper_Stop(void)
{ stop_calls++; last_tx_function = 0xFEU; return ZDT_STEPPER_OK; }
ZdtStepperResult_t ZdtStepper_EmergencyStop(bool disable_after_stop)
{
    emergency_stop_calls++;
    if (disable_after_stop) disable_stop_calls++;
    stop_calls++;
    last_tx_function = disable_after_stop ? 0xF3U : 0xFEU;
    return ZDT_STEPPER_OK;
}
ZdtStepperResult_t ZdtStepper_EmergencyDisable(void)
{
    disable_stop_calls++;
    last_tx_function = 0xF3U;
    return ZDT_STEPPER_OK;
}

static void respond_to_last_tx(void)
{
    if (last_tx_function == 0U) return;
    device.response_sequence++;
    device.last_function = last_tx_function;
    device.last_response = ZDT_STEPPER_RESPONSE_DATA;
    if (last_tx_function == 0x31U) {
        device.encoder_valid = true;
        device.encoder_sequence++;
    } else if (last_tx_function == 0x36U) {
        device.position_valid = true;
        device.position_sequence++;
    } else if (last_tx_function == 0x3AU) {
        device.motor_status_valid = true;
        device.motor_status_sequence++;
    } else if (last_tx_function == 0x42U) {
        device.driver_config_valid = true;
        device.driver_config_sequence++;
    }
    last_tx_function = 0U;
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
    enable_calls = move_calls = velocity_calls = stop_calls = 0U;
    disable_stop_calls = emergency_stop_calls = 0U;
    begin_action_calls = end_action_calls = 0U;
    read_encoder_calls = read_position_calls = 0U;
    read_motor_status_calls = read_config_calls = 0U;
    last_move_pulses = 0U;
    last_move_speed = 0U;
    last_move_direction = ZDT_STEPPER_DIRECTION_CW;
    last_velocity_speed = 0U;
    last_velocity_direction = ZDT_STEPPER_DIRECTION_CW;
    system_state = SYSTEM_STATE_IDLE;
    safety_fault = false;
    last_tx_function = 0U;
    LeverActuator_Init(now_ms);
}

static uint32_t feedback_query_calls(void)
{
    return read_encoder_calls + read_position_calls +
           read_motor_status_calls;
}

static void make_feedback(int32_t counts, uint32_t fingerprint, uint8_t motor)
{
    respond_to_last_tx();
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
    respond_to_last_tx();
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
    assert(move_calls == 0U && disable_stop_calls == 1U);

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
    assert(!snapshot.armed && disable_stop_calls == 1U);

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
    assert(!snapshot.armed && !snapshot.moving && disable_stop_calls == 1U);
}

static void test_uart_queries_are_one_per_20ms_slot_and_actions_are_quiet(void)
{
    uint32_t feedback_before;
    uint32_t config_before;
    uint8_t sample;

    reset_fixture();
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 0U);
    assert(feedback_query_calls() == 0U);

    now_ms = LEVER_BOOT_SETTLE_MS - 1U;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 0U);
    assert(feedback_query_calls() == 0U);

    now_ms = LEVER_BOOT_SETTLE_MS;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 1U);
    assert(feedback_query_calls() == 0U);

    now_ms += 19U;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 1U);
    assert(feedback_query_calls() == 0U);

    now_ms += 1U;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 1U);
    assert(feedback_query_calls() == 1U);

    LeverActuator_Task(now_ms);
    assert(feedback_query_calls() == 1U);

    now_ms += 20U;
    LeverActuator_Task(now_ms);
    assert(feedback_query_calls() == 2U);

    now_ms += 20U;
    LeverActuator_Task(now_ms);
    assert(feedback_query_calls() == 3U);

    device.stats.tx_queued = 1U;
    now_ms = LEVER_BOOT_SETTLE_MS + 500U;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 1U);
    assert(feedback_query_calls() == 3U);
    device.stats.tx_queued = 0U;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 2U);
    assert(feedback_query_calls() == 3U);

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        feedback_before = feedback_query_calls();
        config_before = read_config_calls;
        make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS, 0U,
                           sample == 0U);
        if (sample == LEVER_BOOT_REFERENCE_SAMPLE_COUNT - 1U) {
            assert(enable_calls == 1U);
            assert(feedback_query_calls() == feedback_before);
            assert(read_config_calls == config_before);
        }
        now_ms += 60U;
    }

    feedback_before = feedback_query_calls();
    config_before = read_config_calls;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    assert(feedback_query_calls() == feedback_before);
    assert(read_config_calls == config_before);

    /* X42S may defer 0xFD until the finite move ends.  Dropping that reply
     * must not stop the independent 20 ms feedback pipeline. */
    last_tx_function = 0U;
    now_ms += 20U;
    LeverActuator_Task(now_ms);
    assert(feedback_query_calls() + read_config_calls ==
           feedback_before + config_before + 1U);

    now_ms += 40U;
    feedback_before = feedback_query_calls();
    config_before = read_config_calls;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(stop_calls == 0U);
    assert(feedback_query_calls() == feedback_before + 1U);
    assert(read_config_calls == config_before);
}

static void test_fault_uses_one_disable_and_100ms_quiet(void)
{
    uint32_t feedback_before;
    uint32_t config_before;

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    safety_fault = true;
    feedback_before = feedback_query_calls();
    config_before = read_config_calls;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS, 0U, true);
    assert(disable_stop_calls == 1U);
    assert(emergency_stop_calls == 0U);
    assert(feedback_query_calls() == feedback_before);
    assert(read_config_calls == config_before);

    now_ms += 99U;
    LeverActuator_Task(now_ms);
    assert(feedback_query_calls() == feedback_before);
    assert(read_config_calls == config_before);

    now_ms += 1U;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == config_before + 1U);
    assert(feedback_query_calls() == feedback_before);
}

static void test_missing_feedback_is_fail_closed_without_query_ack_deadlines(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    LeverActuator_Task(now_ms);
    assert(read_config_calls == 1U);

    now_ms += 20U;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);
    assert(disable_stop_calls == 0U);
    assert(enable_calls == 0U && move_calls == 0U);
    assert(feedback_query_calls() == 1U);

    now_ms = LEVER_BOOT_SETTLE_MS + LEVER_BOOT_REFERENCE_TIMEOUT_MS + 1U;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_FEEDBACK_STALE);
    assert(disable_stop_calls == 1U);
    assert(enable_calls == 0U && move_calls == 0U);

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    assert(enable_calls == 1U);
    while (now_ms <= LEVER_BOOT_SETTLE_MS + 1000U) {
        now_ms += 20U;
        LeverActuator_Task(now_ms);
    }
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_FEEDBACK_STALE);
    assert(disable_stop_calls == 1U);
}

static void test_boot_returns_from_lower_soft_limit_to_horizontal(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    assert(!calibration_valid);
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    assert(enable_calls == 1U);
    assert(begin_action_calls == 1U);
    assert(system_state == SYSTEM_STATE_RUN);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_WAIT_ENABLE);

    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    assert(last_move_direction == ZDT_STEPPER_DIRECTION_CW);
    assert(last_move_speed == LEVER_BOOT_HOME_SPEED_RPM);
    assert(last_move_pulses ==
           (uint32_t)(((uint64_t)(LEVER_HORIZONTAL_COUNTS -
                                  LEVER_BOOT_RECOVERY_MIN_COUNTS) *
                       200UL * 16UL +
                       32767UL) /
                      LEVER_POSITION_COUNTS_PER_REVOLUTION));
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
    assert(stop_calls == 0U);
    assert(end_action_calls == 1U);
    assert(system_state == SYSTEM_STATE_IDLE);
}

static void test_boot_rejects_unknown_start_and_reverse_motion(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS + 100L, 0U, true);
    now_ms += 60U;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS + 200L, 0U, false);
    now_ms += 60U;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS + 100L, 0U, false);
    now_ms += LEVER_BOOT_REFERENCE_TIMEOUT_MS + 1U;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS + 200L, 0U, true);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(enable_calls == 0U && move_calls == 0U);

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS + 100L, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS + 100L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    now_ms += 60U;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS + 81L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_DIRECTION_FAULT);
    assert(disable_stop_calls == 1U);
    assert(!snapshot.boot_homed && !snapshot.moving);
}

static void test_boot_returns_from_upper_soft_limit_and_rejects_outside(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint8_t sample;
    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_BOOT_RECOVERY_MAX_COUNTS, 0U,
                           sample == 0U);
        now_ms += 60U;
    }
    assert(enable_calls == 1U);
    make_boot_feedback(LEVER_BOOT_RECOVERY_MAX_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    assert(last_move_direction == ZDT_STEPPER_DIRECTION_CCW);
    assert(last_move_pulses ==
           (uint32_t)(((uint64_t)(LEVER_BOOT_RECOVERY_MAX_COUNTS -
                                  LEVER_HORIZONTAL_COUNTS) *
                       200UL * 16UL + 32767UL) /
                      LEVER_POSITION_COUNTS_PER_REVOLUTION));

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
    make_boot_feedback(LEVER_BOOT_RECOVERY_MAX_COUNTS + 1,
                       0U, true);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(enable_calls == 0U && move_calls == 0U);

    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS - 1,
                       0U, true);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_OUT_OF_RANGE);
    assert(enable_calls == 0U && move_calls == 0U);
}

static void test_horizontal_hold_recovers_inside_safety_envelope(void)
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
                       LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS - 1,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);

    /* Crossing the H3 start window must not release holding torque. */
    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS -
                       LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS - 1,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);
    assert(snapshot.boot_homed);
    assert(disable_stop_calls == 0U);
    assert(LeverActuator_BallArm() == LEVER_ACTUATOR_OUT_OF_RANGE);

    /* The real recovery envelope remains a hard fail-closed boundary. */
    now_ms += 60U;
    make_boot_feedback(LEVER_BOOT_RECOVERY_MIN_COUNTS - 1,
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

    respond_to_last_tx();
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
    respond_to_last_tx();
    now_ms += LEVER_BOOT_ACTIVE_FEEDBACK_TIMEOUT_MS + 1U;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_FEEDBACK_STALE);
    assert(disable_stop_calls == 1U);
}

static void home_horizontal(void)
{
    uint8_t sample;
    reset_fixture();
    now_ms = LEVER_BOOT_SETTLE_MS;
    for (sample = 0U; sample < LEVER_BOOT_REFERENCE_SAMPLE_COUNT; sample++) {
        make_boot_feedback(LEVER_HORIZONTAL_COUNTS, 0U,
                           sample == 0U);
        now_ms += 20U;
    }
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
}

static void home_for_ball_control(void)
{
    home_horizontal();
    assert(LeverActuator_BallArm() == LEVER_ACTUATOR_OK);
}

static void test_ball_arm_uses_the_holding_feedback_window(void)
{
    home_horizontal();
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 40L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(LeverActuator_BallArm() == LEVER_ACTUATOR_OK);

    home_horizontal();

    /* The boot holding state and UART3's complete feedback pipeline use a
     * 150 ms validity window.  K1 must not be rejected merely because its
     * edge lands after the old 100 ms active-control threshold. */
    now_ms += 120U;
    assert(LeverActuator_BallArm() == LEVER_ACTUATOR_OK);

    home_horizontal();
    now_ms += LEVER_BOOT_POSITION_MAX_AGE_MS + 1U;
    assert(LeverActuator_BallArm() == LEVER_ACTUATOR_FEEDBACK_STALE);
}

static void test_ball_velocity_tracks_latest_target_and_brakes_before_reverse(void)
{
    LeverActuatorSnapshot_t snapshot;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(100) == LEVER_ACTUATOR_OK);
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 0U && move_calls == 0U);

    assert(LeverActuator_BallSetTilt(1500) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 1U && move_calls == 0U);
    assert(last_velocity_speed == H3_LEVER_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CW);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.moving);
    assert(snapshot.target_counts == LEVER_HORIZONTAL_COUNTS + 273L);

    assert(LeverActuator_BallSetTilt(500) == LEVER_ACTUATOR_OK);
    assert(LeverActuator_BallSetTilt(-4000) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 2U && last_velocity_speed == 0U);
    LeverActuator_GetSnapshot(&snapshot);
    assert(!snapshot.moving && snapshot.ball_pending == false);

    /* A reversal is split into brake, a bounded coast window, then a reverse
     * command slot so target overshoot cannot cause an immediate fault. */
    now_ms += H3_LEVER_BRAKE_HOLD_MS;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 2U && last_velocity_speed == 0U);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 3U);
    assert(last_velocity_speed == H3_LEVER_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CCW);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.requested_tilt_mdeg == -4000);
    assert(snapshot.moving);
}

static void test_ball_breakaway_profile_has_initial_speed_and_downshifts(void)
{
    home_for_ball_control();
    assert(LeverActuator_BallSetTiltProfile(2000, true,
           H3_LEVER_VELOCITY_RPM, 5U) ==
           LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 1U);
    assert(last_velocity_speed == H3_LEVER_BREAKAWAY_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CW);

    /* The first confirmed positive count movement ends the launch boost.
     * A same-direction speed update must return to configured tracking. */
    assert(LeverActuator_BallSetTiltProfile(2500, false,
           H3_LEVER_VELOCITY_RPM, 5U) ==
           LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS + 16L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 2U);
    assert(last_velocity_speed == H3_LEVER_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CW);
}

static void test_ball_action_allows_one_feedback_refresh_window(void)
{
    LeverActuatorSnapshot_t snapshot;

    home_for_ball_control();

    /* K1 may arrive when the oldest feedback is still valid but already near
     * its age limit.  The finite action then owns one UART slot and a complete
     * encoder/position/status refresh needs three more slots.  This is normal
     * command-to-feedback handoff, not a lost-feedback fault. */
    now_ms += 80U;
    assert(LeverActuator_BallSetTilt(-4000) == LEVER_ACTUATOR_OK);
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.moving);
    assert(velocity_calls == 1U && move_calls == 0U);

    now_ms += 40U;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);
    assert(snapshot.moving);
    assert(disable_stop_calls == 0U);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 40L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);
    assert(snapshot.moving);
    assert(disable_stop_calls == 0U);

    /* The allowance remains bounded: absent every feedback reply, H3 still
     * disables once the command handoff window has elapsed. */
    home_for_ball_control();
    now_ms += 80U;
    assert(LeverActuator_BallSetTilt(-4000) == LEVER_ACTUATOR_OK);
    LeverActuator_Task(now_ms);
    now_ms += H3_LEVER_FEEDBACK_HANDOFF_MS + 1U;
    LeverActuator_Task(now_ms);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_FEEDBACK_STALE);
    assert(disable_stop_calls == 1U);
}

static void test_ball_progress_deadline_is_distinct_from_feedback_loss(void)
{
    LeverActuatorSnapshot_t snapshot;
    uint32_t sample_ms;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(-4000) == LEVER_ACTUATOR_OK);
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 1U && move_calls == 0U);

    for (sample_ms = 80U; sample_ms < H3_LEVER_PROGRESS_TIMEOUT_MS;
         sample_ms += 80U) {
        now_ms += 80U;
        make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                           ZDT_STEPPER_MOTOR_ENABLED, false);
        LeverActuator_GetSnapshot(&snapshot);
        assert(snapshot.fault_latched == 0U);
    }
    now_ms += 80U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_MOVE_TIMEOUT);
    assert(disable_stop_calls == 1U);
}

static void test_ball_velocity_learns_and_corrects_wire_polarity(void)
{
    LeverActuatorSnapshot_t snapshot;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(1500) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 1U);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CW);

    /* The first wire direction moves the encoder opposite to the requested
     * local-count direction.  Brake without latching a fault and remember the
     * inverted velocity convention. */
    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS -
                       H3_LEVER_POLARITY_PROBE_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);
    assert(velocity_calls == 2U && last_velocity_speed == 0U);

    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS -
                       H3_LEVER_POLARITY_PROBE_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 3U);
    assert(last_velocity_speed == H3_LEVER_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CCW);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U && snapshot.moving);
    assert(disable_stop_calls == 0U);
}

static void test_ball_velocity_rejects_reverse_feedback(void)
{
    LeverActuatorSnapshot_t snapshot;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(1500) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 1U);

    /* First establish the velocity polarity with a correctly directed probe;
     * subsequent reverse motion remains a hard actuator fault. */
    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS +
                       H3_LEVER_POLARITY_PROBE_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS -
                       H3_LEVER_DIRECTION_FAULT_COUNTS - 1L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == LEVER_ACTUATOR_DIRECTION_FAULT);
    assert(!snapshot.ball_active && disable_stop_calls == 1U);
}

static void test_ball_velocity_brakes_and_holds_normal_overshoot(void)
{
    LeverActuatorSnapshot_t snapshot;
    int32_t target_counts;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(H3_BREAKAWAY_MAX_MDEG) ==
           LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 1U);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS +
                       H3_LEVER_POLARITY_PROBE_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    target_counts = snapshot.target_counts;

    now_ms += 60U;
    make_boot_feedback(target_counts - 200L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U && !snapshot.moving);
    assert(velocity_calls == 2U && last_velocity_speed == 0U);

    /* Coasting through the target is held at zero speed.  It is not treated
     * as a reason to reverse immediately or latch the r80 direction fault. */
    now_ms += H3_LEVER_BRAKE_HOLD_MS;
    make_boot_feedback(target_counts + 150L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U && !snapshot.moving);
    assert(velocity_calls == 2U && disable_stop_calls == 0U);

    /* Ordinary PD remains held inside the anti-chatter window even after the
     * brake is stationary.  An explicitly bounded breakaway profile may then
     * rearm that reversal, while still using its supplied 3 RPM limit. */
    now_ms += 60U;
    make_boot_feedback(target_counts + 150L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 60U;
    make_boot_feedback(target_counts + 150L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 2U);
    assert(LeverActuator_BallSetTiltProfile(
        H3_BREAKAWAY_MAX_MDEG, true, H3_LEVER_VELOCITY_RPM,
        H3_LEVER_VELOCITY_RPM) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(target_counts + 150L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 3U);
    assert(last_velocity_speed == H3_LEVER_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CCW);
}

static void test_ball_return_waits_for_coast_to_settle_before_reverse(void)
{
    LeverActuatorSnapshot_t snapshot;
    int32_t coast_counts = LEVER_HORIZONTAL_COUNTS +
        H3_LEVER_POLARITY_PROBE_COUNTS +
        H3_LEVER_DIRECTION_FAULT_COUNTS + 40L;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(H3_BREAKAWAY_MAX_MDEG) ==
           LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS +
                       H3_LEVER_POLARITY_PROBE_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(LeverActuator_BallReturnHorizontal() == LEVER_ACTUATOR_OK);

    /* STOP is sent first.  More than the normal direction-fault window may
     * still coast in the old direction and must not be mistaken for a failed
     * reverse command. */
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS +
                       H3_LEVER_POLARITY_PROBE_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == 0U);
    now_ms += 60U;
    make_boot_feedback(coast_counts, ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U && !snapshot.moving);
    assert(velocity_calls == 2U && disable_stop_calls == 0U);

    now_ms += H3_LEVER_BRAKE_HOLD_MS;
    make_boot_feedback(coast_counts, ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 2U);
    now_ms += 60U;
    make_boot_feedback(coast_counts, ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U && snapshot.moving);
    assert(velocity_calls == 3U && last_velocity_speed != 0U);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CCW);
}

static void test_ball_return_reestablishes_finite_horizontal_hold(void)
{
    LeverActuatorSnapshot_t snapshot;
    int32_t hold_target;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(-4000) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(velocity_calls == 1U && last_velocity_speed != 0U);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 400L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(LeverActuator_BallReturnHorizontalProfile(4U) ==
           LEVER_ACTUATOR_OK);

    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 400L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == 0U);

    now_ms += H3_LEVER_BRAKE_HOLD_MS;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 400L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == 0U);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 400L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == 4U);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CW);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 6L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == 0U);

    now_ms += H3_LEVER_BRAKE_HOLD_MS;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 6L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 0U);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 6L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(move_calls == 1U);
    assert(last_move_speed == H3_LEVER_RETURN_SPEED_RPM);
    LeverActuator_GetSnapshot(&snapshot);
    hold_target = snapshot.target_counts;
    assert(snapshot.moving && snapshot.ball_returning);

    now_ms += 60U;
    make_boot_feedback(hold_target + 26L, ZDT_STEPPER_MOTOR_ENABLED |
                       ZDT_STEPPER_MOTOR_POSITION_REACHED, false);
    now_ms += 60U;
    make_boot_feedback(hold_target + 26L, ZDT_STEPPER_MOTOR_ENABLED |
                       ZDT_STEPPER_MOTOR_POSITION_REACHED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(!snapshot.ball_active && !snapshot.ball_returning);
    assert(!snapshot.moving && snapshot.fault_latched == 0U);
    assert(snapshot.target_counts == LEVER_HORIZONTAL_COUNTS);
}

static void test_ball_return_downshifts_before_horizontal_brake(void)
{
    LeverActuatorSnapshot_t snapshot;

    home_for_ball_control();
    assert(LeverActuator_BallSetTilt(-4000) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(LeverActuator_BallReturnHorizontal() == LEVER_ACTUATOR_OK);

    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += H3_LEVER_BRAKE_HOLD_MS;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS - 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == H3_LEVER_RETURN_FAST_VELOCITY_RPM);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS -
                       H3_LEVER_RETURN_SLOW_ZONE_COUNTS + 100L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.fault_latched == 0U);
    assert(last_velocity_speed == H3_LEVER_RETURN_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CW);
}

static void test_ball_active_withdrawal_reproduces_r94_return_speeds(void)
{
    LeverActuatorSnapshot_t snapshot;

    home_for_ball_control();
    assert(LeverActuator_BallSetTiltProfile(
        H3_BREAKAWAY_MAX_MDEG, true, H3_LEVER_VELOCITY_RPM,
        H3_LEVER_BREAKAWAY_VELOCITY_RPM) == LEVER_ACTUATOR_OK);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS + 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(LeverActuator_BallWithdrawHorizontal() == LEVER_ACTUATOR_OK);

    /* Brake the outward command first, then use the same 5/2 RPM staged
     * horizontal withdrawal as the successful r94 host STOP. */
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS + 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == 0U);
    now_ms += H3_LEVER_BRAKE_HOLD_MS;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS + 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    now_ms += 20U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS + 1800L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == H3_LEVER_WITHDRAW_FAST_VELOCITY_RPM);
    assert(last_velocity_direction == ZDT_STEPPER_DIRECTION_CCW);

    now_ms += 60U;
    make_boot_feedback(LEVER_HORIZONTAL_COUNTS +
                       H3_LEVER_RETURN_SLOW_ZONE_COUNTS - 100L,
                       ZDT_STEPPER_MOTOR_ENABLED, false);
    assert(last_velocity_speed == H3_LEVER_WITHDRAW_VELOCITY_RPM);
    LeverActuator_GetSnapshot(&snapshot);
    assert(snapshot.ball_active && !snapshot.ball_returning);
    assert(snapshot.fault_latched == 0U);
}

int main(void)
{
    test_unsafe_states_emit_no_motion();
    test_limits_timeout_and_config_change_stop();
    test_uart_queries_are_one_per_20ms_slot_and_actions_are_quiet();
    test_missing_feedback_is_fail_closed_without_query_ack_deadlines();
    test_fault_uses_one_disable_and_100ms_quiet();
    test_boot_returns_from_lower_soft_limit_to_horizontal();
    test_boot_rejects_unknown_start_and_reverse_motion();
    test_boot_returns_from_upper_soft_limit_and_rejects_outside();
    test_horizontal_hold_recovers_inside_safety_envelope();
    test_boot_feedback_requires_continuous_timeout();
    test_ball_arm_uses_the_holding_feedback_window();
    test_ball_velocity_tracks_latest_target_and_brakes_before_reverse();
    test_ball_breakaway_profile_has_initial_speed_and_downshifts();
    test_ball_action_allows_one_feedback_refresh_window();
    test_ball_progress_deadline_is_distinct_from_feedback_loss();
    test_ball_velocity_learns_and_corrects_wire_polarity();
    test_ball_velocity_rejects_reverse_feedback();
    test_ball_velocity_brakes_and_holds_normal_overshoot();
    test_ball_return_waits_for_coast_to_settle_before_reverse();
    test_ball_return_reestablishes_finite_horizontal_hold();
    test_ball_return_downshifts_before_horizontal_brake();
    test_ball_active_withdrawal_reproduces_r94_return_speeds();
    return 0;
}
