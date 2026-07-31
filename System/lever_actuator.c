#include "lever_actuator.h"

#include "lever_motion_config.h"
#include "lever_calibration.h"
#include "lever_calibration_types.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "zdt_stepper.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS 100UL
#define LEVER_ACTUATOR_CONFIG_POLL_MS       500UL
#define LEVER_ACTUATOR_QUERY_PERIOD_MS       20UL
#define LEVER_ACTUATOR_POSITION_TOLERANCE     8L
#define LEVER_ACTUATOR_DIRECTION_TOLERANCE   16L
#define LEVER_ACTUATOR_HORIZONTAL_TOLERANCE  32L
#define LEVER_ACTUATOR_H3_MIN_MDEG         (-2000L)
#define LEVER_ACTUATOR_H3_MAX_MDEG           2000L
#define LEVER_ACTUATOR_SPEED_RPM               30U
#define LEVER_ACTUATOR_ACCELERATION             10U

static LeverActuatorSnapshot_t s_snapshot;
static LeverCalibrationValues_t s_calibration;
static uint16_t s_reference_encoder;
static uint32_t s_seen_encoder_sequence;
static uint32_t s_seen_position_sequence;
static uint32_t s_seen_motor_sequence;
static uint32_t s_seen_config_sequence;
static uint32_t s_encoder_ms;
static uint32_t s_position_ms;
static uint32_t s_motor_ms;
static uint32_t s_config_ms;
static uint32_t s_next_query_ms;
static uint32_t s_next_config_ms;
static int32_t s_move_start_counts;
static int32_t s_last_motion_counts;
static int8_t s_expected_count_direction;
static uint8_t s_query_phase;
static uint32_t s_boot_deadline_ms;
static uint32_t s_boot_feedback_deadline_ms;
static int32_t s_boot_start_counts;
static int64_t s_boot_start_position;
static uint8_t s_boot_reference_samples;
static int8_t s_boot_expected_direction;
static bool s_boot_owns_run_state;

static bool reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static uint32_t abs_i32(int32_t value)
{
    if (value >= 0) return (uint32_t)value;
    return (uint32_t)(-(value + 1)) + 1U;
}

static uint64_t abs_i64(int64_t value)
{
    if (value >= 0) return (uint64_t)value;
    return (uint64_t)(-(value + 1)) + 1U;
}

static int64_t signed_position(const ZdtStepperSnapshot_t *device)
{
    return device->position_negative ?
        -(int64_t)device->position_raw : (int64_t)device->position_raw;
}

static int32_t divide_round(int64_t numerator, int32_t denominator)
{
    bool negative;
    uint64_t magnitude;
    uint32_t divisor;
    if (denominator == 0) return 0;
    negative = (numerator < 0) != (denominator < 0);
    magnitude = numerator < 0 ? (uint64_t)(-(numerator + 1)) + 1U :
                                (uint64_t)numerator;
    divisor = denominator < 0 ? abs_i32(denominator) :
                                (uint32_t)denominator;
    magnitude = (magnitude + divisor / 2U) / divisor;
    if (negative) return magnitude >= 2147483648ULL ? INT32_MIN :
                                                    -(int32_t)magnitude;
    return magnitude > INT32_MAX ? INT32_MAX : (int32_t)magnitude;
}

static int32_t map_angle(const int32_t *counts, int32_t angle_mdeg)
{
    uint8_t i;
    if (angle_mdeg <= s_calibration.angle_mdeg[0]) return counts[0];
    for (i = 1U; i < LEVER_CALIBRATION_POINT_COUNT; i++) {
        if (angle_mdeg <= s_calibration.angle_mdeg[i]) {
            int32_t da = s_calibration.angle_mdeg[i] -
                         s_calibration.angle_mdeg[i - 1U];
            int32_t dc = counts[i] - counts[i - 1U];
            return counts[i - 1U] + divide_round((int64_t)dc *
                (angle_mdeg - s_calibration.angle_mdeg[i - 1U]), da);
        }
    }
    return counts[LEVER_CALIBRATION_POINT_COUNT - 1U];
}

static void latch_fault(LeverActuatorResult_t fault)
{
    bool boot_active =
        (s_snapshot.boot_state >= LEVER_ACTUATOR_BOOT_WAIT_ENABLE) &&
        (s_snapshot.boot_state <= LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL);
    if (s_snapshot.fault_latched == 0U)
        s_snapshot.fault_latched = (uint32_t)fault;
    s_snapshot.moving = false;
    s_snapshot.armed = false;
    s_snapshot.boot_homed = false;
    s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_FAULT;
    (void)ZdtStepper_EmergencyStop(boot_active);
    if (s_boot_owns_run_state) {
        Safety_EndAction();
        (void)SystemState_RequestIdle();
        s_boot_owns_run_state = false;
    }
}

void LeverActuator_Init(uint32_t now_ms)
{
    ZdtStepperSnapshot_t device;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(&s_calibration, 0, sizeof(s_calibration));
    ZdtStepper_GetSnapshot(&device);
    s_seen_encoder_sequence = device.encoder_sequence;
    s_seen_position_sequence = device.position_sequence;
    s_seen_motor_sequence = device.motor_status_sequence;
    s_seen_config_sequence = device.driver_config_sequence;
    s_encoder_ms = UINT32_MAX;
    s_position_ms = UINT32_MAX;
    s_motor_ms = UINT32_MAX;
    s_config_ms = UINT32_MAX;
    s_next_query_ms = now_ms;
    s_next_config_ms = now_ms;
    s_boot_deadline_ms = now_ms + LEVER_BOOT_SETTLE_MS;
    s_boot_feedback_deadline_ms = now_ms +
        LEVER_BOOT_ACTIVE_FEEDBACK_TIMEOUT_MS;
    s_boot_start_counts = 0;
    s_boot_start_position = 0;
    s_boot_reference_samples = 0U;
    s_boot_expected_direction = 0;
    s_boot_owns_run_state = false;
    s_snapshot.motor_position_age_ms = UINT32_MAX;
    s_snapshot.calibration_valid =
        LeverCalibration_GetActive(&s_calibration);
    s_snapshot.boot_state =
        (LEVER_BOOT_PROFILE_ENABLED && !s_snapshot.calibration_valid) ?
        LEVER_ACTUATOR_BOOT_WAIT_SETTLE : LEVER_ACTUATOR_BOOT_DISABLED;
}

static void observe(uint32_t now_ms, const ZdtStepperSnapshot_t *device)
{
    if (device->encoder_sequence != s_seen_encoder_sequence) {
        s_seen_encoder_sequence = device->encoder_sequence;
        s_encoder_ms = now_ms;
        if (LeverCalibration_GetSessionReference(&s_reference_encoder, NULL))
            s_snapshot.current_counts = ZdtStepper_CircularDelta(
                device->encoder_raw, s_reference_encoder);
        else if (LEVER_ABSOLUTE_PROFILE_VALID)
            s_snapshot.current_counts = ZdtStepper_CircularDelta(
                device->encoder_raw, LEVER_ABSOLUTE_BOTTOM_ENCODER);
        else
            s_snapshot.current_counts = (int32_t)device->encoder_raw;
    }
    if (device->position_sequence != s_seen_position_sequence) {
        s_seen_position_sequence = device->position_sequence;
        s_position_ms = now_ms;
        s_snapshot.motor_position_raw = signed_position(device);
    }
    if (device->motor_status_sequence != s_seen_motor_sequence) {
        s_seen_motor_sequence = device->motor_status_sequence;
        s_motor_ms = now_ms;
    }
    if (device->driver_config_sequence != s_seen_config_sequence) {
        s_seen_config_sequence = device->driver_config_sequence;
        s_config_ms = now_ms;
        s_snapshot.driver_config_fingerprint =
            device->driver_config_fingerprint;
    }
}

static bool boot_feedback_ready(const ZdtStepperSnapshot_t *device)
{
    uint8_t forbidden = ZDT_STEPPER_MOTOR_STALLED |
        ZDT_STEPPER_MOTOR_STALL_PROTECTION |
        ZDT_STEPPER_MOTOR_LEFT_LIMIT |
        ZDT_STEPPER_MOTOR_RIGHT_LIMIT |
        ZDT_STEPPER_MOTOR_POWER_LOSS;
    return device->encoder_valid && device->position_valid &&
        device->motor_status_valid &&
        device->driver_config_valid &&
        (device->driver_config.subdivision != 0U) &&
        (s_snapshot.encoder_age_ms <= LEVER_BOOT_POSITION_MAX_AGE_MS) &&
        (s_snapshot.motor_position_age_ms <=
         LEVER_BOOT_POSITION_MAX_AGE_MS) &&
        (s_snapshot.motor_status_age_ms <=
         LEVER_BOOT_MOTOR_STATUS_MAX_AGE_MS) &&
        (s_config_ms != UINT32_MAX) &&
        (s_snapshot.time_ms - s_config_ms <= LEVER_BOOT_CONFIG_MAX_AGE_MS) &&
        ((device->motor_status & forbidden) == 0U);
}

static bool boot_position_in_mechanical_envelope(int64_t position)
{
    return position >= (int64_t)LEVER_MECHANICAL_MIN_COUNTS -
                       LEVER_MECHANICAL_TOLERANCE_COUNTS &&
           position <= (int64_t)LEVER_MECHANICAL_MAX_COUNTS +
                       LEVER_MECHANICAL_TOLERANCE_COUNTS;
}

static int64_t boot_relative_position(void)
{
    return (int64_t)s_snapshot.current_counts;
}

static void boot_finish_homing(void)
{
    (void)ZdtStepper_Stop();
    s_snapshot.moving = false;
    s_snapshot.boot_homed = true;
    s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL;
    if (s_boot_owns_run_state) {
        Safety_EndAction();
        (void)SystemState_RequestIdle();
        s_boot_owns_run_state = false;
    }
}

static bool boot_move_to_horizontal(uint32_t now_ms,
                                    const ZdtStepperSnapshot_t *device)
{
    int64_t delta = (int64_t)LEVER_HORIZONTAL_COUNTS -
                    (int64_t)s_snapshot.current_counts;
    uint64_t pulses64;
    uint32_t pulses;
    uint32_t pulses_per_revolution;
    ZdtStepperDirection_t direction;
    if (abs_i64(delta) <= LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS) {
        boot_finish_homing();
        return true;
    }
    pulses_per_revolution = 200UL * device->driver_config.subdivision;
    pulses64 = abs_i64(delta) * pulses_per_revolution + 32767U;
    pulses = (uint32_t)(pulses64 / LEVER_POSITION_COUNTS_PER_REVOLUTION);
    if (pulses == 0U) pulses = 1U;
    direction = delta > 0 ? ZDT_STEPPER_DIRECTION_CW :
                            ZDT_STEPPER_DIRECTION_CCW;
    if (device->driver_config.direction != 0U)
        direction = direction == ZDT_STEPPER_DIRECTION_CW ?
            ZDT_STEPPER_DIRECTION_CCW : ZDT_STEPPER_DIRECTION_CW;
    if (ZdtStepper_MovePosition(direction, LEVER_BOOT_HOME_SPEED_RPM,
            LEVER_BOOT_HOME_ACCELERATION, pulses,
            ZDT_STEPPER_POSITION_RELATIVE_CURRENT) != ZDT_STEPPER_OK) {
        latch_fault(LEVER_ACTUATOR_IO);
        return true;
    }
    s_boot_start_counts = s_snapshot.current_counts;
    s_boot_start_position = s_snapshot.motor_position_raw;
    s_boot_expected_direction = delta > 0 ? 1 : -1;
    s_snapshot.moving = true;
    s_snapshot.target_counts = LEVER_HORIZONTAL_COUNTS;
    s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_MOVING_HORIZONTAL;
    s_boot_deadline_ms = now_ms + LEVER_BOOT_MOVE_TIMEOUT_MS;
    return true;
}

static bool handle_boot_home(uint32_t now_ms,
                             const ZdtStepperSnapshot_t *device,
                             bool encoder_updated)
{
    if (s_snapshot.boot_state == LEVER_ACTUATOR_BOOT_DISABLED ||
        s_snapshot.boot_state == LEVER_ACTUATOR_BOOT_FAULT)
        return false;
    if (LeverCalibration_IsActive()) {
        if (s_snapshot.boot_state ==
            LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL)
            (void)ZdtStepper_EmergencyStop(true);
        s_snapshot.boot_homed = false;
        s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_DISABLED;
        return false;
    }
    if (s_snapshot.boot_state == LEVER_ACTUATOR_BOOT_WAIT_SETTLE) {
        if (!reached(now_ms, s_boot_deadline_ms)) return true;
        s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_WAIT_REFERENCE;
        s_boot_deadline_ms = now_ms + LEVER_BOOT_REFERENCE_TIMEOUT_MS;
        s_boot_reference_samples = 0U;
    }
    if (s_snapshot.boot_state == LEVER_ACTUATOR_BOOT_WAIT_REFERENCE) {
        if (Safety_HasFault()) {
            latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
            return true;
        }
        if (reached(now_ms, s_boot_deadline_ms)) {
            latch_fault(LEVER_ACTUATOR_OUT_OF_RANGE);
            return true;
        }
        if (!boot_feedback_ready(device) || !encoder_updated) return true;
        if (!boot_position_in_mechanical_envelope(
                s_snapshot.current_counts)) {
            latch_fault(LEVER_ACTUATOR_OUT_OF_RANGE);
            return true;
        }
        if ((device->motor_status & ZDT_STEPPER_MOTOR_ENABLED) != 0U) {
            latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
            return true;
        }
        if (s_boot_reference_samples == 0U) {
            s_boot_start_counts = s_snapshot.current_counts;
            s_boot_reference_samples = 1U;
            return true;
        }
        if (abs_i32(s_snapshot.current_counts -
                    s_boot_start_counts) >
                    LEVER_GRAVITY_STABILITY_TOLERANCE_COUNTS) {
            s_boot_start_counts = s_snapshot.current_counts;
            s_boot_reference_samples = 1U;
            return true;
        }
        if (s_boot_reference_samples < LEVER_BOOT_REFERENCE_SAMPLE_COUNT)
            ++s_boot_reference_samples;
        if (s_boot_reference_samples < LEVER_BOOT_REFERENCE_SAMPLE_COUNT)
            return true;
        if (SystemState_Get() != SYSTEM_STATE_IDLE ||
            !SystemState_RequestRun())
            return true;
        s_boot_owns_run_state = true;
        s_snapshot.target_counts = LEVER_HORIZONTAL_COUNTS;
        Safety_BeginAction(now_ms, LEVER_BOOT_ENABLE_TIMEOUT_MS +
                          LEVER_BOOT_MOVE_TIMEOUT_MS + 100U, 0U);
        if (ZdtStepper_Enable(true) != ZDT_STEPPER_OK) {
            latch_fault(LEVER_ACTUATOR_IO);
            return true;
        }
        s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_WAIT_ENABLE;
        s_boot_deadline_ms = now_ms + LEVER_BOOT_ENABLE_TIMEOUT_MS;
        s_boot_feedback_deadline_ms = now_ms +
            LEVER_BOOT_ACTIVE_FEEDBACK_TIMEOUT_MS;
        return true;
    }
    if (Safety_HasFault()) {
        latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
        return true;
    }
    if (!boot_feedback_ready(device)) {
        if (reached(now_ms, s_boot_feedback_deadline_ms))
            latch_fault(LEVER_ACTUATOR_FEEDBACK_STALE);
        return true;
    }
    s_boot_feedback_deadline_ms = now_ms +
        LEVER_BOOT_ACTIVE_FEEDBACK_TIMEOUT_MS;
    if (!boot_position_in_mechanical_envelope(
            boot_relative_position())) {
        latch_fault(LEVER_ACTUATOR_OUT_OF_RANGE);
        return true;
    }
    if (s_snapshot.boot_state == LEVER_ACTUATOR_BOOT_WAIT_ENABLE) {
        if ((device->motor_status & ZDT_STEPPER_MOTOR_ENABLED) != 0U)
            return boot_move_to_horizontal(now_ms, device);
        if (reached(now_ms, s_boot_deadline_ms))
            latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
        return true;
    }
    if (s_snapshot.boot_state ==
        LEVER_ACTUATOR_BOOT_MOVING_HORIZONTAL) {
        if ((device->motor_status & ZDT_STEPPER_MOTOR_ENABLED) == 0U) {
            latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
            return true;
        }
        if ((s_boot_expected_direction > 0 &&
             (s_snapshot.current_counts < s_boot_start_counts -
              LEVER_BOOT_DIRECTION_TOLERANCE_COUNTS ||
              s_snapshot.motor_position_raw < s_boot_start_position -
              LEVER_BOOT_DIRECTION_TOLERANCE_COUNTS)) ||
            (s_boot_expected_direction < 0 &&
             (s_snapshot.current_counts > s_boot_start_counts +
              LEVER_BOOT_DIRECTION_TOLERANCE_COUNTS ||
              s_snapshot.motor_position_raw > s_boot_start_position +
              LEVER_BOOT_DIRECTION_TOLERANCE_COUNTS))) {
            latch_fault(LEVER_ACTUATOR_DIRECTION_FAULT);
            return true;
        }
        if (abs_i64(((int64_t)s_snapshot.current_counts -
                     s_boot_start_counts) -
                    (s_snapshot.motor_position_raw -
                     s_boot_start_position)) >
                    LEVER_BOOT_FEEDBACK_DISAGREEMENT_COUNTS) {
            latch_fault(LEVER_ACTUATOR_DIRECTION_FAULT);
            return true;
        }
        if (abs_i64((int64_t)s_snapshot.current_counts -
                    LEVER_HORIZONTAL_COUNTS) <=
                    LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS) {
            boot_finish_homing();
            return true;
        }
        if ((s_boot_expected_direction > 0 &&
             s_snapshot.current_counts > LEVER_HORIZONTAL_COUNTS +
             LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS) ||
            (s_boot_expected_direction < 0 &&
             s_snapshot.current_counts < LEVER_HORIZONTAL_COUNTS -
             LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS)) {
            latch_fault(LEVER_ACTUATOR_OUT_OF_RANGE);
            return true;
        }
        if (reached(now_ms, s_boot_deadline_ms))
            latch_fault(LEVER_ACTUATOR_FEEDBACK_STALE);
        return true;
    }
    if (s_snapshot.boot_state ==
        LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL) {
        if ((device->motor_status & ZDT_STEPPER_MOTOR_ENABLED) == 0U ||
            abs_i64((int64_t)s_snapshot.current_counts -
                    LEVER_HORIZONTAL_COUNTS) >
                    LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS) {
            latch_fault(LEVER_ACTUATOR_OUT_OF_RANGE);
        }
        return true;
    }
    return false;
}

void LeverActuator_Task(uint32_t now_ms)
{
    ZdtStepperSnapshot_t device;
    bool encoder_updated;
    ZdtStepper_GetSnapshot(&device);
    encoder_updated = device.encoder_sequence != s_seen_encoder_sequence;
    observe(now_ms, &device);
    s_snapshot.time_ms = now_ms;
    s_snapshot.session_zero_valid =
        LeverCalibration_GetSessionReference(&s_reference_encoder, NULL);
    s_snapshot.calibration_valid =
        LeverCalibration_GetActive(&s_calibration);
    s_snapshot.encoder_age_ms = (s_encoder_ms == UINT32_MAX) ? UINT32_MAX :
                                now_ms - s_encoder_ms;
    s_snapshot.motor_status_age_ms = (s_motor_ms == UINT32_MAX) ? UINT32_MAX :
                                     now_ms - s_motor_ms;
    s_snapshot.motor_position_age_ms =
        (s_position_ms == UINT32_MAX) ? UINT32_MAX : now_ms - s_position_ms;
    if (!LeverCalibration_IsActive() && s_snapshot.calibration_valid &&
        device.driver_config_valid &&
        (device.driver_config_fingerprint !=
         s_calibration.driver_config_fingerprint)) {
        LeverCalibration_InvalidateDriverConfig(
            device.driver_config_fingerprint);
        s_snapshot.calibration_valid = false;
        if (s_snapshot.armed || s_snapshot.moving)
            latch_fault(LEVER_ACTUATOR_CONFIG_CHANGED);
    }
    if (!LeverCalibration_IsActive() && reached(now_ms, s_next_query_ms)) {
        if (s_query_phase == 0U) (void)ZdtStepper_ReadEncoder();
        else if (s_query_phase == 1U) (void)ZdtStepper_ReadPosition();
        else (void)ZdtStepper_ReadMotorStatus();
        s_query_phase = (uint8_t)((s_query_phase + 1U) % 3U);
        s_next_query_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
    }
    if (!LeverCalibration_IsActive() && reached(now_ms, s_next_config_ms)) {
        (void)ZdtStepper_ReadDriverConfig();
        s_next_config_ms = now_ms + LEVER_ACTUATOR_CONFIG_POLL_MS;
    }
    if (handle_boot_home(now_ms, &device, encoder_updated)) return;
    if (!s_snapshot.armed && !s_snapshot.moving) return;
    if (!s_snapshot.calibration_valid || !s_snapshot.session_zero_valid) {
        latch_fault(LEVER_ACTUATOR_SESSION_INVALID);
        return;
    }
    if ((s_snapshot.encoder_age_ms > LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS) ||
        (s_snapshot.motor_status_age_ms > LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS) ||
        (s_snapshot.motor_position_age_ms >
         LEVER_BOOT_POSITION_MAX_AGE_MS)) {
        latch_fault(LEVER_ACTUATOR_FEEDBACK_STALE);
        return;
    }
    if (!device.driver_config_valid ||
        (device.driver_config_fingerprint !=
         s_calibration.driver_config_fingerprint)) {
        latch_fault(LEVER_ACTUATOR_CONFIG_CHANGED);
        return;
    }
    if (!device.motor_status_valid ||
        ((device.motor_status & (ZDT_STEPPER_MOTOR_STALLED |
          ZDT_STEPPER_MOTOR_STALL_PROTECTION |
          ZDT_STEPPER_MOTOR_LEFT_LIMIT |
          ZDT_STEPPER_MOTOR_RIGHT_LIMIT |
          ZDT_STEPPER_MOTOR_POWER_LOSS)) != 0U)) {
        latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
        return;
    }
    if ((s_snapshot.current_counts < s_calibration.soft_min_counts) ||
        (s_snapshot.current_counts > s_calibration.soft_max_counts)) {
        latch_fault(LEVER_ACTUATOR_OUT_OF_RANGE);
        return;
    }
    if (!s_snapshot.moving) return;
    if (encoder_updated) {
        int32_t step = s_snapshot.current_counts - s_last_motion_counts;
        if ((s_expected_count_direction > 0 &&
             step < -LEVER_ACTUATOR_DIRECTION_TOLERANCE) ||
            (s_expected_count_direction < 0 &&
             step > LEVER_ACTUATOR_DIRECTION_TOLERANCE)) {
            latch_fault(LEVER_ACTUATOR_DIRECTION_FAULT);
            return;
        }
        s_last_motion_counts = s_snapshot.current_counts;
    }
    if ((s_expected_count_direction > 0 &&
         s_snapshot.current_counts <
         s_move_start_counts - LEVER_ACTUATOR_DIRECTION_TOLERANCE) ||
        (s_expected_count_direction < 0 &&
         s_snapshot.current_counts >
         s_move_start_counts + LEVER_ACTUATOR_DIRECTION_TOLERANCE)) {
        latch_fault(LEVER_ACTUATOR_DIRECTION_FAULT);
        return;
    }
    if (abs_i32(s_snapshot.current_counts - s_snapshot.target_counts) <=
        LEVER_ACTUATOR_POSITION_TOLERANCE) {
        s_snapshot.moving = false;
        (void)ZdtStepper_Stop();
    }
}

LeverActuatorResult_t LeverActuator_ArmHorizontal(void)
{
    ZdtStepperSnapshot_t device;
    uint32_t now_ms = SystemTime_Now();
    int32_t horizontal;
    if (!LeverCalibration_GetActive(&s_calibration))
        return LEVER_ACTUATOR_NOT_CALIBRATED;
    if (!LeverCalibration_GetSessionReference(&s_reference_encoder,
                                               &horizontal))
        return LEVER_ACTUATOR_SESSION_INVALID;
    ZdtStepper_GetSnapshot(&device);
    observe(now_ms, &device);
    if ((s_encoder_ms == UINT32_MAX) || (s_position_ms == UINT32_MAX) ||
        (s_motor_ms == UINT32_MAX) ||
        (now_ms - s_encoder_ms > LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS) ||
        (now_ms - s_position_ms > LEVER_BOOT_POSITION_MAX_AGE_MS) ||
        (now_ms - s_motor_ms > LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS))
        return LEVER_ACTUATOR_FEEDBACK_STALE;
    if (!device.driver_config_valid ||
        (device.driver_config_fingerprint !=
         s_calibration.driver_config_fingerprint))
        return LEVER_ACTUATOR_CONFIG_CHANGED;
    if (!device.motor_status_valid ||
        ((device.motor_status & (ZDT_STEPPER_MOTOR_ENABLED |
          ZDT_STEPPER_MOTOR_STALLED |
          ZDT_STEPPER_MOTOR_STALL_PROTECTION)) != 0U))
        return LEVER_ACTUATOR_MOTOR_STATE;
    if (abs_i32(s_snapshot.current_counts - horizontal) >
        LEVER_ACTUATOR_HORIZONTAL_TOLERANCE)
        return LEVER_ACTUATOR_OUT_OF_RANGE;
    if (ZdtStepper_Enable(true) != ZDT_STEPPER_OK)
        return LEVER_ACTUATOR_IO;
    s_snapshot.fault_latched = 0U;
    s_snapshot.armed = true;
    return LEVER_ACTUATOR_OK;
}

LeverActuatorResult_t LeverActuator_SetTilt(int32_t angle_mdeg)
{
    ZdtStepperSnapshot_t device;
    const int32_t *table;
    int32_t target;
    int32_t delta;
    uint64_t pulses64;
    uint32_t pulses;
    uint32_t pulses_per_revolution;
    ZdtStepperDirection_t direction;
    uint32_t now_ms = SystemTime_Now();
    if (!s_snapshot.armed || s_snapshot.moving)
        return LEVER_ACTUATOR_BUSY;
    LeverActuator_Task(now_ms);
    if (s_snapshot.fault_latched != 0U)
        return (LeverActuatorResult_t)s_snapshot.fault_latched;
    if ((angle_mdeg < LEVER_ACTUATOR_H3_MIN_MDEG) ||
        (angle_mdeg > LEVER_ACTUATOR_H3_MAX_MDEG) ||
        (angle_mdeg < s_calibration.soft_min_angle_mdeg) ||
        (angle_mdeg > s_calibration.soft_max_angle_mdeg))
        return LEVER_ACTUATOR_OUT_OF_RANGE;
    table = angle_mdeg >= s_snapshot.target_angle_mdeg ?
        s_calibration.increasing_counts : s_calibration.decreasing_counts;
    target = map_angle(table, angle_mdeg);
    if ((target < s_calibration.soft_min_counts) ||
        (target > s_calibration.soft_max_counts) ||
        (s_snapshot.current_counts < s_calibration.soft_min_counts) ||
        (s_snapshot.current_counts > s_calibration.soft_max_counts))
        return LEVER_ACTUATOR_OUT_OF_RANGE;
    delta = target - s_snapshot.current_counts;
    if (abs_i32(delta) <= LEVER_ACTUATOR_POSITION_TOLERANCE) {
        s_snapshot.target_angle_mdeg = angle_mdeg;
        s_snapshot.target_counts = target;
        return LEVER_ACTUATOR_OK;
    }
    ZdtStepper_GetSnapshot(&device);
    if (!device.driver_config_valid ||
        (device.driver_config.subdivision == 0U))
        return LEVER_ACTUATOR_CONFIG_CHANGED;
    pulses_per_revolution = 200UL * device.driver_config.subdivision;
    pulses64 = (uint64_t)abs_i32(delta) * pulses_per_revolution + 32767U;
    pulses = (uint32_t)(pulses64 / 65536U);
    if (pulses == 0U) pulses = 1U;
    direction = delta > 0 ? ZDT_STEPPER_DIRECTION_CW :
                            ZDT_STEPPER_DIRECTION_CCW;
    if (device.driver_config.direction != 0U)
        direction = direction == ZDT_STEPPER_DIRECTION_CW ?
            ZDT_STEPPER_DIRECTION_CCW : ZDT_STEPPER_DIRECTION_CW;
    if (ZdtStepper_MovePosition(direction, LEVER_ACTUATOR_SPEED_RPM,
            LEVER_ACTUATOR_ACCELERATION, pulses,
            ZDT_STEPPER_POSITION_RELATIVE_CURRENT) != ZDT_STEPPER_OK)
        return LEVER_ACTUATOR_IO;
    s_move_start_counts = s_snapshot.current_counts;
    s_last_motion_counts = s_snapshot.current_counts;
    s_expected_count_direction = delta > 0 ? 1 : -1;
    s_snapshot.target_angle_mdeg = angle_mdeg;
    s_snapshot.target_counts = target;
    s_snapshot.moving = true;
    return LEVER_ACTUATOR_OK;
}

void LeverActuator_RequestStop(void)
{
    (void)ZdtStepper_EmergencyStop(true);
    s_snapshot.moving = false;
    s_snapshot.armed = false;
    s_snapshot.boot_homed = false;
    s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_DISABLED;
    if (s_boot_owns_run_state) {
        Safety_EndAction();
        (void)SystemState_RequestIdle();
        s_boot_owns_run_state = false;
    }
}

void LeverActuator_GetSnapshot(LeverActuatorSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = s_snapshot;
}
