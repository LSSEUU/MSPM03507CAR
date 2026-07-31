#include "lever_actuator.h"

#include "lever_motion_config.h"
#include "h3_control_config.h"
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
#define LEVER_ACTUATOR_STARTUP_QUIET_MS LEVER_BOOT_SETTLE_MS
#define LEVER_ACTUATOR_EMERGENCY_QUIET_MS   100UL
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
static uint32_t s_uart_quiet_until_ms;
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
static uint32_t s_ball_move_deadline_ms;
static uint32_t s_ball_feedback_handoff_ms;
static int32_t s_ball_move_start_counts;
static int64_t s_ball_move_start_position;
static int32_t s_ball_progress_counts;
static int8_t s_ball_expected_direction;
static int8_t s_ball_velocity_direction;
static int8_t s_ball_braked_direction;
static int32_t s_ball_brake_counts;
static uint8_t s_ball_brake_stationary_samples;
static uint8_t s_ball_completion_samples;
static uint8_t s_ball_velocity_polarity_attempts;
static bool s_ball_return_hold_active;
static bool s_ball_velocity_polarity_known;
static bool s_ball_velocity_wire_inverted;
static bool s_ball_breakaway_fast;
static bool s_ball_withdrawing;
static uint32_t s_ball_brake_hold_until_ms;
static uint16_t s_ball_velocity_speed_rpm;
static uint16_t s_ball_tracking_rpm;
static uint16_t s_ball_breakaway_rpm;
static uint16_t s_ball_return_rpm_override;

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

static bool ball_brake_settled(uint32_t now_ms)
{
    return s_ball_braked_direction == 0 ||
        (reached(now_ms, s_ball_brake_hold_until_ms) &&
         s_ball_brake_stationary_samples >=
         H3_LEVER_BRAKE_SETTLE_SAMPLES);
}

static int32_t angle_to_local_counts(int32_t angle_mdeg)
{
    return divide_round((int64_t)angle_mdeg *
        LEVER_POSITION_COUNTS_PER_REVOLUTION, 360000L);
}

static int32_t local_counts_to_angle(int32_t counts)
{
    return divide_round((int64_t)counts * 360000L,
                        LEVER_POSITION_COUNTS_PER_REVOLUTION);
}

static void latch_fault(LeverActuatorResult_t fault)
{
    if (s_snapshot.fault_latched == 0U)
        s_snapshot.fault_latched = (uint32_t)fault;
    s_snapshot.moving = false;
    s_snapshot.armed = false;
    s_snapshot.ball_active = false;
    s_snapshot.ball_returning = false;
    s_snapshot.ball_pending = false;
    s_ball_withdrawing = false;
    s_snapshot.boot_homed = false;
    s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_FAULT;
    /* A single urgent disable frame is fail-closed.  Sending stop and disable
     * back-to-back can overlap X42S replies and corrupt the following frame. */
    (void)ZdtStepper_EmergencyDisable();
    s_uart_quiet_until_ms = s_snapshot.time_ms +
        LEVER_ACTUATOR_EMERGENCY_QUIET_MS;
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
    /* The MCU can start well before the X42S.  Keep UART3 silent until the
     * driver power-up window has elapsed; a manual MCU reset then follows the
     * same deterministic path without depending on the driver's prior state. */
    s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_STARTUP_QUIET_MS;
    s_boot_deadline_ms = now_ms + LEVER_BOOT_SETTLE_MS;
    s_boot_feedback_deadline_ms = now_ms +
        LEVER_BOOT_ACTIVE_FEEDBACK_TIMEOUT_MS;
    s_boot_start_counts = 0;
    s_boot_start_position = 0;
    s_boot_reference_samples = 0U;
    s_boot_expected_direction = 0;
    s_boot_owns_run_state = false;
    s_ball_move_deadline_ms = now_ms;
    s_ball_feedback_handoff_ms = now_ms;
    s_ball_move_start_counts = 0;
    s_ball_move_start_position = 0;
    s_ball_progress_counts = 0;
    s_ball_expected_direction = 0;
    s_ball_velocity_direction = 0;
    s_ball_braked_direction = 0;
    s_ball_brake_counts = 0;
    s_ball_brake_stationary_samples = 0U;
    s_ball_brake_hold_until_ms = now_ms;
    s_ball_completion_samples = 0U;
    s_ball_velocity_polarity_attempts = 0U;
    s_ball_return_hold_active = false;
    s_ball_velocity_polarity_known = false;
    s_ball_velocity_wire_inverted = false;
    s_ball_breakaway_fast = false;
    s_ball_withdrawing = false;
    s_ball_velocity_speed_rpm = 0U;
    s_ball_return_rpm_override = 0U;
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

static bool boot_position_in_recovery_envelope(int64_t position)
{
    return position >= (int64_t)LEVER_BOOT_RECOVERY_MIN_COUNTS &&
           position <= (int64_t)LEVER_BOOT_RECOVERY_MAX_COUNTS;
}

static int64_t boot_relative_position(void)
{
    return (int64_t)s_snapshot.current_counts;
}

static void boot_finish_homing(void)
{
    /* A finite position command stops by itself.  Sending 0xFE here cancels
     * the driver's position hold even though 0x3A can continue to report
     * enabled and reached.  Normal completion therefore sends no command;
     * explicit stop and fault paths still use the emergency primitives. */
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
    /* A finite move reply may arrive only when the long move completes.
     * Do not turn that optional action reply into a 250 ms deadline: after
     * the quiet slot, 0x31/0x36/0x3A feedback owns completion and faults. */
    s_boot_start_counts = s_snapshot.current_counts;
    s_boot_start_position = s_snapshot.motor_position_raw;
    s_boot_expected_direction = delta > 0 ? 1 : -1;
    s_snapshot.moving = true;
    s_snapshot.target_counts = LEVER_HORIZONTAL_COUNTS;
    s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_MOVING_HORIZONTAL;
    s_boot_deadline_ms = now_ms + LEVER_BOOT_MOVE_TIMEOUT_MS;
    s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
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
            LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL) {
            (void)ZdtStepper_EmergencyDisable();
            s_uart_quiet_until_ms = now_ms +
                LEVER_ACTUATOR_EMERGENCY_QUIET_MS;
        }
        s_snapshot.boot_homed = false;
        s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_DISABLED;
        return false;
    }
    /* Once the horizontal hold has explicitly handed control to H3, its
     * bounded action/feedback state machine is the sole owner.  Letting the
     * boot-home freshness gate run first makes the two state machines race
     * during the finite-command UART quiet slot. */
    if ((s_snapshot.boot_state ==
         LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL) &&
        s_snapshot.ball_active)
        return false;
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
            latch_fault(s_boot_reference_samples == 0U ?
                LEVER_ACTUATOR_FEEDBACK_STALE :
                LEVER_ACTUATOR_OUT_OF_RANGE);
            return true;
        }
        if (!boot_feedback_ready(device) || !encoder_updated) return true;
        if (!boot_position_in_recovery_envelope(
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
        s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
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
    if (!boot_position_in_recovery_envelope(
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
        /* A small external deflection is not a fault: keep the driver
         * enabled so its closed position loop can recover the target.  The
         * common recovery-envelope check above still fails closed at the
         * real safety boundary, while BallArm independently requires the
         * tighter horizontal start window. */
        if ((device->motor_status & ZDT_STEPPER_MOTOR_ENABLED) == 0U)
            latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
        return true;
    }
    return false;
}

static bool ball_motor_state_valid(const ZdtStepperSnapshot_t *device)
{
    uint8_t forbidden = ZDT_STEPPER_MOTOR_STALLED |
        ZDT_STEPPER_MOTOR_STALL_PROTECTION |
        ZDT_STEPPER_MOTOR_LEFT_LIMIT |
        ZDT_STEPPER_MOTOR_RIGHT_LIMIT |
        ZDT_STEPPER_MOTOR_POWER_LOSS;
    return device->motor_status_valid &&
        ((device->motor_status & ZDT_STEPPER_MOTOR_ENABLED) != 0U) &&
        ((device->motor_status & forbidden) == 0U);
}

static int32_t ball_requested_counts(void)
{
    int32_t target = LEVER_HORIZONTAL_COUNTS +
        angle_to_local_counts(s_snapshot.requested_tilt_mdeg);
    int32_t hard_min = LEVER_HORIZONTAL_COUNTS -
        angle_to_local_counts(H3_LEVER_NEGATIVE_HARD_LIMIT_MDEG);
    int32_t hard_max = LEVER_HORIZONTAL_COUNTS +
        angle_to_local_counts(H3_LEVER_POSITIVE_HARD_LIMIT_MDEG);
    if (target < hard_min) target = hard_min;
    if (target > hard_max) target = hard_max;
    if (target < LEVER_SOFT_MIN_COUNTS) target = LEVER_SOFT_MIN_COUNTS;
    if (target > LEVER_SOFT_MAX_COUNTS) target = LEVER_SOFT_MAX_COUNTS;
    return target;
}

static uint16_t ball_requested_velocity_speed(void)
{
    if (s_snapshot.ball_returning) {
        if (s_ball_return_rpm_override != 0U)
            return s_ball_return_rpm_override;
        uint32_t error = abs_i32(
            ball_requested_counts() - s_snapshot.current_counts);
        return error > H3_LEVER_RETURN_SLOW_ZONE_COUNTS ?
            H3_LEVER_RETURN_FAST_VELOCITY_RPM :
            H3_LEVER_RETURN_VELOCITY_RPM;
    }
    if (s_ball_withdrawing) {
        uint32_t error = abs_i32(
            ball_requested_counts() - s_snapshot.current_counts);
        return error > H3_LEVER_RETURN_SLOW_ZONE_COUNTS ?
            H3_LEVER_WITHDRAW_FAST_VELOCITY_RPM :
            H3_LEVER_WITHDRAW_VELOCITY_RPM;
    }
    return s_ball_breakaway_fast ? s_ball_breakaway_rpm :
                                   s_ball_tracking_rpm;
}

static ZdtStepperDirection_t ball_wire_direction(
    int8_t logical_direction, const ZdtStepperSnapshot_t *device)
{
    ZdtStepperDirection_t direction = logical_direction >= 0 ?
        ZDT_STEPPER_DIRECTION_CW : ZDT_STEPPER_DIRECTION_CCW;
    if (device->driver_config.direction != 0U)
        direction = direction == ZDT_STEPPER_DIRECTION_CW ?
            ZDT_STEPPER_DIRECTION_CCW : ZDT_STEPPER_DIRECTION_CW;
    return direction;
}

static ZdtStepperDirection_t ball_velocity_wire_direction(
    int8_t logical_direction, const ZdtStepperSnapshot_t *device)
{
    ZdtStepperDirection_t direction = ball_wire_direction(logical_direction,
                                                           device);
    if (s_ball_velocity_wire_inverted)
        direction = direction == ZDT_STEPPER_DIRECTION_CW ?
            ZDT_STEPPER_DIRECTION_CCW : ZDT_STEPPER_DIRECTION_CW;
    return direction;
}

static void ball_begin_feedback_handoff(uint32_t now_ms)
{
    /* Do not let repeated commands extend a stale-link allowance forever.
     * Only a command launched from a complete fresh feedback set opens a new
     * handoff window.  BallArm has its own already-validated initial window. */
    if ((s_snapshot.encoder_age_ms <= LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS) &&
        (s_snapshot.motor_status_age_ms <=
         LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS) &&
        (s_snapshot.motor_position_age_ms <=
         LEVER_BOOT_POSITION_MAX_AGE_MS))
        s_ball_feedback_handoff_ms = now_ms +
            H3_LEVER_FEEDBACK_HANDOFF_MS;
}

static LeverActuatorResult_t ball_set_velocity(
    uint32_t now_ms, const ZdtStepperSnapshot_t *device,
    int8_t logical_direction)
{
    int8_t prior_direction = s_ball_velocity_direction;
    ZdtStepperDirection_t direction = ball_velocity_wire_direction(
        logical_direction != 0 ? logical_direction :
        (s_ball_velocity_direction != 0 ? s_ball_velocity_direction : 1),
        device);
    uint16_t speed = logical_direction == 0 ? 0U :
        ball_requested_velocity_speed();
    uint8_t acceleration = s_snapshot.ball_returning ?
        H3_LEVER_RETURN_VELOCITY_ACCELERATION :
        (s_ball_withdrawing ?
         H3_LEVER_WITHDRAW_VELOCITY_ACCELERATION :
        (s_ball_breakaway_fast ?
         H3_LEVER_BREAKAWAY_VELOCITY_ACCELERATION :
         H3_LEVER_VELOCITY_ACCELERATION));
    if (ZdtStepper_RunVelocity(direction, speed,
            acceleration) != ZDT_STEPPER_OK)
        return LEVER_ACTUATOR_IO;

    if ((logical_direction == 0) && (prior_direction != 0)) {
        s_ball_brake_counts = s_snapshot.current_counts;
        s_ball_brake_stationary_samples = 0U;
    } else if (logical_direction != 0) {
        s_ball_brake_stationary_samples = 0U;
    }

    ball_begin_feedback_handoff(now_ms);
    s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
    s_ball_velocity_direction = logical_direction;
    s_ball_velocity_speed_rpm = speed;
    s_snapshot.ball_velocity_direction = logical_direction;
    s_snapshot.ball_velocity_speed_rpm = speed;
    s_ball_expected_direction = logical_direction;
    s_ball_move_start_counts = s_snapshot.current_counts;
    s_ball_move_start_position = s_snapshot.motor_position_raw;
    s_ball_progress_counts = s_snapshot.current_counts;
    s_ball_move_deadline_ms = now_ms + H3_LEVER_PROGRESS_TIMEOUT_MS;
    s_snapshot.moving = logical_direction != 0;
    s_snapshot.ball_pending = false;
    return LEVER_ACTUATOR_OK;
}

static LeverActuatorResult_t ball_start_return_hold(
    uint32_t now_ms, const ZdtStepperSnapshot_t *device)
{
    int32_t delta = LEVER_HORIZONTAL_COUNTS - s_snapshot.current_counts;
    int32_t commanded_counts;
    uint32_t pulses_per_revolution;
    uint64_t pulses64;
    uint32_t pulses;
    int8_t logical_direction = delta < 0 ? -1 : 1;
    ZdtStepperDirection_t direction;

    if (!device->driver_config_valid ||
        (device->driver_config.subdivision == 0U))
        return LEVER_ACTUATOR_CONFIG_CHANGED;
    pulses_per_revolution = 200UL * device->driver_config.subdivision;
    pulses64 = (uint64_t)abs_i32(delta) * pulses_per_revolution + 32767U;
    pulses = (uint32_t)(pulses64 / 65536U);
    commanded_counts = divide_round(
        (int64_t)pulses * LEVER_POSITION_COUNTS_PER_REVOLUTION,
        (int32_t)pulses_per_revolution);
    if (delta < 0) commanded_counts = -commanded_counts;
    direction = ball_wire_direction(logical_direction, device);
    if (ZdtStepper_MovePosition(direction, H3_LEVER_RETURN_SPEED_RPM,
            H3_LEVER_RETURN_ACCELERATION, pulses,
            ZDT_STEPPER_POSITION_RELATIVE_CURRENT) != ZDT_STEPPER_OK)
        return LEVER_ACTUATOR_IO;

    ball_begin_feedback_handoff(now_ms);
    s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
    s_ball_return_hold_active = true;
    s_ball_velocity_direction = 0;
    s_ball_expected_direction = pulses == 0U ? 0 : logical_direction;
    s_ball_braked_direction = 0;
    s_ball_brake_stationary_samples = 0U;
    s_ball_move_start_counts = s_snapshot.current_counts;
    s_ball_move_start_position = s_snapshot.motor_position_raw;
    s_ball_progress_counts = s_snapshot.current_counts;
    s_ball_completion_samples = 0U;
    s_ball_move_deadline_ms = now_ms + H3_LEVER_RETURN_HOLD_TIMEOUT_MS;
    s_snapshot.target_counts = s_snapshot.current_counts + commanded_counts;
    s_snapshot.target_angle_mdeg = local_counts_to_angle(
        s_snapshot.target_counts - LEVER_HORIZONTAL_COUNTS);
    s_snapshot.moving = true;
    s_snapshot.ball_pending = false;
    return LEVER_ACTUATOR_OK;
}

static void ball_finish_return(void)
{
    s_snapshot.moving = false;
    s_snapshot.ball_pending = false;
    s_snapshot.ball_returning = false;
    s_snapshot.ball_active = false;
    s_snapshot.requested_tilt_mdeg = 0;
    s_snapshot.pending_tilt_mdeg = 0;
    s_snapshot.target_angle_mdeg = 0;
    s_snapshot.target_counts = LEVER_HORIZONTAL_COUNTS;
    s_snapshot.ball_motion_sequence++;
    s_ball_velocity_direction = 0;
    s_ball_expected_direction = 0;
    s_ball_braked_direction = 0;
    s_ball_brake_stationary_samples = 0U;
    s_ball_completion_samples = 0U;
    s_ball_return_hold_active = false;
    s_ball_withdrawing = false;
    s_ball_velocity_speed_rpm = 0U;
    s_ball_return_rpm_override = 0U;
    s_snapshot.ball_velocity_speed_rpm = 0U;
    s_snapshot.ball_velocity_direction = 0;
}

static void ball_task(uint32_t now_ms, const ZdtStepperSnapshot_t *device,
                      bool encoder_updated)
{
    int64_t encoder_delta;
    int64_t motor_delta;
    int32_t target;
    int32_t error;
    int32_t progress;
    int8_t desired_direction;
    LeverActuatorResult_t result;

    if ((s_snapshot.encoder_age_ms > LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS) ||
        (s_snapshot.motor_status_age_ms >
         LEVER_ACTUATOR_FEEDBACK_MAX_AGE_MS) ||
        (s_snapshot.motor_position_age_ms > LEVER_BOOT_POSITION_MAX_AGE_MS)) {
        if (!reached(now_ms, s_ball_feedback_handoff_ms)) return;
        latch_fault(LEVER_ACTUATOR_FEEDBACK_STALE);
        return;
    }
    if (!device->driver_config_valid || !ball_motor_state_valid(device)) {
        latch_fault(LEVER_ACTUATOR_MOTOR_STATE);
        return;
    }
    if ((s_snapshot.current_counts < LEVER_SOFT_MIN_COUNTS) ||
        (s_snapshot.current_counts > LEVER_SOFT_MAX_COUNTS) ||
        (s_snapshot.current_counts < LEVER_HORIZONTAL_COUNTS -
         angle_to_local_counts(H3_LEVER_NEGATIVE_HARD_LIMIT_MDEG)) ||
        (s_snapshot.current_counts > LEVER_HORIZONTAL_COUNTS +
         angle_to_local_counts(H3_LEVER_POSITIVE_HARD_LIMIT_MDEG))) {
        latch_fault(LEVER_ACTUATOR_OUT_OF_RANGE);
        return;
    }

    if ((s_ball_velocity_direction != 0) && encoder_updated &&
        !s_ball_velocity_polarity_known) {
        encoder_delta = (int64_t)s_snapshot.current_counts -
                        s_ball_move_start_counts;
        if (abs_i64(encoder_delta) >= H3_LEVER_POLARITY_PROBE_COUNTS) {
            bool direction_matches =
                ((s_ball_expected_direction > 0) && (encoder_delta > 0)) ||
                ((s_ball_expected_direction < 0) && (encoder_delta < 0));
            if (direction_matches) {
                s_ball_velocity_polarity_known = true;
                s_ball_progress_counts = s_snapshot.current_counts;
                s_ball_move_deadline_ms = now_ms +
                    H3_LEVER_PROGRESS_TIMEOUT_MS;
            } else {
                /* The X42S velocity-direction convention can differ from its
                 * finite-position convention.  Learn it with a sub-degree
                 * encoder probe before the ball can overcome static friction,
                 * brake, then retry through the opposite wire direction. */
                s_ball_velocity_polarity_attempts++;
                if (s_ball_velocity_polarity_attempts >=
                    H3_LEVER_POLARITY_MAX_ATTEMPTS) {
                    latch_fault(LEVER_ACTUATOR_DIRECTION_FAULT);
                    return;
                }
                s_ball_velocity_wire_inverted =
                    !s_ball_velocity_wire_inverted;
                result = ball_set_velocity(now_ms, device, 0);
                if (result != LEVER_ACTUATOR_OK) latch_fault(result);
                return;
            }
        }
    }

    if ((s_ball_velocity_direction == 0) &&
        (s_ball_braked_direction != 0) && encoder_updated) {
        if (abs_i32(s_snapshot.current_counts - s_ball_brake_counts) <=
            H3_LEVER_BRAKE_SETTLE_COUNTS) {
            if (s_ball_brake_stationary_samples <
                H3_LEVER_BRAKE_SETTLE_SAMPLES)
                s_ball_brake_stationary_samples++;
        } else {
            s_ball_brake_stationary_samples = 0U;
        }
        s_ball_brake_counts = s_snapshot.current_counts;
        if (!ball_brake_settled(now_ms) &&
            reached(now_ms, s_ball_move_deadline_ms)) {
            latch_fault(LEVER_ACTUATOR_MOVE_TIMEOUT);
            return;
        }
    }

    if (((s_ball_velocity_direction != 0) &&
         s_ball_velocity_polarity_known) || s_ball_return_hold_active) {
        encoder_delta = (int64_t)s_snapshot.current_counts -
                        s_ball_move_start_counts;
        motor_delta = s_snapshot.motor_position_raw -
                      s_ball_move_start_position;
        if (((s_ball_expected_direction > 0) &&
             ((encoder_delta < -H3_LEVER_DIRECTION_FAULT_COUNTS) ||
              (motor_delta < -H3_LEVER_DIRECTION_FAULT_COUNTS))) ||
            ((s_ball_expected_direction < 0) &&
             ((encoder_delta > H3_LEVER_DIRECTION_FAULT_COUNTS) ||
              (motor_delta > H3_LEVER_DIRECTION_FAULT_COUNTS))) ||
            (abs_i64(encoder_delta - motor_delta) >
             H3_LEVER_FEEDBACK_DISAGREEMENT_COUNTS)) {
            latch_fault(LEVER_ACTUATOR_DIRECTION_FAULT);
            return;
        }
    }

    if (s_ball_return_hold_active) {
        if (encoder_updated) {
            if (abs_i32(s_snapshot.current_counts -
                        s_snapshot.target_counts) <=
                H3_LEVER_COMPLETION_TOLERANCE_COUNTS) {
                if (s_ball_completion_samples <
                    H3_LEVER_COMPLETION_SAMPLES)
                    s_ball_completion_samples++;
            } else {
                s_ball_completion_samples = 0U;
            }
        }
        if (s_ball_completion_samples >= H3_LEVER_COMPLETION_SAMPLES) {
            ball_finish_return();
            return;
        }
        if (reached(now_ms, s_ball_move_deadline_ms)) {
            latch_fault(LEVER_ACTUATOR_MOVE_TIMEOUT);
            return;
        }
        return;
    }

    if ((s_ball_velocity_direction != 0) && encoder_updated) {
        progress = s_snapshot.current_counts - s_ball_progress_counts;
        if (((s_ball_velocity_direction > 0) &&
             (progress >= H3_LEVER_PROGRESS_MIN_COUNTS)) ||
            ((s_ball_velocity_direction < 0) &&
             (progress <= -H3_LEVER_PROGRESS_MIN_COUNTS))) {
            s_ball_progress_counts = s_snapshot.current_counts;
            s_ball_move_deadline_ms = now_ms +
                H3_LEVER_PROGRESS_TIMEOUT_MS;
        }
    }
    if ((s_ball_velocity_direction != 0) &&
        reached(now_ms, s_ball_move_deadline_ms)) {
        latch_fault(LEVER_ACTUATOR_MOVE_TIMEOUT);
        return;
    }

    target = ball_requested_counts();
    error = target - s_snapshot.current_counts;
    s_snapshot.target_counts = target;
    s_snapshot.target_angle_mdeg = local_counts_to_angle(
        target - LEVER_HORIZONTAL_COUNTS);
    s_snapshot.pending_tilt_mdeg = s_snapshot.requested_tilt_mdeg;

    if (s_ball_velocity_direction == 0) {
        desired_direction = abs_i32(error) >
            H3_LEVER_START_DEADBAND_COUNTS ?
            (error > 0 ? 1 : -1) : 0;
        if (!ball_brake_settled(now_ms)) {
            desired_direction = 0;
        } else if (!s_snapshot.ball_returning &&
                   (s_ball_braked_direction != 0) &&
                   (desired_direction == -s_ball_braked_direction) &&
                   !s_ball_breakaway_fast &&
                   (abs_i32(error) <=
                    H3_LEVER_REVERSE_REARM_COUNTS)) {
            /* Normal target overshoot is a stopped hold, not a reason to
             * reverse immediately and trip a direction fault. */
            desired_direction = 0;
        } else if (desired_direction != 0) {
            s_ball_braked_direction = 0;
            s_ball_brake_stationary_samples = 0U;
        }
    } else if ((abs_i32(error) <= H3_LEVER_BRAKE_MARGIN_COUNTS) ||
               ((error > 0 ? 1 : -1) != s_ball_velocity_direction)) {
        /* Brake before reversing.  This bounds overshoot and guarantees that
         * a target crossing consumes a distinct, observable command slot. */
        s_ball_braked_direction = s_ball_velocity_direction;
        s_ball_brake_hold_until_ms = now_ms + H3_LEVER_BRAKE_HOLD_MS;
        desired_direction = 0;
    } else {
        desired_direction = s_ball_velocity_direction;
    }

    if ((desired_direction != s_ball_velocity_direction) ||
        ((desired_direction != 0) &&
         (s_ball_velocity_speed_rpm !=
          ball_requested_velocity_speed()))) {
        if (!reached(now_ms, s_uart_quiet_until_ms) ||
            (device->stats.tx_queued != 0U))
            return;
        result = ball_set_velocity(now_ms, device, desired_direction);
        if (result != LEVER_ACTUATOR_OK) latch_fault(result);
        return;
    }
    s_snapshot.ball_pending = false;

    if (s_snapshot.ball_returning &&
        (s_ball_velocity_direction == 0) &&
        ball_brake_settled(now_ms) &&
        (abs_i32(error) <= H3_LEVER_START_DEADBAND_COUNTS)) {
        if (!reached(now_ms, s_uart_quiet_until_ms) ||
            (device->stats.tx_queued != 0U))
            return;
        result = ball_start_return_hold(now_ms, device);
        if (result != LEVER_ACTUATOR_OK) latch_fault(result);
    }
}

static void schedule_feedback_queries(uint32_t now_ms,
                                      const ZdtStepperSnapshot_t *device)
{
    if (LeverCalibration_IsActive()) return;
    if (!reached(now_ms, s_uart_quiet_until_ms) ||
        (device->stats.tx_queued != 0U))
        return;

    /* Preserve A1's 20 ms feedback pipeline while allowing at most one UART3
     * frame in a slot.  Waiting for each individual reply reduced the measured
     * response rate from 50 Hz to 10 Hz, making the three 150 ms freshness
     * gates impossible to satisfy.  Feedback ages and bounded motion deadlines
     * remain the authoritative fail-closed checks. */
    if (reached(now_ms, s_next_config_ms)) {
        (void)ZdtStepper_ReadDriverConfig();
        s_next_config_ms = now_ms + LEVER_ACTUATOR_CONFIG_POLL_MS;
        s_next_query_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
        return;
    }
    if (!reached(now_ms, s_next_query_ms)) return;

    if (s_query_phase == 0U) {
        if (ZdtStepper_ReadEncoder() == ZDT_STEPPER_OK) {
            s_query_phase = 1U;
        }
    } else if (s_query_phase == 1U) {
        if (ZdtStepper_ReadPosition() == ZDT_STEPPER_OK) {
            s_query_phase = 2U;
        }
    } else if (ZdtStepper_ReadMotorStatus() == ZDT_STEPPER_OK) {
        s_query_phase = 0U;
    }
    s_next_query_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
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
    if (handle_boot_home(now_ms, &device, encoder_updated)) {
        schedule_feedback_queries(now_ms, &device);
        return;
    }
    if (s_snapshot.ball_active) {
        ball_task(now_ms, &device, encoder_updated);
        schedule_feedback_queries(now_ms, &device);
        return;
    }
    schedule_feedback_queries(now_ms, &device);
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
        /* The finite 0xFD command has already completed.  Do not follow it
         * with 0xFE: the target must remain active to provide holding torque. */
        s_snapshot.moving = false;
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
    s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
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
    s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_QUERY_PERIOD_MS;
    s_move_start_counts = s_snapshot.current_counts;
    s_last_motion_counts = s_snapshot.current_counts;
    s_expected_count_direction = delta > 0 ? 1 : -1;
    s_snapshot.target_angle_mdeg = angle_mdeg;
    s_snapshot.target_counts = target;
    s_snapshot.moving = true;
    return LEVER_ACTUATOR_OK;
}

LeverActuatorResult_t LeverActuator_BallArm(void)
{
    ZdtStepperSnapshot_t device;
    uint32_t now_ms = SystemTime_Now();
    LeverActuator_Task(now_ms);
    ZdtStepper_GetSnapshot(&device);
    if (s_snapshot.fault_latched != 0U)
        return (LeverActuatorResult_t)s_snapshot.fault_latched;
    if (!s_snapshot.boot_homed ||
        (s_snapshot.boot_state !=
         LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL) ||
        s_snapshot.moving)
        return LEVER_ACTUATOR_SESSION_INVALID;
    /* Arm inherits the already-validated horizontal-hold feedback window.
     * Active control switches back to the tighter 100 ms ages after its
     * bounded command-to-feedback handoff has refreshed all three channels. */
    if ((s_snapshot.encoder_age_ms > LEVER_BOOT_POSITION_MAX_AGE_MS) ||
        (s_snapshot.motor_status_age_ms >
         LEVER_BOOT_MOTOR_STATUS_MAX_AGE_MS) ||
        (s_snapshot.motor_position_age_ms > LEVER_BOOT_POSITION_MAX_AGE_MS))
        return LEVER_ACTUATOR_FEEDBACK_STALE;
    if (!device.driver_config_valid || !ball_motor_state_valid(&device))
        return LEVER_ACTUATOR_MOTOR_STATE;
    if (abs_i32(s_snapshot.current_counts - LEVER_HORIZONTAL_COUNTS) >
        LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS)
        return LEVER_ACTUATOR_OUT_OF_RANGE;

    s_snapshot.ball_active = true;
    s_snapshot.ball_returning = false;
    s_snapshot.ball_pending = false;
    s_snapshot.armed = true;
    s_snapshot.requested_tilt_mdeg = 0;
    s_snapshot.pending_tilt_mdeg = 0;
    s_snapshot.target_angle_mdeg = 0;
    s_snapshot.target_counts = LEVER_HORIZONTAL_COUNTS;
    s_ball_move_deadline_ms = now_ms + H3_LEVER_PROGRESS_TIMEOUT_MS;
    s_ball_move_start_counts = s_snapshot.current_counts;
    s_ball_move_start_position = s_snapshot.motor_position_raw;
    s_ball_progress_counts = s_snapshot.current_counts;
    s_ball_expected_direction = 0;
    s_ball_velocity_direction = 0;
    s_ball_braked_direction = 0;
    s_ball_brake_counts = s_snapshot.current_counts;
    s_ball_brake_stationary_samples = 0U;
    s_ball_brake_hold_until_ms = now_ms;
    s_ball_completion_samples = 0U;
    s_ball_return_hold_active = false;
    s_ball_breakaway_fast = false;
    s_ball_withdrawing = false;
    s_ball_velocity_speed_rpm = 0U;
    s_snapshot.ball_velocity_speed_rpm = 0U;
    s_snapshot.ball_velocity_direction = 0;
    s_ball_tracking_rpm = H3_LEVER_VELOCITY_RPM;
    s_ball_breakaway_rpm = H3_LEVER_BREAKAWAY_VELOCITY_RPM;
    s_ball_feedback_handoff_ms = now_ms + H3_LEVER_FEEDBACK_HANDOFF_MS;
    return LEVER_ACTUATOR_OK;
}

LeverActuatorResult_t LeverActuator_BallSetTiltProfile(
    int32_t angle_mdeg, bool breakaway_fast,
    uint16_t tracking_rpm, uint16_t breakaway_rpm)
{
    if (!s_snapshot.ball_active || s_snapshot.ball_returning)
        return LEVER_ACTUATOR_BUSY;
    if ((angle_mdeg < -(int32_t)H3_TUNE_NEGATIVE_MAX_MDEG) ||
        (angle_mdeg > (int32_t)H3_TUNE_POSITIVE_MAX_MDEG) ||
        tracking_rpm < 1U || tracking_rpm > 3U ||
        breakaway_rpm < tracking_rpm || breakaway_rpm > 8U)
        return LEVER_ACTUATOR_OUT_OF_RANGE;
    if (s_snapshot.fault_latched != 0U)
        return (LeverActuatorResult_t)s_snapshot.fault_latched;
    s_ball_breakaway_fast = breakaway_fast;
    s_ball_withdrawing = false;
    s_ball_tracking_rpm = tracking_rpm;
    s_ball_breakaway_rpm = breakaway_rpm;
    s_snapshot.requested_tilt_mdeg = angle_mdeg;
    s_snapshot.pending_tilt_mdeg = angle_mdeg;
    s_snapshot.ball_pending = true;
    return LEVER_ACTUATOR_OK;
}

LeverActuatorResult_t LeverActuator_BallSetTilt(int32_t angle_mdeg)
{
    return LeverActuator_BallSetTiltProfile(angle_mdeg, false,
        H3_LEVER_VELOCITY_RPM, H3_LEVER_BREAKAWAY_VELOCITY_RPM);
}

LeverActuatorResult_t LeverActuator_BallWithdrawHorizontal(void)
{
    if (!s_snapshot.ball_active || s_snapshot.ball_returning)
        return LEVER_ACTUATOR_BUSY;
    if (s_snapshot.fault_latched != 0U)
        return (LeverActuatorResult_t)s_snapshot.fault_latched;
    s_ball_breakaway_fast = false;
    s_ball_withdrawing = true;
    s_snapshot.requested_tilt_mdeg = 0;
    s_snapshot.pending_tilt_mdeg = 0;
    s_snapshot.ball_pending = true;
    return LEVER_ACTUATOR_OK;
}

LeverActuatorResult_t LeverActuator_BallReturnHorizontal(void)
{
    s_ball_return_rpm_override = 0U;
    return LeverActuator_BallReturnHorizontalProfile(0U);
}

LeverActuatorResult_t LeverActuator_BallReturnHorizontalProfile(
    uint16_t return_rpm)
{
    if (return_rpm > 8U)
        return LEVER_ACTUATOR_OUT_OF_RANGE;
    if (!s_snapshot.ball_active)
        return LEVER_ACTUATOR_SESSION_INVALID;
    if (s_snapshot.fault_latched != 0U)
        return (LeverActuatorResult_t)s_snapshot.fault_latched;
    s_snapshot.ball_returning = true;
    s_ball_return_rpm_override = return_rpm;
    s_ball_breakaway_fast = false;
    s_ball_withdrawing = false;
    s_snapshot.requested_tilt_mdeg = 0;
    s_snapshot.pending_tilt_mdeg = 0;
    s_snapshot.ball_pending = true;
    return LEVER_ACTUATOR_OK;
}

void LeverActuator_BallStop(bool disable_after_stop)
{
    if (disable_after_stop) {
        (void)ZdtStepper_EmergencyDisable();
        s_uart_quiet_until_ms = s_snapshot.time_ms +
            LEVER_ACTUATOR_EMERGENCY_QUIET_MS;
        s_snapshot.boot_homed = false;
        s_snapshot.boot_state = LEVER_ACTUATOR_BOOT_DISABLED;
        s_snapshot.armed = false;
    }
    s_snapshot.moving = false;
    s_snapshot.ball_active = false;
    s_snapshot.ball_returning = false;
    s_snapshot.ball_pending = false;
    s_snapshot.requested_tilt_mdeg = 0;
    s_snapshot.pending_tilt_mdeg = 0;
    s_ball_expected_direction = 0;
    s_ball_velocity_direction = 0;
    s_ball_braked_direction = 0;
    s_ball_brake_stationary_samples = 0U;
    s_ball_completion_samples = 0U;
    s_ball_return_hold_active = false;
    s_ball_breakaway_fast = false;
    s_ball_withdrawing = false;
    s_ball_velocity_speed_rpm = 0U;
    s_ball_return_rpm_override = 0U;
    s_snapshot.ball_velocity_speed_rpm = 0U;
    s_snapshot.ball_velocity_direction = 0;
}

void LeverActuator_RequestStop(void)
{
    uint32_t now_ms = SystemTime_Now();
    (void)ZdtStepper_EmergencyDisable();
    s_uart_quiet_until_ms = now_ms + LEVER_ACTUATOR_EMERGENCY_QUIET_MS;
    s_snapshot.moving = false;
    s_snapshot.armed = false;
    s_snapshot.ball_active = false;
    s_snapshot.ball_returning = false;
    s_snapshot.ball_pending = false;
    s_ball_velocity_direction = 0;
    s_ball_expected_direction = 0;
    s_ball_braked_direction = 0;
    s_ball_brake_stationary_samples = 0U;
    s_ball_return_hold_active = false;
    s_ball_breakaway_fast = false;
    s_ball_withdrawing = false;
    s_ball_velocity_speed_rpm = 0U;
    s_ball_return_rpm_override = 0U;
    s_snapshot.ball_velocity_speed_rpm = 0U;
    s_snapshot.ball_velocity_direction = 0;
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
