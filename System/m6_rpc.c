#include "m6_rpc.h"

#include "app_manual_drive.h"
#include "app_remote.h"
#include "app_line.h"
#include "app_h2_panel.h"
#include "control_params.h"
#include "diagnostics.h"
#include "grayscale.h"
#include "grayscale_calibration.h"
#include "gyro.h"
#include "heading.h"
#include "lever_actuator.h"
#include "lever_calibration.h"
#include "motor.h"
#include "m6_action_validate.h"
#include "m6_telemetry.h"
#include "m6_vision_uart.h"
#include "parameter_store.h"
#include "reset_diag.h"
#include "safety.h"
#include "scheduler.h"
#include "system_state.h"
#include "system_time.h"
#include "uart_debug.h"
#include "wheel_speed.h"
#include "zdt_stepper_test.h"

#include <math.h>
#include <string.h>

#define M6_ACTION_SAFETY_GRACE_MS 250U

_Static_assert(ZDT_STEPPER_NOT_READY_NO_STATUS ==
               M6_ZDT_NOT_READY_NO_MOTOR_STATUS, "ZDT ready detail mismatch");
_Static_assert(ZDT_STEPPER_NOT_READY_STATUS_STALE ==
               M6_ZDT_NOT_READY_MOTOR_STATUS_STALE, "ZDT ready detail mismatch");
_Static_assert(ZDT_STEPPER_NOT_READY_DISABLED ==
               M6_ZDT_NOT_READY_MOTOR_DISABLED, "ZDT ready detail mismatch");
_Static_assert(ZDT_STEPPER_NOT_READY_INTERNAL ==
               M6_ZDT_NOT_READY_INTERNAL_REJECT, "ZDT ready detail mismatch");

typedef struct {
    uint8_t state;
    uint8_t kind;
    uint8_t wheel_mask;
    uint32_t token;
    uint32_t expires_ms;
    uint32_t action_id;
    uint32_t start_ms;
    uint32_t deadline_ms;
    int16_t left;
    int16_t right;
    int32_t angle_mdeg;
    int32_t yaw_rate_mdeg_s;
    uint32_t duration_ms;
    uint32_t armed_revision;
    uint32_t armed_session;
    uint8_t parameters[24];
} M6Action_t;

typedef struct {
    bool enabled;
    uint16_t period_ms;
    uint32_t due_ms;
    uint32_t sample_id;
} M6Subscription_t;

static ControlProtocolCommitCallback_t s_commit_callback;
static M6Action_t s_action;
static M6Subscription_t s_subscriptions[4];
static uint32_t s_next_sample_id[4];
static uint32_t s_next_action_id = 1U;
static uint32_t s_boot_id;
static uint32_t s_event_session;
static uint32_t s_last_emergency_epoch;
static uint32_t s_last_fault_bits;
static uint32_t s_last_manual_transition;
static uint32_t s_pending_apply_sequence;
static bool fill_telemetry_payload(uint8_t stream, uint32_t now_ms,
                                   uint8_t *payload,
                                   uint16_t *payload_length);
static const uint8_t k_param_units[35] = {
    1,1,1,1,2,4,4,5,6,7,6,7,3,1,1,1,1,2,3,3,8,3,3,2,2,1,1,3,4,2,3,3,3,3,9
};
static const uint8_t k_param_relations[35] = {
    0,0,0,0,0,0,0,0,1,1,2,2,0,0,0,0,0,0,0,0,0,3,3,0,0,0,0,0,0,0,0,4,4,4,0
};

static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int16_t i16(const uint8_t *p) { return (int16_t)u16(p); }
static int32_t i32(const uint8_t *p) { return (int32_t)u32(p); }

static void p16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

static void p32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void p64(uint8_t *p, uint64_t v)
{
    for (uint8_t i = 0U; i < 8U; i++) p[i] = (uint8_t)(v >> (8U * i));
}

static int16_t round_i16(float value);
static int32_t round_i32(float value);
static uint16_t round_u16(float value);

static void fill_zdt_stepper(uint8_t *out)
{
    ZdtStepperTestSnapshot_t s;
    ZdtStepperTest_GetSnapshot(&s, SystemTime_Now());
    memset(out, 0, M6_ZDT_STEPPER_SNAPSHOT_SIZE);
    p32(&out[0], s.time_ms); p32(&out[4], s.device.response_sequence);
    p32(&out[8], s.response_age_ms); out[12] = s.device.last_function;
    out[13] = (uint8_t)s.device.last_response; p16(&out[14], s.valid_flags);
    p16(&out[16], (uint16_t)s.speed_rpm); out[18] = s.device.motor_status;
    out[19] = s.device.homing_status; p64(&out[20], (uint64_t)s.position_raw);
    p32(&out[28], s.device.stats.tx_bytes); p32(&out[32], s.device.stats.rx_bytes);
    p32(&out[36], s.device.stats.tx_overflow); p32(&out[40], s.device.stats.rx_overflow);
    p32(&out[44], s.device.stats.hardware_errors); p32(&out[48], s.device.stats.invalid_frames);
    p32(&out[52], s.device.stats.checksum_errors); p32(&out[56], s.device.stats.discarded_rx_bytes);
    p32(&out[60], s.device.stats.preempted_frames); p32(&out[64], s.command_id);
    p32(&out[68], s.test_id); p32(&out[72], s.remaining_ms);
    out[76] = (uint8_t)s.state; out[77] = (uint8_t)s.kind;
    out[78] = (uint8_t)s.direction; out[79] = s.acceleration;
    p16(&out[80], s.requested_speed_rpm); out[82] = (uint8_t)s.position_mode;
    out[83] = (uint8_t)s.stop_reason; p32(&out[84], s.pulses);
}

static void fill_h_mission(uint8_t *out)
{
    HMissionSnapshot_t s;
    HMission_GetSnapshot(&s);
    memset(out, 0, M6_H_MISSION_SNAPSHOT_SIZE);
    p32(&out[0], SystemTime_Now());
    p32(&out[4], s.active_revision);
    p32(&out[8], s.stage_revision);
    out[12] = (uint8_t)s.selected_mode;
    out[13] = (uint8_t)s.running_mode;
    out[14] = (uint8_t)s.state;
    out[15] = (uint8_t)s.stop_reason;
    out[16] = (s.dirty ? 1U : 0U) |
        (HMission_IsActive() ? 2U : 0U);
    p16(&out[18], s.white_ms);
    p16(&out[20], s.outer_ms);
    p16(&out[22], s.saturation_ms);
    p16(&out[24], round_u16(s.active.h4_straight_speed_mm_s));
    p16(&out[26], round_u16(s.active.h4_b_margin_mm));
    p16(&out[28], round_u16(s.active.h5_straight_speed_mm_s));
    p16(&out[30], round_u16(s.active.h5_curve_speed_mm_s));
    p16(&out[32], round_u16(s.active.development_end_distance_mm));
    p16(&out[34], round_u16(s.staged.h4_straight_speed_mm_s));
    p16(&out[36], round_u16(s.staged.h4_b_margin_mm));
    p16(&out[38], round_u16(s.staged.h5_straight_speed_mm_s));
    p16(&out[40], round_u16(s.staged.h5_curve_speed_mm_s));
    p16(&out[42], round_u16(s.staged.development_end_distance_mm));
    p32(&out[44], (uint32_t)round_i32(s.distance_mm * 10.0f));
    p32(&out[48], s.started_ms);
    p32(&out[52], s.latched_ms);
    p32(&out[56], s.elapsed_ms);
    p16(&out[60], (uint16_t)round_i16(s.final_left_target_mm_s));
    p16(&out[62], (uint16_t)round_i16(s.final_right_target_mm_s));
}

static int16_t round_i16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)((value >= 0.0f) ? value + 0.5f : value - 0.5f);
}

static int32_t round_i32(float value)
{
    double scaled = value;
    if (scaled >= 2147483647.0) return INT32_MAX;
    if (scaled <= -2147483648.0) return INT32_MIN;
    return (int32_t)((scaled >= 0.0) ? scaled + 0.5 : scaled - 0.5);
}

static uint16_t round_u16(float value)
{
    if (value <= 0.0f) return 0U;
    if (value >= 65535.0f) return 65535U;
    return (uint16_t)(value + 0.5f);
}

static uint16_t saturate_u16(uint32_t value)
{
    return value > 65535U ? 65535U : (uint16_t)value;
}

static uint8_t source_wire(ControlParamsSource_t source)
{
    if (source == CONTROL_PARAMS_SOURCE_PERSISTED) return 1U;
    if (source == CONTROL_PARAMS_SOURCE_RUNTIME) return 2U;
    return 0U;
}

static M6Result_t params_result(ControlParamsResult_t result)
{
    switch (result) {
    case CONTROL_PARAMS_OK: return M6_RESULT_OK;
    case CONTROL_PARAMS_ERR_KEY: return M6_RESULT_UNKNOWN_PARAMETER;
    case CONTROL_PARAMS_ERR_READ_ONLY: return M6_RESULT_READ_ONLY;
    case CONTROL_PARAMS_ERR_VALUE: return M6_RESULT_INVALID_VALUE;
    case CONTROL_PARAMS_ERR_RANGE: return M6_RESULT_OUT_OF_RANGE;
    case CONTROL_PARAMS_ERR_RELATION: return M6_RESULT_RELATION;
    case CONTROL_PARAMS_ERR_STATE: return M6_RESULT_WRONG_STATE;
    case CONTROL_PARAMS_ERR_BUSY: return M6_RESULT_BUSY;
    default: return M6_RESULT_INVALID_ARGUMENT;
    }
}

static bool remote_idle(void)
{
    AppRemoteSnapshot_t remote;
    AppRemote_GetSnapshot(&remote);
    return (remote.state == APP_REMOTE_IDLE) && AppManualDrive_IsIdle();
}

static void fill_manual_drive(uint8_t *out)
{
    AppManualDriveSnapshot_t snapshot;
    AppManualDrive_GetSnapshot(&snapshot);
    memset(out, 0, 16U);
    p32(&out[0], snapshot.command_id);
    p32(&out[4], snapshot.lease_deadline_ms);
    p16(&out[8], (uint16_t)snapshot.left_mm_s);
    p16(&out[10], (uint16_t)snapshot.right_mm_s);
    out[12] = (uint8_t)snapshot.source;
    out[13] = snapshot.active ? 1U : 0U;
    out[14] = (uint8_t)snapshot.stop_reason;
}

static bool action_base_safe(void)
{
    return (SystemState_Get() == SYSTEM_STATE_IDLE) && !Safety_HasFault() &&
        Diagnostics_ActionIsIdle() && remote_idle() && (s_action.state == 0U);
}

static bool save_safe(void)
{
    WheelSpeedSnapshot_t wheel;
    SafetySnapshot_t safety;
    MotorStats_t motor;
    if (!action_base_safe()) return false;
    WheelSpeed_GetSnapshot(&wheel);
    Safety_GetSnapshot(&safety);
    Motor_GetStats(&motor);
    return !wheel.enabled && !safety.action_active &&
        (safety.requested_motor_a == 0) && (safety.requested_motor_b == 0) &&
        (safety.applied_motor_a == 0) && (safety.applied_motor_b == 0) &&
        (motor.last_a == 0) && (motor.last_b == 0);
}

static void fill_reset(uint8_t *out)
{
    ResetDiagSnapshot_t r;
    uint8_t flags = 0U;
    ResetDiag_GetSnapshot(&r);
    memset(out, 0, 76U);
    if (r.previous_coarse_valid) flags |= 1U;
    if (r.previous_detail_valid) flags |= 2U;
    if (r.previous_hardfault) flags |= 4U;
    p32(&out[0], r.raw_cause); out[4] = (uint8_t)r.reset_class;
    out[5] = (uint8_t)r.checkpoint; out[6] = flags;
    out[7] = (uint8_t)r.previous_checkpoint;
    p32(&out[8], r.boot_id); p32(&out[12], r.previous_sequence);
    p32(&out[16], r.previous_boot_id); p32(&out[20], r.previous_time_ms);
    p32(&out[24], r.previous_last_watchdog_service_ms);
    p32(&out[28], r.previous_scheduler_missed_total);
    p32(&out[32], r.previous_safety_fault_bits);
    p32(&out[36], r.previous_uart_tx_dropped);
    p32(&out[40], r.previous_uart_rx_dropped);
    p32(&out[44], r.previous_telemetry_dropped);
    p32(&out[48], r.previous_scheduler_pending_mask);
    p32(&out[52], r.previous_scheduler_missed_mask);
    p32(&out[56], r.previous_hardfault_pc);
    p32(&out[60], r.previous_hardfault_lr);
    p32(&out[64], r.previous_hardfault_xpsr);
    out[68] = r.previous_boot_id_low;
    out[69] = r.previous_detail_checkpoint;
    out[70] = r.previous_system_state;
    out[71] = r.previous_telemetry_queued;
    out[72] = r.previous_scheduler_health_mask;
}

static bool publish_event(M6TxPriority_t priority, uint16_t code,
                          uint8_t severity, const uint8_t *body,
                          uint8_t body_length, uint32_t now_ms)
{
    uint8_t payload[48];
    if ((body == NULL) || (body_length > 36U)) return false;
    memset(payload, 0, sizeof(payload));
    p16(&payload[0], code);
    payload[2] = severity;
    payload[3] = 1U;
    p32(&payload[4], now_ms);
    p16(&payload[8], 1U);
    payload[10] = body_length;
    memcpy(&payload[12], body, body_length);
    return M6Link_Publish(priority, M6_FRAME_EVENT, code, payload,
                          (uint16_t)(12U + body_length));
}

static void publish_reset_event(uint32_t now_ms)
{
    ResetDiagSnapshot_t reset;
    uint8_t body[32];
    uint8_t flags = 0U;
    ResetDiag_GetSnapshot(&reset);
    memset(body, 0, sizeof(body));
    if (reset.previous_coarse_valid) flags |= 1U;
    if (reset.previous_detail_valid) flags |= 2U;
    if (reset.previous_hardfault) flags |= 4U;
    p32(&body[0], reset.boot_id);
    p32(&body[4], reset.raw_cause);
    p32(&body[8], reset.previous_time_ms);
    p32(&body[12], reset.previous_last_watchdog_service_ms);
    p32(&body[16], reset.previous_safety_fault_bits);
    p32(&body[20], reset.previous_scheduler_missed_total);
    body[24] = (uint8_t)reset.reset_class;
    body[25] = (uint8_t)reset.checkpoint;
    body[26] = flags;
    body[27] = (uint8_t)reset.previous_checkpoint;
    body[28] = reset.previous_boot_id_low;
    body[29] = reset.previous_detail_checkpoint;
    body[30] = reset.previous_system_state;
    body[31] = reset.previous_scheduler_health_mask;
    (void)publish_event(M6_TX_P0, M6_EVENT_RESET, 0U, body,
                        sizeof(body), now_ms);
}

static void publish_fault_event(const SafetySnapshot_t *safety,
                                uint32_t now_ms)
{
    uint8_t body[36];
    memset(body, 0, sizeof(body));
    p32(&body[0], safety->fault_bits);
    p32(&body[4], safety->first_fault_ms);
    p32(&body[8], safety->last_fault_ms);
    p32(&body[12], safety->fault_count);
    p32(&body[16], safety->required_sensor_mask);
    p32(&body[20], safety->action_deadline_ms);
    body[24] = (uint8_t)SystemState_Get();
    body[25] = s_action.state;
    p16(&body[28], (uint16_t)safety->requested_motor_a);
    p16(&body[30], (uint16_t)safety->requested_motor_b);
    p16(&body[32], (uint16_t)safety->applied_motor_a);
    p16(&body[34], (uint16_t)safety->applied_motor_b);
    (void)publish_event(M6_TX_P0, M6_EVENT_SAFETY_FAULT, 2U, body,
                        sizeof(body), now_ms);
}

static void publish_emergency_event(const SafetySnapshot_t *safety,
                                    uint32_t emergency_epoch,
                                    uint32_t now_ms)
{
    uint8_t body[24];
    memset(body, 0, sizeof(body));
    p32(&body[0], emergency_epoch);
    p32(&body[4], safety->fault_bits);
    body[8] = (uint8_t)SystemState_Get();
    body[9] = s_action.state;
    body[10] = debug_get_emergency_source();
    p16(&body[12], (uint16_t)safety->target_speed_left_mm_s);
    p16(&body[14], (uint16_t)safety->target_speed_right_mm_s);
    p16(&body[16], (uint16_t)safety->requested_motor_a);
    p16(&body[18], (uint16_t)safety->requested_motor_b);
    p16(&body[20], (uint16_t)safety->applied_motor_a);
    p16(&body[22], (uint16_t)safety->applied_motor_b);
    (void)publish_event(M6_TX_P0, M6_EVENT_EMERGENCY_STOP, 3U, body,
                        sizeof(body), now_ms);
}

static void fill_grayscale_calibration(uint8_t *out)
{
    GrayscaleCalibrationStatus_t status;
    uint32_t i;
    GrayscaleCalibration_GetStatus(&status);
    memset(out, 0, 84U);
    out[0] = (uint8_t)status.state;
    out[1] = (uint8_t)status.owner;
    out[2] = status.active_source;
    out[3] = (uint8_t)status.result;
    p16(&out[4], status.progress);
    p16(&out[6], status.required);
    p32(&out[8], status.generation);
    out[12] = status.valid_mask;
    out[13] = status.polarity_fail_mask;
    out[14] = status.span_fail_mask;
    out[15] = status.noise_fail_mask;
    out[16] = status.rail_fail_mask;
    out[17] = status.failed_stage;
    out[18] = status.scan_error ? 1U : 0U;
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
        p16(&out[20U + i * 2U], status.candidate.white_adc[i]);
        p16(&out[36U + i * 2U], status.candidate.black_adc[i]);
        p16(&out[52U + i * 2U], status.span[i]);
        p16(&out[68U + i * 2U], status.max_noise[i]);
    }
}

static M6Result_t grayscale_calibration_result(
    GrayscaleCalibrationResult_t result, uint32_t *detail)
{
    ParameterStoreInfo_t store;
    if (result == GRAYSCALE_CAL_RESULT_OK) return M6_RESULT_OK;
    if (result == GRAYSCALE_CAL_RESULT_BUSY) return M6_RESULT_BUSY;
    if (result == GRAYSCALE_CAL_RESULT_STALE_GENERATION) {
        ParameterStore_GetInfo(&store);
        *detail = store.generation;
        return M6_RESULT_STALE_GENERATION;
    }
    if (result == GRAYSCALE_CAL_RESULT_STORAGE)
        return M6_RESULT_STORAGE_IO;
    if (result == GRAYSCALE_CAL_RESULT_REJECTED)
        return M6_RESULT_INVALID_VALUE;
    if (result == GRAYSCALE_CAL_RESULT_WRONG_OWNER ||
        result == GRAYSCALE_CAL_RESULT_WRONG_STATE)
        return M6_RESULT_WRONG_STATE;
    return M6_RESULT_SENSOR_NOT_READY;
}

static void fill_lever_calibration(uint8_t *out, uint8_t page)
{
    LeverCalibrationStatus_t status;
    LeverActuatorSnapshot_t actuator;
    const int32_t *table = NULL;
    uint8_t flags = 0U;
    uint32_t i;
    LeverCalibration_GetStatus(&status);
    LeverActuator_GetSnapshot(&actuator);
    memset(out, 0, M6_LEVER_CAL_SNAPSHOT_SIZE);
    if (status.capture_pending) flags |= 1U << 0;
    if (status.persisted_valid) flags |= 1U << 1;
    if (status.candidate_valid) flags |= 1U << 2;
    if (status.session_zero_valid) flags |= 1U << 3;
    if (status.rearm_only) flags |= 1U << 4;
    if (actuator.armed) flags |= 1U << 5;
    if (actuator.moving) flags |= 1U << 6;
    if (actuator.calibration_valid) flags |= 1U << 7;
    p32(&out[0], SystemTime_Now());
    p32(&out[4], s_boot_id);
    p32(&out[8], status.generation);
    p32(&out[12], status.expected_generation);
    p32(&out[16], status.candidate_crc32);
    p32(&out[20], status.candidate.driver_config_fingerprint);
    p32(&out[24], status.reject_detail);
    out[28] = (uint8_t)status.state;
    out[29] = (uint8_t)status.result;
    out[30] = flags;
    out[31] = page;
    if (page == 0U) {
        p16(&out[32], status.progress);
        p16(&out[34], status.required);
        out[36] = status.expected_point;
        out[37] = status.expected_direction;
        out[38] = status.expected_repeat;
        out[39] = status.sample_count;
        p16(&out[40], status.candidate.reference_encoder);
        p16(&out[42], status.candidate.reference_spread_counts);
        p16(&out[44], status.candidate.point_spread_counts);
        p16(&out[46], status.candidate.repeatability_mdeg);
        p16(&out[48], status.candidate.backlash_mdeg);
        p32(&out[52], (uint32_t)status.candidate.horizontal_offset_counts);
        p32(&out[56], (uint32_t)status.candidate.soft_min_angle_mdeg);
        p32(&out[60], (uint32_t)status.candidate.soft_max_angle_mdeg);
        p32(&out[64], (uint32_t)status.candidate.soft_min_counts);
        p32(&out[68], (uint32_t)status.candidate.soft_max_counts);
        p32(&out[72], (uint32_t)actuator.current_counts);
        p32(&out[76], (uint32_t)actuator.target_counts);
        p32(&out[80], actuator.fault_latched);
        p32(&out[84], (uint32_t)status.last_capture_counts);
    } else {
        if (page == 1U) table = status.candidate.angle_mdeg;
        else if (page == 2U) table = status.candidate.increasing_counts;
        else table = status.candidate.decreasing_counts;
        for (i = 0U; i < LEVER_CALIBRATION_POINT_COUNT; i++)
            p32(&out[32U + i * 4U], (uint32_t)table[i]);
    }
}

static M6Result_t lever_calibration_result(
    LeverCalibrationResult_t result, uint32_t *detail)
{
    ParameterStoreInfo_t store;
    if (result == LEVER_CAL_RESULT_OK) return M6_RESULT_OK;
    if (result == LEVER_CAL_RESULT_BUSY) return M6_RESULT_BUSY;
    if (result == LEVER_CAL_RESULT_STALE_GENERATION) {
        ParameterStore_GetInfo(&store);
        *detail = store.generation;
        return M6_RESULT_STALE_GENERATION;
    }
    if (result == LEVER_CAL_RESULT_STORAGE) return M6_RESULT_STORAGE_IO;
    if ((result == LEVER_CAL_RESULT_WRONG_OWNER) ||
        (result == LEVER_CAL_RESULT_WRONG_STATE)) return M6_RESULT_WRONG_STATE;
    if (result == LEVER_CAL_RESULT_INVALID_REQUEST)
        return M6_RESULT_INVALID_ARGUMENT;
    if ((result == LEVER_CAL_RESULT_NON_MONOTONIC) ||
        (result == LEVER_CAL_RESULT_REPEATABILITY) ||
        (result == LEVER_CAL_RESULT_BACKLASH) ||
        (result == LEVER_CAL_RESULT_SLOPE)) return M6_RESULT_INVALID_VALUE;
    return M6_RESULT_SENSOR_NOT_READY;
}

static void publish_manual_drive_event(uint32_t now_ms)
{
    uint8_t body[16];
    fill_manual_drive(body);
    (void)publish_event(M6_TX_P1, M6_EVENT_MANUAL_DRIVE_STATE, 0U,
                        body, sizeof(body), now_ms);
}

static void stop_action(uint32_t now_ms)
{
    uint8_t previous_state;
    if ((s_action.state != 1U) && (s_action.state != 2U)) return;
    previous_state = s_action.state;
    if ((previous_state == 2U) &&
        (s_action.kind == M6_ACTION_KIND_LINE_FOLLOW)) {
        (void)AppLine_Stop(now_ms);
    } else {
        WheelSpeed_Disable();
        Heading_Stop();
        Safety_EndAction();
        (void)SystemState_RequestIdle();
    }
    s_action.state = 0U;
    s_action.start_ms = now_ms;
}

void M6Rpc_Init(ControlProtocolCommitCallback_t commit_callback)
{
    ResetDiagSnapshot_t reset;
    SafetySnapshot_t safety;
    memset(&s_action, 0, sizeof(s_action));
    memset(s_subscriptions, 0, sizeof(s_subscriptions));
    for (uint8_t i = 0U; i < 4U; i++) s_next_sample_id[i] = 1U;
    s_commit_callback = commit_callback;
    ResetDiag_GetSnapshot(&reset);
    s_boot_id = reset.boot_id;
    Safety_GetSnapshot(&safety);
    s_event_session = 0U;
    s_last_emergency_epoch = debug_get_emergency_epoch();
    s_last_fault_bits = safety.fault_bits;
    {
        AppManualDriveSnapshot_t manual;
        AppManualDrive_GetSnapshot(&manual);
        s_last_manual_transition = manual.transition_id;
    }
    s_pending_apply_sequence = 0U;
    M6VisionUart_Init();
}

void M6Rpc_ParameterBoundaryCompleted(ControlParamsResult_t result,
                                      const ControlParamsCommit_t *commit)
{
    uint8_t out[20];
    M6Result_t wire_result;
    if (s_pending_apply_sequence == 0U) return;
    if ((result == CONTROL_PARAMS_OK) && (commit != NULL)) {
        memset(out, 0, sizeof(out));
        p32(&out[0], commit->revision);
        p32(&out[4], ControlParams_StageRevision());
        p64(&out[8], commit->change_mask);
        out[16] = source_wire(commit->source);
        out[17] = commit->diagnostic_boundary;
        (void)M6Link_CompletePending(s_pending_apply_sequence,
            M6_RESULT_OK, 0U, out, sizeof(out));
    } else {
        wire_result = params_result(result);
        (void)M6Link_CompletePending(s_pending_apply_sequence,
            wire_result, 0U, NULL, 0U);
    }
    s_pending_apply_sequence = 0U;
}

void M6Rpc_CancelPendingApply(M6Result_t result, uint32_t detail)
{
    if (s_pending_apply_sequence == 0U) return;
    if ((result != M6_RESULT_SAFETY_FAULT) &&
        (result != M6_RESULT_WRONG_STATE)) result = M6_RESULT_WRONG_STATE;
    (void)M6Link_CompletePending(s_pending_apply_sequence, result, detail,
                                NULL, 0U);
    s_pending_apply_sequence = 0U;
}

static M6Result_t execute_read(uint16_t opcode, const uint8_t *body,
                               uint16_t length, uint8_t *out,
                               uint16_t *out_length, uint32_t *detail)
{
    (void)detail;
    if (opcode == M6_RPC_H_MISSION_STATUS_GET) {
        if ((M6Link_EnabledCapabilities() & M6_CAP_H_MISSION_CONTROL) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != 0U) return M6_RESULT_BAD_LENGTH;
        fill_h_mission(out);
        *out_length = M6_H_MISSION_SNAPSHOT_SIZE;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_ZDT_STEPPER_STATUS_GET) {
        if ((M6Link_EnabledCapabilities() & M6_CAP_ZDT_STEPPER_CONTROL) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != M6_ZDT_STEPPER_STATUS_REQUEST_SIZE)
            return M6_RESULT_BAD_LENGTH;
        if ((body[1] | body[2] | body[3]) != 0U) return M6_RESULT_MALFORMED;
        if ((body[0] & (uint8_t)~ZDT_STEPPER_REFRESH_ALL) != 0U)
            return M6_RESULT_INVALID_ARGUMENT;
        ZdtStepperTest_RequestRefresh(body[0]);
        fill_zdt_stepper(out); *out_length = M6_ZDT_STEPPER_SNAPSHOT_SIZE;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_LEVER_CALIBRATION_STATUS_GET) {
        if ((M6Link_EnabledCapabilities() &
             M6_CAP_LEVER_CALIBRATION) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != M6_LEVER_CAL_STATUS_REQUEST_SIZE)
            return M6_RESULT_BAD_LENGTH;
        if ((body[1] | body[2] | body[3]) != 0U)
            return M6_RESULT_MALFORMED;
        if (body[0] > M6_LEVER_CAL_STATUS_PAGE_MAX)
            return M6_RESULT_INVALID_ARGUMENT;
        fill_lever_calibration(out, body[0]);
        *out_length = M6_LEVER_CAL_SNAPSHOT_SIZE;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PING) {
        if (length != 4U) return M6_RESULT_BAD_LENGTH;
        memcpy(out, body, 4U); p32(&out[4], SystemTime_Now());
        p32(&out[8], s_boot_id); *out_length = 12U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_SNAPSHOT_GET) {
        uint8_t telemetry[72];
        uint16_t telemetry_length;
        uint8_t stream;
        if (length != 4U) return M6_RESULT_BAD_LENGTH;
        if ((body[1] | body[2] | body[3]) != 0U)
            return M6_RESULT_MALFORMED;
        stream = body[0];
        if ((stream < M6_STREAM_CORE) || (stream > M6_STREAM_HEALTH))
            return M6_RESULT_INVALID_ARGUMENT;
        if (!fill_telemetry_payload((uint8_t)(stream - 1U),
                                    SystemTime_Now(), telemetry,
                                    &telemetry_length))
            return M6_RESULT_INTERNAL;
        memset(out, 0, 68U);
        out[0] = stream;
        out[1] = 1U;
        p16(&out[2], (uint16_t)(telemetry_length - 8U));
        memcpy(&out[4], &telemetry[8], telemetry_length - 8U);
        *out_length = (uint16_t)(telemetry_length - 4U);
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_MANUAL_DRIVE_GET) {
        if (length != 0U) return M6_RESULT_BAD_LENGTH;
        if ((M6Link_EnabledCapabilities() & M6_CAP_MANUAL_DRIVE) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        fill_manual_drive(out);
        *out_length = 16U;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_VISION_UART_MONITOR_GET) {
        if (length != 0U) return M6_RESULT_BAD_LENGTH;
        if ((M6Link_EnabledCapabilities() &
             M6_CAP_VISION_UART_MONITOR) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        M6VisionUart_GetStatus(out, out_length);
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_GRAYSCALE_CALIBRATION_GET) {
        if (length != 0U) return M6_RESULT_BAD_LENGTH;
        if ((M6Link_EnabledCapabilities() &
             M6_CAP_GRAYSCALE_CALIBRATION) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        fill_grayscale_calibration(out);
        *out_length = 84U;
        return M6_RESULT_OK;
    }
    if (length != 0U && opcode != 0x0101U && opcode != 0x0102U)
        return M6_RESULT_BAD_LENGTH;
    if (opcode == M6_RPC_IDENTITY_GET) {
        ParameterStoreInfo_t store;
        static const char firmware[] = CONTROL_PROTOCOL_FIRMWARE_ID;
        uint8_t firmware_length = (uint8_t)(sizeof(firmware) - 1U);
        ParameterStore_GetInfo(&store);
        memset(out, 0, 63U); out[0] = M6_WIRE_VERSION;
        out[1] = source_wire(ControlParams_Source());
        p16(&out[2], M6_REGISTRY_REVISION);
        p16(&out[4], CONTROL_PARAMS_SCHEMA_VERSION);
#ifdef M2_FAULT_INJECTION_TEST
        out[6] = 3U;
#endif
        out[7] = firmware_length; p32(&out[8], ControlParams_Revision());
        p32(&out[12], ControlParams_StageRevision());
        p32(&out[16], store.generation); p32(&out[20], s_boot_id);
        p64(&out[24], M6Link_Capabilities());
        memcpy(&out[32], firmware, firmware_length);
        *out_length = (uint16_t)(32U + firmware_length); return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_STATUS_GET) {
        SafetySnapshot_t safety; WheelSpeedSnapshot_t wheel;
        HeadingSnapshot_t heading; uint8_t flags = 0U;
        Safety_GetSnapshot(&safety); WheelSpeed_GetSnapshot(&wheel);
        Heading_GetSnapshot(&heading); memset(out, 0, 60U);
        p32(&out[0], SystemTime_Now()); p32(&out[4], safety.fault_bits);
        p32(&out[8], safety.first_fault_ms); p32(&out[12], safety.last_fault_ms);
        p32(&out[16], safety.fault_count); p32(&out[20], safety.required_sensor_mask);
        p32(&out[24], safety.action_deadline_ms);
        p32(&out[28], ControlParams_Revision());
        p32(&out[32], ControlParams_StageRevision());
        out[36] = (uint8_t)SystemState_Get(); out[37] = s_action.state;
        out[38] = s_action.kind;
        if (wheel.enabled) flags |= 1U;
        if (ControlParams_IsDirty()) flags |= 2U;
        if (safety.action_active) flags |= 4U;
        if (heading.valid) flags |= 8U;
        if (heading.calibrated) flags |= 16U;
        if (heading.calibrating) flags |= 32U;
        if (Grayscale_IsScanPending()) flags |= 64U;
        out[39] = flags;
        p16(&out[40], (uint16_t)safety.requested_motor_a);
        p16(&out[42], (uint16_t)safety.requested_motor_b);
        p16(&out[44], (uint16_t)safety.applied_motor_a);
        p16(&out[46], (uint16_t)safety.applied_motor_b);
        p16(&out[48], (uint16_t)safety.target_speed_left_mm_s);
        p16(&out[50], (uint16_t)safety.target_speed_right_mm_s);
        p16(&out[52], (uint16_t)safety.measured_speed_left_mm_s);
        p16(&out[54], (uint16_t)safety.measured_speed_right_mm_s);
        out[56] = source_wire(ControlParams_Source());
        *out_length = 60U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_RESET_GET) { fill_reset(out); *out_length = 76U; return M6_RESULT_OK; }
    if (opcode == M6_RPC_TASK_STATS_GET) {
        SchedulerStats_t s; Scheduler_GetStats(&s); memset(out, 0, 64U);
        out[0] = SCHEDULER_TASK_COUNT;
        out[1] = (s.missed_mask == 0U) &&
            ((s.health_window_mask & SCHEDULER_CRITICAL_TASKS_MASK) ==
             SCHEDULER_CRITICAL_TASKS_MASK);
        p32(&out[4], s.pending_mask); p32(&out[8], s.missed_mask);
        p32(&out[12], s.health_window_mask);
        for (uint8_t i = 0U; i < SCHEDULER_TASK_COUNT; i++) {
            p32(&out[16U + i * 4U], s.release_count[i]);
            p32(&out[32U + i * 4U], s.completion_count[i]);
            p32(&out[48U + i * 4U], s.miss_count[i]);
        }
        *out_length = 64U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_UART_STATS_GET) {
        DebugStats_t s; debug_get_stats(&s); memset(out, 0, 32U);
        p32(&out[0], s.tx_bytes); p32(&out[4], s.rx_bytes);
        p32(&out[8], s.format_errors); p32(&out[12], s.tx_dropped);
        p32(&out[16], s.rx_dropped); p32(&out[20], debug_get_emergency_epoch());
        p16(&out[24], debug_get_rx_queued()); p16(&out[26], debug_get_rx_high_water());
        p16(&out[28], debug_get_tx_queued()); p16(&out[30], debug_get_tx_high_water());
        *out_length = 32U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_LINK_STATS_GET) {
        M6LinkStats_t s; M6Link_GetStats(&s); memset(out, 0, 76U);
        out[0] = (uint8_t)s.mode; out[1] = 1U; out[2] = s.replay_used;
        p32(&out[4], s.session); p32(&out[8], s.next_host_sequence);
        p32(&out[12], s.next_async_sequence); p32(&out[16], s.rx_frames);
        p32(&out[20], s.tx_frames); p32(&out[24], s.parser.crc_error);
        p32(&out[28], s.frame_errors); p32(&out[32],
            s.parser.interbyte_timeout + s.parser.total_timeout);
        p32(&out[36], s.session_mismatch); p32(&out[40], s.sequence_errors);
        p32(&out[44], s.replay_count); p32(&out[48], s.busy_count);
        p32(&out[52], s.p0_drop); p32(&out[56], s.p1_drop);
        p32(&out[60], s.p2_drop); p32(&out[64], s.p3_drop);
        p32(&out[68], s.control_error_drop); out[72] = s.p0_high_water;
        out[73] = s.p1_high_water; out[74] = s.p2_high_water;
        out[75] = s.p3_high_water; *out_length = 76U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_ACTION_GET) {
        memset(out, 0, 28U); p32(&out[0], s_action.action_id);
        out[4] = s_action.state; out[5] = s_action.kind; out[7] = 2U;
        p32(&out[8], s_action.start_ms); p32(&out[12], s_action.deadline_ms);
        p16(&out[16], (uint16_t)s_action.left); p16(&out[18], (uint16_t)s_action.right);
        p32(&out[20], (uint32_t)s_action.angle_mdeg);
        *out_length = 28U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_SUBSCRIPTION_GET) {
        uint8_t count = 0U; uint32_t estimated = 0U;
        static const uint8_t payload_size[4] = {40U,60U,56U,72U};
        memset(out, 0, 24U);
        for (uint8_t i = 0U; i < 4U; i++) if (s_subscriptions[i].enabled) {
            out[8U + count * 4U] = i + 1U;
            p16(&out[10U + count * 4U], s_subscriptions[i].period_ms);
            estimated += ((1000U + s_subscriptions[i].period_ms - 1U) /
                s_subscriptions[i].period_ms) * (17U + payload_size[i]);
            count++;
        }
        out[0] = count; p32(&out[4], estimated);
        *out_length = (uint16_t)(8U + count * 4U); return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_SUMMARY_GET) {
        ParameterStoreInfo_t store; uint8_t flags = 0U;
        ParameterStore_GetInfo(&store); memset(out, 0, 24U);
        p16(&out[0], CONTROL_PARAMS_SCHEMA_VERSION);
        out[2] = (uint8_t)ControlParams_DescriptorCount();
        out[3] = source_wire(ControlParams_Source());
        p32(&out[4], ControlParams_Revision()); p32(&out[8], ControlParams_StageRevision());
        p32(&out[12], store.generation); if (ControlParams_IsDirty()) flags |= 1U;
        if (ControlParams_HasPendingApply()) flags |= 2U;
        if (store.slot_a_valid) flags |= 4U;
        if (store.slot_b_valid) flags |= 8U;
        out[16] = flags; out[17] = (uint8_t)store.selected_slot;
        out[18] = (uint8_t)store.health; p32(&out[20], store.save_failures);
        *out_length = 24U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_VALUE_GET) {
        uint8_t id; const ControlParamDescriptor_t *d; ControlParamReadback_t r;
        ParameterStoreInfo_t store; uint32_t active_bits, staged_bits;
        if (length != 4U) return M6_RESULT_BAD_LENGTH;
        if ((body[1] | body[2] | body[3]) != 0U)
            return M6_RESULT_MALFORMED;
        id = body[0]; d = id ? ControlParams_DescriptorAt(id - 1U) : NULL;
        if ((d == NULL) || (ControlParams_Get(d->key, &r) != CONTROL_PARAMS_OK))
            return M6_RESULT_UNKNOWN_PARAMETER;
        if (r.type == CONTROL_PARAM_VALUE_BOOL) {
            active_bits = r.active.unsigned_value; staged_bits = r.staged.unsigned_value;
        } else {
            memcpy(&active_bits, &r.active.real, 4U); memcpy(&staged_bits, &r.staged.real, 4U);
        }
        ParameterStore_GetInfo(&store); memset(out, 0, 28U); out[0] = id;
        out[1] = r.type == CONTROL_PARAM_VALUE_BOOL ? 2U : 1U;
        out[2] = source_wire(ControlParams_Source());
        out[3] = source_wire(ControlParams_StagedSource());
        p32(&out[4], ControlParams_Revision()); p32(&out[8], ControlParams_StageRevision());
        p32(&out[12], store.generation); p32(&out[16], active_bits); p32(&out[20], staged_bits);
        *out_length = 28U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_DESCRIPTOR_GET) {
        uint8_t id, flags = 5U; const ControlParamDescriptor_t *d;
        ControlParamScalar_t def; uint32_t min_bits, max_bits, def_bits;
        size_t key_length;
        if (length != 4U) return M6_RESULT_BAD_LENGTH;
        if ((body[1] | body[2] | body[3]) != 0U)
            return M6_RESULT_MALFORMED;
        id = body[0]; d = id ? ControlParams_DescriptorAt(id - 1U) : NULL;
        if ((d == NULL) || !ControlParams_ReadValueAt(id - 1U,
            ControlParams_Defaults(), &def)) return M6_RESULT_UNKNOWN_PARAMETER;
        if ((id <= 4U) || (id == 13U)) flags |= 2U;
        if (k_param_relations[id - 1U] != 0U) flags |= 8U;
        if (id == 35U) {
            min_bits = 0U; max_bits = 1U; def_bits = def.unsigned_value;
        } else {
            memcpy(&min_bits, &d->minimum, 4U);
            memcpy(&max_bits, &d->maximum, 4U);
            memcpy(&def_bits, &def.real, 4U);
        }
        key_length = strlen(d->key); memset(out, 0, 51U); out[0] = id;
        out[1] = id == 35U ? 2U : 1U; out[2] = flags;
        out[3] = id == 13U ? 2U : (id == 35U ? 1U : 0U);
        out[4] = k_param_relations[id - 1U]; out[5] = k_param_units[id - 1U];
        out[6] = (uint8_t)key_length; p32(&out[8], min_bits);
        p32(&out[12], max_bits); p32(&out[16], def_bits);
        memcpy(&out[20], d->key, key_length);
        *out_length = (uint16_t)(20U + key_length); return M6_RESULT_OK;
    }
    return M6_RESULT_UNSUPPORTED_OPCODE;
}

static M6Result_t execute_mutating(uint16_t opcode, const uint8_t *body,
                                   uint16_t length, uint8_t *out,
                                   uint16_t *out_length, uint32_t *detail)
{
    ParameterStoreInfo_t store;
    ControlParamsResult_t pr;
    *detail = 0U;
    ParameterStore_GetInfo(&store);
    if (opcode == M6_RPC_H_MISSION_COMMAND) {
        HMissionSnapshot_t snapshot;
        HMissionConfig_t config;
        uint8_t command;
        uint16_t flags;
        uint8_t i;
        if ((M6Link_EnabledCapabilities() & M6_CAP_H_MISSION_CONTROL) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != M6_H_MISSION_COMMAND_REQUEST_SIZE)
            return M6_RESULT_BAD_LENGTH;
        if (u32(body) != s_boot_id) return M6_RESULT_INVALID_ARGUMENT;
        for (i = 26U; i < M6_H_MISSION_COMMAND_REQUEST_SIZE; i++) {
            if (body[i] != 0U) return M6_RESULT_MALFORMED;
        }
        command = body[12];
        flags = u16(&body[14]);
        HMission_GetSnapshot(&snapshot);
        if (command == M6_H_MISSION_COMMAND_ABORT) {
            if ((body[13] != 0U) || ((flags & (uint16_t)~1U) != 0U) ||
                ((body[16] | body[17] | body[18] | body[19] |
                  body[20] | body[21] | body[22] | body[23] |
                  body[24] | body[25]) != 0U))
                return M6_RESULT_INVALID_ARGUMENT;
            if (!AppLine_AbortHMission(SystemTime_Now(),
                    (flags & 1U) ? H_MISSION_STOP_TELEMETRY_GAP :
                    H_MISSION_STOP_USER_CANCEL))
                return M6_RESULT_WRONG_STATE;
            fill_h_mission(out);
            *out_length = M6_H_MISSION_SNAPSHOT_SIZE;
            return M6_RESULT_OK;
        }
        if (!action_base_safe() || !HMission_IsConfigMutable())
            return M6_RESULT_WRONG_STATE;
        if (u32(&body[4]) != snapshot.active_revision) {
            *detail = snapshot.active_revision;
            return M6_RESULT_STALE_ACTIVE_REVISION;
        }
        if (u32(&body[8]) != snapshot.stage_revision) {
            *detail = snapshot.stage_revision;
            return M6_RESULT_STALE_STAGE_REVISION;
        }
        if (flags != 0U) return M6_RESULT_INVALID_ARGUMENT;
        if (command == M6_H_MISSION_COMMAND_START) {
            AppH2StartStatus_t status;
            if ((body[13] | body[16] | body[17] | body[18] | body[19] |
                 body[20] | body[21] | body[22] | body[23] |
                 body[24] | body[25]) != 0U)
                return M6_RESULT_INVALID_ARGUMENT;
            status = AppH2Panel_CheckStart(SystemTime_Now());
            if (status != APP_H2_START_READY) {
                *detail = (uint32_t)status;
                return M6_RESULT_WRONG_STATE;
            }
            if (!AppLine_StartMission(snapshot.selected_mode,
                                      SystemTime_Now()))
                return M6_RESULT_WRONG_STATE;
        } else if (command == M6_H_MISSION_COMMAND_STAGE) {
            if (body[13] != 0U) return M6_RESULT_INVALID_ARGUMENT;
            config.h4_straight_speed_mm_s = (float)u16(&body[16]);
            config.h4_b_margin_mm = (float)u16(&body[18]);
            config.h5_straight_speed_mm_s = (float)u16(&body[20]);
            config.h5_curve_speed_mm_s = (float)u16(&body[22]);
            config.development_end_distance_mm = (float)u16(&body[24]);
            if (!HMission_Stage(&config)) return M6_RESULT_OUT_OF_RANGE;
        } else if (command == M6_H_MISSION_COMMAND_APPLY) {
            if ((body[13] | body[16] | body[17] | body[18] | body[19] |
                 body[20] | body[21] | body[22] | body[23] |
                 body[24] | body[25]) != 0U)
                return M6_RESULT_INVALID_ARGUMENT;
            if (!HMission_Apply()) return M6_RESULT_WRONG_STATE;
        } else if (command == M6_H_MISSION_COMMAND_REVERT) {
            if ((body[13] | body[16] | body[17] | body[18] | body[19] |
                 body[20] | body[21] | body[22] | body[23] |
                 body[24] | body[25]) != 0U)
                return M6_RESULT_INVALID_ARGUMENT;
            if (!HMission_Revert()) return M6_RESULT_WRONG_STATE;
        } else if (command == M6_H_MISSION_COMMAND_SELECT_MODE) {
            if ((body[16] | body[17] | body[18] | body[19] |
                 body[20] | body[21] | body[22] | body[23] |
                 body[24] | body[25]) != 0U)
                return M6_RESULT_INVALID_ARGUMENT;
            if (!HMission_Select((HMissionMode_t)body[13]))
                return M6_RESULT_INVALID_ARGUMENT;
        } else {
            return M6_RESULT_INVALID_ARGUMENT;
        }
        fill_h_mission(out);
        *out_length = M6_H_MISSION_SNAPSHOT_SIZE;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_ZDT_STEPPER_COMMAND) {
        ZdtStepperTestRequest_t request;
        ZdtStepperResult_t result;
        uint8_t command;
        if ((M6Link_EnabledCapabilities() & M6_CAP_ZDT_STEPPER_CONTROL) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != M6_ZDT_STEPPER_COMMAND_REQUEST_SIZE)
            return M6_RESULT_BAD_LENGTH;
        if (u32(body) != s_boot_id) return M6_RESULT_INVALID_ARGUMENT;
        if ((body[21] | body[22] | body[23]) != 0U) return M6_RESULT_MALFORMED;
        if ((body[9] & 0xFEU) != 0U) return M6_RESULT_INVALID_ARGUMENT;
        command = body[8];
        if ((command < ZDT_TEST_COMMAND_ENABLE) ||
            (command > ZDT_TEST_COMMAND_POSITION))
            return M6_RESULT_INVALID_ARGUMENT;
        memset(&request, 0, sizeof(request));
        request.command_id = u32(&body[4]);
        request.command = (ZdtStepperTestCommand_t)command;
        request.confirmed = (body[9] & 1U) != 0U;
        request.direction = (ZdtStepperDirection_t)body[10];
        request.acceleration = body[11];
        request.speed_rpm = u16(&body[12]);
        request.duration_or_timeout_ms = u16(&body[14]);
        request.pulses = u32(&body[16]);
        request.position_mode = (ZdtStepperPositionMode_t)body[20];
        if ((command == ZDT_TEST_COMMAND_ENABLE) &&
            (!request.confirmed || (body[10] | body[11] | body[12] |
             body[13] | body[14] | body[15] | body[16] | body[17] |
             body[18] | body[19] | body[20]) != 0U))
            return M6_RESULT_INVALID_ARGUMENT;
        if (((command == ZDT_TEST_COMMAND_STOP) ||
             (command == ZDT_TEST_COMMAND_DISABLE)) &&
            ((body[9] | body[10] | body[11] | body[12] | body[13] |
              body[14] | body[15] | body[16] | body[17] | body[18] |
              body[19] | body[20]) != 0U))
            return M6_RESULT_INVALID_ARGUMENT;
        if ((command == ZDT_TEST_COMMAND_VELOCITY) &&
            (!request.confirmed || (request.direction > ZDT_STEPPER_DIRECTION_CCW) ||
             (request.speed_rpm < 1U) || (request.speed_rpm > 300U) ||
             (request.duration_or_timeout_ms < 100U) ||
             (request.duration_or_timeout_ms > 5000U) ||
             (request.pulses != 0U) ||
             (request.position_mode != ZDT_STEPPER_POSITION_RELATIVE_LAST_TARGET)))
            return M6_RESULT_OUT_OF_RANGE;
        if ((command == ZDT_TEST_COMMAND_POSITION) &&
            (!request.confirmed || (request.direction > ZDT_STEPPER_DIRECTION_CCW) ||
             (request.speed_rpm < 1U) || (request.speed_rpm > 300U) ||
             (request.duration_or_timeout_ms < 500U) ||
             (request.duration_or_timeout_ms > 10000U) ||
             (request.pulses < 1U) || (request.pulses > 3200U) ||
             (request.position_mode != ZDT_STEPPER_POSITION_RELATIVE_CURRENT)))
            return M6_RESULT_OUT_OF_RANGE;
        result = ZdtStepperTest_Command(&request, SystemTime_Now());
        if (result == ZDT_STEPPER_TX_FULL) {
            *detail = ((uint32_t)M6_ZDT_STEPPER_RESOURCE << 24) |
                M6_ZDT_STEPPER_RETRY_MS;
            return M6_RESULT_BUSY;
        }
        if (result != ZDT_STEPPER_OK) {
            *detail = ZdtStepperTest_GetLastRejectDetail();
            if (*detail == 0U) *detail = ZDT_STEPPER_NOT_READY_INTERNAL;
            return M6_RESULT_WRONG_STATE;
        }
        fill_zdt_stepper(out); *out_length = M6_ZDT_STEPPER_SNAPSHOT_SIZE;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_SESSION_CLOSE) {
        if (length != 0U) return M6_RESULT_BAD_LENGTH;
        return action_base_safe() ? M6_RESULT_OK : M6_RESULT_WRONG_STATE;
    }
    if (opcode == M6_RPC_FAULT_CLEAR) {
        SafetySnapshot_t safety;
        if (length != 8U) return M6_RESULT_BAD_LENGTH;
        Safety_GetSnapshot(&safety);
        if ((u32(body) != s_boot_id) || (u32(&body[4]) != safety.fault_bits))
            return M6_RESULT_INVALID_ARGUMENT;
        if (!Safety_HasFault() || (safety.applied_motor_a != 0) ||
            (safety.applied_motor_b != 0)) return M6_RESULT_WRONG_STATE;
        if (!Safety_ClearFaults(SystemTime_Now())) return M6_RESULT_WRONG_STATE;
        p32(&out[0], s_boot_id); p32(&out[4], safety.fault_bits); p32(&out[8], 0U);
        *out_length = 12U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_STAGE_SET) {
        uint8_t id, expected_type; const ControlParamDescriptor_t *d; float value;
        if (length != 12U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != ControlParams_StageRevision()) {
            *detail = ControlParams_StageRevision();
            return M6_RESULT_STALE_STAGE_REVISION;
        }
        id = body[4]; expected_type = id == 35U ? 2U : 1U;
        if ((body[6] | body[7]) != 0U) return M6_RESULT_MALFORMED;
        d = id ? ControlParams_DescriptorAt(id - 1U) : NULL;
        if (d == NULL) return M6_RESULT_UNKNOWN_PARAMETER;
        if (body[5] != expected_type) return M6_RESULT_INVALID_ARGUMENT;
        if (expected_type == 2U) { if (u32(&body[8]) > 1U) return M6_RESULT_INVALID_VALUE; value = (float)u32(&body[8]); }
        else memcpy(&value, &body[8], 4U);
        pr = ControlParams_Set(d->key, value); if (pr != CONTROL_PARAMS_OK) return params_result(pr);
        memset(out, 0, 12U); out[0] = id; out[1] = expected_type; out[2] = 2U;
        out[3] = ControlParams_IsDirty(); p32(&out[4], ControlParams_StageRevision());
        memcpy(&out[8], &body[8], 4U); *out_length = 12U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_APPLY) {
        ControlParamsApplyContext_t context; ControlParamsCommit_t commit;
        if (length != 12U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != ControlParams_Revision()) {
            *detail = ControlParams_Revision();
            return M6_RESULT_STALE_ACTIVE_REVISION;
        }
        if (u32(&body[4]) != ControlParams_StageRevision()) {
            *detail = ControlParams_StageRevision();
            return M6_RESULT_STALE_STAGE_REVISION;
        }
        if (u32(&body[8]) != store.generation) {
            *detail = store.generation;
            return M6_RESULT_STALE_GENERATION;
        }
        context.wheel_diagnostic_armed =
            Diagnostics_WheelSpeedDiagnosticActive() ||
            ((s_action.state != 0U) &&
             (s_action.kind == M6_ACTION_KIND_WHEEL_SPEED_TEST));
        context.fault_free_idle = action_base_safe();
        pr = ControlParams_RequestApply(context, &commit);
        if (pr == CONTROL_PARAMS_PENDING) {
            s_pending_apply_sequence = M6Link_CurrentRpcSequence();
            return M6_EXECUTOR_PENDING;
        }
        if (pr != CONTROL_PARAMS_OK) return params_result(pr);
        if (s_commit_callback != NULL) s_commit_callback(&commit);
        memset(out, 0, 20U); p32(&out[0], commit.revision);
        p32(&out[4], ControlParams_StageRevision()); p64(&out[8], commit.change_mask);
        out[16] = source_wire(commit.source); out[17] = commit.diagnostic_boundary;
        *out_length = 20U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_REVERT) {
        if (length != 8U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != ControlParams_Revision()) {
            *detail = ControlParams_Revision();
            return M6_RESULT_STALE_ACTIVE_REVISION;
        }
        if (u32(&body[4]) != ControlParams_StageRevision()) {
            *detail = ControlParams_StageRevision();
            return M6_RESULT_STALE_STAGE_REVISION;
        }
        pr = ControlParams_Revert(); if (pr != CONTROL_PARAMS_OK) return params_result(pr);
        memset(out, 0, 12U); p32(out, ControlParams_Revision()); p32(&out[4], ControlParams_StageRevision());
        out[8] = source_wire(ControlParams_Source()); *out_length = 12U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_DEFAULTS) {
        if (length != 4U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != ControlParams_StageRevision()) {
            *detail = ControlParams_StageRevision();
            return M6_RESULT_STALE_STAGE_REVISION;
        }
        pr = ControlParams_StageDefaults(); if (pr != CONTROL_PARAMS_OK) return params_result(pr);
        memset(out, 0, 8U); p32(out, ControlParams_StageRevision()); out[4] = 0U;
        out[5] = ControlParams_IsDirty(); *out_length = 8U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_LOAD) {
        if (length != 8U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != ControlParams_StageRevision()) {
            *detail = ControlParams_StageRevision();
            return M6_RESULT_STALE_STAGE_REVISION;
        }
        if (u32(&body[4]) != store.generation) {
            *detail = store.generation;
            return M6_RESULT_STALE_GENERATION;
        }
        if (ParameterStore_LoadStaged() != PARAMETER_STORE_OK) return M6_RESULT_STORAGE_IO;
        ParameterStore_GetInfo(&store); memset(out, 0, 12U);
        p32(out, ControlParams_StageRevision()); p32(&out[4], store.generation);
        out[8] = source_wire(ControlParams_StagedSource()); out[9] = (uint8_t)store.selected_slot;
        out[10] = (uint8_t)store.health; out[11] = ControlParams_IsDirty();
        *out_length = 12U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_PARAM_SAVE) {
        if (length != 8U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != ControlParams_Revision()) {
            *detail = ControlParams_Revision();
            return M6_RESULT_STALE_ACTIVE_REVISION;
        }
        if (u32(&body[4]) != store.generation) {
            *detail = store.generation;
            return M6_RESULT_STALE_GENERATION;
        }
        if (!save_safe()) return M6_RESULT_WRONG_STATE;
        if (ParameterStore_SaveActive() != PARAMETER_STORE_OK) return M6_RESULT_STORAGE_IO;
        ParameterStore_GetInfo(&store); memset(out, 0, 12U); p32(out, store.generation);
        p32(&out[4], ControlParams_Revision()); out[8] = (uint8_t)store.selected_slot;
        out[9] = (uint8_t)store.health; *out_length = 12U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_SUBSCRIPTION_SET) {
        static const uint16_t minimum[4] = {20U,20U,40U,200U};
        static const uint16_t maximum[4] = {1000U,1000U,1000U,5000U};
        static const uint16_t quantum[4] = {20U,20U,20U,100U};
        static const uint8_t payload_size[4] = {40U,60U,56U,72U};
        M6Subscription_t next[4] = {{0}};
        uint8_t count, seen = 0U; uint32_t estimated_bps = 0U;
        if ((length < 4U) || (length > 20U) || (body[0] != 1U) ||
            ((body[2] | body[3]) != 0U)) return M6_RESULT_INVALID_ARGUMENT;
        count = body[1]; if ((count > 4U) || (length != (uint16_t)(4U + count * 4U))) return M6_RESULT_BAD_LENGTH;
        for (uint8_t i = 0U; i < count; i++) {
            uint8_t stream = body[4U + i * 4U]; uint16_t period = u16(&body[6U + i * 4U]);
            if ((stream < 1U) || (stream > 4U) || body[5U + i * 4U] != 0U)
                return M6_RESULT_INVALID_ARGUMENT;
            if ((seen & (uint8_t)(1U << (stream - 1U))) != 0U)
                return M6_RESULT_INVALID_ARGUMENT;
            seen |= (uint8_t)(1U << (stream - 1U));
            if ((period < minimum[stream - 1U]) ||
                (period > maximum[stream - 1U]) ||
                ((period % quantum[stream - 1U]) != 0U))
                return M6_RESULT_UNSUPPORTED_RATE;
            estimated_bps += ((1000U + period - 1U) / period) *
                (17U + payload_size[stream - 1U]);
            next[stream - 1U].enabled = true;
            next[stream - 1U].period_ms = period;
            next[stream - 1U].due_ms = SystemTime_Now();
        }
        if (estimated_bps > 7000U) return M6_RESULT_BANDWIDTH_EXCEEDED;
        {
            uint16_t affected = 0U;
            for (uint8_t i = 0U; i < 4U; i++)
                if (s_subscriptions[i].enabled || next[i].enabled)
                    affected |= (uint16_t)(1U << i);
            (void)M6Link_ClearTelemetry(affected,
                                        M6_GAP_PRODUCER_DROP);
        }
        memcpy(s_subscriptions, next, sizeof(next));
        memset(out, 0, 24U); out[0] = count;
        p32(&out[4], estimated_bps);
        /* Canonical sorted entries. */
        { uint8_t j = 0U; for (uint8_t i = 0U; i < 4U; i++) if (s_subscriptions[i].enabled) {
            out[8U + j * 4U] = i + 1U; p16(&out[10U + j * 4U], s_subscriptions[i].period_ms); j++; }}
        *out_length = (uint16_t)(8U + count * 4U); return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_MANUAL_DRIVE_SET) {
        uint32_t now = SystemTime_Now();
        uint16_t flags;
        int16_t left;
        int16_t right;
        uint16_t lease;
        if ((M6Link_EnabledCapabilities() & M6_CAP_MANUAL_DRIVE) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != 16U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != s_boot_id) return M6_RESULT_INVALID_ARGUMENT;
        left = i16(&body[8]);
        right = i16(&body[10]);
        lease = u16(&body[12]);
        flags = u16(&body[14]);
        if ((flags & (uint16_t)~(M6_MANUAL_DRIVE_FLAG_TAKEOVER |
                                 M6_MANUAL_DRIVE_FLAG_RELEASE)) != 0U)
            return M6_RESULT_INVALID_ARGUMENT;
        if ((flags & M6_MANUAL_DRIVE_FLAG_RELEASE) != 0U) {
            if ((left != 0) || (right != 0) || (lease != 0U) ||
                ((flags & M6_MANUAL_DRIVE_FLAG_TAKEOVER) != 0U))
                return M6_RESULT_INVALID_ARGUMENT;
            (void)AppManualDrive_Stop(APP_MANUAL_SOURCE_HOST_RPC,
                APP_MANUAL_STOP_RELEASE, now);
            fill_manual_drive(out);
            *out_length = 16U;
            return M6_RESULT_OK;
        }
        if ((left == 0) && (right == 0)) return M6_RESULT_INVALID_ARGUMENT;
        if ((left < -400) || (left > 400) ||
            (right < -400) || (right > 400) ||
            (lease < 100U) || (lease > 500U))
            return M6_RESULT_OUT_OF_RANGE;
        if ((flags & M6_MANUAL_DRIVE_FLAG_TAKEOVER) != 0U) {
            stop_action(now);
            (void)Diagnostics_ActionCancel(now);
            (void)AppRemote_NormalStop(now);
        }
        if (!AppManualDrive_Submit(APP_MANUAL_SOURCE_HOST_RPC,
                left, right, lease, 0U, u32(&body[4]), now))
            return Safety_HasFault() ? M6_RESULT_SAFETY_FAULT :
                                      M6_RESULT_WRONG_STATE;
        fill_manual_drive(out);
        *out_length = 16U;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_VISION_UART_MONITOR_SET) {
        if ((M6Link_EnabledCapabilities() &
             M6_CAP_VISION_UART_MONITOR) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != 8U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != s_boot_id) return M6_RESULT_INVALID_ARGUMENT;
        if ((body[5] | body[6] | body[7]) != 0U)
            return M6_RESULT_MALFORMED;
        if (body[4] > 1U) return M6_RESULT_INVALID_ARGUMENT;
        M6VisionUart_SetEnabled(body[4] != 0U);
        M6VisionUart_GetStatus(out, out_length);
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_GRAYSCALE_CALIBRATION_COMMAND) {
        GrayscaleCalibrationResult_t result;
        if ((M6Link_EnabledCapabilities() &
             M6_CAP_GRAYSCALE_CALIBRATION) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != 12U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != s_boot_id) return M6_RESULT_INVALID_ARGUMENT;
        if ((body[5] | body[6] | body[7]) != 0U)
            return M6_RESULT_MALFORMED;
        if ((body[4] < GRAYSCALE_CAL_COMMAND_START) ||
            (body[4] > GRAYSCALE_CAL_COMMAND_CANCEL))
            return M6_RESULT_INVALID_ARGUMENT;
        result = GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST,
            M6Link_CurrentSession(), (GrayscaleCalibrationCommand_t)body[4],
            u32(&body[8]), SystemTime_Now());
        if (result != GRAYSCALE_CAL_RESULT_OK)
            return grayscale_calibration_result(result, detail);
        fill_grayscale_calibration(out);
        *out_length = 84U;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_LEVER_CALIBRATION_COMMAND) {
        LeverCalibrationResult_t result;
        uint8_t command;
        if ((M6Link_EnabledCapabilities() &
             M6_CAP_LEVER_CALIBRATION) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (length != M6_LEVER_CAL_COMMAND_REQUEST_SIZE)
            return M6_RESULT_BAD_LENGTH;
        if (u32(body) != s_boot_id) return M6_RESULT_INVALID_ARGUMENT;
        if ((body[18] | body[19]) != 0U) return M6_RESULT_MALFORMED;
        command = body[8];
        if ((command < LEVER_CAL_COMMAND_START) ||
            (command > LEVER_CAL_COMMAND_CANCEL))
            return M6_RESULT_INVALID_ARGUMENT;
        if ((command != LEVER_CAL_COMMAND_CAPTURE_POINT) &&
            ((body[9] | body[10] | body[11]) != 0U))
            return M6_RESULT_INVALID_ARGUMENT;
        if ((command != LEVER_CAL_COMMAND_START) && (u16(&body[16]) != 0U))
            return M6_RESULT_INVALID_ARGUMENT;
        result = LeverCalibration_Command(LEVER_CAL_OWNER_HOST,
            M6Link_CurrentSession(), (LeverCalibrationCommand_t)command,
            u32(&body[4]), body[9],
            (LeverCalibrationDirection_t)body[10], body[11],
            i32(&body[12]), u16(&body[16]), SystemTime_Now());
        if (result != LEVER_CAL_RESULT_OK)
            return lever_calibration_result(result, detail);
        fill_lever_calibration(out, 0U);
        *out_length = M6_LEVER_CAL_SNAPSHOT_SIZE;
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_ACTION_ARM) {
        uint32_t now = SystemTime_Now();
        uint8_t token_input[40];
        M6Result_t validation = M6Action_Validate(body, length,
            M6Link_EnabledCapabilities());
        if (validation != M6_RESULT_OK) return validation;
        if (!action_base_safe()) return M6_RESULT_WRONG_STATE;
        if (u32(&body[20]) != ControlParams_Revision()) {
            *detail = ControlParams_Revision();
            return M6_RESULT_STALE_ACTIVE_REVISION;
        }
        memset(&s_action, 0, sizeof(s_action)); s_action.state = 1U; s_action.kind = body[0];
        s_action.wheel_mask = body[1]; s_action.left = i16(&body[4]); s_action.right = i16(&body[6]);
        s_action.angle_mdeg = i32(&body[8]);
        s_action.yaw_rate_mdeg_s = i32(&body[12]);
        s_action.duration_ms = u32(&body[16]);
        s_action.armed_revision = ControlParams_Revision();
        s_action.armed_session = M6Link_CurrentSession();
        memcpy(s_action.parameters, body, sizeof(s_action.parameters));
        p32(&token_input[0], M6Link_CurrentSession());
        p32(&token_input[4], M6Link_CurrentRpcSequence());
        p32(&token_input[8], s_boot_id);
        p32(&token_input[12], s_action.armed_revision);
        memcpy(&token_input[16], body, 24U);
        s_action.token = M6Wire_Crc32(token_input, sizeof(token_input));
        if (s_action.token == 0U) s_action.token = 1U;
        s_action.expires_ms = now + 10000U; p32(out, s_action.token); p32(&out[4], 10000U);
        memcpy(&out[8], body, 24U); *out_length = 32U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_ACTION_CONFIRM) {
        uint32_t now = SystemTime_Now(); bool started = false;
        M6Result_t validation;
        if (length != 8U) return M6_RESULT_BAD_LENGTH;
        if ((u32(body) != s_boot_id) || (u32(&body[4]) != s_action.token)) return M6_RESULT_INVALID_ARGUMENT;
        if (s_action.state != 1U) return M6_RESULT_NOT_ARMED;
        if ((int32_t)(now - s_action.expires_ms) >= 0) { s_action.state = 0U; return M6_RESULT_ARM_EXPIRED; }
        validation = M6Action_Validate(s_action.parameters,
            sizeof(s_action.parameters), M6Link_EnabledCapabilities());
        if (validation != M6_RESULT_OK) return validation;
        if ((M6Link_CurrentSession() != s_action.armed_session) ||
            (ControlParams_Revision() != s_action.armed_revision) ||
            (u32(&s_action.parameters[20]) != ControlParams_Revision()) ||
            (SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
            !Diagnostics_ActionIsIdle() || !remote_idle())
            return M6_RESULT_WRONG_STATE;
        if ((s_action.kind == M6_ACTION_KIND_HEADING_HOLD) ||
            (s_action.kind == M6_ACTION_KIND_HEADING_TURN) ||
            (s_action.kind == M6_ACTION_KIND_HEADING_RATE)) {
            HeadingSnapshot_t heading;
            Heading_GetSnapshot(&heading);
            if (!heading.valid || !heading.calibrated)
                return M6_RESULT_SENSOR_NOT_READY;
        }
        if (s_action.kind == M6_ACTION_KIND_LINE_FOLLOW) {
            started = AppLine_StartTimed(now, s_action.duration_ms);
        } else {
            if (!SystemState_RequestRun()) return M6_RESULT_WRONG_STATE;
            /* M6Rpc_Service owns the normal duration stop.  Give Safety a
             * separate, later deadline so scheduler ordering at the exact
             * action deadline cannot turn a normal completion into an
             * ACTION_TIMEOUT fault. */
            Safety_BeginAction(now,
                s_action.duration_ms + M6_ACTION_SAFETY_GRACE_MS,
                s_action.kind == M6_ACTION_KIND_WHEEL_SPEED_TEST ? 0U :
                SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO));
            WheelSpeed_Enable();
            if (s_action.kind == M6_ACTION_KIND_WHEEL_SPEED_TEST) {
                WheelSpeed_SetTarget((float)s_action.left,
                                     (float)s_action.right);
                started = true;
            } else if (s_action.kind == M6_ACTION_KIND_HEADING_HOLD)
                started = Heading_StartHold((float)s_action.left);
            else if (s_action.kind == M6_ACTION_KIND_HEADING_TURN)
                started = Heading_StartTurn(
                    (float)s_action.angle_mdeg / 1000.0f);
            else if (s_action.kind == M6_ACTION_KIND_HEADING_RATE)
                started = Heading_StartRate(
                    (float)s_action.yaw_rate_mdeg_s / 1000.0f);
        }
        if (!started) { stop_action(now); return M6_RESULT_SENSOR_NOT_READY; }
        s_action.state = 2U; s_action.action_id = s_next_action_id++;
        s_action.start_ms = now;
        s_action.deadline_ms =
            (s_action.duration_ms == M6_LINE_ACTION_CONTINUOUS_DURATION_MS)
                ? 0U
                : (now + s_action.duration_ms);
        memset(out, 0, 16U); p32(out, s_action.action_id); p32(&out[4], now);
        p32(&out[8], s_action.deadline_ms); out[12] = 2U; out[13] = s_action.kind;
        *out_length = 16U; return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_ACTION_CANCEL) {
        uint32_t now = SystemTime_Now(); if (length != 4U) return M6_RESULT_BAD_LENGTH;
        if ((s_action.state == 0U) || ((u32(body) != 0U) && (u32(body) != s_action.action_id))) return M6_RESULT_NOT_ARMED;
        { uint32_t id = s_action.action_id; stop_action(now); memset(out, 0, 12U);
          p32(out, id); p32(&out[4], now); out[8] = 0U; out[9] = 1U; *out_length = 12U; }
        return M6_RESULT_OK;
    }
    if (opcode == M6_RPC_HEADING_CALIBRATE) {
        HeadingSnapshot_t heading; uint32_t now = SystemTime_Now();
        if (length != 4U) return M6_RESULT_BAD_LENGTH;
        if (u32(body) != s_boot_id) return M6_RESULT_INVALID_ARGUMENT;
        if ((M6Link_EnabledCapabilities() & M6_CAP_HEADING_CALIBRATION) == 0U)
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
        if (!action_base_safe()) return M6_RESULT_WRONG_STATE;
        Heading_GetSnapshot(&heading);
        if (!heading.valid) return M6_RESULT_SENSOR_NOT_READY;
        Heading_StartCalibration();
        p32(out, now); p32(&out[4], heading.calibration_count); *out_length = 8U;
        return M6_RESULT_OK;
    }
    return M6_RESULT_UNSUPPORTED_OPCODE;
}

M6Result_t M6Rpc_Execute(uint16_t opcode, const uint8_t *body,
                         uint16_t body_length, uint8_t *success_body,
                         uint16_t *success_length, uint32_t *detail,
                         void *context)
{
    (void)context;
    if ((success_body == NULL) || (success_length == NULL) || (detail == NULL))
        return M6_RESULT_INTERNAL;
    if ((opcode & 0x8000U) != 0U)
        return execute_mutating(opcode, body, body_length, success_body,
                                success_length, detail);
    return execute_read(opcode, body, body_length, success_body,
                        success_length, detail);
}

void M6Rpc_Service(uint32_t now_ms)
{
    AppLineSnapshot_t line;
    HeadingSnapshot_t heading;
    SafetySnapshot_t safety;
    uint32_t session;
    uint32_t emergency_epoch;
    Safety_GetSnapshot(&safety);
    emergency_epoch = debug_get_emergency_epoch();
    session = M6Link_Mode() == M6_LINK_V2_ESTABLISHED ?
        M6Link_CurrentSession() : 0U;
    if ((session == 0U) && (s_event_session != 0U)) {
        GrayscaleCalibration_CancelHost(s_event_session, now_ms);
        LeverCalibration_CancelHost(s_event_session, now_ms);
        memset(s_subscriptions, 0, sizeof(s_subscriptions));
        M6VisionUart_SetEnabled(false);
        s_event_session = 0U;
    }
    if ((s_pending_apply_sequence != 0U) && Safety_HasFault()) {
        ControlParams_CancelPendingApply();
        M6Rpc_CancelPendingApply(M6_RESULT_SAFETY_FAULT,
                                 safety.fault_bits);
    }
    if ((session != 0U) && (session != s_event_session)) {
        s_event_session = session;
        memset(s_subscriptions, 0, sizeof(s_subscriptions));
        M6VisionUart_SetEnabled(false);
        for (uint8_t i = 0U; i < 4U; i++) s_next_sample_id[i] = 1U;
        publish_reset_event(now_ms);
        if (safety.fault_bits != 0U) publish_fault_event(&safety, now_ms);
    }
    if (emergency_epoch != s_last_emergency_epoch) {
        s_last_emergency_epoch = emergency_epoch;
        publish_emergency_event(&safety, emergency_epoch, now_ms);
    }
    if ((safety.fault_bits != 0U) &&
        (safety.fault_bits != s_last_fault_bits))
        publish_fault_event(&safety, now_ms);
    s_last_fault_bits = safety.fault_bits;
    M6VisionUart_Service(now_ms);
    {
        AppManualDriveSnapshot_t manual;
        AppManualDrive_GetSnapshot(&manual);
        if ((session != 0U) &&
            (manual.transition_id != s_last_manual_transition))
            publish_manual_drive_event(now_ms);
        s_last_manual_transition = manual.transition_id;
    }
    if ((s_action.state == 1U) && ((int32_t)(now_ms - s_action.expires_ms) >= 0))
        memset(&s_action, 0, sizeof(s_action));
    if (s_action.state != 2U) return;
    Heading_GetSnapshot(&heading);
    AppLine_GetSnapshot(&line);
    if (Safety_HasFault() ||
        ((s_action.duration_ms != M6_LINE_ACTION_CONTINUOUS_DURATION_MS) &&
         ((int32_t)(now_ms - s_action.deadline_ms) >= 0)) ||
        ((s_action.kind == M6_ACTION_KIND_HEADING_TURN) && heading.complete) ||
        ((s_action.kind == M6_ACTION_KIND_LINE_FOLLOW) &&
         (line.state != APP_LINE_RUNNING)))
        stop_action(now_ms);
}

static bool fill_telemetry_payload(uint8_t stream, uint32_t now_ms,
                                   uint8_t *payload,
                                   uint16_t *payload_length)
{
    WheelSpeedSnapshot_t wheel; HeadingSnapshot_t heading; SafetySnapshot_t safety;
    if ((payload == NULL) || (payload_length == NULL) || (stream >= 4U))
        return false;
    memset(payload, 0, 72U);
    if (stream == 0U) {
            AppLineSnapshot_t line; uint8_t saturation = 0U;
            WheelSpeed_GetSnapshot(&wheel); Heading_GetSnapshot(&heading);
            Safety_GetSnapshot(&safety); AppLine_GetSnapshot(&line);
            p32(&payload[8], now_ms); p32(&payload[12], ControlParams_Revision());
            p16(&payload[16], (uint16_t)round_i16(wheel.requested_mm_s[0]));
            p16(&payload[18], (uint16_t)round_i16(wheel.requested_mm_s[1]));
            p16(&payload[20], (uint16_t)round_i16(wheel.filtered_mm_s[0]));
            p16(&payload[22], (uint16_t)round_i16(wheel.filtered_mm_s[1]));
            p16(&payload[24], (uint16_t)wheel.pwm[0]); p16(&payload[26], (uint16_t)wheel.pwm[1]);
            p16(&payload[28], (uint16_t)round_i16(heading.yaw_deg * 100.0f));
            p32(&payload[30], (uint32_t)round_i32(
                heading.yaw_rate_deg_s * 100.0f));
            if (wheel.saturated[0]) saturation |= 1U;
            if (wheel.saturated[1]) saturation |= 2U;
            if (line.control.raw_correction_mm_s !=
                line.control.correction_mm_s) saturation |= 4U;
            if ((wheel.requested_mm_s[0] >=
                 CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S) ||
                (wheel.requested_mm_s[0] <=
                 -CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S) ||
                (wheel.requested_mm_s[1] >=
                 CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S) ||
                (wheel.requested_mm_s[1] <=
                 -CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S)) saturation |= 8U;
            payload[34] = saturation;
            payload[35] = (uint8_t)heading.rate_source; p32(&payload[36], safety.fault_bits);
            *payload_length = 40U;
        } else if (stream == 1U) {
            uint16_t flags = 0U;
            WheelSpeed_GetSnapshot(&wheel); p32(&payload[8], now_ms);
            p16(&payload[12], (uint16_t)round_i16(wheel.raw_mm_s[0])); p16(&payload[14], (uint16_t)round_i16(wheel.raw_mm_s[1]));
            p16(&payload[16], (uint16_t)round_i16(wheel.filtered_mm_s[0])); p16(&payload[18], (uint16_t)round_i16(wheel.filtered_mm_s[1]));
            p16(&payload[20], (uint16_t)round_i16(wheel.error_mm_s[0])); p16(&payload[22], (uint16_t)round_i16(wheel.error_mm_s[1]));
            p16(&payload[24], (uint16_t)round_i16(wheel.proportional_pwm[0])); p16(&payload[26], (uint16_t)round_i16(wheel.proportional_pwm[1]));
            p16(&payload[28], (uint16_t)round_i16(wheel.integral_pwm[0])); p16(&payload[30], (uint16_t)round_i16(wheel.integral_pwm[1]));
            p16(&payload[32], (uint16_t)round_i16(wheel.feedforward_pwm[0])); p16(&payload[34], (uint16_t)round_i16(wheel.feedforward_pwm[1]));
            p16(&payload[36], (uint16_t)round_i16(wheel.ramped_mm_s[0])); p16(&payload[38], (uint16_t)round_i16(wheel.ramped_mm_s[1]));
            p16(&payload[40], (uint16_t)wheel.pwm[0]); p16(&payload[42], (uint16_t)wheel.pwm[1]);
            p16(&payload[44], saturate_u16(wheel.saturation_ms[0])); p16(&payload[46], saturate_u16(wheel.saturation_ms[1]));
            p16(&payload[48], saturate_u16(wheel.actual_dt_us));
            if (wheel.enabled) flags |= 1U;
            if (wheel.sample_valid) flags |= 2U;
            if (wheel.saturated[0]) flags |= 4U;
            if (wheel.saturated[1]) flags |= 8U;
            if ((wheel.fault_bits & WHEEL_SPEED_FAULT_ENCODER_INVALID) != 0U) flags |= 16U;
            if ((wheel.fault_bits & WHEEL_SPEED_FAULT_STALL) != 0U) flags |= 32U;
            p16(&payload[50], flags);
            p32(&payload[52], (uint32_t)wheel.encoder_delta[0]); p32(&payload[56], (uint32_t)wheel.encoder_delta[1]);
            *payload_length = 60U;
        } else if (stream == 2U) {
            GrayscaleSnapshot_t gray; AppLineSnapshot_t line; uint8_t flags = 0U;
            Grayscale_GetSnapshot(&gray); AppLine_GetSnapshot(&line);
            p32(&payload[8], now_ms);
            for (uint8_t i = 0U; i < 8U; i++) p16(&payload[12U + i * 2U], gray.adc[i]);
            payload[28] = line.control.active_mask; payload[29] = line.control.active_count;
            payload[30] = line.control.cluster_count; payload[31] = (uint8_t)line.control.state;
            payload[32] = (uint8_t)line.state; payload[33] = (uint8_t)line.control.search_direction;
            if (line.control.line_valid) flags |= 1U;
            if (line.control.ambiguous) flags |= 2U;
            if (line.control.branch_direction_latched) flags |= 4U;
            if (gray.last_scan_ok) flags |= 8U;
            if (line.control.calibration.valid) flags |= 16U;
            if (line.control.calibration.compiled_verified) flags |= 32U;
            if (line.control.state == LINE_FOLLOW_SEARCHING) flags |= 64U;
            if ((line.control.left_target_mm_s >=
                 CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S) ||
                (line.control.left_target_mm_s <=
                 -CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S) ||
                (line.control.right_target_mm_s >=
                 CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S) ||
                (line.control.right_target_mm_s <=
                 -CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S)) flags |= 128U;
            payload[34] = flags;
            p16(&payload[36], (uint16_t)round_i16(line.control.raw_position * 1000.0f));
            p16(&payload[38], (uint16_t)round_i16(line.control.position * 1000.0f));
            p32(&payload[40], (uint32_t)round_i32(
                line.control.filtered_derivative * 1000.0f));
            p16(&payload[44], round_u16(line.control.confidence * 1000.0f));
            p16(&payload[46], (uint16_t)round_i16(line.control.dynamic_base_mm_s));
            p16(&payload[48], (uint16_t)round_i16(line.control.correction_mm_s));
            p16(&payload[50], (uint16_t)round_i16(line.control.left_target_mm_s));
            p16(&payload[52], (uint16_t)round_i16(line.control.right_target_mm_s));
            p16(&payload[54], saturate_u16(line.control.lost_since_ms == 0U ?
                0U : now_ms - line.control.lost_since_ms));
            *payload_length = 56U;
        } else {
            ResetDiagSnapshot_t reset; SafetySnapshot_t safety; SchedulerStats_t scheduler;
            DebugStats_t debug; M6TelemetryStats_t legacy; M6LinkStats_t link;
            GyroSnapshot_t gyro; GrayscaleSnapshot_t gray;
            AppLineSnapshot_t line; ParameterStoreInfo_t store; uint8_t flags = 0U;
            ResetDiag_GetSnapshot(&reset); Safety_GetSnapshot(&safety); Scheduler_GetStats(&scheduler);
            debug_get_stats(&debug); M6Telemetry_GetStats(&legacy); M6Link_GetStats(&link);
            Gyro_GetSnapshot(&gyro); Grayscale_GetSnapshot(&gray);
            AppLine_GetSnapshot(&line); ParameterStore_GetInfo(&store);
            p32(&payload[8], now_ms); p32(&payload[12], reset.boot_id);
            p32(&payload[16], reset.raw_cause); p32(&payload[20], safety.fault_bits);
            payload[24] = (uint8_t)SystemState_Get(); payload[25] = (uint8_t)reset.reset_class;
            payload[26] = (uint8_t)reset.checkpoint;
            if ((scheduler.missed_mask == 0U) &&
                ((scheduler.health_window_mask & SCHEDULER_CRITICAL_TASKS_MASK) ==
                 SCHEDULER_CRITICAL_TASKS_MASK)) flags |= 1U;
            if (gyro.valid_frames != 0U) flags |= 2U;
            if (gray.last_scan_ok) flags |= 4U;
            if (line.control.line_valid) flags |= 8U;
            if (store.health == PARAMETER_STORE_HEALTH_DEGRADED) flags |= 16U;
            if (reset.previous_detail_valid) flags |= 32U;
            if (debug.format_errors != 0U) flags |= 64U;
            if ((link.p0_drop | link.p1_drop | link.p2_drop |
                 link.p3_drop | link.control_error_drop) != 0U) flags |= 128U;
            payload[27] = flags;
            for (uint8_t i = 0U; i < 4U; i++) p16(&payload[28U + i * 2U],
                scheduler.miss_count[i] > 65535U ? 65535U : (uint16_t)scheduler.miss_count[i]);
            p32(&payload[36], debug.tx_dropped); p32(&payload[40], debug.rx_dropped);
            p32(&payload[44], debug.format_errors); p32(&payload[48], legacy.dropped);
            p32(&payload[52], Diagnostics_Get100msDropped());
            p32(&payload[56], link.p2_drop); p32(&payload[60], link.p3_drop);
            payload[64] = debug_get_rx_high_water() > 255U ? 255U : (uint8_t)debug_get_rx_high_water();
            payload[65] = link.p0_high_water; payload[66] = link.p1_high_water;
            payload[67] = link.p2_high_water; payload[68] = link.p3_high_water;
            payload[69] = legacy.queued;
            *payload_length = 72U;
    }
    return true;
}

void M6Rpc_Telemetry20ms(uint32_t now_ms)
{
    uint8_t payload[72];
    uint16_t payload_length;
    for (uint8_t stream = 0U; stream < 4U; stream++) {
        M6Subscription_t *sub = &s_subscriptions[stream];
        if (!sub->enabled || ((int32_t)(now_ms - sub->due_ms) < 0)) continue;
        sub->due_ms = now_ms + sub->period_ms;
        if (!fill_telemetry_payload(stream, now_ms, payload,
                                    &payload_length))
            continue;
        payload[0] = stream + 1U;
        payload[1] = 1U;
        p32(&payload[4], s_next_sample_id[stream]++);
        (void)M6Link_Publish(M6_TX_P2, M6_FRAME_TELEMETRY,
                             stream + 1U, payload, payload_length);
    }
}
