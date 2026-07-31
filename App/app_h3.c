#include "app_h3.h"

#include "app_line.h"
#include "ball_pd.h"
#include "ball_vision.h"
#include "h3_control_config.h"
#include "h3_tune.h"
#include "lever_actuator.h"
#include "lever_motion_config.h"
#include "m6_h3_tune_ext.h"
#include "m6_link.h"
#include "m6_vision_uart.h"
#include "m6_wire.h"
#include "oled.h"
#include "safety.h"
#include "system_state.h"
#include "wheel_speed.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define APP_H3_DEBUG_KIND               3U
#define APP_H3_DEBUG_SCHEMA             3U
#define APP_H3_DEBUG_LENGTH            50U
#define APP_H3_TARGET_X10 (H3_TARGET_POSITION_MM * 10L)

static AppH3Snapshot_t s_h3;
static BallPdState_t s_pd;
static int32_t s_stage_start_x_mm_x10;
static uint32_t s_stage_started_ms;
static bool s_stage_direction_confirmed;
static bool s_stage_reversal_grace;
static bool s_stage_launch_withdrawing;
static uint8_t s_stage_launch_pulses;
static bool s_stage_relaunching;
static bool s_plus_topup;
static bool s_plus_topup_available;
static bool s_plus_hard_braking;
static bool s_plus_brake_encoder_reversed;
static int32_t s_plus_brake_start_counts;
static bool s_minus_hard_braking;
static bool s_minus_hard_brake_available;
static bool s_minus_hard_brake_pending;
static bool s_minus_hard_brake_negative_speed_seen;
static int32_t s_minus_hard_brake_tilt_mdeg;
static uint32_t s_minus_hard_brake_due_ms;
static uint32_t s_relaunch_ready_since_ms;
static uint32_t s_settle_started_ms;
static uint32_t s_next_debug_ms;
static bool s_owns_run_state;
static volatile bool s_emergency_requested;

static uint32_t abs_i32(int32_t value)
{
    if (value >= 0) return (uint32_t)value;
    return (uint32_t)(-(value + 1)) + 1U;
}

static bool reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static int32_t adaptive_minus_brake_tilt_mdeg(
    const H3TuneProfile_t *profile, int32_t actuator_counts)
{
    int32_t reference_delta_counts = (int32_t)(((int64_t)
        H3_MINUS_BRAKE_ENCODER_REFERENCE_MDEG *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);
    int32_t reference_counts = LEVER_HORIZONTAL_COUNTS -
        H3_DIRECTION_SIGN * reference_delta_counts;
    int32_t correction_counts = H3_DIRECTION_SIGN *
        (reference_counts - actuator_counts);
    int32_t correction_mdeg = (int32_t)(((int64_t)correction_counts *
        360000L) / LEVER_POSITION_COUNTS_PER_REVOLUTION);
    int32_t tilt_mdeg = (int32_t)profile->minus_brake_mdeg +
        correction_mdeg;
    if (tilt_mdeg < H3_MINUS_BRAKE_ADAPTIVE_MIN_MDEG)
        return H3_MINUS_BRAKE_ADAPTIVE_MIN_MDEG;
    if (tilt_mdeg > H3_MINUS_BRAKE_ADAPTIVE_MAX_MDEG)
        return H3_MINUS_BRAKE_ADAPTIVE_MAX_MDEG;
    return tilt_mdeg;
}

static bool wheel_stopped(void)
{
    WheelSpeedSnapshot_t wheel;
    uint8_t index;
    WheelSpeed_GetSnapshot(&wheel);
    if (wheel.enabled) return false;
    for (index = 0U; index < WHEEL_SPEED_COUNT; index++) {
        float filtered = wheel.filtered_mm_s[index];
        if (filtered < 0.0f) filtered = -filtered;
        if ((wheel.requested_mm_s[index] > 0.5f) ||
            (wheel.requested_mm_s[index] < -0.5f) ||
            (wheel.ramped_mm_s[index] > 0.5f) ||
            (wheel.ramped_mm_s[index] < -0.5f) ||
            (filtered > 5.0f))
            return false;
    }
    return true;
}

static bool start_conditions(uint32_t now_ms,
                             const BallVisionSnapshot_t *vision,
                             const LeverActuatorSnapshot_t *lever)
{
    AppLineSnapshot_t line;
    AppLine_GetSnapshot(&line);
    return !Safety_HasFault() &&
        (SystemState_Get() == SYSTEM_STATE_IDLE) &&
        (line.state == APP_LINE_IDLE) &&
        wheel_stopped() && vision->valid && vision->fresh &&
        (vision->valid_streak >= 2U) &&
        (abs_i32(vision->x_mm_x10) <=
         (uint32_t)(H3_START_POSITION_LIMIT_MM * 10L)) &&
        lever->boot_homed &&
        (lever->boot_state == LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL) &&
        !lever->moving && (lever->fault_latched == 0U) &&
        ((uint32_t)(now_ms - vision->last_valid_ms) <=
         BALL_VISION_FRESH_TIMEOUT_MS);
}

static void finish_run_state(void)
{
    if (!s_owns_run_state) return;
    M6VisionUart_SetRawPaused(false);
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    s_owns_run_state = false;
}

static void set_state(AppH3State_t state)
{
    if (s_h3.state != state) s_h3.transitions++;
    s_h3.state = state;
}

static void severe_fault(AppH3Fault_t fault)
{
    LeverActuatorSnapshot_t lever;
    LeverActuator_GetSnapshot(&lever);
    if ((lever.fault_latched == 0U) &&
        (lever.ball_active || lever.moving))
        LeverActuator_BallStop(true);
    WheelSpeed_Disable();
    WheelSpeed_SetTarget(0.0f, 0.0f);
    finish_run_state();
    s_h3.fault = fault;
    s_h3.tilt_mdeg = 0;
    s_h3.raw_pd_mdeg = 0;
    s_h3.forced_tilt_mdeg = 0;
    s_h3.launch_withdrawing = false;
    s_stage_launch_withdrawing = false;
    s_stage_launch_pulses = 0U;
    s_stage_reversal_grace = false;
    s_stage_relaunching = false;
    s_plus_topup = false;
    s_plus_topup_available = false;
    s_plus_hard_braking = false;
    s_plus_brake_encoder_reversed = false;
    s_plus_brake_start_counts = 0;
    s_minus_hard_braking = false;
    s_minus_hard_brake_available = false;
    s_minus_hard_brake_pending = false;
    s_minus_hard_brake_negative_speed_seen = false;
    s_minus_hard_brake_tilt_mdeg = 0;
    s_minus_hard_brake_due_ms = UINT32_MAX;
    s_relaunch_ready_since_ms = UINT32_MAX;
    s_h3.active = false;
    set_state(APP_H3_FAULT);
}

static void begin_return_profile(AppH3Fault_t fault, uint16_t return_rpm)
{
    LeverActuatorResult_t result;
    if (s_h3.state == APP_H3_RETURNING) return;
    s_h3.fault = fault;
    s_h3.tilt_mdeg = 0;
    s_h3.raw_pd_mdeg = 0;
    s_h3.forced_tilt_mdeg = 0;
    s_h3.launch_withdrawing = false;
    s_stage_launch_withdrawing = false;
    s_stage_launch_pulses = 0U;
    s_stage_reversal_grace = false;
    s_stage_relaunching = false;
    s_plus_topup = false;
    s_plus_topup_available = false;
    s_plus_hard_braking = false;
    s_plus_brake_encoder_reversed = false;
    s_plus_brake_start_counts = 0;
    s_minus_hard_braking = false;
    s_minus_hard_brake_available = false;
    s_minus_hard_brake_pending = false;
    s_minus_hard_brake_negative_speed_seen = false;
    s_minus_hard_brake_tilt_mdeg = 0;
    s_minus_hard_brake_due_ms = UINT32_MAX;
    s_relaunch_ready_since_ms = UINT32_MAX;
    BallPd_Reset(&s_pd, s_h3.time_ms);
    result = return_rpm == 0U ? LeverActuator_BallReturnHorizontal() :
        LeverActuator_BallReturnHorizontalProfile(return_rpm);
    if (result != LEVER_ACTUATOR_OK) {
        severe_fault(APP_H3_FAULT_ACTUATOR);
        return;
    }
    set_state(APP_H3_RETURNING);
}

static void begin_return(AppH3Fault_t fault)
{
    begin_return_profile(fault, 0U);
}

static void update_ready(uint32_t now_ms,
                         const BallVisionSnapshot_t *vision,
                         const LeverActuatorSnapshot_t *lever)
{
    bool condition = start_conditions(now_ms, vision, lever);
    if (!condition) {
        s_h3.ready_since_ms = UINT32_MAX;
        s_h3.ready = false;
        return;
    }
    if (s_h3.ready_since_ms == UINT32_MAX)
        s_h3.ready_since_ms = now_ms;
    s_h3.ready = (uint32_t)(now_ms - s_h3.ready_since_ms) >=
                 H3_READY_DWELL_MS;
}

static bool position_settled(uint32_t now_ms)
{
    bool in_window =
        abs_i32(s_h3.target_mm_x10 - s_h3.x_mm_x10) <=
            (uint32_t)(H3_SETTLE_POSITION_LIMIT_MM * 10L) &&
        abs_i32(s_h3.vx_mm_s) <= H3_SETTLE_SPEED_LIMIT_MM_S;
    if (!in_window) {
        s_settle_started_ms = UINT32_MAX;
        return false;
    }
    if (s_settle_started_ms == UINT32_MAX)
        s_settle_started_ms = now_ms;
    return (uint32_t)(now_ms - s_settle_started_ms) >=
           H3_SETTLE_DWELL_MS;
}

static void p16(uint8_t *p, int32_t value)
{
    uint16_t v = (uint16_t)(int16_t)value;
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void p32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void publish_debug(uint32_t now_ms,
                          const BallVisionSnapshot_t *vision,
                          const LeverActuatorSnapshot_t *lever)
{
    uint8_t payload[APP_H3_DEBUG_LENGTH];
    uint8_t flags = 0U;
    H3TuneSnapshot_t tune;
    if ((M6Link_Mode() != M6_LINK_V2_ESTABLISHED) ||
        ((M6Link_EnabledCapabilities() & M6_CAP_H3_TUNE_CONTROL) == 0U) ||
        !reached(now_ms, s_next_debug_ms))
        return;
    if ((uint32_t)(now_ms - s_next_debug_ms) >=
        (2U * H3_DEBUG_PERIOD_MS)) {
        s_next_debug_ms = now_ms + H3_DEBUG_PERIOD_MS;
    } else {
        do {
            s_next_debug_ms += H3_DEBUG_PERIOD_MS;
        } while (reached(now_ms, s_next_debug_ms));
    }
    if (vision->valid && vision->fresh) flags |= 1U;
    if (s_h3.ready) flags |= 2U;
    if (lever->moving) flags |= 4U;
    if (lever->ball_pending) flags |= 8U;
    if (s_h3.score_time_met) flags |= 16U;
    if (s_stage_direction_confirmed) flags |= 32U;
    if (s_pd.breakaway_active) flags |= 64U;
    H3Tune_GetSnapshot(&tune);
    if (tune.host_run_active) flags |= 128U;
    memset(payload, 0, sizeof(payload));
    payload[0] = APP_H3_DEBUG_KIND;
    payload[1] = APP_H3_DEBUG_SCHEMA;
    payload[2] = (uint8_t)s_h3.state;
    payload[3] = flags;
    p32(&payload[4], now_ms);
    p32(&payload[8], tune.run_id);
    p32(&payload[12], tune.active_revision);
    p16(&payload[16], vision->x_mm_x10);
    p16(&payload[18], vision->vx_mm_s);
    p16(&payload[20], s_h3.target_mm_x10);
    p16(&payload[22], s_h3.target_mm_x10 - vision->x_mm_x10);
    p16(&payload[24], s_h3.raw_pd_mdeg);
    p16(&payload[26], s_h3.forced_tilt_mdeg);
    p16(&payload[28], s_h3.tilt_mdeg);
    p32(&payload[30], (uint32_t)lever->current_counts);
    p32(&payload[34], (uint32_t)lever->target_counts);
    p32(&payload[38], (uint32_t)s_h3.fault |
                          (lever->fault_latched << 8));
    p16(&payload[42], vision->age_ms > 65535U ?
                        65535 : (int32_t)vision->age_ms);
    payload[44] = vision->seq;
    payload[45] = (uint8_t)((lever->moving ? 1U : 0U) |
                            (lever->ball_pending ? 2U : 0U) |
                            (lever->ball_returning ? 4U : 0U));
    payload[46] = (uint8_t)lever->ball_velocity_direction;
    payload[47] = lever->ball_velocity_speed_rpm > 255U ?
                  255U : (uint8_t)lever->ball_velocity_speed_rpm;
    p16(&payload[48], s_pd.integral_mdeg);
    (void)M6Link_Publish(M6_TX_P3, M6_FRAME_BULK,
                         APP_H3_DEBUG_KIND, payload, sizeof(payload));
}

void AppH3_Init(uint32_t now_ms)
{
    memset(&s_h3, 0, sizeof(s_h3));
    s_h3.time_ms = now_ms;
    s_h3.ready_since_ms = UINT32_MAX;
    s_h3.state = APP_H3_WAIT_READY;
    s_settle_started_ms = UINT32_MAX;
    s_next_debug_ms = now_ms;
    s_stage_start_x_mm_x10 = 0;
    s_stage_started_ms = now_ms;
    s_stage_direction_confirmed = false;
    s_stage_reversal_grace = false;
    s_stage_launch_withdrawing = false;
    s_stage_launch_pulses = 0U;
    s_stage_relaunching = false;
    s_plus_topup = false;
    s_plus_topup_available = false;
    s_plus_hard_braking = false;
    s_plus_brake_encoder_reversed = false;
    s_plus_brake_start_counts = 0;
    s_minus_hard_braking = false;
    s_minus_hard_brake_available = false;
    s_minus_hard_brake_pending = false;
    s_minus_hard_brake_negative_speed_seen = false;
    s_minus_hard_brake_tilt_mdeg = 0;
    s_minus_hard_brake_due_ms = UINT32_MAX;
    s_relaunch_ready_since_ms = UINT32_MAX;
    s_owns_run_state = false;
    s_emergency_requested = false;
    BallPd_Reset(&s_pd, now_ms);
}

static bool start_with_target(uint32_t now_ms, int32_t target_mm_x10,
                              AppH3State_t initial_state)
{
    BallVisionSnapshot_t vision;
    LeverActuatorSnapshot_t lever;
    LeverActuatorResult_t result;
    if (!H3_DEVELOPMENT_ENTRY) return false;
    if (s_h3.active) return false;
    BallVision_GetSnapshot(now_ms, &vision);
    LeverActuator_GetSnapshot(&lever);
    update_ready(now_ms, &vision, &lever);
    if (!s_h3.ready) {
        s_h3.fault = APP_H3_FAULT_NOT_READY;
        return false;
    }
    if (!SystemState_RequestRun()) {
        s_h3.fault = APP_H3_FAULT_START_REJECTED;
        return false;
    }
    s_owns_run_state = true;
    result = LeverActuator_BallArm();
    if (result != LEVER_ACTUATOR_OK) {
        finish_run_state();
        s_h3.fault = APP_H3_FAULT_ACTUATOR;
        return false;
    }
    WheelSpeed_Disable();
    WheelSpeed_SetTarget(0.0f, 0.0f);
    Safety_BeginContinuousAction(now_ms, 0U);
    /* During the scoring action reserve P3 for the fixed 20 Hz schema-2 H3
     * record; UART1 parsing continues and kind=2 resumes after level return. */
    M6VisionUart_SetRawPaused(true);
    BallPd_Reset(&s_pd, now_ms);
    s_h3.started_ms = now_ms;
    s_h3.settled_ms = 0U;
    s_h3.target_mm_x10 = target_mm_x10;
    s_h3.fault = APP_H3_FAULT_NONE;
    s_h3.active = true;
    s_h3.score_time_met = false;
    s_stage_start_x_mm_x10 = vision.x_mm_x10;
    s_stage_started_ms = now_ms;
    s_stage_direction_confirmed = false;
    s_stage_reversal_grace = false;
    s_stage_launch_withdrawing = false;
    s_stage_launch_pulses = 0U;
    s_stage_relaunching = false;
    s_plus_topup = false;
    s_plus_topup_available = initial_state == APP_H3_RUN_PLUS;
    s_plus_hard_braking = false;
    s_plus_brake_encoder_reversed = false;
    s_plus_brake_start_counts = 0;
    s_minus_hard_braking = false;
    s_minus_hard_brake_available = false;
    s_minus_hard_brake_pending = false;
    s_minus_hard_brake_negative_speed_seen = false;
    s_minus_hard_brake_tilt_mdeg = 0;
    s_minus_hard_brake_due_ms = UINT32_MAX;
    s_relaunch_ready_since_ms = UINT32_MAX;
    s_settle_started_ms = UINT32_MAX;
    set_state(initial_state);
    return true;
}

#if !H3_SINGLE_TARGET_ONLY
static void begin_minus_stage(uint32_t now_ms,
                              const BallVisionSnapshot_t *vision)
{
    s_h3.target_mm_x10 = vision->x_mm_x10;
    s_stage_start_x_mm_x10 = vision->x_mm_x10;
    s_stage_started_ms = now_ms;
    s_stage_direction_confirmed = false;
    s_stage_reversal_grace = true;
    s_h3.direction_confirmed = false;
    s_stage_launch_withdrawing = false;
    s_stage_launch_pulses = 0U;
    s_h3.launch_withdrawing = false;
    s_stage_relaunching = false;
    s_plus_topup = false;
    s_plus_topup_available = false;
    s_minus_hard_braking = false;
    s_minus_hard_brake_available = true;
    s_minus_hard_brake_pending = false;
    s_minus_hard_brake_negative_speed_seen = false;
    s_minus_hard_brake_tilt_mdeg = 0;
    s_minus_hard_brake_due_ms = UINT32_MAX;
    s_relaunch_ready_since_ms = UINT32_MAX;
    s_settle_started_ms = UINT32_MAX;
    BallPd_Reset(&s_pd, now_ms);
    set_state(APP_H3_RUN_MINUS);
}

static void update_minus_target(uint32_t now_ms)
{
    uint32_t elapsed_ms = (uint32_t)(now_ms - s_stage_started_ms);
    if (elapsed_ms >= H3_MINUS_TARGET_RAMP_MS) {
        s_h3.target_mm_x10 = -APP_H3_TARGET_X10;
    } else {
        int64_t span = -(int64_t)APP_H3_TARGET_X10 -
                       (int64_t)s_stage_start_x_mm_x10;
        s_h3.target_mm_x10 = s_stage_start_x_mm_x10 +
            (int32_t)((span * elapsed_ms) / H3_MINUS_TARGET_RAMP_MS);
    }
}
#endif

bool AppH3_Start(uint32_t now_ms)
{
    return start_with_target(now_ms, APP_H3_TARGET_X10, APP_H3_RUN_PLUS);
}

bool AppH3_StartCenterHold(uint32_t now_ms)
{
    return start_with_target(now_ms, 0, APP_H3_CENTER_HOLD);
}

void AppH3_Task20ms(uint32_t now_ms)
{
    BallVisionSnapshot_t vision;
    LeverActuatorSnapshot_t lever;
    int32_t predicted_x10;
    int32_t brake_predicted_x10;
    int32_t brake_return_predicted_x10;
    int32_t minimum_tilt;
    int32_t forced_sign;
    int32_t directed_actuator_counts;
    int32_t launch_pulse_mdeg;
    int32_t launch_pulse_counts;
    int32_t relaunch_tilt_mdeg;
    int32_t relaunch_pulse_counts;
    int32_t plus_brake_tilt_mdeg;
    int32_t stage_sign;
    int32_t directed_error_x10;
    int32_t directed_speed;
    uint16_t launch_rpm;
    bool predicted_edge_mitigated;
    bool minus_brake_complete = false;
    bool plus_brake_release_ready;
    const H3TuneProfile_t *profile;
    BallPdConfig_t pd_config;
    LeverActuatorResult_t result;

    if (!H3_DEVELOPMENT_ENTRY) return;

    s_h3.time_ms = now_ms;
    BallVision_GetSnapshot(now_ms, &vision);
    LeverActuator_GetSnapshot(&lever);
    s_h3.x_mm_x10 = vision.x_mm_x10;
    s_h3.vx_mm_s = vision.vx_mm_s;
    s_h3.vision_age_ms = vision.age_ms;
    s_h3.vision_seq = vision.seq;
    s_h3.actuator_counts = lever.current_counts;
    s_h3.actuator_target_counts = lever.target_counts;
    s_h3.actuator_fault = lever.fault_latched;
    s_h3.actuator_moving = lever.moving;
    s_h3.actuator_pending = lever.ball_pending;
    s_h3.direction_confirmed = s_stage_direction_confirmed;
    s_h3.launch_withdrawing = s_stage_launch_withdrawing;

    if (s_emergency_requested) {
        s_emergency_requested = false;
        if (s_h3.active) severe_fault(APP_H3_FAULT_EMERGENCY);
    }
    if (!s_h3.active) {
        update_ready(now_ms, &vision, &lever);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    if (Safety_HasFault()) {
        severe_fault(APP_H3_FAULT_EMERGENCY);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    if (s_h3.state == APP_H3_RETURNING) {
        if (lever.fault_latched != 0U) {
            severe_fault(APP_H3_FAULT_ACTUATOR);
            publish_debug(now_ms, &vision, &lever);
            return;
        }
        if (!lever.ball_active) {
            finish_run_state();
            s_h3.active = false;
            set_state((s_h3.fault == APP_H3_FAULT_USER_STOP ||
                       s_h3.fault == APP_H3_FAULT_NONE) ?
                      APP_H3_WAIT_READY : APP_H3_FAULT);
        }
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    if ((lever.fault_latched != 0U) || !lever.ball_active) {
        severe_fault(APP_H3_FAULT_ACTUATOR);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    if (!vision.valid || !vision.fresh) {
        begin_return(APP_H3_FAULT_VISION);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    predicted_x10 = vision.x_mm_x10 +
        ((int32_t)vision.vx_mm_s * (int32_t)H3_BALL_EDGE_PREDICT_MS) / 100;
    /* The projection is an early-warning guard, not a reason to cancel an
     * already-commanded inward counter-tilt.  r120 saw 140 ms of actuator
     * startup latency after accepting the -8 degree target; returning level
     * during that bounded latency removed the brake and made the edge risk
     * worse.  The absolute 58 mm guard and actuator fault remain authoritative
     * while an active hard-brake command is allowed to continue. */
    predicted_edge_mitigated =
        ((predicted_x10 > 0) && (vision.vx_mm_s > 0) &&
         (s_plus_hard_braking || s_minus_hard_brake_pending)) ||
        ((predicted_x10 < 0) && (vision.vx_mm_s < 0) &&
         s_minus_hard_braking);
    if ((abs_i32(vision.x_mm_x10) >=
          (uint32_t)(H3_BALL_EDGE_LIMIT_MM * 10L)) ||
        (!predicted_edge_mitigated &&
         (abs_i32(predicted_x10) >=
          (uint32_t)(H3_BALL_EDGE_LIMIT_MM * 10L)))) {
        begin_return(APP_H3_FAULT_EDGE);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    if ((uint32_t)(now_ms - s_h3.started_ms) >= H3_RUN_TIMEOUT_MS &&
        (s_h3.state != APP_H3_CENTER_HOLD) &&
        (s_h3.state != APP_H3_HOLD_PLUS) &&
        (s_h3.state != APP_H3_HOLD_MINUS)) {
        begin_return(APP_H3_FAULT_TIMEOUT);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
#if !H3_SINGLE_TARGET_ONLY
    /* H3 asks the ball to pass +50 mm before finishing at -50 mm; it does not
     * require a stationary dwell at the intermediate point.  Switching as
     * soon as the ball enters the allowed +40..+60 mm scoring window leaves
     * enough time for the 100 mm reversal inside the five-second limit. */
    if ((s_h3.state == APP_H3_RUN_PLUS) &&
        (vision.x_mm_x10 >=
         (H3_TARGET_POSITION_MM - H3_PASS_POSITION_TOLERANCE_MM) * 10L)) {
        /* Crossing +40 is the scored positive pass.  If the ball is still
         * fast, arm the same encoder-confirmed -8 degree brake before the
         * RUN_MINUS transition so the first negative command is issued in
         * this control slot rather than after the 200 ms stage guard. */
        if (vision.vx_mm_s >= H3_PLUS_HARD_BRAKE_ARM_SPEED_MM_S) {
            s_plus_hard_braking = true;
            s_plus_brake_encoder_reversed = false;
            s_plus_brake_start_counts = lever.current_counts;
        }
        begin_minus_stage(now_ms, &vision);
    }
    if (s_h3.state == APP_H3_RUN_MINUS)
        update_minus_target(now_ms);
#endif
    if (!s_stage_direction_confirmed &&
        ((s_h3.state == APP_H3_RUN_PLUS) ||
         (s_h3.state == APP_H3_RUN_MINUS)) &&
        ((uint32_t)(now_ms - s_stage_started_ms) >=
         H3_BREAKAWAY_TIMEOUT_MS)) {
        /* A stationary or obstructed ball is a normal unsuccessful attempt,
         * not a controller fault.  Return level and become ready again. */
        begin_return(APP_H3_FAULT_NONE);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    if (!s_stage_direction_confirmed &&
        ((s_h3.state == APP_H3_RUN_PLUS) ||
         (s_h3.state == APP_H3_RUN_MINUS))) {
        int32_t expected_sign = s_h3.state == APP_H3_RUN_PLUS ? 1L : -1L;
        int32_t directed_displacement = expected_sign *
            (vision.x_mm_x10 - s_stage_start_x_mm_x10);
        int32_t directed_speed = expected_sign * vision.vx_mm_s;
        bool reversal_grace = s_stage_reversal_grace &&
            ((uint32_t)(now_ms - s_stage_started_ms) <
             H3_STAGE_REVERSAL_GRACE_MS);
        if (directed_displacement >= H3_DIRECTION_CONFIRM_POSITION_X10) {
            s_stage_direction_confirmed = true;
            s_stage_reversal_grace = false;
            s_h3.direction_confirmed = true;
        } else if (!reversal_grace &&
                   ((directed_displacement <=
                    -H3_DIRECTION_REVERSE_POSITION_X10) ||
                   (directed_speed <=
                    -H3_DIRECTION_REVERSE_SPEED_MM_S))) {
            /* Do not expire this check on wall time.  The steel ball can stay
             * stuck until after the low-tilt guard; r77 therefore missed the
             * first observable reverse motion and ran to the negative edge. */
            begin_return(APP_H3_FAULT_DIRECTION);
            publish_debug(now_ms, &vision, &lever);
            return;
        }
    }

    profile = H3Tune_Active();
    pd_config.kp_mdeg_per_mm = profile->kp_mdeg_per_mm;
    pd_config.kd_mdeg_per_mm_s = profile->kd_mdeg_per_mm_s;
    pd_config.ki_mdeg_per_mm_s = profile->ki_mdeg_per_mm_s;
    pd_config.integral_limit_mdeg = H3_INTEGRAL_LIMIT_MDEG;
    pd_config.normal_slew_mdeg_per_20ms =
        profile->normal_slew_mdeg_per_20ms;
    pd_config.breakaway_slew_mdeg_per_20ms =
        profile->breakaway_slew_mdeg_per_20ms;
    pd_config.brake_slew_mdeg_per_20ms =
        profile->brake_slew_mdeg_per_20ms;
    if ((uint32_t)(now_ms - s_stage_started_ms) < H3_TILT_GUARD_MS) {
        pd_config.positive_limit_mdeg = H3_TILT_GUARD_LIMIT_MDEG;
        pd_config.negative_limit_mdeg = H3_TILT_GUARD_LIMIT_MDEG;
    } else {
        pd_config.positive_limit_mdeg = profile->plus_max_mdeg;
        pd_config.negative_limit_mdeg = profile->minus_max_mdeg;
    }
    minimum_tilt = 0;
    forced_sign = s_h3.state == APP_H3_RUN_PLUS ?
                  H3_DIRECTION_SIGN : -H3_DIRECTION_SIGN;
    stage_sign = s_h3.state == APP_H3_RUN_PLUS ? 1L : -1L;
    launch_pulse_mdeg = H3_LAUNCH_PULSE_FEEDBACK_MDEG +
        (int32_t)s_stage_launch_pulses * H3_LAUNCH_RETRY_STEP_MDEG;
    launch_pulse_counts = (int32_t)(((int64_t)
        launch_pulse_mdeg *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);
    relaunch_tilt_mdeg = s_h3.state == APP_H3_RUN_PLUS ?
        H3_PLUS_RELAUNCH_TILT_MDEG : H3_MINUS_RELAUNCH_TILT_MDEG;
    relaunch_pulse_counts = (int32_t)(((int64_t)
        relaunch_tilt_mdeg *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);
    directed_actuator_counts = forced_sign *
        (lever.current_counts - LEVER_HORIZONTAL_COUNTS);
    if (!s_stage_launch_withdrawing && !s_stage_direction_confirmed &&
        ((s_h3.state == APP_H3_RUN_PLUS) ||
         (s_h3.state == APP_H3_RUN_MINUS)) &&
        ((uint32_t)(now_ms - s_stage_started_ms) >= H3_TILT_GUARD_MS) &&
        (directed_actuator_counts >= launch_pulse_counts)) {
        /* The measured 6 degree pulse already carries enough momentum to
         * break static friction.  Withdraw before waiting for camera motion;
         * otherwise the 18 degree command remains applied throughout the
         * actuator/camera delay and the ball coasts to the edge. */
        s_stage_launch_withdrawing = true;
        if (s_stage_launch_pulses < H3_LAUNCH_MAX_PULSES)
            s_stage_launch_pulses++;
        s_h3.launch_withdrawing = true;
        BallPd_Reset(&s_pd, now_ms);
    }
    if (s_stage_relaunching && !s_stage_launch_withdrawing &&
        (directed_actuator_counts + H3_LEVER_START_DEADBAND_COUNTS >=
         relaunch_pulse_counts)) {
        /* A held relaunch angle can pin the ball on static friction.  Treat
         * relaunch as an encoder-bounded impulse just like initial launch:
         * reach 6.5 degrees, withdraw to level, then reassess camera motion. */
        s_stage_relaunching = false;
        s_stage_launch_withdrawing = true;
        s_h3.launch_withdrawing = true;
        s_relaunch_ready_since_ms = UINT32_MAX;
        BallPd_Reset(&s_pd, now_ms);
    }
    if ((s_h3.state == APP_H3_RUN_PLUS) ||
        (s_h3.state == APP_H3_RUN_MINUS)) {
        directed_error_x10 = stage_sign *
            (s_h3.target_mm_x10 - vision.x_mm_x10);
        directed_speed = stage_sign * vision.vx_mm_s;
        if (s_stage_relaunching) {
            if ((directed_error_x10 <= H3_SETTLE_POSITION_LIMIT_MM * 10L) ||
                (directed_speed >= H3_RELAUNCH_MOVING_SPEED_MM_S)) {
                s_stage_relaunching = false;
                s_relaunch_ready_since_ms = UINT32_MAX;
            }
        } else if (!s_plus_topup && !s_stage_launch_withdrawing &&
                   s_stage_direction_confirmed &&
                   (directed_error_x10 >= H3_RELAUNCH_ERROR_X10) &&
                   (abs_i32(vision.vx_mm_s) <=
                    H3_RELAUNCH_STILL_SPEED_MM_S)) {
            if (s_relaunch_ready_since_ms == UINT32_MAX)
                s_relaunch_ready_since_ms = now_ms;
            else if ((uint32_t)(now_ms - s_relaunch_ready_since_ms) >=
                     H3_RELAUNCH_DWELL_MS) {
                s_stage_relaunching = true;
                if (s_h3.state == APP_H3_RUN_PLUS)
                    s_plus_topup_available = false;
            }
        } else {
            s_relaunch_ready_since_ms = UINT32_MAX;
        }
    }
    if (!s_plus_topup && s_plus_topup_available &&
        (s_h3.state == APP_H3_RUN_PLUS) &&
        s_stage_direction_confirmed &&
        (vision.x_mm_x10 >= H3_PLUS_TOPUP_TRIGGER_X10) &&
        (vision.x_mm_x10 < profile->plus_topup_brake_trigger_x10) &&
        (vision.x_mm_x10 <
         (H3_TARGET_POSITION_MM - H3_PASS_POSITION_TOLERANCE_MM) * 10L) &&
        (vision.vx_mm_s >= 0) &&
        (vision.vx_mm_s <= profile->plus_topup_max_speed_mm_s)) {
        /* The initial encoder-bounded launch pulse can leave the ball below
         * the speed threshold near +15 mm.  Keep this +8 degree recovery
         * command until the camera confirms renewed speed, but cap it at
         * +25 mm below.  Withdrawing merely because the encoder reached 8
         * degrees caused r148 to stall repeatedly at +20..30 mm. */
        s_plus_topup = true;
        s_plus_topup_available = false;
        s_stage_launch_withdrawing = false;
        s_h3.launch_withdrawing = false;
        BallPd_Reset(&s_pd, now_ms);
    }
    if (s_plus_topup && (s_h3.state != APP_H3_RUN_PLUS))
        s_plus_topup = false;
    launch_rpm = s_h3.state == APP_H3_RUN_PLUS ?
                 profile->plus_launch_rpm : profile->minus_launch_rpm;
    if (s_plus_topup) {
        minimum_tilt = profile->plus_topup_mdeg;
    } else if (s_stage_relaunching) {
        minimum_tilt = relaunch_tilt_mdeg;
    } else if (!s_stage_direction_confirmed && !s_stage_reversal_grace &&
        !s_stage_launch_withdrawing &&
        ((s_h3.state == APP_H3_RUN_PLUS) ||
         (s_h3.state == APP_H3_RUN_MINUS)) &&
        ((uint32_t)(now_ms - s_stage_started_ms) >= H3_TILT_GUARD_MS)) {
        uint32_t ramp_ms = (uint32_t)(now_ms - s_stage_started_ms) -
                           H3_TILT_GUARD_MS;
        uint16_t start_mdeg = s_h3.state == APP_H3_RUN_PLUS ?
            profile->plus_start_mdeg : profile->minus_start_mdeg;
        uint16_t max_mdeg = s_h3.state == APP_H3_RUN_PLUS ?
            profile->plus_max_mdeg : profile->minus_max_mdeg;
        int64_t ramp = start_mdeg +
            ((int64_t)profile->breakaway_slew_mdeg_per_20ms * ramp_ms) /
            20L;
        minimum_tilt = ramp > max_mdeg ? max_mdeg : (int32_t)ramp;
    }
    s_h3.forced_tilt_mdeg = minimum_tilt * forced_sign;
    if (s_stage_launch_withdrawing) {
        int32_t probe = BallPd_Update(&s_pd, s_h3.target_mm_x10,
            vision.x_mm_x10, vision.vx_mm_s, 0, now_ms, &pd_config);
        int32_t raw_probe = s_pd.raw_requested_mdeg;
        bool level_and_slow =
            (abs_i32(lever.current_counts - LEVER_HORIZONTAL_COUNTS) <=
             H3_LAUNCH_REARM_LEVEL_COUNTS) &&
            (abs_i32(vision.vx_mm_s) <= H3_LAUNCH_REARM_SPEED_MM_S);
        bool retry_launch = !s_stage_direction_confirmed && level_and_slow &&
            (s_stage_launch_pulses < H3_LAUNCH_MAX_PULSES);
        bool safely_rearmed = s_stage_direction_confirmed && level_and_slow;
        if (retry_launch) {
            /* Static friction varies with battery and ball placement.  If the
             * first bounded pulse produced no camera motion, re-arm exactly
             * one pulse one degree higher instead of waiting motionless until
             * the no-roll timeout. */
            s_stage_launch_withdrawing = false;
            s_h3.launch_withdrawing = false;
            BallPd_Reset(&s_pd, now_ms);
            s_h3.raw_pd_mdeg = raw_probe;
            s_h3.tilt_mdeg = 0;
        } else if (!s_stage_direction_confirmed ||
            (((int64_t)forced_sign * raw_probe >= 0L) &&
             !safely_rearmed)) {
            /* Hold a zero command while PD would still accelerate outward.
             * Resetting its slew state keeps the eventual counter-tilt based
             * on the command actually sent to the actuator, not the rejected
             * outward probe. */
            (void)probe;
            BallPd_Reset(&s_pd, now_ms);
            s_h3.raw_pd_mdeg = raw_probe;
            s_h3.tilt_mdeg = 0;
        } else {
            s_stage_launch_withdrawing = false;
            s_h3.launch_withdrawing = false;
            BallPd_Reset(&s_pd, now_ms);
            s_h3.tilt_mdeg = BallPd_Update(&s_pd,
                s_h3.target_mm_x10, vision.x_mm_x10, vision.vx_mm_s,
                0, now_ms, &pd_config);
            s_h3.raw_pd_mdeg = s_pd.raw_requested_mdeg;
        }
    } else {
        s_h3.tilt_mdeg = BallPd_Update(&s_pd, s_h3.target_mm_x10,
            vision.x_mm_x10, vision.vx_mm_s, minimum_tilt, now_ms,
            &pd_config);
        s_h3.raw_pd_mdeg = s_pd.raw_requested_mdeg;
    }
    if (!s_plus_hard_braking &&
        (s_h3.state == APP_H3_RUN_PLUS) &&
        s_stage_direction_confirmed) {
        brake_predicted_x10 = vision.x_mm_x10 +
            ((int32_t)vision.vx_mm_s *
             (int32_t)H3_PLUS_HARD_BRAKE_PREDICT_MS) / 100;
        if (((vision.vx_mm_s >= H3_PLUS_HARD_BRAKE_ARM_SPEED_MM_S) &&
             (brake_predicted_x10 >= profile->plus_brake_trigger_x10)) ||
            (s_plus_topup &&
             ((vision.vx_mm_s >= H3_PLUS_TOPUP_RECOVERY_SPEED_MM_S) ||
              (vision.x_mm_x10 >=
               profile->plus_topup_brake_trigger_x10)))) {
            s_plus_hard_braking = true;
            s_plus_brake_encoder_reversed = false;
            s_plus_brake_start_counts = lever.current_counts;
            /* The one-shot +8 degree top-up must end permanently once the
             * predictive -8 degree brake is accepted.  r131/r134 retained
             * the top-up latch, so releasing the brake at 20 mm/s restored
             * +8 degrees and accelerated the ball into the positive edge. */
            s_plus_topup = false;
            s_plus_topup_available = false;
            /* A launch pulse may still be in its horizontal-withdrawal
             * substate while the ball is already moving quickly.  The
             * predictive counter-tilt must take priority: otherwise the
             * telemetry advertises a negative brake but the actuator keeps
             * receiving BallWithdrawHorizontal() and can only target 0 deg. */
            s_stage_launch_withdrawing = false;
            s_h3.launch_withdrawing = false;
        }
    }
    if (s_plus_hard_braking &&
        ((lever.ball_velocity_direction < 0) ||
         (s_h3.state == APP_H3_RUN_MINUS)) &&
        (lever.current_counts <= s_plus_brake_start_counts - 8L))
        s_plus_brake_encoder_reversed = true;
    plus_brake_release_ready =
        (s_h3.state == APP_H3_RUN_MINUS) ?
        ((uint32_t)(now_ms - s_stage_started_ms) >=
         H3_PLUS_TO_MINUS_BRAKE_HOLD_MS) :
        (vision.vx_mm_s <= H3_PLUS_HARD_BRAKE_RELEASE_MM_S);
    if (s_plus_hard_braking && s_plus_brake_encoder_reversed &&
        plus_brake_release_ready) {
        s_plus_hard_braking = false;
        s_plus_brake_encoder_reversed = false;
        if ((s_h3.state == APP_H3_RUN_MINUS) &&
            s_minus_hard_brake_available) {
            /* Identical hardware runs varied from 160 to 461 ms before the
             * velocity release.  RUN_MINUS now holds the pass brake for the
             * measured successful 300 ms instead.  Preserve the calibrated
             * 50 ms interval, then pre-command the one-shot negative brake. */
            s_minus_hard_brake_pending = true;
            s_minus_hard_brake_due_ms =
                now_ms + H3_PLUS_TO_MINUS_BRAKE_DELAY_MS;
        }
    }
    if ((s_h3.state != APP_H3_RUN_PLUS) &&
        (s_h3.state != APP_H3_RUN_MINUS)) {
        s_plus_hard_braking = false;
        s_plus_brake_encoder_reversed = false;
    }

    if (s_h3.state == APP_H3_RUN_MINUS) {
        brake_predicted_x10 = vision.x_mm_x10 +
            ((int32_t)vision.vx_mm_s *
             (int32_t)H3_MINUS_HARD_BRAKE_PREDICT_MS) / 100;
        if (s_minus_hard_brake_pending &&
            s_minus_hard_brake_available &&
            reached(now_ms, s_minus_hard_brake_due_ms)) {
            s_minus_hard_braking = true;
            s_minus_hard_brake_available = false;
            s_minus_hard_brake_pending = false;
            s_minus_hard_brake_negative_speed_seen = false;
            s_minus_hard_brake_tilt_mdeg =
                adaptive_minus_brake_tilt_mdeg(profile,
                                               lever.current_counts);
        } else if (!s_plus_hard_braking &&
            !s_minus_hard_brake_pending &&
            s_stage_direction_confirmed &&
            s_minus_hard_brake_available && !s_minus_hard_braking &&
            (vision.vx_mm_s <= -H3_MINUS_HARD_BRAKE_ARM_SPEED_MM_S) &&
            (brake_predicted_x10 <= profile->minus_brake_trigger_x10)) {
            s_minus_hard_braking = true;
            s_minus_hard_brake_available = false;
            s_minus_hard_brake_tilt_mdeg =
                adaptive_minus_brake_tilt_mdeg(profile,
                                               lever.current_counts);
            s_minus_hard_brake_negative_speed_seen =
                vision.vx_mm_s <= -H3_MINUS_HARD_BRAKE_RELEASE_MM_S;
        } else if (s_minus_hard_braking) {
            if (vision.vx_mm_s <= -H3_MINUS_HARD_BRAKE_RELEASE_MM_S)
                s_minus_hard_brake_negative_speed_seen = true;
            brake_return_predicted_x10 = vision.x_mm_x10 +
                ((int32_t)vision.vx_mm_s *
                 (int32_t)profile->minus_brake_return_predict_ms) / 100;
            if (s_minus_hard_brake_negative_speed_seen &&
                ((brake_return_predicted_x10 <= -APP_H3_TARGET_X10) ||
                 (vision.vx_mm_s >= 0)))
                minus_brake_complete = true;
        }
    } else {
        s_minus_hard_braking = false;
        s_minus_hard_brake_available = false;
        s_minus_hard_brake_pending = false;
        s_minus_hard_brake_negative_speed_seen = false;
        s_minus_hard_brake_tilt_mdeg = 0;
        s_minus_hard_brake_due_ms = UINT32_MAX;
    }
    if (minus_brake_complete) {
        /* r129 measured about 300 ms from a horizontal-return request until
         * the +5 degree counter-tilt reached level.  r131 then proved that
         * waiting for the literal -50 mm crossing can never release when the
         * physical ball reverses at -46.1 mm while filtered speed is stale.
         * The short position/velocity projection above begins that mechanical
         * return before the hidden reversal.  If the stronger counter-tilt
         * reverses early, the non-negative-speed fallback above releases it
         * immediately instead of accelerating the ball back to the positive
         * edge.  There is deliberately no RUN_MINUS PID phase after this
         * fixed brake. */
        s_h3.score_time_met =
            (uint32_t)(now_ms - s_h3.started_ms) <= H3_SCORE_TIME_LIMIT_MS;
        begin_return_profile(APP_H3_FAULT_NONE,
                             profile->minus_return_rpm);
        publish_debug(now_ms, &vision, &lever);
        return;
    }
    if (s_plus_hard_braking) {
        /* Actuator response varied from 99 to 298 ms on identical hardware
         * runs.  A full -8 degree pre-pass command stopped the fast-response
         * case at +36.5 mm, while delaying all braking risks the edge in the
         * slow-response case.  Use -4 degrees only before the scored +40 mm
         * crossing, then restore the original -8 degree brake in the same
         * control slot that enters RUN_MINUS. */
        plus_brake_tilt_mdeg = profile->plus_brake_mdeg;
        if ((s_h3.state == APP_H3_RUN_PLUS) &&
            (vision.x_mm_x10 <
             (H3_TARGET_POSITION_MM - H3_PASS_POSITION_TOLERANCE_MM) * 10L) &&
            (plus_brake_tilt_mdeg > H3_PLUS_PREPASS_BRAKE_TILT_MDEG))
            plus_brake_tilt_mdeg = H3_PLUS_PREPASS_BRAKE_TILT_MDEG;
        s_h3.forced_tilt_mdeg =
            -plus_brake_tilt_mdeg * H3_DIRECTION_SIGN;
        s_h3.tilt_mdeg = s_h3.forced_tilt_mdeg;
    } else if (s_minus_hard_braking) {
        /* The actuator needs roughly half a second to reverse from the
         * negative launch angle.  Hardware r119 showed that waiting for a
         * -10 mm projection did not reach +8 degrees until the ball was at
         * -47 mm and -140 mm/s.  Start at the +20 mm projection so the same
         * deterministic counter-tilt has time to act. */
        s_h3.forced_tilt_mdeg =
            s_minus_hard_brake_tilt_mdeg * H3_DIRECTION_SIGN;
        s_h3.tilt_mdeg = s_h3.forced_tilt_mdeg;
    }
    if (s_stage_launch_withdrawing && !s_plus_hard_braking &&
        !s_minus_hard_braking) {
        result = LeverActuator_BallWithdrawHorizontal();
    } else {
        result = LeverActuator_BallSetTiltProfile(
            s_h3.tilt_mdeg, (minimum_tilt > 0) || s_plus_hard_braking ||
            s_minus_hard_braking,
            profile->tracking_rpm,
            s_stage_relaunching ? profile->tracking_rpm : launch_rpm);
    }
    if (result != LEVER_ACTUATOR_OK) {
        severe_fault(APP_H3_FAULT_ACTUATOR);
        publish_debug(now_ms, &vision, &lever);
        return;
    }

#if H3_SINGLE_TARGET_ONLY
    if (((s_h3.state == APP_H3_RUN_PLUS) ||
         (s_h3.state == APP_H3_RUN_MINUS)) &&
        position_settled(now_ms)) {
#else
    if ((s_h3.state == APP_H3_RUN_MINUS) &&
        (s_h3.target_mm_x10 == -APP_H3_TARGET_X10) &&
        position_settled(now_ms)) {
#endif
        s_h3.settled_ms = now_ms;
        if (s_h3.state == APP_H3_RUN_PLUS) {
#if H3_SINGLE_TARGET_ONLY
            s_h3.score_time_met =
                (uint32_t)(now_ms - s_h3.started_ms) <=
                H3_SCORE_TIME_LIMIT_MS;
            set_state(APP_H3_HOLD_PLUS);
#else
            begin_minus_stage(now_ms, &vision);
#endif
        } else if (s_h3.state == APP_H3_RUN_MINUS) {
            s_h3.score_time_met =
                (uint32_t)(now_ms - s_h3.started_ms) <=
                H3_SCORE_TIME_LIMIT_MS;
            set_state(APP_H3_HOLD_MINUS);
        }
    }
    publish_debug(now_ms, &vision, &lever);
}

void AppH3_RequestStop(uint32_t now_ms)
{
    s_h3.time_ms = now_ms;
    if (s_h3.active) begin_return(APP_H3_FAULT_USER_STOP);
}

void AppH3_EmergencyStop(void)
{
    s_emergency_requested = true;
}

bool AppH3_IsActive(void)
{
    return s_h3.active;
}

bool AppH3_IsDevelopmentDisplay(void)
{
    return H3_DEVELOPMENT_ENTRY != 0U;
}

static const char *state_text(AppH3State_t state)
{
    switch (state) {
        case APP_H3_RUN_PLUS: return "+50 RUN";
        case APP_H3_HOLD_PLUS: return "+50 HOLD";
        case APP_H3_RUN_MINUS: return "-50 RUN";
        case APP_H3_HOLD_MINUS: return "-50 HOLD";
        case APP_H3_RETURNING: return "RETURN";
        case APP_H3_FAULT: return "FAULT";
        case APP_H3_CENTER_HOLD: return "CENTER 0";
        default: return s_h3.ready ? "READY K1" : "WAIT BALL";
    }
}

void AppH3_Task100ms(uint32_t now_ms)
{
    (void)now_ms;
    /* The OLED driver flushes four bytes per background service call.  A new
     * request every 100 ms used to restart at page zero forever, leaving the
     * lower diagnostic rows stale.  Let the current full-frame transfer
     * finish before replacing its buffer. */
    if (!H3_DEVELOPMENT_ENTRY || !OLED_IsPresent() ||
        OLED_ServicePending()) return;
    OLED_Clear();
    OLED_ShowString(0U, 0U, "H3 ");
    OLED_ShowString(18U, 0U, state_text(s_h3.state));
    OLED_ShowString(0U, 2U, "X");
    OLED_ShowNum(8U, 2U, s_h3.x_mm_x10, 4U);
    OLED_ShowString(38U, 2U, "V");
    OLED_ShowNum(46U, 2U, s_h3.vx_mm_s, 4U);
    OLED_ShowString(76U, 2U, "T");
    OLED_ShowNum(84U, 2U, s_h3.target_mm_x10, 4U);
    OLED_ShowString(0U, 4U, "A");
    OLED_ShowNum(8U, 4U, s_h3.tilt_mdeg, 4U);
    OLED_ShowString(38U, 4U, "C");
    OLED_ShowNum(46U, 4U, s_h3.actuator_counts, 5U);
    OLED_ShowString(82U, 4U, "G");
    OLED_ShowNum(90U, 4U, s_h3.actuator_target_counts, 5U);
    OLED_ShowString(0U, 6U, "M");
    OLED_ShowNum(8U, 6U, s_h3.actuator_moving ? 1 : 0, 1U);
    OLED_ShowString(18U, 6U, "P");
    OLED_ShowNum(26U, 6U, s_h3.actuator_pending ? 1 : 0, 1U);
    OLED_ShowString(36U, 6U, "F");
    OLED_ShowNum(44U, 6U,
                 (int32_t)s_h3.fault +
                 (int32_t)(s_h3.actuator_fault * 100U), 4U);
    OLED_RequestRefresh();
}

void AppH3_GetSnapshot(AppH3Snapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = s_h3;
}
