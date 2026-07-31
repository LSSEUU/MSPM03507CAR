#include "app_h3.h"
#include "h3_control_config.h"
#include "h3_tune.h"
#include "lever_motion_config.h"

#include "app_line.h"
#include "ball_vision.h"
#include "lever_actuator.h"
#include "m6_link.h"
#include "m6_vision_uart.h"
#include "oled.h"
#include "safety.h"
#include "system_state.h"
#include "wheel_speed.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static BallVisionSnapshot_t vision;
static LeverActuatorSnapshot_t lever;
static WheelSpeedSnapshot_t wheel;
static AppLineSnapshot_t line;
static SystemState_t system_state;
static bool safety_fault;
static uint32_t arm_calls;
static uint32_t tilt_calls;
static uint32_t return_calls;
static uint32_t disable_calls;
static uint32_t begin_calls;
static uint32_t end_calls;
static bool raw_paused;
static bool monitor_enabled;
static M6LinkMode_t link_mode;
static uint64_t enabled_capabilities;
static uint32_t publish_calls;
static int32_t last_tilt;
static bool last_breakaway_fast;
static uint16_t last_breakaway_rpm;
static uint32_t withdraw_calls;
static H3TuneProfile_t tune_profile;

const H3TuneProfile_t *H3Tune_Active(void) { return &tune_profile; }
void H3Tune_GetSnapshot(H3TuneSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->active_revision = 1U;
}

void BallVision_GetSnapshot(uint32_t now_ms, BallVisionSnapshot_t *snapshot)
{
    vision.time_ms = now_ms;
    vision.age_ms = now_ms - vision.last_valid_ms;
    if (vision.age_ms > BALL_VISION_FRESH_TIMEOUT_MS) vision.fresh = false;
    *snapshot = vision;
}
void LeverActuator_GetSnapshot(LeverActuatorSnapshot_t *snapshot)
{ *snapshot = lever; }
LeverActuatorResult_t LeverActuator_BallArm(void)
{
    arm_calls++;
    lever.ball_active = true;
    return LEVER_ACTUATOR_OK;
}
LeverActuatorResult_t LeverActuator_BallSetTilt(int32_t tilt)
{
    return LeverActuator_BallSetTiltProfile(tilt, false, 1U, 5U);
}
LeverActuatorResult_t LeverActuator_BallSetTiltProfile(
    int32_t tilt, bool breakaway_fast,
    uint16_t tracking_rpm, uint16_t breakaway_rpm)
{
    (void)tracking_rpm;
    tilt_calls++;
    last_tilt = tilt;
    last_breakaway_fast = breakaway_fast;
    last_breakaway_rpm = breakaway_rpm;
    lever.requested_tilt_mdeg = tilt;
    lever.ball_pending = true;
    return LEVER_ACTUATOR_OK;
}
LeverActuatorResult_t LeverActuator_BallReturnHorizontal(void)
{
    return_calls++;
    lever.ball_returning = true;
    return LEVER_ACTUATOR_OK;
}

LeverActuatorResult_t LeverActuator_BallReturnHorizontalProfile(
    uint16_t return_rpm)
{
    (void)return_rpm;
    return LeverActuator_BallReturnHorizontal();
}
LeverActuatorResult_t LeverActuator_BallWithdrawHorizontal(void)
{
    withdraw_calls++;
    last_tilt = 0;
    last_breakaway_fast = false;
    lever.requested_tilt_mdeg = 0;
    lever.ball_pending = true;
    return LEVER_ACTUATOR_OK;
}
void LeverActuator_BallStop(bool disable_after_stop)
{
    if (disable_after_stop) disable_calls++;
    lever.ball_active = false;
}
void AppLine_GetSnapshot(AppLineSnapshot_t *snapshot) { *snapshot = line; }
void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot)
{ *snapshot = wheel; }
void WheelSpeed_Disable(void) { wheel.enabled = false; }
void WheelSpeed_SetTarget(float left, float right)
{
    wheel.requested_mm_s[0] = left;
    wheel.requested_mm_s[1] = right;
}
bool Safety_HasFault(void) { return safety_fault; }
void Safety_BeginContinuousAction(uint32_t now_ms, uint32_t mask)
{ (void)now_ms; (void)mask; begin_calls++; }
void Safety_EndAction(void) { end_calls++; }
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
bool M6VisionUart_IsEnabled(void) { return monitor_enabled; }
void M6VisionUart_SetRawPaused(bool paused) { raw_paused = paused; }
M6LinkMode_t M6Link_Mode(void) { return link_mode; }
uint64_t M6Link_EnabledCapabilities(void) { return enabled_capabilities; }
bool M6Link_Publish(M6TxPriority_t priority, uint8_t type, uint16_t topic,
                    const uint8_t *payload, uint16_t length)
{
    (void)priority; (void)type; (void)topic; (void)payload; (void)length;
    publish_calls++;
    return true;
}
bool OLED_IsPresent(void) { return false; }
bool OLED_ServicePending(void) { return false; }
void OLED_Clear(void) {}
void OLED_ShowString(uint8_t x, uint8_t y, const char *s)
{ (void)x; (void)y; (void)s; }
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t n, uint8_t len)
{ (void)x; (void)y; (void)n; (void)len; }
void OLED_RequestRefresh(void) {}

static void reset_fixture(uint32_t now_ms)
{
    memset(&vision, 0, sizeof(vision));
    memset(&lever, 0, sizeof(lever));
    memset(&wheel, 0, sizeof(wheel));
    memset(&line, 0, sizeof(line));
    tune_profile.plus_start_mdeg = H3_BREAKAWAY_START_MDEG;
    tune_profile.plus_max_mdeg = H3_PLUS_BREAKAWAY_MAX_MDEG;
    tune_profile.minus_start_mdeg = H3_BREAKAWAY_START_MDEG;
    tune_profile.minus_max_mdeg = H3_BREAKAWAY_MAX_MDEG;
    tune_profile.kp_mdeg_per_mm = H3_CONTROL_KP_MDEG_PER_MM;
    tune_profile.kd_mdeg_per_mm_s = H3_CONTROL_KD_MDEG_PER_MM_S;
    tune_profile.ki_mdeg_per_mm_s = H3_CONTROL_KI_MDEG_PER_MM_S;
    tune_profile.plus_topup_mdeg = H3_PLUS_TOPUP_TILT_MDEG;
    tune_profile.plus_brake_mdeg = H3_PLUS_HARD_BRAKE_TILT_MDEG;
    tune_profile.minus_brake_trigger_x10 = H3_MINUS_HARD_BRAKE_TRIGGER_X10;
    tune_profile.minus_brake_mdeg = H3_MINUS_HARD_BRAKE_TILT_MDEG;
    tune_profile.plus_brake_trigger_x10 = H3_PLUS_HARD_BRAKE_TRIGGER_X10;
    tune_profile.plus_topup_brake_trigger_x10 =
        H3_PLUS_TOPUP_BRAKE_TRIGGER_X10;
    tune_profile.minus_brake_return_predict_ms =
        H3_MINUS_BRAKE_RETURN_PREDICT_MS;
    tune_profile.minus_return_rpm = H3_LEVER_RETURN_VELOCITY_RPM;
    tune_profile.plus_topup_max_speed_mm_s =
        H3_PLUS_TOPUP_MAX_SPEED_MM_S;
    tune_profile.normal_slew_mdeg_per_20ms =
        H3_TILT_SLEW_MDEG_PER_20MS;
    tune_profile.breakaway_slew_mdeg_per_20ms =
        H3_BREAKAWAY_TILT_SLEW_MDEG_PER_20MS;
    tune_profile.brake_slew_mdeg_per_20ms =
        H3_BRAKE_TILT_SLEW_MDEG_PER_20MS;
    tune_profile.tracking_rpm = H3_LEVER_VELOCITY_RPM;
    tune_profile.plus_launch_rpm = H3_PLUS_LEVER_BREAKAWAY_VELOCITY_RPM;
    tune_profile.minus_launch_rpm = H3_LEVER_BREAKAWAY_VELOCITY_RPM;
    system_state = SYSTEM_STATE_IDLE;
    line.state = APP_LINE_IDLE;
    lever.boot_homed = true;
    lever.boot_state = LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL;
    lever.current_counts = 4396;
    lever.target_counts = 4396;
    vision.valid = true;
    vision.fresh = true;
    vision.valid_streak = 2U;
    vision.last_valid_ms = now_ms;
    safety_fault = false;
    arm_calls = tilt_calls = return_calls = disable_calls = 0U;
    begin_calls = end_calls = 0U;
    raw_paused = false;
    monitor_enabled = false;
    link_mode = M6_LINK_V2_ESTABLISHED;
    enabled_capabilities = UINT64_C(1) << 31;
    publish_calls = 0U;
    last_tilt = 0;
    last_breakaway_fast = false;
    last_breakaway_rpm = 0U;
    withdraw_calls = 0U;
    AppH3_Init(now_ms);
}

static void refresh_vision(uint32_t now_ms, int16_t x, int16_t vx)
{
    vision.valid = true;
    vision.fresh = true;
    vision.x_mm_x10 = x;
    vision.vx_mm_s = vx;
    vision.last_valid_ms = now_ms;
    vision.seq++;
}

#if !H3_SINGLE_TARGET_ONLY
static int32_t counts_for_mdeg(int32_t mdeg)
{
    return (int32_t)(((int64_t)mdeg *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);
}

static int32_t minus_brake_reference_counts(void)
{
    return LEVER_HORIZONTAL_COUNTS - H3_DIRECTION_SIGN *
        counts_for_mdeg(H3_MINUS_BRAKE_ENCODER_REFERENCE_MDEG);
}
#endif

static void make_ready_and_start(uint32_t base_ms)
{
    AppH3Snapshot_t snapshot;
    refresh_vision(base_ms, 0, 0);
    AppH3_Task20ms(base_ms);
    refresh_vision(base_ms + 300U, 0, 0);
    AppH3_Task20ms(base_ms + 300U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.ready);
    assert(AppH3_Start(base_ms + 300U));
    assert(arm_calls == 1U && begin_calls == 1U);
    assert(raw_paused);
    assert(system_state == SYSTEM_STATE_RUN);
}

static void test_plus_control_and_direction_guard(void)
{
    AppH3Snapshot_t snapshot;
    reset_fixture(1000U);
    make_ready_and_start(1000U);
    refresh_vision(1320U, 0, 0);
    AppH3_Task20ms(1320U);
    assert(tilt_calls == 1U);
    assert(last_tilt * H3_DIRECTION_SIGN > 0 &&
           last_tilt * H3_DIRECTION_SIGN <= 600);

    refresh_vision(1340U, -50, -30);
    AppH3_Task20ms(1340U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_DIRECTION);
    assert(return_calls == 1U && disable_calls == 0U);

    lever.ball_active = false;
    lever.ball_returning = false;
    AppH3_Task20ms(1360U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.active && snapshot.state == APP_H3_FAULT);
    assert(system_state == SYSTEM_STATE_IDLE && end_calls == 1U);

    /* Keep checking direction after both the 400 ms low-tilt guard and the
     * old 1.5 s wall-time window.  Direction is not confirmed until the ball
     * has actually displaced in the requested direction. */
    reset_fixture(5000U);
    make_ready_and_start(5000U);
    refresh_vision(7000U, -50, 0);
    AppH3_Task20ms(7000U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_DIRECTION);
}

static void test_plus_passes_through_and_minus_finishes_fixed_brake(void)
{
    AppH3Snapshot_t snapshot;
    reset_fixture(2000U);
    make_ready_and_start(2000U);
    refresh_vision(2320U, 500, 0);
    AppH3_Task20ms(2320U);
    refresh_vision(2580U, 500, 0);
    AppH3_Task20ms(2580U);
    AppH3_GetSnapshot(&snapshot);
#if H3_SINGLE_TARGET_ONLY
    assert(snapshot.state == APP_H3_HOLD_PLUS);
    assert(snapshot.active && snapshot.score_time_met);
    assert(snapshot.target_mm_x10 == 500);
#else
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.target_mm_x10 < 500 && snapshot.target_mm_x10 > -500);
    lever.current_counts = minus_brake_reference_counts();
    refresh_vision(2600U, 400, -10);
    AppH3_Task20ms(2600U);
    assert(last_tilt ==
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    refresh_vision(3820U, -500, 0);
    AppH3_Task20ms(3820U);
    refresh_vision(4080U, -500, 0);
    AppH3_Task20ms(4080U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.active && snapshot.score_time_met);
#endif
}

static void test_fast_plus_window_switch_allows_bounded_reversal_grace(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(3000U);
    make_ready_and_start(3000U);

    /* Passing +40 mm while still moving outward is a valid intermediate
     * score, not a request to dwell at +50 mm. */
    refresh_vision(3320U, 405, 35);
    AppH3_Task20ms(3320U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.target_mm_x10 == 405);
    assert(snapshot.fault == APP_H3_FAULT_NONE);

    /* Positive inertia during the short reversal window must not be
     * misclassified as a negative-stage direction fault. */
    refresh_vision(3500U, 430, 20);
    AppH3_Task20ms(3500U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.fault == APP_H3_FAULT_NONE);

    refresh_vision(3700U, 390, -20);
    AppH3_Task20ms(3700U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.direction_confirmed);
#endif
}

static void test_invalid_vision_returns_horizontal(void)
{
    AppH3Snapshot_t snapshot;
    reset_fixture(3000U);
    make_ready_and_start(3000U);
    vision.valid = false;
    vision.fresh = false;
    AppH3_Task20ms(3320U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_VISION);
    assert(return_calls == 1U);
}

static void test_edge_and_actuator_faults_stop_new_targets(void)
{
    AppH3Snapshot_t snapshot;
    reset_fixture(4000U);
    make_ready_and_start(4000U);
    refresh_vision(4320U, 580, 0);
    AppH3_Task20ms(4320U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_EDGE);
    assert(tilt_calls == 0U && return_calls == 1U);

    reset_fixture(4500U);
    make_ready_and_start(4500U);
    refresh_vision(4820U, 500, 50);
    AppH3_Task20ms(4820U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_EDGE);

    reset_fixture(5000U);
    make_ready_and_start(5000U);
    lever.fault_latched = LEVER_ACTUATOR_FEEDBACK_STALE;
    lever.ball_active = false;
    refresh_vision(5320U, 0, 0);
    AppH3_Task20ms(5320U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_FAULT && !snapshot.active);
    assert(snapshot.fault == APP_H3_FAULT_ACTUATOR);
    assert(disable_calls == 0U);
    assert(system_state == SYSTEM_STATE_IDLE);
}

static void test_commanded_brake_survives_predicted_edge(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(4700U);
    make_ready_and_start(4700U);
    refresh_vision(5020U, 20, 5);
    AppH3_Task20ms(5020U);

    /* Arm the positive hard brake before the pass.  +49.3 mm at +57 mm/s
     * projects beyond +58 mm; a bounded actuator startup delay must not cancel
     * the accepted counter-tilt target before the absolute edge guard. */
    refresh_vision(5200U, 380, 44);
    AppH3_Task20ms(5200U);
    refresh_vision(5220U, 493, 57);
    AppH3_Task20ms(5220U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.active);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.fault == APP_H3_FAULT_NONE);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);

    /* Actual position remains the hard authority even while braking. */
    refresh_vision(5240U, H3_BALL_EDGE_LIMIT_MM * 10L, 0);
    AppH3_Task20ms(5240U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_EDGE);
#endif
}

static void test_debug_cadence_averages_20hz(void)
{
    uint32_t now_ms;
    reset_fixture(6000U);
    /* H3 schema-3 telemetry is independent from raw UART mirroring. */
    assert(!monitor_enabled);
    for (now_ms = 6000U; now_ms <= 6200U; now_ms += 20U) {
        refresh_vision(now_ms, 0, 0);
        AppH3_Task20ms(now_ms);
    }
    assert(publish_calls == 5U);

    reset_fixture(7000U);
    link_mode = M6_LINK_V1;
    for (now_ms = 7000U; now_ms <= 7200U; now_ms += 20U)
        AppH3_Task20ms(now_ms);
    assert(publish_calls == 0U);

    reset_fixture(8000U);
    enabled_capabilities = 0U;
    for (now_ms = 8000U; now_ms <= 8200U; now_ms += 20U)
        AppH3_Task20ms(now_ms);
    assert(publish_calls == 0U);
}

static void test_minus_stage_direction_guard(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(7000U);
    make_ready_and_start(7000U);
    refresh_vision(7320U, 500, 0);
    AppH3_Task20ms(7320U);
    refresh_vision(7580U, 500, 0);
    AppH3_Task20ms(7580U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);

    refresh_vision(8840U, 530, 0);
    AppH3_Task20ms(8840U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_DIRECTION);
#endif
}

static void test_no_roll_automatically_ramps_and_returns_without_fault(void)
{
    AppH3Snapshot_t snapshot;
    uint32_t now_ms;

    reset_fixture(8000U);
    make_ready_and_start(8000U);
    for (now_ms = 8320U; now_ms < 8300U + H3_BREAKAWAY_TIMEOUT_MS;
         now_ms += 20U) {
        refresh_vision(now_ms, 0, 0);
        AppH3_Task20ms(now_ms);
    }
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_PLUS);
    assert(snapshot.fault == APP_H3_FAULT_NONE);
    assert(last_tilt == H3_PLUS_BREAKAWAY_MAX_MDEG * H3_DIRECTION_SIGN);
    assert(last_breakaway_fast);

    now_ms = 8300U + H3_BREAKAWAY_TIMEOUT_MS;
    refresh_vision(now_ms, 0, 0);
    AppH3_Task20ms(now_ms);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.fault == APP_H3_FAULT_NONE);
    assert(return_calls == 1U && disable_calls == 0U);

    lever.ball_active = false;
    lever.ball_returning = false;
    refresh_vision(now_ms + 20U, 0, 0);
    AppH3_Task20ms(now_ms + 20U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.active && snapshot.state == APP_H3_WAIT_READY);
    assert(snapshot.fault == APP_H3_FAULT_NONE);
}

static void test_minus_stage_predictive_hard_brake_then_early_return(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(9000U);
    make_ready_and_start(9000U);

    refresh_vision(9320U, 405, 34);
    AppH3_Task20ms(9320U);
    lever.current_counts = minus_brake_reference_counts();
    lever.ball_velocity_direction = -1;
    refresh_vision(9340U, 390, -20);
    AppH3_Task20ms(9340U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.direction_confirmed);

    /* Even an early negative camera velocity cannot shorten the calibrated
     * 300 ms RUN_MINUS pass-brake hold. */
    refresh_vision(9500U, 400, -40);
    AppH3_Task20ms(9500U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);

    /* Release at 300 ms, then preserve the 50 ms neutral handoff before the
     * fixed positive counter-tilt is commanded. */
    refresh_vision(9620U, 400, -40);
    AppH3_Task20ms(9620U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg !=
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    refresh_vision(9680U, 400, -40);
    AppH3_Task20ms(9680U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt == snapshot.forced_tilt_mdeg);
    assert(last_breakaway_fast);
    assert(last_breakaway_rpm == H3_LEVER_BREAKAWAY_VELOCITY_RPM);

    /* r131 physically reversed at -46.1 mm while filtered speed still said
     * -57 mm/s.  At -43.8 mm and -84 mm/s the 75 ms projection is -50.1 mm,
     * so begin the measured mechanical return before the hidden reversal.
     * Do not wait for low speed and do not hand control back to PID. */
    refresh_vision(9700U, -438, -84);
    AppH3_Task20ms(9700U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg !=
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(snapshot.active && snapshot.fault == APP_H3_FAULT_NONE);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(return_calls == 1U);

    /* No later PID approach can re-arm the fixed impulse during return. */
    refresh_vision(9720U, 300, -40);
    AppH3_Task20ms(9720U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg !=
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
#endif
}

static void test_minus_brake_survives_predicted_edge(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(9700U);
    make_ready_and_start(9700U);
    refresh_vision(10020U, 405, 34);
    AppH3_Task20ms(10020U);
    lever.current_counts = minus_brake_reference_counts();
    lever.ball_velocity_direction = -1;
    refresh_vision(10040U, 390, -20);
    AppH3_Task20ms(10040U);
    refresh_vision(10320U, 400, -40);
    AppH3_Task20ms(10320U);
    refresh_vision(10380U, 400, -40);
    AppH3_Task20ms(10380U);

    /* Accepted positive counter-tilt remains the safer command during either
     * startup latency or a stopped-at-target interval. */
    lever.ball_velocity_direction = 0;
    refresh_vision(10400U, -300, -140);
    AppH3_Task20ms(10400U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.active && snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.fault == APP_H3_FAULT_NONE);
    assert(snapshot.forced_tilt_mdeg ==
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
#endif
}

static void test_minus_brake_grows_when_pass_actuator_is_deep(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(15500U);
    make_ready_and_start(15500U);
    refresh_vision(15820U, 405, 34);
    AppH3_Task20ms(15820U);

    /* Three degrees deeper than the measured -7 degree reference would add
     * three degrees to the configured counter-tilt, so the default 4 degree
     * profile must clamp at the safe 6 degree adaptive ceiling. */
    lever.current_counts = minus_brake_reference_counts() -
        H3_DIRECTION_SIGN * counts_for_mdeg(3000L);
    lever.ball_velocity_direction = -1;
    refresh_vision(16120U, 400, -20);
    AppH3_Task20ms(16120U);
    refresh_vision(16180U, 390, -20);
    AppH3_Task20ms(16180U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.forced_tilt_mdeg ==
           H3_MINUS_BRAKE_ADAPTIVE_MAX_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt == snapshot.forced_tilt_mdeg);
#endif
}

static void test_minus_brake_early_reversal_returns_safely(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(14000U);
    make_ready_and_start(14000U);
    refresh_vision(14320U, 405, 34);
    AppH3_Task20ms(14320U);
    lever.current_counts = minus_brake_reference_counts();
    lever.ball_velocity_direction = -1;
    refresh_vision(14340U, 390, -20);
    AppH3_Task20ms(14340U);
    refresh_vision(14620U, 400, -40);
    AppH3_Task20ms(14620U);
    refresh_vision(14680U, 400, -40);
    AppH3_Task20ms(14680U);

    /* A real negative velocity arms the reversal fallback without yet
     * reaching the projected -50 mm return boundary. */
    refresh_vision(14700U, -100, -20);
    AppH3_Task20ms(14700U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.forced_tilt_mdeg ==
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);

    /* If counter-tilt stops the ball early, release it at the first observed
     * reversal instead of driving it all the way back to the positive edge. */
    refresh_vision(14720U, -320, 1);
    AppH3_Task20ms(14720U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RETURNING);
    assert(snapshot.forced_tilt_mdeg !=
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(snapshot.fault == APP_H3_FAULT_NONE);
    assert(return_calls == 1U);
#endif
}

static void test_plus_stage_predictive_brake_persists_across_transition(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(10000U);
    make_ready_and_start(10000U);
    refresh_vision(10320U, 20, 5);
    AppH3_Task20ms(10320U);

    /* +38 mm at +44 mm/s projects to +60 mm in 500 ms.  r114 deliberately
     * waits until this later point so the ball enters the +40 mm scoring
     * window before the fixed counter-tilt reverses it. */
    refresh_vision(10500U, 380, 44);
    AppH3_Task20ms(10500U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_PREPASS_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt == snapshot.forced_tilt_mdeg);
    assert(last_breakaway_fast);
    assert(last_breakaway_rpm == H3_PLUS_LEVER_BREAKAWAY_VELOCITY_RPM);

    /* Crossing +40 switches the target stage, but must not restore the old
     * 0.6 degree guard while outward speed remains high. */
    refresh_vision(10520U, 405, 25);
    AppH3_Task20ms(10520U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);

    lever.current_counts = minus_brake_reference_counts();
    /* RUN_PLUS keeps the original direction-bit requirement.  Once the pass
     * has entered RUN_MINUS, encoder displacement alone completes the handoff
     * even if the transient direction bit has already returned to zero. */
    lever.ball_velocity_direction = 0;
    refresh_vision(10540U, 410, H3_PLUS_HARD_BRAKE_RELEASE_MM_S);
    AppH3_Task20ms(10540U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(snapshot.active && snapshot.fault == APP_H3_FAULT_NONE);

    /* RUN_MINUS deliberately ignores the noisy velocity-release instant.
     * Hold through 280 ms and release exactly 300 ms after the +40 pass. */
    refresh_vision(10800U, 410, 5);
    AppH3_Task20ms(10800U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    refresh_vision(10820U, 410, 5);
    AppH3_Task20ms(10820U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg !=
           -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);

    /* During the 50 ms pending interval the measured actuator is still at
     * the accepted negative pass brake.  A predicted positive-edge warning
     * must not cancel that physical brake, but the absolute edge remains
     * authoritative. */
    refresh_vision(10840U, 540, 20);
    AppH3_Task20ms(10840U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.active && snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.fault == APP_H3_FAULT_NONE);

    /* Preserve the calibrated 50 ms neutral handoff before pre-commanding
     * the negative-stage counter-tilt, before direction confirmation. */
    refresh_vision(10860U, 410, 5);
    AppH3_Task20ms(10860U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg !=
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    refresh_vision(10880U, 410, 5);
    AppH3_Task20ms(10880U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.direction_confirmed);
    assert(snapshot.forced_tilt_mdeg ==
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);

    /* Do not finish the pre-command merely because speed is still positive.
     * Finish only after real negative motion reaches -50 mm. */
    refresh_vision(10940U, 390, -20);
    AppH3_Task20ms(10940U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.direction_confirmed);
    assert(snapshot.forced_tilt_mdeg ==
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    refresh_vision(10960U, -500, -50);
    AppH3_Task20ms(10960U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg !=
           H3_MINUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(snapshot.state == APP_H3_RETURNING);
#endif
}

static void test_fast_plus_pass_arms_brake_in_transition_slot(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(10800U);
    make_ready_and_start(10800U);
    refresh_vision(11120U, 20, 5);
    AppH3_Task20ms(11120U);

    /* A camera sample may jump directly into the +40 mm scoring window
     * before the 500 ms forecast reaches its pre-pass trigger.  Transition
     * and -8 degree braking must therefore be atomic for a fast pass. */
    refresh_vision(11300U, 405, 40);
    AppH3_Task20ms(11300U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt == snapshot.forced_tilt_mdeg);
    assert(last_breakaway_fast);
#endif
}

static void test_encoder_launch_pulse_withdraws_then_allows_only_braking(void)
{
    AppH3Snapshot_t snapshot;
    int32_t pulse_counts = (int32_t)(((int64_t)
        H3_LAUNCH_PULSE_FEEDBACK_MDEG *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);

    reset_fixture(10000U);
    make_ready_and_start(10000U);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS + pulse_counts;
    refresh_vision(10520U, 0, 0);
    AppH3_Task20ms(10520U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.active && snapshot.state == APP_H3_RUN_PLUS);
    assert(snapshot.launch_withdrawing);
    assert(snapshot.forced_tilt_mdeg == 0 && last_tilt == 0);
    assert(withdraw_calls == 1U);
    assert(!last_breakaway_fast && return_calls == 0U);
    assert(snapshot.raw_pd_mdeg > 0);

    /* Once camera motion is confirmed, an outward PD request remains
     * suppressed.  Predictive braking must nevertheless override the
     * withdrawal substate even while raw PD is still outward; otherwise the
     * real actuator receives only a horizontal target. */
    refresh_vision(10540U, 20, 5);
    AppH3_Task20ms(10540U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && last_tilt == 0);
    /* At +31 mm and +54 mm/s the 500 ms projection is +58 mm, while
     * Kp*error still narrowly exceeds Kd*speed.  This isolates the priority
     * rule: predictive braking, not ordinary negative PD, ends withdrawal. */
    refresh_vision(10560U, 280, 64);
    AppH3_Task20ms(10560U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.launch_withdrawing);
    assert(snapshot.direction_confirmed);
    assert(snapshot.raw_pd_mdeg > 0);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_PREPASS_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt == snapshot.forced_tilt_mdeg);
    assert(last_breakaway_fast);
    assert(snapshot.active && return_calls == 0U);
}

static void test_launch_withdrawal_rearms_normal_pd_only_level_and_slow(void)
{
    AppH3Snapshot_t snapshot;
    int32_t pulse_counts = (int32_t)(((int64_t)
        H3_LAUNCH_PULSE_FEEDBACK_MDEG *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);

    reset_fixture(11000U);
    make_ready_and_start(11000U);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS + pulse_counts;
    refresh_vision(11520U, 0, 0);
    AppH3_Task20ms(11520U);
    refresh_vision(11540U, 20, 5);
    AppH3_Task20ms(11540U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && last_tilt == 0);

    /* Level alone is insufficient while the ball is still fast. */
    lever.current_counts = LEVER_HORIZONTAL_COUNTS;
    refresh_vision(11560U, 180, 26);
    AppH3_Task20ms(11560U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && last_tilt == 0);

    /* Once level and slow, ordinary bounded PD may resume toward the target;
     * the large forced launch angle remains permanently withdrawn. */
    refresh_vision(11580U, 140, 8);
    AppH3_Task20ms(11580U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.launch_withdrawing);
    assert(snapshot.forced_tilt_mdeg == 0);
    assert(last_tilt > 0 && last_tilt <=
           H3_TILT_SLEW_MDEG_PER_20MS);
    assert(!last_breakaway_fast);
}

static void test_plus_withdrawal_holds_topup_until_recovery(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    int32_t pulse_counts = (int32_t)(((int64_t)
        H3_LAUNCH_PULSE_FEEDBACK_MDEG *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);
    int32_t topup_counts = (int32_t)(((int64_t)
        H3_PLUS_TOPUP_TILT_MDEG *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);

    reset_fixture(11800U);
    make_ready_and_start(11800U);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS + pulse_counts;
    refresh_vision(12320U, 0, 0);
    AppH3_Task20ms(12320U);
    refresh_vision(12340U, 20, 5);
    AppH3_Task20ms(12340U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && snapshot.direction_confirmed);

    /* Once withdrawal leaves the ball above +20 mm and nearly stopped, issue
     * the bounded +8 degree top-up at launch velocity.  The validated final
     * profile deliberately waits for the 5 mm/s static-friction window. */
    refresh_vision(12360U, 210, H3_PLUS_TOPUP_MAX_SPEED_MM_S);
    AppH3_Task20ms(12360U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.launch_withdrawing);
    assert(snapshot.forced_tilt_mdeg ==
           H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt > 0 && last_tilt < snapshot.forced_tilt_mdeg);
    assert(last_breakaway_fast);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS + topup_counts -
                           H3_LEVER_START_DEADBAND_COUNTS + 1L;
    refresh_vision(12380U, 220, 24);
    AppH3_Task20ms(12380U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.launch_withdrawing);
    assert(snapshot.forced_tilt_mdeg ==
           H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);

    /* Encoder arrival alone is not evidence that the ball recovered.  Hold
     * +8 degrees until camera speed responds, then brake in the same slot. */
    refresh_vision(12400U, 230, H3_PLUS_TOPUP_RECOVERY_SPEED_MM_S);
    AppH3_Task20ms(12400U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           -H3_PLUS_PREPASS_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);

    /* Entering the +40 mm pass window switches to RUN_MINUS and removes the
     * positive top-up before the negative-stage controller is evaluated. */
    refresh_vision(12420U, 405, 20);
    AppH3_Task20ms(12420U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_MINUS);
    assert(snapshot.forced_tilt_mdeg !=
           H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);
#endif
}

static void test_plus_topup_survives_early_withdrawal_exit(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(12600U);
    make_ready_and_start(12600U);
    refresh_vision(12920U, 20, 5);
    AppH3_Task20ms(12920U);

    /* The one-shot top-up remains available after withdrawal exits, provided
     * the ball has not yet entered the predictive braking window. */
    refresh_vision(13100U, 210, H3_PLUS_TOPUP_MAX_SPEED_MM_S);
    AppH3_Task20ms(13100U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_PLUS);
    assert(!snapshot.launch_withdrawing);
    assert(snapshot.forced_tilt_mdeg ==
           H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt > 0 && last_breakaway_fast);
#endif
}

static void test_plus_topup_cannot_enter_at_its_brake_boundary(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(13100U);
    make_ready_and_start(13100U);
    refresh_vision(13420U, 20, 5);
    AppH3_Task20ms(13420U);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS;
    refresh_vision(13440U, 140, 8);
    AppH3_Task20ms(13440U);

    /* A coarse camera update may first report a slow ball only after it has
     * already crossed the top-up brake boundary.  Do not enter +8 degrees and
     * immediately flip to -8 degrees in that same control period. */
    refresh_vision(13460U, H3_PLUS_TOPUP_BRAKE_TRIGGER_X10 + 10L,
                   H3_PLUS_TOPUP_MAX_SPEED_MM_S);
    AppH3_Task20ms(13460U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_PLUS);
    assert(snapshot.forced_tilt_mdeg == 0);
    assert(last_tilt != H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt != -H3_PLUS_HARD_BRAKE_TILT_MDEG * H3_DIRECTION_SIGN);
#endif
}

static void test_plus_predictive_brake_permanently_clears_topup(void)
{
#if !H3_SINGLE_TARGET_ONLY
    AppH3Snapshot_t snapshot;
    reset_fixture(13200U);
    make_ready_and_start(13200U);
    refresh_vision(13520U, 20, 5);
    AppH3_Task20ms(13520U);

    /* Reproduce r134: a slow +20 mm pass first latches the +8 degree top-up. */
    refresh_vision(13700U, 210, H3_PLUS_TOPUP_MAX_SPEED_MM_S);
    AppH3_Task20ms(13700U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);

    /* Bound the special path at +25 mm.  The simulated encoder may confirm
     * an immediate brake release in this slot, but the +8 degree top-up must
     * already be cleared permanently. */
    refresh_vision(13720U, H3_PLUS_TOPUP_BRAKE_TRIGGER_X10 + 10L, 10);
    AppH3_Task20ms(13720U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg !=
           H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);

    /* Encoder-confirmed release at 20 mm/s must never restore +8 degrees. */
    lever.current_counts -= 16L;
    lever.ball_velocity_direction = -1;
    refresh_vision(13740U, 329, H3_PLUS_HARD_BRAKE_RELEASE_MM_S);
    AppH3_Task20ms(13740U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_H3_RUN_PLUS);
    assert(snapshot.forced_tilt_mdeg !=
           H3_PLUS_TOPUP_TILT_MDEG * H3_DIRECTION_SIGN);
#endif
}

static void test_no_roll_retries_one_degree_higher_then_stops_pulsing(void)
{
    AppH3Snapshot_t snapshot;
    int32_t first_counts = (int32_t)(((int64_t)
        H3_LAUNCH_PULSE_FEEDBACK_MDEG *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);
    int32_t retry_counts = (int32_t)(((int64_t)
        (H3_LAUNCH_PULSE_FEEDBACK_MDEG + H3_LAUNCH_RETRY_STEP_MDEG) *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);

    reset_fixture(12000U);
    make_ready_and_start(12000U);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS + first_counts;
    refresh_vision(12520U, 0, 0);
    AppH3_Task20ms(12520U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && withdraw_calls == 1U);

    /* Back at level with no camera displacement, the first withdrawal arms
     * one stronger pulse instead of waiting motionless for timeout. */
    lever.current_counts = LEVER_HORIZONTAL_COUNTS;
    refresh_vision(12540U, 0, 0);
    AppH3_Task20ms(12540U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.launch_withdrawing && last_tilt == 0);
    refresh_vision(12560U, 0, 0);
    AppH3_Task20ms(12560U);
    assert(last_tilt > 0 && last_breakaway_fast);

    lever.current_counts = LEVER_HORIZONTAL_COUNTS + retry_counts;
    refresh_vision(12580U, 0, 0);
    AppH3_Task20ms(12580U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && withdraw_calls == 2U);

    /* Two pulses is the fixed safety budget.  A second no-roll withdrawal
     * remains level until the ordinary no-roll timeout. */
    lever.current_counts = LEVER_HORIZONTAL_COUNTS;
    refresh_vision(12600U, 0, 0);
    AppH3_Task20ms(12600U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && last_tilt == 0);
}

static void test_stalled_far_error_gets_bounded_tracking_relaunch(void)
{
    AppH3Snapshot_t snapshot;
    int32_t pulse_counts = (int32_t)(((int64_t)
        H3_LAUNCH_PULSE_FEEDBACK_MDEG *
        LEVER_POSITION_COUNTS_PER_REVOLUTION + 359999L) / 360000L);
    int32_t relaunch_counts = (int32_t)(((int64_t)
        H3_PLUS_RELAUNCH_TILT_MDEG * LEVER_POSITION_COUNTS_PER_REVOLUTION +
        359999L) / 360000L);

    reset_fixture(13000U);
    make_ready_and_start(13000U);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS + pulse_counts;
    refresh_vision(13520U, 0, 0);
    AppH3_Task20ms(13520U);
    refresh_vision(13540U, 20, 5);
    AppH3_Task20ms(13540U);
    lever.current_counts = LEVER_HORIZONTAL_COUNTS;
    refresh_vision(13560U, 140, 8);
    AppH3_Task20ms(13560U);
    AppH3_GetSnapshot(&snapshot);
    assert(!snapshot.launch_withdrawing);

    refresh_vision(13600U, 140, 0);
    AppH3_Task20ms(13600U);
    refresh_vision(13720U, 140, 0);
    AppH3_Task20ms(13720U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg ==
           H3_PLUS_RELAUNCH_TILT_MDEG * H3_DIRECTION_SIGN);
    assert(last_tilt > 0 && last_breakaway_fast);
    assert(last_breakaway_rpm == H3_LEVER_VELOCITY_RPM);

    /* Reaching the relaunch threshold within the actuator start deadband is
     * physically complete and must start withdrawal; exact equality is not
     * reachable on the real mechanism. */
    lever.current_counts = LEVER_HORIZONTAL_COUNTS + relaunch_counts -
                           H3_LEVER_START_DEADBAND_COUNTS + 1;
    refresh_vision(13740U, 140, 0);
    AppH3_Task20ms(13740U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.launch_withdrawing && snapshot.forced_tilt_mdeg == 0);
    assert(withdraw_calls >= 2U);

    /* Confirmed motion immediately withdraws the bounded relaunch feedforward
     * and hands control back to ordinary PD braking/tracking. */
    lever.current_counts = LEVER_HORIZONTAL_COUNTS;
    refresh_vision(13760U, 190, H3_RELAUNCH_MOVING_SPEED_MM_S);
    AppH3_Task20ms(13760U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.forced_tilt_mdeg == 0);
    assert(!last_breakaway_fast);
}

static void test_center_hold_uses_pd_without_breakaway_or_mission_timeout(void)
{
    AppH3Snapshot_t snapshot;
    uint32_t base_ms = 12000U;

    reset_fixture(base_ms);
    refresh_vision(base_ms, 0, 0);
    AppH3_Task20ms(base_ms);
    refresh_vision(base_ms + 300U, 0, 0);
    AppH3_Task20ms(base_ms + 300U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.ready);
    assert(AppH3_StartCenterHold(base_ms + 300U));

    refresh_vision(base_ms + 320U, 100, 20);
    AppH3_Task20ms(base_ms + 320U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.active && snapshot.state == APP_H3_CENTER_HOLD);
    assert(snapshot.target_mm_x10 == 0);
    assert(snapshot.raw_pd_mdeg < 0 && snapshot.tilt_mdeg < 0);
    assert(snapshot.forced_tilt_mdeg == 0 && !last_breakaway_fast);

    /* CENTER_HOLD is bounded by the host's 12 s authorization deadline, not
     * the competition mission's 11 s trajectory timeout. */
    refresh_vision(base_ms + 11400U, 0, 0);
    AppH3_Task20ms(base_ms + 11400U);
    AppH3_GetSnapshot(&snapshot);
    assert(snapshot.active && snapshot.state == APP_H3_CENTER_HOLD);
    assert(snapshot.fault == APP_H3_FAULT_NONE);
}

int main(void)
{
    test_plus_control_and_direction_guard();
    test_plus_passes_through_and_minus_finishes_fixed_brake();
    test_fast_plus_window_switch_allows_bounded_reversal_grace();
    test_invalid_vision_returns_horizontal();
    test_edge_and_actuator_faults_stop_new_targets();
    test_commanded_brake_survives_predicted_edge();
    test_debug_cadence_averages_20hz();
    test_minus_stage_direction_guard();
    test_no_roll_automatically_ramps_and_returns_without_fault();
    test_minus_stage_predictive_hard_brake_then_early_return();
    test_minus_brake_survives_predicted_edge();
    test_minus_brake_grows_when_pass_actuator_is_deep();
    test_minus_brake_early_reversal_returns_safely();
    test_plus_stage_predictive_brake_persists_across_transition();
    test_fast_plus_pass_arms_brake_in_transition_slot();
    test_encoder_launch_pulse_withdraws_then_allows_only_braking();
    test_launch_withdrawal_rearms_normal_pd_only_level_and_slow();
    test_plus_withdrawal_holds_topup_until_recovery();
    test_plus_topup_survives_early_withdrawal_exit();
    test_plus_topup_cannot_enter_at_its_brake_boundary();
    test_plus_predictive_brake_permanently_clears_topup();
    test_no_roll_retries_one_degree_higher_then_stops_pulsing();
    test_stalled_far_error_gets_bounded_tracking_relaunch();
    test_center_hold_uses_pd_without_breakaway_or_mission_timeout();
    return 0;
}
