#include "h3_tune.h"

#include "app_h3.h"
#include "h3_control_config.h"
#include "lever_actuator.h"
#include "safety.h"
#include "system_state.h"

#include <stddef.h>
#include <string.h>

#define H3_TUNE_MAX_RUNS             8U
#define H3_TUNE_MAX_TTL_MS     1800000UL
#define H3_TUNE_MAX_RUN_MS       12000UL
#define H3_TUNE_HEARTBEAT_MS       300UL

_Static_assert(sizeof(H3TuneProfile_t) == 44U,
               "H3 internal tune profile must contain 22 packed u16 values");

static H3TuneSnapshot_t s_tune;
static uint32_t s_run_timeout_ms;
static H3TuneProfile_t s_authorized_golden;
static bool s_golden_valid;

static H3TuneProfile_t defaults(void)
{
    H3TuneProfile_t profile;
    profile.plus_start_mdeg = (uint16_t)H3_BREAKAWAY_START_MDEG;
    profile.plus_max_mdeg = (uint16_t)H3_PLUS_BREAKAWAY_MAX_MDEG;
    profile.minus_start_mdeg = (uint16_t)H3_BREAKAWAY_START_MDEG;
    profile.minus_max_mdeg = (uint16_t)H3_BREAKAWAY_MAX_MDEG;
    profile.kp_mdeg_per_mm = (uint16_t)H3_CONTROL_KP_MDEG_PER_MM;
    profile.kd_mdeg_per_mm_s = (uint16_t)H3_CONTROL_KD_MDEG_PER_MM_S;
    profile.normal_slew_mdeg_per_20ms =
        (uint16_t)H3_TILT_SLEW_MDEG_PER_20MS;
    profile.breakaway_slew_mdeg_per_20ms =
        (uint16_t)H3_BREAKAWAY_TILT_SLEW_MDEG_PER_20MS;
    profile.brake_slew_mdeg_per_20ms =
        (uint16_t)H3_BRAKE_TILT_SLEW_MDEG_PER_20MS;
    profile.tracking_rpm = H3_LEVER_VELOCITY_RPM;
    profile.plus_launch_rpm = H3_PLUS_LEVER_BREAKAWAY_VELOCITY_RPM;
    profile.minus_launch_rpm = H3_LEVER_BREAKAWAY_VELOCITY_RPM;
    profile.ki_mdeg_per_mm_s = H3_CONTROL_KI_MDEG_PER_MM_S;
    profile.plus_topup_mdeg = H3_PLUS_TOPUP_TILT_MDEG;
    profile.plus_brake_mdeg = H3_PLUS_HARD_BRAKE_TILT_MDEG;
    profile.minus_brake_trigger_x10 = H3_MINUS_HARD_BRAKE_TRIGGER_X10;
    profile.minus_brake_mdeg = H3_MINUS_HARD_BRAKE_TILT_MDEG;
    profile.plus_brake_trigger_x10 = H3_PLUS_HARD_BRAKE_TRIGGER_X10;
    profile.plus_topup_brake_trigger_x10 =
        H3_PLUS_TOPUP_BRAKE_TRIGGER_X10;
    profile.minus_brake_return_predict_ms =
        H3_MINUS_BRAKE_RETURN_PREDICT_MS;
    profile.minus_return_rpm = H3_LEVER_RETURN_VELOCITY_RPM;
    profile.plus_topup_max_speed_mm_s = H3_PLUS_TOPUP_MAX_SPEED_MM_S;
    return profile;
}

static bool elapsed(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

bool H3Tune_ProfileValid(const H3TuneProfile_t *p)
{
    if (p == NULL) return false;
    return p->plus_start_mdeg >= 1000U &&
        p->plus_start_mdeg <= p->plus_max_mdeg &&
        p->plus_max_mdeg >= 3000U &&
        p->plus_max_mdeg <= H3_TUNE_POSITIVE_MAX_MDEG &&
        p->minus_start_mdeg >= 1000U &&
        p->minus_start_mdeg <= p->minus_max_mdeg &&
        p->minus_max_mdeg >= 3000U &&
        p->minus_max_mdeg <= H3_TUNE_NEGATIVE_MAX_MDEG &&
        p->kp_mdeg_per_mm >= 20U && p->kp_mdeg_per_mm <= 300U &&
        p->kd_mdeg_per_mm_s <= 120U &&
        p->normal_slew_mdeg_per_20ms >= 100U &&
        p->normal_slew_mdeg_per_20ms <= 500U &&
        p->breakaway_slew_mdeg_per_20ms >= 200U &&
        p->breakaway_slew_mdeg_per_20ms <= 1500U &&
        p->brake_slew_mdeg_per_20ms >= 200U &&
        p->brake_slew_mdeg_per_20ms <= 2500U &&
        p->normal_slew_mdeg_per_20ms <=
            p->breakaway_slew_mdeg_per_20ms &&
        p->breakaway_slew_mdeg_per_20ms <=
            p->brake_slew_mdeg_per_20ms &&
        p->tracking_rpm >= 1U && p->tracking_rpm <= 3U &&
        p->plus_launch_rpm >= p->tracking_rpm &&
        p->plus_launch_rpm <= 8U &&
        p->minus_launch_rpm >= p->tracking_rpm &&
        p->minus_launch_rpm <= 8U &&
        p->ki_mdeg_per_mm_s <= 100U &&
        p->plus_topup_mdeg >= 6500U &&
        p->plus_topup_mdeg <= 12000U &&
        p->plus_brake_mdeg >= 3000U &&
        p->plus_brake_mdeg <= 12000U &&
        p->minus_brake_trigger_x10 <= 400U &&
        p->minus_brake_mdeg >= 3000U &&
        p->minus_brake_mdeg <= 12000U &&
        p->plus_brake_trigger_x10 >= 400U &&
        p->plus_brake_trigger_x10 <= 600U &&
        p->plus_topup_brake_trigger_x10 >= 200U &&
        p->plus_topup_brake_trigger_x10 <= 390U &&
        p->plus_topup_brake_trigger_x10 < p->plus_brake_trigger_x10 &&
        p->minus_brake_return_predict_ms <= 250U &&
        p->minus_return_rpm >= 1U && p->minus_return_rpm <= 8U &&
        p->plus_topup_max_speed_mm_s <= 30U;
}

static bool config_mutable(void)
{
    LeverActuatorSnapshot_t lever;
    LeverActuator_GetSnapshot(&lever);
    return SystemState_Get() == SYSTEM_STATE_IDLE &&
        !Safety_HasFault() && !AppH3_IsActive() &&
        lever.boot_homed &&
        lever.boot_state == LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL &&
        !lever.moving && !lever.ball_active && lever.fault_latched == 0U;
}

static H3TuneResult_t revision_check(uint32_t active, uint32_t staged)
{
    if (active != s_tune.active_revision) return H3_TUNE_STALE_ACTIVE;
    if (staged != s_tune.stage_revision) return H3_TUNE_STALE_STAGE;
    return H3_TUNE_OK;
}

static H3TuneResult_t auth_check(uint32_t token, uint32_t now_ms)
{
    if (!s_tune.authorized || token == 0U || token != s_tune.auth_token)
        return H3_TUNE_NOT_AUTHORIZED;
    if (elapsed(now_ms, s_tune.auth_expires_ms))
        return H3_TUNE_AUTH_EXPIRED;
    return H3_TUNE_OK;
}

static void clear_authorization(H3TuneStopReason_t reason,
                                bool restore_golden)
{
    if (s_tune.host_run_active || AppH3_IsActive()) {
        AppH3Snapshot_t h3;
        AppH3_GetSnapshot(&h3);
        /* A real H3 fault has already started the bounded horizontal return.
         * Revoking host authority must not overwrite that root cause with an
         * artificial EMERGENCY or disable the return in progress.  Heartbeat,
         * expiry and explicit revoke while RUN/HOLD still stop immediately. */
        if (!AppH3_IsActive() ||
            ((h3.state != APP_H3_RETURNING) &&
             (h3.state != APP_H3_FAULT)))
            AppH3_EmergencyStop();
    }
    if (restore_golden && s_golden_valid &&
        memcmp(&s_tune.active, &s_authorized_golden,
               sizeof(s_authorized_golden)) != 0) {
        s_tune.active = s_authorized_golden;
        s_tune.staged = s_authorized_golden;
        s_tune.active_revision++;
        s_tune.stage_revision++;
        s_tune.dirty = false;
    }
    s_tune.authorized = false;
    s_tune.auth_token = 0U;
    s_tune.auth_expires_ms = 0U;
    s_tune.run_deadline_ms = 0U;
    s_tune.remaining_runs = 0U;
    s_tune.host_run_active = false;
    s_tune.stop_reason = reason;
    s_golden_valid = false;
}

void H3Tune_Init(uint32_t now_ms)
{
    memset(&s_tune, 0, sizeof(s_tune));
    s_tune.active = defaults();
    s_tune.staged = s_tune.active;
    s_tune.active_revision = 1U;
    s_tune.stage_revision = 1U;
    s_tune.last_heartbeat_ms = now_ms;
    s_run_timeout_ms = H3_TUNE_MAX_RUN_MS;
    s_golden_valid = false;
}

const H3TuneProfile_t *H3Tune_Active(void)
{
    return &s_tune.active;
}

H3TuneResult_t H3Tune_StageField(H3TuneParameter_t parameter,
                                uint16_t value, uint32_t active_revision,
                                uint32_t stage_revision)
{
    H3TuneProfile_t candidate;
    H3TuneResult_t result = revision_check(active_revision, stage_revision);
    if (result != H3_TUNE_OK) return result;
    if (!config_mutable()) return H3_TUNE_WRONG_STATE;
    candidate = s_tune.staged;
    switch (parameter) {
    case H3_TUNE_PARAM_PLUS_START_MDEG: candidate.plus_start_mdeg = value; break;
    case H3_TUNE_PARAM_PLUS_MAX_MDEG: candidate.plus_max_mdeg = value; break;
    case H3_TUNE_PARAM_MINUS_START_MDEG: candidate.minus_start_mdeg = value; break;
    case H3_TUNE_PARAM_MINUS_MAX_MDEG: candidate.minus_max_mdeg = value; break;
    case H3_TUNE_PARAM_KP: candidate.kp_mdeg_per_mm = value; break;
    case H3_TUNE_PARAM_KD: candidate.kd_mdeg_per_mm_s = value; break;
    case H3_TUNE_PARAM_NORMAL_SLEW: candidate.normal_slew_mdeg_per_20ms = value; break;
    case H3_TUNE_PARAM_BREAKAWAY_SLEW: candidate.breakaway_slew_mdeg_per_20ms = value; break;
    case H3_TUNE_PARAM_BRAKE_SLEW: candidate.brake_slew_mdeg_per_20ms = value; break;
    case H3_TUNE_PARAM_TRACKING_RPM: candidate.tracking_rpm = value; break;
    case H3_TUNE_PARAM_PLUS_LAUNCH_RPM: candidate.plus_launch_rpm = value; break;
    case H3_TUNE_PARAM_MINUS_LAUNCH_RPM: candidate.minus_launch_rpm = value; break;
    case H3_TUNE_PARAM_KI: candidate.ki_mdeg_per_mm_s = value; break;
    case H3_TUNE_PARAM_PLUS_TOPUP_MDEG: candidate.plus_topup_mdeg = value; break;
    case H3_TUNE_PARAM_PLUS_BRAKE_MDEG: candidate.plus_brake_mdeg = value; break;
    case H3_TUNE_PARAM_MINUS_BRAKE_TRIGGER_X10:
        candidate.minus_brake_trigger_x10 = value; break;
    case H3_TUNE_PARAM_MINUS_BRAKE_MDEG: candidate.minus_brake_mdeg = value; break;
    case H3_TUNE_PARAM_PLUS_BRAKE_TRIGGER_X10:
        candidate.plus_brake_trigger_x10 = value; break;
    case H3_TUNE_PARAM_PLUS_TOPUP_BRAKE_TRIGGER_X10:
        candidate.plus_topup_brake_trigger_x10 = value; break;
    case H3_TUNE_PARAM_MINUS_BRAKE_RETURN_PREDICT_MS:
        candidate.minus_brake_return_predict_ms = value; break;
    case H3_TUNE_PARAM_MINUS_RETURN_RPM:
        candidate.minus_return_rpm = value; break;
    case H3_TUNE_PARAM_PLUS_TOPUP_MAX_SPEED_MM_S:
        candidate.plus_topup_max_speed_mm_s = value; break;
    default: return H3_TUNE_INVALID_ARGUMENT;
    }
    if (!H3Tune_ProfileValid(&candidate)) return H3_TUNE_OUT_OF_RANGE;
    s_tune.staged = candidate;
    s_tune.stage_revision++;
    s_tune.dirty = memcmp(&s_tune.active, &s_tune.staged,
                          sizeof(candidate)) != 0;
    s_tune.last_result = H3_TUNE_OK;
    return H3_TUNE_OK;
}

H3TuneResult_t H3Tune_Apply(uint32_t active_revision,
                            uint32_t stage_revision)
{
    H3TuneResult_t result = revision_check(active_revision, stage_revision);
    if (result != H3_TUNE_OK) return result;
    if (!config_mutable()) return H3_TUNE_WRONG_STATE;
    if (!H3Tune_ProfileValid(&s_tune.staged)) return H3_TUNE_OUT_OF_RANGE;
    s_tune.active = s_tune.staged;
    s_tune.active_revision++;
    s_tune.dirty = false;
    s_tune.last_result = H3_TUNE_OK;
    return H3_TUNE_OK;
}

H3TuneResult_t H3Tune_Revert(uint32_t active_revision,
                             uint32_t stage_revision)
{
    H3TuneResult_t result = revision_check(active_revision, stage_revision);
    if (result != H3_TUNE_OK) return result;
    if (!config_mutable()) return H3_TUNE_WRONG_STATE;
    s_tune.staged = s_tune.active;
    s_tune.stage_revision++;
    s_tune.dirty = false;
    s_tune.last_result = H3_TUNE_OK;
    return H3_TUNE_OK;
}

H3TuneResult_t H3Tune_Authorize(uint32_t token, uint8_t runs,
                                uint32_t ttl_ms, uint32_t run_timeout_ms,
                                uint32_t now_ms)
{
    if (token == 0U || runs == 0U) return H3_TUNE_INVALID_ARGUMENT;
    if (runs > H3_TUNE_MAX_RUNS || ttl_ms == 0U ||
        ttl_ms > H3_TUNE_MAX_TTL_MS || run_timeout_ms == 0U ||
        run_timeout_ms > H3_TUNE_MAX_RUN_MS)
        return H3_TUNE_OUT_OF_RANGE;
    if (!config_mutable()) return H3_TUNE_WRONG_STATE;
    s_tune.auth_token = token;
    s_tune.authorized = true;
    s_tune.remaining_runs = runs;
    s_tune.auth_expires_ms = now_ms + ttl_ms;
    s_tune.last_heartbeat_ms = now_ms;
    s_tune.stop_reason = H3_TUNE_STOP_NONE;
    s_authorized_golden = s_tune.active;
    s_golden_valid = true;
    s_run_timeout_ms = run_timeout_ms;
    s_tune.last_result = H3_TUNE_OK;
    return H3_TUNE_OK;
}

static H3TuneResult_t start_run(uint32_t token, uint32_t now_ms,
                                bool center_hold)
{
    H3TuneResult_t result = auth_check(token, now_ms);
    if (result != H3_TUNE_OK) return result;
    if (s_tune.remaining_runs == 0U) return H3_TUNE_AUTH_EXHAUSTED;
    if (s_tune.host_run_active ||
        !(center_hold ? AppH3_StartCenterHold(now_ms) : AppH3_Start(now_ms)))
        return H3_TUNE_START_REJECTED;
    s_tune.remaining_runs--;
    s_tune.run_id++;
    s_tune.host_run_active = true;
    s_tune.last_heartbeat_ms = now_ms;
    s_tune.run_deadline_ms = now_ms + s_run_timeout_ms;
    s_tune.stop_reason = H3_TUNE_STOP_NONE;
    s_tune.last_result = H3_TUNE_OK;
    return H3_TUNE_OK;
}

H3TuneResult_t H3Tune_Start(uint32_t token, uint32_t now_ms)
{
    return start_run(token, now_ms, false);
}

H3TuneResult_t H3Tune_StartCenterHold(uint32_t token, uint32_t now_ms)
{
    return start_run(token, now_ms, true);
}

H3TuneResult_t H3Tune_Stop(uint32_t token, uint32_t now_ms)
{
    H3TuneResult_t result = auth_check(token, now_ms);
    if (result != H3_TUNE_OK) return result;
    if (!s_tune.host_run_active || !AppH3_IsActive())
        return H3_TUNE_WRONG_STATE;
    AppH3_RequestStop(now_ms);
    s_tune.stop_reason = H3_TUNE_STOP_HOST;
    return H3_TUNE_OK;
}

H3TuneResult_t H3Tune_Heartbeat(uint32_t token, uint32_t now_ms)
{
    H3TuneResult_t result = auth_check(token, now_ms);
    if (result != H3_TUNE_OK) return result;
    s_tune.last_heartbeat_ms = now_ms;
    return H3_TUNE_OK;
}

H3TuneResult_t H3Tune_Revoke(uint32_t token, H3TuneStopReason_t reason)
{
    if (s_tune.authorized && token != s_tune.auth_token)
        return H3_TUNE_NOT_AUTHORIZED;
    clear_authorization(reason == H3_TUNE_STOP_NONE ?
                        H3_TUNE_STOP_REVOKED : reason,
                        Safety_HasFault() || s_tune.host_run_active ||
                        AppH3_IsActive());
    return H3_TUNE_OK;
}

void H3Tune_OnDisconnect(uint32_t now_ms)
{
    (void)now_ms;
    if (s_tune.authorized || s_tune.host_run_active)
        clear_authorization(H3_TUNE_STOP_DISCONNECT, true);
}

void H3Tune_Task20ms(uint32_t now_ms)
{
    AppH3Snapshot_t h3;
    if (s_tune.host_run_active && !AppH3_IsActive()) {
        s_tune.host_run_active = false;
        s_tune.run_deadline_ms = 0U;
    }
    if (!s_tune.authorized) return;
    AppH3_GetSnapshot(&h3);
    if (Safety_HasFault()) {
        clear_authorization(H3_TUNE_STOP_FAULT, true);
        return;
    }
    if (s_tune.host_run_active && h3.fault != APP_H3_FAULT_NONE &&
        h3.fault != APP_H3_FAULT_USER_STOP) {
        clear_authorization(H3_TUNE_STOP_FAULT, true);
        return;
    }
    if (elapsed(now_ms, s_tune.auth_expires_ms)) {
        clear_authorization(H3_TUNE_STOP_AUTH_EXPIRED, true);
        return;
    }
    if (s_tune.host_run_active &&
        (uint32_t)(now_ms - s_tune.last_heartbeat_ms) >
            H3_TUNE_HEARTBEAT_MS) {
        clear_authorization(H3_TUNE_STOP_HEARTBEAT, true);
        return;
    }
    if (s_tune.host_run_active &&
        elapsed(now_ms, s_tune.run_deadline_ms))
        clear_authorization(H3_TUNE_STOP_RUN_TIMEOUT, true);
}

void H3Tune_GetSnapshot(H3TuneSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = s_tune;
}

void H3Tune_RecordResult(H3TuneResult_t result)
{
    s_tune.last_result = result;
}
